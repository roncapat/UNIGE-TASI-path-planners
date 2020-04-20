#include <iostream>
#include <string>
#include <chrono>
#include <iomanip>
#include "FieldDPlanner.h"
#include "bitmap/BMP.h"
#include "Graph.h"
char **argv;

Position next_point;
bool goal_reached = false;

std::shared_ptr<Map> map_info = nullptr;
float g_length = INFINITY; float g_cost = INFINITY;

void poses_cb(std::vector<Pose> &poses, float length, float cost) {
    g_length = length;
    g_cost = cost;
    std::ofstream logfile;
    std::string filename(argv[9]);
    logfile.open(filename);
    const size_t bufsize = 256*1024;
    char buf[bufsize];
    logfile.rdbuf()->pubsetbuf(buf, bufsize);
    logfile << std::setprecision(4);
    logfile << "{\"poses\":[";
    for (const auto &pose : poses) {
        logfile << "[" << pose.x
                << "," << pose.y
                << "," << pose.orientation
                << "],";
    }
    logfile.seekp(-1, std::ios::cur);
    logfile << "], \"length\": " << length << ", \"cost\": " << cost << "}";
    logfile.close();
    next_point = {poses[1].x, poses[1].y};
}

void expanded_cb(std::tuple<std::vector<std::tuple<int, int, float>>&, int, int> exp_info) {

    std::ofstream logfile;
    std::string filename(argv[10]);
    logfile.open(filename);
    const size_t bufsize = 256*1024;
    char buf[bufsize];
    logfile.rdbuf()->pubsetbuf(buf, bufsize);
    logfile << std::setprecision(3);
    logfile << "{\"num_expanded\":" << std::get<1>(exp_info)
            << ",\"num_updated\":" << std::get<2>(exp_info)
            << ",\"expanded\":[";
    for (const auto &node : std::get<0>(exp_info)) {
        logfile << "[" << std::get<0>(node)
                << "," << std::get<1>(node)
                << "," << ((std::get<2>(node) == std::numeric_limits<float>::infinity()) ? -1 : std::get<2>(node))
                << "],";
    }
    logfile.seekp(-1, std::ios::cur);
    logfile << "]}";
    logfile.close();
}

std::tuple<int,int,float> map_traversability_stats(uint8_t* data, unsigned int size){
    float avg = 0;
    int count = 0, min = 254, max = 0;
    for (auto p = data; p < (data + size); ++p) {
        if (*p < 255) {
            avg += (float) *p;
            ++count;
            if (*p < min) min = *p;
            if (*p > max) max = *p;
        }
    }
    avg /= (float) count;
    return {min,max,avg};
}

int main(int _argc, char **_argv) {
    argv = _argv;
    if (_argc < 13) {
        std::cerr << "Missing required argument." << std::endl;
        std::cerr << "Usage:" << std::endl;
        std::cerr << "\t" << argv[0]
                  << " <mapfile.bmp> <from_x> <from_y> <to_x> <to_y>"
                     " lookahead cspace optimiziation_lvl <logfile.json> <dbgfile.json> <infofile.json> <fifo_in> <fifo_out>"
                  << std::endl;
        return 1;
    }

    char ack = -1;
    unsigned int size;
    int32_t width=0, height=0, top=0, left=0;
    std::ifstream in__fifo{argv[12], std::ios::in | std::ios::binary};
    std::ofstream out_fifo{argv[13], std::ios::out | std::ios::binary};

    ack = 0;
    out_fifo.write((char*)&ack,1); //Send 0
    out_fifo.flush();
    ack = -1;
    while (ack != 0){
        in__fifo.read((char*)&ack, 1); //Wait for 0
    }
    in__fifo.read((char*)&width, 4);
    in__fifo.read((char*)&height, 4);
    std::cout << "[PLANNER]   Size: [" << width << ", " << height << "]" << std::endl;

    size = width*height;
    std::shared_ptr<uint8_t[]>data (new uint8_t[size], std::default_delete<uint8_t[]>());
    in__fifo.read((char*)data.get(), size); //Receive image

    map_info = std::make_shared<Map>(Map{
        .image = data,
        .resolution = 1,
        .orientation = 0,
        .length = static_cast<int>(height),
        .width = static_cast<int>(width),
        .x = std::stoi(argv[2]),
        .y = std::stoi(argv[3]),
        .x_initial = 0,
        .y_initial = 0
    });

    next_point.x = map_info->x;
    next_point.y = map_info->y;

    FieldDPlanner planner{};
    planner.occupancy_threshold_ = 1;
    planner.configuration_space_ = std::stoi(argv[7]);
    planner.optimization_lvl = std::stoi(argv[8]);
    planner.first_run_trick = false;
    planner.init();
    auto [min,max,avg] = map_traversability_stats(data.get(), size);
    std::cout << "Average traversability: " << avg << std::endl;
    std::cout << "Minimum traversability: " << min << std::endl;
    std::cout << "Maximum traversability: " << max << std::endl;

    planner.set_map(map_info);
    planner.set_start_position(next_point);
    planner.set_goal({std::stoi(argv[4]), std::stoi(argv[5])});
    planner.set_lookahead(std::stoi(argv[6]));
    // FIXME handle remplanning, for now i put the lowest possible value for consistency
    //planner.set_heuristic_multiplier(std::ceil(0.5*min)); // Ferguson and Stenz heuristic
    //planner.set_heuristic_multiplier(std::ceil(min)); // What I think is correct (consistent?)
    planner.set_heuristic_multiplier(1);
    planner.set_poses_cb(poses_cb);
    planner.set_expanded_cb(expanded_cb);

    float time;

    while(not goal_reached) {
        std::cout << "[PLANNER]   New position: [" << next_point.x << ", " << next_point.y << "]" << std::endl;
        ack = 1;
        out_fifo.write((char *) &ack, 1);
        out_fifo.write((char *) &(next_point.x), 4);
        out_fifo.write((char *) &(next_point.y), 4);
        out_fifo.flush();
        ack = -1;
        while (ack != 1) {
            in__fifo.read((char *) &ack, 1); //Wait for 1
        }
        in__fifo.read((char*)&top, 4);
        in__fifo.read((char*)&left, 4);
        in__fifo.read((char*)&height, 4);
        in__fifo.read((char*)&width, 4);
        std::cout << "[PLANNER]   New patch: position [" << top << ", " << left << "], shape [" << width << ", " << height << "]" << std::endl;
        size = width*height;
        std::shared_ptr<uint8_t[]>patch (new uint8_t[size], std::default_delete<uint8_t[]>());
        in__fifo.read((char*)patch.get(), size); //Receive image
        planner.node_grid_.updateGraph(patch, top, left, width, height);

        auto begin = std::chrono::steady_clock::now();
        planner.step();
        auto end = std::chrono::steady_clock::now();
        planner.publish_expanded_set();
        planner.publish_path();

        auto time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
        auto time_us = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
        time = time_ms + time_us / 1000.0f;
        std::cout << "Step time = " << time << std::endl;

        if  (next_point.x==std::stoi(argv[4]) and next_point.y==std::stoi(argv[5]))
            break; //Goal reached
        planner.set_start_position(next_point, true);
        //planner.init(); //PLAN FROM SCRATCH INSTEAD OF REPLANNING
    }

    ack = 2;
    out_fifo.write((char *) &ack, 1);
    out_fifo.flush();
    ack = -1;
    while (ack != 2) {
        in__fifo.read((char *) &ack, 1); //Wait for 2
    }

    std::ofstream infofile;
    std::string filename(argv[11]);
    infofile.open(filename);
    infofile << "{";
    infofile << "\"planner_cspace\":" << planner.configuration_space_;
    infofile << ",";
    infofile << "\"planner_opt_lvl\":" << planner.optimization_lvl;
    infofile << ",";
    infofile << "\"planner_lookahead\":" << planner.lookahead;
    infofile << ",";
    infofile << "\"map_size\":" << width*height;
    infofile << ",";
    infofile << "\"map_width\":" << width;
    infofile << ",";
    infofile << "\"map_height\":" << height;
    infofile << ",";
    infofile << "\"map_avg\":" << avg;
    infofile << ",";
    infofile << "\"map_min\":" << min;
    infofile << ",";
    infofile << "\"map_max\":" << max;
    infofile << ",";
    infofile << "\"map_start_x\":" << argv[2];
    infofile << ",";
    infofile << "\"map_start_y\":" << argv[3];
    infofile << ",";
    infofile << "\"map_goal_x\":" << argv[4];
    infofile << ",";
    infofile << "\"map_goal_y\":" << argv[5];
    infofile << ",";
    infofile << "\"nodes_expanded\":" << planner.num_nodes_expanded;
    infofile << ",";
    infofile << "\"step_time\":" << time;
    infofile << ",";
    infofile << "\"path_length\":" << g_length;
    infofile << ",";
    infofile << "\"path_cost\":" << g_cost;
    infofile << "}";
    infofile.close();

    #ifdef FDSTAR_SHOW_RESULT
    std::string cmd = "python3 plot_path_gui.py";
    cmd.append(argv[1]);
    cmd.append(" ");
    cmd.append(argv[9]);
    cmd.append(" ");
    cmd.append(argv[10]);
    cmd.append(" result.bmp");
    int _ = std::system(cmd.data()); (void)_;
    #endif
    out_fifo.close();
    in__fifo.close();
    return 0;
}