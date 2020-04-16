#include <iostream>
#include <string>
#include <chrono>
#include <iomanip>
#include "FieldDPlanner.h"
#include "bitmap/BMP.h"
#include "Graph.h"
char **argv;

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

int main(int _argc, char **_argv) {
    argv = _argv;
    if (_argc < 11) {
        std::cerr << "Missing required argument." << std::endl;
        std::cerr << "Usage:" << std::endl;
        std::cerr << "\t" << argv[0]
                  << " <mapfile.bmp> <from_x> <from_y> <to_x> <to_y>"
                     " lookahead cspace optimiziation_lvl <logfile.json> <dbgfile.json> <infofile.json>"
                  << std::endl;
        return 1;
    }
    auto map = BMP(argv[1]);
    // Flip color schema. In BMP, white is 255 and black 0.
    // Here, obstacles (black) are 255 and free space is lighter
    std::transform(map.dataptr.get(),
                   map.dataptr.get() + map.size,
                   map.dataptr.get(),
                   [](auto v) { return 255 - v; });
    // Do not allow free paths
    std::transform(map.dataptr.get(),
                   map.dataptr.get() + map.size,
                   map.dataptr.get(),
                   [](auto v) { return v == 0 ? 1 : v; });

    /*
    TODO: transform those lines in assertions on validity of start and goal positions
    std::cout << std::to_string(map.data[24][8]) << std::endl;
    std::cout << std::to_string(map.data[24][24]) << std::endl;
    std::cout << std::to_string(map.data[24][40]) << std::endl;
    */
    map_info = std::make_shared<Map>(Map{
        .image = map.dataptr,
        .resolution = 1,
        .orientation = 0,
        .length = map.height,
        .width = map.width,
        .x = std::stoi(argv[2]),
        .y = std::stoi(argv[3]),
        .x_initial = 0,
        .y_initial = 0
    });

    float avg = 0;
    int count = 0, min = 254, max = 0;
    for (auto p = map.dataptr.get(); p < (map.dataptr.get() + map.size); ++p) {
        if (*p < 255) {
            avg += (float) *p;
            ++count;
            if (*p < min) min = *p;
            if (*p > max) max = *p;
        }
    }
    avg /= (float) count;
    std::cout << "Average traversability: " << avg << std::endl;
    std::cout << "Minimum traversability: " << min << std::endl;
    std::cout << "Maximum traversability: " << max << std::endl;

    FieldDPlanner planner{};
    planner.occupancy_threshold_ = 1;
    planner.configuration_space_ = std::stoi(argv[7]);
    planner.optimization_lvl = std::stoi(argv[8]);
    planner.first_run_trick = true;
    planner.init();
    planner.set_map(map_info);
    planner.set_goal({std::stoi(argv[4]), std::stoi(argv[5])});
    planner.set_lookahead(std::stoi(argv[6]));
    //planner.set_heuristic_multiplier(std::ceil(0.5*min)); // Ferguson and Stenz heuristic
    planner.set_heuristic_multiplier(std::ceil(min)); // What I think is correct (consistent?)
    planner.set_poses_cb(poses_cb);
    planner.set_expanded_cb(expanded_cb);
    auto begin = std::chrono::steady_clock::now();
    planner.step();
    auto end = std::chrono::steady_clock::now();
    planner.publish_expanded_set();
    planner.publish_path();

    auto time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
    auto time_us = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
    std::cout << "Step time = " << time_ms+time_us/1000.0f << std::endl;

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
    infofile << "\"map_size\":" << map.width*map.height;
    infofile << ",";
    infofile << "\"map_width\":" << map.width;
    infofile << ",";
    infofile << "\"map_height\":" << map.height;
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
    infofile << "\"step_time\":" << time_ms+time_us/1000.0f;
    infofile << ",";
    infofile << "\"path_length\":" << g_length;
    infofile << ",";
    infofile << "\"path_cost\":" << g_cost;
    infofile << "}";
    infofile.close();

    #ifdef FDSTAR_SHOW_RESULT
    std::string cmd = "python3 plot_path_gui.py ";
    cmd.append(argv[1]);
    cmd.append(" ");
    cmd.append(argv[9]);
    cmd.append(" ");
    cmd.append(argv[10]);
    cmd.append(" result.bmp");
    int _ = std::system(cmd.data()); (void)_;
    #endif

    return 0;
}