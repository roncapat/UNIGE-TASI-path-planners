#include <iostream>
#include <string>
#include <chrono>
#include <iomanip>
#include "FieldDPlanner.h"
#include "bitmap/BMP.h"
#include "Graph.h"

Position next_point;
float step_cost = 0;
bool goal_reached = false;

std::shared_ptr<Map> map_info = nullptr;
float g_length = INFINITY;
float g_cost = INFINITY;

std::tuple<int, int, float> map_traversability_stats(uint8_t *data, unsigned int size) {
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
    return {min, max, avg};
}

int main(int _argc, char **_argv) {
    if (_argc < 11) {
        std::cerr << "Missing required argument." << std::endl;
        std::cerr << "Usage:" << std::endl;
        std::cerr << "\t" << _argv[0]
                  << " <mapfile.bmp> <from_x> <from_y> <to_x> <to_y>"
                     " lookahead cspace optimiziation_lvl <fifo_in> <fifo_out>"
                  << std::endl;
        return 1;
    }

    std::system((std::string("python3 ../FieldDStar/run_simulator.py ") +
        _argv[1] + " " + _argv[7] + " " + _argv[10] + " " + _argv[9] + " &").data());

    char ack = -1;
    unsigned int size;
    int32_t width = 0, height = 0, top = 0, left = 0;
    std::ifstream in__fifo{_argv[9], std::ios::in | std::ios::binary};
    std::ofstream out_fifo{_argv[10], std::ios::out | std::ios::binary};

    ack = 0;
    out_fifo.write((char *) &ack, 1); //Send 0
    out_fifo.flush();
    ack = -1;
    while (ack != 0) {
        in__fifo.read((char *) &ack, 1); //Wait for 0
    }
    in__fifo.read((char *) &width, 4);
    in__fifo.read((char *) &height, 4);
    std::cout << "[PLANNER]   Size: [" << width << ", " << height << "]" << std::endl;

    size = width * height;
    std::shared_ptr<uint8_t[]> data(new uint8_t[size], std::default_delete<uint8_t[]>());
    in__fifo.read((char *) data.get(), size); //Receive image

    map_info = std::make_shared<Map>(Map{
        .image = data,
        .resolution = 1,
        .orientation = 0,
        .length = static_cast<int>(height),
        .width = static_cast<int>(width),
        .x = std::stoi(_argv[2]),
        .y = std::stoi(_argv[3]),
        .x_initial = 0,
        .y_initial = 0
    });

    next_point.x = map_info->x;
    next_point.y = map_info->y;

    FieldDPlanner planner{};
    planner.occupancy_threshold_ = 1;
    planner.optimization_lvl = std::stoi(_argv[8]);
    planner.first_run_trick = false;
    planner.init();
    auto[min, max, avg] = map_traversability_stats(data.get(), size);
    std::cout << "Average traversability: " << avg << std::endl;
    std::cout << "Minimum traversability: " << min << std::endl;
    std::cout << "Maximum traversability: " << max << std::endl;

    planner.set_map(map_info);
    planner.set_start_position(next_point);
    planner.set_goal({std::stoi(_argv[4]), std::stoi(_argv[5])});
    planner.set_lookahead(std::stoi(_argv[6]));
    // FIXME handle remplanning, for now i put the lowest possible value for consistency
    //planner.set_heuristic_multiplier(std::ceil(0.5*min)); // Ferguson and Stenz heuristic
    //planner.set_heuristic_multiplier(std::ceil(min)); // What I think is correct (consistent?)
    planner.set_heuristic_multiplier(1);

    float time;

    while (not goal_reached) {
        std::cout << "[PLANNER]   New position: [" << next_point.x << ", " << next_point.y << "]" << std::endl;
        ack = 1;
        out_fifo.write((char *) &ack, 1);
        out_fifo.write((char *) &(next_point.x), 4);
        out_fifo.write((char *) &(next_point.y), 4);
        out_fifo.write((char *) &(step_cost), 4);
        out_fifo.flush();
        ack = -1;
        while (ack != 1) {
            in__fifo.read((char *) &ack, 1); //Wait for 1
        }
        in__fifo.read((char *) &top, 4);
        in__fifo.read((char *) &left, 4);
        in__fifo.read((char *) &height, 4);
        in__fifo.read((char *) &width, 4);
        std::cout << "[PLANNER]   New patch: position [" << top << ", " << left << "], shape [" << width << ", "
                  << height << "]" << std::endl;
        size = width * height;
        std::shared_ptr<uint8_t[]> patch(new uint8_t[size], std::default_delete<uint8_t[]>());
        in__fifo.read((char *) patch.get(), size); //Receive image
        planner.node_grid_.updateGraph(patch, top, left, width, height);

        auto begin = std::chrono::steady_clock::now();
        planner.step();
        auto end = std::chrono::steady_clock::now();

        auto time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
        auto time_us = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
        time = time_ms + time_us / 1000.0f;
        std::cout << "Step time = " << time << std::endl;
        //planner.init(); //PLAN FROM SCRATCH INSTEAD OF REPLANNING

        ack = 3;
        out_fifo.write((char *) &ack, 1);
        auto path_size = (int) planner.path_.size();
        out_fifo.write((char *) &path_size, 4);
        for (const auto &pose : planner.path_) {
            out_fifo.write((char *) &(pose.x), 4);
            out_fifo.write((char *) &(pose.y), 4);
        }
        out_fifo.flush();
        for (const auto &step_cost: planner.cost_) {
            out_fifo.write((char *) &(step_cost), 4);
        }
        out_fifo.flush();
        out_fifo.write((char *) &(planner.total_dist), 4);
        out_fifo.write((char *) &(planner.total_cost), 4);

        ack = 4;
        out_fifo.write((char *) &ack, 1);
        auto expanded_size = (long long) planner.expanded_map_.size();
        out_fifo.write((char *) &expanded_size, 8);
        for (const auto &expanded : planner.expanded_map_) {
            auto[x, y] = expanded.first.getIndex();
            auto[g, rhs] = expanded.second;
            out_fifo.write((char *) &(x), 4);
            out_fifo.write((char *) &(y), 4);
            out_fifo.write((char *) &(g), 4);
            out_fifo.write((char *) &(rhs), 4);
        }
        out_fifo.flush();

        next_point = {planner.path_[1].x, planner.path_[1].y};
        step_cost = planner.cost_.front();
        if (next_point.x == std::stoi(_argv[4]) and next_point.y == std::stoi(_argv[5]))
            break; //Goal reached
        planner.set_start_position(next_point, true);
    }

    ack = 2;
    out_fifo.write((char *) &ack, 1);
    out_fifo.flush();
    ack = -1;
    while (ack != 2) {
        in__fifo.read((char *) &ack, 1); //Wait for 2
    }
/*
    std::ofstream infofile;
    std::string filename(argv[11]);
    infofile.open(filename);
    infofile << "{";
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
    */
    out_fifo.close();
    in__fifo.close();
    return 0;
}