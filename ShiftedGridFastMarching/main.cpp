#include <iostream>
#include <string>
#include <chrono>
#include <iomanip>
#include <fstream>
#include <pthread.h>
#include "ShiftedGridPlanner.h"
#include "Graph.h"

int main(int _argc, char **_argv) {
    if (_argc < 13) {
        std::cerr << "Missing required argument." << std::endl;
        std::cerr << "Usage:" << std::endl;
        std::cerr << "\t" << _argv[0]
                  << " <mapfile.bmp> <from_x> <from_y> <to_x> <to_y>"
                     " lookahead cspace optimiziation_lvl <fifo_in> <fifo_out> <gui> <outpath>"
                  << std::endl;
        return 1;
    }

    Position next_point, goal;
    float next_step_cost = 0;
    auto res = std::system((std::string("python3 -m simulator.run_simulator ") +
                            _argv[1] + " " + _argv[7] + " " + _argv[10] + " " + _argv[9] + " " + _argv[11] + " " +
                            _argv[12] + " 'SG-DFM V"+_argv[8]+"' n &").data());
    (void) res;

    cpu_set_t cset;
    CPU_ZERO( &cset);
    CPU_SET( 0, &cset);
    auto ret = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cset);
    if (ret != 0) abort();

    struct sched_param priomax;
    priomax.sched_priority=sched_get_priority_max(SCHED_FIFO);

    ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &priomax);
    if (ret != 0)
        std::cout << "No privileges for setting maximum scheduling priority" << std::endl;

    char ack = -1;
    long size;
    int32_t width = 0, height = 0, top = 0, left = 0, min = 0;
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
    in__fifo.read((char *) &min, 4); //Receive heuristic hint

    next_point.x = std::stof(_argv[2]);
    next_point.y = std::stof(_argv[3]);

    goal.x = std::stof(_argv[4]);
    goal.y = std::stof(_argv[5]);

    ShiftedGridPlanner planner{};
    planner.init();
    planner.set_optimization_lvl(std::stoi(_argv[8]));
    planner.set_first_run_trick(false);
    planner.set_occupancy_threshold(1);
    planner.set_lookahead(std::stoi(_argv[6]));
    // FIXME handle remplanning, for now i put the lowest possible value for consistency
    planner.set_heuristic_multiplier(min);
    planner.set_map(data, width, height);
    planner.set_start(next_point);
    planner.set_goal(goal);

    float time = 0;

    while (true) {
        std::cout << "[PLANNER]   New position: [" << next_point.x << ", " << next_point.y << "]" << std::endl;
        ack = 1;
        out_fifo.write((char *) &ack, 1);
        out_fifo.write((char *) &(next_point.x), 4);
        out_fifo.write((char *) &(next_point.y), 4);
        out_fifo.write((char *) &(next_step_cost), 4);
        out_fifo.flush();
        ack = -1;
        while (ack != 1) {
            in__fifo.read((char *) &ack, 1); //Wait for 1
        }
        in__fifo.read((char *) &top, 4);
        in__fifo.read((char *) &left, 4);
        in__fifo.read((char *) &height, 4);
        in__fifo.read((char *) &width, 4);
        std::cout << "[PLANNER]   New patch: position [" << top << ", " << left
                  << "], shape [" << width << ", " << height << "]" << std::endl;
        size = width * height;
        std::shared_ptr<uint8_t[]> patch(new uint8_t[size], std::default_delete<uint8_t[]>());
        in__fifo.read((char *) patch.get(), size); //Receive image
        planner.patch_map(patch, top, left, width, height);
        in__fifo.read((char *) &min, 4); //Receive heuristic hint
        planner.set_heuristic_multiplier(min);

        auto begin = std::chrono::steady_clock::now();
        planner.step();
        auto end = std::chrono::steady_clock::now();

        time += std::chrono::duration<float, std::milli>(end - begin).count();

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
        out_fifo.flush();
        out_fifo.write((char *) &(planner.u_time), 4);
        out_fifo.write((char *) &(planner.p_time), 4);
        out_fifo.write((char *) &(planner.e_time), 4);
        out_fifo.flush();

        ack = 4;
        out_fifo.write((char *) &ack, 1);
        auto expanded_size = (long long) planner.map.size();
        out_fifo.write((char *) &expanded_size, 8);
        for (const auto &expanded : planner.map) {
            const Node &exp = expanded.first;
            auto[g, rhs, _] = expanded.second;
            (void) _;
            out_fifo.write((char *) &(exp.x), 4);
            out_fifo.write((char *) &(exp.y), 4);
            out_fifo.write((char *) &(g), 4);
            out_fifo.write((char *) &(rhs), 4);
        }
        out_fifo.flush();

        next_point = {planner.path_[1].x, planner.path_[1].y};
        next_step_cost = planner.cost_.front();
        if (next_point == goal)
            break; //Goal reached
        planner.set_start(next_point);
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
    std::cout << "Cumulative planning time = " << time << std::endl;
    return 0;
}