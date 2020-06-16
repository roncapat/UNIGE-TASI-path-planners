#include <iostream>
#include <string>
#include <chrono>
#include <iomanip>
#include <fstream>
#include <pthread.h>
#include "FieldDPlanner.h"
#include "LinearInterpolationPathExtractor.h"
#include "Graph.h"

#ifndef OPT_LVL
#define OPT_LVL 1
#endif

int main(int _argc, char **_argv) {
    if (_argc < 12) {
        std::cerr << "Missing required argument." << std::endl;
        std::cerr << "Usage:" << std::endl;
        std::cerr << "\t" << _argv[0] << " <mapfile> <from_x> <from_y> <to_x> <to_y> "
                                         "<cspace> <fifo_in> <fifo_out> <gui> <tof> <outpath>"
                  << std::endl;
        return 1;
    }

    auto from_x = std::stof(_argv[2]);
    auto from_y = std::stof(_argv[3]);
    auto to_x = std::stof(_argv[4]);
    auto to_y = std::stof(_argv[5]);
    auto fifo_in = _argv[7];
    auto fifo_out = _argv[8];
    bool tof = bool(std::stoi(_argv[10]));

    Position next_point, goal;
    float next_step_cost = 0;

    cpu_set_t cset;
    CPU_ZERO( &cset);
    CPU_SET( 0, &cset);
    auto ret = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cset);
    if (ret != 0) abort();

    struct sched_param priomax;
    priomax.sched_priority=sched_get_priority_max(SCHED_FIFO);

    ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &priomax);
    std::cout << ret << std::endl;
    if (ret != 0)
        std::cout << "No privileges for setting maximum scheduling priority" << std::endl;

    char ack = -1;
    long size;
    int32_t width = 0, height = 0, top = 0, left = 0, min = 0;
    std::ifstream in__fifo{fifo_in, std::ios::in | std::ios::binary};
    std::ofstream out_fifo{fifo_out, std::ios::out | std::ios::binary};

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

    next_point.x = from_x;
    next_point.y = from_y;

    goal.x = to_x;
    goal.y = to_y;

    FieldDPlanner<OPT_LVL> planner{};
    LinearInterpolationPathExtractor<typename FieldDPlanner<OPT_LVL>::Base::Info> extractor(planner.get_expanded_map(), planner.get_grid());
    planner.reset();
    planner.set_occupancy_threshold(1);
    planner.set_heuristic_multiplier(min);
    planner.set_map(data, width, height);
    planner.set_start(next_point);
    planner.set_goal(goal);

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

        planner.step();
        extractor.extract_path();

        ack = 3;
        out_fifo.write((char *) &ack, 1);
        auto path_size = (int) extractor.path_.size();
        out_fifo.write((char *) &path_size, 4);
        for (const auto &pose : extractor.path_) {
            out_fifo.write((char *) &(pose.x), 4);
            out_fifo.write((char *) &(pose.y), 4);
        }
        out_fifo.flush();
        for (const auto &step_cost: extractor.cost_) {
            out_fifo.write((char *) &(step_cost), 4);
        }
        out_fifo.flush();
        out_fifo.write((char *) &(extractor.total_dist), 4);
        out_fifo.write((char *) &(extractor.total_cost), 4);
        out_fifo.flush();
        out_fifo.write((char *) &(planner.u_time), 4);
        out_fifo.write((char *) &(planner.p_time), 4);
        out_fifo.write((char *) &(extractor.e_time), 4);
        out_fifo.flush();

        if (tof){
            ack = 4;
            out_fifo.write((char *) &ack, 1);
            auto expanded_size = (long long) planner.map.size();
            out_fifo.write((char *) &expanded_size, 8);
            for (auto b: planner.map.buckets){
                for (const auto &expanded : b) {
                    const Node &exp = expanded.first;
                    float g,rhs;
                    std::tie(g,rhs,std::ignore) = expanded.second;
                    out_fifo.write((char *) &(exp.x), 4);
                    out_fifo.write((char *) &(exp.y), 4);
                    out_fifo.write((char *) &(g), 4);
                    out_fifo.write((char *) &(rhs), 4);
                }
                out_fifo.flush();
            }
        }
        auto prev_point = next_point;
        for (unsigned int i = 1; i < extractor.path_.size(); i++){
            next_point = {extractor.path_[i].x, extractor.path_[i].y};
            next_step_cost = extractor.cost_[i-1];
            if (Cell(next_point).distance(Cell(prev_point))>5)
                break;
        }
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
    in__fifo.close();
    out_fifo.close();
    return 0;
}