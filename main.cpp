#include <fstream>
#include <iostream>
#include <string>
#include "FieldDPlanner.h"
#include "include/bitmap/BMP.h"
#include "Graph.h"

std::shared_ptr<Map> map_info = nullptr;
uint8_t logcount = 0;

void poses_cb(std::vector<Pose> poses) {
    std::ofstream logfile;
    std::string filename = "logfile_" + std::to_string(logcount) + ".json";
    logfile.open(filename);
    logfile << "{\"poses\": [";
    for (auto pose : poses) {
        logfile << "[" << std::to_string(pose.x)
                << ", " << std::to_string(pose.y)
                << ", " << std::to_string(pose.orientation)
                << "],";
    }
    logfile.seekp(-1, std::ios::cur);
    logfile << "]}";
    logfile.close();
}

void expanded_cb(std::tuple<std::vector<std::tuple<int, int, float, float>>, int, int> exp_info) {

    std::ofstream logfile;
    std::string filename = "dbgfile_" + std::to_string(logcount) + ".json";
    logfile.open(filename);
    logfile << "{\"num_expanded\": " << std::get<1>(exp_info)
            << ", \"num_updated\": " << std::get<2>(exp_info)
            << ", \"expanded\": [";
    for (auto node : std::get<0>(exp_info)) {
        logfile << "[" << std::to_string(std::get<0>(node))
                << ", " << std::to_string(std::get<1>(node))
                << ", "
                << std::to_string(std::get<2>(node) == std::numeric_limits<float>::infinity() ? -1 : std::get<2>(node))
                << ", "
                << std::to_string(std::get<3>(node) == std::numeric_limits<float>::infinity() ? -1 : std::get<3>(node))
                << "],";
    }
    logfile.seekp(-1, std::ios::cur);
    logfile << "]}";
    logfile.close();
}

int main(int, char **) {
    auto map = BMP("test.bmp");
    std::transform(map.data.data(),
                   map.data.data() + map.data.num_elements(),
                   map.data.data(),
                   [](auto v) { return 255 - v; });

    /*
    TODO: transform those lines in assertions on validity of start and goal positions
    std::cout << std::to_string(map.data[24][8]) << std::endl;
    std::cout << std::to_string(map.data[24][24]) << std::endl;
    std::cout << std::to_string(map.data[24][40]) << std::endl;
    */
    map_info = std::make_shared<Map>(Map{
        .image = map.data,
        .resolution = 1,
        .orientation = 0,
        .length = map.bmp_info_header.height,
        .width = map.bmp_info_header.width,
        .x = 24,
        .y = 8,
        .x_initial = 0,
        .y_initial = 0
    });
    FieldDPlanner planner{};
    planner.init();
    planner.set_map(map_info);
    planner.set_goal({24, 40});
    planner.set_poses_cb(poses_cb);
    planner.set_expanded_cb(expanded_cb);
    planner.step();
    return 0;
}