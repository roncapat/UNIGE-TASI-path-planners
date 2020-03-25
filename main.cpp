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

int main(int, char **) {
    auto map = BMP("test.bmp");
    map_info = std::make_shared<Map>(Map{
        .image = map.data,
        .resolution = 1,
        .orientation = 0,
        .length = map.bmp_info_header.height,
        .width = map.bmp_info_header.width,
        .x = 20,
        .y = 20,
        .x_initial = 0,
        .y_initial = 0
    });
    FieldDPlanner planner{};
    planner.init();
    planner.set_map(map_info);
    planner.set_goal({30, 30});
    planner.set_poses_cb(poses_cb);
    //planner.set_expanded_cb(void(*)(std::tuple<pcl::PointCloud<pcl::PointXYZRGB>,int,int>));
    planner.step();
    return 0;
}