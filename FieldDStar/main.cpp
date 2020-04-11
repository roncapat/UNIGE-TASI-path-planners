#include <fstream>
#include <iostream>
#include <string>
#include "FieldDPlanner.h"
#include "bitmap/BMP.h"
#include "Graph.h"
char **argv;

std::shared_ptr<Map> map_info = nullptr;
uint8_t logcount = 0;

void poses_cb(std::vector<Pose> poses) {
    std::ofstream logfile;
    std::string filename(argv[7]);
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

void expanded_cb(std::tuple<std::vector<std::tuple<int, int, float>>, int, int> exp_info) {

    std::ofstream logfile;
    std::string filename(argv[8]);
    logfile.open(filename);
    logfile << "{\"num_expanded\": " << std::get<1>(exp_info)
            << ", \"num_updated\": " << std::get<2>(exp_info)
            << ", \"expanded\": [";
    for (auto node : std::get<0>(exp_info)) {
        logfile << "[" << std::to_string(std::get<0>(node))
                << ", " << std::to_string(std::get<1>(node))
                << ", "
                << std::to_string(std::get<2>(node) == std::numeric_limits<float>::infinity() ? -1 : std::get<2>(node))
                << "],";
    }
    logfile.seekp(-1, std::ios::cur);
    logfile << "]}";
    logfile.close();
}

int main(int _argc, char **_argv) {
    argv = _argv;
    if (_argc < 8) {
        std::cerr << "Missing required argument." << std::endl;
        std::cerr << "Usage:" << std::endl;
        std::cerr << "\t" << argv[0]
                  << " <mapfile.bmp> <from_x> <from_y> <to_x> <to_y> lookahead <logfile.json> <dbgfile.json>"
                  << std::endl;
        return 1;
    }
    auto map = BMP(argv[1]);
    // Flip color schema. In BMP, white is 255 and black 0.
    // Here, obstacles (black) are 255 and free space is lighter
    std::transform(map.data.data(),
                   map.data.data() + map.data.num_elements(),
                   map.data.data(),
                   [](auto v) { return 255 - v; });
    // Do not allow free paths
    std::transform(map.data.data(),
                   map.data.data() + map.data.num_elements(),
                   map.data.data(),
                   [](auto v) { return v == 0 ? 1 : v; });

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
        .x = std::stoi(argv[2]),//27,
        .y = std::stoi(argv[3]),// 10,
        .x_initial = 0,
        .y_initial = 0
    });
    FieldDPlanner planner{};
    planner.lookahead = bool(std::stoi(argv[6]));
    planner.init();
    planner.set_map(map_info);
    planner.set_goal({std::stoi(argv[4]), std::stoi(argv[5])});
    planner.set_poses_cb(poses_cb);
    planner.set_expanded_cb(expanded_cb);
    planner.step();
#ifdef FDSTAR_SHOW_RESULT
    std::string cmd = "python3 plot_path_gui.py ";
    cmd.append(argv[1]);
    cmd.append(" ");
    cmd.append(argv[7]);
    cmd.append(" ");
    cmd.append(argv[8]);
    cmd.append(" result.bmp");
    std::system(cmd.data());
#endif
    return 0;
}