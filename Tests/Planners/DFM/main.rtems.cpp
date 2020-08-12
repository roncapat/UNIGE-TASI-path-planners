#define CONFIGURE_INIT
#include <rtems.h>
#include <bsp.h>
#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
#define CONFIGURE_MAXIMUM_TASKS 2
#define CONFIGURE_MAXIMUM_SEMAPHORES 5
#define CONFIGURE_INIT_TASK_NAME rtems_build_name( 'D', 'E', 'M', 'O' )
#define CONFIGURE_RTEMS_INIT_TASKS_TABLE
#define CONFIGURE_INIT_TASK_STACK_SIZE    (RTEMS_MINIMUM_STACK_SIZE * 2)
#define CONFIGURE_EXTRA_TASK_STACKS       RTEMS_MINIMUM_STACK_SIZE
#define CONFIGURE_INIT_TASK_ATTRIBUTES RTEMS_FLOATING_POINT
#include <rtems/confdefs.h>

#include <iostream>
#include <string>
#include <chrono>
#include <iomanip>
#include <cstdio>
#include <fstream>
#include <pthread.h>
#include "DynamicFastMarching.h"
#include "LinearInterpolationPathExtractor.h"
#include "Graph.h"

#ifndef OPT_LVL
#define OPT_LVL 1
#endif

rtems_task Init(rtems_task_argument) {
    std::cout << std::endl;
    std::cout << "############################" << std::endl;
    std::cout << "#####PATH PLANNER DEMO #####" << std::endl;
    std::cout << "############################" << std::endl;

    Position next_point, goal;

    long size;
    int32_t width = 0, height = 0;

    width = height = 10;

    size = width * height;
    std::shared_ptr<uint8_t> data(new uint8_t[size], std::default_delete<uint8_t[]>());
    memset(data.get(), 1, size);

    next_point.x = 2;
    next_point.y = 2;

    goal.x = 8;
    goal.y = 8;

    DFMPlanner<OPT_LVL> planner{};
    DFMPlanner<OPT_LVL> planner{};
    LinearInterpolationPathExtractor<
        typename DFMPlanner<OPT_LVL>::Map::ElemType,
        typename DFMPlanner<OPT_LVL>::Base::Info>
        extractor(planner.get_expanded_map(), planner.get_grid());
    extractor.allow_indirect_traversals = true;
    planner.reset();
    planner.set_occupancy_threshold(1);
    planner.set_heuristic_multiplier(1);
    planner.set_map(data, width, height);
    planner.set_goal(goal);

    planner.step();
    extractor.extract_path();

    std::cout << "PATH" << std::endl;
    for (const auto &pose : extractor.path_)
      std::cout << "\tSTEP X=" << pose.x << " Y=" << pose.y << std::endl;
    std::cout << "TIME: " << planner.p_time << std::endl;

    next_point = {extractor.path_[0].x, extractor.path_[0].y};

    planner.set_start(next_point);

    std::shared_ptr<uint8_t> patch(new uint8_t[4], std::default_delete<uint8_t[]>());
    memset(patch.get(), 10, 4);
    planner.patch_map(patch, 4, 4, 2, 2);

    planner.step();
    extractor.extract_path();

    std::cout << "PATH" << std::endl;
    for (const auto &pose : extractor.path_)
      std::cout << "\tSTEP X=" << pose.x << " Y=" << pose.y << std::endl;
    std::cout << "TIME: " << planner.p_time << std::endl;
    exit(EXIT_SUCCESS);
}
