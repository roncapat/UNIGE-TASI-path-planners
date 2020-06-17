#include <iostream>
#include <string>
#include <chrono>
#include <iomanip>
#include <cstdio>
#include <fstream>
#include <pthread.h>
#include "DynamicFastMarching.h"
#include "DirectLinearInterpolationPathExtractorCells.h"
#include "Graph.h"

#include <rtems.h>
/* The Console Driver supplies Standard I/O. */
#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER
/* The Clock Driver supplies the clock tick. */
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
#define CONFIGURE_MAXIMUM_TASKS 2
#define CONFIGURE_INIT_TASK_NAME rtems_build_name( 'D', 'E', 'M', 'O' )
#define CONFIGURE_RTEMS_INIT_TASKS_TABLE
#define CONFIGURE_INIT_TASK_ATTRIBUTES RTEMS_FLOATING_POINT
#define CONFIGURE_INIT
#include <rtems/confdefs.h>

#ifndef OPT_LVL
#define OPT_LVL 1
#endif

extern "C"
{
extern rtems_task Init(rtems_task_argument);
}

rtems_task Init(rtems_task_argument) {

    Position next_point, goal;

    // FIXME maybe I compiler RTEMS with no SMP support, so this is not avaiable 
    //cpu_set_t cset;
    //CPU_ZERO( &cset);
    //CPU_SET( 0, &cset);
    //auto ret = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cset); //TODO not declared in RTEMS?
    //if (ret != 0) abort();

    struct sched_param priomax;
    priomax.sched_priority=sched_get_priority_max(SCHED_FIFO);

    auto ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &priomax);
    std::cout << ret << std::endl;
    if (ret != 0)
        std::cout << "No privileges for setting maximum scheduling priority" << std::endl;

    long size;
    int32_t width = 0, height = 0;

    width = height = 10;

    size = width * height;
    std::shared_ptr<uint8_t[]> data(new uint8_t[size], std::default_delete<uint8_t[]>());
    memset(data.get(), 1, size);
    
    next_point.x = 2;
    next_point.y = 2;

    goal.x = 8;
    goal.y = 8;

    DFMPlanner<OPT_LVL> planner{};
    DirectLinearInterpolationPathExtractorCells<typename DFMPlanner<OPT_LVL>::Base::Info> extractor(planner.get_expanded_map(), planner.get_grid());
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

    std::shared_ptr<uint8_t[]> patch(new uint8_t[4], std::default_delete<uint8_t[]>());
    memset(data.get(), 10, 4);    
    planner.patch_map(patch, 4, 4, 2, 2);

    planner.step();
    extractor.extract_path();

    std::cout << "PATH" << std::endl;
    for (const auto &pose : extractor.path_)
      std::cout << "\tSTEP X=" << pose.x << " Y=" << pose.y << std::endl;
    std::cout << "TIME: " << planner.p_time << std::endl;
    exit(EXIT_SUCCESS);
}
