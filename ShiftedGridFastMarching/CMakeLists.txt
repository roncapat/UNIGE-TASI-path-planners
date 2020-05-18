cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(SYSTEM ../RobinHoodHashMap/src/include)
include_directories(include)
include_directories(../ProjectToolkit/include)
add_executable(shifted_grid_planner
        ShiftedGridPlanner_impl.h
        main.cpp)
target_link_libraries(shifted_grid_planner gcov)
target_link_libraries(shifted_grid_planner pthread)
target_link_libraries(shifted_grid_planner ProjectToolkit)
target_link_libraries(shifted_grid_planner robin_hood)
add_custom_command(TARGET shifted_grid_planner
        POST_BUILD
        COMMAND sudo setcap 'cap_sys_nice=eip' "${CMAKE_CURRENT_BINARY_DIR}/shifted_grid_planner")