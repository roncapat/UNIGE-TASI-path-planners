cmake_minimum_required(VERSION 2.8.3)
project(roncapat_global_planners)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-W -Wall -Wextra)

# set default build type
IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
    add_compile_options(-march=native)
ENDIF (NOT CMAKE_BUILD_TYPE)

add_executable(field_d_planner
               FieldDPlanner.cpp
               main.cpp
               Graph.cpp
               Node.cpp
               PriorityQueue.cpp)