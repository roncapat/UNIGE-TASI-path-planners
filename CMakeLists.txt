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

find_package(PCL REQUIRED)
find_package(Boost REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(field_d_planner
               FieldDPlanner.cpp
               main.cpp
               Graph.cpp
               Node.cpp
               NodeUtils.hpp
               PriorityQueue.cpp)

target_link_libraries(field_d_planner
                      ${PCL_LIBRARIES}
                      ${Boost_LIBRARIES})
