cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-W -Wall -Wextra)

# set default build type
IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF (NOT CMAKE_BUILD_TYPE)

message("Build type: ${CMAKE_BUILD_TYPE}")

IF (CMAKE_BUILD_TYPE MATCHES Release)
    add_compile_options(-march=native -mtune=native)
ENDIF (CMAKE_BUILD_TYPE MATCHES Release)

IF (CMAKE_BUILD_TYPE MATCHES Debug)
    #add_compile_options(-DVERBOSE_EXTRACTION)
    add_compile_options(--coverage)
ENDIF (CMAKE_BUILD_TYPE MATCHES Debug)

include_directories(include)
include_directories(../include)
add_executable(field_d_planner
               FieldDPlanner.cpp
               main.cpp
               Graph.cpp
               PriorityQueue.cpp
               interpolation.cpp)

target_link_libraries(field_d_planner gcov)

add_executable(field_d_planner_gui
               FieldDPlanner.cpp
               main.cpp
               Graph.cpp
               PriorityQueue.cpp
               interpolation.cpp)

target_link_libraries(field_d_planner_gui gcov)
target_compile_definitions(field_d_planner_gui PRIVATE FDSTAR_SHOW_RESULT)