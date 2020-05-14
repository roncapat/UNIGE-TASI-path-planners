cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)
include_directories(../../ProjectToolkit/include)
add_executable(dfm_planner
               DynamicFastMarching.cpp
               main.cpp)

target_link_libraries(dfm_planner gcov)
target_link_libraries(dfm_planner ProjectToolkit)