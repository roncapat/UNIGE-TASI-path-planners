cmake_minimum_required(VERSION 3.9)
include_directories(SYSTEM ../BoostHeap)
include_directories(include)
add_library(ProjectToolkit STATIC
            Graph.cpp
            Macros.cpp
            interpolation.cpp)