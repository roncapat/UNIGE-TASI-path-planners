cmake_minimum_required(VERSION 3.9)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include_directories(SYSTEM ../BoostHeap)
include_directories(SYSTEM ../RobinHoodHashMap/src/include)
include_directories(SYSTEM ../TartanLlamaOptional/include)
include_directories(include)
add_library(ProjectToolkit STATIC
            Graph.cpp
            Macros.cpp
            InterpolatedTraversal.cpp
            Interpolation.cpp
            Node.cpp
            Position.cpp
            Cell.cpp
            builtin_unreachable.cpp)