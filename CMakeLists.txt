cmake_minimum_required(VERSION 2.8.3)
project(roncapat_global_planners)
set(CMAKE_CXX_STANDARD 11)
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
#    add_compile_options(-DVERBOSE_EXTRACTION)
ENDIF (CMAKE_BUILD_TYPE MATCHES Debug)

IF (CMAKE_BUILD_TYPE MATCHES DebugNoAssert)
    #add_compile_options(-DVERBOSE_EXTRACTION)
    add_compile_options(-DNDEBUG)
ENDIF (CMAKE_BUILD_TYPE MATCHES DebugNoAssert)

add_subdirectory(Simulator)
add_subdirectory(ProjectToolkit)
add_subdirectory(RobinHoodHashMap)

set(OPT_LVL 0)
set(NO_HEURISTIC 1)
add_subdirectory(Tests/Planners/FDSTAR FD_0_no_heur)
set(NO_HEURISTIC 0)
add_subdirectory(Tests/Planners/FDSTAR FD_0_heur)
set(OPT_LVL 1)
set(NO_HEURISTIC 1)
add_subdirectory(Tests/Planners/FDSTAR FD_1_no_heur)
set(NO_HEURISTIC 0)
add_subdirectory(Tests/Planners/FDSTAR FD_1_heur)
set(OPT_LVL 0)
set(NO_HEURISTIC 1) # up to now DFM only works with no heuristic reliably
add_subdirectory(Tests/Planners/DFM DFM_0)
set(OPT_LVL 1)
add_subdirectory(Tests/Planners/DFM DFM_1)
set(OPT_LVL 0)
set(NO_HEURISTIC 1)
add_subdirectory(Tests/Planners/SGDFM SGDFM_0_no_heur)
set(NO_HEURISTIC 0)
add_subdirectory(Tests/Planners/SGDFM SGDFM_0_heur)
set(OPT_LVL 1)
set(NO_HEURISTIC 1)
add_subdirectory(Tests/Planners/SGDFM SGDFM_1_no_heur)
set(NO_HEURISTIC 0)
add_subdirectory(Tests/Planners/SGDFM SGDFM_1_heur)
set(OPT_LVL 2)
set(NO_HEURISTIC 1)
add_subdirectory(Tests/Planners/SGDFM SGDFM_2_no_heur)
set(NO_HEURISTIC 0)
add_subdirectory(Tests/Planners/SGDFM SGDFM_2_heur)

add_dependencies(ProjectToolkit robin_hood)
foreach(target
        field_d_planner_0_heur
        field_d_planner_1_heur
        field_d_planner_0_no_heur
        field_d_planner_1_no_heur
        dfm_planner_0
        dfm_planner_1
        shifted_grid_planner_0_heur
        shifted_grid_planner_1_heur
        shifted_grid_planner_2_heur
        shifted_grid_planner_0_no_heur
        shifted_grid_planner_1_no_heur
        shifted_grid_planner_2_no_heur
        )
    add_dependencies(${target} simulator ProjectToolkit)
endforeach()


execute_process(COMMAND rm -rf "${CMAKE_BINARY_DIR}/pipe_1" "${CMAKE_BINARY_DIR}/pipe_2")
execute_process(COMMAND mkfifo "${CMAKE_BINARY_DIR}/pipe_1" "${CMAKE_BINARY_DIR}/pipe_2")