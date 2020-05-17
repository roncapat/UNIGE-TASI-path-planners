cmake_minimum_required(VERSION 2.8.3)
project(roncapat_global_planners)

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
ENDIF (CMAKE_BUILD_TYPE MATCHES Debug)

IF (CMAKE_BUILD_TYPE MATCHES DebugNoAssert)
    #add_compile_options(-DVERBOSE_EXTRACTION)
    add_compile_options(-DNDEBUG)
ENDIF (CMAKE_BUILD_TYPE MATCHES DebugNoAssert)

add_subdirectory(Simulator)
add_subdirectory(FieldDStar)
add_subdirectory(DynamicFastMarching)
add_subdirectory(ShiftedGridFastMarching)
add_subdirectory(ProjectToolkit)
add_subdirectory(RobinHoodHashMap)

add_dependencies(ProjectToolkit robin_hood)
add_dependencies(field_d_planner simulator ProjectToolkit)
add_dependencies(dfm_planner simulator ProjectToolkit)
add_dependencies(shifted_grid_planner simulator ProjectToolkit)

execute_process(COMMAND rm -rf "${CMAKE_BINARY_DIR}/pipe_1" "${CMAKE_BINARY_DIR}/pipe_2")
execute_process(COMMAND mkfifo "${CMAKE_BINARY_DIR}/pipe_1" "${CMAKE_BINARY_DIR}/pipe_2")