cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (NOT DEFINED OPT_LVL)
    set(OPT_LVL 1)
endif ()
add_definitions(-DOPT_LVL=${OPT_LVL})
#add_compile_options(-fopenmp)
#add_compile_options(-fsanitize=address -fsanitize=undefined)
#add_compile_options(-DVERBOSE_EXTRACTION)
include_directories(SYSTEM ../RobinHoodHashMap/src/include)
include_directories(SYSTEM ../TartanLlamaOptional/include)
include_directories(SYSTEM ../BoostHeap)
include_directories(include)
include_directories( ../PathExtraction)
include_directories(../ProjectToolkit/include)
add_executable(dfm_planner_${OPT_LVL}
               DynamicFastMarching_impl.h
               main.cpp)

#find_package(OpenMP)
#if(OpenMP_CXX_FOUND)
#    target_link_libraries(dfm_planner_${OPT_LVL} OpenMP::OpenMP_CXX)
#endif()
#target_link_libraries(dfm_planner_${OPT_LVL} ubsan)
#target_link_libraries(dfm_planner_${OPT_LVL} -fsanitize=address)
target_link_libraries(dfm_planner_${OPT_LVL} gcov)
target_link_libraries(dfm_planner_${OPT_LVL} pthread)
target_link_libraries(dfm_planner_${OPT_LVL} ProjectToolkit)
target_link_libraries(dfm_planner_${OPT_LVL} robin_hood)

add_custom_command(TARGET dfm_planner_${OPT_LVL}
                   POST_BUILD
                   COMMAND sudo setcap 'cap_sys_nice=eip' "${CMAKE_CURRENT_BINARY_DIR}/dfm_planner_${OPT_LVL}")
