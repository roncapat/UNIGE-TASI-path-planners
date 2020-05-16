cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)
include_directories(../ProjectToolkit/include)
add_executable(field_d_planner
        FieldDPlanner.cpp
               main.cpp)
target_link_libraries(field_d_planner gcov)
target_link_libraries(field_d_planner pthread)
target_link_libraries(field_d_planner ProjectToolkit)

add_custom_command(TARGET field_d_planner
        POST_BUILD
        COMMAND sudo setcap 'cap_sys_nice=eip' "${CMAKE_CURRENT_BINARY_DIR}/field_d_planner")