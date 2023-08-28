#This is a CMakeLists.txt
cmake_minimum_required(VERSION 3.13)
project(krock1_project)

set(SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/group_bulkread.cpp)
include_directories(/usr/local/include/dynamixel_sdk/)

set(MACRO -DMY_DEBUG)
add_definitions(${MACRO})

add_executable(ping_test ${SRC})
target_link_libraries(ping_test dxl_x64_cpp)