#This is a CMakeLists.txt
cmake_minimum_required(VERSION 3.13)
project(krock1_demo)

# set(SRC ${CMAKE_CURRENT_SOURCE_DIR}/Task/source/*.cpp)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/Task/source SRC)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/Driver/source SRC1)
set(SRC ${SRC};${SRC1};${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)
message(${SRC})


#include file directory
include_directories(/usr/local/include/dynamixel_sdk/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Task/include/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Driver/include/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Libraries/utils/)

#add some macro to the project
set(MACRO -DMY_DEBUG)
add_definitions(${MACRO})

#get executable file from the src files
add_executable(app ${SRC})

#link to the shared library
target_link_libraries(app dxl_x64_cpp pthread)
