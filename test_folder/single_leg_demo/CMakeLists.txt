#This is a CMakeLists.txt
cmake_minimum_required(VERSION 3.13)
project(single_leg_project)

#get all the source
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} SRC)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/tools/utils SRC1)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/tools/log SRC2)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/application/servo_driver/source SRC3)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/application/locomotion_plan/source SRC4)
set(SRC ${SRC};${SRC1};${SRC2};${SRC3};${SRC4})

#set include directories---from the project
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/tools/utils)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/tools/log)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/application/locomotion_plan/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/application/servo_driver/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/tools)

#set include directories---from the PC(dynamixel and yaml-cpp)
include_directories(/usr/local/include/dynamixel_sdk)
include_directories(/usr/local/include/yaml-cpp/)

#set the debug macro
set(MACRO -DMY_DEBUG)
add_definitions(${MACRO})

add_executable(app ${SRC})
target_link_libraries(app dxl_x64_cpp pthread ${CMAKE_CURRENT_SOURCE_DIR}/library/libyaml-cpp.a qpOASES)
