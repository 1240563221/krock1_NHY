#This is a CMakeLists.txt
cmake_minimum_required(VERSION 3.13)
project(yaml_test_project)

# set(SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/.cpp)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src SRC)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/)
# include_directories(/usr/local/include/yaml-cpp/)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/)

set(MACRO -DMY_DEBUG)
add_definitions(${MACRO})

add_executable(app ${SRC})
target_link_libraries(app dxl_x64_cpp pthread ${CMAKE_CURRENT_SOURCE_DIR}/include/libyaml-cpp.a)
