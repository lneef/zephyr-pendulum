cmake_minimum_required(VERSION 3.20.0)
set(CMAKE_CXX_STANDARD 20)

find_package(Zephyr REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-fenable-matrix)

project(control)
include_directories(${CMAKE_SOURCE_DIR}/include)
FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})
