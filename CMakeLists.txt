cmake_minimum_required(VERSION 3.20.0)
set(CMAKE_CXX_STANDARD 20)

find_package(Zephyr REQUIRED)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(control)
include_directories(${CMAKE_SOURCE_DIR}/include)
FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})
