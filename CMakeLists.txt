cmake_minimum_required(VERSION 3.17)
project(Vision)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_subdirectory(src)
add_subdirectory(calibration)