#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(RealSenseVO)
find_package(OpenCV REQUIRED )
set (CMAKE_CXX_STANDARD 11)

add_executable(RealSenseVO main.cpp)
target_link_libraries(RealSenseVO realsense2 ${OpenCV_LIBS})
