cmake_minimum_required (VERSION 2.6)
project (vio_odom)
find_package( OpenCV REQUIRED )

set (CMAKE_CXX_STANDARD 11)

file(GLOB vio_odom_src
      "src/*.hpp"
      "src/*.cpp"
      "src/*.h")

add_executable(vio_odom ${vio_odom_src})
target_link_libraries(vio_odom ${OpenCV_LIBS} phidget22 )
