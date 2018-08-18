cmake_minimum_required (VERSION 2.6)
project (visual_odom)
find_package( OpenCV REQUIRED )


file(GLOB visual_odom_src
      "odom/*.hpp"
      "odom/*.cpp")

add_executable(visual_odom ${visual_odom_src})
target_link_libraries(visual_odom ${OpenCV_LIBS} phidget22 )


add_executable(phidget_test phidget_test.c)
target_link_libraries(phidget_test phidget22 )
