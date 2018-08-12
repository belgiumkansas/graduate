cmake_minimum_required (VERSION 2.6)
project (visual_odom_test)
find_package( OpenCV REQUIRED )

add_executable(visual_odom_test visual_odom_test.cpp)
target_link_libraries(visual_odom_test ${OpenCV_LIBS} )
