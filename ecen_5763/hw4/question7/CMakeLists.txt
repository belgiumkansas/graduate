cmake_minimum_required (VERSION 2.6)
project (capture_stereo)
find_package( OpenCV REQUIRED )
add_executable(capture_stereo capture_stereo.cpp)
target_link_libraries(capture_stereo ${OpenCV_LIBS} )
