cmake_minimum_required (VERSION 2.6)
project (final_project)
find_package( OpenCV REQUIRED )

add_executable(flow_test flow_test.cpp)
target_link_libraries(flow_test ${OpenCV_LIBS} )

add_executable(record_cv record_cv.cpp)
target_link_libraries(record_cv ${OpenCV_LIBS} )
