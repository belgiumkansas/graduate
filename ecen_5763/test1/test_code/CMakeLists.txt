cmake_minimum_required (VERSION 2.6)
project (test_code)
find_package( OpenCV REQUIRED )
find_package (Threads)

add_executable(test_code test_code.cpp)
target_link_libraries( test_code ${OpenCV_LIBS} )
target_link_libraries (test_code ${CMAKE_THREAD_LIBS_INIT})
