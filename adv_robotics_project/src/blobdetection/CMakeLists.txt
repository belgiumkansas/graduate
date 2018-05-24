cmake_minimum_required(VERSION 2.8)
project(blobdetection)

find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport message_generation sensor_msgs)

# add the resized image message
add_message_files(DIRECTORY msg
   FILES 
   BALL.msg
)
generate_messages(DEPENDENCIES sensor_msgs std_msgs)

catkin_package(CATKIN_DEPENDS cv_bridge image_transport message_runtime sensor_msgs)

find_package(OpenCV)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# add the image publisher executable
add_executable(image_pub src/image_pub.cpp)
add_dependencies(image_pub ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(image_pub ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# add the blob detector executable
add_executable(blobdetector src/blobdetector.cpp)
add_dependencies(blobdetector ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(blobdetector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# Mark executables and/or libraries for installation
install(TARGETS image_pub blobdetector
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

