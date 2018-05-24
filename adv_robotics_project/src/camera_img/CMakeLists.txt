cmake_minimum_required(VERSION 2.8)
project(camera_img)

find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport message_generation sensor_msgs)

# add the resized image message
add_message_files(DIRECTORY msg
   FILES 
)
generate_messages(DEPENDENCIES sensor_msgs)

catkin_package(CATKIN_DEPENDS cv_bridge image_transport message_runtime sensor_msgs)

find_package(OpenCV)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# add the image publisher executable
add_executable(image_publisher src/image_pub.cpp)
add_dependencies(image_publisher ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(image_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} -lpthread -ludev -lv4l2)

# Mark executables and/or libraries for installation
install(TARGETS image_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# add the image publisher executable
add_executable(image_blur src/blur.cpp)
add_dependencies(image_blur ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(image_blur ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} -lpthread -ludev -lv4l2)

# Mark executables and/or libraries for installation
install(TARGETS image_blur
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

