# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++11 -Wno-deprecated-declarations")

project(libuvc_camera)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp camera_info_manager dynamic_reconfigure image_transport nodelet sensor_msgs)

generate_dynamic_reconfigure_options(cfg/UVCCamera.cfg)

#find_package(libuvc REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/libuvc_ros/libuvc_camera")
message("Module path: ${CMAKE_MODULE_PATH}")
find_package(libusb-1.0 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    camera_info_manager
    dynamic_reconfigure
    image_transport
    nodelet
    sensor_msgs
  LIBRARIES libuvc_camera_nodelet
  )

include_directories(include ${LIBUSB_1_INCLUDE_DIRS} libuvc/include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LINK_DIRS})

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})
add_library(uvc_custom libuvc/src/ctrl.c
			  libuvc/src/ctrl-gen.c
			  libuvc/src/device.c
			  libuvc/src/diag.c
			  libuvc/src/frame.c
			  libuvc/src/frame-mjpeg.c
			  libuvc/src/init.c
			  libuvc/src/misc.c
			  libuvc/src/stream.c)
target_link_libraries(uvc_custom jpeg ${LIBUSB_1_LIBRARIES})

add_executable(camera_node src/main.cpp src/camera_driver.cpp)
target_link_libraries(camera_node ${LIBUSB_1_LIBRARIES} uvc_custom ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(camera_node ${PROJECT_NAME}_gencfg)

add_library(libuvc_camera_nodelet src/nodelet.cpp src/camera_driver.cpp)
add_dependencies(libuvc_camera_nodelet ${libuvc_camera_EXPORTED_TARGETS})
target_link_libraries(libuvc_camera_nodelet ${LIBUSB_1_LIBRARIES} ${libuvc_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(libuvc_camera_nodelet ${PROJECT_NAME}_gencfg)

install(TARGETS camera_node libuvc_camera_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES libuvc_camera_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
