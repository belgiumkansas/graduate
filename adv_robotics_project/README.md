# Robotics Project

### Challenges
##### A:
* Ball Avoidance
* Rock Salt/Water
##### B:
* Stop Sign
##### C:
* HRI Paper

## Software stack overview

![software_overview](https://cloud.githubusercontent.com/assets/2038191/24169309/ce3f1ce2-0e42-11e7-8205-12ad96951e97.jpg)


## Phidget imu
 * use script install.sh
 * or:
 * $sudo apt-get install ros-kinetic-phidgets-drivers
 * $cd /opt/ros/kinetic/share/phidgets_api/
 * $sh setup-udev.sh
 * use imu_test.launch too check on operation

## opencv dependencies
 * [OpenCV 3.2](https://github.com/milq/milq/blob/master/scripts/bash/install-opencv.sh) use provided insall script
 * [opencv_contrib](http://www.pyimagesearch.com/2015/06/22/install-opencv-3-0-and-python-2-7-on-ubuntu/)follow steps 10 and 11
 * libv4l, libudev, libopencv-dev. run `sudo apt-get install libv4l-dev libudev-dev libopencv-dev`

## camera driver
 * [libuvc](https://github.com/ktossell/libuvc) a cross-platform library for USB video devices
 * [libuvc_ros](https://github.com/AdvancedRoboticsCUBoulder/libuvc_ros) ROS Driver for USB Video Class Cameras
  
    git clone https://github.com/AdvancedRoboticsCUBoulder/libuvc_ros
    cd libuvc_ros/libuvc_camera/libuvc

    mkdir build

    cd build
    
    cmake ..
 * [lsd_slam](https://github.com/tum-vision/lsd_slam ) LSD-SLAM: Large-Scale Direct Monocular SLAM

   to build lsd_slam: add link_directories( /opt/ros/kinetic/lib ) to lsd_slam_score CMakeLists.txt

   to build message: add add_dependencies(viewer mypackage_generate_messages_cpp)

   Note:install libsuitesparse-dev then install g2o

   disable depth map by changing /lsd_slam/lsd_slam_core/src/util/settings.cpp "bool displayDepthMap = false;"
   run lsd_slam:rosrun lsd_slam_core live_slam image:=/camera/image_raw calib:=/camera/camera_info
   run lsd_slam_viewer:rosrun lsd_slam_viewer viewer

 * [orb_slam](https://github.com/raulmur/ORB_SLAM2) 
 * run orb slam: rosrun ORB_SLAM2 Mono src/ORB_SLAM2/Vocabulary/ORBvoc.txt src/ORB_SLAM2/Examples/Stereo/KITTI03.yaml
=======
 * [orb slam](https://github.com/raulmur/ORB_SLAM2)
   to run orb slam rosrun ORB_SLAM2 Mono src/ORB_SLAM2/Vocabulary/ORBvoc.txt src/ORB_SLAM2/Examples/Stereo/KITTI03.yaml 

 
## Building the Workspace
* navigate to the directory where you downloaded the Github Files 
* `source devel/setup.bash`
* `catkin_make`
