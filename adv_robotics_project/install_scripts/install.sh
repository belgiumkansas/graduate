#!/bin/sh
echo Starting_Install_script
: $(sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list')
echo added_source.list
: $(sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116)
echo added_ros_keys
: $(apt-get update)
echo update_done
apt-get install ros-kinetic-phidgets-drivers
echo phidget_driver_installed

chmod +x /opt/ros/kinetic/share/phidgets_api/setup-udev.sh
/opt/ros/kinetic/share/phidgets_api/setup-udev.sh
chmod -x /opt/ros/kinetic/share/phidgets_api/setup-udev.sh
