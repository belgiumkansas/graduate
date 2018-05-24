ROS Driver for USB Video Class Cameras
======================================

`libuvc_camera` is a ROS driver that supports webcams and other UVC-standards-compliant video devices.
It's a cross-platform replacement for `uvc_camera`, a Linux-only webcam driver.

Documentation is available on the ROS wiki: [libuvc_camera](http://wiki.ros.org/libuvc_camera).

You need to checkout the UVC library as a submodule once you clone this repo:

```
git submodule init
git submodule update
```

Also, you'll need to copy the udev rules from libuvc_camera/53-uvc.rules to /etc/udev/rules.d, then run:
```
udevadm control -R 
```
as root to reload the hotplug rules. Your camera should be detected as /dev/video[0-9]

There is a launch file uvccam.launch as a demo that has a set of correct values for the OCam camera