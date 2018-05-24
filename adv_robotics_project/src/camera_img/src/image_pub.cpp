/*
 * image publisher node for oCam
 */
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <stdio.h>
#include <errno.h>

#include "opencv2/opencv.hpp"
#include "camera_img/withrobot_camera.hpp"	/* withrobot camera API */
#include "camera_img/withrobot_camera.cpp"
#include "camera_img/withrobot_utility.hpp"
#include "camera_img/withrobot_utility.cpp"


/*
 *	Main
 */
int main (int argc, char* argv[])
{
     const char* devPath;
    if(argv[1]!=NULL){
        devPath = "/dev/video1";
    }
    else{
        devPath = "/dev/video0";
    }
	//const char* devPath = "/dev/video0";

    Withrobot::Camera camera(devPath);

    /* USB 3.0 */
    /* 8-bit Greyscale 1280 x 720 60 fps */
    camera.set_format(1280, 720, Withrobot::fourcc_to_pixformat('G','R','E','Y'), 1, 60);

    Withrobot::camera_format camFormat;
    camera.get_current_format(camFormat);

    /*
     * Print infomations
     */
    std::string camName = camera.get_dev_name();
    std::string camSerialNumber = camera.get_serial_number();

    printf("dev: %s, serial number: %s\n", camName.c_str(), camSerialNumber.c_str());
    printf("----------------- Current format informations -----------------\n");
    camFormat.print();
    printf("---------------------------------------------------------------\n");

    int brightness = camera.get_control("Brightness");
    int exposure = camera.get_control("Exposure (Absolute)");

    camera.set_control("Brightness", brightness);
    camera.set_control("Exposure (Absolute)", exposure);

    /*
     * Start streaming
     */
    if (!camera.start()) {
    	perror("Failed to start.");
    	exit(0);
    }

    /*
     * Initialize Node
     */
    ros::init(argc, argv, "image_pub");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_transport::Publisher pub = it.advertise("camera/image_raw", 1);

    /*
     * Main loop
     */
    cv::Mat srcImg(cv::Size(camFormat.width, camFormat.height), CV_8UC1);
    sensor_msgs::ImagePtr msg;
    ros::Rate rate(20);
    while (n.ok()) {
    	/* Copy a single frame(image) from camera(oCam-1MGN). This is a blocking function. */
    	int size = camera.get_frame(srcImg.data, camFormat.image_size, 1);

    	/* If the error occured, restart the camera. */
    	if (size == -1) {
    	    printf("error number: %d\n", errno);
    	    perror("Cannot get image from camera");
    	    camera.stop();
    	    camera.start();
    	    continue;
    	}
        //cv::Mat desImg;
        //cv::cvtColor(srcImg, desImg, CV_GRAY2BGR);
        msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", srcImg).toImageMsg();
        //msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", srcImg).toImageMsg();
        pub.publish(msg);

        ros::spinOnce();
        rate.sleep();

    	
    }

    cv::destroyAllWindows();

    /*
     * Stop streaming
     */
    camera.stop();

	printf("Done.\n");

	return 0;
}
