/*
 * subscribe camera fromaes and do edge detection
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;

// set the edge detection parameters
int lowThreshold = 30;
int ratio = 3;
int kernel_size = 3;

/*
 * get images by subscribing to topic,
 * use canny edge detection to process images
 */
void edge_detect(const sensor_msgs::ImageConstPtr& msg){
    try{
        // Mat im = imread("test.jpg", IMREAD_GRAYSCALE);
        Mat im = cv_bridge::toCvShare(msg, "bgr8")->image;

        if(!im.data)
            return;

        //cvtColor(im, im, CV_BGR2GRAY);
    
        blur(im, im, Size(3,3));
        //cvtColor(im, im, CV_GRAY2RGB);

        //Canny(im, im, lowThreshold, lowThreshold*ratio, kernel_size);

        imshow("edges", im);

        waitKey(30);
    }
    catch(cv_bridge::Exception e){
        ROS_ERROR("failed to process image");
    }
    
}

int main(int argc, char **argv){
    ros::init(argc, argv, "edge_detector");
    ros::NodeHandle n;
    namedWindow("edges");
    startWindowThread();

    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub = it.subscribe("camera/image", 1, edge_detect);
    ros::spin();
    destroyWindow("edges");
    return 1;
}

