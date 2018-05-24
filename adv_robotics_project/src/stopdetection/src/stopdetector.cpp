/*
 * subscribe camera fromaes and detect stop sign
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include "stopdetection/STOP_SIGNAL.h"
using namespace cv;
using namespace std;

// set the edge detection parameters
int lowThreshold = 30;
int ratio = 3;
int kernel_size = 3;

// stop signal publisher
ros::Publisher stop_signal_pub;

// when camera capture a stop scalar_sign_op
ros::Time capture_time;

/*
 * get images by subscribing to topic,
 * use canny edge detection to process images
 */
void stop_sign_detect(const sensor_msgs::ImageConstPtr& msg){
    try{
        // Mat im = imread("test.jpg", IMREAD_GRAYSCALE);
        Mat im = cv_bridge::toCvShare(msg, "mono8")->image;
        Mat des(im.size(), CV_8UC1, Scalar(0, 0, 0));
        vector<vector<Point> > contours;
        vector<Point> approx;

        if(!im.data)
            return;

        blur(im, im, Size(3,3));
        Canny(im, im, lowThreshold, lowThreshold*ratio, kernel_size);
        findContours(im.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
        
        for(int i=0;i<contours.size();i++){
            approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);
            // filter out the small objects
            if(fabs(contourArea(contours[i]))<1000 || !isContourConvex(approx))
                continue;
            
            // look for octagon
            if(approx.size()==8){
                drawContours(des, contours, i, Scalar(255, 255, 255), CV_FILLED);
                cout<<"octagon size"<<endl;
                cout<<arcLength(contours[i], true)<<endl;

                // if we are closed to the stop sign enough, scalar_tan_op
                ros::Time current = ros::Time::now();
                
                if(arcLength(contours[i], true)>850.0&&current.sec-capture_time.sec>=1.0){
                    capture_time = ros::Time::now();
                    // publish stop signal
                    stopdetection::STOP_SIGNAL stop;
                    stop.STOP = true;
                    stop_signal_pub.publish(stop);
                }
            }
        }

        imshow("stop_sign", des);

        waitKey(30);
    }
    catch(cv_bridge::Exception e){
        ROS_ERROR("failed to process image");
    }
    
}

int main(int argc, char **argv){
    ros::init(argc, argv, "stop_sign_detector");
    ros::NodeHandle n;
    namedWindow("stop_sign");
    startWindowThread();

    // subscribe the image
    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub = it.subscribe("camera/image", 1, stop_sign_detect);
    
    // publish stop signal
    stop_signal_pub = n.advertise<stopdetection::STOP_SIGNAL>("stop_signal", 100);
    
    ros::spin();
    destroyWindow("stop_sign");
    return 1;
}

