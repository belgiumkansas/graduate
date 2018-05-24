/*
 subscribe camera frams and do blob detection for rolling ball challenge
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "blobdetection/BALL.h"

using namespace cv;
using namespace std;

SimpleBlobDetector::Params params;
Ptr<SimpleBlobDetector> detector;
KeyPoint last_pos;

ros::Publisher ball_pub;

bool init_params(Mat im)
{
    //set blob detector parameter
    //set threshold
    params.minThreshold = 0;
    params.maxThreshold = 250;

    //Filter by area
    params.filterByArea = true;
    params.minArea = im.cols*im.rows/100;
    params.maxArea = im.cols*im.rows/2;

    //filter by circularity
    //params.filterByCircularity = true;
    params.minCircularity = 0.8;

    //filter by convexity
    //params.filterByCoggnvexity = true;
    params.minConvexity = 0.8;

    //filter by Inertia
    params.filterByInertia = true;
    params.minInertiaRatio = 0.01;
}

void blob_detect(const sensor_msgs::ImageConstPtr& msg){
    
    try{
        //convert image
        Mat im;
        im = cv_bridge::toCvShare(msg, "mono8")->image;
	    //cvtColor(im, im_hsv, CV_RGB2HSV);
        static bool init_FLAG = init_params(im);

        if(im.empty()){
            ROS_ERROR("failed to convert image");
            return;
        }
        vector<Vec3f> circles;
        GaussianBlur( im, im, Size(9, 9), 2, 2 );

        HoughCircles(im, circles, CV_HOUGH_GRADIENT, 480.0/640.0, im.rows, 100, 30, 100, 200);
        for( size_t i = 0; i < circles.size(); i++ )
        {
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            // circle center
            circle( im, center, 3, Scalar(0,255,0), -1, 8, 0 );
            // circle outline
            circle( im, center, radius, Scalar(0,0,255), 3, 8, 0 );
            blobdetection::BALL ball;
            ball.x = circles[i][0];
            ball.r = radius;
            ball_pub.publish(ball);
        }
        //imshow("video", im);
        waitKey(30);
        /*
        //detect blobs
        vector<KeyPoint> keypoints;
        vector<KeyPoint> keypoints2;

        //detect dark blobs
        params.blobColor = 0;
        detector = SimpleBlobDetector::create(params);
        detector->detect(im, keypoints);

        //detect light blobs
        params.blobColor = 255;
        detector = SimpleBlobDetector::create(params);
        detector->detect(im, keypoints2);

        //keypoints.insert(keypoints.begin(), keypoints2.begin(), keypoints2.end());

        cout<<"last_pos x:"<<last_pos.pt.x<<endl;

        if(keypoints.empty()){
            cout<<"didn't detect ball"<<endl;
        }
        else{
            if(last_pos.pt.x==0){
                last_pos = keypoints.at(0);
                cout<<"updated pose"<<last_pos.pt.x<<","<<last_pos.pt.y<<endl;
            }
            else{
                //compare current ball position with the position we saw last time
                if(keypoints.at(0).pt.x>last_pos.pt.x){
                    cout<<"ball going left"<<endl;
                }
                else{
                    cout<<"ball going right"<<endl;
                }
                last_pos = keypoints.at(0);
            }
        }
        

        Mat im_with_keypoints;
        try{
            drawKeypoints(im, keypoints, im_with_keypoints, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        }
        catch(Exception e){
            ROS_ERROR("%s", e.msg);
        }

        imshow("video", im_with_keypoints);
        //must call waitKey so that the window can be updated
        waitKey(30);
        */
    }
    catch(cv_bridge::Exception& e){
        ROS_ERROR("failed to convert image");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "blob_detector");
    ros::NodeHandle n;
    //namedWindow("video");
    //startWindowThread();

    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub = it.subscribe("camera/image_raw", 1, blob_detect);
    ball_pub = n.advertise<blobdetection::BALL>("ball", 100);
    ros::spin();
    //destroyWindow("video");
}
