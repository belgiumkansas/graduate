/*
capture camera and publish frames
*/
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>

using namespace cv;
using namespace std;

int main(int argc, char **argv){
    // specify camera used.
    if(argv[1] == NULL) return 1;
    
    ros::init(argc, argv, "image_pub");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_transport::Publisher pub = it.advertise("camera/image", 1);
    
    //convert string to device number
    std::istringstream video_source_str(argv[1]);
    int video_source;

    if(!(video_source_str>>video_source)) return 1;

    //get video stream.
    VideoCapture cap(video_source);
    if(!cap.isOpened()) return 1;
    
    Mat frame;
    sensor_msgs::ImagePtr msg;

    ros::Rate rate(20);

    while(n.ok()){
        cap.read(frame);
        if(!frame.empty())
        {
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            pub.publish(msg);
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
