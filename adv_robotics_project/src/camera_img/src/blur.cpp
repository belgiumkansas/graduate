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

class Image_Blur{
private:
    ros::NodeHandle n;
    image_transport::Publisher pub;
    image_transport::Subscriber sub;
public:
    Image_Blur(){
        image_transport::ImageTransport it(n);
        pub = it.advertise("camera/image_blur", 100);        
        sub = it.subscribe("/camera/image_rect", 1, &Image_Blur::image_blur, this);
    }
    void image_blur(const sensor_msgs::ImageConstPtr& msg){
        try{
            Mat im = cv_bridge::toCvShare(msg, "mono8")->image;
            // do gaussian blur
            blur(im, im, Size(3,3));
            // publish image
            sensor_msgs::ImagePtr msg;
            msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", im).toImageMsg();
            pub.publish(msg);
        }
        catch(cv_bridge::Exception e){
            ROS_ERROR("failed to process image");
        }
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "image_blur");

    Image_Blur blur_obj;
    ros::spin();
    return 1;
}