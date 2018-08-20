

#include "opencv2/opencv.hpp"
#include "opencv2/rgbd.hpp"
#include <iostream>
#include <vector>
#include <librealsense2/rs.hpp>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_datatypes.h>

using namespace std;
using namespace cv;

#define FOCUS_LENGTH 525.0
#define CX 319.5
#define CY 239.5

const float MIN_DEPTH = 0.2f;        // in meters
const float MAX_DEPTH = 10.0f;        // in meters
const float MAX_DEPTH_DIFF = 0.08f;  // in meters
const float MAX_POINTS_PART = 0.09f;



//void render_slider(rect location, float& clipping_dist);
//void remove_background(rs2::video_frame& other, const rs2::depth_frame& depth_frame, float depth_scale, float clipping_dist);
float get_depth_scale(rs2::device dev);
rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);
bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev);
Vec3f rotationMatrixToEulerAngles(Mat &R);
bool isRotationMatrix(Mat &R);




int main(int argc, char** argv)try{


  ros::init(argc, argv, "real_sense_vo_node");
  ros::NodeHandle n;
  ros::Publisher vo_data_pub = n.advertise<nav_msgs::Odometry>("realsense_vo", 10);

  rs2::colorizer c;
  rs2::pipeline pipe;
  rs2::pipeline_profile profile = pipe.start();

  //Setup alignment frames
  float depth_scale = get_depth_scale(profile.get_device());
  rs2_stream align_to = find_stream_to_align(profile.get_streams());
  rs2::align align(align_to);


  float depth_clipping_distance = 1.f;
  float vals[] = {FOCUS_LENGTH, 0., CX, 0., FOCUS_LENGTH, CY, 0., 0., 1.};
  const Mat cameraMatrix = Mat(3, 3, CV_32FC1, vals);
  bool isFirst = true;
  Mat rotationMatrix, translationMatrix;


  std::shared_ptr<rgbd::Odometry> odom;
  const auto window_name = "Display Image";
  namedWindow(window_name, WINDOW_AUTOSIZE);
  cout << "get first frame" << endl;
  rs2::frameset frameset = pipe.wait_for_frames();
  cout << "first frame" << endl;
  auto processed = align.process(frameset);

  rs2::video_frame other_frame = processed.first(align_to);
  int w = other_frame.as<rs2::video_frame>().get_width();
  int h = other_frame.as<rs2::video_frame>().get_height();
  Mat image_prev(Size(w, h), CV_8UC3, (void*)other_frame.get_data(), Mat::AUTO_STEP);
  cvtColor(image_prev, image_prev, COLOR_BGR2GRAY);

  rs2::depth_frame aligned_depth_frame = processed.get_depth_frame();
  w = aligned_depth_frame.as<rs2::video_frame>().get_width();
  h = aligned_depth_frame.as<rs2::video_frame>().get_height();
  Mat depth_prev(Size(w, h), CV_16U, (void*)aligned_depth_frame.get_data(), Mat::AUTO_STEP);
  depth_prev.convertTo(depth_prev, CV_32FC1, depth_scale);




  while (waitKey(1) < 0){
    frameset = pipe.wait_for_frames();

    if(profile_changed(pipe.get_active_profile().get_streams(), profile.get_streams())){
      //If the profile was changed, update the align object, and also get the new device's depth scale
      profile = pipe.get_active_profile();
      align_to = find_stream_to_align(profile.get_streams());
      align = rs2::align(align_to);
      depth_scale = get_depth_scale(profile.get_device());
    }

    //Get processed aligned frame
    auto processed = align.process(frameset);

    // caputure frames and other things
    rs2::video_frame other_frame = processed.first(align_to);
    w = other_frame.as<rs2::video_frame>().get_width();
    h = other_frame.as<rs2::video_frame>().get_height();
    Mat image_curr(Size(w, h), CV_8UC3, (void*)other_frame.get_data(), Mat::AUTO_STEP);
    cvtColor(image_curr, image_curr, COLOR_BGR2GRAY);


    rs2::depth_frame aligned_depth_frame = processed.get_depth_frame();
    w = aligned_depth_frame.as<rs2::video_frame>().get_width();
    h = aligned_depth_frame.as<rs2::video_frame>().get_height();
    Mat depth_curr(Size(w, h), CV_16U, (void*)aligned_depth_frame.get_data(), Mat::AUTO_STEP);
    depth_curr.convertTo(depth_curr, CV_32FC1, depth_scale);


    rs2::frame depth = c(aligned_depth_frame);


    //If one of them is unavailable, continue iteration
    if (!aligned_depth_frame || !other_frame) continue;

    Mat rigidTransform;

    if (!odom) {
      vector<int> iterCounts(4);
      iterCounts[0] = 7;
      iterCounts[1] = 7;
      iterCounts[2] = 7;
      iterCounts[3] = 10;

      vector<float> minGradMagnitudes(4);
      minGradMagnitudes[0] = 12;
      minGradMagnitudes[1] = 5;
      minGradMagnitudes[2] = 3;
      minGradMagnitudes[3] = 1;

      odom = std::make_shared<rgbd::RgbdOdometry>(
          cameraMatrix, MIN_DEPTH, MAX_DEPTH, MAX_DEPTH_DIFF, iterCounts,
          minGradMagnitudes, MAX_POINTS_PART,
          rgbd::Odometry::RIGID_BODY_MOTION);

      std::cerr << "Init tracker" << std::endl;
    }

    bool isSuccess = odom->compute(image_prev, depth_prev, Mat(), image_curr,
                                  depth_curr, Mat(), rigidTransform);

    //cout << isSuccess << endl;
    Mat rotationMat = rigidTransform(cv::Rect(0, 0, 3, 3)).clone();
    Mat translateMat = rigidTransform(cv::Rect(3, 0, 1, 3)).clone();
    // If compute successfully, then update rotationMatrix and translationMatrix
    if (isSuccess == true) {
      if (isFirst == true) {
        geometry_msgs::PoseStamped initial;
        initial = *(ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/vrpn_client_node/right_arm/pose", n));
        double q[4];
        q[0] = initial.pose.orientation.x;
        q[1] = initial.pose.orientation.y;
        q[2] = initial.pose.orientation.z;
        q[3] = initial.pose.orientation.w;


        rotationMatrix = rotationMat.clone();

        double x2 = q[0] + q[0];
        double y2 = q[1] + q[1];
        double z2 = q[2] + q[2];
        double w2 = q[3] + q[3];
        double yy2 = q[1] * y2;
        double xy2 = q[0] * y2;
        double xz2 = q[0] * z2;
        double yz2 = q[1] * z2;
        double zz2 = q[2] * z2;
        double wz2 = q[3] * z2;
        double wy2 = q[3] * y2;
        double wx2 = q[3] * x2;
        double xx2 = q[0] * x2;
        rotationMatrix.at<double>(0,0) = - yy2 - zz2 + 1.0f;
        rotationMatrix.at<double>(0,1) = xy2 + wz2;
        rotationMatrix.at<double>(0,2) =  xz2 - wy2;

        rotationMatrix.at<double>(1,0) = xy2 - wz2;
        rotationMatrix.at<double>(1,1) = - xx2 - zz2 + 1.0f;
        rotationMatrix.at<double>(1,2) = yz2 + wx2;

        rotationMatrix.at<double>(2,0) = xz2 + wy2;
        rotationMatrix.at<double>(2,1) = yz2 - wx2;
        rotationMatrix.at<double>(2,2) =  - xx2 - yy2 + 1.0f;


        translationMatrix = translateMat.clone();
        translationMatrix.at<double>(0,0) = initial.pose.position.x;
        translationMatrix.at<double>(1,0) = initial.pose.position.y;
        translationMatrix.at<double>(2,0) = initial.pose.position.z;

        cout << rotationMatrix << endl;
        cout << translationMatrix << endl;
        isFirst = false;
        continue;
      }
      translationMatrix = translationMatrix + (rotationMatrix * translateMat);
      rotationMatrix = rotationMat * rotationMatrix;
    }


    imshow(window_name, image_curr);

    image_prev = image_curr;
    depth_prev = depth_curr;

    //cout << translationMatrix << endl;

    Vec3f euler = rotationMatrixToEulerAngles(rotationMatrix);
    tf::Quaternion q = tf::createQuaternionFromRPY(euler[2], euler[0], euler[1]);
    q.normalize();

    //cout << "roll: " << euler[2] << " pitch: " << euler[0] <<" yaw: " <<euler[1] <<endl;

    nav_msgs::Odometry msg;
    msg.header.frame_id = "RealSense_vo";
    msg.child_frame_id = "RealSense_vo";
    msg.pose.pose.position.x = translationMatrix.at<double>(0);
    msg.pose.pose.position.y = translationMatrix.at<double>(1);
    msg.pose.pose.position.z = translationMatrix.at<double>(2);

    msg.pose.pose.orientation.x = q[0];
    msg.pose.pose.orientation.y = q[1];
    msg.pose.pose.orientation.z = q[2];
    msg.pose.pose.orientation.w = q[3];
    vo_data_pub.publish(msg);


  }
}

catch (const rs2::error & e){
  std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
  return EXIT_FAILURE;
}

catch (const std::exception& e){
  std::cerr << e.what() << std::endl;
  return EXIT_FAILURE;
}


float get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams)
{
    //Given a vector of streams, we try to find a depth stream and another stream to align depth with.
    //We prioritize color streams to make the view look better.
    //If color is not available, we take another stream that (other than depth)
    rs2_stream align_to = RS2_STREAM_ANY;
    bool depth_stream_found = false;
    bool color_stream_found = false;
    for (rs2::stream_profile sp : streams)
    {
        rs2_stream profile_stream = sp.stream_type();
        if (profile_stream != RS2_STREAM_DEPTH)
        {
            if (!color_stream_found)         //Prefer color
                align_to = profile_stream;

            if (profile_stream == RS2_STREAM_COLOR)
            {
                color_stream_found = true;
            }
        }
        else
        {
            depth_stream_found = true;
        }
    }

    if(!depth_stream_found)
        throw std::runtime_error("No Depth stream available");

    if (align_to == RS2_STREAM_ANY)
        throw std::runtime_error("No stream found to align with Depth");

    return align_to;
}




bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev)
{
    for (auto&& sp : prev)
    {
        //If previous profile is in current (maybe just added another)
        auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const rs2::stream_profile& current_sp) { return sp.unique_id() == current_sp.unique_id(); });
        if (itr == std::end(current)) //If it previous stream wasn't found in current
        {
            return true;
        }
    }
    return false;
}


// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(Mat &R)
{
    Mat Rt;
    transpose(R, Rt);
    Mat shouldBeIdentity = Rt * R;
    Mat I = Mat::eye(3,3, shouldBeIdentity.type());

    return  norm(I, shouldBeIdentity) < 1e-6;

}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
Vec3f rotationMatrixToEulerAngles(Mat &R)
{

    //assert(isRotationMatrix(R));

    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return Vec3f(x, y, z);



}
