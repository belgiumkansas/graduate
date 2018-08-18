

#include "opencv2/opencv.hpp"
#include "opencv2/rgbd.hpp"
#include <iostream>
#include <vector>
#include <librealsense2/rs.hpp>

using namespace std;


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


int main(int argc, char** argv)try{

  rs2::colorizer c;


  rs2::pipeline pipe;
  rs2::pipeline_profile profile = pipe.start();

  //scale of depth
  float depth_scale = get_depth_scale(profile.get_device());

  rs2_stream align_to = find_stream_to_align(profile.get_streams());

  rs2::align align(align_to);

  float depth_clipping_distance = 1.f;

  using namespace cv;

  float vals[] = {FOCUS_LENGTH, 0., CX, 0., FOCUS_LENGTH, CY, 0., 0., 1.};
  const Mat cameraMatrix = Mat(3, 3, CV_32FC1, vals);
  bool isFirst = true;
  Mat rotationMatrix, tranlslationMatrix;


  std::shared_ptr<rgbd::Odometry> odom;
  const auto window_name = "Display Image";
  namedWindow(window_name, WINDOW_AUTOSIZE);

  rs2::frameset frameset = pipe.wait_for_frames();
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




  while (waitKey(1) < 0 && cvGetWindowHandle(window_name)){
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

    // Trying to get both other and aligned depth frames
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
    // If compute successfully, then update rotationMatrix and tranlslationMatrix
    if (isSuccess == true) {
      if (isFirst == true) {
        rotationMatrix = rotationMat.clone();
        tranlslationMatrix = translateMat.clone();
        isFirst = false;
        continue;
      }
      tranlslationMatrix = tranlslationMatrix + (rotationMatrix * translateMat);
      rotationMatrix = rotationMat * rotationMatrix;
    }


    imshow(window_name, image_curr);

    image_prev = image_curr;
    depth_prev = depth_curr;

    cout << tranlslationMatrix << endl;
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
