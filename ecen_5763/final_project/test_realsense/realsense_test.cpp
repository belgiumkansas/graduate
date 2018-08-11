#include "opencv2/opencv.hpp"
#include <iostream>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include "opencv2/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/xfeatures2d.hpp"
#include <librealsense2/rs.hpp>
//#include "opencv2/xfeatures2d/nonfree.hpp"



int main(int argc, char** argv){
  VideoCapture cap(argv[1]);
  Mat src;
  cap >> src;
/*
  rs2::colorizer color_map;
  rs2::pipeline pipe;
  pipe.start();

  using namespace cv;
  const auto window_name = "Display Image";
  namedWindow(window_name, WINDOW_AUTOSIZE);

  while (waitKey(1) < 0 && cvGetWindowHandle(window_name))
    {
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        rs2::frame depth = color_map(data.get_depth_frame());

        // Query frame size (width and height)
        const int w = depth.as<rs2::video_frame>().get_width();
        const int h = depth.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        Mat image(Size(w, h), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);

        // Update the window with new data
        imshow(window_name, image);
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
*/

}
