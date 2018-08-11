// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

//#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/core.hpp>   // Include OpenCV API
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

int main(int argc, char * argv[])
{
  VideoCapture cap(argv[1]);
  Mat src;
  cap >> src;
/*
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start();*/


}
