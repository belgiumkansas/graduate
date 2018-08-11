#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

int main(int argc, char** argv){
  if(argc != 2){
    cout << "Usage: display_image ImageToLoadAndDisplay" << endl;
    return -1;
  }

  Mat src;
  Mat bgr[3];
  Mat dst;
  src = imread(argv[1], CV_LOAD_IMAGE_COLOR);
  split(src, bgr);

  if(! src.data){
    cout << "Could not open or find the image" << endl;
    return -1;
  }

  medianBlur(bgr[1], dst, 3);

  namedWindow( "Display window", WINDOW_AUTOSIZE);
  imshow("Display window", dst);
  imwrite("median_grey.jpg", bgr[1]);


  waitKey(0);
  return 0;
}
