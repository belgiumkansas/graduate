

#include <time.h>
#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <phidget22.h>


#include "opencv2/core.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"


#include "odom_funcs.hpp"

using namespace std;
using namespace cv;

#define MIN_NUM_FEATURES 500
#define FRAME_HEIGHT 480
#define FAME_WIDTH 640





int main(int argc, char** argv){

  if(argc !=2 ){
    cout << "Usage: VideoToUse" << endl;
    return -1;
  }
  // initalize phidget
  PhidgetReturnCode prc;
  PhidgetDigitalInputHandle ch;

  prc = PhidgetDigitalInput_create(&ch);
  if (prc != EPHIDGET_OK) {
  	cout << "Runtime Error -> Creating DigitalInput:" << endl;
  	return 1;
  }

  prc = Phidget_setDeviceSerialNumber((PhidgetHandle)ch, 297689);
  if (prc != EPHIDGET_OK) {
  	//fprintf(stderr, "Runtime Error -> Setting DeviceSerialNumber: \n\t");
  	//fprintf(stderr, "Code: 0x%x\n", error);
  	return 1;
  }
  Phidget_openWaitForAttachment((PhidgetHandle)ch, 5000);
  cout << "phidget attached" << endl;



  double scale = 0.0;

  char text[100];
  int fontFace = FONT_HERSHEY_PLAIN;
  double fontScale = 1;
  int thickness = 1;
  cv::Point textOrg(10, 50);

  Mat R_f, t_f;

  VideoCapture cap(argv[1]);
  cap.set(CV_CAP_PROP_FRAME_WIDTH, FAME_WIDTH);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);

  Mat prev_img, curr_img;
  cap >> prev_img;
  cap >> curr_img;
  cout << curr_img.rows << curr_img.cols << endl;
  Mat grey_prev, grey_curr;
  cvtColor(prev_img, grey_prev, COLOR_BGR2GRAY);
  cvtColor(prev_img, grey_prev, COLOR_BGR2GRAY);

  vector<Point2f> prev_points, curr_points;
  freatureDetect(grey_prev, prev_points);
  //vector<uchar> status;
  //featureTracking(grey_prev, grey_curr, prev_points, curr_points, status);

  vector<KeyPoint> keypoints1;
  int fast_threshold = 20;
  bool nonmaxSuppression = true;
  FAST(grey_curr, keypoints1, fast_threshold, nonmaxSuppression);
  vector<Point2f> points_test ;
  KeyPoint::convert(keypoints1, points_test);
  cout << "point convert" << points_test.size() << endl;

  // some calibration stuff
  double focal = 604;
  cv::Point2d pp(228, 330);
  Mat E, R, t, mask;

  cout << keypoints1[4].pt << endl;
  cout << points_test[4] << endl;
  E = findEssentialMat(points_test, points_test, focal, pp, RANSAC, .99, 3.0, mask);
  //decomposeEssentialMat(E)
  recoverPose(E, points_test, points_test, R, t, focal, pp, mask);

  //cout << "mask array:" << endl << mask << endl;

  /*
  R_f = R.clone();
  t_f = t.clone();

  //move frames and points
  prev_points = curr_points;
  prev_img = curr_img;
  grey_prev = grey_curr;

  namedWindow("source image", WINDOW_AUTOSIZE);
  namedWindow("Trajectory", WINDOW_AUTOSIZE);

  Mat traj = Mat::zeros(600, 600, CV_8UC3);

  char c;
  int64 t0 = cv::getTickCount();


  while(1){
    int64 t1 = cv::getTickCount();
    double secs = (t1-t0)/cv::getTickFrequency();
    cout << 1/secs << endl;
    //cout << t <<endl;
    //cout << E << endl;
    double fps = cap.get(CV_CAP_PROP_FPS);
    cout << fps << endl;
    t0 = t1;
    cout << curr_img.cols << curr_img.rows << endl;

    cap >> curr_img;
    cvtColor(curr_img, grey_curr, COLOR_BGR2GRAY);
    featureTracking(grey_prev, grey_curr, prev_points, curr_points, status);

    E = findEssentialMat(curr_points, prev_points, focal, pp, RANSAC, 0.999, 1.0, mask);
    recoverPose(E, curr_points, prev_points, R, t, focal, pp, mask);

    Mat prevPts(2,prev_points.size(), CV_64F), currPts(2,curr_points.size(), CV_64F);

    //translating the point data types
    for(int i=0; i<prev_points.size(); i++){
      prevPts.at<double>(0,i) = prev_points.at(i).x;
  		prevPts.at<double>(1,i) = prev_points.at(i).y;

  		currPts.at<double>(0,i) = curr_points.at(i).x;
  		currPts.at<double>(1,i) = curr_points.at(i).y;
    }


    t_f = t_f + scale*(R_f*t);
    R_f = R*R_f;


    cout << "num features: " << prev_points.size() << endl;

    // re detect if features get too low
    if(prev_points.size() < MIN_NUM_FEATURES){
      freatureDetection(grey_prev, prev_points);
      featureTracking(grey_prev, grey_curr, prev_points, curr_points, status);
    }

    prev_points = curr_points;
    prev_img = curr_img;
    grey_prev = grey_curr;

    int x = int(t_f.at<double>(0)) + 300;
    int y = int(t_f.at<double>(2)) + 100;
    circle(traj, Point(x, y) ,1, CV_RGB(255,0,0), 2);

    rectangle( traj, Point(10, 30), Point(550, 50), CV_RGB(0,0,0), CV_FILLED);

    sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm",
                  t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));

    putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);


    //drawKeypoints(curr_img, curr_points, curr_img);
    imshow("source image", curr_img);
    imshow("Trajectory", traj);

    c = (char)waitKey(25);
    if(c==27) break;
  }

  cvDestroyWindow("source image");
  cvDestroyWindow("Trajectory");
*/

}
