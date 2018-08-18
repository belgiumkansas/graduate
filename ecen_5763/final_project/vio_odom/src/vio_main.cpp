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


#include "MadgwickAHRS.h"
#include "inertial_handler.h"
#include "odom_funcs.hpp"

using namespace std;
using namespace cv;

extern double scale;

#define MIN_NUM_FEATURES 500
#define FRAME_HEIGHT 480
#define FAME_WIDTH 640

void pp_capture(VideoCapture &cap_dev, Mat &img){
  // pre processed images from a capture currently just grey scaling
  cap_dev >> img;
  cvtColor(img, img, COLOR_BGR2GRAY);


}


int main(int argc, char** argv){

  if(argc !=2 ){
    cout << "Usage: VideoToUse" << endl;
    return -1;
  }
  //initalize camera
  VideoCapture cap(argv[1]);
  cap.set(CV_CAP_PROP_FRAME_WIDTH, FAME_WIDTH);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
  Mat prev_img, curr_img;
  pp_capture(cap, prev_img);
  pp_capture(cap, curr_img);
  cout << "rows: "<< curr_img.rows << "cols: " << curr_img.cols << endl;

  // initialize phidget controller
  PhidgetSpatialHandle spatial;
 	PhidgetSpatial_create(&spatial);
	Phidget_setDeviceSerialNumber((PhidgetHandle)spatial, 297689);
	Phidget_setOnAttachHandler((PhidgetHandle)spatial, onAttachHandler, NULL);
	Phidget_openWaitForAttachment((PhidgetHandle)spatial, 5000);
	Phidget_setDataInterval((PhidgetHandle)spatial, 4);
	PhidgetSpatial_setOnSpatialDataHandler( spatial, onSpatialData, NULL);
  PhidgetSpatial_zeroGyro(spatial);  //keep gyro steady for 2 seconds

  //camera model parameters TODO load from config file
  double focal = 604;         //lens focal length in pixels
  Point2d pp(228, 330);  // principal point of camera

  //3D transformation matrixes
  Mat E, R;       // (E)sential matrix, (R)otation matrix
  Mat t, mask;    // (t)ransformation (linear), point mask
  Mat t_f, R_f;   // transformation_final, Rotation_final


  //initalize visual odometry
  vector<Point2f>points_prev, points_curr; //2floats for feature location
  vector<uchar> status; //status array
  featureDetect(prev_img, points_prev);
  ftf_tracking(prev_img, prev_img, points_prev, points_curr, status);
  //first essential matrix
  E = findEssentialMat(points_prev, points_curr,
                       focal, pp, RANSAC, .99, 1.0, mask);
  //recover pose using essential matrix
  recoverPose(E, points_prev, points_curr,
              R, t, focal, pp, mask);

  //Scalling based of distance travelled
  //initialize with arbitray small scale
  //due to no movement
  scale = 0.001;
  //TODO scalling method.

  R_f = R.clone();
  t_f = t.clone();


  cout << "intial position" << endl << E << endl;

  //initalize frame rate counter
  int64 t_start, t_end;
  double hz;
  t_start = getTickCount();

  while(1){
    t_end = getTickCount();
    hz = getTickFrequency()/(t_end - t_start);
    t_start = t_end;
    cout << "frame hz: " << hz << endl;
    prev_img = curr_img;
    pp_capture(cap, curr_img);
    ftf_tracking(prev_img, curr_img, points_prev, points_curr, status);

    //re-track features if points are not available
    if(points_curr.size() < MIN_NUM_FEATURES){
      cout << "feature count low: " << points_curr.size() << " retrack" << endl;
      featureDetect(prev_img, points_prev);
      ftf_tracking(prev_img, curr_img, points_prev, points_curr, status);
    }

    E = findEssentialMat(points_prev, points_curr,
                         focal, pp, RANSAC, .99, 3.0, mask);
    recoverPose(E, points_prev, points_curr,
                R, t, focal, pp, mask);

    //create class for scale

    t_f = t_f + 1*(R_f*t);
    R_f = R*R_f;

    /*translating the point data types
    for(int i=0; i<points_prev.size(); i++){
      points_prev.at<double>(0,i) = points_prev.at(i).x;
  		points_prev.at<double>(1,i) = points_prev.at(i).y;

  		points_prev.at<double>(0,i) = points_curr.at(i).x;
  		points_prev.at<double>(1,i) = points_curr.at(i).y;
    }*/


    cout << t_f  << endl;
  }

}
