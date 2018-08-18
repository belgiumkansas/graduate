#include "opencv2/opencv.hpp"
#include <iostream>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include "opencv2/core.hpp"
#include <opencv2/highgui/highgui.hpp>
//#include "opencv2/xfeatures2d.hpp"
//#include "opencv2/xfeatures2d/nonfree.hpp"

using namespace cv;
using namespace std;
using namespace xfeatures2d;



int main(int argc, char** argv){
  if(argc !=2 ){
    cout << "Usage: VideoToUse" << endl;
    return -1;
  }

  VideoCapture cap(argv[1]);
  Mat src;
  cap >> src;

  int frame_width=   cap.get(CV_CAP_PROP_FRAME_WIDTH);
  int frame_height=   cap.get(CV_CAP_PROP_FRAME_HEIGHT);
  VideoWriter video("../videos/bounded.avi",CV_FOURCC('M','J','P','G'),10, Size(frame_width,frame_height),true);



  if(!cap.isOpened()){
    cout << "Error opening video stream or file" << endl;
    return -1;
  }
  Ptr<Tracker> tracker;
  tracker = TrackerMIL::create();
  Rect2d bbox(287, 23, 86, 320);
  bbox = selectROI(src, false);

  rectangle(src, bbox, Scalar( 255, 0, 0 ), 2, 1 );
  imshow("Tracking", src);
  string trackerType = "MIL";
  tracker->init(src, bbox);


  int minHessian = 500;
  Ptr<SURF> detector = SURF::create(minHessian);
  std::vector<KeyPoint> keypoints_1;
  Mat img_keypoints_1;

  Mat out;

  while(1){
    cap >> src;
    bool ok = tracker->update(src, bbox);
    detector->detect(src, keypoints_1);
    out = src.clone();


    if (ok)
        {
            // Tracking success : Draw the tracked object
            rectangle(out, bbox, Scalar( 255, 0, 0 ), 2, 1 );
        }
    else
      {
       // Tracking failure detected.
       putText(out, "Tracking failure detected", Point(100,80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,255),2);
      }
      putText(out, trackerType + " Tracker", Point(100,20), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50),2);



    drawKeypoints(out, keypoints_1, out, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
    //imshow("Keypoints 1", img_keypoints_1 );

    imshow("Tracking", out );
    video.write(src);
    char c = (char)waitKey(25);
    if(c==27) break;
  }
}
