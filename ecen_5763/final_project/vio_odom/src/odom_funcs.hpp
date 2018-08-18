

#include "opencv2/core.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/cuda.hpp"




using namespace cv;
using namespace std;

#define PYRAMID_LEVEL 1

#define FAST_THRESHOLD 20
#define NONMAX_SUPPRESSION true
#define GPU 0


void ftf_tracking(Mat img_prev, Mat img_next,
                     vector<Point2f>& points_prev, vector<Point2f>& points_next,
                     vector<uchar>& status)
{
  vector<float> err;
  Size winSize = Size(5,5);
  TermCriteria termcrit=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS,
                                     30, 0.01);

  calcOpticalFlowPyrLK(img_prev, img_next, points_prev, points_next,
                       status, err, winSize, PYRAMID_LEVEL, termcrit, 0, 0.001);

 //remove errenous points
  int indexOffset = 0;
  for( int i=0; i<status.size(); i++){
    Point2f pt = points_next.at(i- indexOffset);
   	if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0)){
   		points_prev.erase (points_prev.begin() + (i - indexOffset));
   		points_next.erase (points_next.begin() + (i - indexOffset));
   		indexOffset++;
   	}
  }
  if( points_prev.size() != points_next.size()){
    cout << "ftf_tracking mismatch" << endl;
  }
}

void featureDetect(Mat img, vector<Point2f>& points){
  vector<KeyPoint> keypoints;
  #ifdef GPU
    //cuda::FAST_CUDA(img, keypoints, FAST_THRESHOLD, NONMAX_SUPPRESSION);
  #else

  #endif
  FAST(img, keypoints, FAST_THRESHOLD, NONMAX_SUPPRESSION);

  KeyPoint::convert(keypoints, points);
}
