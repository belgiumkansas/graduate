#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <list>

using namespace std;
using namespace cv;

#define NBRIGHTEST 20

int main(int argc, char** argv){
  if(argc !=2 ){
    cout << "Usage: VideoToUse" << endl;
    return -1;
  }

  VideoCapture cap(argv[1]);

  if(!cap.isOpened()){
    cout << "Error opening video stream or file" << endl;
    return -1;
  }


  //setup histogram bins
  int G1bins = 256, G2bins = 100, G3bins = 30;
  int histSize[] = {G1bins, G2bins, G3bins};
  //range bins
  float G1ranges[] = {0, 256};
  float G2ranges[] = {0, 256};
  float G3ranges[] = {0, 256};
  const float* histRange[] = {G1ranges, G2ranges, G3ranges};
  //only use first channel but they are all the same
  int channels[] = {0};
  //histogram Mat
  MatND hist;

  namedWindow("image test", WINDOW_AUTOSIZE);
  //Pull in first frame for Mat values
  Mat frame;
  cap >> frame;

  int nrows = frame.rows;
  int ncols = frame.cols;
  int nchannels = frame.channels();

  // startup info for debugging
  cout << nrows <<" "<< ncols <<" "<< nchannels <<endl;
  cout << frame.type()<< endl;
  cout << frame.step << endl;

  //filtering frames
  Mat median;
  //file name iter
  int file_iter = 0;

  while(1){
    //bring in frame
    cap >> frame;

    //apply median blur to the differenced frame
    medianBlur(frame, median, 3);

    //do histogram calculations
    calcHist(&median, 1, channels, Mat(),
             hist, 1, histSize, histRange,
             true,
             false);

    // threshold picking function
    //minimum of 0 pixels summed from brightest
    //to get the threshold function
    int bin_sum = 0;
    int thres_bin = 0;
    while(bin_sum <= NBRIGHTEST){
      bin_sum += hist.at<uint16_t>(256-thres_bin);
      thres_bin ++;
    }
    //catch statement of lazer pointer disapeared
    if(thres_bin >= 50) thres_bin = 0;
    //the threshold location
    int threshold = 256 - thres_bin;

    // min and max row and columns
    int min_row = nrows, min_col = ncols;
    int max_row =0 , max_col = 0;
    //Iterate through all pixels to find edge of laser
    for( int row=0; row<median.rows; row++){
      for (int col=0; col<median.cols; col++){
        int pix = median.at<Vec3b>(row, col)[0];
        if(pix >= threshold){
          if(row <= min_row) min_row = row;
          if(row >= max_row) max_row = row;
          if(col <= min_col) min_col = col;
          if(col >= max_col) max_col = col;
        }
      }
    }

    // draw lines
    int COM_row = 0;
    if(min_row == nrows && max_row == 0){}
    else COM_row = (min_row + max_row)/2;
    line(frame, Point(0, COM_row), Point(ncols, COM_row), Scalar(255, 255, 255));

    int COM_col = 0;
    if(min_col == ncols && max_col == 0){}
    else COM_col = (min_col + max_col)/2;
    line(frame, Point(COM_col, 0), Point(COM_col, nrows), Scalar(255, 255, 255));


    imshow("image test", frame);
    char c = (char)waitKey(25);
    if(c==27) break;

    //record
    ostringstream fn;
    fn << "../images/image" << file_iter << ".pgm";
    imwrite(fn.str(), frame);
    file_iter++;

  }

  cap.release();
  destroyAllWindows();
  return 0;
}
