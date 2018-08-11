#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <list>

using namespace std;
using namespace cv;

#define NBRIGHTEST 10
#define BUFFER_SIZE 20


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
  int Bbins = 256, Gbins = 256, Rbins = 256;
  int histSize[] = {Bbins, Gbins, Rbins};
  //range bins
  float Branges[] = {0, 256};
  float Granges[] = {0, 256};
  float Rranges[] = {0, 256};
  const float* histRange[] = {Branges, Granges, Rranges};
  //use only the green channel
  int channels[] = {0, 1, 2};
  //histogram Mat
  MatND hist;

  namedWindow("image test", WINDOW_AUTOSIZE);
  // pull in first frame to start buffer and
  // create buffer for frame differencing
  Mat frame;
  list<Mat> frames;
  cap >> frame;
  frames.push_front(frame.clone());

  int nrows = frame.rows;
  int ncols = frame.cols;
  int nchannels = frame.channels();

  // startup info for debugging
  cout << nrows <<" "<< ncols <<" "<< nchannels <<endl;
  cout << frame.type()<< endl;
  cout << frame.step << endl;

  //filtering frames
  Mat median;
  Mat diff;

  //file iter
  int file_iter = 0;

  while(1){
    //bring in frame
    cap >> frame;
    //create frame differenced buffer and frame
    frames.push_front(frame.clone());
    if(frames.size() >= BUFFER_SIZE){
      frames.pop_back();
      diff = frames.front()- frames.back();
    }
    //apply median blur to the differenced frame
    medianBlur(diff, median, 3);

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
    if(thres_bin >= 100) thres_bin = 0;
    //the threshold location
    int threshold = 256 - thres_bin;

    // min and max row and columns
    int min_row = nrows, min_col = ncols;
    int max_row =0 , max_col = 0;
    //iterate through all pixels to find edge of laser
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
    line(frame, Point(0, COM_row), Point(ncols, COM_row), Scalar(0, 255, 0));

    int COM_col = 0;
    if(min_col == ncols && max_col == 0){}
    else COM_col = (min_col + max_col)/2;
    line(frame, Point(COM_col, 0), Point(COM_col, nrows), Scalar(0, 255, 0));


    imshow("image test", frame);
    char c = (char)waitKey(25);
    if(c==27) break;

    //write images
    ostringstream fn;
    fn << "../images/image" << file_iter << ".pgm";
    imwrite(fn.str(), frame);
    file_iter++;
  }

  cap.release();
  destroyAllWindows();
  return 0;
}
