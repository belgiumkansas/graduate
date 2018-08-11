#include "opencv2/opencv.hpp"
#include <iostream>
#include <list>

using namespace std;
using namespace cv;

// frame buffer size
#define BUFFER_SIZE 10

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

  Mat frame;
  Mat diff;
  //creat list of mats to build the buffer
  list<Mat> frames;

  int file_iter = 0;
  while(1){

    ostringstream fn;
    fn << "../images/image" << file_iter << ".pgm";

    cap >> frame;
    //push frame onto list for the buffer
    frames.push_front(frame.clone());
    if(frames.size() >= BUFFER_SIZE){
      frames.pop_back();
      // subtract newest frame from oldest one
      // to remove background
      diff = frames.back() - frames.front();
      imshow("frame", diff);
      char c = (char)waitKey(25);
      if(c==27) break;

      imwrite(fn.str(), diff);
      file_iter ++;
    }
  }

  cap.release();
  destroyAllWindows();
  return 0;
}
