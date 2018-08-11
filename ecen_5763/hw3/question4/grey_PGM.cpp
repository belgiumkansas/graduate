#include "opencv2/opencv.hpp"
#include <iostream>
#include <sstream>


using namespace std;
using namespace cv;

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

  Mat src;
  Mat bgr[3];
  int file_iter = 0;

  while(1){
    ostringstream fn;

    fn << "../images/image" << file_iter << ".pgm";

    cap >> src;

    split(src, bgr);
    imshow("Display window", bgr[1]);

    imwrite(fn.str(), bgr[1]);
    char c = (char)waitKey(25);
    if(c==27) break;
    file_iter++;
  }

  cap.release();

  destroyAllWindows();

  return 0;

}
