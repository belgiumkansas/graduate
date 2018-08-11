/*
 *
 *  Example by Sam Siewert
 *
 */
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sys/time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;


static double readTOD(void){
  /* takes gettimeofday and turns into single double
  in order to negate rolloever issue*/
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (tv.tv_sec*1000000 + tv.tv_usec);
}


double worst_frame = 100; // fake high rate to be subbed in time_func
double average_rate = 26; //average frame rate based on previous tests

static void time_func(void){
  static double counter = 0;
  static double TOD = 0;
  static double TOD_prev = 0;
  static double prev_interval = 0;
  static double interval = 0;
  static double frame_rate = 0;
  static double prev_frame_rate = 0;
  static double jitter;

  counter ++;
  TOD_prev = TOD;
  TOD = readTOD();
  interval = (TOD - TOD_prev)/1000000;
  jitter = interval - (1/average_rate);
  frame_rate = 1/interval;

  if(counter > 1){
    if(frame_rate < worst_frame) worst_frame = frame_rate;
    average_rate = (average_rate*(counter -1))/counter + (frame_rate/counter);
  }

  printf("average frame rate: %f\n", average_rate);
  printf("worst frame rate: %f\n", worst_frame);
  printf("jitter : %f seconds\n\n", jitter);
}

int main( int argc, char** argv )
{
    cvNamedWindow("Capture Example", CV_WINDOW_AUTOSIZE);
    CvCapture* capture = cvCreateCameraCapture(0);
    IplImage* frame;

    while(1)
    {
        frame=cvQueryFrame(capture);
        time_func();

        if(!frame) break;

        cvShowImage("Capture Example", frame);

        char c = cvWaitKey(33);
        if( c == 27 ) break;
    }

    cvReleaseCapture(&capture);
    cvDestroyWindow("Capture Example");

};
