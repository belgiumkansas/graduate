#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"


using namespace cv;
using namespace std;


int main( int argc, char** argv )
{
    cvNamedWindow("Capture Example", CV_WINDOW_AUTOSIZE);
    // I prefere Mat ofver Ipl
    //CvCapture* capture = cvCreateCameraCapture(0);
    //IplImage* frame;
    VideoCapture cap(0);
    Mat frame;

    Mat gray, binary, mfblur;

    cap >> frame;





    while(1)
    {
        cap >> frame;
        //frame=cvQueryFrame(capture);

        //if(!frame) break;
        cvtColor(frame, gray, CV_BGR2GRAY);
        //thresholding done maually with gimp
        threshold(gray, binary, 70, 255, CV_THRESH_BINARY);
        binary = 255 - binary;

        medianBlur(binary, mfblur, 1);

        Mat skel(frame.size(), CV_8UC1, Scalar(0));
        Mat temp;
        Mat eroded;
        Mat element = getStructuringElement(MORPH_CROSS, Size(3, 3));
        bool done;
        int iterations=0;

        do{
          erode(mfblur, eroded, element);
          dilate(eroded, temp, element);
          subtract(mfblur, temp, temp);
          bitwise_or(skel, temp, skel);
          eroded.copyTo(mfblur);

          done = (countNonZero(mfblur) == 0);
          iterations++;
        } while(!done && (iterations < 100));

        cout << "iterations" << iterations << endl;


        imshow("Capture Example", skel);


        char c = cvWaitKey(33);
        if( c == 27 ) break;
    }

    //cvReleaseCapture(&capture);
    cvDestroyWindow("Capture Example");

};
