#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/video.hpp>
#include <opencv2/bgsegm.hpp>


using namespace cv;
using namespace std;

Ptr<BackgroundSubtractor> pMOG;
Mat fgMaskMOG;

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

    pMOG = bgsegm::createBackgroundSubtractorMOG();




    while(1)
    {
        Mat test_mask;

        cap >> frame;
        //frame=cvQueryFrame(capture);

        //if(!frame) break;
        cvtColor(frame, gray, CV_BGR2GRAY);
        //thresholding done maually with gimp

        pMOG->apply(frame, fgMaskMOG);

        frame.copyTo(test_mask, fgMaskMOG);

        imshow("Capture Example", test_mask);


        char c = cvWaitKey(33);
        if( c == 27 ) break;
    }

    //cvReleaseCapture(&capture);
    cvDestroyWindow("Capture Example");

};
