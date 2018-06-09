#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>


#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/contrib/contrib.hpp"

using namespace cv;
using namespace std;

// Default resolution is 360p
#define VRES_ROWS (360)
#define HRES_COLS (640)


#define ESC_KEY (27)

// Buffer for highest resolution visualization possible
unsigned char imagebuffer[1440*2560*3]; // 1440 rows, 2560 cols/row, 3 channel

int main(int argc, char **argv)
{
    int hres = HRES_COLS;
    int vres = VRES_ROWS;
    Mat basicimage(vres, hres, CV_8UC3, imagebuffer);
    int frameCnt=0;

    printf("hres=%d, vres=%d\n", hres, vres);

    // interactive computer vision loop
    namedWindow("Solution Visualization", CV_WINDOW_AUTOSIZE);

    basicimage = imread("Cactus360p.jpg", CV_LOAD_IMAGE_COLOR);

    if(!basicimage.data)  // Check for invalid input
    {
        printf("Could not open or find the refernece starting image\n");
        exit(-1);
    }

    // create solution with mat array
    resize(basicimage, basicimage, Size(320, 180));
    Mat output;
    int border = 4;
    copyMakeBorder(basicimage, output, border, border, border, border, BORDER_CONSTANT, Scalar(0, 255, 255));
    line(output, Point(160, 0), Point(160, 180), Scalar(0, 255, 255));
    line(output, Point(0, 90), Point(320 ,90), Scalar(0, 255, 255));
    imwrite("mat_cactus.jpg", output);

    // view loop mat image
    while(1)
    {
        imshow("Solution Visualization", output);

        // set pace on shared memory sampling in msecs
        char c = cvWaitKey(10);

        if(c == ESC_KEY)
        {
            exit(1);
        }
    }

}
