#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <pthread.h>
#include <sys/types.h>

#define IMG_HEIGHT (240)
#define IMG_WIDTH (320)
#define NUM_ROW_THREADS (2)
#define NUM_COL_THREADS (2)
#define IMG_H_SLICE (IMG_HEIGHT/NUM_ROW_THREADS)
#define IMG_W_SLICE (IMG_WIDTH/NUM_COL_THREADS)


pthread_t threads[NUM_ROW_THREADS*NUM_COL_THREADS];

typedef struct _threadArgs
{
    int thread_idx;
    int i;
    int j;
    int h;
    int w;
} threadArgsType;

threadArgsType threadarg[NUM_ROW_THREADS*NUM_COL_THREADS];


using namespace cv; using namespace std;
double alpha=1.0;  int beta=10;  /* contrast and brightness control */

Mat new_image;
Mat image;


void *brightness_thread(void *threadptr){
  threadArgsType thargs=*((threadArgsType *)threadptr);
  int i=thargs.j;
  int j=thargs.i;

  for(i=thargs.i; i<(thargs.i+thargs.h); i++)
  {
      for(j=thargs.j; j<(thargs.j+thargs.w); j++)
      {
        for( int c = 0; c < 3; c++ )
            new_image.at<Vec3b>(i,j)[c] =
            saturate_cast<uchar>( alpha*( image.at<Vec3b>(i,j)[c] ) + beta );
      }
   }
   pthread_exit((void **)0);
}


int main( int argc, char** argv )
{
    image = imread( argv[1] ); // read in image file
    new_image = Mat::zeros( image.size(), image.type() );
    cout<<"* Enter alpha brighten factor [1.0-3.0]: "; cin>>alpha;
    cout<<"* Enter beta contrast increase value [0-100]: "; cin>>beta;
    //IMG_HEIGHT = image.height;
    //IMG_WIDTH = image.width;


    unsigned int thread_idx;
    int  i, j, idx, jdx;
    for(thread_idx=0; thread_idx<(NUM_ROW_THREADS*NUM_COL_THREADS); thread_idx++)
    {

        if(thread_idx == 0) {idx=1; jdx=1;}
        if(thread_idx == 1) {idx=1; jdx=(thread_idx*(IMG_W_SLICE-1));}
        if(thread_idx == 2) {idx=1; jdx=(thread_idx*(IMG_W_SLICE-1));}
        if(thread_idx == 3) {idx=1; jdx=(thread_idx*(IMG_W_SLICE-1));}


        //printf("idx=%d, jdx=%d\n", idx, jdx);

        threadarg[thread_idx].i=idx;
        threadarg[thread_idx].h=IMG_H_SLICE-1;
        threadarg[thread_idx].j=jdx;
        threadarg[thread_idx].w=IMG_W_SLICE-1;

        //printf("create thread_idx=%d\n", thread_idx);
        pthread_create(&threads[thread_idx],NULL, brightness_thread, (void *)&threadarg[thread_idx]);
        cout << "thread made" << endl;
    }

    for(thread_idx=0; thread_idx<(NUM_ROW_THREADS*NUM_COL_THREADS); thread_idx++)
    {
            //printf("join thread_idx=%d\n", thread_idx);
            if((pthread_join(threads[thread_idx], (void **)0)) < 0)
                perror("pthread_join");
    }


    namedWindow("Original Image", 1); namedWindow("New Image", 1);
    imshow("Original Image", image); imshow("New Image", new_image);
    waitKey(); return 0;
}
