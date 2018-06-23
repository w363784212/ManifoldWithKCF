#include <stdio.h>
#include <setjmp.h>
#include <stdlib.h>
#include <malloc.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <assert.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <getopt.h>
#include <pthread.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <ctype.h>
#include <chrono>
#include <cstdio>
#include "djicam.h"

//opencv include, add in 2018.6.13
#include <cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv.hpp>

#include "kcftracker.hpp"
#include "tracking_utility.hpp"

#define FRAME_SIZE              (1280*720*3/2)  /*format NV12*/
#define BLOCK_MODE                     1

typedef std::chrono::time_point<std::chrono::high_resolution_clock> timer;
typedef std::chrono::duration<float> duration;

using namespace cv;

static unsigned char buffer[FRAME_SIZE+8] = {0};
static unsigned int nframe = 0;
static int mode = 0;

static void print_usage(const char *prog)
{
    printf("Usage: sudo %s [-dgt]\n",prog);
    puts("  -d --display    display vedio stream\n"
         "  -g --getbuffer  get NV12 format buffer\n"
         "  -t --transfer   transfer vedio datas to RC\n"
         "  Note: -d and -g cannot be set at the same time\n");
}

static void parse_opts(int argc, char **argv)
{

    int c;
    static const struct option lopts[] = {
        {"display",   0,0,'d'},
        {"getbuffer", 0,0,'g'},
        {"transfer",  0,0,'t'},
        {NULL,        0,0, 0 },
    };

    while((c = getopt_long(argc, argv, "dgt", lopts, NULL)) != -1)
    {

        switch(c)
        {
            case 'd':
                mode |=  DISPLAY_MODE;
                break;
            case 'g':
                mode |=  GETBUFFER_MODE;
                break;
            case 't':
                mode |=  TRANSFER_MODE;
                break;
            default:
                print_usage(argv[0]);
                exit(0);
        }

    }

}

static void *get_images_loop(void *data)
{
    int ret;
    KCFTracker *tracker=NULL;
    TrackingUtility tu;
    Rect roi(0,0,0,0);
    char message1[100],message2[100];
    const char winName[]="X3 Camera";
    namedWindow(winName,1);
    setMouseCallback(winName,TrackingUtility::mouseCallback,(void*)&tu);

    while(!manifold_cam_exit()) /*Ctrl+c or 'q' to break out*/
    {
        if(mode & GETBUFFER_MODE)
        {
#if BLOCK_MODE
            ret = manifold_cam_read(buffer, &nframe, CAM_BLOCK); /*blocking read*/
            if(ret < 0)
            {
                printf("manifold_cam_read error \n");
                break;
            }

            Mat yuvImg,rgbImg;
            yuvImg.create(720*3/2,1280,CV_8UC1);
            memcpy(yuvImg.data,buffer,FRAME_SIZE*sizeof(unsigned char));
            cvtColor(yuvImg,rgbImg,CV_YUV2RGB_NV12);
            //imshow("X3 Camera",rgbImg);
            char c=waitKey(10);
            tu.getKey(c);
            int dx,dy,yawRate,pitchRate;
            timer trackerStartTime,trackerFinishTime;
            duration trackerTimeDiff;

            switch(tu.getState()){
            case TrackingUtility::STATE_IDLE:
                roi=tu.getROI();
                sprintf(message2,"please select ROI and press g");
                break;

            case TrackingUtility::STATE_INIT:
                sprintf(message2,"g pressed,initialize tracker");
                roi=tu.getROI();
                tracker=new KCFTracker(true,true,false,false);
                tracker->init(roi,rgbImg);
                tu.startTracker();
                break;

            case TrackingUtility::STATE_ONGOING:
                trackerStartTime =std::chrono::high_resolution_clock::now();
                roi = tracker->update(rgbImg);
                trackerFinishTime = std::chrono::high_resolution_clock::now();
                trackerTimeDiff=trackerFinishTime-trackerStartTime;
                sprintf(message2,"Tracking:bounding box update time = %.2f ms\0",trackerTimeDiff.count()*1000.0);
                dx = (int)(roi.x+roi.width/2-640);
                dy = (int)(roi.y+roi.height/2-360);
                yawRate=dx;
                pitchRate=-dy;
                if(abs(dx)<10) yawRate=0;
                if(abs(dy)<10) pitchRate=0;
                break;

            case TrackingUtility::STATE_STOP:
                sprintf(message2,"s pressed, stop tracker");
                delete tracker;
                tracker=NULL;
                tu.stopTracker();
                roi=tu.getROI();
                break;

            default:
                break;

            }

            dx=roi.x+roi.width/2-640;
            dy=roi.y+roi.height/2-360;
            circle(rgbImg,Point(640,360),5,Scalar(255,0,0),2,8);
            if(roi.width!=0){
                circle(rgbImg,Point(roi.x+roi.width/2,roi.y+roi.height/2),3,Scalar(0,0,255),1,8);
                line(rgbImg,Point(640,360),Point(roi.x+roi.width/2,roi.y+roi.height/2),Scalar(0,255,255));
            }
            rectangle(rgbImg,roi,Scalar(0,255,0),1,8,0);
            cvtColor(rgbImg,rgbImg,COLOR_RGB2BGR);
            sprintf(message1,"dx=%04d, dy=%04d",dx,dy);
            putText(rgbImg,message1,Point2f(20,30),FONT_HERSHEY_SIMPLEX,1,Scalar(0,0,255));
            putText(rgbImg,message2,Point2f(20,60),FONT_HERSHEY_SIMPLEX,1,Scalar(0,0,255));
            imshow(winName,rgbImg);

#else
            ret = manifold_cam_read(buffer, &nframe, CAM_NON_BLOCK); /*non_blocking read*/
            if(ret < 0)
            {
                printf("manifold_cam_read error \n");
                break;
            }
#endif
        }

        usleep(1000);
    }

    if(tracker !=NULL){
        delete tracker;
        tracker=NULL;
    }

    printf("get_images_loop thread exit! \n");

}

int main(int argc, char **argv)
{
    int ret;
    pthread_attr_t attr;
    struct sched_param schedparam;
    pthread_t read_thread;

    if(0 != geteuid())
    {
        printf("Please run ./test as root!\n");
        print_usage(argv[0]);
        return -1;
    }
    parse_opts(argc, argv); /*get parameters*/
    if(0 == mode || 3 == mode || 7 == mode) /*invalid mode*/
    {
        print_usage(argv[0]);
        return -1;
    }
    ret = manifold_cam_init(mode);
    if(-1 == ret)
    {
        printf("manifold init error \n");
        return -1;
    }

    /*
     * if the cpu usage is high, the scheduling policy of the read thread
     * is recommended setting to FIFO, and also, the priority of the thread should be high enough.
     */
    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy((pthread_attr_t *)&attr, SCHED_FIFO);
    schedparam.sched_priority = 90;
    pthread_attr_setschedparam(&attr,&schedparam);
    pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM);

    if (pthread_create(&read_thread, &attr, get_images_loop, NULL) != 0)
    {
        perror ("usbRead_thread create");
        assert(0);
    }

    if(pthread_attr_destroy(&attr) != 0)
    {
        perror("pthread_attr_destroy error");
    }

    pthread_join(read_thread, NULL);/*wait for read_thread exit*/

    while (!manifold_cam_exit())
    {
        sleep(1);
    }

    return 0;
}
