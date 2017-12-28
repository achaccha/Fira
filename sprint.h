#ifndef _VIDEO_PROCESS_
#define _VIDEO_PROCESS_

#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/highgui.h>
#include <iostream>
#include <cmath>
#include <raspicam/raspicam_cv.h>


#define PI 3.1415926
#define MIN_RECGONIZE 16


void sprint();//ImageFromCamera 복사본


void* ImageFromCamera(void *) ;
void* ImageProcess(void *) ;///////////////////
void BinaryRed(cv::Mat &frame, cv::Mat &Binaryframe);
void BinaryGreen(cv::Mat &frame);
void BinaryBlue(cv::Mat &frame);
void BinaryYellow(cv::Mat &frame);
void PreProcess(cv::Mat &frame);

void *updateFrame(void* arg);


#endif 
