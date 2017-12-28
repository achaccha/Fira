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

///////////////////////////------------함수-------------/////////////////////////////////////
void marathon();
void find_direction(std::vector<cv::Point2f> approx);
void GetColor(bool is_color, cv::Mat image, int j, int i, int* B, int* G, int* R );
void SetColor(bool is_color, cv::Mat image, int j, int i, int B, int G = 0, int R = 0);
void LineTrace(std::vector<cv::Point2f> approx);
void BinaryColors(cv::Mat &frame, int color[], int num);
bool COLOR(cv::Mat& image, cv::Vec3b color, int y, int x);
void draw_approx(cv::Mat& image, std::vector<cv::Point2f> approx, cv::Scalar color );
void *updateFrame(void* arg);
int GetAngleABC(cv::Point a, cv::Point b, cv::Point c);
std::vector<cv::Point2f> DrawShapeDetection_Control(cv::Mat &m_image, cv::Mat &out_image);
void line_detect(cv::Mat &m_image);

//////////////////////////////////////////////////////////////////////////////////////////////



#endif 
 