 #ifndef _VIDEO_PROCESS_
#define _VIDEO_PROCESS_

#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/highgui.h>
#include <iostream>
#include <cmath>
#include <raspicam/raspicam_cv.h>


#define RETURN_CENTER 0
#define RETURN_LOW_Y 1

#define RETURN_DECIDE 2
#define RETURN_ONLY_YELLOW 3
#define RETURN_ONLY_BLUE 4

#define SLEEP 5
#define NO_SLEEP 6




#define PI 3.1415926
#define MIN_RECGONIZE 16

///////////////////////////------------함수-------------/////////////////////////////////////

void obstacle();



std::vector<std::vector<cv::Point2f> > line_detect(cv::Mat &m_image);

int Get_close_y(cv::Mat &m_image, std::vector<cv::Point2f> approx);
int Get_long_side(std::vector<cv::Point2f> approx);


int watch_left_down();
int watch_right_down();
void obstacle_action(cv::Mat &Red_image, cv::Mat &Blue_image, cv::Mat &Yellow_image);
//////////////////////////////////////////////////////////////////////////////////////////////

void g(int color);
void s(int direction);
void f(int color, int direction);
void find_red_gate();
void c(int isIDX);
int get_red_pixel();
void check_left_right(bool Is_recursive);
int watch_forward(int direction);
int measure_distance_down_blue_yellow(int angle, int return_value = RETURN_LOW_Y, int return_only = RETURN_DECIDE, int Is_sleep = SLEEP);
int get_right_approx(cv::Mat m_image, std::vector<cv::Point2f > approx);
int get_left_approx(cv::Mat m_image, std::vector<cv::Point2f > approx);
void watch_right_up();
void watch_left_up();
cv::Point Get_Center_of_approx(cv::Mat &m_image, std::vector<cv::Point2f> approx);
std::vector<cv::Point2f> Draw_Shape_Detection_Control(cv::Mat &m_image, cv::Mat &out_image);
int Get_low_y(cv::Mat &m_image, std::vector<cv::Point2f> approx);
int GetAngleABC(cv::Point a, cv::Point b, cv::Point c);
void single_BinaryColors(cv::Mat &frame, cv::Mat &out_frame, int color[], int num);
void BinaryColors(cv::Mat &frame,  int color[], int num);
bool COLOR(cv::Mat& image, cv::Vec3b color, int y, int x) ;
void SetColor(bool is_color, cv::Mat image, int j, int i, int B, int G, int R);
void GetColor(bool is_color, cv::Mat image, int j, int i, int* B, int* G, int* R );
void *updateFrame(void* arg);
void balance_white(cv::Mat &mat);
void ImShow(std::string str, cv::Mat &image);
void draw_approx(cv::Mat& image, std::vector<cv::Point2f> approx) ;


#endif 
 