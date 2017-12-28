#include "marathon.h"
#include "RobotProtocol.h"
 
#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <string>
 
 
 
///////////////////////////------------함수-------------/////////////////////////////////////
//void find_direction(std::vector<cv::Point2f> approx);
//void GetColor(bool is_color, cv::Mat image, int j, int i, int* B, int* G , int* R );
//void SetColor(bool is_color, cv::Mat&  image, int j, int i, int B, int G = 0, int R = 0);
//void LineTrace(cv::Mat& image);
//void BinaryColors(cv::Mat &frame, int color[], int num);
//bool COLOR(cv::Mat& image, cv::Vec3b color, int y, int x);
//void draw_approx(cv::Mat& image, std::vector<cv::Point2f> approx);
//void *updateFrame(void* arg);
//int GetAngleABC(cv::Point a, cv::Point b, cv::Point c)
//std::vector<cv::Point2f> DrawShapeDetection_Control(cv::Mat &m_image, cv::Mat &out_image);
//////////////////////////////////////////////////////////////////////////////////////////////
 
 
//MODE
#define L_NO_LINE 0
#define L_GOOD_LINE 1
#define L_CUT_LINE 2
#define L_RIGHT_LINE 3
#define L_LEFT_LINE 4
#define L_WAIT 10
#define L_SLOW 11
 
#define D_FIND 100
#define D_FORWARD 101
#define D_LEFT 102
#define D_RIGHT 103
#define D_END 104
 
//동작
#define M_base 99
#define M_head_down 250 
#define M_walk_forward 141//140
#define M_walk_left 138
#define M_walk_right 139
#define M_walk_slow 251
#define M_hoke_left 252
#define M_hoke_right 253
#define M_turn_left 136
#define M_turn_right 137
#define M_turn_head_left 151
#define M_turn_head_right 155
 
#define M_turn_head_forward 158
 
 
int NEW = 0;
int MODE = L_GOOD_LINE;
 
cv::Vec3b V_RED = cv::Vec3b(0, 0, 255);
cv::Vec3b V_BLUE = cv::Vec3b(255, 0, 0);
cv::Vec3b V_GREEN = cv::Vec3b(0, 255, 0);
cv::Vec3b V_YELLOW = cv::Vec3b(0, 255, 255);
cv::Vec3b V_PURPLE = cv::Vec3b(255, 0, 255);
cv::Vec3b V_WHITE = cv::Vec3b(255, 255, 255);
cv::Vec3b V_BLACK = cv::Vec3b(0, 0, 0);
 
 
cv::Mat Frame, current_Frame;
cv::Mat Binaryframe, small_Binaryframe;
cv::Mat squareframe;
 
int NINE = 0;
// 영상 16배 축소
int divide = 4;
int ScreenX = 640 / divide;
int ScreenY = 480 / divide;
int right = 0, left = 0;
int redposition;
bool Right = true;
int nored = 0;
int start_first = 0;
 
int thresh = 200;
int max_thresh = 255;
 
int one, two, three;
 
int UP = 80, DOWN = 150;
 
/////////////////////////////////////////////////
pthread_mutex_t frameLocker; //뮤텍스 잠금
pthread_t updateThread; //thread ID는 updateThread
cv::VideoCapture capture;
 
 
void cut_bin(cv::Mat& image) {
	int up = UP, down = DOWN;
 
	for (int y = 0; y < image.rows; y++) {
		if (y == up) y = down;
 
		for (int x = 0; x < image.cols; x++) {
			if ((y > up - 3 && y < up + 3) || (y > down - 3 && y < down + 3)) {
				SetColor(1, image, y, x, 0, 0, 0);
			}
			else if (COLOR(image, V_GREEN, y, x)) { // green
				SetColor(1, image, y, x, 255, 0, 0);
			}
		}
	}
}
 
//Motion(216);
//usleep(1800000);
 
void marathon()
{
	usleep(1000000);
		
	
	Motion(198);
	usleep(1000000);
	int c_black[] = { 6 }, c_line[] = { 3 };
 
	capture.open(0);
	cv::Mat image;
	std::vector<cv::Point2f> approx_temp;
 
	Motion(135);
	Motion(135);
	Motion(135);
	Motion(135);
	Motion(135);
	Motion(135);
	Motion(135);
usleep(100000);
	Motion(M_turn_head_forward);
Motion(M_turn_head_forward);
Motion(M_turn_head_forward);
Motion(M_turn_head_forward);
 
Motion(M_turn_head_forward);
Motion(M_turn_head_forward);
 
 
	pthread_mutex_init(&frameLocker, NULL);
	pthread_create(&updateThread, NULL, updateFrame, NULL);
 
	if (!capture.isOpened()) {
		std::cerr << "Could not open camera" << std::endl;
		exit(1);
	}
Motion(M_turn_head_forward);Motion(M_turn_head_forward);Motion(M_turn_head_forward); 
	Motion(135);
	Motion(135);
 
	int NO_LINE_CNT = 0;
	while (true) {
		std::cout << "--------------------------------------" << std::endl;
		std::cout << "<<MODE>> " << MODE << std::endl;
 
 
		pthread_mutex_lock(&frameLocker);
		current_Frame = Frame;
		pthread_mutex_unlock(&frameLocker);
 
		if (!current_Frame.empty())
		{
			cv::resize(current_Frame, image, cv::Size(320, 240), 0, 0, CV_INTER_NN);
 
			if (MODE == D_FIND) {
				Motion(M_base);
				BinaryColors(image, c_black, 1);
 
				approx_temp = DrawShapeDetection_Control(small_Binaryframe, squareframe);
 
				if (NINE) { find_direction(approx_temp); }
 
				if (MODE == D_LEFT) {
					MODE = D_END;
					usleep(800000); Motion(M_turn_left); usleep(800000); Motion(M_turn_left); usleep(800000);
					
				}
				else if (MODE == D_RIGHT) {
					MODE = D_END;
					usleep(8000000); Motion(M_turn_right); usleep(800000); Motion(M_turn_right); usleep(800000);
					
				}
				else if (MODE == D_FORWARD) {
					MODE = D_END;
					usleep(800000);  Motion(M_walk_forward); usleep(1800000);
					
				}
			}
			else if (MODE == D_END){
				usleep(800000);  Motion(M_walk_forward); 
				usleep(800000);  Motion(M_walk_forward); 
				usleep(800000);  Motion(M_walk_forward); 
				BinaryColors(image, c_line, 1);
				cut_bin(small_Binaryframe);
				line_detect(small_Binaryframe);
				if(NEW){ MODE =L_SLOW; }
			}
			else if (MODE == L_NO_LINE) {
				
				
				if (NO_LINE_CNT / 3 == 0) { // 진짜 노라인
					NO_LINE_CNT = 0;
					usleep(800000);
					Motion(M_base);
 
					//
					usleep(800000);
					Motion(M_turn_head_left);
					usleep(500000);
					pthread_mutex_lock(&frameLocker);
					current_Frame = Frame;
					pthread_mutex_unlock(&frameLocker);
 
					if (!current_Frame.empty())
					{
						cv::resize(current_Frame, image, cv::Size(320, 240), 0, 0, CV_INTER_NN);
						BinaryColors(image, c_line, 1);
						cut_bin(small_Binaryframe);
						line_detect(small_Binaryframe);
						if (MODE != L_NO_LINE){
							MODE = L_SLOW;
							usleep(100000);
							Motion(M_walk_left);
							Motion(M_walk_left);
							Motion(M_walk_left);
 							usleep(800000);
							Motion(M_turn_head_forward);
				std::cout << "left" << std::endl;
						}
					}
					//
					if (MODE != L_SLOW) {
						usleep(800000);
						Motion(M_turn_head_right);
						usleep(500000);
						pthread_mutex_lock(&frameLocker);
						current_Frame = Frame;
						pthread_mutex_unlock(&frameLocker);;
 
						if (!current_Frame.empty())
						{
							cv::resize(current_Frame, image, cv::Size(320, 240), 0, 0, CV_INTER_NN);
							BinaryColors(image, c_line, 1);
							cut_bin(small_Binaryframe);
							line_detect(small_Binaryframe);
							if (MODE != L_NO_LINE) {
								MODE = L_SLOW;
								usleep(100000);
								Motion(M_walk_right);
								Motion(M_walk_right);
								Motion(M_walk_right);
								usleep(800000);
								Motion(M_turn_head_forward);
								std::cout << "right" << std::endl;
							}
						}
					}
 
					if (MODE != L_SLOW) {
						usleep(800000);
						Motion(199);
						usleep(800000);
						Motion(199);
usleep(800000);
						Motion(199);
						std::cout << "??????쁘엥??????????????" << std::endl;
					}
 
				}
 
 
				NO_LINE_CNT++;
				usleep(800000);
				Motion(M_turn_head_forward);
				usleep(500000);
 
				pthread_mutex_lock(&frameLocker);
				current_Frame = Frame;
				pthread_mutex_unlock(&frameLocker);
 
				if (!current_Frame.empty())
				{
					BinaryColors(image, c_line, 1);
					cut_bin(small_Binaryframe);
					line_detect(small_Binaryframe);
					if (MODE != L_NO_LINE) { 
						NO_LINE_CNT = 0; 
						MODE = L_SLOW;
					}
				}
			}
			else {
				if (MODE == L_SLOW) {}
				else if (MODE == L_LEFT_LINE) { Motion(M_hoke_left); }
				else if (MODE == L_RIGHT_LINE) { Motion(M_hoke_right); }
				else if (MODE == L_WAIT) {}
				else if (MODE == L_GOOD_LINE) { Motion(M_walk_forward); }
				BinaryColors(image, c_line, 1);
				cut_bin(small_Binaryframe);
				line_detect(small_Binaryframe);
			}
			imshow("koominju", image);
			imshow("binary", small_Binaryframe);
 
		}
		/////////////////////////
		if (cv::waitKey(10) == 27) break;
	}
}
 
 
void line_detect(cv::Mat &m_image)
{
	cv::Mat img_result, img_gray = m_image, a = m_image;
 
	int X = 0, Y = 0;
	one = 0; two = 0;  three = 0;
	//그레이스케일 이미지로 변환 
	if (!m_image.empty()) {
		cvtColor(m_image, img_gray, CV_RGB2GRAY);
	}
 
	//이진화 이미지로 변환
	cv::Mat binary_image;
	if (!m_image.empty()) {
		threshold(img_gray, img_gray, 180, 200, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
	}
	//contour를 찾는다.
	std::vector<std::vector<cv::Point> > contours;
	findContours(img_gray, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
 
	//contour를 근사화한다.(윤곽선)
	std::vector<cv::Point2f> approx;
	std::vector<cv::Point2f> approx_1;
	std::vector<cv::Point2f> approx_2;
	std::vector<cv::Point2f> approx_3;
	//	std::vector<std::vector<cv::Point2f> > out_approxs;
 
	img_result = m_image.clone();
	cv::Mat rec = m_image.clone();
 
	for (size_t i = 0; i < contours.size(); i++)
	{
		X = 0; Y = 0;
		//가장 바깥에 있는 외곽선들 좌표
		cv::Point first(1, 1);
		cv::Point second(1, 238);
		cv::Point third(318, 238);
		cv::Point fourth(318, 1);
 
		if ((contours[i][0] != first) && (contours[i][1] != second) && (contours[i][2] != third) && (contours[i][3] != fourth)) //가장 바깥에 있는 외곽선 안잡게
			cv::approxPolyDP(cv::Mat(contours[i]), approx, arcLength(cv::Mat(contours[i]), true)*0.02, true);
		else
			continue;
 
		if (fabs(cv::contourArea(cv::Mat(approx))) > 500)//면적이 일정크기 이상이어야 한다.//isContourConvex(Mat(approx)) //convex인지 검사한다.
		{
			draw_approx(img_result, approx, cv::Scalar(255, 255, 255));
		}
		//모든 코너의 각도를 구해서 더한다.
		int ang_sum = 0;
		int size = approx.size();
 
		for (int k = 0; k < size; k++) {
			int ang = GetAngleABC(approx[k], approx[(k + 1) % size], approx[(k + 2) % size]);
 
			ang_sum += abs(ang);
		}
 
		int ang_threshold = 8;
		int ang_sum_min = (180 - ang_threshold) * (size - 2);
		int ang_sum_max = (180 + ang_threshold) * (size - 2);
		ang_sum = abs(ang_sum);
 
		//ang_sum >= ang_sum_min && ang_sum <= ang_sum_max
		if (1)
		{
			if (fabs(cv::contourArea(cv::Mat(approx))) > 500) {
				cv::approxPolyDP(cv::Mat(contours[i]), approx, arcLength(cv::Mat(contours[i]), true)*0.02, true);
				for (int z = 0; z < approx.size(); z++)
				{
					X += approx[z].x;
					Y += approx[z].y;
				}
				X /= approx.size();
				Y /= approx.size();
				cv::circle(rec, cv::Point2f(X, Y), 3, cv::Scalar(255, 255, 0));
				int x[size] = { 0, }, y[size] = { 0, }, cnt = 0;;
				for (int h = 0; h < size; h++) {
					x[h] = (X * 1 + approx[h].x * 1) / 2;
					y[h] = (Y * 1 + approx[h].y * 1) / 2;
					cv::circle(rec, cv::Point2f(x[h], y[h]), 3, cv::Scalar(255, 255, 255));
					if (COLOR(m_image, V_GREEN, y[h], x[h]) && Y > UP && Y < DOWN) { cnt++; }
					if (COLOR(m_image, V_BLUE, y[h], x[h]) && Y < UP || Y>DOWN) { cnt++; }
				}
				if ((float)cnt / (float)size > 0.5) {
					if (Y > UP && Y < DOWN) {
						cv::approxPolyDP(cv::Mat(contours[i]), approx_2, arcLength(cv::Mat(contours[i]), true)*0.02, true);
						draw_approx(rec, approx_2, cv::Scalar(255, 0, 0));
						two = 1;
					}
					else if (Y < UP) {
						cv::approxPolyDP(cv::Mat(contours[i]), approx_1, arcLength(cv::Mat(contours[i]), true)*0.02, true);
						draw_approx(rec, approx_1, cv::Scalar(0, 255, 0));
						one = 1;
					}
					else {
						cv::approxPolyDP(cv::Mat(contours[i]), approx_3, arcLength(cv::Mat(contours[i]), true)*0.02, true);
						draw_approx(rec, approx_3, cv::Scalar(0, 0, 255));
						three = 1;
					}
 
				}
			}
		}
	}
	imshow("recc", rec);
 	NEW =0;
	if (two) { LineTrace(approx_2); }
 	if (one) {NEW = 1;}	
	if(MODE!=D_END){
		if (!one && two && three) { std::cout << "선 끊길라해 천천히 걷자" << std::endl; }
		else if (!one && !two && three) { std::cout << "이제 검정네모찾자" << std::endl; MODE = D_FIND; }
		else if (!one && !two && !three) {
		std::cout << "검정 네모 아니면 선 찾아서 고개 돌리자" << std::endl; MODE = L_NO_LINE;
		}
	}
	//윈도우에 출력 
	if (!current_Frame.empty())
	{
		imshow("Shape ", img_result);
	}
 
}
 
 
int REC;
std::vector<cv::Point2f> DrawShapeDetection_Control(cv::Mat &m_image, cv::Mat &out_image)
{
	cv::Mat img_result, img_gray = m_image, a = m_image;
 
	NINE = 0; REC = 0;
 
	int black_x = -1, black_y = -1;
 
	int X_7[50] = { 0, }, Y_7[50] = { 0, };
	int X_9[50] = { 0, }, Y_9[50] = { 0, };
	int IDX_9[50];
	int cnt_7 = 0, cnt_9 = 0;
 
	//그레이스케일 이미지로 변환 
	if (!m_image.empty()) {
		cvtColor(m_image, img_gray, CV_RGB2GRAY);
	}
 
	//이진화 이미지로 변환
	cv::Mat binary_image;
	if (!m_image.empty()) {
		threshold(img_gray, img_gray, 180, 200, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
	}
	//contour를 찾는다.
	std::vector<std::vector<cv::Point> > contours;
	findContours(img_gray, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
 
	//contour를 근사화한다.(윤곽선)
	std::vector<cv::Point2f> approx;
	std::vector<cv::Point2f> rectangle_approx;
	std::vector<cv::Point2f> nine_approx;
 
	img_result = m_image.clone();
 
 
	for (size_t i = 0; i < contours.size(); i++)
	{
		//가장 바깥에 있는 외곽선들 좌표
		cv::Point first(1, 1);
		cv::Point second(1, 238);
		cv::Point third(318, 238);
		cv::Point fourth(318, 1);
 
		if ((contours[i][0] != first) && (contours[i][1] != second) && (contours[i][2] != third) && (contours[i][3] != fourth)) //가장 바깥에 있는 외곽선 안잡게
			cv::approxPolyDP(cv::Mat(contours[i]), approx, arcLength(cv::Mat(contours[i]), true)*0.02, true);
		else
			continue;
 
		if (fabs(cv::contourArea(cv::Mat(approx))) > 100)//면적이 일정크기 이상이어야 한다.//isContourConvex(Mat(approx)) //convex인지 검사한다.
		{
			draw_approx(img_result, approx, cv::Scalar(255, 255, 255));
		}
		//모든 코너의 각도를 구해서 더한다.
		int ang_sum = 0;
		int size = approx.size();
 
		for (int k = 0; k < size; k++) {
			int ang = GetAngleABC(approx[k], approx[(k + 1) % size], approx[(k + 2) % size]);
 
			ang_sum += abs(ang);
 
		}
 
		int ang_threshold = 8;
		int ang_sum_min = (180 - ang_threshold) * (size - 2);
		int ang_sum_max = (180 + ang_threshold) * (size - 2);
		ang_sum = abs(ang_sum);
 
 
		if (size == 4 && ang_sum >= ang_sum_min && ang_sum <= ang_sum_max)
		{
			if (fabs(cv::contourArea(cv::Mat(approx))) > 5000) {
				cv::approxPolyDP(cv::Mat(contours[i]), approx, arcLength(cv::Mat(contours[i]), true)*0.02, true);
				for (int z = 0; z < approx.size(); z++)
				{
					black_x += approx[z].x;
					black_y += approx[z].y;
				}
				black_x /= approx.size();
				black_y /= approx.size();
 
				int x[4] = { 0, }, y[4] = { 0, }, cnt = 0;;
				for (int h = 0; h < 4; h++) {
					x[h] = (black_x * 1 + approx[h].x * 5) / 6;
					y[h] = (black_y * 1 + approx[h].y * 5) / 6;
					if (COLOR(m_image, V_PURPLE, y[h], x[h])) { cnt++; }
				}
				if (cnt > 2) {
 
					REC = 1;
					cv::approxPolyDP(cv::Mat(contours[i]), rectangle_approx, arcLength(cv::Mat(contours[i]), true)*0.02, true);
 
					cv::Mat rec = m_image.clone();
					draw_approx(rec, rectangle_approx, cv::Scalar(255, 255, 255));
					cv::circle(rec, cv::Point2f(black_x, black_y), 3, cv::Scalar(255, 255, 255));
					imshow("recc", rec);
				}
			}
		}
		else if (size == 7 && ang_sum >= ang_sum_min && ang_sum <= ang_sum_max)
		{
			if (fabs(cv::contourArea(cv::Mat(approx))) > 400) {
				cv::approxPolyDP(cv::Mat(contours[i]), approx, arcLength(cv::Mat(contours[i]), true)*0.02, true);
				for (int z = 0; z < approx.size(); z++)
				{
					X_7[cnt_7] += approx[z].x;
					Y_7[cnt_7] += approx[z].y;
				}
				X_7[cnt_7] /= approx.size();
				Y_7[cnt_7++] /= approx.size();
			}
 
		}
		else if (size == 9 && ang_sum >= 500 && ang_sum <= 1600)
		{
			if (fabs(cv::contourArea(cv::Mat(approx))) > 600 && fabs(cv::contourArea(cv::Mat(approx))) < 1300) {
 
				cv::approxPolyDP(cv::Mat(contours[i]), approx, arcLength(cv::Mat(contours[i]), true)*0.02, true);//사각형일때만 리턴할 배열에 저장
				for (int z = 0; z < approx.size(); z++)
				{
					X_9[cnt_9] += approx[z].x;
					Y_9[cnt_9] += approx[z].y;
				}
				IDX_9[cnt_9] = i;
				X_9[cnt_9] /= approx.size();
				Y_9[cnt_9++] /= approx.size();
			}
			//else
			//std::cout << "else" << std::endl;
 
		}
 
	}
 
	if (REC) {
		int idx_7 = 100, idx_9 = 100;
		int isSeven = 0, isNine = 0;
 
		for (int k = 0; k < cnt_7; k++) {
			if (black_x + 5 > X_7[k] && black_x - 5 < X_7[k] && black_y + 10 > Y_7[k] && black_y - 10 < Y_7[k]) {
				idx_7 = k;
				isSeven = 1;
			}
		}
 
		cv::Mat b;
		for (int k = 0; k < cnt_9; k++) {
 
			if (black_x + 5 > X_9[k] && black_x - 5 < X_9[k] && black_y + 10 > Y_9[k] && black_y - 10 < Y_9[k]) {
				idx_9 = k;
				isNine = 1;
				cv::approxPolyDP(cv::Mat(contours[IDX_9[k]]), nine_approx, arcLength(cv::Mat(contours[IDX_9[k]]), true)*0.02, true);
				b = m_image.clone();
				draw_approx(b, nine_approx, cv::Scalar(255, 255, 255));
 
				imshow("SELECTED", b);
			}
		}
 
		//
 
 
		if (isSeven && isNine) {
			std::cout << "???" << std::endl;
		}
		else if (isSeven) {
			std::cout << "seven" << std::endl;
			MODE = D_FORWARD;
		}
		else if (isNine) {
			std::cout << "nine" << std::endl;
			NINE = 1;
		}
		else {
			std::cout << "ㅜㅜ없엉" << std::endl;
		}
	}
 
 
	//윈도우 생성 
	namedWindow("Shape Detection", cv::WINDOW_AUTOSIZE);
	//윈도우에 출력 
	if (!current_Frame.empty())
	{
		imshow("Shape Detection", img_result);
	}
	out_image = img_result;
	return nine_approx;
	//return 1;
}
 
 
void LineTrace(std::vector<cv::Point2f> approx) {
	int size = approx.size();
	int len[2] = { 0, };
	cv::Mat img = small_Binaryframe.clone();
 
	float slope[2] = { 0, };
	int idx[4] = { 0, }, idx_cnt = 0;
 
	int idx_up[2], idx_down[2], up_cnt = 0, down_cnt = 0;
 
	for (int i = 0; i < size; i++) {
		if (approx[i].y > UP - 5 && approx[i].y < UP + 5) { idx_up[up_cnt++] = i; idx[idx_cnt++] = i; }
		else if (approx[i].y > DOWN - 5 && approx[i].y < DOWN + 5) { idx_down[down_cnt++] = i; idx[idx_cnt++] = i; }
	}
 
	if (up_cnt == 1) { idx_up[1] = idx_up[0]; }
	if (down_cnt == 1) { idx_down[1] = idx_down[0]; }
 
	if (1) {
		//std::cout << "                 " << idx_cnt << std::endl;
		int centerX = 0, centerY = 0;
		for (int i = 0; i < 4; i++) {
			centerX += approx[idx[i]].x;
			centerY += approx[idx[i]].y;
		}
		centerX /= 4;
		centerY /= 4;
		//std::cout<<"center"<<centerX<<" " << centerY<<std::endl;	
		if (centerX < 100) {
			std::cout << "왼쪽으로 게걸음" << std::endl;
			Motion(M_walk_left); return;
		}
		if (centerX > 220) {
			std::cout << "오른쪽으로 게걸음" << std::endl;
			Motion(M_walk_right); return;
		}
 
	}
	if (1) {
 
		cv::circle(img, approx[idx[0]], 3, cv::Scalar(0, 0, 255));
		cv::circle(img, approx[idx[1]], 3, cv::Scalar(0, 255, 255));
		cv::circle(img, approx[idx[2]], 3, cv::Scalar(255, 0, 0));
		cv::circle(img, approx[idx[3]], 3, cv::Scalar(0, 255, 0));
		imshow("long", img);
 
 
		for (int i = 0; i < 4; i++) {
			int j = i + 1;
			if (i == 3) j = 0;
			if ((approx[idx[i]].y < UP + 5 && approx[idx[i]].y > UP - 5) && (approx[idx[j]].y> DOWN - 5 && approx[idx[j]].y< DOWN + 5)) {
				len[0] = sqrt(pow(approx[idx[i]].x - approx[idx[j]].x, 2) + pow(approx[idx[i]].y - approx[idx[j]].y, 2));
				std::cout << i << ":" << 0 << "   " << len[0] << std::endl;
			}
			else if ((approx[idx[j]].y < UP + 5 && approx[idx[j]].y > UP - 5) && (approx[idx[i]].y> DOWN - 5 && approx[idx[i]].y< DOWN + 5)) {
				len[1] = sqrt(pow(approx[idx[i]].x - approx[idx[j]].x, 2) + pow(approx[idx[i]].y - approx[idx[j]].y, 2));
				std::cout << i << ":" << 1 << "   " << len[1] << std::endl;
			}
		}
 
 
		for (int i = 0; i < 4; i++) {
			int j = i + 1;
			if (i == 3) j = 0;
			if ((approx[idx[i]].y < UP + 5 && approx[idx[i]].y > UP - 5) && (approx[idx[j]].y> DOWN - 5 && approx[idx[j]].y< DOWN + 5)) {
				slope[0] = (float)(approx[idx[i]].y - approx[idx[j]].y) / (float)(approx[idx[i]].x - approx[idx[j]].x);
				std::cout << i << ":" << 0 << "   " << slope[0] << std::endl;
			}
			else if ((approx[idx[j]].y < UP + 5 && approx[idx[j]].y > UP - 5) && (approx[idx[i]].y> DOWN - 5 && approx[idx[i]].y< DOWN + 5)) {
				slope[1] = (float)(approx[idx[i]].y - approx[idx[j]].y) / (float)(approx[idx[i]].x - approx[idx[j]].x);
				std::cout << i << ":" << 1 << "   " << slope[1] << std::endl;
			}
		}
 
		float slope_avg = (slope[0] + slope[1]) / 2;
 
 
		if (slope_avg <2 && slope_avg >0) {
			std::cout << "왼쪽" << std::endl;
			MODE = L_LEFT_LINE;
 
		}
		else if (slope_avg > -2 && slope_avg<0) {
			std::cout << "오른쪽" << std::endl;
			MODE = L_RIGHT_LINE;
		}
		else {
			std::cout << "직진" << std::endl;
			MODE = L_GOOD_LINE;
		}
	}
}
 
void find_direction(std::vector<cv::Point2f> approx) {
	int len[9] = { 0, };
	int idx[9] = { 0,1,2,3,4,5,6,7,8 };
 
	/*std::vector<cv::Scalar> col;
	col.push_back(cv::Scalar(0, 0, 255));
	col.push_back(cv::Scalar(0, 100, 255));
	col.push_back(cv::Scalar(0, 255, 255));
	col.push_back(cv::Scalar(0, 255, 0));
	col.push_back(cv::Scalar(255, 0, 0));
	col.push_back(cv::Scalar(155, 0, 0));
	col.push_back(cv::Scalar(255, 0, 255));
	col.push_back(cv::Scalar(200, 200, 200));
	col.push_back(cv::Scalar(100, 100, 100));
	cv::Mat image = small_Binaryframe.clone();*/
	for (int i = 0; i < 9; i++) {
		int j = i + 1;
		if (i == 8) j = 0;
		len[i] = sqrt(pow(approx[i].x - approx[j].x, 2) + pow(approx[i].y - approx[j].y, 2));
 
		//cv::circle(image, approx[i], 3, col[i]);
 
		//std::cout << "len  - " << i << " : " << len[i] << std::endl;
	}
	//	imshow("slope", image);
 
	cv::Mat img = small_Binaryframe.clone();
	for (int i = 0; i < 9; i++) {
		for (int j = 0; j < 8; j++) {
			if (len[j] < len[j + 1]) {
				int temp = len[j];
				len[j] = len[j + 1];
				len[j + 1] = temp;
				temp = idx[j];
				idx[j] = idx[j + 1];
				idx[j + 1] = temp;
			}
		}
	}
 
	cv::circle(img, approx[idx[0]], 3, cv::Scalar(0, 0, 255));
	cv::circle(img, approx[idx[1]], 3, cv::Scalar(255, 0, 0));
	imshow("same", img);
 
	int IDX = idx[1];
	if (idx[0] > idx[1] && idx[1] != 8) IDX = idx[0];
 
	int FIND_IDX = (IDX + 3) % 9;
	if (approx[IDX].x > approx[FIND_IDX].x) {
		std::cout << "왼쪽!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
		MODE = D_LEFT;
	}
	else {
		std::cout << "오른쪽!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
		MODE = D_RIGHT;
	}
}
 
int GG = 0;
void GetColor(bool is_color, cv::Mat image, int j, int i, int* B, int* G, int* R)
{
	uchar *data = image.data;
	uchar *blue, *green, *red;
 
	int nRow = image.rows;
	int nCol = image.cols;
 
	if (i > nCol - 1 || j > nRow - 1) {
		GG++;
		if (GG % 100 == 0) {
			std::cout << nCol << " " << nRow << "/";
		}
		return;
	}
 
	blue = image.data + j*image.step + i*image.elemSize() + 0;
	if (is_color) {
		green = image.data + j*image.step + i*image.elemSize() + 1;
		red = image.data + j*image.step + i*image.elemSize() + 2;
	}
	*B = (int)*blue;
	if (is_color) { *G = (int)*green; *R = (int)*red; }
}
 
 
void SetColor(bool is_color, cv::Mat image, int j, int i, int B, int G, int R)
{
	uchar *data = image.data;
	uchar *blue, *green, *red;
 
	int nRow = image.rows;
	int nCol = image.cols;
 
	if (i > nCol - 1 || j > nRow - 1) {
		std::cout << "S";
		return;
	}
 
	blue = image.data + j*image.step + i*image.elemSize() + 0;
	if (is_color) {
		green = image.data + j*image.step + i*image.elemSize() + 1;
		red = image.data + j*image.step + i*image.elemSize() + 2;
	}
	*blue = B;
	if (is_color) { *green = G;	*red = R; }
 
}
 
 
bool COLOR(cv::Mat& image, cv::Vec3b color, int y, int x) {
	int R, G, B;
 
	GetColor(1, image, y, x, &B, &G, &R);
	//std::cout<<B<<" "<<G<<" "<<R<<std::endl;
 
	return (B == color[0] && G == color[1] && R == color[2]);
}
 
 
void BinaryColors(cv::Mat &frame, int color[], int num) {
	// hsv로 바꾸고
	cv::Mat Temp, temp, temp2, F_red, F_black, F_blue, F_green, F_yellow, F_white, sum, x;
	cv::Mat a, b;
 
	cv::Mat small(2, 2, CV_8U, cv::Scalar(1));
	cv::Mat element(4, 4, CV_8U, cv::Scalar(1));
 
	cvtColor(frame, Temp, CV_BGR2HSV);
	cvtColor(frame, x, CV_BGR2HSV);
 
	for (int i = 0; i<num; i++) {
		switch (color[i]) {
		case 1: // RED 
 
				//std::cout<<"1"<<std::endl; 
			inRange(Temp, cv::Scalar(0, 100, 80), cv::Scalar(10, 255, 255), temp);
			inRange(Temp, cv::Scalar(170, 100, 80), cv::Scalar(180, 255, 255), temp2);
 
			a = x; b = x;
 
			a.setTo(0); b.setTo(0);
 
			a.setTo(V_RED, temp);
			b.setTo(V_RED, temp2);
 
			F_red = a | b;
 
			cv::erode(F_red, F_red, small);
			cv::dilate(F_red, F_red, cv::Mat());
 
			sum = F_red;
 
			break;
 
		case 2: // BLUE // hsv로 바꾸고	
				//std::cout<<"2"<<std::endl; 
			cvtColor(frame, Temp, CV_BGR2HSV);
			inRange(Temp, cv::Scalar(80, 65, 65), cv::Scalar(140, 255, 255), temp);
 
			a = x;
			a.setTo(0);
			a.setTo(V_BLUE, temp);
			F_blue = a;
			cv::erode(F_blue, F_blue, small);
			cv::dilate(F_blue, F_blue, cv::Mat());
 
			sum = F_blue;
			break;
 
 
		case 3: //GREEN
			cvtColor(frame, Temp, CV_BGR2HSV);
			inRange(Temp, cv::Scalar(70, 140, 20), cv::Scalar(85, 255, 120), temp);
			//inRange(Temp, cv::Scalar(150, 140, 0), cv::Scalar(190, 180, 20), temp);
			a = x;
			a.setTo(0);
			a.setTo(V_GREEN, temp);
			F_green = a;
			cv::erode(F_green, F_green, small);
			cv::dilate(F_green, F_green, cv::Mat());
 
			sum = F_green;
			break;
 
 
 
			/*	// YCBCR 바꾸고
			//std::cout<<"3"<<std::endl;
			cvtColor(frame, F_green, CV_BGR2YCrCb);
 
			// 초록색으로 이진화
			inRange(F_green, cv::Scalar(1, 0, 0), cv::Scalar(160, 119, 119), temp);
			inRange(F_green, cv::Scalar(50, 0, 0), cv::Scalar(250, 115, 115), temp2);
 
			temp = temp | temp2;
 
			a = x;
			a.setTo(0);
			a.setTo(V_GREEN, temp);
 
			F_green = a;
 
			cv::erode(F_green, F_green, small); // default 3x3
			cv::dilate(F_green, F_green, element); // element
			cv::dilate(F_green, F_green, element);
 
			sum = F_green;
			break;
			*/
		case 4:
			//std::cout<<"4"<<std::endl;
 
			inRange(Temp, cv::Scalar(25, 110, 110), cv::Scalar(32, 255, 255), temp);
			a = x;
			a.setTo(0);
			a.setTo(V_YELLOW, temp);
 
			F_yellow = a;
 
			cv::erode(F_yellow, F_yellow, small);
			cv::dilate(F_yellow, F_yellow, cv::Mat());
 
			sum = F_yellow;
			break;
		case 5:
			inRange(frame, cv::Scalar(200, 200, 200), cv::Scalar(255, 255, 255), temp);
			a = x;
			a.setTo(0);
			a.setTo(V_WHITE, temp);
 
			F_white = a;
 
			cv::erode(F_white, F_white, small);
			cv::dilate(F_white, F_white, cv::Mat());
 
			sum = F_white;
			break;
		case 6:
			inRange(frame, cv::Scalar(30, 30, 30), cv::Scalar(100, 100, 100), temp);
			a = x;
			a.setTo(0);
			a.setTo(V_PURPLE, temp);
 
			F_black = a;
 
			cv::erode(F_black, F_black, small);
			cv::dilate(F_black, F_black, cv::Mat());
 
			sum = F_black;
			break;
		}//switch	
 
		if (i == 0) Binaryframe = sum;
		else Binaryframe = sum | Binaryframe;
 
	}//for
	for (int y = 0; y < Binaryframe.rows; y++) {
		for (int x = 0; x < Binaryframe.cols; x++) {
			if (y == 0 || y == Binaryframe.rows - 1 || x == 0 || x == Binaryframe.cols - 1
				|| y == 1 || y == Binaryframe.rows - 2 || x == 1 || x == Binaryframe.cols - 2) {
				SetColor(1, Binaryframe, y, x, 0, 0, 0);
			}
		}
	}
 
	cv::resize(Binaryframe, small_Binaryframe, cv::Size(320, 240), 0, 0, CV_INTER_NN);
 
 
}
 
 
int GetAngleABC(cv::Point a, cv::Point b, cv::Point c)
{
	cv::Point ab(b.x - a.x, b.y - a.y);
	cv::Point cb(b.x - c.x, b.y - c.y);
 
	float dot = (ab.x * cb.x + ab.y * cb.y); // dot product
	float cross = (ab.x * cb.y - ab.y * cb.x); // cross product
 
	float alpha = atan2(cross, dot);
 
	return (int)floor(alpha * 180.0 / CV_PI + 0.5);
}
 
 
void draw_approx(cv::Mat& image, std::vector<cv::Point2f> approx, cv::Scalar color) {
 
	int size = approx.size();
 
	//Contour를 근사화한 직선을 그린다.
	if (size % 2 == 0) {
		cv::line(image, approx[0], approx[approx.size() - 1], color, 3);
 
		for (int k = 0; k < size - 1; k++)
			cv::line(image, approx[k], approx[k + 1], color, 3);
 
		for (int k = 0; k < size; k++)
			cv::circle(image, approx[k], 3, cv::Scalar(0, 0, 0));
	}
	else {
		cv::line(image, approx[0], approx[approx.size() - 1], color, 3);
 
		for (int k = 0; k < size - 1; k++)
			cv::line(image, approx[k], approx[k + 1], color, 3);
 
		for (int k = 0; k < size; k++)
			cv::circle(image, approx[k], 3, cv::Scalar(0, 0, 0));
	}
}
 
 
void *updateFrame(void* arg)
{
	while (1)
	{
		cv::Mat tempframe;
		capture >> tempframe;
		pthread_mutex_lock(&frameLocker);
		Frame = tempframe;
		pthread_mutex_unlock(&frameLocker);
	}
	pthread_exit((void *)0);
}