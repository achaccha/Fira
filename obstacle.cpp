#include "obstacle.h"
#include "RobotProtocol.h"

#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>

cv::Mat Frame;
cv::Mat Binaryframe, small_Binaryframe;
cv::Mat currentFrame, small_currentFrame;

cv::Mat Red_binaryframe, Blue_binaryframe, Yellow_binaryframe;
// 영상 16배 축소
int divide = 4 ;
int ScreenX = 640 / divide ;
int ScreenY = 480 / divide ;
int right = 0 , left = 0  ;
int redposition ;
bool Right = true ; 
int nored = 0 ;
int start_first = 0 ;


int left_x, right_x;

int I = 0;

#define RED 1
#define BLUE 2
#define YELLOW 4


#define Large_s_H_1 0
#define Large_e_H_1 20
#define Large_s_H_2 170
#define Large_e_H_2 180

#define Large_s_S 30
#define Large_e_S 255

#define Large_s_V 30
#define Large_e_V 255


#define Small_s_H_1 0 
#define Small_e_H_1 5
#define Small_s_H_2 170 
#define Small_e_H_2 180

#define Small_s_S 50
#define Small_e_S 255

#define Small_s_V 20
#define Small_e_V 255


std::vector<cv::Vec3b> V_colors;
cv::Vec3b V_RED = cv::Vec3b(0, 0, 255);
cv::Vec3b V_BLUE = cv::Vec3b(255, 0, 0);
cv::Vec3b V_GREEN = cv::Vec3b(0, 255, 0);
cv::Vec3b V_YELLOW = cv::Vec3b(0, 255, 255);
cv::Vec3b V_PURPLE = cv::Vec3b(255, 0, 255);
cv::Vec3b V_WHITE = cv::Vec3b(255, 255, 255);
cv::Vec3b V_BLACK = cv::Vec3b(0, 0, 0);



#define GO_LEFT 10
#define GO_RIGHT 11
#define NO_OBSTACLE 12
#define STILL_OBSTACLE 13
#define OVERCOME_RED 14

#define CRAWLING 199//177
#define CRAWLING_SET 122//177
#define WALK_LEFT_2 178
#define WALK_RIGHT_2 179
#define CAMERA_UP 163
#define CAMERA_DOWN 162
#define CAMERA_MORE_DOWN 164
#define CAMERA_LEFT45 151
#define CAMERA_LEFT60 152
#define CAMERA_LEFT90 153
#define CAMERA_RIGHT45 155
#define CAMERA_RIGHT60 156
#define CAMERA_RIGHT90 157
#define CAMERA_FORWARD 158
#define WALK_FORWARD 105
#define WALK_LEFT 110
#define WALK_RIGHT 111
#define WALK_BACK 106
#define BASE 99


#define CAMERA_WHOLE 120
#define CAMERA_SIDE 121
#define CAMERA_SMALL 123



#define RETURN_CENTER 0
#define RETURN_LOW_Y 1

#define RETURN_DECIDE 2
#define RETURN_ONLY_YELLOW 3
#define RETURN_ONLY_BLUE 4

#define SLEEP 5
#define NO_SLEEP 6




/*
int s_H_1 = Small_s_H_1;
int e_H_1 = Small_e_H_1;
int s_H_2 = Small_s_H_2;
int e_H_2 = Small_e_H_2;
int s_S = Small_s_S;
int e_S = Small_e_S;
int s_V = Small_s_V;
int e_V = Small_e_V;
*/

int s_H_1 = Large_s_H_1;
int e_H_1 = Large_e_H_1;
int s_H_2 = Large_s_H_2;
int e_H_2 = Large_e_H_2;
int s_S = Large_s_S;
int e_S = Large_e_S;
int s_V = Large_s_V;
int e_V = Large_e_V;




void hard(){
	
	int y = 4, r = 1, b = 2, ll = 0, rr = 1;
	
	std::cout<<"<<<<<<<<<<" << I <<">>>>>>>>>>" << std::endl;
	if(I ==100){}
	else if (I == 0) { Motion2(CAMERA_WHOLE); I++;} 
	
	else if (I == 1) { g(b);} 
	else if (I == 2) { f(b,rr);  }
	else if (I == 3) { g(y); 	}
	else if (I == 4) { s(ll); 	}
	else if (I == 5) { g(b); 	}
	else if (I == 6) { f(b, rr); 	}
	else if (I == 7) { c(1); }

	else if (I == 8) { Motion2(199); I++; }
	else { Motion2(WALK_FORWARD); I=100;}
}





void draw_approx(cv::Mat& image, std::vector<cv::Point2f> approx) {
 
	int size = approx.size();
 
	//Contour를 근사화한 직선을 그린다.
	if (size % 2 == 0 && size !=0) {
		cv::line(image, approx[0], approx[approx.size() - 1], cv::Scalar(255, 255, 255), 3);
 
		for (int k = 0; k < size - 1; k++)
			cv::line(image, approx[k], approx[k + 1], cv::Scalar(255, 255, 255), 3);
 
		for (int k = 0; k < size; k++)
			cv::circle(image, approx[k], 3, cv::Scalar(255, 255, 255));
	}
	else if(size!=0) {
		cv::line(image, approx[0], approx[approx.size() - 1], cv::Scalar(255, 255, 255), 3);
 
		for (int k = 0; k < size - 1; k++)
			cv::line(image, approx[k], approx[k + 1], cv::Scalar(255, 255, 255), 3);
 
		for (int k = 0; k < size; k++)
			cv::circle(image, approx[k], 3, cv::Scalar(0, 0, 0));
	}
}
 
 


void ImShow(std::string str, cv::Mat &image){
	imshow(str, image);
}

void balance_white(cv::Mat &mat){
        double discard_ratio = 0.05;
        int hists[3][256];
        memset(hists, 0, 3*256*sizeof(int));

        for(int y=0 ; y <mat.rows; ++y){
                uchar* ptr = mat.ptr<uchar>(y);
                for(int x=0 ; x<mat.cols ; ++x){
                        for(int j=0; j<3 ; ++j){
                                hists[j][ptr[x*3+j]] += 1;
                        }
                }
        }

        int total = mat.cols * mat.rows;
        int vmin[3], vmax[3];
        for(int i=0; i<3; ++i){
                for(int j=0; j<255; ++j){
                        hists[i][j+1] += hists[i][j];
                }
                vmin[i]=0;
                vmax[i]=255;
                while(hists[i][vmin[i]] < discard_ratio * total)
                        vmin[i] += 1;
                while(hists[i][vmax[i]] > (1 - discard_ratio) * total)
                        vmax[i] -= 1;
                if(vmax[i] < 255-1)
                        vmax[i] += 1;
        }

        for(int y=0; y < mat.rows ; ++y){
                uchar* ptr = mat.ptr<uchar>(y);
                for(int x=0; x<mat.cols; ++x){
                        for(int j=0; j<3; ++j){
                                int val = ptr[x*3 + j];
                                if(val < vmin[j])
                                        val = vmin[j];
                                if(val > vmax[j])
                                        val = vmax[j];
                                ptr[x*3 +j] = static_cast<uchar>((val - vmin[j]) * 255.0 / (vmax[j] - vmin[j]));
                        }
                }
        }
}

/////////////////////////////////////////////////
pthread_mutex_t frameLocker; //뮤텍스 잠금
pthread_t updateThread; //thread ID는 updateThread
cv::VideoCapture capture;

void *updateFrame(void* arg)
{
	while(1)
	{
		cv::Mat tempframe;
		capture >> tempframe;
		pthread_mutex_lock(&frameLocker);
		//balance_white(tempframe);
		Frame = tempframe;
		balance_white(Frame);/////////////////////////////////////////////////////////
		pthread_mutex_unlock(&frameLocker);
	}
	pthread_exit((void *)0);
}
////////////////////////////////////////////////////

void GetColor(bool is_color, cv::Mat image, int j, int i, int* B, int* G = 0, int* R = 0)
{
	uchar *data = image.data;
	uchar *blue, *green, *red;

	int nRow = image.rows;
	int nCol = image.cols;

	if (i > nCol - 1 || j > nRow - 1)
	{
		return;
	}

	blue = image.data + j*image.step + i*image.elemSize() + 0;
	if (is_color)
	{
		green = image.data + j*image.step + i*image.elemSize() + 1;
		red = image.data + j*image.step + i*image.elemSize() + 2;
	}

	*B = (int)*blue;
	if (is_color)
	{
		*G = (int)*green;
		*R = (int)*red;
	}
}


void SetColor(bool is_color, cv::Mat image, int j, int i, int B, int G, int R)
{
	uchar *data = image.data;
	uchar *blue, *green, *red;

	int nRow = image.rows;
	int nCol = image.cols;

	if (i > nCol - 1 || j > nRow - 1)
	{
		return;
	}

	blue = image.data + j*image.step + i*image.elemSize() + 0;
	if (is_color)
	{
		green = image.data + j*image.step + i*image.elemSize() + 1;
		red = image.data + j*image.step + i*image.elemSize() + 2;
	}
	*blue = B;
	if (is_color)
	{
		*green = G;
		*red = R;
	}
}

bool COLOR(cv::Mat& image, cv::Vec3b color, int y, int x) {
	int R, G, B;

	GetColor(1, image, y, x, &B, &G, &R);
	//std::cout<<B<<" "<<G<<" "<<R<<std::endl;
	return (B == color[0] && G == color[1] && R == color[2]);
}


//컬러 번호들 이진화
void BinaryColors(cv::Mat &frame,  int color[], int num){
	// hsv로 바꾸고
	cv::Mat Temp , temp, temp2, F_red, F_blue, F_green, F_yellow, F_white , sum,x;
	cv::Mat a,b;
	
	cv::Mat small(2, 2, CV_8U, cv::Scalar(1));
	cv::Mat element(4, 4, CV_8U, cv::Scalar(1));
	cv::Vec3b red = cv::Vec3b(0,0,255);
	cv::Vec3b blue = cv::Vec3b(255,0,0);
	cv::Vec3b green = cv::Vec3b(0,255,0);
	cv::Vec3b yellow = cv::Vec3b(0,255,255);
	cv::Vec3b white = cv::Vec3b(255,255,255);

	cvtColor(frame, Temp, CV_BGR2HSV) ;
	cvtColor(frame, x, CV_BGR2HSV) ;

	for (int i =0; i<num; i++){
		switch (color[i]){
			case 1 : // RED 
				inRange(Temp, cv::Scalar(s_H_1,s_S,s_V), cv::Scalar(e_H_1,e_S,e_V), temp);				
				inRange(Temp, cv::Scalar(s_H_2,s_S,s_V), cv::Scalar(e_H_2,e_S,e_V),temp2);
				a = x.clone(); b = x.clone();
				
				a.setTo(0); b.setTo(0);
				
				a.setTo(red, temp);
				b.setTo(red, temp2);	
				
				F_red = a | b;

				cv::erode(F_red, F_red , small ) ;	
				cv::dilate(F_red, F_red , cv::Mat()) ;
	
				sum = F_red.clone();

			break;
	
			case 2 : // BLUE // hsv로 바꾸고	
				cvtColor(frame, Temp, CV_BGR2HSV) ; 
				inRange(Temp, cv::Scalar(90,30,0), cv::Scalar(140,255,255),temp);	//S	 			
//inRange(Temp, cv::Scalar(100,50,0), cv::Scalar(140,255,255),temp);	//S 65 240 V 65 255
//				inRange(Temp, cv::Scalar(100,120,100), cv::Scalar(130,255,255),temp);	//S 65 240 V 65 255
				a = x.clone();
				a.setTo(0);
				a.setTo(blue, temp);
				F_blue = a.clone();
				cv::erode(F_blue, F_blue , small ) ;
				cv::dilate(F_blue,F_blue , cv::Mat() ) ;
	
				sum = F_blue.clone();
			break;

			case 3 : //GREEN
				// YCBCR 바꾸고
				cvtColor(frame, F_green, CV_BGR2YCrCb) ; 
			
				// 초록색으로 이진화 
				inRange(F_green, cv::Scalar(1,0,0), cv::Scalar(160,119,119),temp )  ;
				inRange(F_green, cv::Scalar(50,0,0), cv::Scalar(250,115,115),temp2 )  ;
	
				temp = temp | temp2 ;
					
				a = x.clone();
				a.setTo(0);
				a.setTo(green, temp);

				F_green= a.clone();
			
				cv::erode(F_green,F_green, small) ; // default 3x3
				cv::dilate(F_green,F_green , element) ; // element
				cv::dilate(F_green, F_green, element) ;

				sum = F_green.clone();
			break;
	
			case 4 :
				inRange(Temp, cv::Scalar(20,40,100), cv::Scalar(28,255,255),temp) ;
				
				//inRange(Temp, cv::Scalar(25,50,140), cv::Scalar(28,255,255),temp) ;
				a = x.clone();
				a.setTo(0);				
				a.setTo(yellow,temp);

				F_yellow = a.clone();

				cv::erode(F_yellow, F_yellow , small ) ;
				cv::dilate(F_yellow,F_yellow , cv::Mat() ) ;
	
				sum = F_yellow.clone();
			break;

			case 5 :
		 		inRange(frame,cv::Scalar(200,200,200), cv::Scalar(255,255,255),temp) ;
				a = x.clone();
				a.setTo(0);				
				a.setTo(white,temp);

				F_white = a.clone();

				cv::erode(F_white, F_white , small ) ;
				cv::dilate(F_white,F_white , cv::Mat() ) ;
	
				sum = F_white.clone();
			break;
		}//switch	

		if (i == 0) Binaryframe = sum;
		else Binaryframe = sum | Binaryframe;
		cv::resize(Binaryframe, small_Binaryframe, cv::Size(320,240) , 0 , 0 , CV_INTER_NN) ;
		for (int y = 0; y < small_Binaryframe.rows; y++) {
			for (int x = 0; x < small_Binaryframe.cols; x++) {
				if (y == 0 || y == small_Binaryframe.rows - 1 || x == 0 || x == small_Binaryframe.cols - 1
					|| y == 1 || y == small_Binaryframe.rows - 2 || x == 1 || x == small_Binaryframe.cols - 2) {
					SetColor(true, small_Binaryframe, y, x, 0, 0, 0);
				}
			}
		}
	}//for
	
}
void single_BinaryColors(cv::Mat &frame, cv::Mat &out_frame, int color[], int num) {
	// hsv로 바꾸고
	cv::Mat Temp , temp, temp2, F_red, F_blue, F_green, F_yellow, F_white , sum,x;
	cv::Mat a,b;
	
	cv::Mat small(2, 2, CV_8U, cv::Scalar(1));
	cv::Mat element(4, 4, CV_8U, cv::Scalar(1));
	cv::Vec3b red = cv::Vec3b(0,0,255);
	cv::Vec3b blue = cv::Vec3b(255,0,0);
	cv::Vec3b green = cv::Vec3b(0,255,0);
	cv::Vec3b yellow = cv::Vec3b(0,255,255);
	cv::Vec3b white = cv::Vec3b(255,255,255);

	cvtColor(frame, Temp, CV_BGR2HSV) ;
	cvtColor(frame, x, CV_BGR2HSV) ;

	for (int i =0; i<num; i++){
		switch (color[i]){
			case 1 : // RED 
				inRange(Temp, cv::Scalar(s_H_1,s_S,s_V), cv::Scalar(e_H_1,e_S,e_V), temp);				
				inRange(Temp, cv::Scalar(s_H_2,s_S,s_V), cv::Scalar(e_H_2,e_S,e_V),temp2);
				a = x.clone(); b = x.clone();
				
				a.setTo(0); b.setTo(0);
				
				a.setTo(red, temp);
				b.setTo(red, temp2);	
				
				F_red = a | b;

				cv::erode(F_red, F_red , small ) ;	
				cv::dilate(F_red, F_red , cv::Mat()) ;
	
				sum = F_red.clone();

			break;
	
			case 2 : // BLUE // hsv로 바꾸고	
				cvtColor(frame, Temp, CV_BGR2HSV) ; 
	 			inRange(Temp, cv::Scalar(0,0,0), cv::Scalar(140,255,255),temp);	//S 65 240 V 65 255
//				inRange(Temp, cv::Scalar(100,120,100), cv::Scalar(130,255,255),temp);	//S 65 240 V 65 255
				a = x.clone();
				a.setTo(0);
				a.setTo(blue, temp);
				F_blue = a.clone();
				cv::erode(F_blue, F_blue , small ) ;
				cv::dilate(F_blue,F_blue , cv::Mat() ) ;
	
				sum = F_blue.clone();
			break;

			case 3 : //GREEN
				// YCBCR 바꾸고
				cvtColor(frame, F_green, CV_BGR2YCrCb) ; 
			
				// 초록색으로 이진화 
				inRange(F_green, cv::Scalar(1,0,0), cv::Scalar(160,119,119),temp )  ;
				inRange(F_green, cv::Scalar(50,0,0), cv::Scalar(250,115,115),temp2 )  ;
	
				temp = temp | temp2 ;
					
				a = x.clone();
				a.setTo(0);
				a.setTo(green, temp);

				F_green= a.clone();
			
				cv::erode(F_green,F_green, small) ; // default 3x3
				cv::dilate(F_green,F_green , element) ; // element
				cv::dilate(F_green, F_green, element) ;

				sum = F_green.clone();
			break;
	
			case 4 :
				inRange(Temp, cv::Scalar(25,50,140), cv::Scalar(28,255,255),temp) ;
				a = x.clone();
				a.setTo(0);				
				a.setTo(yellow,temp);

				F_yellow = a.clone();

				cv::erode(F_yellow, F_yellow , small ) ;
				cv::dilate(F_yellow,F_yellow , cv::Mat() ) ;
	
				sum = F_yellow.clone();
			break;

			case 5 :
		 		inRange(frame,cv::Scalar(200,200,200), cv::Scalar(255,255,255),temp) ;
				a = x.clone();
				a.setTo(0);				
				a.setTo(white,temp);

				F_white = a.clone();

				cv::erode(F_white, F_white , small ) ;
				cv::dilate(F_white,F_white , cv::Mat() ) ;
	
				sum = F_white.clone();
			break;
		}//switch	

		if (i == 0) out_frame = sum;
		//else out_frame = sum | Binaryframe;
		cv::resize(out_frame, out_frame, cv::Size(320,240) , 0 , 0 , CV_INTER_NN) ;
			
	}//for
}
/*
void single_BinaryColors(cv::Mat &frame, cv::Mat &out_frame, int color) {
	// hsv로 바꾸고
	cv::Mat Temp, temp, temp2, F_red, F_blue, F_green, F_yellow, F_white, sum, x;
	cv::Mat a, b;

	cv::Mat large(20, 20, CV_8U, cv::Scalar(1));

	cv::Mat small(2, 2, CV_8U, cv::Scalar(1));
	cv::Mat element(4, 4, CV_8U, cv::Scalar(1));
	cv::Vec3b red = cv::Vec3b(0, 0, 255);
	cv::Vec3b blue = cv::Vec3b(255, 0, 0);
	cv::Vec3b green = cv::Vec3b(0, 255, 0);
	cv::Vec3b yellow = cv::Vec3b(0, 255, 255);
	cv::Vec3b white = cv::Vec3b(255, 255, 255);

	cvtColor(frame, Temp, CV_BGR2HSV);
	cvtColor(frame, x, CV_BGR2HSV);

	switch (color) {
	case 1: // RED 
		//inRange(Temp, cv::Scalar(0,100,80), cv::Scalar(5,255,255), temp);				
		//inRange(Temp, cv::Scalar(170,100,80), cv::Scalar(180,255,255),temp2);
		inRange(Temp, cv::Scalar(0,100,30), cv::Scalar(5,255,200), temp);				
		inRange(Temp, cv::Scalar(170,100,30), cv::Scalar(180,255,200),temp2);

		a = x.clone(); b = x.clone();

		a.setTo(0); b.setTo(0);

		a.setTo(red, temp);
		b.setTo(red, temp2);

		F_red = a | b;

		cv::erode(F_red, F_red, small);
		cv::dilate(F_red, F_red, cv::Mat());

		sum = F_red.clone();

		break;

	case 2: // BLUE // hsv로 바꾸고	
		cvtColor(frame, Temp, CV_BGR2HSV);
		inRange(Temp, cv::Scalar(100,50,50), cv::Scalar(130,255,255),temp);	//S 65 240 V 65 255
//		inRange(Temp, cv::Scalar(100,120,100), cv::Scalar(130,255,255),temp);	//S 65 240 V 65 255
		a = x.clone();
		a.setTo(0);
		a.setTo(blue, temp);
		F_blue = a.clone();
		cv::erode(F_blue, F_blue, small);
		cv::dilate(F_blue, F_blue, cv::Mat());

		sum = F_blue.clone();
		break;


	case 3: //GREEN
			// YCBCR 바꾸고
		cvtColor(frame, F_green, CV_BGR2YCrCb);

		// 초록색으로 이진화 
		inRange(F_green, cv::Scalar(1, 0, 0), cv::Scalar(160, 119, 119), temp);
		inRange(F_green, cv::Scalar(50, 0, 0), cv::Scalar(250, 115, 115), temp2);

		temp = temp | temp2;

		a = x.clone();
		a.setTo(0);
		a.setTo(green, temp);

		F_green = a.clone();

		cv::erode(F_green, F_green, small); // default 3x3
		cv::dilate(F_green, F_green, element); // element
		cv::dilate(F_green, F_green, element);

		sum = F_green.clone();
		break;

	case 4:
		inRange(Temp, cv::Scalar(25,80,140), cv::Scalar(32,255,255),temp) ;
		a = x.clone();
		a.setTo(0);
		a.setTo(yellow, temp);

		F_yellow = a.clone();

		cv::erode(F_yellow, F_yellow, small);
		cv::dilate(F_yellow, F_yellow, cv::Mat());

		sum = F_yellow.clone();
		break;
	case 5:
		inRange(frame, cv::Scalar(200, 200, 200), cv::Scalar(255, 255, 255), temp);
		a = x.clone();
		a.setTo(0);
		a.setTo(white, temp);

		F_white = a.clone();

		cv::erode(F_white, F_white, small);
		cv::dilate(F_white, F_white, cv::Mat());

		sum = F_white.clone();
		break;
	}
	out_frame = sum.clone();//여기서 에러가 나는 이유는??

	cv::resize(out_frame, out_frame, cv::Size(320, 240), 0, 0, CV_INTER_NN);
	for (int y = 0; y < out_frame.rows; y++) {
		for (int x = 0; x < out_frame.cols; x++) {
			if (y == 0 || y == out_frame.rows - 1 || x == 0 || x == out_frame.cols - 1
				|| y == 1 || y == out_frame.rows - 2 || x == 1 || x == out_frame.cols - 2) {
				SetColor(true, out_frame, y, x, 0, 0, 0);
			}
		}
	}
}
*/

//세 점이 주어질 때 사이 각도 구하기
int GetAngleABC(cv::Point a, cv::Point b, cv::Point c)
{
	cv::Point ab(b.x - a.x, b.y - a.y);
	cv::Point cb(b.x - c.x, b.y - c.y);

	float dot = (ab.x * cb.x + ab.y * cb.y); // dot product
	float cross = (ab.x * cb.y - ab.y * cb.x); // cross product

	float alpha = atan2(cross, dot);

	return (int)floor(alpha * 180.0 / CV_PI + 0.5);
}

/*
void get_boundary_of_shape(cv::Mat &m_image, std::vector<cv::Point2f> approx)
{
	int low_y, left, right;
	for(int i=0; i<approx.size(); i++)
	{
		int temp_low_y = Get_low_y(m_image, approx);
		int temp_left = get_left_approx(m_image, approx);
		int temp_right = get_right_approx(m_image, approx);
	}
}
*/

int Get_low_y(cv::Mat &m_image, std::vector<cv::Point2f> approx)
{
	int low_y = -1;
	for (int i = 0; i<approx.size(); i++)
	{
		if(approx[i].y > low_y)
		{
			low_y = approx[i].y;
		}
	}

	if(low_y >= m_image.rows)
		low_y = m_image.rows - 1;
	else if(low_y < 0)
		low_y = 0;
	
	return low_y;
}


std::vector<cv::Point2f> Draw_Shape_Detection_Control(cv::Mat &m_image, cv::Mat &out_image)
{
	cv::Mat img_result, img_gray = m_image, tempp=m_image;

	int idx = 0;

	//그레이스케일 이미지로 변환 
	if (!m_image.empty()) {
		cvtColor(m_image, img_gray, CV_BGR2GRAY);
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

	std::vector<cv::Point2f> max_approx;
	int max_area = 0;

	img_result = m_image.clone();

	int low_y = 0;


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

		
		int temp_low_y = Get_low_y(img_gray, approx);
		


		if (fabs(cv::contourArea(cv::Mat(approx))) > 10000)//면적이 일정크기 이상이어야 한다.
														 //isContourConvex(Mat(approx)) //convex인지 검사한다.
		{
			cv::approxPolyDP(cv::Mat(contours[i]), approx, arcLength(cv::Mat(contours[i]), true)*0.02, true);
			draw_approx(tempp,approx);
			if(temp_low_y > low_y)
			{
				cv::approxPolyDP(cv::Mat(contours[i]), max_approx, arcLength(cv::Mat(contours[i]), true)*0.02, true);
				low_y = temp_low_y;
			}
			/*
			if (fabs(cv::contourArea(cv::Mat(approx))) > max_area)//면적이 가장 큰 도형만을 리턴함
			{
				cv::approxPolyDP(cv::Mat(contours[i]), max_approx, arcLength(cv::Mat(contours[i]), true)*0.02, true);
				max_area = fabs(cv::contourArea(cv::Mat(approx)));
			}
			*/
		}
	}//for
std::cout<< "********************size : "<<fabs(cv::contourArea(cv::Mat(max_approx))) <<std::endl;

	m_image = img_result;


	draw_approx(m_image, max_approx);

	ImShow("ALL Shape", tempp);
	ImShow("LOW Shape", m_image);

	return max_approx;
}

cv::Point Get_Center_of_approx(cv::Mat &m_image, std::vector<cv::Point2f> approx)
{
	cv::Point center;

	if (approx.size() != 0)//검출된 도형 중심 찾기
	{

		int sum_x = 0, sum_y = 0;
		for (int i = 0; i<approx.size(); i++)
		{
			sum_x += approx[i].x;
			sum_y += approx[i].y;
		}
		sum_x /= approx.size();
		sum_y /= approx.size();


		center = cv::Point(sum_x, sum_y);//도형 중심

		circle(m_image, center, 20, cv::Scalar(255, 255, 255), 2, 8, 0);
	}

	if(center.x >= m_image.cols)
		center.x = m_image.cols - 1;
	else if(center.x < 0)
		center.x = 0;

	if(center.y >= m_image.rows)
		center.y = m_image.rows - 1;
	else if(center.y < 0)
		center.y = 0;

	ImShow("center",m_image);

	return center;
}


int Get_long_side(std::vector<cv::Point2f> approx)
{
	int side = 0;//직사각형 긴 변 구하기
	for (int i = 1; i<approx.size(); i++)
	{
		int temp_side = sqrt(pow(approx[i].x - approx[i - 1].x, 2) + pow(approx[i].y - approx[i - 1].y, 2));//변 길이
		if (temp_side > side)
			side = temp_side;
	}

	return side;
}




int left_blue_count = 0, left_red_count = 0, left_yellow_count = 0;
int right_blue_count = 0, right_red_count = 0, right_yellow_count = 0;

void watch_left_up()
{
	int R, G, B;
	left_red_count = 0;


	Motion2(CAMERA_UP);//고개 들기
	Motion2(CAMERA_LEFT45);//카메라 왼쪽으로 회전
	usleep(2000000);

	pthread_mutex_lock(&frameLocker);
	currentFrame = Frame;  // 웹캠으로 부터 새로운 이미지를 받는다
	pthread_mutex_unlock(&frameLocker);

	if (!currentFrame.empty())
	{
		int colors[] = { 1 };
		int num = 1;
		cv::resize(currentFrame, small_currentFrame, cv::Size(320, 240), 0, 0, CV_INTER_NN);

		BinaryColors(small_currentFrame, colors, num);
	}
	else
	{
		std::cout<<"err"<<std::endl;
		return;
	}


	for (int r = 0; r < small_Binaryframe.rows; r++)
	{
		for (int c = 0; c < small_Binaryframe.cols; c++)
		{
			GetColor(true, small_Binaryframe, r, c, &B, &G, &R);

			if (R == 255)
				left_red_count++;
		}
	}
	std::cout<<"left_red_count : "<<left_red_count<<std::endl;	
	
	    ImShow("left_up", small_Binaryframe);
}

void watch_right_up()
{
	int R, G, B;
	right_red_count = 0;

	Motion2(CAMERA_UP);//고개 들기
	Motion2(CAMERA_RIGHT45);//카메라 오른쪽으로 회전
	usleep(2000000);

	pthread_mutex_lock(&frameLocker);
	currentFrame = Frame;  // 웹캠으로 부터 새로운 이미지를 받는다
	pthread_mutex_unlock(&frameLocker);

	if (!currentFrame.empty())
	{
		int colors[] = { 1 };
		int num = 1;
		cv::resize(currentFrame, small_currentFrame, cv::Size(320, 240), 0, 0, CV_INTER_NN);

		BinaryColors(small_currentFrame, colors, num);
	}
	else
	{
		std::cout<<"err"<<std::endl;
		return;
	}


	for (int r = 0; r < small_Binaryframe.rows; r++)
	{
		for (int c = 0; c < small_Binaryframe.cols; c++)
		{
			GetColor(true, small_Binaryframe, r, c, &B, &G, &R);

			if (R == 255)
				right_red_count++;
		}
	}
	std::cout<<"right_red_count : "<<right_red_count<<std::endl;
	//확인 용이하게 그림 그리기
	

    
	    ImShow("right_up", small_Binaryframe);
	

}





int get_left_approx(cv::Mat m_image, std::vector<cv::Point2f > approx)
{
	if (approx.size() != 0)//검출된 도형 중심 찾기
	{		
		int left_x = m_image.cols;

		for (int i = 0; i<approx.size(); i++)
		{
			if(approx[i].x < left_x)
			{
				left_x = approx[i].x;
			}
		}
	
		if(left_x >= m_image.cols)
			left_x = m_image.cols - 1;
		else if(left_x < 0)
			left_x = 0;
		/////////////////////////////////////////////////////////%%

		for(int i =3; i<237;i++){
			SetColor(1, m_image, i, left_x,255,255,255);
		}
		ImShow("left_x",m_image);

		return left_x;
	}
}

int get_right_approx(cv::Mat m_image, std::vector<cv::Point2f > approx)
{
	if (approx.size() != 0)//검출된 도형 중심 찾기
	{
		int right_x = 0;

		for (int i = 0; i<approx.size(); i++)
		{
			if(approx[i].x > right_x)
			{
				right_x = approx[i].x;
			}
		}
	
		if(right_x >= m_image.cols)
			right_x = m_image.cols - 1;
		else if(right_x < 0)
			right_x = 0;
				
			/////////////////////////////////%%
		for(int i =3; i<237;i++){
			SetColor(1, m_image, i, right_x,255,255,255);
		}
		ImShow("right_x",m_image);

		return right_x;
	}
}



int measure_distance_down_blue_yellow(int angle, int return_value, int return_only, int Is_sleep )
{
	cv::Mat Blue_binaryframe2;
	cv::Mat Yellow_binaryframe2;

	Motion2(CAMERA_DOWN);//숙이기
	if(angle == CAMERA_FORWARD)
	{
		Motion2(CAMERA_FORWARD);
	}
	if(angle == CAMERA_LEFT45)
	{
		Motion2(CAMERA_LEFT45);//카메라 왼쪽으로 45도 회전
	}
	else if(angle == CAMERA_LEFT60)
	{
		Motion2(CAMERA_LEFT60);//카메라 왼쪽으로 60도 회전
	}
	else if(angle == CAMERA_LEFT90)
	{
		Motion2(CAMERA_LEFT90);//카메라 왼쪽으로 90도 회전
	}
	else if(angle == CAMERA_RIGHT45)
	{
		Motion2(CAMERA_RIGHT45);//카메라 오른쪽으로 45도 회전
	}
	else if(angle == CAMERA_RIGHT60)
	{
		Motion2(CAMERA_RIGHT60);//카메라 왼쪽으로 60도 회전
	}
	else if(angle == CAMERA_RIGHT90)
	{
		Motion2(CAMERA_RIGHT90);//카메라 오른쪽으로 90도 회전
	}
	if(Is_sleep != NO_SLEEP)
		//usleep(2000000);
	

	pthread_mutex_lock(&frameLocker);
	currentFrame = Frame;  // 웹캠으로 부터 새로운 이미지를 받는다
	pthread_mutex_unlock(&frameLocker);
	
	
	if (!currentFrame.empty())
	{	
		cv::resize(currentFrame, small_currentFrame, cv::Size(320, 240), 0, 0, CV_INTER_NN);

		//balance_white(small_currentFrame);

		int colors[] = { 2 };
		int colors2[] = { 4 };

		BinaryColors(small_currentFrame,colors,1);
		Blue_binaryframe2 = small_Binaryframe.clone();

		BinaryColors(small_currentFrame,colors2,1);
		Yellow_binaryframe2 = small_Binaryframe.clone();
		
		//Blue_binaryframe2 = small_currentFrame.clone();
		//single_BinaryColors(small_currentFrame, Blue_binaryframe2, colors, 1);

		//Yellow_binaryframe2 = small_currentFrame.clone();
		//single_BinaryColors(small_currentFrame, Yellow_binaryframe2, colors2, 1);
	}
	else
	{
		std::cout << "err" << std::endl;
		return 0;
	}

	//ImShow("yellow_during_measreu", Yellow_binaryframe2);
	//ImShow("blue_during_measure", Blue_binaryframe2);

	std::vector<cv::Point2f > Blue_approx = Draw_Shape_Detection_Control(Blue_binaryframe2, Blue_binaryframe2);//윤곽선 꼭지점 좌표 따오기
	std::vector<cv::Point2f > Yellow_approx = Draw_Shape_Detection_Control(Yellow_binaryframe2, Yellow_binaryframe2);//윤곽선 꼭지점 좌표 따오기

   
	ImShow("now_now1", Blue_binaryframe2);
	ImShow("now_now1_Yellow", Yellow_binaryframe2);


	cv::Point Blue_center = Get_Center_of_approx(Blue_binaryframe2, Blue_approx);//중심 구하기
	cv::Point Yellow_center = Get_Center_of_approx(Yellow_binaryframe2, Yellow_approx);//중심 구하기

	int Blue_low_y = Get_low_y(Blue_binaryframe2, Blue_approx);//윤곽선 가장 밑에 있는 꼭지점 y좌표 따오기
	int Yellow_low_y = Get_low_y(Yellow_binaryframe2, Yellow_approx);//윤곽선 가장 밑에 있는 꼭지점 y좌표 따오기

	int low_line_distance = 0;//위에서부터의 거리(숫자가 클수록 로봇한테 가까이 있는 거)
	int center_distance = 0;

	int Cols;//영상 너비 받아오기
	cv::Mat selected_image;//노란색이랑 파란색중에 그림 그려줄 이미지 선택하기

	if(return_only == RETURN_DECIDE)
	{
		if (Blue_low_y < Yellow_low_y)//노란색이랑 파란색 y좌표 구해서 더 가까이 있는 놈을 선택
		{
			low_line_distance = Yellow_low_y;
			Cols = Yellow_binaryframe2.cols;
			selected_image = Yellow_binaryframe2;
			left_x = get_left_approx(Yellow_binaryframe2, Yellow_approx);
			right_x = get_right_approx(Yellow_binaryframe2, Yellow_approx);
		}
		else
		{
			low_line_distance = Blue_low_y;
			Cols = Blue_binaryframe2.cols;
			selected_image = Blue_binaryframe2;
			left_x = get_left_approx(Blue_binaryframe2, Blue_approx);
			right_x = get_right_approx(Blue_binaryframe2, Blue_approx);
		}
	
		if (Blue_center.y < Yellow_center.y)//센터 y 좌표가 더 밑에 있는 놈 선택
		{
			center_distance = Yellow_center.y;
		}
		else
		{
			center_distance = Blue_center.y;
		}
	}
	else if(return_only == RETURN_ONLY_BLUE)
	{
		low_line_distance = Blue_low_y;
		Cols = Blue_binaryframe2.cols;
		selected_image = Blue_binaryframe2;

		center_distance = Blue_center.y;
	}
	else
	{
		low_line_distance = Yellow_low_y;
		Cols = Yellow_binaryframe2.cols;
		selected_image = Yellow_binaryframe2;

		center_distance = Yellow_center.y;
	}
	
	int real_distance = selected_image.rows - low_line_distance;//위에서 아래까지의 거리였으니까 전체 높이에서 그만큼을 뺴야 아래에서 로봇에서 그 지점까지의 거리임
	int real_center_distance = selected_image.rows - center_distance;

	if(return_only == RETURN_ONLY_YELLOW)
	{
		std::cout<<"yelow"<<real_distance<<std::endl;
	}


	//확인 용이하게 그림 그리기
	for (int i = 0; i < Cols; i++)
	{
		SetColor(true, selected_image, low_line_distance, i, 255, 255, 255);
		SetColor(true, selected_image, center_distance, i, 0, 0, 255);
	}


	//-------------------------------------------------
/*
	Blue_approx.clear();
	Yellow_approx.clear();

	std::vector<cv::Point2f >().swap(Blue_approx);
	std::vector<cv::Point2f >().swap(Yellow_approx);
*/
	//-------------------------------------------------

	std::cout<<"distance:"<<real_distance<<std::endl;

	
	ImShow("direction", selected_image);
	

	if(return_value == RETURN_LOW_Y)
		return real_distance;
	else
		return real_center_distance;
}

#define GOING_LEFT 0
#define GOING_RIGHT 1

int watch_forward(int direction)
{ //

	if (direction == GOING_LEFT)
	{// && left_x < 280
		int dis = measure_distance_down_blue_yellow(CAMERA_FORWARD);
		if (dis < 20)
		{
			std::cout<<"left_x dis : "<<dis<<std::endl;
			return STILL_OBSTACLE;
		}
		else {
			std::cout << "obstacle done" << std::endl;
			return NO_OBSTACLE;
		}
	}
	else
	{//&& right_x > 40
		int dis = measure_distance_down_blue_yellow(CAMERA_FORWARD);
		if(dis< 20 )
		{
			std::cout<<"right_x dis : "<<dis<<std::endl;
			return STILL_OBSTACLE;
		}
		else {
			return NO_OBSTACLE;
			std::cout << "obstacle done" << std::endl;
		}
	}
}

void check_left_right(bool Is_recursive)
{
	int right_distance = measure_distance_down_blue_yellow(CAMERA_RIGHT90);
	usleep(1000000);
	int left_distance = measure_distance_down_blue_yellow(CAMERA_LEFT90);
	usleep(1000000);	

	int limit_distance = 5;
	int limit_distance2 = 70;

	if (left_distance > right_distance)//작을수록 가까운 거 => right가 가까이 있는 거니까 멀리있는 left 쪽으로 이동
	{
		if(Is_recursive)
			Motion2(WALK_LEFT_2);//몇 번 가는 거 동작쪽에서 묶어주기
		while (true)
		{
			if (watch_forward(GOING_LEFT) == STILL_OBSTACLE){
				int left_distance_temp = measure_distance_down_blue_yellow(CAMERA_LEFT90);
				std::cout<<"temp"<<left_distance_temp<<std::endl;
				if(left_distance_temp < limit_distance)
				{
					while(measure_distance_down_blue_yellow(CAMERA_LEFT90) < limit_distance2)
					{
						Motion2(WALK_BACK);//후진
					}
					check_left_right(true);
					break;
				}
				else
				{
					Motion2(WALK_LEFT);//좌측으로 걷기
				}
			}
			else{
				break;
			}
		}

	}
	else
	{
		if(Is_recursive)
			Motion2(WALK_RIGHT_2);//몇 번 가는 거 동작쪽에서 묶어주기

		while (true)
		{
			if (watch_forward(GOING_RIGHT) == STILL_OBSTACLE){
				int right_distance_temp = measure_distance_down_blue_yellow(CAMERA_RIGHT90);
				if(right_distance_temp < limit_distance)
				{
					while(measure_distance_down_blue_yellow(CAMERA_RIGHT90) < limit_distance2)
					{
						Motion2(WALK_BACK);//후진
					}
					check_left_right(true);
					break;
				}
				else
				{
					Motion2(WALK_RIGHT);//좌측으로 걷기
				}
			}
			else{
				break;
			}
		}
	}
}

int get_red_pixel()
{
	pthread_mutex_lock(&frameLocker);
	currentFrame = Frame;  // 웹캠으로 부터 새로운 이미지를 받는다
	pthread_mutex_unlock(&frameLocker);
	
	if (!currentFrame.empty())
	{
		int colors[] = { 1,2,4 };
		int num = 3;
		cv::resize(currentFrame, small_currentFrame, cv::Size(320, 240), 0, 0, CV_INTER_NN);

		//balance_white(small_currentFrame);
		BinaryColors(small_currentFrame, colors, num);

		ImShow("red pixel_ binary", small_Binaryframe);
	}

	int R, G, B;
	int Blue_count = 0, Yellow_count = 0, Red_count = 0;


	for(int r=0;r<small_Binaryframe.rows;r++)
	{
		for(int c=0;c<small_Binaryframe.cols;c++)
		{
			GetColor(true,small_Binaryframe,r,c,&B,&G,&R);
			if(B==255)
				Blue_count++;
			else if(G==255&&R==255)
				Yellow_count++;
			else if(R==255)
				Red_count++;
		}
	}
	std::cout<<"Blue"<<Blue_count<<"Yellow"<<Yellow_count<<"Red"<<Red_count<<std::endl;


	    ImShow("red pixel_aa",small_Binaryframe);


	return Red_count;
}

int find_cnt = 0;
void c(int isIDX)
{
	Motion2(CAMERA_DOWN);
	Motion2(CAMERA_FORWARD);
	usleep(100000);
	find_cnt = 0;
	while(true)
	{
		find_cnt++;
		pthread_mutex_lock(&frameLocker);
		currentFrame = Frame;  // 웹캠으로 부터 새로운 이미지를 받는다
		pthread_mutex_unlock(&frameLocker);
		
		if (!currentFrame.empty())
		{
			int colors[] = { 1,2,4 };
			int num = 3;
			cv::resize(currentFrame, small_currentFrame, cv::Size(320, 240), 0, 0, CV_INTER_NN);
	
			//balance_white(small_currentFrame);
			BinaryColors(small_currentFrame, colors, num);
	
			//ImShow("find_red_gate_binary", small_Binaryframe);
		}
	
	
		int Area_Blue[3] = {0,};
		int Area_Red[3] = {0,};
		int Area_length[4] = {40, 80, 240, 280};
		
		int R, G, B;
		int Blue_count = 0, Yellow_count = 0, Red_count = 0;
		int surface = small_Binaryframe.rows * small_Binaryframe.cols;
	
		for(int r = 0; r < small_Binaryframe.rows; r++)
		{
			for(int c = 0; c < small_Binaryframe.cols; c++)
			{
				GetColor(true, small_Binaryframe, r, c, &B, &G, &R);
	
	
				if(c >= Area_length[0] && c <= Area_length[1])
				{
					if (B == 255)
						Area_Blue[0]++;
					else if (R == 255)
						Area_Red[0]++;
				}
				else if(c >= Area_length[1] && c <= Area_length[2])
				{
					if (B == 255)
						Area_Blue[1]++;
					else if (R == 255)
						Area_Red[1]++;
				}
				else if(c >= Area_length[2] && c <= Area_length[3])
				{
					if (B == 255)
						Area_Blue[2]++;
					else if (R == 255)
						Area_Red[2]++;
				}
			}
		}
		if(Area_Blue[1] <= 1000 || abs(Area_Blue[0] - Area_Blue[2]) <200 || find_cnt>20)
		{
			if (isIDX) I++;
			find_cnt = 0;
			break;
		}
		else if(Area_Blue[0] <= Area_Blue[2])
		{
			Motion2(WALK_LEFT);
		}
		else
		{
			Motion2(WALK_RIGHT);
		}
	}

	
}

bool at_start = true;

void find_red_gate()
{
	Motion2(CAMERA_UP);//고개 들기
	Motion2(CAMERA_FORWARD);//정면 보기
	usleep(1000000);

	pthread_mutex_lock(&frameLocker);
	currentFrame = Frame;  // 웹캠으로 부터 새로운 이미지를 받는다
	pthread_mutex_unlock(&frameLocker);
	
	if (!currentFrame.empty())
	{
		int colors[] = { 1,2,4 };
		int num = 3;
		cv::resize(currentFrame, small_currentFrame, cv::Size(320, 240), 0, 0, CV_INTER_NN);

		//balance_white(small_currentFrame);
		BinaryColors(small_currentFrame, colors, num);

		ImShow("find_red_gate_binary", small_Binaryframe);
	}

	int R, G, B;
	int Blue_count = 0, Yellow_count = 0, Red_count = 0;
	int surface = small_Binaryframe.rows * small_Binaryframe.cols;

	for(int r = 0; r < small_Binaryframe.rows; r++)
	{
		for(int c = 0; c < small_Binaryframe.cols; c++)
		{
			GetColor(true, small_Binaryframe, r, c, &B, &G, &R);

			if (B == 255)
				Blue_count++;
			else if (G == 255 && R == 255)
				Yellow_count++;
			else if (R == 255)
				Red_count++;
		}
	}

	//ImShow("image",image);
	
	//ImShow("original", small_currentFrame);

	//앞에 빨간색이 있다
	std::cout<<"Red_gate_count"<<std::endl;
	if(Red_count > 35000)	//마지막날 수정(20000)
	{	
		int red_pixel = 0;
		//빨간색이 가까이 있다면 기어가기
		while(red_pixel < 35000)//71000
		{
			red_pixel = get_red_pixel(); 
			std::cout<<red_pixel<<std::endl;
			Motion2(WALK_FORWARD);
			usleep(2000000);
		}
		std::cout<<"out"<<red_pixel<<std::endl;

		c(0);

		Motion2(CRAWLING);//기어가기
	}

	Motion2(CAMERA_DOWN);//고개 숙이기
	usleep(1000000);
}


int no_obstacle_cnt = 0 ;



void f(int color, int direction)
{
	int move;
	if(direction==0){
		move = WALK_LEFT;
	}
	else {
		move = WALK_RIGHT;
	}
	Motion2(CAMERA_FORWARD);//카메라 앞으로 회전
	Motion2(CAMERA_DOWN);//고개 내리기

	pthread_mutex_lock(&frameLocker);
	currentFrame = Frame;  // 웹캠으로 부터 새로운 이미지를 받는다
	pthread_mutex_unlock(&frameLocker);

	if (!currentFrame.empty())
	{
		int colors[] = { color };
		int num = 1;
		cv::resize(currentFrame, small_currentFrame, cv::Size(320, 240), 0, 0, CV_INTER_NN);

		BinaryColors(small_currentFrame, colors, num);
	}
	else
	{
		std::cout << "err" << std::endl;
		return;
	}

	if (watch_forward(GOING_RIGHT) == STILL_OBSTACLE) {
		Motion2(move);
	}
	else {
		no_obstacle_cnt++;
		if (no_obstacle_cnt % 3 == 0) {
			no_obstacle_cnt = 0;
			std::cout << "OUT_OBSTACLE_FOR" << std::endl;
			I++;
		}
	}	
}



void s(int direction) {
	int move, cam;
	if(direction==0){
		move = WALK_LEFT;
		cam = CAMERA_LEFT90;
	}
	else {
		move = WALK_RIGHT;
		cam = CAMERA_RIGHT90;
	}

	Motion2(cam);//카메라 좌우
	Motion2(CAMERA_DOWN);//고개 내리기

	pthread_mutex_lock(&frameLocker);
	currentFrame = Frame;  // 웹캠으로 부터 새로운 이미지를 받는다
	pthread_mutex_unlock(&frameLocker);

	if (!currentFrame.empty())
	{
		int colors[] = { YELLOW };
		int num = 1;
		cv::resize(currentFrame, small_currentFrame, cv::Size(320, 240), 0, 0, CV_INTER_NN);

		BinaryColors(small_currentFrame, colors, num);
	}
	else
	{
		std::cout << "err" << std::endl;
		return;
	}
	int a;
	 a = RETURN_ONLY_YELLOW;
	
	int dis = measure_distance_down_blue_yellow(move, RETURN_LOW_Y, a, NO_SLEEP);
	std::cout << "DIS "<< dis << std::endl;

	if (dis > 10) {
		Motion2(move);
	}
	else {
		std::cout << "IN SIDE" << std::endl;
		Motion2(CAMERA_SMALL);
		I++;
	}

}

void g(int color) {
	Motion2(CAMERA_FORWARD);//정면 보기
	Motion2(CAMERA_DOWN);//숙이기
	
	pthread_mutex_lock(&frameLocker);
	currentFrame = Frame;  // 웹캠으로 부터 새로운 이미지를 받는다
	pthread_mutex_unlock(&frameLocker);

	if (!currentFrame.empty())
	{
		int colors[] = { color };
		int num = 1;
		cv::resize(currentFrame, small_currentFrame, cv::Size(320, 240), 0, 0, CV_INTER_NN);

		BinaryColors(small_currentFrame, colors, num);
	}
	else
	{
		std::cout << "err" << std::endl;
		return;
	}

	int cnt = 0;

	for (int y = 0; y < small_Binaryframe.rows; y++)
	{
		for (int x = 0; x < small_Binaryframe.cols; x++)
		{
			if (COLOR(small_Binaryframe, V_colors[color], y, x)) {
				cnt++;
			}
		}
	}
	int x=RETURN_ONLY_BLUE;
	if(color == 4) x = RETURN_ONLY_YELLOW;
	if (measure_distance_down_blue_yellow(0, RETURN_LOW_Y, x, NO_SLEEP) > 2 )//blue 원래 surface / 4 * 3
																													//if(measure_distance_down_blue_yellow(0, RETURN_LOW_Y, RETURN_ONLY_BLUE, NO_SLEEP) > 2 && measure_distance_down_blue_yellow(0, RETURN_LOW_Y, RETURN_ONLY_YELLOW, NO_SLEEP) > 10)
	{
		std::cout << "++++++++++GOTO OBSTACLE++++++++++++   :  " << color <<std::endl;
		std::cout << "Count : " << cnt << std::endl;
		std::cout << "+++++++++++++++++++++++++++++++++++" << std::endl;
		ImShow("walk while goto", small_Binaryframe);
		Motion2(WALK_FORWARD);//전진
	}
	else//찾았다
	{
		if(color == YELLOW) Motion2(CAMERA_SIDE);
		else Motion2(CAMERA_WHOLE);
		I++;
	}
}

void obstacle_action(cv::Mat &image)//image는 색깔별로 이진화된 binaryframe
{
	int R, G, B;
	int Blue_count = 0, Yellow_count = 0, Red_count = 0;
	int surface = image.rows * image.cols;


	if(at_start)//시작하자마자 빨간색이 있을 경우를 대비해서 고개 들어보기
	{
		find_red_gate();
		at_start = false;

		return;
	}
	else
	{
		Motion2(CAMERA_DOWN);//숙이기
		Motion2(CAMERA_FORWARD);//정면 보기
		//usleep(1000000);
	} 

	Blue_count = 0, Yellow_count = 0, Red_count = 0;

	for(int r = 0; r < image.rows; r++)
	{
		for(int c = 0; c < image.cols; c++)
		{
			GetColor(true, image, r, c, &B, &G, &R);

			if (B == 255)
				Blue_count++;
			else if (G == 255 && R == 255)
				Yellow_count++;
			else if (R == 255)
				Red_count++;
		}
	}

	//ImShow("image",image);
	//ImShow("original",small_currentFrame);

	if(Red_count > 9000)//5000
	{
		find_red_gate();
//		+_center();

		return;
	}

	//파란색이나 노란색이 보일 때까지는 직진
	if(Blue_count < 40000 && measure_distance_down_blue_yellow(0, RETURN_LOW_Y,RETURN_ONLY_YELLOW, NO_SLEEP) > 10)//blue 원래 surface / 4 * 3
	//if(measure_distance_down_blue_yellow(0, RETURN_LOW_Y, RETURN_ONLY_BLUE, NO_SLEEP) > 2 && measure_distance_down_blue_yellow(0, RETURN_LOW_Y, RETURN_ONLY_YELLOW, NO_SLEEP) > 10)
	{	
		std::cout<<"NO OBSTACLE++++++++++++++++++++++"<<std::endl;
		std::cout<<"Blue Count : "<<Blue_count<<std::endl;
		std::cout<<"Yellow Count : "<<Yellow_count<<std::endl;
		std::cout<<"Red Count : "<<Red_count<<std::endl;
		std::cout<<"+++++++++++++++++++++++++++++++++"<<std::endl;
		ImShow("walk while find",small_Binaryframe);
		Motion2(WALK_FORWARD);//전진
		//usleep(1000000);
	}
	else//파란색이나 노란색이 보일 때는 회피
	{
		watch_left_up();
		watch_right_up();

		if(left_red_count > 40000)//왼쪽 위 확인해서 빨간색 게이트가 있나 탐색
		{
			std::cout<<"LEFT"<<left_red_count<<std::endl;

			while (true)
			{
				if (watch_forward(GOING_LEFT) == STILL_OBSTACLE){
					Motion2(WALK_LEFT);//좌측으로 걷기
				}
				else
				{
					at_start = true;
					Motion2(WALK_LEFT_2);//빨간색 게이트 쪽으로 움직이다가 파란색이 빠졌을 때 좌측으로 두 번 걷기
					find_red_gate();
//					c(0);
					break;
				}
			}
		}
		else if(right_red_count > 40000)//오른쪽 위 확인해서 빨간색 게이트가 있나 탐색
		{
			std::cout<<"RIGHT"<<right_red_count<<std::endl;
			while (true)
			{	
				if (watch_forward(GOING_RIGHT) == STILL_OBSTACLE)
					Motion2(WALK_RIGHT);//우측으로 걷기
				else
				{
					at_start = true;
					Motion2(WALK_RIGHT_2);//빨간색 게이트 쪽으로 움직이다가 파란색이 빠졌을 때 우측으로 두 번 걷기
					find_red_gate();
               		break;
				}
			}
		}
		else//빨간색 게이트가 없어서 가장 먼 장애물 쪽으로 이동
		{
			check_left_right(false);
			at_start = true;
		}

	}

}

void obstacle() 
{
	V_colors.push_back(V_RED);
	V_colors.push_back(V_RED);
	V_colors.push_back(V_BLUE);
	V_colors.push_back(V_YELLOW);
	V_colors.push_back(V_YELLOW);


//	Motion2(CAMERA_UP);
	Motion2(CAMERA_DOWN);
//	Motion2(CAMERA_MORE_DOWN);

	Motion2(CAMERA_FORWARD);//정면 보기
	
//	usleep(100000);

	capture.open(0);

	cv::Mat currentFrame, small_currentFrame;				
		
	pthread_mutex_init(&frameLocker, NULL);
	pthread_create(&updateThread, NULL, updateFrame, NULL);

	if( !capture.isOpened() ) {
		std::cerr << "Could not open camera" << std::endl;
		exit(1) ;
	}

	while (true) 
	{
		pthread_mutex_lock(&frameLocker);
		currentFrame = Frame;  // 웹캠으로 부터 새로운 이미지를 받는다
		pthread_mutex_unlock(&frameLocker);
			
		if (!currentFrame.empty())
		{
			cv::resize(currentFrame, small_currentFrame, cv::Size(320,240), 0, 0, CV_INTER_NN);
			
			int color[] = {1,2,4};
			int num = 3;
			balance_white(small_currentFrame);
			BinaryColors(small_currentFrame, color, num);
			
//			hard();
int y = 4, r = 1, b = 2, ll = 0, rr = 1;
	
	std::cout<<"<<<<<<<<<<" << I <<">>>>>>>>>>" << std::endl;
	if(I ==100){}
	else if (I == 0) { Motion2(CAMERA_WHOLE); I++;} 
	
	else if (I == 1) { g(b);} 
	else if (I == 2) { f(b,rr);  }
	else if (I == 3) { g(y); 	}
	else if (I == 4) { s(ll); 	}
	else if (I == 5) { g(b); 	}
	else if (I == 6) { f(b, rr); 	}
	else if (I == 7) { c(1); }

	else if (I == 8) { Motion2(199); I++; }
	else { Motion2(WALK_FORWARD); I=100;}


			//obstacle_action(small_Binaryframe);
			//get_red_pixel();
			std::cout << "-------------------------------------" << std::endl;
			ImShow("koominjun", small_currentFrame);
			ImShow("bin", small_Binaryframe);
			if (cv::waitKey(30) == 27) break;


		}

	}
}
