#include "video.h"
#include "RobotProtocol.h"

#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>



cv::Mat Frame;
cv::Mat Binaryframe, small_Binaryframe;
cv::Mat squareframe;
// 영상 16배 축소
int divide = 4 ;
int ScreenX = 640 / divide ;
int ScreenY = 480 / divide ;
int right = 0 , left = 0  ;
int redposition ;
bool Right = true ; 
int nored = 0 ;
int start_first = 0 ;


int thresh = 200;
int max_thresh = 255;

void DrawShapeDetection_Control(cv::Mat &m_image, cv::Mat &out_image);

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
		Frame = tempframe;
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

	if(i > nCol - 1 || j > nRow - 1)
	{
		return;
	}

	blue = image.data + j*image.step +i*image.elemSize() + 0;
	if(is_color)
	{
		green = image.data + j*image.step + i*image.elemSize() + 1;
		red = image.data + j*image.step + i*image.elemSize() + 2;
	}

	*B = (int)*blue;
	if(is_color)
	{
		*G = (int)*green;
		*R = (int)*red;
	}
}

void SetColor(bool is_color, cv::Mat image, int j, int i, int B, int G=0, int R=0)
{
	uchar *data = image.data;
	uchar *blue, *green, *red;

	int nRow = image.rows;
	int nCol = image.cols;

	if(i > nCol - 1 || j > nRow - 1)
	{
		return;
	}

	blue = image.data + j*image.step + i*image.elemSize() + 0;
	if(is_color)
	{
		green = image.data + j*image.step + i*image.elemSize() + 1;
		red = image.data + j*image.step + i*image.elemSize() + 2;
	}
	*blue = B;
	if(is_color)
	{
		*green = G;
		*red = R;
	}
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
				inRange(Temp, cv::Scalar(0,100,80), cv::Scalar(10,255,255), temp ) ;
				inRange(Temp, cv::Scalar(170,100,80), cv::Scalar(180,255,255),temp2) ;
				
				a=x; b = x;
				
				a.setTo(0); b.setTo(0);
				
				a.setTo(red, temp);
				b.setTo(red, temp2);	
				
				F_red = a | b;

				cv::erode(F_red, F_red , small ) ;	
				cv::dilate(F_red, F_red , cv::Mat()) ;
	
				sum = F_red;

			break;
	
			case 2 : // BLUE // hsv로 바꾸고	
				cvtColor(frame, Temp, CV_BGR2HSV) ; 
	 			inRange(Temp, cv::Scalar(80,65,65), cv::Scalar(140,255,255),temp);	
				a = x;
				a.setTo(0);
				a.setTo(blue, temp);
				F_blue = a;
				cv::erode(F_blue, F_blue , small ) ;
				cv::dilate(F_blue,F_blue , cv::Mat() ) ;
	
				sum = F_blue;
			break;


			case 3 : //GREEN
				// YCBCR 바꾸고
				cvtColor(frame, F_green, CV_BGR2YCrCb) ; 
			
				// 초록색으로 이진화 
				inRange(F_green, cv::Scalar(1,0,0), cv::Scalar(160,119,119),temp )  ;
				inRange(F_green, cv::Scalar(50,0,0), cv::Scalar(250,115,115),temp2 )  ;
	
				temp = temp | temp2 ;
					
				a =x;
				a.setTo(0);
				a.setTo(green, temp);

				F_green= a;
			
				cv::erode(F_green,F_green, small) ; // default 3x3
				cv::dilate(F_green,F_green , element) ; // element
				cv::dilate(F_green, F_green, element) ;

				sum = F_green;
			break;
	
			case 4 :

		 		inRange(Temp, cv::Scalar(25,110,110), cv::Scalar(32,255,255),temp) ;
				a=x;
				a.setTo(0);				
				a.setTo(yellow,temp);

				F_yellow = a;

				cv::erode(F_yellow, F_yellow , small ) ;
				cv::dilate(F_yellow,F_yellow , cv::Mat() ) ;
	
				sum = F_yellow;
			break;
			case 5 :
		 		inRange(frame,cv::Scalar(200,200,200), cv::Scalar(255,255,255),temp) ;
				a=x;
				a.setTo(0);				
				a.setTo(white,temp);

				F_white = a;

				cv::erode(F_white, F_white , small ) ;
				cv::dilate(F_white,F_white , cv::Mat() ) ;
	
				sum = F_white;
			break;
		}//switch	

		if (i == 0) Binaryframe = sum;
		else Binaryframe = sum | Binaryframe;
		cv::resize(Binaryframe, small_Binaryframe, cv::Size(320,240) , 0 , 0 , CV_INTER_NN) ;
			
	}//for
	for(int r=0;r<small_Binaryframe.rows;r++)
	{
		for(int c=0;c<small_Binaryframe.cols;c++)
		{
			if(r == 0 || r == small_Binaryframe.rows - 1 || c == 0 || c == small_Binaryframe.cols - 1)
				SetColor(1, small_Binaryframe, r, c, 255, 255, 255);
		}
	}
}

bool exist = false;
bool move_back = false;

void sprint()
{
	capture.open(0);
	cv::Mat currentFrame, small_currentFrame;				
	
	
	pthread_mutex_init(&frameLocker, NULL);
	pthread_create(&updateThread, NULL, updateFrame, NULL);

	if( !capture.isOpened() ) {
		std::cerr << "Could not open camera" << std::endl;
		exit(1) ;
	}


	while (true) {
		pthread_mutex_lock(&frameLocker);
		currentFrame = Frame;  // 웹캠으로 부터 새로운 이미지를 받는다
		pthread_mutex_unlock(&frameLocker);
			
		if (!currentFrame.empty())
		{

			int colors[] = {1};		
			int num = 1;
			cv::resize(currentFrame, small_currentFrame, cv::Size(320,240), 0, 0, CV_INTER_NN);			
			
			BinaryColors(small_currentFrame, colors, num);	
			DrawShapeDetection_Control(small_Binaryframe, squareframe);
			

			imshow("koominjun", small_currentFrame);
			//imshow("binary", small_Binaryframe);

			if (cv::waitKey(30) == 27) break;
	
			}

		}
		/////////////////////////

		

	
}






//세 점이 주어질 때 사이 각도 구하기
int GetAngleABC(cv::Point a, cv::Point b, cv::Point c)
{
    cv::Point ab(b.x - a.x, b.y - a.y);
    cv::Point cb( b.x - c.x, b.y - c.y);
 
    float dot = (ab.x * cb.x + ab.y * cb.y); // dot product
    float cross = (ab.x * cb.y - ab.y * cb.x); // cross product
 
    float alpha = atan2(cross, dot);
 
    return (int)floor(alpha * 180.0 / CV_PI + 0.5);
}
 //int COpenCVDlg::



bool near_rectangle = false;
void DrawShapeDetection_Control(cv::Mat &m_image, cv::Mat &out_image)
{
    cv::Mat img_result, img_gray=m_image;

	int idx=0;
     
    //그레이스케일 이미지로 변환 
	if(!m_image.empty()){
    	cvtColor(m_image, img_gray, CV_BGR2GRAY);
	}



    //이진화 이미지로 변환
    cv::Mat binary_image;
	if(!m_image.empty()){
 	   threshold(img_gray, img_gray, 180, 200, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
 	}
    //contour를 찾는다.
    std::vector<std::vector<cv::Point> > contours;
    findContours(img_gray, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
 
    //contour를 근사화한다.(윤곽선)
    std::vector<cv::Point2f> approx;
	std::vector<cv::Point2f> rectangle_approx;
	std::vector<cv::Point2f> rectangle_approx2;

    img_result = m_image.clone();
 
    for (size_t i = 0; i < contours.size(); i++)
    	{
		//가장 바깥에 있는 외곽선들 좌표
		cv::Point first( 1 , 1 );
		cv::Point second( 1 , 238 );
		cv::Point third( 318 , 238 );
		cv::Point fourth( 318 , 1 );


		if((contours[i][0] != first) && (contours[i][1] != second) && (contours[i][2] != third) && (contours[i][3] != fourth)) //가장 바깥에 있는 외곽선 안잡게
	        cv::approxPolyDP(cv::Mat(contours[i]), approx, arcLength(cv::Mat(contours[i]), true)*0.02, true);
		else
			continue;

        if (fabs(cv::contourArea(cv::Mat(approx) ) ) > 100)//면적이 일정크기 이상이어야 한다.
                                                  		   //isContourConvex(Mat(approx)) //convex인지 검사한다.
        	{
            int size = approx.size();
 
            //Contour를 근사화한 직선을 그린다.
            if (size % 2 == 0) {
                cv::line(img_result, approx[0], approx[approx.size() - 1], cv::Scalar(255, 255, 255), 3);
 
                for (int k = 0; k < size - 1; k++)
                    cv::line(img_result, approx[k], approx[k + 1], cv::Scalar(255, 255, 255), 3);
 
                for (int k = 0; k < size; k++)
                    cv::circle(img_result, approx[k], 3, cv::Scalar(255, 255, 255));
            	}
            else {
                cv::line(img_result, approx[0], approx[approx.size() - 1], cv::Scalar(255, 255, 255), 3);
 
                for (int k = 0; k < size - 1; k++)
                    cv::line(img_result, approx[k], approx[k + 1], cv::Scalar(255, 255, 255), 3);
 
                for (int k = 0; k < size; k++)
                    cv::circle(img_result, approx[k], 3, cv::Scalar(0,0,0));
            	}
 
 
            //모든 코너의 각도를 구해서 더한다.
            int ang_sum = 0;
 
            for (int k = 0; k < size; k++) {
                int ang = GetAngleABC(approx[k], approx[(k + 1) % size], approx[(k + 2) % size]);
 
                ang_sum += abs(ang);
            }
 
            int ang_threshold = 8;
            int ang_sum_min = (180 - ang_threshold) * (size - 2);
            int ang_sum_max = (180 + ang_threshold) * (size - 2);

			//std::cout<<"ang_sum_min"<<ang_sum_min<<"ang_sum_max"<<ang_sum_max<<"a:::"<<ang_sum<<"size"<<size<<std::endl;

            //도형을 판정한다.

            if (size == 4 && ang_sum >= ang_sum_min && ang_sum <= ang_sum_max)
			{
				//std::cout << "rectangle" << std::endl;
				
				if(idx == 0)
				{
					cv::approxPolyDP(cv::Mat(contours[i]), rectangle_approx, arcLength(cv::Mat(contours[i]), true)*0.02, true);//사각형일때만 리턴할 배열에 저장
					idx = 1;
				}
				else
				{
					cv::approxPolyDP(cv::Mat(contours[i]), rectangle_approx2, arcLength(cv::Mat(contours[i]), true)*0.02, true);//사각형일때만 리턴할 배열에 저장
					idx = 0;
				}				
			}
			else 
				std::cout << "else" << std::endl;	
			//setLabel(img_result, to_string(approx.size()), contours[i]);//알수 없는 경우에는 찾아낸 꼭지점 갯수를 표시

        }
 
    }

/*
	if(rectangle_approx.size() != 0)
	{
		for(int i = 0; i < rectangle_approx.size(); i++)
		{
			std::cout<<"0:"<<rectangle_approx[i]<<std::endl;
		}
	}
	if(rectangle_approx2.size() != 0)
	{
		for(int i = 0; i < rectangle_approx2.size(); i++)
		{
			std::cout<<"1:"<<rectangle_approx2[i]<<std::endl;
		}
	}
*/

	cv::Point center;
	int side;
	cv::Point center2;
	int side2;

	if(rectangle_approx.size() != 0)//검출된 도형 중심 찾기
	{

		int sum_x = 0, sum_y = 0;
		for(int i=0; i<rectangle_approx.size(); i++)
		{
			sum_x += rectangle_approx[i].x;
			sum_y += rectangle_approx[i].y;
		}
		sum_x /= rectangle_approx.size();
		sum_y /= rectangle_approx.size();


		center = cv::Point( sum_x , sum_y );//도형 중심
		
		side = 99999;//직사각형 짧은 변 구하기
		for(int i = 1; i<rectangle_approx.size(); i++)
		{
			int temp_side = sqrt( pow(rectangle_approx[i].x - rectangle_approx[i - 1].x, 2) + pow(rectangle_approx[i].y - rectangle_approx[i - 1].y, 2) );//변 길이
			if(temp_side < side)
				side = temp_side;
		}

		circle( img_result, center, side,  cv::Scalar(255,255,255), 2, 8, 0 );
	}


	if(rectangle_approx2.size() != 0)//검출된 도형 중심 찾기
	{

		int sum_x = 0, sum_y = 0;
		for(int i=0; i<rectangle_approx2.size(); i++)
		{
			sum_x += rectangle_approx2[i].x;
			sum_y += rectangle_approx2[i].y;
		}
		sum_x /= rectangle_approx2.size();
		sum_y /= rectangle_approx2.size();


		center2 = cv::Point( sum_x , sum_y );//도형 중심

		side2 = 99999;//직사각형 짧은 변 구하기
		for(int i = 1; i<rectangle_approx2.size(); i++)
		{
			int temp_side = sqrt( pow(rectangle_approx2[i].x - rectangle_approx2[i - 1].x, 2) + pow(rectangle_approx2[i].y - rectangle_approx2[i - 1].y, 2) );//변 길이
			if(temp_side < side2)
				side2 = temp_side;
		}


		circle( img_result, center2, side,  cv::Scalar(255,255,255), 2, 8, 0 );

	}
	
	cv::Point result_center;
	int result_side;

	if(rectangle_approx.size() != 0 && rectangle_approx2.size() != 0)
	{
		int x = (center.x + center2.x) / 2;
		int y = (center.y + center2.y) / 2;

		result_center = cv::Point(x,y);
		circle( img_result, result_center, 20,  cv::Scalar(255,255,0), 2, 8, 0 );

		result_side = side + side2;

		exist = true;
	}
	else
		exist = false;

	if (!exist)
	{
		if (!move_back)
			if(!near_rectangle)
				Motion(100);//직진
			else
				ProcessMotion(100);
		else{
			//Motion(251);//전진 천천히
			Motion(109);//후진
		}
	}
	else
	{
		if (result_center.x < 100)
			/*if (!move_back)
				Motion(107);//왼쪽 호크돌기
			else*/
				Motion(110);//왼쪽으로 걷기
		else if (result_center.x > 220)
			/*if (!move_back)
				Motion(108);//오른쪽 호크돌기
			else*/
				Motion(111);//오른쪽으로 걷기
		else
			if (!move_back)
				Motion(100);//직진
			else
				Motion(109);//후진

		if (result_side > 90 && !move_back)
		{
			move_back = true;
			
			Motion(251);
		}
		else if (result_side > 70)
			near_rectangle = true;
		std::cout<<"result_side"<<result_side<<std::endl;
	}



    //윈도우 생성 
    namedWindow("Shape Detection", cv::WINDOW_AUTOSIZE);
    //윈도우에 출력 
	if(!Frame.empty())
	{
	    imshow("Shape Detection", img_result);
	} 


}
