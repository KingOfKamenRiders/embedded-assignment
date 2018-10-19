#include <pthread.h>
#include <cstdio>
#include <cstdlib>
#include <sys/time.h>
#include <cstdlib>
#include <iostream>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "GPIOlib.h"

#define PI 3.1415926

//Uncomment this line at run-time to skip GUI rendering
#define _DEBUG

using namespace cv;
using namespace std;
using namespace GPIO;

const string CAM_PATH="/dev/video0";
const string MAIN_WINDOW_NAME="Processed Image";
const string CANNY_WINDOW_NAME="Canny";

const int CANNY_LOWER_BOUND=50;
const int CANNY_UPPER_BOUND=250;
const int HOUGH_THRESHOLD=100;
const int BIN_THRESHOLD = 100;
const double KP = .05,KI = 1, KD = .1;
double error_x = 0 ,last_error_x = 0;
typedef struct timeval TV;
typedef struct timespec TS;
TV now;
TS outtime;

pthread_mutex_t mutex     = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  cond   = PTHREAD_COND_INITIALIZER;


void * func1(void* ptr){
	while(1){
		//PD control
		double degree = 0;
		degree = KP * error_x + KD * (error_x - last_error_x);
		last_error_x = error_x;
		turnTo(degree);
		
		
		
		pthread_mutex_lock( &mutex );
		
		pthread_cond_wait(&cond, &mutex);
		
		pthread_mutex_unlock(&mutex);
    	printf("i'm awake\n");
	
	}
	

}


int main(){

	//create check thread
	pthread_t thread1;
	if(pthread_create(&thread1,NULL,func1,NULL)){
		printf("can't create thread1\n");
		exit(0);
	}
	
		
	gettimeofday(&now, NULL);
	TV nexttime = now;
	nexttime.tv_sec++;
	init();	
	turnTo(0);
	
	VideoCapture capture(CAM_PATH);
	//If this fails, try to open as a video camera, through the use of an integer param
	if (!capture.isOpened())
	{
		capture.open(atoi(CAM_PATH.c_str()));
	}

	double dWidth=capture.get(CV_CAP_PROP_FRAME_WIDTH);			//the width of frames of the video
	double dHeight=capture.get(CV_CAP_PROP_FRAME_HEIGHT);		//the height of frames of the video
	clog<<"Frame Size: "<<dWidth<<"x"<<dHeight<<endl;

	Mat image;
	
	while(1){
		capture>>image;
		
		if(image.empty()){
			clog<<"empty image"<<endl;
			break;
		}
		//Set the ROI for the image
		Rect roi(0,image.rows/3,image.cols,image.rows/3);

		Mat imgROI=image(roi);
		#ifdef _DEBUG
		imshow("ROI",imgROI);
		#endif
		Mat imgROI_Gray,imgROI_Bin,imgROI_Dilation,imgROI_Erosion;
		Mat imgROI_Perspective = Mat::zeros( 160,640, CV_8UC3);
		 Mat element = getStructuringElement(MORPH_RECT, Size(2, 2));
		cvtColor(imgROI, imgROI_Gray,CV_RGB2GRAY);
		threshold(imgROI_Gray, imgROI_Bin, BIN_THRESHOLD, 200.0, CV_THRESH_BINARY);
		dilate(imgROI_Bin,imgROI_Dilation,element);
		erode( imgROI_Dilation, imgROI_Erosion,element);

		//fanzhuan
		for(int i =0 ;i<imgROI_Erosion.rows;i++){
			for(int j=0;j<imgROI_Erosion.cols*imgROI_Erosion.channels();j++){
				imgROI_Erosion.at<uchar>(i, j)= 255- imgROI_Erosion.at<uchar>(i, j);
			}
		}

		//Canny algorithm
		Mat contours;
		Canny(imgROI_Erosion,contours,CANNY_LOWER_BOUND,CANNY_UPPER_BOUND);
		#ifdef _DEBUG
		imshow(CANNY_WINDOW_NAME,contours);
		#endif

		vector<Vec2f> lines,filtered_lines;
		HoughLines(contours,lines,1,PI/180,HOUGH_THRESHOLD);
		Mat result(imgROI.size(),CV_8U,Scalar(255));
		imgROI.copyTo(result);
		//clog<<lines.size()<<endl;
		
		float maxRad=-2*PI;
		float minRad=2*PI;
		int maxLr = 0,leftIndex=0;
    	int maxRr = -999,rightIndex=0; 
		//Draw the lines and judge the slope
		for(vector<Vec2f>::const_iterator it=lines.begin();it!=lines.end();++it)
		{
			float rho=(*it)[0];			//First element is distance rho
			float theta=(*it)[1];		//Second element is angle theta

			//Filter to remove vertical and horizontal lines,
			//and atan(0.09) equals about 5 degrees.
			if(rho>0 && theta < 1.6){
				if(rho>maxLr){
					leftIndex = it-lines.begin();
					maxLr = rho;
				}
			}else if(rho < 0){
				if(rho > maxRr){
					rightIndex = it-lines.begin();
					maxRr = rho;
				}
			}
			#ifdef _DEBUG
			clog<<"Line: ("<<rho<<","<<theta<<")\n";
			#endif
		}
		if(maxLr>0&&maxRr>-999){
			double rho1 = lines[leftIndex][0],theta1 = lines[leftIndex][1];
			double rho2 = lines[rightIndex][0],theta2 = lines[rightIndex][1];
			clog << rho1 <<" "<<theta1<<endl;
			clog<<rho2<<" "<<theta2<<endl;
			Point pt1(rho1/cos(theta1),0);
			//point of intersection of the line with last row
			Point pt2((rho1-result.rows*sin(theta1))/cos(theta1),result.rows);
			//Draw a line
			line(result,pt1,pt2,Scalar(0,255,255),3,CV_AA);
	
			Point pt3(rho2/cos(theta2),0);
			//point of intersection of the line with last row
			Point pt4((rho2-result.rows*sin(theta2))/cos(theta2),result.rows);
			line(result,pt3,pt4,Scalar(0,0,255),3,CV_AA);
			
			double x = (rho1*sin(theta2)-rho2*sin(theta1))/sin(theta2-theta1);
			double y = (rho1*cos(theta2)-rho2*cos(theta1))/sin(theta1-theta2);
			error_x = result.cols/2 - x;
			clog<<"x: "<<x<<" y: "<<y<<endl;
		
			#ifdef _DEBUG
			stringstream overlayedText;
			overlayedText<<"Lines: "<<filtered_lines.size();
			putText(result,overlayedText.str(),Point(10,result.rows-10),2,0.8,Scalar(0,0,255),0);
			imshow(MAIN_WINDOW_NAME,result);
			#endif
			controlLeft(FORWARD,4);
			controlRight(FORWARD,4);
			//PD control
			double degree = 0;
			degree = KP * error_x + KD * (error_x - last_error_x);
			last_error_x = error_x;
			turnTo(degree);
		}
		
		lines.clear();
		gettimeofday(&now, NULL);
		if(now.tv_sec == nexttime.tv_sec){
			clog<<"to PD controll";		
			//PD control
			double degree = 0;
			degree = KP * error_x + KD * (error_x - last_error_x);
			last_error_x = error_x;
			turnTo(degree);

			pthread_cond_signal(&cond);
			gettimeofday(&now, NULL);
			nexttime = now;
			nexttime.tv_sec+=1;
		}
		waitKey(1);
	}	
	clog<<"break out"<<endl;

}
