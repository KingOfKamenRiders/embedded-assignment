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

#define PI 3.1415926

//Uncomment this line at run-time to skip GUI rendering
#define _DEBUG

using namespace cv;
using namespace std;

const string CAM_PATH="/dev/video0";
const string MAIN_WINDOW_NAME="Processed Image";
const string CANNY_WINDOW_NAME="Canny";

const int CANNY_LOWER_BOUND=50;
const int CANNY_UPPER_BOUND=250;
const int HOUGH_THRESHOLD=150;
typedef struct timeval TV;
typedef struct timespec TS;
TV now;
TS outtime;

pthread_mutex_t mutex     = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  cond   = PTHREAD_COND_INITIALIZER;


void * func1(void* ptr){
	while(1){
		for(int i=0;i<2;i++)
			printf("int thread1\n");
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
		if(image.empty())
			break;

		//Set the ROI for the image
		Rect roi(0,image.rows/3*2,image.cols,image.rows/3);

		Mat imgROI=image(roi);
		imshow(MAIN_WINDOW_NAME,image);
		imshow("myframe",imgROI);
		Mat imgROI_Gray,imgROI_Bin,imgROI_Dilation,imgROI_Erosion;
		Mat imgROI_Perspective = Mat::zeros( 160,640, CV_8UC3);
		 Mat element = getStructuringElement(MORPH_RECT, Size(2, 2));
		cvtColor(imgROI, imgROI_Gray,CV_RGB2GRAY);
		threshold(imgROI_Gray, imgROI_Bin, 150, 200.0, CV_THRESH_BINARY);
		dilate(imgROI_Bin,imgROI_Dilation,element);
		erode( imgROI_Dilation, imgROI_Erosion,element);

		//fanzhuan
		for(int i =0 ;i<imgROI_Erosion.rows;i++){
			for(int j=0;j<imgROI_Erosion.cols*imgROI_Erosion.channels();j++){
				imgROI_Erosion.at<uchar>(i, j)= 255- imgROI_Erosion.at<uchar>(i, j);
			}
		}

		//tou shi bian huan
		//Point2f srcFixed[] = {Point2f(5,100), Point2f(5,540),Point2f(160,5),Point2f(160,640)};
		//Point2f dstFixed[] = { Point2f(5,5),  Point2f(5,640),Point2f(160,5),Point2f(160,640)};
		//Mat M = getPerspectiveTransform(srcFixed, dstFixed);
		//warpPerspective(imgROI_Erosion, imgROI_Perspective, M, imgROI_Perspective.size());

		//Canny algorithm
		Mat contours;
		Canny(imgROI_Erosion,contours,CANNY_LOWER_BOUND,CANNY_UPPER_BOUND);
		#ifdef _DEBUG
		imshow(CANNY_WINDOW_NAME,contours);
		#endif

		vector<Vec2f> lines,filtered_lines;
		HoughLines(contours,lines,1,PI/90,HOUGH_THRESHOLD);
		Mat result(imgROI.size(),CV_8U,Scalar(255));
		imgROI.copyTo(result);
		//clog<<lines.size()<<endl;
		
		float maxRad=-2*PI;
		float minRad=2*PI;
		//Draw the lines and judge the slope
		for(vector<Vec2f>::const_iterator it=lines.begin();it!=lines.end();++it)
		{
			float rho=(*it)[0];			//First element is distance rho
			float theta=(*it)[1];		//Second element is angle theta

			//Filter to remove vertical and horizontal lines,
			//and atan(0.09) equals about 5 degrees.
			if((theta>0.09&&theta<1.48)||(theta>1.62&&theta<3.05))
			{
				filtered_lines.push_back(*it);
				if(theta>maxRad)
					maxRad=theta;
				if(theta<minRad)
					minRad=theta;
				
				#ifdef _DEBUG
				//point of intersection of the line with first row
				Point pt1(rho/cos(theta),0);
				//point of intersection of the line with last row
				Point pt2((rho-result.rows*sin(theta))/cos(theta),result.rows);
				//Draw a line
				line(result,pt1,pt2,Scalar(0,255,255),3,CV_AA);
				#endif
			}

			#ifdef _DEBUG
			clog<<"Line: ("<<rho<<","<<theta<<")\n";
			#endif
		}

		#ifdef _DEBUG
		stringstream overlayedText;
		overlayedText<<"Lines: "<<filtered_lines.size();
		putText(result,overlayedText.str(),Point(10,result.rows-10),2,0.8,Scalar(0,0,255),0);
		imshow(MAIN_WINDOW_NAME,result);
		#endif

		lines.clear();
		waitKey(1);
		
		
		
		gettimeofday(&now, NULL);
		if(now.tv_sec == nexttime.tv_sec){
			printf("to wake thread1 up \n");
			pthread_cond_signal(&cond);
			gettimeofday(&now, NULL);
			nexttime = now;
			nexttime.tv_sec+=1;
		}
	}	
	

}
