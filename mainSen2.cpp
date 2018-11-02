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
const int HOUGH_THRESHOLD=90;
const int BIN_THRESHOLD=130;

const int MAINTAIN = 20;
const float COE = -3;
const int STEP = 8;

int hCount = 0,maxCount = 0;
struct Pid {
	float setAng;
	float actAng;
	float err;
	float err_last;
	float err_pre;
	float kp,ki,kd;
}pid;
void pid_init() {
	pid.setAng = 0.0;
	pid.actAng = 0.0;
	pid.err = 0.0;
	pid.err_last = 0.0;
	pid.err_pre = 0.0;
	pid.kp = 1.0;
	pid.ki = 0.025;
	pid.kd = 0.2;
}
int main()
{
	init();
	pid_init();
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
	while(true)
	{
		capture>>image;
		int mid = image.cols/2;
		int y_length = image.rows/3;
		if(image.empty())
			break;

		//Set the ROI for the image
		Rect roi(0,image.rows/3,image.cols,image.rows/3);
		Mat imgROI=image(roi);
		///-------------------------------
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
///--------------------------------------------

		//Canny algorithm
		Mat contours;
		Canny(imgROI_Erosion,contours,CANNY_LOWER_BOUND,CANNY_UPPER_BOUND);
		#ifdef _DEBUG
		imshow(CANNY_WINDOW_NAME,contours);
		#endif

		vector<Vec2f> lines;
		HoughLines(contours,lines,1,PI/180,HOUGH_THRESHOLD);
		Mat result(imgROI.size(),CV_8U,Scalar(255));
		imgROI.copyTo(result);
		clog<<lines.size()<<endl;
		
		float maxRad=-2*PI;
		float minRad=2*PI;
		//Draw the lines and judge the slope
		
		float rho1 = 0;
		float rho2 = 0;
		float theta1 = 0;
		float theta2 = 0;
		int maxroh1 = 0;
		int minrho2 = 0;
		int index1 = 0;
		int index2 = 0;
		int countH = 0;
		for(vector<Vec2f>::const_iterator it=lines.begin();it!=lines.end();++it)
		{
			float rho=(*it)[0];			//First element is distance rho
			float theta=(*it)[1];		//Second element is angle theta

			//Filter to remove vertical and horizontal lines,
			//and atan(0.09) equals about 5 degrees.
			if(rho<0&&theta>1.62&&theta<3.05) {
			/*
				theta2 = theta;
				rho2 = rho;
			*/
				if(rho<rho2) {
					theta2 = theta;
					rho2 = rho;
				}
			}
			if(rho>0&&theta>0.08&&theta<1.47) {
/*
				theta1 = theta;
				rho1 = rho;
*/				if(rho>rho1) {
					theta1 = theta;
					rho1 = rho;
				}			
			}
			if((theta>0.09&&theta<1.48)||(theta>1.62&&theta<3.05))
			{	
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
			//Logan's try to auto-stop
			//80 degree to 100 degree
			if (theta>=1.4&&theta<=1.74)
			{
				countH++;
			}
		}
		if(countH>0){
			hCount ++ ;
			cout<<"hcount:"<<hCount<<endl;
			if(hCount>maxCount)maxCount = hCount;
			cout<<"maxCount:"<<maxCount<<endl;
			//if(hCount>=22){
			//	goto halt;
			//}
		}else{
			hCount = 0;
		}

		
		/*
		//中点离左边直线距离,正表示中点在其右边，反之。
		float dis1 = (mid*cos(theta1)-rho1);
		//右边
		float dis2 = (mid*cos(theta2)+rho2);
		//>0表示小车车头偏右 <0表示小车车头偏左
		float dis = dis1-dis2;
		pid.err_pre = pid.err_last;
		pid.err_last = pid.err;
		pid.err = dis;
		pid.actAng = dis;
		
		float changeAngle = pid.kp*(pid.err-pid.err_last)+pid.ki*(pid.err)+kd*(pid.err-2*pid.err_last+pid.err_pre);
		pid.actAng += changeAngle*COE;
		*/
		//求出交点坐标偏移角
		float x = 0;
		float y = 0;
		float ta = 0;
		if(theta1!=0&&theta2!=0) {
			x = (rho1/sin(theta1)-rho2/sin(theta2))/(cos(theta1)/sin(theta1)+cos(theta2)/sin(theta2));
			y = x*cos(theta2)/sin(theta2)+rho2/sin(theta2);
			ta = (x-mid)/(y_length-y);  //x-mid control neg or not
		}
		float degree = atan(ta);
		if(rho1!=0&&rho2==0) {degree = 14;}
		if(rho2!=0&&rho1==0)  {degree = -12	;}
		clog<<"rho1 rho2 "<<rho1<<"  "<<rho2<<"degree:"<<degree<<endl;
		pid.err_pre = pid.err_last;
		pid.err_last = pid.err;
		pid.err = degree;
		float changeAngle = pid.kp*(pid.err-pid.err_last)+pid.ki*(pid.err)+pid.kd*(pid.err-2*pid.err_last+pid.err_pre);
		clog<<"last ang:"<<pid.actAng<<" changeAng:"<<changeAngle<<endl;
		pid.actAng += changeAngle;
		turnTo(pid.actAng);
		controlLeft(FORWARD,STEP);
		controlRight(FORWARD,STEP);
		
		stringstream overlayedText;
		overlayedText<<"Lines: "<<lines.size();
		putText(result,overlayedText.str(),Point(10,result.rows-10),2,0.8,Scalar(0,0,255),0);
		imshow(MAIN_WINDOW_NAME,result);
		

		lines.clear();
		waitKey(1);
	}

	halt:
	clog<<"I think it's time to cease!";
	delay(2000);
	stopLeft();
	stopRight();

	return 0;
}
