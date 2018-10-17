#include <cstdlib>
#include <iostream>
#include <vector>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>
#define PI 3.1415926

using namespace cv;
using namespace std;

const int CANNY_LOWER_BOUND=50;
const int CANNY_UPPER_BOUND=250;
const int HOUGH_THRESHOLD=100;


int main(){
	Mat image = imread("image_124.jpg",IMREAD_COLOR);
	Rect roi(0,image.rows/3,image.cols,image.rows/3);
	Mat imgROI=image(roi);
	Mat imgROI_Gray,imgROI_Bin,imgROI_Dilation,imgROI_Erosion;
	Mat imgROI_Perspective = Mat::zeros( 160,640, CV_8UC3);
	 Mat element = getStructuringElement(MORPH_RECT, Size(2, 2));
	cvtColor(imgROI, imgROI_Gray,CV_RGB2GRAY);
	threshold(imgROI_Gray, imgROI_Bin, 120, 200.0, CV_THRESH_BINARY);
	dilate(imgROI_Bin,imgROI_Dilation,element);
	erode( imgROI_Dilation, imgROI_Erosion,element);
	imshow("erode",imgROI_Erosion);
	//Canny algorithm
	Mat contours;
	Canny(imgROI_Erosion,contours,CANNY_LOWER_BOUND,CANNY_UPPER_BOUND);
	imshow("lunkuojiance",contours);
	
	
	vector<Vec2f> lines,filtered_lines;
	map<double,int> lines_record;
	HoughLines(contours,lines,1,PI/180,HOUGH_THRESHOLD);
	Mat result(imgROI.size(),CV_8U,Scalar(255));
	imgROI.copyTo(result);
	clog<<"num of lines: "<<lines.size()<<endl;
	
	float maxRad=-2*PI;
	float minRad=2*PI;
    int maxLr = 0,leftIndex=0;
    int maxRr = -999,rightIndex=0; 
	//Draw the lines and judge the slope
	for(vector<Vec2f>::const_iterator it=lines.begin();it!=lines.end();++it)
	{
		float rho=(*it)[0];			//First element is distance rho
		float theta=(*it)[1];		//Second element is angle theta
		
		if(rho>0){
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

		clog<<"Line: ("<<rho<<","<<theta<<")\n";
	
	}
	clog<<"leftIndex: "<<leftIndex<<" rightIndex: "<<rightIndex<<endl;
	//point of intersection of the line with first row
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
	
	
	stringstream overlayedText;
	overlayedText<<"Lines: "<<filtered_lines.size();
	putText(result,overlayedText.str(),Point(10,result.rows-10),2,0.8,Scalar(0,0,255),0);
	imshow("result",result);
	waitKey(0);
}
