#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

void toHSV(Mat image)  //产出是一个mask
{
	int rows = image.rows;
	int cols = image.cols;
	cols *=image.channels();
	uchar* p;
	for(int i =0;i<rows;i++){
		p = image.ptr<uchar>(i);
		for(int j = 0;j<cols;j+=3){
			if(abs(p[j]-24)<20&&abs(p[j+1]-47)<20&&abs(p[j+2]-157)<20){
				cout<<"find a pixel"<<endl;
				p[j] = 0;
				p[j+1] = 0;
				p[j+2] = 0;
			}
		}
	}
	cout<<"transform finished"<<endl;
	
}
int main(){
	Mat m = imread("./view.jpg",IMREAD_COLOR);
	cout<<"read"<<endl;
	toHSV(m);
	imshow("haha",m);
	waitKey(0);
}

