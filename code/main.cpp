#include <cstdlib>
#include <iostream>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "GPIOlib.h"

#define PI 3.1415926

//Uncomment this line at run-time to skip GUI rendering
//#define _DEBUG

using namespace cv;
using namespace std;
using namespace GPIO;

const string CAM_PATH="/dev/video0";
const string MAIN_WINDOW_NAME="Processed Image";
const string CANNY_WINDOW_NAME="Canny";

const int CANNY_LOWER_BOUND=50;
const int CANNY_UPPER_BOUND=250;
const int HOUGH_THRESHOLD=90;
int speed = 6;
int turn_left = 0;
int turn_flag = 0;
int turn_to_init = 0;
int i=0;


int main()
{
	GPIO::init();

	VideoCapture capture(CAM_PATH);

	if (!capture.isOpened())
	{

		capture.open(atoi(CAM_PATH.c_str()));
	}

	double dWidth=capture.get(CV_CAP_PROP_FRAME_WIDTH);			
	double dHeight=capture.get(CV_CAP_PROP_FRAME_HEIGHT);		
	clog<<"Frame Size: "<<dWidth<<"x"<<dHeight<<endl;

	Mat image;
	waitKey(100);
	turnTo(0);
	
	while(true)
	{
		
		if(turn_flag==1){
			turn_to_init++;
			if(turn_to_init==4){
				turn_to_init = 0;
				turn_flag = 0;
				clog << "turn to init" << endl;
				turnTo(0);
			}
		}
		i++;
		
        controlLeft(FORWARD, speed);
        controlRight(FORWARD, speed);
		
		if(turn_left>0){
			//clog << "turn left" << endl;
			turn_flag = 1;
			turn_to_init = 0;
            turnTo(-10);
		}else if(turn_left<0){
			//clog << "turn right" << endl;
			turn_flag = 1;
			turn_to_init = 0;
            turnTo(15);
		}else{
			//clog << "move forward" << endl;
		}
		

		capture>>image;
		if(image.empty())
			break;


		Rect roi(0,image.rows/3,image.cols,image.rows/3);

		Mat imgROI=image(roi);

		//Canny algorithm
		Mat contours;

		Canny(imgROI,contours,CANNY_LOWER_BOUND,CANNY_UPPER_BOUND);
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
		turn_left = 0;
		for(vector<Vec2f>::const_iterator it=lines.begin();it!=lines.end();++it)
		{
			float rho=(*it)[0];			//First element is distance rho
			float theta=(*it)[1];		//Second element is angle theta

			#ifdef _DEBUG
			//point of intersection of the line with first row
			Point pt1(rho/cos(theta),0);
			//point of intersection of the line with last row
			Point pt2((rho-result.rows*sin(theta))/cos(theta),result.rows);
			//Draw a line
			line(result,pt1,pt2,Scalar(0,255,255),3,CV_AA);
			#endif
			
			//Filter to remove vertical and horizontal lines,
			//and atan(0.09) equals about 5 degrees.
			if(rho < 0 && theta>=1.90 && theta<=2.40){
				turn_left=1;
			}else if(rho > 0 && 2.0>=theta && 1.10<=theta){
				turn_left=-1;
			}else{
			}
			
			#ifdef _DEBUG
			clog << "Line: ("<<rho<<","<<theta<<")\n";
			clog << "turn_left: " << turn_left << endl;
			#endif
		}

		#ifdef _DEBUG
		stringstream overlayedText;
		overlayedText<<"Lines: "<<lines.size();
		putText(result,overlayedText.str(),Point(10,result.rows-10),2,0.8,Scalar(0,0,255),0);
		imshow(MAIN_WINDOW_NAME,result);
		#endif

		lines.clear();

		waitKey(1);
		clog << "i:" << i << endl;
		if(i>=880)break;
	}
		stopLeft();
    	stopRight();
	return 0;
}
