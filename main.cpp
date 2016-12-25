#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2\imgproc\imgproc.hpp"
#include "opencv2\highgui\highgui.hpp"
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include "util.h"

using namespace cv;
using namespace std;

float leftAngle;
float rightAngle;
bool rightIsFirst = true;
bool leftIsFirst = true;

#define USE_VIDEO 1

struct MyLine {
	MyLine(){}
	MyLine(CvPoint a): p0(a) { }
	MyLine(double b): x(b) { }

	double x;
	CvPoint p0;
};

struct Lane {
	Lane(){}
	Lane(CvPoint a, CvPoint b, float angle, float kl, float bl): p0(a),p1(b),angle(angle),
		votes(0),visited(false),found(false),k(kl),b(bl) { }

	CvPoint p0, p1;
	int votes;
	bool visited, found;
	float angle, k, b;
	int countnear;
};


std::vector<Lane> laneLeft;
std::vector<Lane> laneRight;
std::vector<double> xLines;
std::vector<double> yLines;
enum{
	LINE_REJECT_DEGREES = 10, // in degrees

	CANNY_MIN_TRESHOLD = 80,	  // edge detector minimum hysteresis threshold
	CANNY_MAX_TRESHOLD = 60, // edge detector maximum hysteresis threshold

	HOUGH_TRESHOLD = 5,		// line approval vote threshold
	HOUGH_MIN_LINE_LENGTH = 5,	// remove lines shorter than this treshold
	HOUGH_MAX_LINE_GAP = 10,   // join lines to one with smaller than this gaps
};


void polyfit(cv::Mat &src_x, cv::Mat &src_y, cv::Mat &dst, int order){
	CV_FUNCNAME("polyfit");
	__CV_BEGIN__;
	{
		CV_ASSERT((src_x.rows>0)&&(src_y.rows>0)&&(src_x.cols==1)&&(src_y.cols==1)
				&&(dst.cols==1)&&(dst.rows==(order+1))&&(order>=1));
		Mat X;
		X = Mat::zeros(src_x.rows, order+1,CV_64FC1);
		Mat copy;
		for(int i = 0; i <=order;i++)
		{
			copy = src_x.clone();
			pow(copy,i,copy);
			Mat M1 = X.col(i);
			copy.col(0).copyTo(M1);
		}
		Mat X_t, X_inv;
		transpose(X,X_t);
		Mat temp = X_t*X;
		Mat temp2;
		invert (temp,temp2);
		Mat temp3 = temp2*X_t;
		Mat W = temp3 * src_y;
		
		

		
#ifdef DEBUG
		cout<<"PRINTING INPUT AND OUTPUT FOR VALIDATION AGAINST MATLAB RESULTS\n";
		cout<<"SRC_X: "<<src_x<<endl;
		cout<<"SRC_Y: "<<src_y<<endl;
		cout<<"X: "<<X<<endl;
		cout<<"X_T: "<<X_t<<endl;
		cout<<"W:"<<W<<endl;
#endif
		W.copyTo(dst);
		//dst = W.clone();
	}
	__CV_END__;
}

void processLanes(CvSeq* lines, IplImage* temp_frame, int frameIndex) {

	// classify lines to left/right side
	std::vector<Lane> left, right;
	std::vector<Lane> tempLane;
	std::vector<Lane> resultLane;
	std::string urlpretreatment = "D:\\file\\fram\\excute\\pretreatment\\temp_frame";
	std::string urlafter = "D:\\file\\fram\\excute\\urlafter\\temp_frame";
	std::string url = "D:\\file\\fram\\excute\\url\\temp_frame";
	urlpretreatment.append(std::to_string(frameIndex) + ".png");
	urlafter.append(std::to_string(frameIndex) + ".png");
	if(frameIndex == 10){
		int a  =10;
	}
	cvSaveImage(urlpretreatment.c_str(),temp_frame);
	float *angles;

	// TODO chi lay 7 diem gan nhau

	for(int i = 0; i < lines->total ; i++ )
    {
		// Xử lý từng line
        CvPoint* line = (CvPoint*)cvGetSeqElem(lines,i); // Điểm
		int dx = line[1].x - line[0].x;
		int dy = line[1].y - line[0].y;
		float angle = atan2f(dy, dx) * 180/CV_PI;

		if (fabs(angle) <= LINE_REJECT_DEGREES || (fabs(angle) <= 90 && fabs(angle) >= 83)) { // reject near horizontal lines
			continue;
		}

		

		// assume that vanishing point is close to the image horizontal center
		// calculate line parameters: y = kx + b;
		dx = (dx == 0) ? 1 : dx; // prevent DIV/0!  
		float k = dy/(float)dx;
		float b = line[0].y - k*line[0].x;


		// assign lane's side based by its midpoint position 
		int midx = (line[0].x + line[1].x) / 2;
		if (midx < temp_frame->width/2+20) {
			left.push_back(Lane(line[0], line[1], angle, k, b));
			

		} else if (midx > temp_frame->width/2) {
			right.push_back(Lane(line[0], line[1], angle, k, b));	
		}
    }
	if(left.size() > 0){
		for(int i = 0; i< left.size() - 1; i++){
			double a;
			for(int j = i; j < left.size(); j++){
				if(left[j].angle > left[i].angle){
					Lane temp  = left[i];
					left[i] = left[j];
					left[j] = temp;
				}
			}
		}

		int index = 0;
	double rang = 0;

	int index1 = 0;
	double rang1 = 0;
	

	//float median = medianAngles(angles, left.size());
	float median = left[(int) left.size()/2].angle;
	for	(int i=0; i<left.size(); i++) {

		if((abs(left[i].angle - median)) < 30){
			if(rang1 < abs((abs(left[i].angle) - median))){
				tempLane.push_back(left[i]);
			}	
		}
	}

	// Xap xep tu lop toi nho theo do dai
	for	(int i=0; i<tempLane.size() - 1; i++) {
		for(int j = i; j < tempLane.size(); j++){
			if(util::distanceBetweenTwoCVPoint(tempLane[i].p0, tempLane[i].p1) < util::distanceBetweenTwoCVPoint(tempLane[j].p0, tempLane[j].p1)){
				Lane temp  = tempLane[i];
				tempLane[i] = tempLane[j];
				tempLane[j] = temp;
			}
		}
	}
	
	int countnear = 0;
	int indexnearmost = 0;
	for(int i =0; i< 3; i++){
		int temp = 0;
		for(int j =0; j< tempLane.size(); j++){
			if(util::distancePointToLine(tempLane[i].p0, tempLane[i].p1, tempLane[j].p0) < 30 
				|| util::distancePointToLine(tempLane[i].p0, tempLane[i].p1, tempLane[j].p1) < 30){
					temp ++;
			}
		}
		if(countnear < temp)
		{
			countnear = temp;
			indexnearmost = i;
		}
	}
	


	
	//cvLine(temp_frame, tempLane[indexnearmost].p0, tempLane[indexnearmost].p1, CV_RGB(255, 0, 0), 2);
	// indexnearmost ==> index cua line chuan
	for(int i = 0; i < tempLane.size(); i++){
		int j = 0;
		if(util::distancePointToLine(tempLane[indexnearmost].p0, tempLane[indexnearmost].p1, tempLane[i].p0) < 30 
				&& util::distancePointToLine(tempLane[indexnearmost].p0, tempLane[indexnearmost].p1, tempLane[i].p1) < 30){
			//cvLine(temp_frame, tempLane[i].p0, tempLane[i].p1, CV_RGB(255, 0, 0), 2);
					resultLane.push_back(tempLane[i]);

		}

	}
	
	
	for(int i = 0 ; i < resultLane.size(); i++){
		xLines.push_back(resultLane[i].p0.x);
		xLines.push_back(resultLane[i].p1.x);
		yLines.push_back(resultLane[i].p0.y);
		yLines.push_back(resultLane[i].p1.y);
	}

	cv::Mat srcx = Mat::Mat(xLines);
	cv::Mat srcy = Mat::Mat(yLines);
	cv::Mat dist = Mat::Mat(2, 1, CV_32FC1);

	polyfit(srcy, srcx, dist, 1);

	float* pData=(float*)dist.data;
	float value = pData[0];
	float value2 = pData[1];

	float *pSrcy = (float*)srcy.data;
	float *pSrcx = (float*)srcx.data;

	float y = pSrcy[0];
	float y1 = pSrcy[1];

	float xA = pSrcx[0];
	float xA1 = pSrcx[1];

	float abx = value2* y + value;
	float abx1 = value2* y1 + value;

	CvPoint P1;
	P1.x = abx;
	P1.y = y;

	CvPoint P2;
	P2.x = abx1;
	P2.y = y1;

	cvLine(temp_frame, P1, P2, CV_RGB(255, 0, 0), 2);
	//cvLine(temp_frame, P1, P2, CV_RGB(0, 0, 255), 2);
	cvSaveImage(urlafter.c_str(),temp_frame);
	}
	

	
}


int main(void)
{

	//VideoCapture videoCapture("D:\\file\\road.avi");
#ifdef USE_VIDEO
	CvCapture *input_video = cvCreateFileCapture("D:\\file\\05.avi");
#else
	CvCapture *input_video = cvCaptureFromCAM(0);
#endif

	if (input_video == NULL) {
		fprintf(stderr, "LOG: Error: Can't open video \n");
		system("PAUSE");
		return -1;

	}

	CvFont font;
	cvInitFont( &font, CV_FONT_VECTOR0, 0.25f, 0.25f);

	/*
	*	get size video
	*/
	CvSize video_size;
	video_size.height = (int) cvGetCaptureProperty(input_video, CV_CAP_PROP_FRAME_HEIGHT);
	video_size.width = (int) cvGetCaptureProperty(input_video, CV_CAP_PROP_FRAME_WIDTH);

	long current_frame = 0;
	int key_pressed = 0;
	IplImage *frame = NULL;

	

	//	Set Fram size
	//	size yêu cầu: 1/4 khung nhìn
	CvSize frame_size = cvSize(video_size.width, video_size.height/4);

	//	Create image
	IplImage *temp_frame = cvCreateImage(frame_size, IPL_DEPTH_8U, 3);
	IplImage *grey = cvCreateImage(frame_size, IPL_DEPTH_8U, 1);
	IplImage *edges = cvCreateImage(frame_size, IPL_DEPTH_8U, 1);
	//IplImage *half_frame = cvCreateImage(cvSize(video_size.width/2, video_size.height/2), IPL_DEPTH_8U, 3);

	CvMemStorage* houghStorage = cvCreateMemStorage(0);

	//	Nhận diện đồ vật
	//CvMemStorage* haarStorage = cvCreateMemStorage(0);
	//CvHaarClassifierCascade* cascade = (CvHaarClassifierCascade*)cvLoad("haar/cars3.xml");

	//cvSetCaptureProperty(input_video, CV_CAP_PROP_POS_FRAMES, current_frame)
	int i =0;
	std::string url = "D:\\file\\fram\\video5\\frame";
	while(key_pressed != 27) {

		frame = cvQueryFrame(input_video);


		if (frame == NULL) {
			fprintf(stderr, "Error: null frame received\n");
			return -1;
		}
		
		std::string temp = url;
		temp.append(std::to_string(i) + ".png");
		
		//GaussianBlur(temp, temp, (7, 7), 2);

		cvSaveImage(temp.c_str(),frame);
		i ++;
		util::cropImage(frame, temp_frame, cvRect(0,video_size.height-frame_size.height,frame_size.width,frame_size.height));

		//cvSaveImage("D:\\file\\fram\\fram4.png",temp_frame);
		cvCvtColor(temp_frame, grey, CV_BGR2GRAY); // convert to grayscale

		// TODO: 
		// Perform a Gaussian blur ( Convolving with 5 X 5 Gaussian) & detect edges
		cvSmooth(grey, grey, CV_GAUSSIAN, 7, 7, 3);
		//cv::fastNlMeansDenoising(grey.clone(), img1, 3.0, 7, 21);
		

		/*Mat tempMat =cv::cvarrToMat(grey);
		cv::fastNlMeansDenoising(tempMat, tempMat);

		grey = cvCloneImage(&(IplImage)tempMat);*/
		cvCanny(grey, edges, CANNY_MAX_TRESHOLD*0.25, CANNY_MAX_TRESHOLD * 0.5);


		// do Hough transform to find lanes
		double rho = 1;
		double theta = CV_PI/180;

		// TODO using 
		// HOUGH_MIN_LINE_LENGTH:50  video4 
		// HOUGH_MIN_LINE_LENGTH: 5 video 5

		CvSeq* lines = cvHoughLines2(edges, houghStorage, CV_HOUGH_PROBABILISTIC, 
			rho, theta, HOUGH_TRESHOLD, 5, HOUGH_MAX_LINE_GAP);
		
		processLanes(lines, temp_frame, i);

		cvLine(temp_frame, cvPoint(frame_size.width/2,0), 
			cvPoint(frame_size.width/2,frame_size.height), CV_RGB(255, 255, 0), 1);

		cvShowImage("Grey", grey);
		cvShowImage("Edges", edges);
		//cvShowImage("Color", temp_frame);
		cvSaveImage(temp.c_str(),edges);
		//	Set location screen
		cvMoveWindow("Grey", 0, 0); 
		cvMoveWindow("Edges", 0, 2*(frame_size.height+25));
		cvMoveWindow("Color", 0, frame_size.height+25); 

		cvShowImage("Color", temp_frame);
		key_pressed = cvWaitKey(5);
	}

	cvReleaseMemStorage(&houghStorage);

	cvReleaseImage(&grey);
	cvReleaseImage(&edges);
	cvReleaseImage(&temp_frame);

	cvReleaseCapture(&input_video);
}