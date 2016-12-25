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
	HOUGH_MAX_LINE_GAP = 15,   // join lines to one with smaller than this gaps
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
double calLineDistance(Lane line) {
	return sqrt(pow(line.p0.x - line.p1.x, 2) + pow(line.p0.y - line.p1.y, 2));
}
void processLanesImage(CvSeq* lines, IplImage* edges, IplImage* temp_frame, int frameIndex) {

	// classify lines to left/right side
	std::vector<Lane> left, right;
	std::string urlpretreatment = "D:\\file\\fram\\excute\\pretreatment\\temp_frame";
	//std::string urlafter = "D:\\file\\fram\\excute\\urlafter\\temp_frame";
	std::string urlafter = "D:\\file\\fram\\excute\\temp_frame";
	std::string url = "D:\\file\\fram\\excute\\url\\temp_frame";
	urlpretreatment.append(std::to_string(frameIndex) + ".png");
	urlafter.append(std::to_string(frameIndex) + ".png");
	cvSaveImage(urlpretreatment.c_str(),temp_frame);
	if(frameIndex == 22){
		int a = 100;
	}
	if(lines->total > 0){
		for(int i = 0; i < lines->total; i++ )
		{
			CvPoint* line = (CvPoint*)cvGetSeqElem(lines,i);
			int dx = line[1].x - line[0].x;
			int dy = line[1].y - line[0].y;
			float angle = atan2f(dy, dx) * 180/CV_PI;

			if (abs(angle) < 10 || abs(angle) > 83) { // reject near horizontal lines
				continue;
			}

			// assume that vanishing point is close to the image horizontal center
			// calculate line parameters: y = kx + b;
			dx = (dx == 0) ? 1 : dx; // prevent DIV/0!  
			float k = dy/(float)dx;
			float b = line[0].y - k*line[0].x;

			// assign lane's side based by its midpoint position 
			int midx = (line[0].x + line[1].x) / 2;
			if (midx < temp_frame->width/2) {
				left.push_back(Lane(line[0], line[1], angle, k, b));
			} else if (midx > temp_frame->width/2) {
				right.push_back(Lane(line[0], line[1], angle, k, b));
			}
		}
	
		std::vector<Lane> s_angle_left;
		Lane temp;
		// SẮP XẾP ĐOẠN THẲNG THEO THEO ĐỘ DÀI GIẢM DẦN
					// VẼ ĐƯỜNG THẲNG
		double m = temp_frame->height>temp_frame->width?temp_frame->height:temp_frame->width;
		int MAX_LINE = 15;
		double medianAngle;
		double a, b;
		if(left.size() > 0){
			for (int i = 0; i < left.size()-1; i++) 
			{
				for (int j = i; j < left.size(); j++) 
				if (calLineDistance(left[i]) < calLineDistance(left[j])) {
					temp = left[i];
					left[i] = left[j];
					left[j] = temp;
				}
			}
			// TÌM GÓC TRUNG VỊ CỦA 15 ĐOẠN DÀI NHẤT (NẾU SỐ ĐOẠN TÌM ĐƯỢC ÍT HƠN 15 THÌ TÍNH GÓC TRUNG VỊ CỦA TẤT CẢ ĐOẠN TÌM ĐƯỢC)
			for (int i = 0; i < (MAX_LINE < left.size() ? MAX_LINE : left.size()) - 1; i++)
				for (int j = i; j < (MAX_LINE < left.size() ? MAX_LINE : left.size()); j++)
					if (left[i].angle < left[j].angle) {
						temp = left[i];
						left[i] = left[j];
						left[j] = temp;
					}
			// GÓC TRUNG VỊ
			medianAngle = left[(MAX_LINE < left.size() ? MAX_LINE : left.size())/2].angle;
			// LỌC CÁC ĐOẠN LỆCH VỚI GÓC TRUNG VỊ NHỎ HƠN 30 ĐỘ
			for (int i = 0; i < left.size(); i++) {
				double angle = left[i].angle;
				if (abs(left[i].angle - medianAngle) < 30)
					s_angle_left.push_back(left[i]);
			}
			// SẮP XẾP CÁC ĐOẠN CÒN LẠI THEO ĐỘ DÀI ĐOẠN
			for (int i = 0; i < s_angle_left.size() - 1; i++)
				for (int j = i; j < s_angle_left.size(); j++)
					if (calLineDistance(s_angle_left[i]) < calLineDistance(s_angle_left[j])) {
						temp = s_angle_left[i];
						s_angle_left[i] = s_angle_left[j];
						s_angle_left[j] = temp;
					}
			// TÌM CÁC ĐOẠN LỆCH SO VỚI ĐOẠN DÀI NHẤT ÍT HƠN 30 PIXEL SO VỚI PHƯƠNG X
			std::vector<Lane> remain_left;
			a = tan(atan2f((s_angle_left[0].p0.x - s_angle_left[0].p1.x),(s_angle_left[0].p0.y - s_angle_left[0].p1.y)));
			b = s_angle_left[0].p0.x - a*s_angle_left[0].p0.y;
			for (int i = 0; i < s_angle_left.size(); i++) {
				double x_long = a*s_angle_left[i].p0.y +b;
				if (abs(x_long - s_angle_left[i].p0.x) < 30)
					remain_left.push_back(s_angle_left[i]);
			}
			// 
			CvPoint *left_point = new CvPoint[2 * remain_left.size()];
			for (int i = 0; i < remain_left.size(); i++) {
				left_point[2 * i] = remain_left[i].p0;
				left_point[2 * i + 1] = remain_left[i].p1;
			}
			CvMat leftMat = cvMat(1, 2 * remain_left.size(), CV_32SC2, left_point);

			float left_lines[4];
			cvFitLine(&leftMat, CV_DIST_L2, 0, 0.01, 0.01, left_lines);
	
			CvPoint leftP0; leftP0.x = left_lines[2] - m*left_lines[0]; leftP0.y = left_lines[3] - m*left_lines[1];
			CvPoint leftP1; leftP1.x = left_lines[2] + m*left_lines[0]; leftP1.y = left_lines[3] + m*left_lines[1];
			cvLine(temp_frame, leftP0, leftP1, CV_RGB(0, 0, 255), 2);

			
		}

		

		if(right.size() > 0){
			// THỰC HIỆN TƯƠNG TỰ VỚI CÁC ĐOẠN BÊN PHẢI
			std::vector<Lane> s_angle_right;
			//SẮP XẾP LON TOI NHO
			for (int i = 0; i < right.size()-1; i++)
				for (int j = i; j < right.size(); j++)
					if (calLineDistance(right[i]) < calLineDistance(right[j])) {
						temp = right[i];
						right[i] = right[j];
						right[j] = temp;
					}


			for (int i = 0; i < (MAX_LINE < right.size() ? MAX_LINE : right.size()) - 1; i++)
				for (int j = i; j < (MAX_LINE < right.size() ? MAX_LINE : right.size()); j++)
					if (right[i].angle < right[j].angle) {
						temp = right[i];
						right[i] = right[j];
						right[j] = temp;
					}
	
			medianAngle = right[(MAX_LINE < right.size() ? MAX_LINE : right.size())/2].angle;

			for (int i = 0; i < right.size(); i++) {
				double angle = right[i].angle;
				if (abs(right[i].angle - medianAngle) < 30)
					s_angle_right.push_back(right[i]);
			}

			for (int i = 0; i < s_angle_right.size() - 1; i++)
				for (int j = i; j < s_angle_right.size(); j++)
					if (calLineDistance(s_angle_right[i]) < calLineDistance(s_angle_right[j])) {
						temp = s_angle_right[i];
						s_angle_right[i] = s_angle_right[j];
						s_angle_right[j] = temp;
					}
	
			std::vector<Lane> remain_right;
			a = tan(atan2f((s_angle_right[0].p0.x - s_angle_right[0].p1.x),(s_angle_right[0].p0.y - s_angle_right[0].p1.y)));
			b = s_angle_right[0].p0.x - a*s_angle_right[0].p0.y;
			for (int i = 0; i < s_angle_right.size(); i++) {
				double x_long = a*s_angle_right[i].p0.y +b;
				if (abs(x_long - s_angle_right[i].p0.x) < 30)
					remain_right.push_back(s_angle_right[i]);
			}
	
			// SỬ DỤNG CvFitLine ĐỂ TÌM ĐƯỜNG THẲNG ĐI QUA CÁC ĐIỂM CHO TRƯỚC
			CvPoint *right_point = new CvPoint[2*remain_right.size()];
			for (int i = 0; i < remain_right.size(); i++) {
				right_point[2 * i] = remain_right[i].p0;
				right_point[2 * i + 1] = remain_right[i].p1;
			}
			CvMat rightMat = cvMat(1, 2 * remain_right.size(), CV_32SC2, right_point);

			float right_lines[4];
			cvFitLine(&rightMat, CV_DIST_L2, 0, 0.01, 0.01, right_lines);


			CvPoint rightP0; rightP0.x = right_lines[2] - m*right_lines[0]; rightP0.y = right_lines[3] - m*right_lines[1];
			CvPoint rightP1; rightP1.x = right_lines[2] + m*right_lines[0]; rightP1.y = right_lines[3] + m*right_lines[1];
			cvLine(temp_frame, rightP0, rightP1, CV_RGB(0, 0, 255), 2);
		}
		
		cvSaveImage(urlafter.c_str(),temp_frame);
	}
}

int main(int argc, char * argv[])
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
	IplImage *gauss_grey = cvCreateImage(frame_size, IPL_DEPTH_8U, 1);
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

		// CHUYỂN TỪ ẢNH MÀU THÀNH ẢNH ĐEN TRẮNG
		cvCvtColor(temp_frame, grey, CV_BGR2GRAY);
		// LỌC TRUNG VỊ ĐỂ KHỬ NHIỄU
		cvSmooth(grey, grey, CV_MEDIAN, 3, 3);
		// LỌC GAUSS LÀM TRƠN ẢNH
		cvSmooth(grey, gauss_grey, CV_GAUSSIAN, 13, 13, 5);
		// TÌM BIÊN BẰNG PHƯƠNG PHÁP CANNY
		cvCanny(gauss_grey, edges, CANNY_MIN_TRESHOLD, CANNY_MAX_TRESHOLD);


		// do Hough transform to find lanes
		double rho = 1;
		double theta = CV_PI/180;

		// TODO using 
		// HOUGH_MIN_LINE_LENGTH:50  video4 
		// HOUGH_MIN_LINE_LENGTH: 5 video 5

		CvSeq* lines = cvHoughLines2(edges, houghStorage, CV_HOUGH_PROBABILISTIC, 
			rho, theta, HOUGH_TRESHOLD, HOUGH_MIN_LINE_LENGTH, HOUGH_MAX_LINE_GAP);
		
		if (lines->total < 30) {
			cvSmooth(grey, gauss_grey, CV_GAUSSIAN, 3, 3, 2);
			cvCanny(gauss_grey, edges, CANNY_MIN_TRESHOLD, CANNY_MAX_TRESHOLD);
			houghStorage = cvCreateMemStorage(0);
			lines = cvHoughLines2(edges, houghStorage, CV_HOUGH_PROBABILISTIC,
				rho, theta, HOUGH_TRESHOLD, HOUGH_MIN_LINE_LENGTH, HOUGH_MAX_LINE_GAP/3);
		}
		// XỬ LÝ VỚI ĐOẠN THẲNG
		processLanesImage(lines, edges, temp_frame,i);

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
	

	// TEST XU LY TUNG ANH
	/* SIMPLE CODE TO PROCESS THE IMAGE */
	/*// XỬ LÝ ẢNH VỚI TỪNG FRAME
	IplImage *frame = cvLoadImage("D:\\file\\fram\\excute\\pretreatment\\temp.png", 1);
	//CvSize frame_size = cvSize(video_size.width, video_size.height/4);
	CvSize frame_size = cvSize(frame->width, frame->height);
	//CvSize frame_size = cvSize(video_size.width, video_size.height/4);
	IplImage *temp_frame = cvLoadImage("D:\\file\\fram\\excute\\pretreatment\\temp.png", 1);
	temp_frame = frame;


	IplImage *grey = cvCreateImage(frame_size, IPL_DEPTH_8U, 1);
	IplImage *gauss_grey = cvCreateImage(frame_size, IPL_DEPTH_8U, 1);
	IplImage *edges = cvCreateImage(frame_size, IPL_DEPTH_8U, 1);

	// CHUYỂN TỪ ẢNH MÀU THÀNH ẢNH ĐEN TRẮNG
	cvCvtColor(temp_frame, grey, CV_BGR2GRAY);
	// LỌC TRUNG VỊ ĐỂ KHỬ NHIỄU
	cvSmooth(grey, grey, CV_MEDIAN, 3, 3);
	// LỌC GAUSS LÀM TRƠN ẢNH
	cvSmooth(grey, gauss_grey, CV_GAUSSIAN, 13, 13, 5);

	// TÌM BIÊN BẰNG PHƯƠNG PHÁP CANNY
	cvCanny(gauss_grey, edges, CANNY_MIN_TRESHOLD, CANNY_MAX_TRESHOLD);
	CvMemStorage* houghStorage = cvCreateMemStorage(0);
	double rho = 1;
	double theta = CV_PI / 180;
	// TÌM ĐƯỜNG THẲNG BẰNG HOUGH TRANSFORM
	CvSeq* lines = cvHoughLines2(edges, houghStorage, CV_HOUGH_PROBABILISTIC,
		rho, theta, HOUGH_TRESHOLD, HOUGH_MIN_LINE_LENGTH, HOUGH_MAX_LINE_GAP);
	// NẾU SỐ ĐOẠN TÌM ĐƯỢC NHỎ HƠN 30, GIẢM LỌC GAUSS ĐỂ TÌM THÊM ĐOẠN THẲNG
	if (lines->total < 30) {
		cvSmooth(grey, gauss_grey, CV_GAUSSIAN, 3, 3, 2);
		cvCanny(gauss_grey, edges, CANNY_MIN_TRESHOLD, CANNY_MAX_TRESHOLD);
		houghStorage = cvCreateMemStorage(0);
		lines = cvHoughLines2(edges, houghStorage, CV_HOUGH_PROBABILISTIC,
			rho, theta, HOUGH_TRESHOLD, HOUGH_MIN_LINE_LENGTH, HOUGH_MAX_LINE_GAP/3);

		if(lines->total < 15){
			// TIEP TUC GIAM NHIEU ANH
			Mat tempMat =cv::cvarrToMat(grey);
			cv::fastNlMeansDenoising(tempMat, tempMat);
			grey = cvCloneImage(&(IplImage)tempMat);


			cvSmooth(grey, gauss_grey, CV_GAUSSIAN, 3, 3, 2);
			cvCanny(gauss_grey, edges, CANNY_MIN_TRESHOLD, CANNY_MAX_TRESHOLD);
			houghStorage = cvCreateMemStorage(0);
			lines = cvHoughLines2(edges, houghStorage, CV_HOUGH_PROBABILISTIC,
				rho, theta, HOUGH_TRESHOLD, HOUGH_MIN_LINE_LENGTH, HOUGH_MAX_LINE_GAP/3);
		}
	}
	// XỬ LÝ VỚI ĐOẠN THẲNG
	processLanesImage(lines, edges, temp_frame, 1000);

	system ("PAUSE");*/

}