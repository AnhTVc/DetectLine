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

#define USE_VIDEO 1
string URL_OUT_PUT = "D:\\file\\result\\Contestant.txt";
char *URL_OUTPUT_VIDEO = "D:\\file\\result\\BK Pro_Video.avi";
char *URL_INPUT_VIDEO = "D:\\file\\clip5_BKHN.mp4";
string data_result = "";

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

struct CvPointResult{
	CvPointResult(){}
	CvPointResult(CvPoint left, CvPoint right): leftP(left),rightP(right){ }
	
	CvPoint leftP, rightP;
};

cv::KalmanFilter KFCenter, kalmainFilterLeft, kalmainFilterRight;
cv::Mat_<float> measurement(2,1); 

Mat_<float> state(4, 1); // (x, y, Vx, Vy)
enum{
	LINE_REJECT_DEGREES = 10, // in degrees
	CANNY_MIN_TRESHOLD = 60, // edge detector minimum hysteresis threshold
	CANNY_MAX_TRESHOLD = 80, // edge detector maximum hysteresis threshold
	HOUGH_TRESHOLD = 15,		// line approval vote threshold
	HOUGH_MIN_LINE_LENGTH = 45,	// remove lines shorter than this treshold
	HOUGH_MAX_LINE_GAP = 50,   // join lines to one with smaller than this gaps
	DISTANCE_MIN_BEFOR = 50,	// ĐỘ lệch cho phép mà tâm đường lệnh so với trước đó
	MAX_CAR_MOVE = 5,			// Độ dịch chuyển cho phép của tâm đường 5 px;
	NUMBER_LINE_IN_FRAME = 30,
};

double calPointDistance(CvPoint p0, CvPoint p1) {
	return sqrt(pow(p0.x - p1.x, 2) + pow(p0.y - p1.y, 2));
}
KalmanFilter initKalman(float x, float y, KalmanFilter kf)
{
    // Instantate Kalman Filter with
    // 4 dynamic parameters and 2 measurement parameters,
    // where my measurement is: 2D location of object,
    // and dynamic is: 2D location and 2D velocity.
    kf.init(4, 2, 0);

    measurement = Mat_<float>::zeros(2,1);
    measurement.at<float>(0, 0) = x;
    measurement.at<float>(0, 0) = y;


    kf.statePre.setTo(0);
    kf.statePre.at<float>(0, 0) = x;
    kf.statePre.at<float>(1, 0) = y;

    kf.statePost.setTo(0);
    kf.statePost.at<float>(0, 0) = x;
    kf.statePost.at<float>(1, 0) = y; 

    setIdentity(kf.transitionMatrix);
    setIdentity(kf.measurementMatrix);
    setIdentity(kf.processNoiseCov, Scalar::all(.005)); //adjust this for faster convergence - but higher noise
    setIdentity(kf.measurementNoiseCov, Scalar::all(1e-1));
    setIdentity(kf.errorCovPost, Scalar::all(.1));
	return kf;
}
Point kalmanPredict(KalmanFilter kf) 
{
    Mat prediction = kf.predict();
    Point predictPt(prediction.at<float>(0),prediction.at<float>(1));
    return predictPt;
}
KalmanFilter kalmanCorrect(float x, float y, KalmanFilter kf)
{
    measurement(0) = x;
    measurement(1) = y;
    Mat estimated = kf.correct(measurement);
    Point statePt(estimated.at<float>(0),estimated.at<float>(1));
    return kf;
}
double calLineDistance(Lane line) {
	return sqrt(pow(line.p0.x - line.p1.x, 2) + pow(line.p0.y - line.p1.y, 2));
}
CvPointResult processLanes(CvSeq* lines, IplImage* temp_frame, int frameIndex) {
	CvPointResult cvPointResult;
	// classify lines to left/right side
	std::vector<Lane> left, right;
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

			double a1 = left_lines[0]/ left_lines[1];
			double b1 = leftP1.x - a1 * leftP1.y;

			double XResult = a1 * temp_frame->height/2 + b1; 

			CvPoint resultL; resultL.x = XResult; resultL.y = temp_frame->height/2;

			//CvPoint leftResult; leftResult.x = (leftP0.x + leftP1.x)/2; leftResult.y = (leftP0.y + leftP1.y)/2;

			cvPointResult.leftP = resultL;
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

			double a1 = right_lines[0]/ right_lines[1];
			double b1 = rightP1.x - a1 * rightP1.y;

			double XResult = a1 * temp_frame->height/2 + b1; 

			CvPoint resultR; resultR.x = XResult; resultR.y = temp_frame->height/2;
			//CvPoint rightResult; rightResult.x = (rightP0.x + rightP1.x)/2;rightResult.y = (rightP0.y + rightP1.y)/2;
			cvPointResult.rightP = resultR;

			cvLine(temp_frame, rightP0, rightP1, CV_RGB(0, 0, 255), 2);
		}
		
		// Chia 1/4 khung hinh
		CvPoint lineP0, lineP1;
		lineP0.x = 0; lineP0.y = 0; 
		lineP1.x =  temp_frame->width;lineP1.y = 0;
		cvLine(temp_frame, lineP0, lineP1, CV_RGB(255, 0, 255), 2);
		return cvPointResult;
	}
}


int main(int argc, char * argv[])
{
	
	//VideoCapture videoCapture("D:\\file\\road.avi");
#ifdef USE_VIDEO
	CvCapture *input_video = cvCreateFileCapture(URL_INPUT_VIDEO);
#else
	CvCapture *input_video = cvCaptureFromCAM(0);
#endif
	if (input_video == NULL) {
		fprintf(stderr, "LOG: Error: Can't open video \n");
		system("PAUSE");
		return -1;

	}

	CvSize video_size;
	video_size.height = (int) cvGetCaptureProperty(input_video, CV_CAP_PROP_FRAME_HEIGHT);
	video_size.width = (int) cvGetCaptureProperty(input_video, CV_CAP_PROP_FRAME_WIDTH);

	long current_frame = 0;
	int key_pressed = 0;
	IplImage *frame = NULL;
	CvPoint pointLeftPrior, pointRightPrior, pointCenterPrior;		// Lưu các point trước và sau.
	

	//	Set Fram size
	//	size yêu cầu: 1/4 khung nhìn
	CvSize frame_size = cvSize(video_size.width, video_size.height/4);

    double fps = cvGetCaptureProperty(input_video, CV_CAP_PROP_FPS);    
	CvVideoWriter* outputVideo = NULL;
    outputVideo = cvCreateVideoWriter(URL_OUTPUT_VIDEO, CV_FOURCC('M','J','P','G'), fps, cvSize((int)video_size.width,(int)video_size.height), 1);
    if (outputVideo == NULL)
    {
        printf("!!! ERROR: cvCreateVideoWriter\n");
        return -1;
    }

	//	Create image
	//IplImage *current_frame = cvCreateImage(cvSize(video_size.width, video_size.height), IPL_DEPTH_8U, 3);
	IplImage *temp_frame = cvCreateImage(frame_size, IPL_DEPTH_8U, 3);
	IplImage *grey = cvCreateImage(frame_size, IPL_DEPTH_8U, 1);
	IplImage *edges = cvCreateImage(frame_size, IPL_DEPTH_8U, 1);
	IplImage *gauss_grey = cvCreateImage(frame_size, IPL_DEPTH_8U, 1);

	
	//IplImage *half_frame = cvCreateImage(cvSize(video_size.width/2, video_size.height/2), IPL_DEPTH_8U, 3);

	CvMemStorage* houghStorage = cvCreateMemStorage(0);
	int i =0;
	// load output from fpt
	//vector<CvPoint> output = util::readOutputClipFPT("D:\\file\\Output\\output_Clip5.txt");
	 for( ; ; ){

		frame = cvQueryFrame(input_video);
		if (frame == NULL) {
			fprintf(stderr, "Error: null frame received\n");
			break;
		}
		
		util::cropImage(frame, temp_frame, cvRect(0,video_size.height-frame_size.height,frame_size.width,frame_size.height));

		// CHUYỂN TỪ ẢNH MÀU THÀNH ẢNH ĐEN TRẮNG
		cvCvtColor(temp_frame, grey, CV_BGR2GRAY);
		// LỌC TRUNG VỊ ĐỂ KHỬ NHIỄU
		cvSmooth(grey, grey, CV_MEDIAN, 3, 3);
		// LỌC GAUSS LÀM TRƠN ẢNH
		cvSmooth(grey, gauss_grey, CV_GAUSSIAN, 13, 13, 5);
		// TÌM BIÊN BẰNG PHƯƠNG PHÁP CANNY
		cvCanny(gauss_grey, edges, CANNY_MIN_TRESHOLD, CANNY_MAX_TRESHOLD);

		double rho = 1;
		double theta = CV_PI/180;

		CvSeq* lines = cvHoughLines2(edges, houghStorage, CV_HOUGH_PROBABILISTIC, 
			rho, theta, HOUGH_TRESHOLD, HOUGH_MIN_LINE_LENGTH, HOUGH_MAX_LINE_GAP);
		
		if (lines->total < NUMBER_LINE_IN_FRAME) {
			cvSmooth(grey, gauss_grey, CV_GAUSSIAN, 3, 3, 2);
			cvCanny(gauss_grey, edges, CANNY_MIN_TRESHOLD * 0.8, CANNY_MAX_TRESHOLD * 0.8);
			houghStorage = cvCreateMemStorage(0);
			lines = cvHoughLines2(edges, houghStorage, CV_HOUGH_PROBABILISTIC,
				rho, theta, HOUGH_TRESHOLD, HOUGH_MIN_LINE_LENGTH, HOUGH_MAX_LINE_GAP/3);
		}
		// XỬ LÝ VỚI ĐOẠN THẲNG
		CvPointResult cvPointResult = processLanes(lines, temp_frame,i);
		
		CvPoint leftResult = cvPointResult.leftP;
		CvPoint rightResult = cvPointResult.rightP;
		
		// XÁC ĐỊNH TÂM ĐƯỜNG
		CvPoint centerLineResult = CvPoint((leftResult.x + rightResult.x)/2, (leftResult.y + rightResult.y)/2);

		// KALMAIN CHO TÂM ĐƯỜNG
		if(i == 0){
			// INIT KALMAN
			KFCenter = initKalman(centerLineResult.x, centerLineResult.y, KFCenter); KFCenter = kalmanCorrect(centerLineResult.x, centerLineResult.y, KFCenter);
			kalmainFilterLeft = initKalman(leftResult.x, leftResult.y, kalmainFilterLeft); kalmainFilterLeft = kalmanCorrect(leftResult.x, leftResult.y, kalmainFilterLeft);
			kalmainFilterRight = initKalman(rightResult.x, rightResult.y, kalmainFilterRight); kalmainFilterRight = kalmanCorrect(rightResult.x, rightResult.y, kalmainFilterRight);

			// SAVE POINT
			pointLeftPrior = leftResult;
			pointRightPrior = rightResult;
			pointCenterPrior = centerLineResult;
		}else{
			// NẾU MÀ VỊ TRÍ TRÁI PHẢI LỆCH QUÁ NHIỀU SO VỚI ĐIỂM TRƯỚC ĐÓ ==> QUÁ TRÌNH SỬ LÝ LỖI => LẤY ĐIỂM KALMAIN
			if(calPointDistance(centerLineResult, pointCenterPrior) > DISTANCE_MIN_BEFOR){
				CvPoint tempP = kalmanPredict(KFCenter);
				if(tempP.x > pointCenterPrior.x){
					// TO RIGHT
					centerLineResult.x = pointCenterPrior.x + MAX_CAR_MOVE;
				}else{
					// TO LEFT
					centerLineResult.x = pointCenterPrior.x - MAX_CAR_MOVE;
				}

				centerLineResult.y = tempP.y;
			}else{
				 if(centerLineResult.x > pointCenterPrior.x){
					// TO RIGHT
					centerLineResult.x = pointCenterPrior.x + MAX_CAR_MOVE;
				}else{
					// TO LEFT
					centerLineResult.x = pointCenterPrior.x - MAX_CAR_MOVE;
				}
			}

			// SAVE POINT
			kalmainFilterLeft = kalmanCorrect(centerLineResult.x, centerLineResult.y, kalmainFilterLeft);
			kalmainFilterRight = kalmanCorrect(centerLineResult.x, centerLineResult.y, kalmainFilterRight);
			KFCenter = kalmanCorrect(centerLineResult.x, centerLineResult.y, KFCenter);
			pointLeftPrior = leftResult;
			pointRightPrior = rightResult;
			pointCenterPrior = centerLineResult;
		}
		
					
		// SAVE TO FILE OUT PUT
		data_result = std::to_string(i);
		data_result.append(" ");
		data_result.append(std::to_string(centerLineResult.x));
		data_result.append(" ");
		data_result.append(std::to_string(temp_frame->height * 3 + centerLineResult.y));
		data_result.append("\n");

		std::string abc = util::readFile(URL_OUT_PUT);
		if(abc.empty()){
			// frame dau tien
			util::writeFile(std::to_string( i + 1).append("\n"), URL_OUT_PUT);
			util::writeFile(data_result, URL_OUT_PUT);
		}else{
			// thay the
			int index_temp = abc.find_first_of("\n");
			string temp_out = abc.substr(index_temp, abc.length());
			string final_out = std::to_string( i + 1).append(temp_out);
			
			final_out.append(data_result);

			util::overwriteFile(final_out, URL_OUT_PUT);
		}

		//circle(cv::cvarrToMat(temp_frame), leftResult, temp_frame->width/64.0, CV_RGB(255, 0, 0));
		//circle(cv::cvarrToMat(temp_frame), rightResult, temp_frame->width/64.0, CV_RGB(0, 0, 255));
		circle(cv::cvarrToMat(temp_frame), centerLineResult, temp_frame->width/64.0, CV_RGB(255, 255, 0));
		//circle(cv::cvarrToMat(temp_frame), output[i], temp_frame->width/64.0, CV_RGB(169, 7, 142));
		
		cvShowImage("Grey", grey);
		cvShowImage("Edges", edges);
		//	Set location screen
		cvMoveWindow("Grey", 0, 0); 
		cvMoveWindow("Edges", 0, 2*(frame_size.height+25));
		cvMoveWindow("Color", 0, frame_size.height+25); 
		cvShowImage("Color", temp_frame);

		util::copyImage(temp_frame, frame);
		cvWriteFrame(outputVideo, frame);
		waitKey(5);
		i ++;
	}

	// free mem
	cvReleaseMemStorage(&houghStorage);
	cvReleaseVideoWriter(&outputVideo);
	cvReleaseImage(&grey);
	cvReleaseImage(&edges);
	cvReleaseImage(&temp_frame);

	cvReleaseCapture(&input_video);

}