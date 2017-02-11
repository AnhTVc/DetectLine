#pragma once
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
class util
{
public:
	util(void);
	~util(void);
	static double squareDouble(double a);																//	Tính bình phương 1 số bất kỳ
	static int compare(const void *a, const void *b);													//	Compare between
	static float medianAngles(float *angles, int n);													//	Median Angles
	static double distanceBetweenTwoPoint(cv::Point line_start, cv::Point line_end);					//	Tính khoảng cách giữa 2 điểm
	static double distanceBetweenTwoCVPoint(CvPoint &line_start, CvPoint &line_end);					//	Tính khoảng cách giữa 2 điểm
	static double distancePointToLine(cv::Point line_start, cv::Point line_end, cv::Point point);		//	Tính khoảng cách từ điểm tới đường thẳng
	static void cropImage(IplImage* src,  IplImage* dest, CvRect rect);									//	Crop imagesegment
	//static void polyfit(cv::Mat &src_x, cv::Mat &src_y, cv::Mat &dst, int order);
	static void writeFile(string data);
	static void writeFile(string data, string file_path);
	static void overwriteFile(string data, string file_path);
	static std::string readFile(string file_path);
	static vector<CvPoint> readOutputClipFPT(string url);
	static void copyImage(IplImage* small_image, IplImage* big_image);								//	Crop imagesegment
};

