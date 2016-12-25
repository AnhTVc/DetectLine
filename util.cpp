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
util::util(void)
{	
}
util::~util(void)
{
}

/********************************************************************************/
/*	Author: Viet Anh															*/
/*	Method squareDouble															*/
/*	parameter: double a															*/
/*	return: a*a																	*/
/********************************************************************************/
double util::squareDouble(double a){
	return a * a;
}

/********************************************************************************/
/*	Author: Viet Anh															*/
/*	Method compare																*/
/*	parameter: void *a, *b														*/
/*	return: a*a																	*/
/********************************************************************************/
int util::compare(const void *a, const void *b){
	float fa = *(float *) a;
    float fb = *(float *) b;
    return (fa > fb) - (fa < fb);
}


/********************************************************************************/
/*	Author: Viet Anh															*/
/*	Method medianAngles															*/
/*	parameter: float *angles, int n												*/
/*	return: a*a																	*/
/********************************************************************************/
float util::medianAngles(float *angles, int n){
	float result;
	qsort(angles, n, sizeof(float), util::compare);
	if(n%2 == 0)
	{
		 return (angles[n / 2] + angles[n / 2 - 1]) / 2;
	}else
		 return angles[n / 2];

}

/********************************************************************************/
/*	Author: Viet Anh															*/
/*	Method distanceBetweenTwoPoint												*/
/*	Describe:	Tính khoảng các giữa 2 điểm										*/
/*	parameter: cv::Point line_start, cv::Point line_end							*/
/*	return: 																	*/
/********************************************************************************/
double util::distanceBetweenTwoPoint(cv::Point line_start, cv::Point line_end){
	return sqrt(util::squareDouble(line_end.x - line_start.x) + util::squareDouble(line_end.y - line_start.y));;
}

/********************************************************************************/
/*	Author: Viet Anh															*/
/*	Method distanceBetweenTwoCVPoint												*/
/*	Describe:	Tính khoảng các giữa 2 điểm										*/
/*	parameter: cv::Point line_start, cv::Point line_end							*/
/*	return: 																	*/
/********************************************************************************/
double util::distanceBetweenTwoCVPoint(CvPoint &p, CvPoint &p1){
	return sqrt(pow(p1.x - p.x,2) +  pow(p1.y - p.y, 2));
}

/********************************************************************************/
/*	Author: Viet Anh															*/
/*	Method distancePointToLine													*/
/*	Describe:	Tính khoảng cách giữa 1 điểm tới 1 đường thẳng					*/
/*	parameter:																	*/
/*	return: 																	*/
/********************************************************************************/
double util::distancePointToLine(cv::Point line_start, cv::Point line_end, cv::Point point){
	// y = ax + b
	// y' = -ax' + b'
	float a = (line_end.y - line_start.y)/(line_end.x - line_start.x);
	float b = line_start.y - a * line_start.x;

	int b1 = point.y + a * point.x;
	cv::Point temp;
	temp.x = (b1 - b)/ (2*a);
	temp.y = temp.x * a + b;
	return util::distanceBetweenTwoPoint(point, temp);;
}

/********************************************************************************/
/*	Author: Viet Anh															*/
/*	Method cropImage															*/
/*	Describe:	Crop image in OpenCV											*/
/*	parameter:	src: image source												*/
/*				dest:image taget												*/
/*				rect: size of crop												*/
/*	return: 																	*/
/********************************************************************************/
void util::cropImage(IplImage* src,  IplImage* dest, CvRect rect){
	cvSetImageROI(src, rect); 
    cvCopy(src, dest); 
    cvResetImageROI(src);
	
}


/********************************************************************************/
/*	Author: Viet Anh															*/
/*	Method polyfit																*/
/*	Describe:	polyfit as matlab												*/
/*	parameter:	cv::Mat &src_x, cv::Mat &src_y, cv::Mat &dst, int order			*/
/*	return: 																	*/
/********************************************************************************/
