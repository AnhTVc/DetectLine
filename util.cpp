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

void util::copyImage(IplImage* small_image, IplImage* big_image){
	cv::Mat small = cv::cvarrToMat(small_image);
	cv::Mat big = cv::cvarrToMat(big_image);
	small.copyTo(big(cv::Rect(0, big_image -> height * 3/4,small.cols, small.rows)));
}
/********************************************************************************/
/*	Author: Viet Anh															*/
/*	Method polyfit																*/
/*	Describe:	polyfit as matlab												*/
/*	parameter:	cv::Mat &src_x, cv::Mat &src_y, cv::Mat &dst, int order			*/
/*	return: 																	*/
/********************************************************************************/
void util::writeFile(string data){
	ofstream myfile;
	myfile.open ("result.txt", fstream::app);
	myfile << data;
	myfile.close();
  
}

void util::writeFile(string data, string file_path){
	ofstream myfile;
	myfile.open (file_path, fstream::app);
	myfile << data;
	myfile.close();
}
string util::readFile(string file_path){
	std::ifstream t(file_path);
	std::string str((std::istreambuf_iterator<char>(t)),
					 std::istreambuf_iterator<char>());

	return str;
}
void util::overwriteFile(string data, string file_path){
	ofstream myfile;
	myfile.open (file_path);
	myfile << data;
	myfile.close();
}
vector<string> splitMy(string str, char delimiter) {
  vector<string> internal;
  stringstream ss(str); // Turn the string into a stream.
  string tok;
  
  while(getline(ss, tok, delimiter)) {
    internal.push_back(tok);
  }
  
  return internal;
}
vector<CvPoint> util::readOutputClipFPT(string url){
	string line;
	ifstream file (url);
	vector<CvPoint> result;
	CvPoint temp;
	int size = 0;
	if(file.is_open()){
		while (getline(file, line))
		{
			if(size == 0){
				size = atoi(line.c_str());
			}else{
				
				vector<string> a = splitMy(line, ' ');
				temp.x = atoi(a[1].c_str());
				temp.y = atoi(a[2].c_str()) - 144 * 3;
				result.push_back(temp);
			}
			
		}
		
	}

	return result;


}
