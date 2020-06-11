#ifndef __HEAD__
#define __HEAD__

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp> 
#include<ctime>
#include<vector>
#include "putText.h"

using namespace std;
using namespace cv;

class Move {
public:
	Move() {};
	~Move() {};
	Point startPoint;
	Point endPoint;
	int firstFrameCount = 0;
	int secondFrameCount = 0;
	void Init() {
		startPoint = Point(0, 0);
		endPoint = Point(0, 0);
		firstFrameCount = 0;
		secondFrameCount = 0;
	}
};
typedef struct Blob {
	vector<Point> centerPoint;
	vector<Point> bottomRight;
	vector<Point> topLeft;
}Blob1;

Mat getRoI(Mat img, const Point** ppt, int* npt);
Mat getShow(Mat img, const Point** ppt, int* npt);
void setRoIPoint();
void settransPoints();
void createBackgroundImg(const Mat roiImg, const int frame_count);
double getDistance(Point pointO, Point pointA);
double getSpeed(Move& centerPointLeftN);
void getCarNumberAndSpeed(Blob& blob, const int frame_count);
void getDetecPoint(Blob& blob, Mat roiImg, Mat erodeImg);

#endif 

