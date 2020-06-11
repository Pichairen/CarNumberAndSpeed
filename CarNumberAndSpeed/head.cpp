#include "head.h"

int gap = (302 - 110) / 3;
pair<int, int> left_1 = { 110, 110 + gap };//左1车道
pair<int, int> left_2 = { 110 + gap, 110 + gap * 2 };//左2车道
pair<int, int> left_3 = { 110 + gap * 2, 302 };//左3车道

int WIDTH = 640;
int HEIGHT = 360;
int carNumber = 0;//车辆数

bool isTransform = false;//只进行一次透视变换即可

bool isDetectedLeft_1_zoom_1 = false;
bool isDetectedLeft_2_zoom_1 = false;
bool isDetectedLeft_3_zoom_1 = false;

bool isDetectedLeft_1_zoom_2 = false;
bool isDetectedLeft_2_zoom_2 = false;
bool isDetectedLeft_3_zoom_2 = false;

Vec4i regin_split = { 140,160,180,200 };//前后区域带，前两个为第一个区域，后两个为第二个区域
Point roiPoints[1][4];
Point transPoints[1][4];
vector<Point2f> roi(4);
vector<Point2f> trans(4);

Mat H;
Move carPointOnLeft_1, carPointOnLeft_2, carPointOnLeft_3;//统计三个车道的质心数量
Mat getRoI(Mat img, const Point** ppt, int* npt) {
	Mat mask, roiImage;
	img.copyTo(mask);
	mask.setTo(Scalar::all(0));  //全0填充
	fillPoly(mask, ppt, npt, 1, Scalar(255, 255, 255));  //全白填充多边形区域
	img.copyTo(roiImage, mask);  //img与mask交集存到roiImg
	return roiImage;
}
Mat getShow(Mat img, const Point** ppt, int* npt) {
	Mat mask, mergeImg;
	img.copyTo(mask);
	fillPoly(mask, ppt, npt, 1, Scalar(0, 0, 255));  //填充对象、顶点数组、顶点个数、绘制多边形个数、颜色
	addWeighted(img, 0.8, mask, 0.2, 0, mergeImg);  //img1,weight1,img2,weight2,gama,融合后的图像
	return mergeImg;
}
void setRoIPoint() {
	//RoI的顶点
	roiPoints[0][0] = Point(110, 127);
	roiPoints[0][1] = Point(302, 127);
	roiPoints[0][2] = Point(298, 210);
	roiPoints[0][3] = Point(20, 210);

	roi[0] = Point(110, 127);
	roi[1] = Point(302, 127);
	roi[2] = Point(298, 210);
	roi[3] = Point(20, 210);
}
void settransPoints() {
	transPoints[0][0] = Point(110, 127);
	transPoints[0][1] = Point(302, 127);
	transPoints[0][2] = Point(298, 210);
	transPoints[0][3] = Point(20, 210);

	trans[0] = Point(110, 127);
	trans[1] = Point(302, 127);
	trans[2] = Point(302, 210);
	trans[3] = Point(110, 210);
}
void createBackgroundImg(const Mat roiImg, const int frame_count) {
	if (frame_count % 20 == 0) {
		char tempchar[1024];
		sprintf_s(tempchar, "%s_%d.%s", "background/bg", frame_count, "png");
		imwrite(tempchar, roiImg);
	}
}
double getDistance(Point pointO, Point pointA)

{
	double distance;
	distance = powf((pointO.x - pointA.x), 2) + powf((pointO.y - pointA.y), 2);
	distance = sqrtf(distance);
	return distance;
}
double getSpeed(Move& centerPointLeftN) {
	double speed = -1;
	if (centerPointLeftN.endPoint.y > centerPointLeftN.startPoint.y) {//判断有效果性质
		//double distance = getDistance(centerPointLeftN.startPoint,centerPointLeftN.endPoint);//用质心的话使用该公式
		double distance = centerPointLeftN.endPoint.y - centerPointLeftN.startPoint.y;//用边框顶点用这个公式
		double time = (centerPointLeftN.secondFrameCount - centerPointLeftN.firstFrameCount)*(1. / 25);//25是帧率
		speed = distance / time;
		centerPointLeftN.Init();//重置一下
	}
	return speed;
}

void getDetecPoint(Blob& blob, Mat roiImg, Mat erodeImg) {
	//找轮廓边界
	vector<vector<Point>>contours;
	vector<Vec4i>hierarchy;
	findContours(erodeImg, contours, hierarchy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
	vector<Rect> boundRect(contours.size());

	//绘制外接矩形
	for (int i = 0; i < contours.size(); ++i) {
		boundRect[i] = boundingRect(Mat(contours[i]));
	}
	for (int i = 0; i < contours.size(); ++i) {
		int area = static_cast<int>(contourArea(contours[i]));
		if (area >= 300) {//过滤小面积
			//增加一个车道线的判断，如果车辆超过车道线的话就截取到车道线
			blob.centerPoint.push_back(Point((boundRect[i].tl().x + boundRect[i].br().x) / 2, (boundRect[i].tl().y + boundRect[i].br().y) / 2));
			blob.bottomRight.push_back(Point(boundRect[i].br().x, boundRect[i].br().y));//用这个去计算速度。
			rectangle(roiImg, boundRect[i].tl(), boundRect[i].br(), Scalar(0, 255, 0), 2, 8, 0);//tl:topleft,br:bottomright
			//circle(mergeImg, centerPoint[i], 2, Scalar(255, 0, 0));
		}
	}
	//绘制质心
	for (int i = 0; i < blob.centerPoint.size(); ++i) {
		circle(roiImg, blob.bottomRight[i], 2, Scalar(0, 0, 255));
		circle(roiImg, blob.centerPoint[i], 2, Scalar(0, 0, 255));
	}
	// 统计车流量并测速
}

// 后面的优化可以将质心和边界放在一个类中
void getCarNumberAndSpeed(Blob& blob, const int frame_count) {
	/* *****************************************划分车道线使用质心统计车流量***************************************/
		/* 思路：
		一、分车道线
		二、在感兴趣区内绘制一个狭窄的区域
		三、统计每个车道狭窄区域车辆质心的个数
			1、给当前帧狭窄区域一个标志位表示是否已经检测过
			2、判断质心是否在在区域内，在则统计质心个数并将标志位置1（这段区域只有一辆车）
			3、当质心离开该区域，恢复标志位
		*/


	for (int i = 0; i < blob.centerPoint.size(); ++i) {//判断当前帧在这个范围内的质心个数

		//left_1车道
		if (blob.centerPoint[i].x > left_1.first && blob.centerPoint[i].x <= left_1.second) {//使用质心判断车道
			if (!isDetectedLeft_1_zoom_1 && blob.bottomRight[i].y >= regin_split[0] && blob.bottomRight[i].y <= regin_split[1]) {//检测带1:150-160
				carNumber++;
				isDetectedLeft_1_zoom_1 = true;
				carPointOnLeft_1.startPoint = Point(blob.bottomRight[i].x, blob.bottomRight[i].y);//记录第一个右下角
				carPointOnLeft_1.firstFrameCount = frame_count;//记录当前位于那一帧
			}
			else if (blob.bottomRight[i].y > regin_split[1]) isDetectedLeft_1_zoom_1 = false;//质心离开狭窄区域，恢复未检测状态

			//下面是关于测速使用的
			if (!isDetectedLeft_1_zoom_2 && blob.bottomRight[i].y >= regin_split[2] && blob.bottomRight[i].y <= regin_split[3]) {//检测带2:160-180
				if (carPointOnLeft_1.startPoint != Point(0, 0)) {
					isDetectedLeft_1_zoom_2 = true;
					carPointOnLeft_1.endPoint = Point(blob.bottomRight[i].x, blob.bottomRight[i].y);
					carPointOnLeft_1.secondFrameCount = frame_count;//记录当前位于那一帧
				}
			}
			else if (blob.bottomRight[i].y > 190) isDetectedLeft_1_zoom_2 = false;
		}
		//left_2车道
		else if (blob.centerPoint[i].x > left_2.first&&blob.centerPoint[i].x <= left_2.second) {
			if (!isDetectedLeft_2_zoom_1 && blob.bottomRight[i].y >= regin_split[0] && blob.bottomRight[i].y <= regin_split[1]) {//在这个范围内统计车辆个数
				carNumber++;
				isDetectedLeft_2_zoom_1 = true;
				carPointOnLeft_2.startPoint = Point(blob.bottomRight[i].x, blob.bottomRight[i].y);//记录第二个右下角
				carPointOnLeft_2.firstFrameCount = frame_count;

			}
			else if (blob.bottomRight[i].y > regin_split[1]) isDetectedLeft_2_zoom_1 = false;//质心离开狭窄区域，恢复未检测状态

			//下面是关于测速使用的
			if (!isDetectedLeft_2_zoom_2 && blob.bottomRight[i].y >= regin_split[2] && blob.bottomRight[i].y <= regin_split[3]) {//检测带2:160-180
				if (carPointOnLeft_2.startPoint != Point(0, 0)) {
					isDetectedLeft_2_zoom_2 = true;
					carPointOnLeft_2.endPoint = Point(blob.bottomRight[i].x, blob.bottomRight[i].y);
					carPointOnLeft_2.secondFrameCount = frame_count;//记录当前位于那一帧
				}
			}
			else if (blob.bottomRight[i].y > regin_split[3]) isDetectedLeft_2_zoom_2 = false;
		}
		//left_3车道
		else if (blob.centerPoint[i].x > left_3.first&&blob.centerPoint[i].x <= left_3.second) {
			if (!isDetectedLeft_3_zoom_1 && blob.bottomRight[i].y >= regin_split[0] && blob.bottomRight[i].y <= regin_split[1]) {//在这个范围内统计车辆个数
				carNumber++;
				isDetectedLeft_3_zoom_1 = true;
				carPointOnLeft_3.startPoint = Point(blob.bottomRight[i].x, blob.bottomRight[i].y);//记录第三个右下角
				carPointOnLeft_3.firstFrameCount = frame_count;
			}
			else if (blob.bottomRight[i].y > regin_split[1]) isDetectedLeft_3_zoom_1 = false;//质心离开狭窄区域，恢复未检测状态

			//下面是关于测速使用的
			if (!isDetectedLeft_3_zoom_2 && blob.bottomRight[i].y >= regin_split[2] && blob.bottomRight[i].y <= regin_split[3]) {//检测带2:160-180
				if (carPointOnLeft_3.startPoint != Point(0, 0)) {
					isDetectedLeft_3_zoom_2 = true;
					carPointOnLeft_3.endPoint = Point(blob.bottomRight[i].x, blob.bottomRight[i].y);
					carPointOnLeft_3.secondFrameCount = frame_count;//记录当前位于那一帧
				}
			}
			else if (blob.bottomRight[i].y > regin_split[3]) isDetectedLeft_3_zoom_2 = false;
		}
		else {
			cout << "CenterPoint Is out ofRange!" << endl;
			continue;
		}
		/* *****************************************划分车道线使用质心统计车流量***************************************/

		//计算车速并重置
		double speedLeft_1 = getSpeed(carPointOnLeft_1);
		double speedLeft_2 = getSpeed(carPointOnLeft_2);
		double speedLeft_3 = getSpeed(carPointOnLeft_3);

		vector<double> speedRes = { speedLeft_1,speedLeft_2,speedLeft_3 };
		for (int i = 0; i < speedRes.size(); ++i) {
			if (speedRes[i] != -1) {
				char temp[1024];
				sprintf_s(temp, "车道 %d 的速度是 %.3fpix/s", i + 1, speedRes[i]);
				cout << temp << endl;
			}
		}
	}
}

