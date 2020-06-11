#include "head.h"

//可以把这一堆全局变量写到类中...
extern int WIDTH;
extern  int HEIGHT;
extern int carNumber;
extern bool isTransform;

extern Point roiPoints[1][4];
extern Point transPoints[1][4];
extern vector<Point2f> roi;
extern vector<Point2f> trans;
extern Mat H;
extern Move carPointOnLeft_1, carPointOnLeft_2, carPointOnLeft_3;//统计三个车道的质心数量

int main() {

	VideoCapture cap;
	cap.open("道路4K6车道_cut1.avi");
	if (!cap.isOpened()) {
		cout << "Open Fail" << endl;
		return -1;
	}

	VideoWriter writer;
	writer.open("RoI.avi", CV_FOURCC('H', '2', '6', '4'), 20, Size(WIDTH, HEIGHT), true);

	Ptr<BackgroundSubtractorMOG2> bgsubtractor;//MOG2背景减法器
	bgsubtractor = createBackgroundSubtractorMOG2(500, 16, false);

	Mat fgmask,img;
	cap >> img;
	double time_count = 0;//计算背景建模所需时间
	int frame_count = 1;//计算当前帧数

	while (!img.empty()) {
		frame_count++;

		// 压缩图像
		Mat resizedImg;
		resize(img, resizedImg, Size(WIDTH, HEIGHT));
		//imshow("resize.png", resizedImg);

		//绘制RoI区域
		Mat imgCopy = resizedImg.clone();
		const Point* ppt[1] = { roiPoints[0] };
		int npt[] = { 4 };
		//polylines(imgCopy, ppt, npt, 1, 1, Scalar(0, 0, 0), 2, 8, 0);  //绘制多边形
		Mat roiImg, mergeImg;  //融合后的mergeImage只是为了最后的展示,实际处理的图像为roiImage
		setRoIPoint();  //设置roi顶点参数
		roiImg = getRoI(imgCopy, ppt, npt);
		//imshow("roiImg", roiImg);
		mergeImg = getShow(imgCopy, ppt, npt);
		imshow("mergeImg", mergeImg);

		//进行透视变换
		settransPoints();
		if (!isTransform) {//透视变换只需求一次即可
			H = findHomography(roi, trans, RANSAC);//计算透视矩阵
			isTransform = true;
		}
		warpPerspective(roiImg,roiImg, H,roiImg.size());//图像透视变换
		//imshow("trans", roiImg);

		Mat roiImgGray;
		cvtColor(roiImg, roiImgGray, CV_BGR2GRAY);
		//高斯模糊
		GaussianBlur(roiImgGray, roiImgGray, Size(7, 7), 0);
		//imshow("roiImg_2", roiImgGray);
		
		//背景建模
		clock_t startTime = clock();
		bgsubtractor->apply(roiImgGray, fgmask);
		if (frame_count < 3) {
			cap >> img;
			continue;
		}
		clock_t endTime = clock();
		time_count +=  double(endTime - startTime) / CLOCKS_PER_SEC;
		//if (frame_count % 10 == 0) cout << time_count / frame_count << endl;
		//imshow("fgmask", fgmask);

		//形态学滤波操作
		Mat dilateImg, erodeImg;
		Mat dilateStruct = getStructuringElement(MORPH_RECT, Size(5, 5));
		Mat erodeStruct = getStructuringElement(MORPH_RECT, Size(5, 5));

		dilate(fgmask, dilateImg, dilateStruct);
		//imshow("dilateImg", dilateImg);
		erode(dilateImg, erodeImg, erodeStruct);
		//imshow("erodeImg", erodeImg);

		Blob blob;
		getDetecPoint(blob,roiImg,erodeImg);//通过roiImg和eroImg获取Blob数据
		getCarNumberAndSpeed(blob,frame_count);

		//输出信息
		char tempchar[1024];
		sprintf_s(tempchar, "车辆数 : %d", carNumber);
		//putText(roiImg, tempchar, Point(10, 20), FONT_HERSHEY_PLAIN, 2, Scalar(0, 255, 0), 3);
		putTextZH(roiImg, tempchar, Point(10, 10), Scalar(0, 255, 0), 30, "宋体");
		imshow("drawing", roiImg);

		waitKey(1);
		cap >> img;
	}
	return 0;
}
