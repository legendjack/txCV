#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include "serialsom.h"

using namespace std;
using namespace cv;

#define up 0x11
#define DOWN 0x12
#define FROUNT 0x13
#define BACK 0x14
#define LEFT 0x15
#define RIGHT 0x16
//#define DEBUG

/*
需要注意的地方：
1.增加了XSTOP与YSTOP方向标志位，当某一方向到达指定位置时，这一方向的速度为0
2.对距离的判断是模糊值，未经过测试，现在一个设置20，一个设置30
3.图像大小未确定，现在采用800*400
4.速度值暂时给0
5.逻辑顺序为先判断X，当X到达指定位置时，再执行Y方向
*/

void Delay(int time);
void send_select(int flag);
void sortPoints(vector<Point2f>& p_);
int countDifference(Mat mask, Mat mask0);
uint8_t stop_flag, dir, speed;//改为uint8_t
Point center_point;
Point detect_point;
int detect_flag = 0;
Serialport Serialport1("/dev/ttyTHS1");

const int width = 640;
const int height = 480;

int x_stop = 0;
int y_stop = 1;
int count_ = 0;
int final_flag = 0;
int recordVideo = 0;				// 是否录像
int videoName;						// 录像文件名

									// 检测神符立柱，视频
int main()
{
	int detectColor = 0;
	int key = 0;

	Point2f srcPoints[4];
	Point2f dstPoints[4];
	center_point.x = width / 2 + 30;
	center_point.y = height / 2 - 20;
	dstPoints[0] = Point2f(0, 0);
	dstPoints[1] = Point2f(50, 0);
	dstPoints[2] = Point2f(50, 50);
	dstPoints[3] = Point2f(0, 50);

	Mat warpMat(2, 4, CV_32FC1);

	Mat dstImage(50, 50, CV_8UC1);

	Mat mask0 = imread("mask.png", -1);

	Mat frame, frame_ycrcb, binary_image, binary_clone;

	Mat yMat(height, width, CV_8UC1);
	Mat rMat(height, width, CV_8UC1);
	Mat bMat(height, width, CV_8UC1);

	Mat channels[3] = { yMat, rMat, bMat };

	// 打开摄像头
	VideoCapture cap(0);
	while (!cap.isOpened()) {
		sleep(1);
		cap.open(0);
	}
	cap.set(3, width);
	cap.set(4, height);

	VideoWriter writer;

	// 读取配置文件
	FileStorage fs("config.xml", FileStorage::READ);
	fs["detectColor"] >> detectColor;
	fs["recordVideo"] >> recordVideo;
	fs.release();

	if (recordVideo) {
		FileStorage fs1("record.xml", FileStorage::READ);
		fs1["videoName"] >> videoName;
		fs1.release();
		stringstream ss;
		ss << videoName;
		string s = ss.str();
		int fourcc = CV_FOURCC('M', 'J', 'P', 'G');
		writer.open(s + ".avi", fourcc, 25.0, Size(width, height));
		videoName++;
		FileStorage fs2("record.xml", FileStorage::WRITE);
		fs2 << "videoName" << videoName;
		fs2.release();
	}

	// 初始化串口类
	int fd = Serialport1.open_port("/dev/ttyTHS1");
	while (fd < 0) {
		sleep(1);
		fd = Serialport1.open_port("/dev/ttyTHS1");
	}
	Serialport1.set_opt(115200, 8, 'N', 1);

	while (true)
	{
		Delay(200);

		cap >> frame;
		if (!frame.data) continue;

		if (recordVideo)
			writer.write(frame);

		cvtColor(frame, frame_ycrcb, COLOR_BGR2YCrCb);
		split(frame_ycrcb, channels);

		double thresh;

		if (detectColor)
			thresh = threshold(bMat, binary_image, 140, 255, THRESH_BINARY);	// 如果是蓝色
		else
			thresh = threshold(rMat, binary_image, 165, 255, THRESH_BINARY);	// 如果是红色

																				//cout << thresh << endl;

#ifdef DEBUG
		imshow("bin", binary_image);
#endif

		binary_clone = binary_image.clone();
		vector<vector<Point> > contours;
		findContours(binary_clone, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

		if (contours.size() < 3) {
#ifdef DEBUG			
			imshow("frame", frame);
			key = (waitKey(10) & 255);
			if (key == 27) break;
			else if (key == 32) waitKey(0);
#endif			
			continue;
		}

		vector<RotatedRect> contours_rotatedRect;
		for (int i = 0; i < contours.size(); i++)
			if (contourArea(contours[i]) > 5000)
				contours_rotatedRect.push_back(minAreaRect(contours[i]));

		for (int i = 0; i < contours_rotatedRect.size(); i++) {
			vector<Point2f> p(4);
			contours_rotatedRect[i].points(p.data());
			sortPoints(p);

			srcPoints[0] = p[0];
			srcPoints[1] = p[1];
			srcPoints[2] = p[2];
			srcPoints[3] = p[3];

			// 透视变换，将密码区变换成 50*50 的Mat
			warpMat = getPerspectiveTransform(srcPoints, dstPoints);
			warpPerspective(binary_image, dstImage, warpMat, dstImage.size());

			circle(dstImage, Point(25, 25), 20, Scalar(0), -1);

			if (countDifference(dstImage, mask0) < 500) {
#ifdef DEBUG
				circle(frame, contours_rotatedRect[i].center, 4, Scalar(0, 255, 0), 2, 16);
#endif
				detect_point = contours_rotatedRect[i].center;
				detect_flag = 1;
				stop_flag = 0x01;//运动模式
			}
			else
			{
				detect_flag = 0;
			}
			send_select(detect_flag);
		}
#ifdef DEBUG
		imshow("frame", frame);
		key = (waitKey(10) & 255);
		if (key == 27) break;
		else if (key == 32) waitKey(0);
#endif
	}

	return 0;
}

void send_select(int flag)
{

	if (detect_flag == 1)
	{
		stop_flag = 0x01;//运动模式
		int x_value = detect_point.x - center_point.x;
		int y_value = detect_point.y - center_point.y;
		uint8_t speed = 0x00;
		//判断X方向
		if (((y_stop == 1) || (y_stop == 2)) && (final_flag == 0))
		{
			if ((x_value <= 25) && (x_value >= -25))
			{
				x_stop = 1;
				count_ = 1;
				if (y_stop == 2)
				{
					count_ = 2;
				}
			}
			else if (x_value > 25)
			{
				dir = LEFT;
				x_stop = 0;
			}
			else if (x_value < -25)
			{
				dir = RIGHT;
				x_stop = 0;
			}
		}
		//判断y方向
		if ((x_stop) && (final_flag == 0))
		{
			if ((y_value <= 25) && (y_value >= -25))
			{
				y_stop = 2;
			}
			else if (y_value > 25)
			{

				dir = BACK;
				y_stop = 0;

			}
			else if (y_value < -25)
			{

				dir = FROUNT;
				y_stop = 0;
			}
		}
		if ((x_value <= 25) && (x_value >= -25) && (y_value <= 25) && (y_value >= -25) && (count_ = 2))
		{
			dir = DOWN;

		}


	}
	else if (detect_flag != 1)
	{
		stop_flag = 0x00;//悬停模式
		dir = 0x00;
		speed = 0x00;
	}

	if (dir == DOWN)
	{
		Serialport1.usart3_send(stop_flag, dir, speed);
		Delay(100);
	}
	else
	{
		Serialport1.usart3_send(stop_flag, dir, speed);
	}//暂时速度先发0

#ifndef DEBUG
	cout << static_cast<int>(stop_flag) << ", " << static_cast<int>(dir) << ", " << static_cast<int>(speed) << endl;
#endif
}

void sortPoints(vector<Point2f>& p_)
{
	for (int i = 0; i < 3; i++)
		for (int j = i + 1; j < 4; j++)
			if (p_[i].y > p_[j].y)
				swap(p_[i], p_[j]);

	if (p_[0].x > p_[1].x)
		swap(p_[0], p_[1]);
	if (p_[2].x < p_[3].x)
		swap(p_[2], p_[3]);
}

int countDifference(Mat mask, Mat mask0) {
	int sum = 0;
	for (int i = 0; i < 2500; i++) {
		int a = mask.data[i];
		int b = mask0.data[i];
		if (a != b)
			sum++;
	}

	return sum;
}

//延时 time 毫秒
void Delay(int time) {
	time = time * 1000;
	clock_t now = clock();
	while (clock() - now < time);
}

// 提取神符立柱上面的红色、蓝色圆
int main2()
{
	Point2f srcPoints[4];
	Point2f dstPoints[4];

	dstPoints[0] = Point2f(0, 0);
	dstPoints[1] = Point2f(50, 0);
	dstPoints[2] = Point2f(50, 50);
	dstPoints[3] = Point2f(0, 50);

	Mat src = imread("2.jpg");

	int w = src.cols;
	int h = src.rows;

	Mat src_ycrcb;
	cvtColor(src, src_ycrcb, COLOR_BGR2YCrCb);

	Mat yMat(h, w, CV_8UC1);
	Mat rMat(h, w, CV_8UC1);
	Mat bMat(h, w, CV_8UC1);

	Mat channels[3] = { yMat, rMat, bMat };

	split(src_ycrcb, channels);

	// 	imshow("y", yMat);
	// 	imshow("r", rMat);
	// 	imshow("b", bMat);
	// 	imshow("src", src);

	Mat binary;
	threshold(bMat, binary, 0, 255, THRESH_OTSU);

	imshow("bin", binary);
	waitKey(0);

	Mat bin_ = binary.clone();
	vector<vector<Point> > contours;
	findContours(bin_, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	vector<RotatedRect> contours_rotatedRect;
	for (int i = 0; i < contours.size(); i++)
		contours_rotatedRect.push_back(minAreaRect(contours[i]));

	for (int i = 0; i < contours.size(); i++) {
		vector<Point2f> p(4);
		contours_rotatedRect[i].points(p.data());
		sortPoints(p);

		srcPoints[0] = p[0];
		srcPoints[1] = p[1];
		srcPoints[2] = p[2];
		srcPoints[3] = p[3];

		Mat warpMat(2, 4, CV_32FC1);
		warpMat = getPerspectiveTransform(srcPoints, dstPoints);

		//imshow("bin", binary);

		// 透视变换，将密码区变换成50*50的Mat
		Mat dstImage(50, 50, CV_8UC1);
		warpPerspective(binary, dstImage, warpMat, dstImage.size());

		circle(dstImage, Point(25, 25), 20, Scalar(0), -1);

		//imwrite("mask" + to_string(i) + ".png", dstImage);

		imshow("dst", dstImage);
		waitKey(0);
	}

	return 0;
}
