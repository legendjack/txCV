//#include <pthread.h>
#include <sstream>
#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "functions.h"
#include "getConfig.h"
#include "serialsom.h"
//#include "MyQueue.h"

// 宏定义
#define WINNAME	"Armor Recognition"
#define WINNAME1 "Binary Image"
#define MaxContourArea 450		// 面积大于该值的轮廓不是装甲的灯条
#define MinContourArea 25		// 面积小于该值的轮廓不是装甲的灯条
#define Width	800				// 视频宽
#define Height	600				// 视频高
//#define DEBUG
#define SEND

// 全局变量
VideoCapture cap;
VideoWriter writer;
map<string, string> config;
Mat frame, gray;				// 视频帧及其灰度图
Mat binaryImage, hsvImage;		// 二值图及HSV图，使用cvtColor得到
Mat element, element1;			// 开运算参数
string fileName;				// 视频的文件名
bool showBinaryImage = false;	// 是否显示二值图
int key = 0;					// waitKey() 返回值
int m_threshold;				// 灰度阈值
int detectColor;				// 敌军装甲的颜色：0-红色，1-蓝色
float tmpAngle0;
float tmpAngle1;
uint8_t yawOut = 250;			// 发送的 yaw 值
uint8_t pitchOut = 250;			// 发送的 pitch 值
uint8_t shot = 0;				// 是否射击
int frameCount = 450;
bool findArmor;					// 是否检测到装甲
Point targetPoint(370, 340);
Point centerOfArmor;
float areaOfLightContour = 0;	// 某个灯条的面积，用于大致表示目标装甲的远近。
SearchWindow searchWindow(Width, Height);		// 搜索窗口
bool useSW = false;				// 是否使用搜索窗口
int swSizeCache[5] = { 0,0,0,0,0 }; // 缓存 5 帧搜索窗口的尺寸（宽）
int recordVideo = 0;			// 是否录像
int videoName;					// 录像文件名

Mat bMat, gMat, rMat;			// BGR单通道图

void on_Mouse(int event, int x, int y, int flags, void*);
void* capFrameThread(void *arg);

int main()
{
	sleep(1);

#ifdef SEND
	// 初始化串口类
	Serialport Serialport1("/dev/ttyUSB0");
	int fd = Serialport1.open_port("/dev/ttyUSB0");
	while (fd < 0) {
		sleep(1);
		fd = Serialport1.open_port("/dev/ttyUSB0");
	}
	Serialport1.set_opt(115200, 8, 'N', 1);
#endif 

	/***************************************
			读取 video.cfg 里面的键值对
	****************************************/
	bool read = ReadConfig("video.cfg", config);	// 把video.cfg读入config键值对，成功返回true
	if (!read) {
		cout << "cannot open file : video.cfg" << endl;
		return -1;
	}

	/***************************************
				全局变量初始化
	****************************************/
	m_threshold = atoi(config["THRESHOLD"].c_str());
	detectColor = atoi(config["DETECTCOLOR"].c_str());
	fileName = config["FILENAME"].c_str();
	recordVideo = atoi(config["RECORDVIDEO"].c_str());

	element = getStructuringElement(MORPH_ELLIPSE, Size(2, 2));
	element1 = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
	bMat.create(Size(Width, Height), CV_8UC1);
	gMat.create(Size(Width, Height), CV_8UC1);
	rMat.create(Size(Width, Height), CV_8UC1);
	Mat chan[3] = { bMat, gMat, rMat };

	vector<RotatedRect> rotatedRects;			// 对面积在指定范围内的轮廓拟合椭圆，得到相应的旋转矩形
	vector<RotatedRect> rotatedRectsOfLights;	// 蓝色/红色灯条的RotatedRect

#ifdef DEBUG
	namedWindow(WINNAME, WINDOW_AUTOSIZE);
	createTrackbar("Threshold", WINNAME, &m_threshold, 255, 0);
	setMouseCallback(WINNAME, on_Mouse);		// 鼠标响应函数获取targetPoint
#endif

	//VideoCapture cap("1.avi");
	cap.open(0);
	while (!cap.isOpened()) {
		sleep(1);
		cap.open(0);
	}
	//cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));	// 需要在设置宽高之前设置，否则无效
	//cap.set(CV_CAP_PROP_SATURATION, 80);
	cap.set(CAP_PROP_FRAME_WIDTH, Width);
	cap.set(CAP_PROP_FRAME_HEIGHT, Height);
	
	if (recordVideo) {
		FileStorage fs1("record.xml", FileStorage::READ);
		fs1["videoName"] >> videoName;
		fs1.release();
		stringstream ss;
		ss << videoName;
		string s = ss.str();
		int fourcc = CV_FOURCC('M', 'J', 'P', 'G');
		writer.open(s + ".avi", fourcc, 25.0, Size(Width, Height));
		videoName++;
		FileStorage fs2("record.xml", FileStorage::WRITE);
		fs2 << "videoName" << videoName;
		fs2.release();
	}

	// 开启读取视频帧的线程（貌似程序在有些计算机上运行会因此崩溃）
/*	pthread_t id;
	int ret = pthread_create(&id, NULL, capFrameThread, NULL);
	if (!ret) {
		cout << "open thread to capture frame: success" << endl;
	} else {
		cout << "open thread to capture frame: failed" << endl;
	}*/

	/***************************************
				开始处理每一帧
	****************************************/
	while (true)
	{
		// double time0 = static_cast<double>(getTickCount());
		cap >> frame;

		if (frame.empty()) {
#ifdef DEBUG
			break;
#else
			continue;
#endif
		}
		
		if (recordVideo)
			writer.write(frame);

		// 如果当前帧使用搜索窗口，则仅保留目标区域图像，其他区域改为黑色
		if (useSW) {
			Mat frameTemp(Height, Width, CV_8UC3, Scalar(0, 0, 0));
			frame(searchWindow.getRect()).copyTo(frameTemp(searchWindow.getRect()));
			frame = frameTemp;
		}
		
#ifdef DEBUG
		Mat frame_ = frame.clone();		// 帧图像备份，调试用
#endif // DEBUG

		split(frame, chan);

		cvtColor(frame, gray, COLOR_BGR2GRAY);

		threshold(gray, binaryImage, m_threshold, 255, THRESH_BINARY);

		morphologyEx(binaryImage, binaryImage, MORPH_OPEN, element);	// 开运算，先腐蚀，后膨胀。去掉小的白色轮廓

#ifdef DEBUG
		Mat binaryImage_ = binaryImage.clone();		// 二值图像备份，调试用
#endif

		// 寻找轮廓，注意参数四不能设为 CHAIN_APPROX_SIMPLE，因为如果如果这样设置，表示一个轮廓的点的数量可能少于4
		// 而 fitEllipse 函数要求点的数量不能小于 4 ，否则会报错
		vector<vector<Point> > contours;		// 所有轮廓
		findContours(binaryImage, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);

		vector<vector<Point> > contoursInAreaRange;	// 面积在(MinContourArea, MaxContourArea)范围内的轮廓
		for (int i = 0; i < contours.size(); i++) {
			double areaTemp = contourArea(contours[i]);
			if (areaTemp > MinContourArea && areaTemp < MaxContourArea)
				contoursInAreaRange.push_back(contours[i]);
		}

		// 如果面积在指定范围内的轮廓数量小于2，则进入下一次循环（注意这种情况非常少）
		if (contoursInAreaRange.size() < 2) {
			frameCount++;
			
			if (frameCount > 9)
				useSW = false;
			
			if (frameCount < 60) {
				yawOut = 100;
				pitchOut = 100;
				shot = 0;
			} else if (frameCount >= 60) {
				frameCount--;
				yawOut = 250;
				pitchOut = 250;
				shot = 0;
			}
			goto HERE;
		}

		// 得到轮廓的旋转矩形 RotatedRect
		// 为每一个符合条件的 RotatedRect 制作一个掩模，判断掩模区域的主要颜色是红色还是蓝色
		rotatedRectsOfLights.clear();
		for (int i = 0; i < contoursInAreaRange.size(); i++) {
			RotatedRect rotatedRectTemp = fitEllipse(contoursInAreaRange[i]);
			float tmpA = rotatedRectTemp.angle;		// 旋转矩形的角度
			if (tmpA > 25 && tmpA < 155)			// 旋转矩形的角度是否在指定范围内
				continue;

			Mat mask(Height, Width, CV_8UC1, Scalar(0));
			drawContours(mask, contoursInAreaRange, i, Scalar(255), -1);	// 绘制（轮廓的）掩模
			dilate(mask, mask, element1);
			Mat mask1;
			dilate(mask, mask1, element1);
			bitwise_not(mask, mask);
			Mat mask2;								// 带形掩模，白色轮廓周围的光带
			bitwise_and(mask, mask1, mask2);

			bool b1 = false;
			if (detectColor)
				b1 = JudgeColor(bMat, rMat, 90, mask2);		// 蓝色装甲
			else
				b1 = JudgeColor(rMat, bMat, 60, mask2);		// 红色装甲

			if (b1) {
				rotatedRectsOfLights.push_back(rotatedRectTemp);
			}
		}

		/* 如果检测到灯条的数量等于0，则发送未检测到目标的信号，进入下一帧（这种情况较多）
		 * 如果之前检测到了装甲（frameCount=0），而后又出现灯条数量为0的情况
		 * 可能是对面步兵车被打败、撤退或者移动，发送云台静止不动的信号，等待几秒再发送云台进入搜索模式的信号
		 */
		if (rotatedRectsOfLights.size() == 0) {
			if (frameCount > 9)
				useSW = false;
			
			// 如果某一帧开始没有检测到装甲，frameCount自加一
            frameCount++;
            if (frameCount <= 60) {
				pitchOut = 100;
				yawOut = 100;
				shot = 0;
            } else if (frameCount > 60) {
				// 如果连续60帧没有检测到装甲，则认定为没有目标，串口发送信息进入搜索模式
				frameCount--;
                pitchOut = 250;
				yawOut = 250;
				shot = 0;
			}
			goto HERE;
		}

#ifdef DEBUG
		// 绘制每个灯条的拟合椭圆
		for (int i = 0; i < rotatedRectsOfLights.size(); i++)
			ellipse(frame_, rotatedRectsOfLights[i], Scalar(0, 255, 0), 2, LINE_AA);
#endif

        tmpAngle0 = 10;		// 两个灯条的角度差（绝对值），如果是这种情况：// 或 \\ ，则角度差在10以内有可能是一对装甲
		tmpAngle1 = 170;	// 如果是这种情况：/\ 或 \/ ，则角度差大于170度可能是一对装甲
		findArmor = false;
		// 寻找属于一个装甲的两个灯条，从而确定装甲的位置
		for (int i = 0; i < rotatedRectsOfLights.size() - 1; i++) {
			for (int j = i + 1; j < rotatedRectsOfLights.size(); j++) {
				float angleDifference = abs(rotatedRectsOfLights[i].angle - rotatedRectsOfLights[j].angle);		// 灯带的角度差
				float yDifference = abs(rotatedRectsOfLights[i].center.y - rotatedRectsOfLights[j].center.y);	// 灯带的Y轴差
				float xDifference = abs(rotatedRectsOfLights[i].center.x - rotatedRectsOfLights[j].center.x);	// 灯带的X轴差
				float rotatedRectHeight = rotatedRectsOfLights[i].size.height;
				float rotatedRectWidth = rotatedRectsOfLights[i].size.width;
				if (rotatedRectHeight < rotatedRectWidth)
					exchange(rotatedRectHeight, rotatedRectWidth);
				if (xDifference < rotatedRectHeight*2/3)
					continue;
				if ((angleDifference < 10 || angleDifference > 170) &&
					(angleDifference < tmpAngle0 || angleDifference > tmpAngle1) &&
					yDifference < 10) {
#ifdef DEBUG
					circle(frame_, rotatedRectsOfLights[i].center, 3, Scalar(0, 0, 255), -1, LINE_AA);
					circle(frame_, rotatedRectsOfLights[j].center, 3, Scalar(0, 0, 255), -1, LINE_AA);
#endif
					centerOfArmor = centerOf2Points(rotatedRectsOfLights[i].center, rotatedRectsOfLights[j].center);
					
					// 更新 tmpAngle0 和 tmpAngle1，目的是找到角度差最小的一对灯条，认为它们属于一个装甲
					if (angleDifference < 10)
						tmpAngle0 = angleDifference;
					else if (angleDifference > 170)
						tmpAngle1 = angleDifference;
					findArmor = true;
					
					useSW = true;

					for (int k = 0; k < 4; k++)
						swSizeCache[k] = swSizeCache[k + 1];
					swSizeCache[4] = rotatedRectHeight;
					int swSizeCacheSum = 0;
					for (int k = 0; k < 5; k++)
						swSizeCacheSum += swSizeCache[k];

					searchWindow.setCenter(centerOfArmor.x, centerOfArmor.y);
					searchWindow.setSize(swSizeCacheSum / 5 * 25, rotatedRectHeight * 15);
				}
			}
		}

		if (findArmor) {
			
			// 如果检测到了装甲的位置，frameCount置零，并向串口发送装甲的位置信息
			frameCount = 0;

#ifdef DEBUG
			circle(frame_, centerOfArmor, 10, Scalar(0, 0, 255), 2, LINE_AA);
#endif

			int disX = centerOfArmor.x - targetPoint.x;
			int disY = centerOfArmor.y - targetPoint.y;

			disX = -disX;
			disX += 100;
			if (disX > 200)
				disX = 200;
			else if (disX < 0)
				disX = 0;

			disY += 100;
			if (disY > 200)
				disY = 200;
			else if (disY < 0)
				disY = 0;

			yawOut = static_cast<uint8_t>(disX);
			pitchOut = static_cast<uint8_t>(disY);
			shot = 1;

		} else {
			
			// 检测到灯条但是没有匹配到装甲
			frameCount++;
			if (frameCount > 6)
				useSW = false;
			
			if (frameCount > 10) {
				int disX, disY;
				
				// 如果没有检测到装甲，且画面中灯条的数量大于2,则向窗口发送灯条的位置信息
				float minAngle = 200.;
				for (int i = 0; i < rotatedRectsOfLights.size(); i++)  {
					float angleTemp = rotatedRectsOfLights[i].angle;
					if (angleTemp  > 155)
						angleTemp = 180 - angleTemp;
					
					if (angleTemp < minAngle) {
						disX = rotatedRectsOfLights[i].center.x - targetPoint.x;
						disY = rotatedRectsOfLights[i].center.y - targetPoint.y;
						minAngle = angleTemp;
					}
				}
				
				disX = -disX;
				disX += 100;
				if (disX > 200)
					disX = 200;
				else if (disX < 0)
					disX = 0;

				disY += 100;
				if (disY > 200)
					disY = 200;
				else if (disY < 0)
					disY = 0;

				yawOut = static_cast<uint8_t>(disX);
				pitchOut = static_cast<uint8_t>(disY);
				shot = 0;
			}
		}

HERE:

#ifdef SEND
		Serialport1.usart3_send(pitchOut, yawOut, shot);
#else
		continue;
#endif
		//time0 = ((double)getTickCount() - time0) / getTickFrequency();
		//cout << "time : " << time0 * 1000 << "ms"  << endl;
#ifdef DEBUG
        cout << static_cast<int>(pitchOut) << ", " << static_cast<int>(yawOut) << ", " << static_cast<int>(shot) << endl;

		imshow(WINNAME, frame_);

		key = (waitKey(30)&255);
		if (key == 27) {
			break;
		}  else if (key == int('0')) {
			if (showBinaryImage)	// 如果这时候窗口是显示的，则关闭它
				destroyWindow(WINNAME1);
			showBinaryImage = !showBinaryImage;
		} else if (key == 32) {
			waitKey(0);
		}
#endif
	}
	return 0;
}

void* capFrameThread(void *arg)
{
	while(true) {
		if (cap.isOpened())
			cap >> frame;
	}
    return NULL;
}

void on_Mouse(int event, int x, int y, int flags, void*) {
	if (event == EVENT_LBUTTONDOWN) {
		targetPoint.x = x;
		targetPoint.y = y;
	}
}
