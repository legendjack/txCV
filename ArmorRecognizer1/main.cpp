#include "getConfig.h"
#include "functions.h"
#include "StereoVision.h"
//#include <fstream>
// 宏定义
#define WINNAME	"Armor Recognition"
#define WINNAME1 "Binary Image"
#define MaxContourArea 450		// 面积大于该值的轮廓不是装甲的灯条
#define MinContourArea 15		// 面积小于该值的轮廓不是装甲的灯条
#define Width	800				// 视频宽
#define Height	600				// 视频高
#define DEBUG

// 双目相机参数
Mat cameraMatrixL = (Mat_<double>(3, 3) << 496.27399321974093, 0.0, 374.0668138204964,
	0.0, 498.3793858306596, 250.94355859808064, 0., 0., 1.);
Mat distCoeffL = (Mat_<double>(1, 5) << 0.07744253729392198, -0.1719657313330516,
	-3.7548985291613806E-4, -0.0030113572086701416, 0.0);

Mat cameraMatrixR = (Mat_<double>(3, 3) << 497.93776864936774, 0.0, 320.86747235423115,
	0.0, 500.1540267095238, 237.97815325244434, 0., 0., 1.);
Mat distCoeffR = (Mat_<double>(1, 5) << 0.06579129374751846, -0.1301467847601383,
	2.1072510864203017E-4, -0.0018800056606088189, 0.0);

Mat T = (Mat_<double>(3, 1) << -99.42713985814156, 0.7655538864613708, 0.44040505692313575);
Mat R = (Mat_<double>(3, 3) << 0.9999889135362465, -0.0032021007711087665, -0.0034524419255046505,
	0.0031651868496217588, 0.9999383299746322, -0.010645085239239016,
	0.0034863156489765962, 0.010634039599105673, 0.999937379441833);

// 全局变量
map<string, string> config;
Mat frameL, grayL;				// 视频帧及其灰度图
Mat frameR, grayR;				// 视频帧及其灰度图
Mat grayLP, grayRP;				// part of gray image
Mat binaryImage, hsvImage;		// 二值图及HSV图，使用cvtColor得到
int m_threshold;				// 阈值
bool showBinaryImage = false;	// 是否显示二值图
bool isFirstFrame = true;
Mat element, element1;			// 开运算参数
int detectColor;				// 敌军装甲的颜色：0-红色，1-蓝色
string fileNameL, fileNameR;	// 视频的文件名
Mat mapLx, mapLy, mapRx, mapRy;	// 映射表
float distanceValue = 2000;
int numDisparities = 64;
float disX, disY, disZ;
Vec3f lastCoordinate;
double time0 = 0;

// 计算直方图需要的参数
Mat hMat, sMat, vMat;			// HSV单通道图
int channels = 0;				// 计算第0个通道的直方图，calcHist参数
int sizeHist = 180;				// 180个色度，calcHist参数
MatND dstHist;					// calcHist结果

int main()
{

	/***************************************
			读取 video.cfg 里面的键值对
	****************************************/
	bool read = ReadConfig("video.cfg", config);	// 把video.cfg读入config键值对，成功返回true
	if (!read)
	{
		cout << "无法读取 video.cfg" << endl;
#ifdef WIN32
		system("pause");
#endif
		return -1;
	}
	
	/***************************************
				全局变量初始化
	****************************************/
	m_threshold = atoi(config["THRESHOLD"].c_str());
	detectColor = atoi(config["DETECTCOLOR"].c_str());
	fileNameL = config["FILENAMEL"].c_str();
	fileNameR = config["FILENAMER"].c_str();

	element = getStructuringElement(MORPH_ELLIPSE, Size(2, 2));
	element1 = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
	hMat.create(Size(Width, Height), CV_8UC1);
	sMat.create(Size(Width, Height), CV_8UC1);
	vMat.create(Size(Width, Height), CV_8UC1);
	Mat chan[3] = { hMat, sMat, vMat };
	float hranges[] = { 0, 180 };
	const float *ranges[] = { hranges };
	lastCoordinate = Vec3f(0, 0, 2000);

	/* 声明StereoVision对象 */
	StereoVision stereoVision(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, T, R, Width, Height);

	stereoVision.getMaps(mapLx, mapLy, mapRx, mapRy);

	namedWindow(WINNAME, WINDOW_AUTOSIZE);
	createTrackbar("Threshold", WINNAME, &m_threshold, 255, 0);

	VideoCapture capL("outputLeft.avi");
	VideoCapture capR("outputRight.avi");

	if (!capL.isOpened())
	{
		cout << "未打开视频文件" << endl;
		cout << fileNameL << endl;
		cout << fileNameR << endl;
#ifdef WIN32
		system("pause");
#endif
		return -1;
	}

// 	capL.set(CAP_PROP_POS_FRAMES, 375);
// 	capR.set(CAP_PROP_POS_FRAMES, 375);

	/***************************************
				开始处理对每一帧
	****************************************/
	while (true)
	{
		double time0 = static_cast<double>(getTickCount());

		capL >> frameL;
		capR >> frameR;

		if (frameL.empty())
			return -1;

		remap(frameL, frameL, mapLx, mapLy, INTER_LINEAR);

#ifdef DEBUG
		Mat frame_ = frameL.clone();		// 帧图像备份，调试用
#endif // DEBUG
		
		cvtColor(frameL, grayL, COLOR_BGR2GRAY);
		cvtColor(frameL, hsvImage, COLOR_BGR2HSV);

		cvtColor(frameR, grayR, COLOR_BGR2GRAY);
		remap(grayR, grayR, mapRx, mapRy, INTER_LINEAR);

		//time0 = static_cast<double>(getTickCount());
		/* 对双目图进行立体匹配 */
// 		grayLP = grayL(Rect(0, 180, 640, 160));
// 		grayRP = grayR(Rect(0, 180, 640, 160));

		if (distanceValue >= 1700 && numDisparities != 48) {
			stereoVision.setNumDisparities(48);
			numDisparities = 48;
		}
		if (distanceValue >= 1000 && distanceValue < 1700 && numDisparities != 64) {
			stereoVision.setNumDisparities(64);
			numDisparities = 64;
		}
// 		else if (distanceValue < 1300 && distanceValue >= 900 && numDisparities != 64) {
// 			stereoVision.setNumDisparities(64);
// 			numDisparities = 64;
// 		}
// 		else if (distanceValue < 900 && numDisparities != 80) {
// 			stereoVision.setNumDisparities(80);
// 			numDisparities = 80;
// 		}
		if (isFirstFrame) {
			isFirstFrame = false;
			stereoVision.setNumDisparities(64);
		}
//		stereoVision.stereoMatch(grayLP, grayRP);
		stereoVision.stereoMatch(grayL, grayR);

		//time0 = ((double)getTickCount() - time0) / getTickFrequency();
		//cout << "time : " << time0 * 1000 << "ms" << endl;

		Mat distImg;
		stereoVision.getDisparityImage(distImg);
		imshow("distImg", distImg);

		threshold(grayL, binaryImage, m_threshold, 255, THRESH_BINARY);

		morphologyEx(binaryImage, binaryImage, MORPH_OPEN, element);	// 开运算，先腐蚀，后膨胀。去掉小的白色轮廓

#ifdef DEBUG
		Mat binaryImage_ = binaryImage.clone();  // 二值图像备份，调试用
#endif // DEBUG

		vector<vector<Point> > contours;		// 所有轮廓，findContours函数的结果
		findContours(binaryImage, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);	// 寻找轮廓

		vector<vector<Point> > contoursInAreaRange;	// 面积在(MinContourArea, MaxContourArea)范围内的轮廓
		for (int i = 0; i < contours.size(); i++)
		{
			double areaTemp = contourArea(contours[i]);
			if (areaTemp > MinContourArea && areaTemp < MaxContourArea)
				contoursInAreaRange.push_back(contours[i]);
		}

		vector<RotatedRect> rotatedRects;	// 对面积在指定范围内的轮廓拟合椭圆，得到相应的旋转矩形
 		for (int i = 0; i < contoursInAreaRange.size(); i++)
 			rotatedRects.push_back(fitEllipse(contoursInAreaRange[i]));

		/* 为每一个符合条件的RotatedRect制作一个掩模，然后计算掩模区域的直方图，
		 * 通过直方图判断该RotatedRect的主要颜色是红色还是蓝色
		 */
		vector<RotatedRect> rotatedRectsOfLights;	// 蓝色/红色灯条的RotatedRect
		split(hsvImage, chan);						// 把HSV图像分为3个通道，用于计算直方图
		
		for (int i = 0; i < rotatedRects.size(); i++)
		{
			Point2f pointTemp[4];
			rotatedRects[i].points(pointTemp);
			vector<Point> corners;
			for (int j = 0; j < 4; j++)
				corners.push_back(pointTemp[j]);

			vector<vector<Point> > corners_;
			corners_.push_back(corners);
			Mat mask(Height, Width, CV_8UC1, Scalar::all(0));
			drawContours(mask, corners_, -1, Scalar(255), -1, LINE_AA);
			dilate(mask, mask, element1);	// 膨胀

			calcHist(&hMat, 1, &channels, mask, dstHist, 1, &sizeHist, ranges);
			
			// 选择目标颜色，且角度在范围内的RotatedRect
			if ((JudgeColor(dstHist) == detectColor) && !(rotatedRects[i].angle > 80 && rotatedRects[i].angle < 100))
				rotatedRectsOfLights.push_back(rotatedRects[i]);
		}
		
 		for (int i = 0; i < rotatedRectsOfLights.size(); i++) {
 			ellipse(frame_, rotatedRectsOfLights[i], Scalar(0, 255, 0), 2, LINE_AA);
 		}
		
		// 装甲灯条的世界坐标
		vector<Vec3f> coordinateOfLights;
		for (int i = 0; i < rotatedRectsOfLights.size(); i++)
		{
			Vec3f temp;
			//stereoVision.getXYZ(itContours.center, temp);
			stereoVision.getXYZ(rotatedRectsOfLights[i].center, temp);
			coordinateOfLights.push_back(temp);
		}

// 		for (int i = 0; i < coordinateOfLights.size(); i++) {
// 			cout << coordinateOfLights[i] << endl;
// 		}

		// 遍历已检测到的灯条，如果其中两个灯条的世界坐标系距离在一定范围内，则认为是某个装甲板上的一对灯条
		if (coordinateOfLights.size() > 1) {
			for (int i = 0; i < coordinateOfLights.size() - 1; i++) {
				for (int j = i + 1; j < coordinateOfLights.size(); j++) {
					if (isPair(coordinateOfLights[i], coordinateOfLights[j])) {
						circle(frame_, rotatedRectsOfLights[i].center, 3, Scalar(0, 0, 255), -1, LINE_AA);
						circle(frame_, rotatedRectsOfLights[j].center, 3, Scalar(0, 0, 255), -1, LINE_AA);
						Point p = centerOf2Points(rotatedRectsOfLights[i].center, rotatedRectsOfLights[j].center);
						circle(frame_, p, 10, Scalar(0, 0, 255), 2, LINE_AA);						
						disX = (coordinateOfLights[i][0] + coordinateOfLights[j][0]) / 2;
						disY = (coordinateOfLights[i][1] + coordinateOfLights[j][1]) / 2;
						disZ = (coordinateOfLights[i][2] + coordinateOfLights[j][2]) / 2;
						distanceValue = disZ;
						//fout << disX << " " << disY << " " << disZ << endl;
						//cout << i << "  " << j << endl;
					}
				}
			}
		}
		//cout << "--------------" << endl;
		char distance[20];  // 用于存放距离值的字符串
		snprintf(distance, 20, "%fmm", distanceValue);
		putText(frame_, distance, Point(10, 30), CV_FONT_HERSHEY_DUPLEX, 0.8, Scalar(0, 0, 255), 1, LINE_AA);
		cv::line(frame_, Point(366-10, 238-10), Point(366+10, 238+10), Scalar(67, 206, 255), 2, LINE_AA);
		cv::line(frame_, Point(366-10, 238+10), Point(366+10, 238-10), Scalar(67, 206, 255), 2, LINE_AA);
		
		float yawAngle = atan(disX / lastCoordinate[2]) / CV_PI * 180;
		float pitchAngle = atan(disY / lastCoordinate[2]) / CV_PI * 180;
		char yawAngleC[20];
		snprintf(yawAngleC, 20, "yaw=%f", yawAngle);
		putText(frame_, yawAngleC, Point(10, 50), CV_FONT_HERSHEY_DUPLEX, 0.8, Scalar(0, 0, 255), 1, LINE_AA);
		char pitchAngleC[20];
		snprintf(pitchAngleC, 20, "pitch=%f", pitchAngle);
		putText(frame_, pitchAngleC, Point(10, 70), CV_FONT_HERSHEY_DUPLEX, 0.8, Scalar(0, 0, 255), 1, LINE_AA);
		
		imshow(WINNAME, frame_);

		if (showBinaryImage)
			imshow(WINNAME1, binaryImage_);

		int key = waitKey(1);
		if (key == 27)
		{
			break;
		} 
		else if (key == int('0'))
		{
			if (showBinaryImage)	// 如果这时候窗口是显示的，则关闭它
				destroyWindow(WINNAME1);
			showBinaryImage = !showBinaryImage;
		}
		else if (key == int('1'))
		{
			waitKey(0);
		}

		time0 = ((double)getTickCount() - time0) / getTickFrequency();


		lastCoordinate = Vec3f(disX, disY, disZ);
		//cout << "用时" << time0 * 1000 << "毫秒" << endl;
	}

	return 0;
}
