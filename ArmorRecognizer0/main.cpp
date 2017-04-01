/* version 1.3
 * ��߷ֱ��ʣ�800*600
 * ���֡�ʣ�cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
 *			 cap.set(CAP_PROP_FPS, 60);
 * ��߱��Ͷȣ�cap.set(CV_CAP_PROP_SATURATION, 80);
 * ��߱��ͶȺͷֱ��ʿ�����߼��װ�׵ľ���
 *
 * version 1.2
 * �޸�BUG��û�м�⵽װ�ף���̨�ͻ����̹�λ��BUG
 * ���Ԥ��
 * 
 * version 1.3
 * ���̣߳�һ���߳�ר�Ŷ�ȡ��Ƶ֡����һ���̴߳�����Ƶ֡
 *
 * version 1.4
 * ȥ����KalmanFilter��KF����Ŀ������֡���λ�õõ��ٶȣ�������̨ת���ȶ���Ŀ��
 * �ڻ����е�λ����Թ̶�����Ϊ����ͷ�ǹ̶�����̨�ϵģ�������������ٶ�Ϊ0������KFʧЧ
 * ��������Ӧ��ȡtargetPoint
 */

#include <pthread.h>
#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "functions.h"
#include "getConfig.h"
#include "serialsom.h"
#include "MyQueue.h"

// �궨��
#define WINNAME	"Armor Recognition"
#define WINNAME1 "Binary Image"
#define MaxContourArea 450		// ������ڸ�ֵ����������װ�׵ĵ���
#define MinContourArea 15		// ���С�ڸ�ֵ����������װ�׵ĵ���
#define Width	800				// ��Ƶ��
#define Height	600				// ��Ƶ��
#define DEBUG

// ȫ�ֱ���
VideoCapture cap;
map<string, string> config;
Mat frame, gray;				// ��Ƶ֡����Ҷ�ͼ
Mat binaryImage, hsvImage;		// ��ֵͼ��HSVͼ��ʹ��cvtColor�õ�
Mat element, element1;			// ���������
string fileName;				// ��Ƶ���ļ���
int m_threshold;				// ��ֵ
bool showBinaryImage = false;	// �Ƿ���ʾ��ֵͼ
int detectColor;				// �о�װ�׵���ɫ��0-��ɫ��1-��ɫ
//int disX, disY, disZ;
float tmpAngle0;
float tmpAngle1;
uint8_t yawOut = 250;
uint8_t pitchOut = 250;
int frameCount = 150;
int lightsCount = 0;			// ͼ����װ�׵���������
bool findArmor;
bool sended;					// ������Ϣ�Ƿ��Ѿ�����
Point targetPoint(566, 315);
Point centerOfArmor;
MyQueue mq(20);
//Point predictPoint;				// Ԥ��װ�װ��λ��

// ����ֱ��ͼ��Ҫ�Ĳ���
Mat hMat, sMat, vMat;			// HSV��ͨ��ͼ
int channels = 0;				// �����0��ͨ����ֱ��ͼ��calcHist����
int sizeHist = 180;				// 180��ɫ�ȣ�calcHist����
MatND dstHist;					// calcHist���

// �������˲�������
const int stateNum = 4;			// ״ֵ̬4��1����(x,y,��x,��y)
const int measureNum = 2;		// ����ֵ2��1����(x,y)
Mat measurement = Mat::zeros(measureNum, 1, CV_32F); // ��ʼ����ֵx'(0)����Ϊ����Ҫ�������ֵ�����Ա����ȶ���

void on_Mouse(int event, int x, int y, int flags, void*);
void* capFrameThread(void *arg);

int main()
{
	//showText();

	// ��ʼ��������
	Serialport Serialport1("/dev/ttyUSB0");
	int fd = Serialport1.open_port("/dev/ttyUSB0");
	if (fd >= 0)
		Serialport1.set_opt(115200, 8, 'N', 1);
 	else
		cout << "open serialport : failed" << endl;

	/***************************************
			��ȡ video.cfg ����ļ�ֵ��
	****************************************/
	bool read = ReadConfig("video.cfg", config);	// ��video.cfg����config��ֵ�ԣ��ɹ�����true
	if (!read) {
		cout << "cannot open file : video.cfg" << endl;
		return -1;
	}

	/***************************************
				ȫ�ֱ�����ʼ��
	****************************************/
	m_threshold = atoi(config["THRESHOLD"].c_str());
	detectColor = atoi(config["DETECTCOLOR"].c_str());
	fileName = config["FILENAME"].c_str();

	element = getStructuringElement(MORPH_ELLIPSE, Size(2, 2));
	element1 = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
	hMat.create(Size(Width, Height), CV_8UC1);
	sMat.create(Size(Width, Height), CV_8UC1);
	vMat.create(Size(Width, Height), CV_8UC1);
	Mat chan[3] = { hMat, sMat, vMat };
	float hranges[] = { 0, 180 };
	const float *ranges[] = { hranges };

	vector<RotatedRect> rotatedRects;			// �������ָ����Χ�ڵ����������Բ���õ���Ӧ����ת����
	vector<RotatedRect> rotatedRectsOfLights;	// ��ɫ/��ɫ������RotatedRect

/*
	// KalmanFilter��ʼ��
	KalmanFilter kf(stateNum, measureNum, 0);
	int t = 20;
	int t1 = 100;
	kf.transitionMatrix = (Mat_<float>(4, 4) <<
		1, 0, t, 0,
		0, 1, 0, t,
		0, 0, 1, 0,
		0, 0, 0, 1);
	setIdentity(kf.measurementMatrix);						// ��������H��setIdentity�����ǳ�ʼ�����Խ��ߵ�ֵ
	setIdentity(kf.processNoiseCov, Scalar::all(1e-5));		// ϵͳ�����������Q
	setIdentity(kf.measurementNoiseCov, Scalar::all(1e-1));	// ���������������R
	setIdentity(kf.errorCovPost, Scalar::all(1));			// ����������Э�������P
*/

#ifdef DEBUG
	namedWindow(WINNAME, WINDOW_AUTOSIZE);
	createTrackbar("Threshold", WINNAME, &m_threshold, 255, 0);
	//createTrackbar("t1", WINNAME, &t1, 200);
	setMouseCallback(WINNAME, on_Mouse);					// �����Ӧ������ȡtargetPoint
#endif

	//VideoCapture cap(fileName);
	cap.open(1);
	cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));	// ��Ҫ�����ÿ��֮ǰ���ã�������Ч
	cap.set(CV_CAP_PROP_SATURATION, 80);
	cap.set(CAP_PROP_FRAME_WIDTH, Width);
	cap.set(CAP_PROP_FRAME_HEIGHT, Height);

	if (!cap.isOpened()) {
		cout << "VideoCapture initialize : failed" << endl;
		return -1;
	}

	// ������ȡ��Ƶ֡���߳�
	pthread_t id;
	int ret = pthread_create(&id, NULL, capFrameThread, NULL);
	if (!ret) {
		cout << "open thread to capture frame: success" << endl;
	} else {
		cout << "open thread to capture frame: failed" << endl;
	}
	
	/***************************************
				��ʼ����ÿһ֡
	****************************************/
	while (true)
	{
		// double time0 = static_cast<double>(getTickCount());
		
		sended = false;
		
		if (frame.empty())
			continue;

#ifdef DEBUG
		Mat frame_ = frame.clone();		// ֡ͼ�񱸷ݣ�������
#endif // DEBUG

		cvtColor(frame, gray, COLOR_BGR2GRAY);

		cvtColor(frame, hsvImage, COLOR_BGR2HSV);

		threshold(gray, binaryImage, m_threshold, 255, THRESH_BINARY);

		morphologyEx(binaryImage, binaryImage, MORPH_OPEN, element);	// �����㣬�ȸ�ʴ�������͡�ȥ��С�İ�ɫ����

#ifdef DEBUG
		Mat binaryImage_ = binaryImage.clone();  // ��ֵͼ�񱸷ݣ�������
#endif
		vector<vector<Point> > contours;		// ����������findContours�����Ľ��
		findContours(binaryImage, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);	// Ѱ������

		vector<vector<Point> > contoursInAreaRange;	// �����(MinContourArea, MaxContourArea)��Χ�ڵ�����
		for (int i = 0; i < contours.size(); i++) {
			double areaTemp = contourArea(contours[i]);
			if (areaTemp > MinContourArea && areaTemp < MaxContourArea)
				contoursInAreaRange.push_back(contours[i]);
		}

		// ��������ָ����Χ�ڵ���������С��2���������һ��ѭ��
		if (contoursInAreaRange.size() < 2) {
			yawOut = 250;
			pitchOut = 250;
			goto HERE;
		}

		// �������ָ����Χ�ڵ����������Բ���õ���Ӧ����ת����
		rotatedRects.clear();
		for (int i = 0; i < contoursInAreaRange.size(); i++)
			rotatedRects.push_back(fitEllipse(contoursInAreaRange[i]));

		/* Ϊÿһ������������RotatedRect����һ����ģ��Ȼ�������ģ�����ֱ��ͼ��
		 * ͨ��ֱ��ͼ�жϸ�RotatedRect����Ҫ��ɫ�Ǻ�ɫ������ɫ
		 */
		rotatedRectsOfLights.clear();
		split(hsvImage, chan);						// ��HSVͼ���Ϊ3��ͨ�������ڼ���ֱ��ͼ
		for (int i = 0; i < rotatedRects.size(); i++) {
			Point2f pointTemp[4];
			rotatedRects[i].points(pointTemp);		// �õ���ת���ε�4���ǵ�
			vector<Point> corners;
			for (int j = 0; j < 4; j++)
				corners.push_back(pointTemp[j]);

			vector<vector<Point> > corners_;
			corners_.push_back(corners);
			Mat mask(Height, Width, CV_8UC1, Scalar::all(0));
			drawContours(mask, corners_, -1, Scalar(255), -1, LINE_AA);	// ������ģ
			dilate(mask, mask, element1);								// ���ʹ���
			calcHist(&hMat, 1, &channels, mask, dstHist, 1, &sizeHist, ranges);	// ������ģ��ֱ��ͼ
			
			float tmpA = rotatedRects[i].angle;
			float HdivideW = rotatedRects[i].size.height / rotatedRects[i].size.width;
			if (HdivideW < 1)
				HdivideW = 1 / HdivideW;

			bool b1 = false, b2 = false;
			if (!(tmpA > 25 && tmpA < 155))
				b1 = true;
			if (HdivideW > 2.5 && HdivideW < 8.0)
				b2 = true;

			if ((JudgeColor(dstHist) == detectColor) && b1) {
				rotatedRectsOfLights.push_back(rotatedRects[i]);
			}
		}

		/* �����⵽��������������0������δ��⵽Ŀ����źţ�������һ֡
		 * ���֮ǰ��⵽��װ�ף�frameCount=0���������ֳ��ֵ�������Ϊ0�����
		 * �����Ƕ��沽��������ܻ��ˣ�������̨��ֹ�������źţ��ȴ������ٷ�����̨��������ģʽ���ź�
		 */
		if (rotatedRectsOfLights.size() == 0) {
			// ���ĳһ֡��ʼû�м�⵽װ�ף�frameCount�Լ�һ
            frameCount++;
            if (frameCount >= 150) {
				// �������150֡û�м�⵽װ�ף����϶�Ϊû��Ŀ�꣬���ڷ�����Ϣ��������ģʽ
                frameCount--;
                pitchOut = 250;
				yawOut = 250;
            } else {
				pitchOut = 100;
				yawOut = 100;
			}
			goto HERE;
		}

#ifdef DEBUG
		// ����ÿ�������������Բ
		for (int i = 0; i < rotatedRectsOfLights.size(); i++)
			ellipse(frame_, rotatedRectsOfLights[i], Scalar(0, 255, 0), 2, LINE_AA);
#endif

        tmpAngle0 = 10;
		tmpAngle1 = 170;
		findArmor = false;
		// Ѱ������һ��װ�׵������������Ӷ�ȷ��װ�׵�λ��
		for (int i = 0; i < rotatedRectsOfLights.size() - 1; i++) {
			for (int j = i + 1; j < rotatedRectsOfLights.size(); j++) {
				float angleDifference = abs(rotatedRectsOfLights[i].angle - rotatedRectsOfLights[j].angle);		// �ƴ��ĽǶȲ�
				float yDifference = abs(rotatedRectsOfLights[i].center.y - rotatedRectsOfLights[j].center.y);	// �ƴ���Y���
				float xDifference = abs(rotatedRectsOfLights[i].center.x - rotatedRectsOfLights[j].center.x);	// �ƴ���X���
				float rotatedRectHeight = rotatedRectsOfLights[i].size.height;
				float rotatedRectWidth = rotatedRectsOfLights[i].size.width;
				if (rotatedRectHeight < rotatedRectWidth)
					exchange(rotatedRectHeight, rotatedRectWidth);
				if (xDifference < rotatedRectHeight*2/3)
					continue;
				if ((angleDifference < 10 || angleDifference > 170) &&
					(angleDifference < tmpAngle0 || angleDifference > tmpAngle1) &&
					yDifference < 7) {
#ifdef DEBUG
					circle(frame_, rotatedRectsOfLights[i].center, 3, Scalar(0, 0, 255), -1, LINE_AA);
					circle(frame_, rotatedRectsOfLights[j].center, 3, Scalar(0, 0, 255), -1, LINE_AA);
#endif
					centerOfArmor = centerOf2Points(rotatedRectsOfLights[i].center, rotatedRectsOfLights[j].center);
					//targetPoint.x = 17 * rotatedRectHeight / 21 + 311;
					if (angleDifference < 10)
						tmpAngle0 = angleDifference;
					else if (angleDifference > 170)
						tmpAngle1 = angleDifference;
					findArmor = true;
				}
			}
		}

		if (findArmor) {
			
			// �����⵽��װ�׵�λ�ã�frameCount���㣬���򴮿ڷ���װ�׵�λ����Ϣ	
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

			mq.push(disX);
			
			// ���Ŀ�겻�������˶���״̬����״ֵ̬����10
			int status = 10;
			
			/* ���Ŀ���������ƶ�����disX����110������״ֵ̬20����̨����׷��
			 * ���Ŀ���������ƶ�����disXС��110���Ѿ�׷���ϣ�������״ֵ̬15����̨�����ƶ�
			 */
			if (mq.dataSize == 20 && mq.min > 145 && mq.max < 170 && disX >= 110)
				status = 20;
			else if (mq.dataSize == 20 && mq.min > 145 && mq.max < 170 && disX < 110)
				status = 15;
			
			if (mq.dataSize == 20 && mq.min > 25 && mq.max < 65 && disX <= 90)
				status = 0;
			else if (mq.dataSize == 20 && mq.min > 25 && mq.max < 65 && disX > 90)
				status = 5;
			
			yawOut = static_cast<uint8_t>(disX);
			pitchOut = static_cast<uint8_t>(disY);

 			if (fd >= 0)
				sended = Serialport1.usart3_send(pitchOut, yawOut, static_cast<uint8_t>(status));	// ������ֱ�����ˮƽ�����ƶ��ٶ�
			
		} else {
			// ��⵽��������û��ƥ�䵽װ��
			frameCount++;
			mq.clear();
			if (frameCount < 10) {
				if (fd >= 0)
					sended = Serialport1.usart3_send(pitchOut, yawOut, static_cast<uint8_t>(10));
			} else { // if (rotatedRectsOfLights.size() >= 2)
				// ���û�м�⵽װ�ף��һ����е�������������2,���򴰿ڷ��͵�����λ����Ϣ
				int disX = rotatedRectsOfLights[0].center.x - targetPoint.x;
				int disY = rotatedRectsOfLights[0].center.y - targetPoint.y;

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

				if (fd >= 0)
					sended = Serialport1.usart3_send(pitchOut, yawOut, static_cast<uint8_t>(10));	// ������ֱ�����ˮƽ�����ƶ��ٶ�
			}
		}

HERE:
		if (!sended)
			Serialport1.usart3_send(pitchOut, yawOut, static_cast<uint8_t>(10));
		time0 = ((double)getTickCount() - time0) / getTickFrequency();
		cout << "time : " << time0 * 1000 << "ms"  << endl;
#ifdef DEBUG		
        cout << static_cast<int>(pitchOut) << ", " << static_cast<int>(yawOut) << endl;
		imshow(WINNAME, frame_);

		if (showBinaryImage)
			imshow(WINNAME1, binaryImage_);

		int key = waitKey(1);
		if (key == 27) {
			break;
		}  else if (key == int('0')) {
			if (showBinaryImage)	// �����ʱ�򴰿�����ʾ�ģ���ر���
				destroyWindow(WINNAME1);
			showBinaryImage = !showBinaryImage;
		} else if (key == int('1')) {
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
