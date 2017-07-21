//#include <pthread.h>
#include <sstream>
#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "functions.h"
#include "getConfig.h"
#include "serialsom.h"
//#include "MyQueue.h"

// �궨��
#define WINNAME	"Armor Recognition"
#define WINNAME1 "Binary Image"
#define MaxContourArea 450		// ������ڸ�ֵ����������װ�׵ĵ���
#define MinContourArea 25		// ���С�ڸ�ֵ����������װ�׵ĵ���
#define Width	800				// ��Ƶ��
#define Height	600				// ��Ƶ��
//#define DEBUG
#define SEND

// ȫ�ֱ���
VideoCapture cap;
VideoWriter writer;
map<string, string> config;
Mat frame, gray;				// ��Ƶ֡����Ҷ�ͼ
Mat binaryImage, hsvImage;		// ��ֵͼ��HSVͼ��ʹ��cvtColor�õ�
Mat element, element1;			// ���������
string fileName;				// ��Ƶ���ļ���
bool showBinaryImage = false;	// �Ƿ���ʾ��ֵͼ
int key = 0;					// waitKey() ����ֵ
int m_threshold;				// �Ҷ���ֵ
int detectColor;				// �о�װ�׵���ɫ��0-��ɫ��1-��ɫ
float tmpAngle0;
float tmpAngle1;
uint8_t yawOut = 250;			// ���͵� yaw ֵ
uint8_t pitchOut = 250;			// ���͵� pitch ֵ
uint8_t shot = 0;				// �Ƿ����
int frameCount = 450;
bool findArmor;					// �Ƿ��⵽װ��
Point targetPoint(370, 340);
Point centerOfArmor;
float areaOfLightContour = 0;	// ĳ����������������ڴ��±�ʾĿ��װ�׵�Զ����
SearchWindow searchWindow(Width, Height);		// ��������
bool useSW = false;				// �Ƿ�ʹ����������
int swSizeCache[5] = { 0,0,0,0,0 }; // ���� 5 ֡�������ڵĳߴ磨��
int recordVideo = 0;			// �Ƿ�¼��
int videoName;					// ¼���ļ���

Mat bMat, gMat, rMat;			// BGR��ͨ��ͼ

void on_Mouse(int event, int x, int y, int flags, void*);
void* capFrameThread(void *arg);

int main()
{
	sleep(1);

#ifdef SEND
	// ��ʼ��������
	Serialport Serialport1("/dev/ttyUSB0");
	int fd = Serialport1.open_port("/dev/ttyUSB0");
	while (fd < 0) {
		sleep(1);
		fd = Serialport1.open_port("/dev/ttyUSB0");
	}
	Serialport1.set_opt(115200, 8, 'N', 1);
#endif 

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
	recordVideo = atoi(config["RECORDVIDEO"].c_str());

	element = getStructuringElement(MORPH_ELLIPSE, Size(2, 2));
	element1 = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
	bMat.create(Size(Width, Height), CV_8UC1);
	gMat.create(Size(Width, Height), CV_8UC1);
	rMat.create(Size(Width, Height), CV_8UC1);
	Mat chan[3] = { bMat, gMat, rMat };

	vector<RotatedRect> rotatedRects;			// �������ָ����Χ�ڵ����������Բ���õ���Ӧ����ת����
	vector<RotatedRect> rotatedRectsOfLights;	// ��ɫ/��ɫ������RotatedRect

#ifdef DEBUG
	namedWindow(WINNAME, WINDOW_AUTOSIZE);
	createTrackbar("Threshold", WINNAME, &m_threshold, 255, 0);
	setMouseCallback(WINNAME, on_Mouse);		// �����Ӧ������ȡtargetPoint
#endif

	//VideoCapture cap("1.avi");
	cap.open(0);
	while (!cap.isOpened()) {
		sleep(1);
		cap.open(0);
	}
	//cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));	// ��Ҫ�����ÿ��֮ǰ���ã�������Ч
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

	// ������ȡ��Ƶ֡���̣߳�ò�Ƴ�������Щ����������л���˱�����
/*	pthread_t id;
	int ret = pthread_create(&id, NULL, capFrameThread, NULL);
	if (!ret) {
		cout << "open thread to capture frame: success" << endl;
	} else {
		cout << "open thread to capture frame: failed" << endl;
	}*/

	/***************************************
				��ʼ����ÿһ֡
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

		// �����ǰ֡ʹ���������ڣ��������Ŀ������ͼ�����������Ϊ��ɫ
		if (useSW) {
			Mat frameTemp(Height, Width, CV_8UC3, Scalar(0, 0, 0));
			frame(searchWindow.getRect()).copyTo(frameTemp(searchWindow.getRect()));
			frame = frameTemp;
		}
		
#ifdef DEBUG
		Mat frame_ = frame.clone();		// ֡ͼ�񱸷ݣ�������
#endif // DEBUG

		split(frame, chan);

		cvtColor(frame, gray, COLOR_BGR2GRAY);

		threshold(gray, binaryImage, m_threshold, 255, THRESH_BINARY);

		morphologyEx(binaryImage, binaryImage, MORPH_OPEN, element);	// �����㣬�ȸ�ʴ�������͡�ȥ��С�İ�ɫ����

#ifdef DEBUG
		Mat binaryImage_ = binaryImage.clone();		// ��ֵͼ�񱸷ݣ�������
#endif

		// Ѱ��������ע������Ĳ�����Ϊ CHAIN_APPROX_SIMPLE����Ϊ�������������ã���ʾһ�������ĵ��������������4
		// �� fitEllipse ����Ҫ������������С�� 4 ������ᱨ��
		vector<vector<Point> > contours;		// ��������
		findContours(binaryImage, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);

		vector<vector<Point> > contoursInAreaRange;	// �����(MinContourArea, MaxContourArea)��Χ�ڵ�����
		for (int i = 0; i < contours.size(); i++) {
			double areaTemp = contourArea(contours[i]);
			if (areaTemp > MinContourArea && areaTemp < MaxContourArea)
				contoursInAreaRange.push_back(contours[i]);
		}

		// ��������ָ����Χ�ڵ���������С��2���������һ��ѭ����ע����������ǳ��٣�
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

		// �õ���������ת���� RotatedRect
		// Ϊÿһ������������ RotatedRect ����һ����ģ���ж���ģ�������Ҫ��ɫ�Ǻ�ɫ������ɫ
		rotatedRectsOfLights.clear();
		for (int i = 0; i < contoursInAreaRange.size(); i++) {
			RotatedRect rotatedRectTemp = fitEllipse(contoursInAreaRange[i]);
			float tmpA = rotatedRectTemp.angle;		// ��ת���εĽǶ�
			if (tmpA > 25 && tmpA < 155)			// ��ת���εĽǶ��Ƿ���ָ����Χ��
				continue;

			Mat mask(Height, Width, CV_8UC1, Scalar(0));
			drawContours(mask, contoursInAreaRange, i, Scalar(255), -1);	// ���ƣ������ģ���ģ
			dilate(mask, mask, element1);
			Mat mask1;
			dilate(mask, mask1, element1);
			bitwise_not(mask, mask);
			Mat mask2;								// ������ģ����ɫ������Χ�Ĺ��
			bitwise_and(mask, mask1, mask2);

			bool b1 = false;
			if (detectColor)
				b1 = JudgeColor(bMat, rMat, 90, mask2);		// ��ɫװ��
			else
				b1 = JudgeColor(rMat, bMat, 60, mask2);		// ��ɫװ��

			if (b1) {
				rotatedRectsOfLights.push_back(rotatedRectTemp);
			}
		}

		/* �����⵽��������������0������δ��⵽Ŀ����źţ�������һ֡����������϶ࣩ
		 * ���֮ǰ��⵽��װ�ף�frameCount=0���������ֳ��ֵ�������Ϊ0�����
		 * �����Ƕ��沽��������ܡ����˻����ƶ���������̨��ֹ�������źţ��ȴ������ٷ�����̨��������ģʽ���ź�
		 */
		if (rotatedRectsOfLights.size() == 0) {
			if (frameCount > 9)
				useSW = false;
			
			// ���ĳһ֡��ʼû�м�⵽װ�ף�frameCount�Լ�һ
            frameCount++;
            if (frameCount <= 60) {
				pitchOut = 100;
				yawOut = 100;
				shot = 0;
            } else if (frameCount > 60) {
				// �������60֡û�м�⵽װ�ף����϶�Ϊû��Ŀ�꣬���ڷ�����Ϣ��������ģʽ
				frameCount--;
                pitchOut = 250;
				yawOut = 250;
				shot = 0;
			}
			goto HERE;
		}

#ifdef DEBUG
		// ����ÿ�������������Բ
		for (int i = 0; i < rotatedRectsOfLights.size(); i++)
			ellipse(frame_, rotatedRectsOfLights[i], Scalar(0, 255, 0), 2, LINE_AA);
#endif

        tmpAngle0 = 10;		// ���������ĽǶȲ����ֵ������������������// �� \\ ����ǶȲ���10�����п�����һ��װ��
		tmpAngle1 = 170;	// ��������������/\ �� \/ ����ǶȲ����170�ȿ�����һ��װ��
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
					yDifference < 10) {
#ifdef DEBUG
					circle(frame_, rotatedRectsOfLights[i].center, 3, Scalar(0, 0, 255), -1, LINE_AA);
					circle(frame_, rotatedRectsOfLights[j].center, 3, Scalar(0, 0, 255), -1, LINE_AA);
#endif
					centerOfArmor = centerOf2Points(rotatedRectsOfLights[i].center, rotatedRectsOfLights[j].center);
					
					// ���� tmpAngle0 �� tmpAngle1��Ŀ�����ҵ��ǶȲ���С��һ�Ե�������Ϊ��������һ��װ��
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

			yawOut = static_cast<uint8_t>(disX);
			pitchOut = static_cast<uint8_t>(disY);
			shot = 1;

		} else {
			
			// ��⵽��������û��ƥ�䵽װ��
			frameCount++;
			if (frameCount > 6)
				useSW = false;
			
			if (frameCount > 10) {
				int disX, disY;
				
				// ���û�м�⵽װ�ף��һ����е�������������2,���򴰿ڷ��͵�����λ����Ϣ
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
			if (showBinaryImage)	// �����ʱ�򴰿�����ʾ�ģ���ر���
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
