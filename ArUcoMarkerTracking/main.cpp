#include <pthread.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "ArUcoMarker.h"
#include "serialsom.h"
#include "MyQueue.h"

using namespace cv;
using namespace std;

#define Width	800				// 视频宽
#define Height	600				// 视频高

const Mat cameraMatrixL = (Mat_<double>(3, 3) << 958.7326269471805, 0.0, 414.90474446730957,
	0.0, 963.1929585405643, 307.1677431460919, 0., 0., 1.);
const Mat distCoeffL = (Mat_<double>(1, 5) << 0.09925244594814076, -0.18192645110674135,
	-4.843643319098382E-4, 0.0021184953031879723, 0.0);

VideoCapture cap;
Mat frame;
Point targetPoint(566, 315);
Point centerOfArmor;
MyQueue mq(20);
bool isUniformSpeed = false;
uint8_t yawOut = 250;
uint8_t pitchOut = 250;

void* thread(void *arg)
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

int main()
{
	cap.open(1);
	cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
	cap.set(CAP_PROP_FRAME_WIDTH, 800);
	cap.set(CAP_PROP_FRAME_HEIGHT, 600);

    //pthread_t id;

	//int ret = pthread_create(&id,NULL, thread,NULL);

	Serialport Serialport1("/dev/ttyUSB0");
	int fd = Serialport1.open_port("/dev/ttyUSB0");
	if (fd >= 0)
		Serialport1.set_opt(115200, 8, 'N', 1);
 	else
		cout << "open serialport : failed" << endl;

	ArUcoMarker marker;

    namedWindow("frame");
	setMouseCallback("frame", on_Mouse);

	while(true) {
		double time0 = static_cast<double>(getTickCount());

		cap >>frame;
		if(!frame.empty()) {

			marker.detect(frame);

			if (marker.getMarkersNum() > 0) {
				vector<vector<Point2f> > corners;
				marker.getMarkersCorners(corners);
				centerOfArmor.x = corners[0][0].x;
				centerOfArmor.y = corners[0][0].y;

				circle(frame, centerOfArmor, 10, Scalar(0, 0, 255), 2, LINE_AA);

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

				if(!isUniformSpeed)
					mq.push(disX);

				// 如果目标不在匀速运动的状态，则状态值设置10
				int status = 10;

				/* 如果目标在匀速移动，且disX大于110，则发送状态值20，云台加速追赶
				 * 如果目标在匀速移动，且disX小于110（已经追赶上），则发送状态值15，云台匀速移动
				 */
				if (mq.dataSize == 20 && mq.min > 145 && mq.max < 170 && disX >= 110) {
					status = 20;
					isUniformSpeed = true;
				}
				else if (mq.dataSize == 20 && mq.min > 145 && mq.max < 170
					&& disX < 110 && disX > 90 ) {
					status = 15;
					isUniformSpeed = true;
				}
				else if (mq.dataSize == 20 && mq.min > 145 && mq.max < 170 && disX < 90 ) {
					status = 10;
					isUniformSpeed = false;
				}

				if (mq.dataSize == 20 && mq.min > 25 && mq.max < 65 && disX <= 90) {
					status = 0;
					isUniformSpeed = true;
				}
				else if (mq.dataSize == 20 && mq.min > 25 && mq.max < 65
					&& disX > 90 && disX < 110) {
					status = 5;
					isUniformSpeed = true;
				}
				else if (mq.dataSize == 20 && mq.min > 25 && mq.max < 65 && disX > 110) {
					status = 10;
					isUniformSpeed = false;
				}

				yawOut = static_cast<uint8_t>(disX);
				pitchOut = static_cast<uint8_t>(disY);

				if (fd >= 0)
					Serialport1.usart3_send(pitchOut, yawOut, static_cast<uint8_t>(status));
			} else {
                if (fd >= 0)
					Serialport1.usart3_send(static_cast<uint8_t>(250), static_cast<uint8_t>(250), static_cast<uint8_t>(10));
			}

			imshow("frame", frame);
			waitKey(1);
		}
		time0 = ((double)getTickCount() - time0) / getTickFrequency();
		cout << "time : " << time0 * 1000 << "ms"  << endl;
	}
	return 0;
}
