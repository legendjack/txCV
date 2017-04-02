#pragma once
#ifndef __ARUCOMARKER_H__
#define __ARUCOMARKER_H__

#include <opencv.hpp>
#include <aruco.hpp>

using namespace std;
using namespace cv;
using namespace aruco;

/* @brief ��ʾ��ά����࣬Ĭ�Ϲ��캯��ʹ�õ�DictionaryΪDICT_6X6_250
 *		ʹ��ArUcoMarker::detect(Mat img)������ͼ���еĶ�ά����м��
 *		�ṩ��һϵ�е�get��������ȡ�����
 */
class ArUcoMarker {
private:
	PREDEFINED_DICTIONARY_NAME name;
	Ptr<Dictionary> dictionary;

	int num;							// ͼ���ж�ά��ĸ���
	vector<vector<Point2f> > corners;	// ÿ����ά����ĸ��ǵ���ɵ�vector
	vector<int> ids;					// ÿ����ά����Dictionary�е�id��ɵ�vector
	vector<Vec3d> rvecs, tvecs;			// ÿ����ά�����ת������ƽ������

public:

	/* @brief Ĭ�Ϲ��캯����Ĭ�ϵ�Dictionary��DICT_6X6_250
	 */
	ArUcoMarker();
	ArUcoMarker(PREDEFINED_DICTIONARY_NAME name_);
	~ArUcoMarker();

	/* @brief ���ͼ�еĶ�ά�룬�õ�ÿ����ά����ĸ��ǵ�corners�Ͷ�Ӧ�ı��ids��
	 *		���ͼ���д��ڶ�ά�룬�򷵻�true�����򷵻�false��
	 *		Ҳ����ͨ��getMarkersNum()��ö�ά�����Ŀ���ж��Ƿ���ڶ�ά��
	 * @param img ������ͼ��
	 */
	bool detect(Mat img);

	/* @brief ����ÿ����ά�����̬���õ�ÿ����ά�����ת������ƽ������rvecs, tvecs
	 * @param markerLength ��ά��ʵ�ʵı߳�����λΪ��
	 */
	void estimatePose(float markerLength, Mat cameraMatrix, Mat distCoeffs);

	/* @brief ����ÿ����ά���λ�ã���Ҫ��ִ��estimatePose()����
	 * @param showAxis���Ƿ��ڶ�ά���ϻ�������
	 */
	void drawMarkers(Mat img, Mat cameraMatrix, Mat distCoeffs, bool showAxis = false);

	// ���ͼ���ж�ά��ĸ���
	int getMarkersNum();

	// ���ÿ����ά����ĸ��ǵ���ɵ�vector
	void getMarkersCorners(vector<vector<Point2f> >& corners_);

	// ���ÿ����ά����Dictionary�е�id��ɵ�vector
	void getMarkersIds(vector<int>& ids_);

	// ���ÿ����ά�����ת������ƽ������
	void getMarKersRvecsAndTvecs(vector<Vec3d>& rvecs_, vector<Vec3d>& tvecs_);
};

ArUcoMarker::ArUcoMarker() {
	ArUcoMarker::name = DICT_6X6_250;
	ArUcoMarker::dictionary = getPredefinedDictionary(DICT_6X6_250);
	ArUcoMarker::num = 0;
	ArUcoMarker::corners.clear();
	ArUcoMarker::ids.clear();
	ArUcoMarker::rvecs.clear();
	ArUcoMarker::tvecs.clear();
}

ArUcoMarker::ArUcoMarker(PREDEFINED_DICTIONARY_NAME name_) {
	ArUcoMarker::name = name_;
	ArUcoMarker::dictionary = getPredefinedDictionary(name_);
	ArUcoMarker::num = 0;
	ArUcoMarker::corners.clear();
	ArUcoMarker::ids.clear();
	ArUcoMarker::rvecs.clear();
	ArUcoMarker::tvecs.clear();
}

ArUcoMarker::~ArUcoMarker() {
	
}

bool ArUcoMarker::detect(Mat img) {
	detectMarkers(img, dictionary, corners, ids);
	ArUcoMarker::num = static_cast<int>(ids.size());
	if (num > 0)
		return true;
	else
		return false;
}

void ArUcoMarker::estimatePose(float markerLength, Mat cameraMatrix, Mat distCoeffs) {
	if (num > 0)
	{
		estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);
	}
	else
	{
		//cout << "No markers!" << endl;
	}
}

void ArUcoMarker::drawMarkers(Mat img, Mat cameraMatrix, Mat distCoeffs, bool showAxis) {
	if (ArUcoMarker::num > 0 && ArUcoMarker::rvecs.size() > 0)
	{
		drawDetectedMarkers(img, ArUcoMarker::corners, ArUcoMarker::ids);
		if (showAxis)
			for (int i = 0; i < num; i++)
				drawAxis(img, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], float(0.03));
	}
	else
	{
		//cout << "No markers!" << endl;
	}
}

int ArUcoMarker::getMarkersNum() {
	return ArUcoMarker::num;
}

void ArUcoMarker::getMarkersCorners(vector<vector<Point2f> >& corners_) {
	corners_ = ArUcoMarker::corners;
}

void ArUcoMarker::getMarkersIds(vector<int>& ids_) {
	ids_ = ArUcoMarker::ids;
}

void ArUcoMarker::getMarKersRvecsAndTvecs(vector<Vec3d>& rvecs_, vector<Vec3d>& tvecs_) {
	rvecs_ = ArUcoMarker::rvecs;
	tvecs_ = ArUcoMarker::tvecs;
}

#endif