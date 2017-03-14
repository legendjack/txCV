#pragma once
#ifndef _STEREOVISION_H_
#define _STEREOVISION_H_

#include <opencv.hpp>

using namespace std;
using namespace cv;

class StereoVision {
private:
	Size imageSize;					// 图像尺寸（宽高，像素）

	Rect validROIL, validROIR;		// 图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域  
  
	Mat Rl, Rr, Pl, Pr, Q;			// 校正旋转矩阵R，投影矩阵P 重投影矩阵Q
	Mat mapLx, mapLy, mapRx, mapRy;	// 映射表

	Mat disp, disp8;
	Mat XYZ;						// 三维坐标

	Ptr<StereoBM> bm;				// 立体匹配算法，BM块匹配。StereoBM::create()创建对象，bm->compute()计算匹配

public:
	/* @brief 默认构造函数
	 */
	StereoVision();

	/* @brief 构造函数
	 * @param cameraMatrixL_, cameraMatrixR_ 双目相机的相机矩阵
	 * @param distCoeffL_, distCoeffR_ 双目相机的畸变系数
	 * @param T_ 双目相机的平移向量
	 * @param R_ 双目相机的旋转矩阵
	 * @param imageWidth_, imageHeight_ 图像宽高
	 */
	StereoVision(Mat cameraMatrixL_, Mat distCoeffL_, Mat cameraMatrixR_, Mat distCoeffR_, Mat T_, Mat R_, int imageWidth_, int imageHeight_);

	/* @brief 析构函数
	 */
	~StereoVision();

	/* @brief 立体匹配，必须在执行其他get函数之前执行该函数
	 * @param grayImageL_, grayImageR_ 双目相机图像，CV_8UC1
	 * @param blockSize_ SAD窗口大小，5~21之间为宜
	 * @param uniquenessRatio_ 视差窗口，即最大视差值与最小视差值之差，小于16
	 * @param numDisparities_ uniquenessRatio主要可以防止误匹配，小于50
	 */
	void stereoMatch(Mat grayImageL_, Mat grayImageR_);
	
	/* @brief 获得指定点p_对应的世界坐标系XYZ_
	 */
	void getXYZ(Point p_, Vec3f &XYZ_);

	/* @brief 获得深度信息图像
	 */
	void getDisparityImage(Mat &disp_);

	void getMaps(Mat& mapLx_, Mat& mapLy_, Mat& mapRx_, Mat& mapRy_);

	void setNumDisparities(int num_);
};


StereoVision::StereoVision() {

}

StereoVision::StereoVision(Mat cameraMatrixL_, Mat distCoeffL_, Mat cameraMatrixR_, Mat distCoeffR_, Mat T_, Mat R_, int imageWidth_, int imageHeight_) {
	imageSize = Size(imageWidth_, imageHeight_);

	bm = StereoBM::create();

	stereoRectify(cameraMatrixL_, distCoeffL_, cameraMatrixR_, distCoeffR_, imageSize, R_, T_, Rl, Rr, Pl, Pr, Q,
		CALIB_ZERO_DISPARITY, 0, imageSize, &validROIL, &validROIR);
	initUndistortRectifyMap(cameraMatrixL_, distCoeffL_, Rl, Pl, imageSize, CV_32FC1, mapLx, mapLy);
	initUndistortRectifyMap(cameraMatrixR_, distCoeffR_, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);
	
	bm->setBlockSize(9);			// 窗口大小，5~21之间为宜
	bm->setROI1(validROIL);
	bm->setROI2(validROIR);
	bm->setPreFilterCap(31);
	bm->setMinDisparity(0);			// 最小视差，默认值为0, 可以是负值，int型
	bm->setNumDisparities(64);		// 视差窗口，即最大视差值与最小视差值之差，窗口大小必须是16的整数倍
	bm->setTextureThreshold(10);
	bm->setUniquenessRatio(1);		// uniquenessRatio主要可以防止误匹配
	bm->setSpeckleWindowSize(100);
	bm->setSpeckleRange(32);
	bm->setDisp12MaxDiff(-1);
}

StereoVision::~StereoVision() {

}

void StereoVision::stereoMatch(Mat grayImageL_, Mat grayImageR_) {
	bm->compute(grayImageL_, grayImageR_, disp);	// 输入图像必须为灰度图，计算出的视差是CV_16S格式
	disp.convertTo(disp8, CV_8U, 15.9375 / 176);	// 用于显示深度信息的图像，可以去掉
	reprojectImageTo3D(disp, XYZ, Q, true);
	XYZ = XYZ * 16;
}

void StereoVision::getXYZ(Point p_, Vec3f &XYZ_) {
	XYZ_ = XYZ.at<Vec3f>(p_);
// 	Mat xyz1(imageSize.height, imageSize.width, CV_32FC3, Scalar::all(0));
// 	XYZ.copyTo(xyz1(Rect(0, 180, 640, 160)));
// 	XYZ_ = xyz1.at<Vec3f>(p_);
}

void StereoVision::getDisparityImage(Mat &disp_) {
	disp_ = disp8;
}

void StereoVision::getMaps(Mat& mapLx_, Mat& mapLy_, Mat& mapRx_, Mat& mapRy_) {
	mapLx_ = mapLx;
	mapLy_ = mapLy;
	mapRx_ = mapRx;
	mapRy_ = mapRy;
}

void StereoVision::setNumDisparities(int num_) {
	bm->setNumDisparities(num_);
}

#endif