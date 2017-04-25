#pragma once
#ifndef _FUNCTIONS_H_
#define _FUNCTIONS_H_

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

// �Թ�������1~9
void RectSort(vector<Rect>& nineRect_) {
	for (int i = 0; i < 9; ++i)//��y����ð������
		for (int j = 0; j < 8 - i; ++j)
			if (nineRect_[j].y > nineRect_[j + 1].y)
				swap(nineRect_[j], nineRect_[j + 1]);

	for (int k = 0; k < 3; ++k)//ÿ3��һ�飬��x��������
		for (int i = 0; i < 3; ++i)
			for (int j = 0; j < 2 - i; ++j)
				if (nineRect_[3 * k + j].x > nineRect_[3 * k + j + 1].x)
					swap(nineRect_[3 * k + j], nineRect_[3 * k + j + 1]);
}

void sortRect(vector<Rect>& nineRect_) {
	for (int i = 0; i < nineRect_.size() - 1; ++i)//��y����ð������
		for (int j = i + 1; j < nineRect_.size(); ++j)
			if (nineRect_[i].x > nineRect_[j].x)
				swap(nineRect_[i], nineRect_[j]);
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

void sortContours(vector<vector<Point> >& number_contours)
{
	for (int i = 0; i < number_contours.size() - 1; i++)
		for (int j = i + 1; j < number_contours.size(); j++)
			if (number_contours[i][0].x > number_contours[j][0].x)
				swap(number_contours[i], number_contours[j]);
}

void sortRotatedRect(vector<RotatedRect>& contours_rotatedRect_)
{
	for (int i = 0; i < 9; ++i)//��y����ð������
		for (int j = 0; j < 8 - i; ++j)
			if (contours_rotatedRect_[j].center.y > contours_rotatedRect_[j + 1].center.y)
				swap(contours_rotatedRect_[j], contours_rotatedRect_[j + 1]);

	for (int k = 0; k < 3; ++k)//ÿ3��һ�飬��x��������
		for (int i = 0; i < 3; ++i)
			for (int j = 0; j < 2 - i; ++j)
				if (contours_rotatedRect_[3 * k + j].center.x > contours_rotatedRect_[3 * k + j + 1].center.x)
					swap(contours_rotatedRect_[3 * k + j], contours_rotatedRect_[3 * k + j + 1]);
}

// ��⹬��������Ƿ�Ϊ�հף����� 40*40 ��	ROI �а�ɫ���ص������
int sum_mat(Mat m_) {
	int sum_value = 0;
	for (int i = 0; i < 800; i++)
		if (m_.data[i])
			sum_value++;

	return sum_value;
}

// ����һ������ ROI ����͵ĻҶ�ֵ
int  min_mat(Mat m_) {
	int min_value = 255;
	for (int i = 0; i < 800; i++)
		if (m_.data[i] < min_value)
			min_value = m_.data[i];			
	
	return min_value;
}

// ��Ťб����
void deskew(Mat& img) {
	Moments mu = moments(img);
	if (abs(mu.mu02) < 1e-2)
		return;

	double skew = mu.mu11 / mu.mu02;
	Mat m = (Mat_<float>(2, 3) << 1, skew, -0.5 * 20 * skew, 0, 1, 0);

	warpAffine(img, img, m, Size(40, 40), 16);
}

// ����������ص�
void connectClosedPoint(Mat& m) {
	for (int i = 0; i < m.rows - 5; i++) {
		for (int j = 0; j < m.cols; j++) {
			if (m.at<uchar>(i, j) && m.at<uchar>(i + 5, j) && !(m.at<uchar>(i + 1, j))) {
				line(m, Point(j, i), Point(j, i + 5), Scalar(255));
			}
		}
	}
}

// �ж����� a �Ƿ��������� b
bool isBelongTo(int a, int* b) {
	for (int i = 0; i < 9; i++) {
		if (a == b[i])
			return true;
	}
	return false;
}

// ������ b ����ʽ���������� a ��ȱ�ٵ� n ������
void findMissingNumber(int* a, int* b, int n) {
	int aa = 0;
	for (int i = 1; i < 10; i++) {
		if (!isBelongTo(i, a)) {
			b[aa] = i;
			aa++;
			if (aa == n)
				return;
		}
	}
}
#endif