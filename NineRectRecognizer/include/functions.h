#pragma once
#ifndef _FUNCTIONS_H_
#define _FUNCTIONS_H_

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

// 对宫格排序，1~9
void RectSort(vector<Rect>& nineRect_) {
	for (int i = 0; i < 9; ++i)//对y进行冒泡排序
		for (int j = 0; j < 8 - i; ++j)
			if (nineRect_[j].y > nineRect_[j + 1].y)
				swap(nineRect_[j], nineRect_[j + 1]);

	for (int k = 0; k < 3; ++k)//每3个一组，对x进行排序
		for (int i = 0; i < 3; ++i)
			for (int j = 0; j < 2 - i; ++j)
				if (nineRect_[3 * k + j].x > nineRect_[3 * k + j + 1].x)
					swap(nineRect_[3 * k + j], nineRect_[3 * k + j + 1]);
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
	for (int i = 0; i < 9; ++i)//对y进行冒泡排序
		for (int j = 0; j < 8 - i; ++j)
			if (contours_rotatedRect_[j].center.y > contours_rotatedRect_[j + 1].center.y)
				swap(contours_rotatedRect_[j], contours_rotatedRect_[j + 1]);

	for (int k = 0; k < 3; ++k)//每3个一组，对x进行排序
		for (int i = 0; i < 3; ++i)
			for (int j = 0; j < 2 - i; ++j)
				if (contours_rotatedRect_[3 * k + j].center.x > contours_rotatedRect_[3 * k + j + 1].center.x)
					swap(contours_rotatedRect_[3 * k + j], contours_rotatedRect_[3 * k + j + 1]);
}

int sum_mat(Mat m_) {
	int sum_value = 0;
	int sum_max = 0;
	for (int i = 0; i < 160; i++) {
		if (m_.data[i] > 0) {
			sum_value++;
			if (sum_value > sum_max)
				sum_max = sum_value;
		}
		else
			sum_value = 0;
	}

	return sum_max;
}

#endif