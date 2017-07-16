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

// ��д���ֹǸ���ȡ���ʻ���ϸΪ 1 ���أ�
void chao_thinimage(Mat &srcimage) {
	vector<Point> deletelist1;
	int Zhangmude[9];		// ��¼�� i �С��� j �е����ؼ�����Χ���Ÿ����ص�����ֵ�������255���Ӧֵ��Ϊ1������Ϊ0
	int nl = 40;			// ͼ���
	int nc = 40;			// ͼ���
	while (true)
	{
		for (int j = 1; j < (nl - 1); j++)
		{
			// ͼ���������е�����ָ��
			uchar* data_last = srcimage.ptr<uchar>(j - 1);
			uchar* data = srcimage.ptr<uchar>(j);
			uchar* data_next = srcimage.ptr<uchar>(j + 1);
			for (int i = 1; i < (nc - 1); i++)
			{
				// data[i] ��ͼ��� j �У��� i �е�����ֵ
				if (data[i] == 255)
				{
					// ����˳ʱ���˳��
					Zhangmude[0] = 1;
					Zhangmude[1] = (data_last[i] == 255) ? 1 : 0;		// �Ϸ�
					Zhangmude[2] = (data_last[i + 1] == 255) ? 1 : 0;	// ���Ͻ�
					Zhangmude[3] = (data[i + 1] == 255) ? 1 : 0;		// �ҷ�
					Zhangmude[4] = (data_next[i + 1] == 255) ? 1 : 0;	// ���·�
					Zhangmude[5] = (data_next[i] == 255) ? 1 : 0;		// �·�
					Zhangmude[6] = (data_next[i - 1] == 255) ? 1 : 0;	// ���·�
					Zhangmude[7] = (data[i - 1] == 255) ? 1 : 0;		// ��
					Zhangmude[8] = (data_last[i - 1] == 255) ? 1 : 0;	// ���Ϸ�

					int whitepointtotal = 0;
					for (int k = 1; k < 9; k++)
						whitepointtotal += Zhangmude[k];

					if ((whitepointtotal >= 2) && (whitepointtotal <= 6)) {
						int ap = 0;
						for (int k = 1; k < 9; k++)
							if (!Zhangmude[k] && Zhangmude[k % 8 + 1])
								ap++;

						if (ap == 1)
							if ((Zhangmude[1] * Zhangmude[7] * Zhangmude[5] == 0) && (Zhangmude[3] * Zhangmude[5] * Zhangmude[7] == 0))
								deletelist1.push_back(Point(i, j));
					}
				}
			}
		}

		if (deletelist1.size() == 0) break;
		for (size_t i = 0; i < deletelist1.size(); i++) {
			Point tem;
			tem = deletelist1[i];
			uchar* data = srcimage.ptr<uchar>(tem.y);
			data[tem.x] = 0;
		}
		deletelist1.clear();

		for (int j = 1; j < (nl - 1); j++) {
			uchar* data_last = srcimage.ptr<uchar>(j - 1);
			uchar* data = srcimage.ptr<uchar>(j);
			uchar* data_next = srcimage.ptr<uchar>(j + 1);
			for (int i = 1; i < (nc - 1); i++)
			{
				if (data[i] == 255)
				{
					Zhangmude[0] = 1;
					Zhangmude[1] = (data_last[i] == 255) ? 1 : 0;		// �Ϸ�
					Zhangmude[2] = (data_last[i + 1] == 255) ? 1 : 0;	// ���Ͻ�
					Zhangmude[3] = (data[i + 1] == 255) ? 1 : 0;		// �ҷ�
					Zhangmude[4] = (data_next[i + 1] == 255) ? 1 : 0;	// ���·�
					Zhangmude[5] = (data_next[i] == 255) ? 1 : 0;		// �·�
					Zhangmude[6] = (data_next[i - 1] == 255) ? 1 : 0;	// ���·�
					Zhangmude[7] = (data[i - 1] == 255) ? 1 : 0;		// ��
					Zhangmude[8] = (data_last[i - 1] == 255) ? 1 : 0;	// ���Ϸ�

					int whitepointtotal = 0;
					for (int k = 1; k < 9; k++)
						whitepointtotal += Zhangmude[k];

					if ((whitepointtotal >= 2) && (whitepointtotal <= 6)) {
						int ap = 0;
						for (int k = 1; k < 9; k++)
							if (!Zhangmude[k] && Zhangmude[k % 8 + 1])
								ap++;

						if (ap == 1)
							if ((Zhangmude[1] * Zhangmude[3] * Zhangmude[5] == 0) && (Zhangmude[3] * Zhangmude[1] * Zhangmude[7] == 0))
								deletelist1.push_back(Point(i, j));
					}
				}
			}
		}

		if (deletelist1.size() == 0) break;
		for (size_t i = 0; i < deletelist1.size(); i++) {
			Point tem;
			tem = deletelist1[i];
			uchar* data = srcimage.ptr<uchar>(tem.y);
			data[tem.x] = 0;
		}
		deletelist1.clear();
	}
}

// �ҵ�ͼ�����а�ɫ���ع�ͬ������
void findAllContour(Mat src, vector<vector<Point> > & contours) {
	contours.clear();
	vector<Point> points_;
	for (int i = 0; i < 40; i++)
		for (int j = 0; j < 40; j++)
			if (src.at<uchar>(i, j))
				points_.push_back(Point(j, i));
	contours.push_back(points_);
}


// ��������ܵķָ���ֵ
int calcNixietubeThreshold(Mat m) {
	int thresholdLevel[11] = { 0,0,0,0,0,0,0,0,0,0,0 };
	int k = m.cols * m.rows;
	for (int i = 0; i < k; i++)
	{
		if (m.data[i] > 225)
			thresholdLevel[0]++;
		else if (m.data[i] > 210 && (m.data[i] < 235))
			thresholdLevel[1]++;
		else if (m.data[i] > 195 && (m.data[i] < 220))
			thresholdLevel[2]++;
		else if (m.data[i] > 180 && (m.data[i] < 205))
			thresholdLevel[3]++;
		else if (m.data[i] > 165 && (m.data[i] < 190))
			thresholdLevel[4]++;
		else if (m.data[i] > 150 && (m.data[i] < 175))
			thresholdLevel[5]++;
		else if (m.data[i] > 135 && (m.data[i] < 160))
			thresholdLevel[6]++;
// 		else if (m.data[i] > 175)
// 			thresholdLevel[7]++;
// 		else if (m.data[i] > 165)
// 			thresholdLevel[8]++;
// 		else if (m.data[i] > 155)
// 			thresholdLevel[9]++;
// 		else if (m.data[i] > 145)
// 			thresholdLevel[10]++;
	}

	int x = 0, z = 0;
	for (int i = 0; i < 10; i++) {
		if (thresholdLevel[i] > x) {
			x = thresholdLevel[i];
			z = i;
		}
	}

	return (210 - 15 * z);
}

#endif