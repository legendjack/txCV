#pragma once
#ifndef _FUNCTIONS_H_
#define _FUNCTIONS_H_

#include <opencv2/opencv.hpp>
#include <iostream>

#define RED 0					// �����ɫ
#define BLUE 1					// ������ɫ

using namespace std;
using namespace cv;


void showText()
{
	cout << "������ʾ��" << endl;
	cout << "\t����0������ʾ��ֵ�ָ�Ľ��" << endl;
	cout << "\t����1������ͣ��Ƶ" << endl;
}

void choise(int *a, int *b, int n)
{
	int i, j, k, temp;

	for (i = 0; i < n - 1; i++)
	{
		k = i;	// ���ǺŸ�ֵ

		for (j = i + 1; j < n; j++)
			if (a[k] < a[j]) k = j;	// ��k����ָ����СԪ��

		// ��k!=i�ǲŽ���������a[i] ��Ϊ��С
		if (i != k)
		{
			temp = a[i];
			a[i] = a[k];
			a[k] = temp;

			temp = b[i];
			b[i] = b[k];
			b[k] = temp;
		}
	}
}

// ͨ��������ģ����Ŀ����ɫͨ���ĻҶ�ֵ���ж�Ŀ�������Ƿ�Ϊָ����ɫ
bool JudgeColor(Mat _m1, Mat _m2, uchar _threshold, Mat mask) {
	int _size = _m1.cols * _m1.rows;
	int _sum = 0;
	int _total = 0;
	for (int i = 0; i < _size; i++) {
		if (mask.data[i]) {
			if ((_m1.data[i] - _m2.data[i]) > _threshold)
				_sum++;
			_total++;
		}
	}
	
	if (float(_sum) / _total > 0.3)
		return true;
	else
		return false;
}

// ����ֱ��ͼ�зֲ�������ɫ����ɫ���⣩��ѡȡǰ20���ֲ����Ե���ɫֵ�����ĸ��������
int JudgeColor(MatND hist)
{
	// �����Ǹ����飬color��ʾ��ɫ��countֱ��ͼ�е�i����ɫ��Ӧ��ֵ
	int color[180];
	for (int i = 0; i < 180; i++)
		color[i] = i;

	int value[180];
	for (int i = 0; i < 180; i++)
		value[i] = static_cast<int>(hist.at<float>(i));

	choise(value, color, 180);

	int maxValue = color[0];

	int red = 0;
	int blue = 0;
	int otherColor = 0;

	for (int i = 0; i < 10; i++)
	{
		int binValue = color[i];
		if (binValue > 99 && binValue < 125)
		{
			blue++;
		}
		else if ((binValue > -1 && binValue < 11) || (binValue > 155 && binValue < 181))
		{
			red++;
		}
		else
			otherColor++;
	}

	if (red > blue && red > otherColor)
	{
		return RED;
	}
	else if (red < blue && blue > otherColor)
	{
		return BLUE;
	}
	else
		return -1;
}

Point centerOf2Points(Point p1, Point p2) {
	Point p;
	p.x = (p1.x + p2.x) / 2;
	p.y = (p1.y + p2.y) / 2;
	return p;
}

void exchange(float &a, float &b) {
    float temp = a;
    a = b;
    b = temp;
}

class SearchWindow {
private:
	int centerX;
	int centerY;
	int windowHeight;
	int windowWidth;

public:
	int width;
	int height;

	SearchWindow() {
		centerX = 0;
		centerY = 0;
		width = 0;
		height = 0;
		windowHeight = 0;
		windowWidth = 0;
	}

	SearchWindow(int _windowWidth, int _windowHeight) {
		centerX = 0;
		centerY = 0;
		width = 0;
		height = 0;
		windowHeight = _windowHeight;
		windowWidth = _windowWidth;
	}

	SearchWindow(int _centerX, int _centerY, int _width, int _height, int _windowWidth, int _windowHeight) {
		windowHeight = _windowHeight;
		windowWidth = _windowWidth;

		if (_centerX < 0)
			centerX = 0;
		else if (_centerX >= windowWidth)
			centerX = windowWidth - 1;
		else
			centerX = _centerX;

		if (_centerY < 0)
			centerY = 0;
		else if (_centerY >= windowHeight)
			centerY = windowHeight - 1;
		else
			centerY = _centerY;

		if (_width > windowWidth)
			width = windowWidth;
		else
			width = _width;

		if (_height > windowHeight)
			height = windowHeight;
		else
			height = _height;
	}
	~SearchWindow() {};

	void setCenter(int _centerX, int _centerY) {
		if (_centerX < 0)
			centerX = 0;
		else if (_centerX >= windowWidth)
			centerX = windowWidth - 1;
		else
			centerX = _centerX;

		if (_centerY < 0)
			centerY = 0;
		else if (_centerY >= windowHeight)
			centerY = windowHeight - 1;
		else
			centerY = _centerY;
	}

	void setSize(int _width, int _height) {
		if (_width > windowWidth)
			width = windowWidth;
		else
			width = _width;

		if (_height > windowHeight)
			height = windowHeight;
		else
			height = _height;
	}

	void zoomIn() {
		width = width * 2;
	}

	Rect getRect() {
		Rect rect;

		if ((centerX + width / 2) >= windowWidth)
			rect.width = windowWidth - (centerX - width / 2);
		else if (centerX < width / 2)
			rect.width = centerX + width / 2;
		else
			rect.width = width;

		if ((centerY + height / 2) >= windowHeight)
			rect.height = windowHeight - (centerY - height / 2);
		else if (centerY < height / 2)
			rect.height = centerY + height / 2;
		else
			rect.height = height;

		if ((centerX - width / 2) < 0)
			rect.x = 0;
		else
			rect.x = centerX - width / 2;

		if ((centerY - height / 2) < 0)
			rect.y = 0;
		else
			rect.y = centerY - height / 2;

		return rect;
	}
};

#endif