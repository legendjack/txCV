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

	for (int i = 0; i < 15; i++)
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

	if ((red > blue && red > otherColor) || (maxValue > 155 && maxValue < 181))
	{
		return RED;
	}
	else if ((red < blue && blue > otherColor) || (maxValue > 99 && maxValue < 125))
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

#endif