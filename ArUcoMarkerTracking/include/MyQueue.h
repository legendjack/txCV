#ifndef _MYQUEUE_H_
#define _MYQUEUE_H_

#include <vector>

class MyQueue {
private:
	int sum;		// 队列所有元素的和
	int maxSize;	// 队列最大的size
	std::vector<int> data;

public:
	const int max;	// 最大值
	int min;		// 最小值
	int dataSize;	// 队列当前的size
	
	MyQueue() {};
	MyQueue(int i);
	~MyQueue() {};

	void push(int k);
	int average();
	void clear();
	int operator[](int i);
};

MyQueue::MyQueue(int i) {
	max = 0;
	min = 0;
	sum = 0;
	dataSize = 0;
	maxSize = i;
}

void MyQueue::push(int k) {
	sum += k;
	if (dataSize == 0) {
		data.push_back(k);
		dataSize++;
		min = k;
		max = k;
	}
	else if (dataSize < maxSize) {
		data.push_back(k);
		dataSize++;
		if (k > max)
			max = k;
		if (k < min)
			min = k;
	}
	else {
		sum -= data[0];
		for (int count = 0; count < maxSize - 1; count++)
			data[count] = data[count + 1];
		data[maxSize - 1] = k;
		if (k > max)
			max = k;
		if (k < min)
			min = k;
	}
}

int MyQueue::average() {
	if (!dataSize)
		return 0;
	else
		return sum / dataSize;
}

void MyQueue::clear() {
	max = 0;
	min = 0;
	sum = 0;
	dataSize = 0;
	data.clear();
}

int MyQueue::operator[](int i) {
	if (i < dataSize && i >= 0)
		return data[i];
	else
		return 0;
}

#endif // endif