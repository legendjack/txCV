#ifndef _MYQUEUE_H_
#define _MYQUEUE_H_

#include <vector>

class MyQueue {
private:
	int sum;		// 队列所有元素的和
	int maxSize;	// 队列最大的size
	std::vector<int> data;

public:
	int max;		// 最大值
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
	if (dataSize < maxSize) {
		data.push_back(k);
		dataSize++;
	}
	else {
		sum -= data[0];
		for (int count = 0; count < maxSize - 1; count++)
			data[count] = data[count + 1];
		data[maxSize - 1] = k;
	}
	
	min = data[0];
	max = data[0];
	for (int count = 0; count < dataSize; count++) {
		if (data[count] < min)
			min = data[count];
		if (data[count] > max)
			max = data[count];
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
