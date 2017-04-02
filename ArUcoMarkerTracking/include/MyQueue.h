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
	~MyQueue() { data.clear(); };

	void push(int k);
	int average();
	void clear();
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
		std::vector<int> _data(maxSize);
		for (int count = 0; count < maxSize - 1; count++)
			_data[count] = data[count + 1];
		_data[maxSize - 1] = k;
		data.clear();
		data = _data;
		_data.clear();
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

#endif // endif