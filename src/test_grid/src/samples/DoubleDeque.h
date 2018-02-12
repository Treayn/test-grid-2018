#ifndef DOUBLE_DEQUE_H
#define DOUBLE_DEQUE_H

#include <deque>
#include <memory>

class DoubleDeque {
public:
	std::deque<std::deque<std::unique_ptr<int>>> test;
	
	DoubleDeque();
	~DoubleDeque();
};

#endif
