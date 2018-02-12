#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <utility>

#include "DoubleDeque.h"

DoubleDeque::DoubleDeque()
{
	test.resize(10);
	std::for_each(test.begin(), test.end(), [](std::deque<std::unique_ptr<int>>& col) {
		col.resize(10);
		std::cout << "column size: " << col.size() << std::endl;
		
		std::generate(col.begin(), col.end(), []() {
			return std::unique_ptr<int>{ new int(std::rand()) };
		});
	});
}


DoubleDeque::~DoubleDeque() {
}

