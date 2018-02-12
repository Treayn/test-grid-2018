#include <algorithm>
#include <iostream>
#include <string>

#include "DoubleDeque.h"

int main() {
	DoubleDeque deck;

	std::for_each(deck.test.begin(), deck.test.end(), [](std::deque<std::unique_ptr<int>>& col) {
		std::for_each(col.begin(), col.end(), [](std::unique_ptr<int>& ptr) {
			std::cout << *ptr.get() << std::endl;
		});
	});
	
	return 0;
}
