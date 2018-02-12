#include "AllocatedCell.h"

AllocatedCell::AllocatedCell() {
	this->status_registers.fill(0);
}

AllocatedCell::~AllocatedCell() {

}

uint8_t * const AllocatedCell::getCellAddress() {
	return (uint8_t *) this;
}

uint8_t AllocatedCell::getCellStatus(uint8_t stride) const {
	return status_registers.at(stride);
}

void AllocatedCell::setCellStatus(uint8_t stride, uint8_t data) {
	status_registers.at(stride) = data;
}
