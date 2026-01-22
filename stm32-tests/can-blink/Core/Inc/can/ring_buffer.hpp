#pragma once

#include "can/can_frame.hpp"
#include <cstdint>

class CanRxRingBuffer {
public:
	// buffer frame-capacity
	static constexpr uint16_t kCapacity = 32;

	// Insert one frame into the buffer
	bool push(const CanFrame& f) {
		if (count_ >= kCapacity) return false; // drop frames if full
		buf_[head_] = f;
		head_ = (head_ + 1) % kCapacity;
		count_++;
		return true;
	}

	bool pop(CanFrame& out) {
		if (count_ == 0) return false;
		out = buf_[tail_];
		tail_ = (tail_ + 1) % kCapacity;
		count_--;
		return true;
	}

	//Returns the current number of queued frames
	uint16_t size() const { return count_; }

private:
	  CanFrame  buf_[kCapacity]; // fixed-size array of frames
	  uint16_t head_ = 0; // index where the next pushed element goes
	  uint16_t tail_ = 0; // index where the next pop reads
	  uint16_t count_ = 0; // how many frames are in the buffer
};
