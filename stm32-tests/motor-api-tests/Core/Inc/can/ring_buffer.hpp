#pragma once

#include "can/can_frame.hpp"
#include <cstdint>

class CanRxRingBuffer {
public:
  static constexpr uint16_t kCapacity = 32;

  bool push(const CanFrame& f) {
    if (count_ >= kCapacity) return false;
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

  uint16_t size() const { return count_; }

private:
  CanFrame  buf_[kCapacity];
  uint16_t head_ = 0;
  uint16_t tail_ = 0;
  uint16_t count_ = 0;
};
