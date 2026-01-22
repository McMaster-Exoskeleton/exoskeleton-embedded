#pragma once

#include "can/can_frame.hpp"
#include "can/ring_buffer.hpp"

#include "stm32f4xx_hal.h"

class CanBusStm32 {
public:
  bool init(CAN_HandleTypeDef* hcan);
  bool sendStd(uint16_t std_id, const uint8_t* data, uint8_t dlc);
  bool recv(CanFrame& out);

  // Called from the HAL RX callback
  void onRxFifo0Pending();

private:
  CAN_HandleTypeDef* hcan_ = nullptr;
  CanRxRingBuffer rxq_;
};
