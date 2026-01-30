#pragma once

#include "can/can_frame.hpp"
#include "can/ring_buffer.hpp"
#include "stm32f4xx_hal.h"

class CanBusStm32 {
public:
	bool init(CAN_HandleTypeDef* hcan, const uint16_t* accept_std_ids, uint8_t num_ids);

	bool sendStd(uint16_t std_id, const uint8_t* data, uint8_t dlc);
	bool recv(CanFrame& out);

	// Called from the HAL RX callback
	void onRxFifo0Pending();

private:

	static uint32_t packStdIdToFilter32(uint16_t std_id);

	CAN_HandleTypeDef* hcan_ = nullptr;
	CanRxRingBuffer rxq_;
};
