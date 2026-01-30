#include "can/can_bus_stm32.hpp"
#include <cstring>

uint32_t CanBusStm32::packStdIdToFilter32(uint16_t std_id) {
	// catch the ID bits of the message, turn into 32bit for CAN use
	// bits 31:21 = STID (11-bit standard ID)
	return (static_cast<uint32_t>(std_id & 0x7FFU) << 21);
}

bool CanBusStm32::init(CAN_HandleTypeDef* hcan, const uint16_t* accept_std_ids, uint8_t num_ids) {
	hcan_ = hcan;
	if (!hcan_ || !accept_std_ids || num_ids == 0) return false;

	// Configure as many filter banks as needed.
	// Each bank in 32-bit IDLIST mode holds TWO exact IDs:
	//  - "FilterId" holds ID A (FR1)
	//  - "FilterMaskId" holds ID B (FR2)  (in list mode it's not a mask; it's a 2nd ID)
	uint8_t bank = 0;
	uint8_t i = 0;

	while (i < num_ids) {
		const uint16_t id_a = accept_std_ids[i++];
		const bool has_b = (i < num_ids);
		const uint16_t id_b = has_b ? accept_std_ids[i++] : 0x7FF; // safe-ish unused fallback

		CAN_FilterTypeDef f = {};
		f.FilterBank = bank++;
		f.FilterMode = CAN_FILTERMODE_IDLIST;
		f.FilterScale = CAN_FILTERSCALE_32BIT;
		f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
		f.FilterActivation = ENABLE;
		f.SlaveStartFilterBank = 14;

		const uint32_t a32 = packStdIdToFilter32(id_a);
		const uint32_t b32 = packStdIdToFilter32(id_b);

		f.FilterIdHigh     = static_cast<uint16_t>((a32 >> 16) & 0xFFFF);
		f.FilterIdLow      = static_cast<uint16_t>((a32      ) & 0xFFFF);
		f.FilterMaskIdHigh = static_cast<uint16_t>((b32 >> 16) & 0xFFFF);
		f.FilterMaskIdLow  = static_cast<uint16_t>((b32      ) & 0xFFFF);

		if (HAL_CAN_ConfigFilter(hcan_, &f) != HAL_OK) return false;
	}

	if (HAL_CAN_Start(hcan_) != HAL_OK) return false;

	const uint32_t notif = CAN_IT_RX_FIFO0_MSG_PENDING |
						 CAN_IT_ERROR |
						 CAN_IT_BUSOFF |
						 CAN_IT_LAST_ERROR_CODE;

	if (HAL_CAN_ActivateNotification(hcan_, notif) != HAL_OK) return false;

	return true;
	}

bool CanBusStm32::sendStd(uint16_t std_id, const uint8_t* data, uint8_t dlc) {
	if (!hcan_) return false; // must be initialized
	if (std_id > 0x7FF) return false; // must be lower than 11 bits
	if (dlc > 8) return false; // must be lower than 8 bytes

	CAN_TxHeaderTypeDef hdr = {};
	hdr.IDE = CAN_ID_STD;
	hdr.RTR = CAN_RTR_DATA;
	hdr.StdId = std_id;
	hdr.DLC = dlc;
	hdr.TransmitGlobalTime = DISABLE;

	// queue frame to hardware mailbox to be sent
	uint32_t mailbox = 0;
	return (HAL_CAN_AddTxMessage(hcan_, &hdr, const_cast<uint8_t*>(data), &mailbox) == HAL_OK);
}

// read software buffer
bool CanBusStm32::recv(CanFrame& out) {
	return rxq_.pop(out);
}

// drain hardware FIFO, fill software buffer
void CanBusStm32::onRxFifo0Pending() {
	if (!hcan_) return;

	CAN_RxHeaderTypeDef hdr = {};
	uint8_t data[8] = {0};

	if (HAL_CAN_GetRxMessage(hcan_, CAN_RX_FIFO0, &hdr, data) != HAL_OK) {
		return;
	}

	CanFrame f;
	if (hdr.IDE == CAN_ID_STD) {
		f.id = hdr.StdId;
	} else {
		f.id = hdr.ExtId;
	}

	f.dlc = hdr.DLC;
	std::memcpy(f.data, data, 8);

	// Push into buffer, if full, drop
	(void)rxq_.push(f);
}
