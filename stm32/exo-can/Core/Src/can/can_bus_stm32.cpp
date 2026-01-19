#include "can/can_bus_stm32.hpp"
#include <cstring>

bool CanBusStm32::init(CAN_HandleTypeDef* hcan) {
  hcan_ = hcan;

  //filter accepts any CAN message
  CAN_FilterTypeDef filter = {};
  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0x0000;
  filter.FilterIdLow  = 0x0000;
  filter.FilterMaskIdHigh = 0x0000;
  filter.FilterMaskIdLow  = 0x0000;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(hcan_, &filter) != HAL_OK) return false;

  // Start CAN peripheral
  if (HAL_CAN_Start(hcan_) != HAL_OK) return false;

  // Enable RX
  uint32_t notif = CAN_IT_RX_FIFO0_MSG_PENDING |
                   CAN_IT_ERROR |
                   CAN_IT_BUSOFF |
                   CAN_IT_LAST_ERROR_CODE;
  // Activate interrupts
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

  // If full, drop
  (void)rxq_.push(f);
}
