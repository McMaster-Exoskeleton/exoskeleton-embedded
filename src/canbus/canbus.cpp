/**
 * @file canbus.cpp
 * @brief 
 */

#include "canbus.h"
#include "stm32f4xx_hal.h"
#include <cstring>  // for memcpy

// Used by the STM32  
static CAN_HandleTypeDef hcan1;

CANBus::CANBus() : initialized_(false), baudrate_(0), error_status_(0) {
}

CANBus::~CANBus() {
    deinitialize();
}

bool CANBus::initialize(uint32_t baudrate) {
    if (initialized_) {
        return true;  
    }
//** 
    // need to change the pin assignment according to the board and how we set it up lol
    //im gonna comment this out for now
// __HAL_RCC_CAN1_CLK_ENABLE();
//__HAL_RCC_GPIOA_CLK_ENABLE();

    // GPIO_InitStruct = {0};
   // GPIO_InitStruct.Pin       = GPIO_PIN_11 | GPIO_PIN_12;
  //  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
   // GPIO_InitStruct.Pull      = GPIO_NOPULL;
   // GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
   // GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
   // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    //parameters 
//hcan1.Instance = CAN1;



    uint32_t pclk = HAL_RCC_GetPCLK1Freq();
    const uint32_t tq = 16U;
    uint32_t prescaler = 1;

    if (baudrate > 0) {
        prescaler = pclk / (baudrate * tq);
        if (prescaler < 1U)    prescaler = 1U;
        if (prescaler > 1024U) prescaler = 1024U;
    }

    hcan1.Init.Prescaler           = prescaler;
    hcan1.Init.Mode                = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth       = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1            = CAN_BS1_13TQ;
    hcan1.Init.TimeSeg2            = CAN_BS2_2TQ;
    hcan1.Init.TimeTriggeredMode   = DISABLE;
    hcan1.Init.AutoBusOff          = DISABLE;
    hcan1.Init.AutoWakeUp          = DISABLE;
    hcan1.Init.AutoRetransmission  = ENABLE;
    hcan1.Init.ReceiveFifoLocked   = DISABLE;
    hcan1.Init.TransmitFifoPriority= DISABLE;

    if (HAL_CAN_Init(&hcan1) != HAL_OK) {
        error_status_ = HAL_CAN_GetError(&hcan1);
        return false;
    }

    // Default: accept all frames
    CAN_FilterTypeDef filter = {0};
    filter.FilterBank           = 0;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = 0x0000;
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = 0x0000;   // accept al of these as the dont care condition
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank = 14;    

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) {
        error_status_ = HAL_CAN_GetError(&hcan1);
        return false;
    }

    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        error_status_ = HAL_CAN_GetError(&hcan1);
        return false;
    }

    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    baudrate_    = baudrate;
    initialized_ = true;
    error_status_= 0;
    return true;
}

void CANBus::deinitialize() {
    if (!initialized_) {
        return;
    }

    HAL_CAN_DeInit(&hcan1);
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);

    __HAL_RCC_CAN1_CLK_DISABLE();

    initialized_ = false;
    baudrate_    = 0;
    error_status_= 0;
}

bool CANBus::sendMessage(const CANMessage* message) {
    if (!initialized_ || message == nullptr) {
        return false;
    }

    if (message->dlc > 8) {
        return false;  
    }

    CAN_TxHeaderTypeDef txHeader{};
    uint8_t txData[8];

    // Copy data into a non-const buffer for HAL
    std::memcpy(txData, message->data, message->dlc);

    txHeader.DLC = message->dlc;
    txHeader.RTR = message->rtr ? CAN_RTR_REMOTE : CAN_RTR_DATA;

    if (message->extended) {
        txHeader.IDE   = CAN_ID_EXT;
        txHeader.ExtId = (message->id & 0x1FFFFFFF);  // 29 bits
    } else {
        txHeader.IDE   = CAN_ID_STD;
        txHeader.StdId = (message->id & 0x7FF);       // 11 bits
    }

    txHeader.TransmitGlobalTime = DISABLE;

    uint32_t txMailbox;
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox) != HAL_OK) {
        error_status_ = HAL_CAN_GetError(&hcan1);
        return false;
    }

    return true;
}

bool CANBus::receiveMessage(CANMessage* message) {
    if (!initialized_ || message == nullptr) {
        return false;
    }

    // If no message waiting, just return false
    if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) == 0) {
        return false;
    }

    CAN_RxHeaderTypeDef rxHeader{};
    uint8_t rxData[8];

    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK) {
        error_status_ = HAL_CAN_GetError(&hcan1);
        return false;
    }

    if (rxHeader.IDE == CAN_ID_STD) {
        message->extended = false;
        message->id       = rxHeader.StdId;
    } else {
        message->extended = true;
        message->id       = rxHeader.ExtId;
    }

    message->rtr = (rxHeader.RTR == CAN_RTR_REMOTE);
    message->dlc = rxHeader.DLC;

    for (uint8_t i = 0; i < rxHeader.DLC; ++i) {
        message->data[i] = rxData[i];
    }

    return true;
}

bool CANBus::setFilter(uint8_t filter_id, uint32_t can_id, uint32_t mask) {
    if (!initialized_) {
        return false;
    }

    if (filter_id >= 14) {
        return false;
    }

    CAN_FilterTypeDef filter{};
    filter.FilterBank           = filter_id;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank = 14;

    // Standard-ID-only version:
    // In 32-bit scale, the 11-bit ID is stored in bits [15:5] of FilterIdHigh.
    filter.FilterIdHigh     = static_cast<uint16_t>((can_id & 0x7FF) << 5);
    filter.FilterIdLow      = 0x0000;
    filter.FilterMaskIdHigh = static_cast<uint16_t>((mask   & 0x7FF) << 5);
    filter.FilterMaskIdLow  = 0x0000;

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) {
        error_status_ = HAL_CAN_GetError(&hcan1);
        return false;
    }

    return true;
}

bool CANBus::clearFilter(uint8_t filter_id) {
    if (!initialized_) {
        return false;
    }

    if (filter_id >= 14) {
        return false;
    }

    CAN_FilterTypeDef filter{};
    filter.FilterBank           = filter_id;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation     = DISABLE;
    filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) {
        error_status_ = HAL_CAN_GetError(&hcan1);
        return false;
    }

    return true;
}

uint32_t CANBus::getPendingMessageCount() const {
    if (!initialized_) {
        return 0;
    }
    auto handle = const_cast<CAN_HandleTypeDef*>(&hcan1);

    uint32_t fifo0 = HAL_CAN_GetRxFifoFillLevel(handle, CAN_RX_FIFO0);
    uint32_t fifo1 = HAL_CAN_GetRxFifoFillLevel(handle, CAN_RX_FIFO1);

    return fifo0 + fifo1;
}

bool CANBus::isInitialized() const {
    return initialized_;
}

uint32_t CANBus::getErrorStatus() const {
    if (!initialized_) {
        return 0;
    }

    auto handle = const_cast<CAN_HandleTypeDef*>(&hcan1);
    return HAL_CAN_GetError(handle);
}

