#include "can/can_app.h"
#include "can/can_bus_stm32.hpp"
#include "can/ring_buffer.hpp"
#include "main.h"
#include <cstdint>

extern UART_HandleTypeDef huart2;
extern CAN_HandleTypeDef hcan1;

static CanBusStm32 g_can;

// Use the correct class name found in your .hpp file
static CanRxRingBuffer uart_to_can_buffer;

extern "C" void CanApp_PushUartByte(uint8_t byte) {
    // Create a temporary frame to hold the single UART byte
    CanFrame f;
    f.id = 0x124;
    f.dlc = 1;
    f.data[0] = byte;
    uart_to_can_buffer.push(f); // Push the frame into the buffer
}

void CanApp_Init(void) {
    (void)g_can.init(&hcan1);
}

void CanApp_Tick(void) {
    // 1. FORWARD: UART (Computer) -> CAN (Bus)
    CanFrame txFrame;
    // Use the .size() method from your class to check for data
    while (uart_to_can_buffer.size() > 0) {
        if (uart_to_can_buffer.pop(txFrame)) { // Pop the frame
            g_can.sendStd(txFrame.id, txFrame.data, txFrame.dlc);
        }
    }

    // 2. FORWARD: CAN (Bus) -> UART (Computer)
    CanFrame rxFrame;
    while (g_can.recv(rxFrame)) {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        HAL_UART_Transmit(&huart2, rxFrame.data, rxFrame.dlc, 10);
    }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    if (hcan->Instance != CAN1) return;
    g_can.onRxFifo0Pending();
}
