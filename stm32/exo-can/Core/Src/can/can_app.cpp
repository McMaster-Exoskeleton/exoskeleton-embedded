#include "can/can_app.h"
#include "can/can_bus_stm32.hpp"
#include "main.h"   // for pin definitions and CAN handle externs

#include <cstdint>

extern CAN_HandleTypeDef hcan1;

static CanBusStm32 g_can;

// Initialize CAN
void CanApp_Init(void) {

	(void)g_can.init(&hcan1);
}

// Send 0x123 every 100ms, the data is the counter of frames sent so far
void CanApp_Tick(void) {

	static uint32_t last_ms = 0;
	static uint8_t ctr = 0;

	uint32_t now = HAL_GetTick();
	if (now - last_ms >= 100) {
		last_ms = now;
		uint8_t data[1] = { ctr++ };
		(void)g_can.sendStd(0x123, data, 1);
	}

	// Process any received frames (toggle LED per frame)
	CanFrame f;
	while (g_can.recv(f)) {
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}
}

// HAL calls this callback whenever RX FIFO0 has a message pending
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
	if (hcan->Instance != CAN1) return;
	g_can.onRxFifo0Pending();
}
