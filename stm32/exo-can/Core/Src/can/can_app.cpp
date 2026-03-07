#include "can/can_app.h"
#include "can/can_bus_stm32.hpp"
#include "can/can_protocol.hpp"
#include "main.h"	// for pin definitions and CAN handle externs
#include <cstdint>

extern CAN_HandleTypeDef hcan1;

static CanBusStm32 g_can;

// Set per-board (compile-time for now)
static constexpr uint8_t ThisNode = 2; // change to 2 on the other board
static constexpr uint8_t PeerNode = 1; // the node you wanna talk to
//add more nodes?



// Initialize CAN
void CanApp_Init(void) {
	static_assert(canproto::IsValidNode(ThisNode), "Invalid node id");

	const uint16_t accept_ids[] = {
	    canproto::CmdId(ThisNode),
	    canproto::HbId(ThisNode),
		canproto::ReqDumpId(ThisNode)
	};


	  (void)g_can.init(&hcan1, accept_ids, static_cast<uint8_t>(sizeof(accept_ids)/sizeof(accept_ids[0])));

}

void CanApp_Tick(void) {
	  // Send a command frame to the peer every 100 ms
	  static uint32_t last_ms = 0;
	  static uint8_t seq = 0;

	  const uint32_t now = HAL_GetTick();
	  if (now - last_ms >= 100) {
		  last_ms = now;

		  canproto::CommandPayload cmd{};
		  cmd.pos_q = 0;
		  cmd.vel_q = 0;
		  cmd.tau_q = 0;
		  cmd.mode  = static_cast<uint8_t>(canproto::ControlMode::Disabled);
		  cmd.seq   = seq++;

		  (void)g_can.sendStd(0x555, reinterpret_cast<const uint8_t*>(&cmd), sizeof(cmd));
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
