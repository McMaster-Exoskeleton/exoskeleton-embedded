#pragma once

#include <cstdint>

namespace canproto {

// ---------------------------
// Node / ID layout (11-bit)
// ---------------------------
//
// Convention: node_id in [1..4] for joints
//
// 0x100 + node_id : Command   (master -> joint)
// 0x200 + node_id : State     (joint -> master)
// 0x(300-330) + node_id : IMU data dump    (either direction)
// 0x700 + node_id : Heartbeat (either direction)
//
//

	constexpr uint16_t CmdBase 		= 0x100;
	constexpr uint16_t StateBase 	= 0x200;
	constexpr uint16_t ReqDumpBase 	= 0x300;
	constexpr uint16_t DumpMetaBase 	= 0x310;
	constexpr uint16_t DumpDataBase 	= 0x320;
	constexpr uint16_t DumpEndBase 	= 0x330;
	constexpr uint16_t HbBase 		= 0x700;


	constexpr uint8_t  NodeMin = 1;
	constexpr uint8_t  NodeMax = 4;

	constexpr bool IsValidNode(uint8_t node_id) {
	  return (node_id >= NodeMin) && (node_id <= NodeMax);
	}

	constexpr uint16_t CmdId(uint8_t node_id)   	{ return CmdBase   + node_id; }
	constexpr uint16_t StateId(uint8_t node_id) 	{ return StateBase + node_id; }
	constexpr uint16_t ReqDumpId(uint8_t node_id) 	{ return ReqDumpBase + node_id; }
	constexpr uint16_t DumpMetaId(uint8_t node_id) 	{ return DumpMetaBase + node_id; }
	constexpr uint16_t DumpDataId(uint8_t node_id) 	{ return DumpDataBase + node_id; }
	constexpr uint16_t DumpEndId(uint8_t node_id) 	{ return DumpEndBase + node_id; }
	constexpr uint16_t HbId(uint8_t node_id)    	{ return HbBase    + node_id; }


	// ---------------------------
	// Payloads (must be 8 bytes)
	// ---------------------------
	// 8 bytes is classic CAN max payload.
	// We use packed structs + static_assert to guarantee size.

	#pragma pack(push, 1) // keep compiler from padding structs and ruining CAN expected size

	enum class ControlMode : uint8_t {
	  Disabled = 0,
	  Position = 1,
	  Velocity = 2,
	  Torque   = 3,
	};

	struct CommandPayload {
	  int16_t pos_q;
	  int16_t vel_q;
	  int16_t tau_q;
	  uint8_t mode;     // ControlMode
	  uint8_t seq;      // sequence counter
	};

	struct StatePayload {
	  int16_t pos_q;
	  int16_t vel_q;
	  int16_t tau_q;
	  uint8_t status;   // bitfield later (faults, enabled, etc.)
	  uint8_t seq;      // echo last command seq (or its own counter)
	};

	struct DumpRequestPayload {
		uint8_t seq;				// sequence counter
		uint8_t reserved0;
		uint16_t max_samples;   // cap how many samples to send this time
		uint32_t reserved1;
	};

	struct DumpMetaPayload {
		uint8_t seq;				// sequence counter
		uint8_t reserved0;
		uint16_t total_samples;		// total samples node will send
		uint32_t reserved1;
	};


	struct DumpDataPayload {
		uint8_t seq;				// sequence counter
		uint8_t idx;     			// 0..255
		int16_t ax;
		int16_t ay;
		int16_t az;
	};

	struct DumpEndPayload {
		uint8_t seq;				// sequence counter
		uint8_t status;       // idk yet
		uint16_t sent_samples;
		uint32_t reserved1;
	};

	struct HeartbeatPayload {
	  uint32_t uptime_ms;  // uptime in ms (wraps)
	  uint8_t  node_id;
	  uint8_t  role;       // 0=unknown, 1=master, 2=joint
	  uint16_t flags;      // TODO
	};

	#pragma pack(pop)

	static_assert(sizeof(CommandPayload)   		== 8, "CommandPayload must be 8 bytes");
	static_assert(sizeof(StatePayload)     		== 8, "StatePayload must be 8 bytes");
	static_assert(sizeof(DumpRequestPayload) 	== 8, "Payload must be 8 bytes");
	static_assert(sizeof(DumpMetaPayload) 		== 8, "Payload must be 8 bytes");
	static_assert(sizeof(DumpDataPayload) 		== 8, "Payload must be 8 bytes");
	static_assert(sizeof(DumpEndPayload) 		== 8, "Payload must be 8 bytes");
	static_assert(sizeof(HeartbeatPayload) 		== 8, "HeartbeatPayload must be 8 bytes");

}
