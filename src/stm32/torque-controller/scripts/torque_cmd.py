#!/usr/bin/env python3
"""
torque_cmd.py - Raspberry Pi torque command sender for the exoskeleton CAN network.

Sends torque commands (Nm) to STM32 joint controllers over CAN bus.
Each STM32 converts torque to motor current: Iq = torque / (Kt * gear_ratio)

Setup (run once on Pi):
    sudo ip link set can1 up type can bitrate 1000000

Usage:
    python3 torque_cmd.py [can_interface]

    Default CAN interface: can1
"""

import sys
import can_common
import can_motor
import can_system

# Node name mapping
NODES = {
    can_common.NODE_LEFT_HIP:   "Left Hip",
    can_common.NODE_RIGHT_HIP:  "Right Hip",
    can_common.NODE_LEFT_KNEE:  "Left Knee",
    can_common.NODE_RIGHT_KNEE: "Right Knee",
}

# AK70-9 KV60 effective torque constant (Kt * gear_ratio)
KT_EFFECTIVE = 1.431  # Nm/A

# Test-safe torque limit (matches STM32 CURRENT_LIMIT of 5A)
TORQUE_LIMIT = 5.0 * KT_EFFECTIVE  # ~7.155 Nm


def print_help():
    print("Commands:")
    print("  <node_id> <torque_nm>  - Send torque to a joint")
    print("  all <torque_nm>        - Send same torque to all joints")
    print("  stop                   - Zero torque on all joints")
    print("  estop                  - Emergency stop (latches on STM32)")
    print("  help                   - Show this help")
    print("  quit                   - Exit")
    print()
    print("Nodes:")
    for nid, name in NODES.items():
        print(f"  {nid} = {name}")
    print()
    print(f"Torque range: +/- {TORQUE_LIMIT:.1f} Nm (clamped by STM32)")


def main():
    channel = sys.argv[1] if len(sys.argv) > 1 else "can1"

    print("Exoskeleton Torque Command Sender")
    print("=" * 50)
    print(f"CAN Interface: {channel}")
    print(f"Bitrate:       1 Mbit/s (must be configured at OS level)")
    print("=" * 50)
    print()
    print_help()

    bus = can_common.create_bus(channel)

    try:
        while True:
            try:
                raw = input("\nTorque> ").strip()
            except EOFError:
                break

            if not raw:
                continue

            parts = raw.lower().split()
            cmd = parts[0]

            if cmd == "quit":
                break

            elif cmd == "help":
                print_help()

            elif cmd == "estop":
                can_system.send_estop(bus, can_common.NODE_PI)
                print("ESTOP broadcast sent")

            elif cmd == "stop":
                for nid, name in NODES.items():
                    can_motor.send_torque_cmd(bus, nid, 0.0)
                    print(f"  {name}: 0.000 Nm")
                print("All joints zeroed")

            elif cmd == "all":
                if len(parts) != 2:
                    print("Usage: all <torque_nm>")
                    continue
                try:
                    torque = float(parts[1])
                except ValueError:
                    print("Torque must be a number")
                    continue
                for nid, name in NODES.items():
                    can_motor.send_torque_cmd(bus, nid, torque)
                    print(f"  {name}: {torque:.3f} Nm")

            else:
                if len(parts) != 2:
                    print("Usage: <node_id> <torque_nm>  (type 'help' for commands)")
                    continue
                try:
                    node_id = int(parts[0])
                    torque = float(parts[1])
                except ValueError:
                    print("node_id must be int, torque must be float")
                    continue

                if node_id not in NODES:
                    print(f"Unknown node {node_id}. Valid: {list(NODES.keys())}")
                    continue

                can_motor.send_torque_cmd(bus, node_id, torque)
                iq = torque / KT_EFFECTIVE
                print(f"Sent {torque:.3f} Nm to {NODES[node_id]} "
                      f"(STM32 will command ~{iq:.3f} A)")

    except KeyboardInterrupt:
        print("\nCtrl+C: sending ESTOP...")
        can_system.send_estop(bus, can_common.NODE_PI)

    finally:
        bus.shutdown()
        print("CAN bus closed")


if __name__ == "__main__":
    main()
