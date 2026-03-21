# UART Character Counter - Setup Guide

## Build and Flash
1. Build the project in STM32CubeIDE
2. Flash firmware to microcontroller
3. Connect UART2 to computer via USB-to-Serial/ST-Link

## Find COM Port

**Windows:** Device Manager → Ports (COM & LPT) → Note COM port number

**Linux:** `ls /dev/ttyACM* /dev/ttyUSB*`

**macOS:** `ls /dev/tty.*`

## Install Dependencies
```bash
pip install pyserial
```

## Configure and Run
1. Edit `uart_test.py` line 15 with your COM port:
   ```python
   PORT = 'COM3'  # Change to your port
   ```

2. Run the test script:
   ```bash
   python uart_test.py
   ```

## Usage
Type a message and press Enter. The microcontroller responds with the character count.

```
Your message: Hello
Sent: 'Hello' (5 characters)
Response: Character count: 5
```

## Troubleshooting
- **Permission denied (Linux):** `sudo usermod -a -G dialout $USER` (log out/in)
- **No response:** Check UART2 pin connections (TX↔RX, RX↔TX)
- **Wrong port:** Verify COM port number
- **Garbled output:** Confirm baud rate is 115200 in both firmware and script
