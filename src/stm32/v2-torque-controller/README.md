# STM32F446RET6 Clock Configuration — HSE + PLL Setup

## Background

The NUCLEO development board used during testing defaults to using an HSE clock in bypass mode — it receives its clock signal directly from the on-board ST-Link probe rather than from a dedicated crystal. This is different from the default behaviour of the bare STM32F446RET6 chip, which defaults to the internal 16 MHz HSI RC oscillator when no external clock source is configured.

The electrical team has added an 8 MHz external crystal oscillator to all final joint board PCBs. As a result, the clock configuration must be manually updated in STM32CubeIDE to use the HSE and PLL rather than the default HSI. Additionally, a new STM32CubeIDE project must be created targeting the bare **STM32F446RET6 chip directly**, not the NUCLEO-F446RE board profile.

### Why HSE + PLL?

The external crystal provides significantly better frequency accuracy and stability compared to the internal HSI oscillator, which can drift ±1–2% with temperature and voltage. This matters for time-sensitive peripherals like CAN where all nodes on a bus must agree on precise timing. The PLL then multiplies the 8 MHz crystal up to 180 MHz, giving the chip its maximum processing headroom.

---

## Step 1 — Enable HSE in the RCC Pinout

In the **Pinout & Configuration** tab:

1. In the left panel under **System Core**, click **RCC**
2. Set **High Speed Clock (HSE)** → **Crystal/Ceramic Resonator**

This activates the OSC_IN and OSC_OUT pins and tells CubeMX that a physical crystal is present on the board.

---

## Step 2 — Configure the Clock Tree

Click the **Clock Configuration** tab and work through the settings left to right.

### 2a — Switch PLL Source to HSE

Find the **PLL Source Mux** and select **HSE**.

### 2b — Set PLL Multipliers

The STM32F446 PLL output formula is:

```
f_VCO  = (HSE / PLLM) × PLLN    →    VCO must stay between 100–432 MHz
SYSCLK = f_VCO / PLLP
```

The PLLM divider runs first to bring HSE down to a 1–2 MHz reference, then PLLN multiplies that up to the VCO frequency.

Configure the PLL as follows to achieve 180 MHz SYSCLK from the 8 MHz crystal:

| Parameter | Value | Calculation |
|-----------|-------|-------------|
| PLLM | 8 | 8 MHz ÷ 8 = 1 MHz reference |
| PLLN | 360 | 1 MHz × 360 = 360 MHz VCO |
| PLLP | 2 | 360 MHz ÷ 2 = **180 MHz SYSCLK** |

### 2c — Select PLLCLK as System Clock

Find the **System Clock Mux** and select **PLLCLK**.

### 2d — Set Bus Prescalers

APB1 has a maximum of 45 MHz on the F446, so it must be divided down from 180 MHz. CubeIDE will highlight any bus in red if you exceed its maximum.

| Bus | Prescaler | Resulting Clock | Maximum |
|-----|-----------|-----------------|---------|
| AHB (HCLK) | /1 | 180 MHz | 180 MHz |
| APB1 (PCLK1) | /4 | 45 MHz | 45 MHz |
| APB2 (PCLK2) | /2 | 90 MHz | 90 MHz |

---

## Step 3 — Recalculate CAN Bit Timing

With APB1 now running at 45 MHz instead of the default 16 MHz, the CAN bit timing registers must be recalculated. The baud rate formula is:

```
Baud Rate = f_APB1 / (Prescaler × (1 + BS1 + BS2))
Prescaler × (1 + BS1 + BS2) = 45,000,000 / 1,000,000 = 45
```

For 1 Mbps with APB1 = 45 MHz, use the following values:

| Parameter | Value |
|-----------|-------|
| Prescaler | 5 |
| Time Quanta in Bit Segment 1 (BS1) | 6 |
| Time Quanta in Bit Segment 2 (BS2) | 2 |
| ReSynchronization Jump Width (SJW) | 1 |

**Verification:**

- Time Quantum (TQ) = 5 / 45,000,000 ≈ 111.1 ns
- Bit time = (1 + 6 + 2) × 111.1 ns = 9 × 111.1 ns = **1000 ns = 1 µs**
- Baud rate = **1,000,000 bit/s ✓**
- Sample point = (1 + 6) / 9 = **77.8% ✓** (target range: 75–87.5%)

---

## Step 4 — Flashing the Custom Board with ST-LINK/V2 (SWD)

The custom PCB has no on-board ST-Link, so flashing is done with an external **ST-LINK/V2** probe wired to the board's debug header in **SWD mode**. Only four wires from the probe are required:

| ST-LINK/V2 pin | Board pin     | Notes |
|----------------|---------------|-------|
| SWDIO          | PA13          | data line, also called JTMS |
| SWCLK          | PA14          | clock line, also called JTCK |
| GND            | GND           | required |
| NRST           | NRST (pin 7)  | optional but recommended — enables "connect under reset" |

VCC from the probe is **not** wired to the board — the PCB is powered separately by the joint's main supply. Make sure the ST-LINK and the board share a common ground.

### Project debug configuration

The committed `v2-torque-controller.launch` file is pre-configured for this exact setup:

| Setting                | Value                  | Why |
|------------------------|------------------------|-----|
| GDB server             | ST-LINK GDB Server     | matches ST-LINK/V2 |
| Interface              | SWD (`swd_mode=true`)  | only PA13/PA14 are reserved by the `.ioc` |
| Reset strategy         | Connect under reset    | works even if the firmware is currently asserting an infinite loop or wedging the clock |
| Verify flash download  | enabled                | catches silent ST-LINK comm errors before you waste a debug session |
| SWV trace HCLK         | 180 MHz                | matches the 180 MHz system clock so SWV timing is correct **if** you later enable trace |
| Stop at `main`         | enabled                | classic CubeIDE default; uncheck if you need to debug the startup code |

To flash and debug: right-click the project in CubeIDE → **Debug As → STM32 C/C++ Application**, or just hit the bug icon. CubeIDE will build, download to flash, and halt at `main`.

To flash without debugging (one-shot programming): **Run As → STM32 C/C++ Application** uses the same launch config but doesn't open the debug perspective.

### Pre-flight checks for a brand-new board

1. **BOOT0 = GND.** The chip boots from system memory (DFU bootloader) when BOOT0 is high. The PCB should pull BOOT0 to ground via a resistor; if the chip ignores ST-LINK and refuses to flash, this is the first thing to check.
2. **NRST debounce.** A long pull-down or RC delay on NRST will keep the chip in reset longer than the ST-LINK expects; "connect under reset" can time out. The CubeMX-generated reset strategy gracefully retries, but if you see "no target found" errors, briefly disconnect NRST during the connect attempt.
3. **VDD stable before connecting SWD.** Power the board first, then plug the ST-LINK; hot-plugging SWDIO into an unpowered chip can latch up the pin.
4. **PCB pull-ups.** SWDIO needs a weak pull-up (~40 kΩ). The chip has one internal, so a missing external pull-up is usually OK on a clean PCB but can cause flaky enumeration on noisy boards — add a 10 kΩ external if needed.
5. **Read protection (RDP).** A fresh chip ships at RDP=0 (unprotected). If a previous flash accidentally raised RDP to level 1, the ST-LINK will connect but flash writes will fail silently. Use `STM32CubeProgrammer` → Option Bytes → set RDP back to AA to recover.

---

## Step 5 — Diagnostic LED (PC13)

The v2 PCB does not break out USART2, so there is **no serial console**. The single status LED on PC13 is the only out-of-band signal the firmware uses to tell you what state the board is in:

| LED behavior        | Meaning                                                              |
|---------------------|----------------------------------------------------------------------|
| Solid OFF           | MCU not running (no power, NRST held low, or hung pre-`HAL_Init`)    |
| Solid ON forever    | **`CAN INIT FAILED`** at boot (HAL filter setup rejected — almost never happens) |
| Fast blink (5 Hz)   | **`IMU NOT FOUND`** at boot (WHO_AM_I read failed; check I²C wiring, SA0 strap, IMU power) |
| Slow blink (1 Hz)   | Running normally — IMU connected, main loop ticking                  |

There is no other indication. If the LED is doing the slow blink and the Pi still sees no frames from this node, the problem is on the CAN bus itself (transceiver, wiring, termination, or the Pi's interface), not the firmware.

### When SWO trace would help (optional)

The `.ioc` does **not** reserve PB3 for SWO (`SYS_JTDO-SWO`). If you want richer debug output without re-spinning the PCB, you can re-enable SWO printf via the ST-LINK: in CubeMX go to `SYS` → set Debug to **Trace Asynchronous SW**, then in `Core/Inc/main.h` retarget `_write` to push to the ITM stimulus port. CubeIDE's SWV viewer will display the stream live without needing a UART pin.

