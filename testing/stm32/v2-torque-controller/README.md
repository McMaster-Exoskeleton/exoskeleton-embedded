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
f_VCO  = HSE × (PLLN / PLLM)    →    VCO must stay between 100–432 MHz
SYSCLK = f_VCO / PLLP
```

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

