# STM32F303K8 — CubeMX Peripheral Configuration
# Winder Firmware

---

## 1. Clock Configuration
- HSI 8MHz → PLL → 64MHz SYSCLK
- APB1 = 32MHz (I2C, TIM2/3)
- APB2 = 64MHz (TIM1)

In CubeMX: Clock Configuration tab
  PLLMUL = x16, PLLDIV = /2 → 64MHz

---

## 2. I2C1 — AS5600 Encoder (CM7)
Mode: I2C
Speed: Fast Mode 400kHz

Pins (LQFP32):
  PB6  → I2C1_SCL
  PB7  → I2C1_SDA

Parameter Settings:
  I2C Speed Mode:        Fast Mode
  I2C Clock Speed (Hz):  400000

Enable DMA or interrupt as needed (polling fine for initial bring-up).

---

## 3. TIM2 — Stepper STEP Pulse Generator
Mode: PWM Generation CH1

Pin:
  PA0  → TIM2_CH1   (STEP signal to TMC2208/A4988)

GPIO:
  PA1  → GPIO_Output (DIR)
  PB1  → GPIO_Output (EN, active LOW for TMC2208)

Configuration:
  Prescaler:     63        → Timer clock = 64MHz / 64 = 1MHz
  Counter Mode:  Up
  Period (ARR):  computed at runtime (controls STEP frequency)
  Pulse (CCR1):  = ARR / 2 (50% duty cycle, ~1µs min pulse width)
  Auto-Reload Preload: Enable

STEP frequency formula:
  ARR = (1,000,000 / step_rate_hz) - 1
  step_rate_hz = omega_rad_s * (200 * 16) / (2*PI)   // 200 steps, 1/16 microstep

Range:
  Min omega ~0.01 rad/s → ~51 steps/s   → ARR ≈ 19607
  Max omega ~8.0 rad/s  → ~40960 steps/s → ARR ≈ 23

Enable TIM2 Update interrupt for ramp handling.

---

## 4. TIM3 — DC Spool Motor PWM (Pololu 2997)
Mode: PWM Generation CH1 + CH2

Pins:
  PA6  → TIM3_CH1   (PWM1 on 2997)
  PA4  → TIM3_CH2   (PWM2 on 2997)

Configuration:
  Prescaler:  0          → Timer clock = 32MHz (APB1 × 2)
  Period:     3199       → PWM frequency = 32MHz / 3200 = 10kHz
  Pulse CH1:  0 (set at runtime)
  Pulse CH2:  0 (set at runtime)
  Auto-Reload Preload: Enable


---

## 5. USART2 — Debug (Optional but recommended)
Mode: Asynchronous
Baud: 115200
Pins:
  PA2  → USART2_TX
  PA3  → USART2_RX

---

## 6. SysTick
Leave enabled (HAL timebase). 1ms tick.

---

## 7. GPIO Summary
| Pin  | Function | Direction | Notes                 |
| ---- | -------- | --------- | --------------------- |
| PA0  | TIM2_CH1 | AF        | Stepper STEP          |
| PA1  | DIR      | Output PP | Stepper DIR           |
| PA2  | USART_TX | Output PP | USART for debug       |
| PA3  | USART_RX | Output PP | USART (not used)      |
| PA4  | TIM3_CH2 | AF        | DC motor PWM2         |
| PA6  | TIM3_CH1 | AF        | DC motor PWM1         |
| PA8  | DC_IN    | Output PP | unsure                |
| PA9  | DC_STBY  | Output PP | DC EN (low to enable) |
| PA13 | SWDIO    | N/A       | SYS_JTMS-SWDIO        |
| PA14 | SWCLK    | N/A       | SYS_JTMS-SWCLK        |
| PB0  | LED      | Output PP | Heartbeat             |
| PB1  | STEP_EN  | Output PP | Stepper EN            |
| PB6  | I2C1_SCL | AF OD     | AS5600 clock          |
| PB7  | I2C1_SDA | AF OD     | AS5600 data           |

---

## 8. Code Generation Settings
- Generate peripheral initialization in separate .c/.h pairs
- Generate HAL_Init + MX_xxx_Init stubs in main.c
- Copy necessary library files
