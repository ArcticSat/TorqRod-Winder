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
Speed: Standard Mode 100kHz

Pins (LQFP32):
  PB6  → I2C1_SCL
  PB7  → I2C1_SDA

Parameter Settings:
  I2C Speed Mode:        Standard Mode
  I2C Clock Speed (Hz):  100000

Enable DMA or interrupt as needed (polling fine for initial bring-up).

---

## 3. TIM2 — Stepper STEP Pulse Generator
Mode: PWM Generation CH1

Pin:
  PA0  → TIM2_CH1   (STEP signal to A4988)

GPIO:
  PA1  → GPIO_Output (DIR)
  PB1  → GPIO_Output (EN, active LOW for A4988)

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
  PA7  → TIM3_CH2   (PWM2 on 2997)
  PA9  → DC_EN (EN on 2997)

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
| Pin  | Label    | Direction                     | Speed | Notes                 |
| ---- | -------- | ----------------------------- | ----- | --------------------- |
| PA0  | TIM2_CH1 | Alternate Function Push Pull  | Low   | Stepper STEP          |
| PA1  | STEP_DIR | Output Push Pull              | Low   | Stepper DIR           |
| PA2  | USART_TX | Alternate Function Push Pull  | High  | USART for debug       |
| PA3  | USART_RX | Alternate Function Push Pull  | High  | USART (not used)      |
| PA6  | TIM3_CH1 | Alternate Function Push Pull  | High  | DC driver PWM1        |
| PA7  | TIM3_CH2 | Alternate Function Push Pull  | High  | DC driver PWM2        |
| PA9  | DC_EN    | Output Push Pull              | High  | DC EN (low to enable) |
| PA13 | SWDIO    | Input                         | n/a   | For programming       |
| PA14 | SWCLK    | Input                         | n/a   | For programming       |
| PB0  | LED      | Output Push Pull              | Low   | Heartbeat             |
| PB1  | STEP_EN  | Output Push Pull              | Low   | Stepper EN            |
| PB6  | I2C1_SCL | Alternate Function Open Drain | High  | AS5600 clock          |
| PB7  | I2C1_SDA | Alternate Function Open Drain | High  | AS5600 data           |

---

## 8. Code Generation Settings
- Generate peripheral initialization in separate .c/.h pairs
- Generate HAL_Init + MX_xxx_Init stubs in main.c
- Copy necessary library files
