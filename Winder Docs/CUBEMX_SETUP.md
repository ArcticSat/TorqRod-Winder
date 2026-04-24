# STM32F303K8 — CubeMX Peripheral Configuration
# Inductor Winder Firmware

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
  PA2  → GPIO_Output (EN, active LOW for TMC2208)

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

## 4. TIM3 — DC Spool Motor PWM (DRV8833 IN1/IN2)
Mode: PWM Generation CH1 + CH2

Pins:
  PA6  → TIM3_CH1   (IN1 on DRV8833)
  PA7  → TIM3_CH2   (IN2 on DRV8833)

Configuration:
  Prescaler:  0          → Timer clock = 32MHz (APB1 × 2)
  Period:     3199       → PWM frequency = 32MHz / 3200 = 10kHz
  Pulse CH1:  0 (set at runtime)
  Pulse CH2:  0 (set at runtime)
  Auto-Reload Preload: Enable

DRV8833 drive scheme (slow decay preferred for smooth control):
  Forward:  IN1 = PWM(duty),  IN2 = LOW
  Reverse:  IN1 = LOW,        IN2 = PWM(duty)
  Brake:    IN1 = HIGH,       IN2 = HIGH
  Coast:    IN1 = LOW,        IN2 = LOW

---

## 5. USART2 — Debug (Optional but recommended)
Mode: Asynchronous
Baud: 115200
Pins:
  PA2  → USART2_TX   (NOTE: conflicts with EN if used — reassign EN to PB0)
  PA3  → USART2_RX

---

## 6. SysTick
Leave enabled (HAL timebase). 1ms tick.

---

## 7. GPIO Summary
| Pin | Function        | Direction | Notes                          |
|-----|-----------------|-----------|--------------------------------|
| PA0 | TIM2_CH1        | AF        | Stepper STEP                   |
| PA1 | DIR             | Output PP | Stepper direction (HIGH=fwd)   |
| PA2 | EN (or TX)      | Output PP | TMC2208 enable (active LOW)    |
| PA6 | TIM3_CH1        | AF        | DC motor IN1                   |
| PA7 | TIM3_CH2        | AF        | DC motor IN2                   |
| PB6 | I2C1_SCL        | AF OD     | AS5600 clock                   |
| PB7 | I2C1_SDA        | AF OD     | AS5600 data                    |
| PB0 | LED / spare     | Output PP | Heartbeat                      |

---

## 8. Code Generation Settings
- Generate peripheral initialization in separate .c/.h pairs
- Generate HAL_Init + MX_xxx_Init stubs in main.c
- Copy necessary library files
