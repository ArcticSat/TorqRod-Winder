# Protoboard Assembly Guide – Winder Driver & Sensor Interface

## Overview
This protoboard serves as the central interface between the **STM32F303K8 Nucleo MCU**, the **DC motor driver**, the **stepper motor driver**, and the **AS5600 magnetic encoder**. Due to physical spacing constraints, this board will house the drivers and provide terminal blocks/headers for extended harnessing to the motors and sensor.

> **Critical Warning:** NEVER power the DC motor driver from the Nucleo's 5V or 3.3V pins. The stall current (~1.6A) will instantly damage the MCU. All motor power must come from dedicated, isolated supplies with a **common ground**.

---

## Required Components & Materials
| Item                 | Specification                                   | Notes                                   |
| -------------------- | ----------------------------------------------- | --------------------------------------- |
| MCU Board            | STM32F303K8 Nucleo-32                           | Programming/debug via SWD               |
| DC Motor Driver      | TB9051FTG / Pololu 2997                         | 12V motor supply                        |
| Stepper Driver       | A4988                                           | 12V motor supply                        |
| Encoder Breakout     | AS5600 Adapter Board                            | I2C interface                           |
| Power Supply         | 12V DC                                          | Separate from MCU USB                   |
| Decoupling Capacitor | ≥100µF, 12V+ rated                              | Electrolytic/polarized for stepper VMOT |
| Pullup Resistor      | 2x 3.3 kΩ resistors                             | For I2C pullups                         |
| Connectors           | 2-pin, 4-pin, 5-pin screw terminals or JST      | For motor/sensor harnesses              |
| Headers/Pins         | 0.1" pitch, male/female                         | MCU & AS5600 breakout                   |
| Wire                 | 18-22 AWG (power), 22-24 AWG (logic)            | Stranded for flexibility                |
| Miscellaneous        | Jumper wires, heat shrink, zip ties, label tape | Harness organization                    |

---

## 📐 Board Layout Recommendations
1. **Driver Placement:** Mount drivers close to the power input terminals to minimize trace resistance and voltage drop.
2. **Logic Grouping:** Keep all MCU-facing connections (EN, STEP, DIR, PWM, I2C) clustered on one side of the board for clean routing to the Nucleo headers.
3. **Harness Terminals:** Place motor/sensor terminal blocks on the edge of the board facing outward for easy cable routing.
4. **Ground Plane:** Use a continuous copper pour or thick bus wire for `GND`. All supplies and drivers must share a single common ground.
5. **Separation:** Keep high-current paths (8V/12V) physically separated from low-voltage I2C/sensor lines to reduce EMI.

---

## 🔌 Power Distribution Scheme

The goal with power distribution is to take all input power from a 12 V DC power supply, and provide 12 V power to both motors, and regulated power to drivers and sensors. The MCU Nucleo board does NOT provide consistent voltage, please do not use these for power.

| Supply   | Voltage | Connected To                          | Notes                                     |
| -------- | ------- | ------------------------------------- | ----------------------------------------- |
| `12V DC` | 12V     | DC Driver `VIN` & `GND`               | Dedicated supply, isolated from MCU USB   |
| `12V DC` | 12V     | Stepper Driver `VMOT` & `GND`         | Place 100µF+ capacitor across these rails |
| `DC 3V3` | 3.3V    | AS5600 `VDD`                          | Logic power                               |
| `DC 5V`  | 5V      | Stepper Driver `VDD`, DC Driver `VCC` | Driver logic power                        |
| `COMMON` | GND     | All `GND` pins, MCU `GND`, Supplies   | **Must be star-grounded or heavily tied** |

---

## 🔗 Pin-to-Pin Wiring Reference

### 1. DC Motor Driver → MCU & Power
| Driver Pin    | Nucleo Header | STM32 Pin | Function                                  |
| ------------- | ------------- | --------- | ----------------------------------------- |
| `VIN`         | —             | —         | Connect to 12V supply +                   |
| `GND`         | Any `GND`     | —         | Common ground                             |
| `VCC`         | `5V`          | —         | 3.3V logic power                          |
| `EN`          | `D1`          | PA9       | Enable driver (logic high)                |
| `PWM1`        | `A5`          | PA6       | TIM3_CH1 (Motor + PWM)                    |
| `PWM2`        | `A6`          | PA7       | TIM3_CH2 (Motor - PWM)                    |
| `ENB`         | `GND`         | —         | Tie low to enable                         |
| `OUT1`/`OUT2` | —             | —         | To DC Motor Harness (polarity irrelevant) |

### 2. Stepper Driver → MCU & Power
| Driver Pin        | Nucleo Header | STM32 Pin | Function                              |
| ----------------- | ------------- | --------- | ------------------------------------- |
| `SLEEP` ↔ `RESET` | —             | —         | Jumper together (keeps driver active) |
| `VDD`             | `5V`          | —         | 5V logic power                        |
| `GND`             | Any `GND`     | —         | Common ground                         |
| `STEP`            | `A0`          | PA0       | TIM2_CH1 (Pulse generator)            |
| `DIR`             | `A1`          | PA1       | Direction control (logic high/low)    |
| `EN`              | `D6`          | PB1       | Enable (active LOW)                   |
| `A1`/`A2`         | —             | —         | To Stepper Coil 1 Harness             |
| `B1`/`B2`         | —             | —         | To Stepper Coil 2 Harness             |
| `VMOT`/`GND`      | —             | —         | Connect to 12V supply + capacitor     |

### 3. AS5600 Encoder Header → MCU
Create a 4-pin male header on the board for the sensor harness:

| Header Pin | Nucleo Header | STM32 Pin | Function              | Notes                               |
| ---------- | ------------- | --------- | --------------------- | ----------------------------------- |
| VDD        | 3V3           | —         | 3V3 Power             | Jumper `J1` closed on adapter board |
| GND        | GND           | —         | Ground                | —                                   |
| DIR        | GND           | —         | Measurement direction | To flip direction, connect to 3V3   |
| SCL        | D5            | PB6       | I2C Clock             | Add 3.3kΩ pull-up to 3.3V           |
| SDA        | D4            | PB7       | I2C Data              | Add 3.3kΩ pull-up to 3.3V           |

---

##  Harnessing Specifications
Since motors and the sensor are physically distant, follow these rules for the cable runs:
- **DC Motor Harness:** 2-wire, 20-18 AWG stranded. Label `DC+` / `DC-`.
- **Stepper Motor Harness:** 4-wire, 22-20 AWG stranded. Use an Molex or JST 4-pin connector. Label `A1`, `A2`, `B1`, `B2`.
- **AS5600 Harness:** 4-wire shielded or twisted pair, 24 AWG. Keep length ≤1m to preserve I2C signal integrity. Label `VDD`, `GND`, `SCL`, `SDA`.
- **Routing:** Keep motor power cables at least 2-3 inches away from I2C/sensor cables. Avoid running parallel to step/dir signals.
- **Strain Relief:** Use zip ties or hot glue near terminal blocks. Add heat shrink on all exposed crimps.

---

##  Assembly & Calibration Steps
1. **Dry Fit:** Place all components on the protoboard. Verify clearances for headers, screw terminals, and the decoupling capacitor.
2. **Solder/Wire:** 
   - Solder power buses first (`GND`, `3V3`, `5V`, `12V`).
   - Route logic wires next. Keep them short and tidy.
   - Install terminal blocks and headers.
3. **Capacitor Installation:** Solder the 100µF+ capacitor directly across the stepper `VMOT` and `GND` pins. **Observe polarity.**
4. **Potentiometer Calibration:** 
   - Power only the stepper driver logic (`VDD` 5V).
   - Measure `VREF` on the driver potentiometer with a multimeter.
   - Adjust until `VREF = 0.70 – 0.80 V` (sets 1A current limit: `Vref = I_lim × 8 × 0.1Ω`).
5. **Continuity Check:** Verify no shorts between `VCC`/`GND`, and that all signal traces reach their intended pins.
6. **I2C Pull-ups:** If the AS5600 adapter board has `R4`/`R5` unpopulated, solder 4.7kΩ resistors from `SCL`/`SDA` to `3.3V`.

---

## Pre-Power Verification Checklist
- [ ] Common `GND` connects Nucleo, 12V supply, and both drivers.
- [ ] DC motor driver `VIN` is **NOT** connected to Nucleo `5V` or `3.3V`.
- [ ] Stepper `SLEEP` and `RESET` are jumpered together.
- [ ] `VMOT` decoupling capacitor is correctly polarized and rated ≥12V.
- [ ] Stepper `VREF` measures 0.7–0.8V.
- [ ] I2C pull-ups are installed (if needed).
- [ ] All harness wires are labeled and terminated securely.
- [ ] No stray solder bridges or loose strands on the board.

---

## 🔍 First Power-On Sequence
1. Connect MCU via USB only. Verify `3V3` and `5V` are stable.
2. Power 12V motor supply. Check for abnormal heating or buzzing.
3. Run firmware I2C scan to verify AS5600 responds at address `0x36`.
4. Test stepper at low step rate (~50 steps/s). Verify direction and enable logic.
5. Test DC motor at low PWM duty cycle (~10%). Verify smooth rotation and no driver thermal throttling.
6. Ramp to operational speeds while monitoring driver temperatures.

---

*For firmware pin mappings, refer to `CUBEMX_SETUP.md`. If any pin conflicts arise during testing, cross-check the Nucleo-F303K8 silkscreen with the STM32F303K8 datasheet.*