
## DC Motor Driver (TB9051FTG)
***Do NOT power the motor from the Nucleo's 5V pin.** The Pololu 4887 has a stall current of ~1.6A, which will instantly fry your Nucleo board if powered via USB. You **must** use a separate power supply.*
Powering the driver:
	1. Connect VIN on the driver to the power line.
	2. Connect GND on the driver screw terminal to common ground.

Connecting to MCU:
	1. Connect GND on the driver to GND on the Nucleo (any is fine)
	2. Connect VCC on the driver to 3V3 on the Nucleo
	3. Connect EN on the driver to A5 on the Nucleo
	4. Connect PWM1 on the driver to A6 on the Nucleo
	5. Connect PWM2 on the driver to D9 on Nucleo

Connecting to motor:
	1. The motor's power leads are red and black. Connect these to OUT1 and OUT2 on the driver (any order)
