
### STM32 F303K8
There is a pretty detailed setup process for this, and it is important that the MCU is set up the same so that pins match. See [[STM32 F303K8 setup]].

### Stepper Motor

Find coil pairs:
	1. Rotate the output shaft of the motor, feel how much resistance there is.
	2. Connect a pair of wires from the stepper motor using a jumper wire.
	3. Rotate the output shaft again. If there is more resistance, these are from a coil pair! Keep this pair together.
	4. Find both coil pairs using this method. We will use this to connect to our motor driver later.

### Stepper Motor Driver
Following this tutorial https://www.youtube.com/watch?v=wcLeXXATCR4
Power the driver:
	1. Locate the SLEEP and RESET pins. Put a short jumper wire between the two. This will keep the driver powered on
	2. Locate the GROUND and VDD pins. Connect GROUND to the common ground. Connect VDD to the 5V output on the NUCLEO board.
	
Set the potentiometer:
	1. Follow the video, the Rachel way works great
	2. V_ref = I_lim * 8 * R_sense
	3. Stepper I_lim = 1 A, driver R_sense = 0.1 Ohm
	4. Therefore we want V_ref = 1.1-1.2 V

Connect to MCU:
	1. Connect DIR on driver to A1 on the NUCLEO (PA_1 on chip).
	2. Connect STEP on driver to A0 on the NUCLEO (PA_0 on chip).
	3. Connect EN on driver to D6 on the NUCLEO (PB_1 on chip).

Connect to stepper motor:
	1. Locate the A1, A2, B1, and B2 pins on the driver. These correspond to the coil pairs on the motor.
	2. With the motor's coil pairs identified, connect one pair to A1 and A2 (order does not matter), and the other to B1 and B2.

Connect to power supply:
	1. Find a capacitor. Make sure it is at least 100 microF rated for at least 12 V. For the breadboard setup I am using 220 microF rated for 25 V.
	2. Connect the capacitor to the breadboard. If it is polarized, identify the positive and negative load pins (positive pin is longer).
	3. Put the positive pin in the positive power terminal, and the negative pin to ground. 
	*On the breadboard, the common lines are convenient, but on a protoboard, the capacitor should be as close as possible to the driver's pins.*
	4. Place jumper wires on one end of the common power/ground lines. This will make the connection to the power supply.
	5. On the opposite side of the capacitor from the input lines, connect a jumper between the power line and VMOT on the driver, and the ground line and the GND pin next to VMOT.


Congrats! The hardware setup is complete.
Set the power supply to 12 V, and you should be able to run the stepper using the drivers in the firmware.



