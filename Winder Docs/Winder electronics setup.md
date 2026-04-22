Designing a winder to make magnetic torque rods for the satellite projects. Need to wind some very thin wire consistently along the length of a metallic rod. The challenge is even winding wire that is very easy to tear and break. Thankfully, the consistent winding can be achieved directly through tension control. More info on winder physics here https://library.e.abb.com/public/548d998c195241aca84f170d2561a8b7/G563e%20%20Part%203%20DCS800-Winder%20WINDER_PHYSICS_01R0201.pdf

Electronics for the winder include:
- STM32F303K8 microcontroller
	- datasheet in folder
- Pololu stepper motor
	- turning the spindle to wrap torque rod
	- info: https://www.pololu.com/product/1472
- Pololu stepper motor driver
	- drives the stepper motor, controlled by the STM32
	- info: https://www.pololu.com/product/3765/resources
	- datasheet: https://www.ti.com/lit/ds/symlink/drv8434a.pdf 
- AS5600 magnetic sensor
	- can detect a rotating magnetic field, magnet will be on the pivot axis of a dancer arm to provide tension feedback
	- datasheet in folder
	- generic C drivers, need modification to work (github copilot in VS code is helpful for this): https://github.com/raulgotor/ams_as5600/tree/master
- Pololu 4887 dc motor with encoder
	- acts as a brake on the supply spool to control tension
	- info: https://www.pololu.com/product/4887
- Pololu 2997 DC motor driver
	- controls dc motor
	- info/datasheet: https://www.pololu.com/product-info-merged/2997


I HIGHLY recommend installing VS code and making github account, it will give you access to github copilot (free Claude code)
STM32CubeIDE needed to program the microcontroller: https://www.st.com/en/development-tools/stm32cubeide.html

