# PID Temperature Control of a Miniature Thermal Chamber
The PID Temperature Control of a miniature thermal chamber is a system designed for educational purposes. It can be used as an inexpensive lab for developing an understanding of process control and the effects of process dynamics on selecting the optimum PID (proportional, integral, and derivative) tuning constants. The system can be implemented inexpensively and would be ideal for a student doing an on-line course in process control systems. The system allows operating, PI, or PID control with reverse or forward (direct) acting control., by using selectable final control elements (resistive heater, or variable fan speed). The system provides 2 sets of process dynamics by tightly coupling a temperature sensor to the resistive heater or alternatively using a sensor in the centre of the chamber. These very different process dynamics require very different sets of PID tuning.

The system hardware consists of the miniature (100x68x50mm) thermal chamber, a control board and an interface board.

The control board consists of an Arduino Nano, a setpoint potentiometer for setting desire temperature in deg C and a disturbance potentiometer, selectable for either heater or fan. The outputs from the Nano are 2 pulse width modulated signals that produce a variable DC voltage that drives the fan and heater.

The interface board utilizes 2 N Channel power Mosfets and uses the PWM signals from the Nano to produce a PWM output with a maximum voltage of 12VDC to supply the thermal chamber heaters and fan.

The thermal chamber consists or 2 Cement resistor heaters of 5 watts each, a 12VDC fan, and 2 LM35DZ temperature sensors. One of the sensors measures the air temperature of the Chamber, while the other sensor is firmly attached to one of the temperature resistor heaters.

The system hardware consists of the miniature (100x68x50mm) thermal chamber, a control board and an interface board.

The control board consists of an Arduino Nano, a setpoint potentiometer for setting desire temperature in deg C and a disturbance potentiometer, selectable for either heater or fan. The outputs from the Nano are 2 pulse width modulated signals that produce a variable DC voltage that drives the fan and heater.

The interface board utilizes 2 N Channel power Mosfets and uses the PWM signals from the Nano to produce a PWM output with a maximum voltage of 12VDC to supply the thermal chamber heaters and fan.

The thermal chamber consists or 2 Cement resistor heaters of 5 watts each, a 12VDC fan, and 2 LM35DZ temperature sensors. One of the sensors measures the air temperature of the Chamber, while the other sensor is firmly attached to one of the temperature resistor heaters.

The following image shows a block diagram for the system. Note that the Arduino Serial Plotter is used to display the Temperatures and Setpoints. It is also used to generate the following commands:

TM35_1 sets the PID controlled variable as the temperature of one of the cement resistor heaters. In the phot, it is the cement resistor on the right.

TM35_2 sets the PID Controlled variable as the temperature of the sensor in the middle of the chamber.

Reverse sets the PID controller to Reverse acting, where the temperature is controlled by the PID adjusting the voltage to the cement resistor heater. The fan then can be manually adjusted by the potentiometer to act as a disturbance.

Forward sets the PID controller to Forward(Direct) acting where the temperature is controlled by adjusting the fan voltage, and therefor air flow to the chamber. The heater then can be manually adjusted by the potentiometer to act as a disturbance.

Manual disables the PID controller and allows the the setpoint potentiometer to fix either the heater or fan setting from 0 to 100%. There is no feed back. This is referred to as open loop.

Auto enables the PID controller, either Reverse or Forward acting.

The features of the PID Temperature Control of a miniatur

## Thermal Chamber Temperature Control System Features.
## Chamber
* Small volume chamber 100mmx68mmx50mm
* Heater Temperature sensor affixed to resistive heater
*	Air temperature sensor
*	2 resistive heaters (30 and 36 ohms), 12 VDC supply to heaters
*	Maximum heater temperature approx. 135 deg C
*	Maximum air temp about 60 deg C
*	2 distinctly different sets of process dynamics
  
## PID Controller
*	Standard (sometimes called Mixed) algorithm
*	Most commonly used in industry
*	Select Reverse or Forward (Direct) acting
*	Reverse uses heater as final control element
*	Forward uses fan as final control element
*	2 sets of PID tuning constants for 2 different process dynamic cases
*	Auto/Manual selection
  
## Display
* Runs independently with LCD display or along with Arduino Serial Plotter
* LCD
*	Heater temperature deg C
*	Air temperature deg C
*	Setpoint in deg C
*	Fan or Heater manual setting 0 to 100%
*	PID components displayed in real time
*	Proportional component
*	Integral component
*	Derivative component
*	Cout - total of three PID components

## Plotting
*	Uses Arduino IDE serial plotter
*	Plots Heater and air temperatures
*	Plots Setpoint or Manual setting
*	Generates the following commands
* Auto – sets PID controller to automatic
*	Manual – sets system to manual  open loop operation
*	Reverse – sets PID to reverse acting using heater as controller output
*	Forward – sets PID to forward acting using fan as controller output
*	LM35_1 – sets PID controlled variable to heater temperature sensor
*	LM35_2 – sets PID controlled variable to air temperature sensor
 
## Electronics
* Arduino Nano microcontroller*Connects to LCD Display via I2C bus
*	Generates 2 PWM outputs to Power Mosfet Interface board
* C Code generated from Arduino IDE
*	Setpoint and disturbance generated from 2 potentiometers connected to Nano analog inputs 
*	Temperature sensors – LM35 – generate 0 to 1.5 volts for 0 to 150 deg C
*	Power Mosfets IRF540N are switched  at 490 Hz and operate via Pulse Width Modulation to vary the voltage on the heaters

