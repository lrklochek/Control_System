# PID Temperature Control of a Miniature Thermal Chamber
The PID Temperature Control of a miniature thermal chamber is a system designed for educational purposes. It can be used as an inexpensive lab for developing an understanding of process control and the effects of process dynamics on selecting the optimum PID (proportional, integral, and derivative) tuning constants.
The system can be implemented inexpensively and would be ideal for a student doing an on-line course in process control systems.
The system allows operating, PI, or PID control with reverse or forward (direct) acting control., by using selectable final control elements (resistive heater, or variable fan speed).
The system provides 2 sets of process dynamics by tightly coupling a temperature sensor to the resistive heater or alternatively using a sensor in the centre of the chamber. These very different process dynamics require very different sets of PID tuning.

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

