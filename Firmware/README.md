# PCB reflow firmware
This firmware is for the PCB reflow hotplate designed by Spatz.

It has the following features:
- reflow profiles
- OLED display
- profile PID tracking
- temperature setpoint tracking
- safe voltage calculation
- memory validation 

## Build and load process
to build and load, you need a UDPI programmer and core definitions for the Atmega4809.

If you don't have a UDPI programmer you can use another arduino like in this link:
https://github.com/ElTangas/jtag2updi 

You can download the Atmega4809 definitions here:
https://github.com/MCUdude/MegaCoreX/tree/master

Make sure the DallasTemperature and Adafruit display libraries are installed. 

build and load through the arduino IDE as normal.  Select the Atmega4809 as the target
and make sure the correct programmer is selected.

## TODO