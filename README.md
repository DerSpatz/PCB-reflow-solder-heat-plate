# PCB reflow solder heat plate

## Introduction

The PCB reflow solder heat plate can be used to solder small PCBs with SMD parts. The user can control the maximum temperature and heating curve.

## Hardware design

The idea for this design comes from https://github.com/AfterEarthLTD/Solder-Reflow-Plate. But when I checked the original design, I found some issues that could cause safety problems and other design choices that are not optimal.

### Feature list

- PCB size 100x100mm, heat plate size 81x69mm
- DC barrel plug (5.5x2.5mm), runs with 12V/5A power supply
- resettable 5A PTC fuse
- inrush current limiter to protect power supply when capacitors are discharged
- 2.7mF input power filter for smooth current draw
- capacitor discharging resistors
- ATMEGA4809 MCU, can be programmed via UPDI header
- high power NMOS with gate driver circuit
- flyback diode connected to the heat plate
- AMS1117 LDO
- additional diodes for protection
- dual buttons
- status LED
- 0.91" OLED display
- LMT85 analog temperature sensor as main sensor
- option to add additional digital sensors for calibration purposes

### Building your own

You can order almost all parts needed from LCSC/JLCPCB. Only the OLED display and the power supply need to be sourced elsewhere. The BOM includes LCSC part numbers for hand soldering and their automated SMT manufacturing. Minimal part size is 0805 and there's enough space between all solder pads, so hand soldering should be no problem if you have a steady hand. Hardest part will be the MCU, but this is hand solderable, too.

### Safety measures compared to the original design

The original design had a too low heat plate resistance, which caused very high current spikes when the heat plate was turned on. This should be filtered by PWM and the input capacitors, but the input capacitor in the original design was far too small (100µF). Because of this, the fuse always blew when the device was turned on, so eventually the fuse was removed again.
In my design I made the input capacitors 27 times larger to have a smoother current draw. I also added a fuse, and an inrush current limiter. After disconnecting the power supply, the output capacitors are quickly discharged to GND.
The original also had no gate driver, so the NMOS was only driven with logic levels. I added a simple NPN gate driver.
Switching inductive loads (like the long heat plate trace) causes voltage spikes. In the original design, the filtering was done with an ESD diode and a small capacitor. The usual way to filter these voltage spikes is a flyback diode.
The resistance of the heat plate was very low, so the current draw with 100% PWM duty cycle was very high. If the MCUs hangs up and the power MOSFET stays active, the original design could cause too much stress for the power supply. Furthermore, even with a hot heat plate, the resistance would still be so low that the board would draw more than 5A.
In my design, the heat plate resistance was chosen in a way that a hot heat plate will draw less than 5A, so it is much safer to run the new design, as it will automatically reach electrically safe working conditions.

## Firmware

The original firmware was reworked by Nathan Heidt (https://github.com/heidtn) and works mostly, only the temperature sensing is incorrect (as the firmware was written with an earlier board revision). So for now, you just need to unplug the device when the PCB that needed soldering is done. Also, the dual color LED is not yet included into the code, for now it just lights green to show power. And the buttons sometimes don't react properly.

Daniel Oltmanns (https://github.com/oltdaniel) is doing a rework of the firmware using platformio.

### To-do list for software:
- use of the dual color LED (green = power, orange = heating, red = hot)
- better temperature sensing
- better working buttons
- calibration function for the analog temperature sensor by taping a digital probe directly to the heat plate

Please note that I can only do hardware design, so I can't contribute anything to the software. 

### Programming the MCU

The MCU can be programmed with JTAG2UPDI (https://github.com/ElTangas/jtag2updi). For programming, you need an Arduino with ATMEGA328p (Uno or Nano), some wires, a 4.7k resistor and  a 10µF capacitor or 120 Ohm resistor to disable the auto-reset.

JCM from the Discord explained the process pretty good:

> 1. Download/Clone this project: https://github.com/ElTangas/jtag2updi and rename the folder "source" to "jtag2updi" (otherwise the Arduino IDE won't like it)
> 2. Open jtag2updi/jtag2updi.ino in your Arduino IDE
> 3. Configure the flasher options for your Arduino Nano and flash it
> 4. Connect D6 of your Arduino Nano over the 4.7kOhm resistor to the UPDI pin of the board and 5V to 5V and GND to 0V
> 5. Add the MegaCoreX hardware package to the Ardunio IDE (see https://github.com/MCUdude/MegaCoreX#how-to-install)
> 6. Install the Adafruit_GFX, Adafruit_SSD1306, DallasTemperature and Debounce2 libraries with the Library Manager (you might not need all of them depending on which firmware you plan to use)
> 7. Download and open the ino you want to upload to the ATMEGA4809 (https://github.com/DerSpatz/PCB-reflow-solder-heat-plate/blob/main/Firmware/pcb_reflow_fw/pcb_reflow_fw.ino)
> 8. Select the options for the programmer (Board: ATmega4809, Clock: Internal 16 MHz, BOD: 2.6V or 2.7V, EEPROM: retained, Pinout: 48 pin standard, Reset pin: Reset, no Bootloader) and select the port of your Ardunio Nano as Port
> 9. Select Burn Bootloader and see if it runs through
> 10. Temporarily disable auto reset for the Arduino Nano: https://playground.arduino.cc/Main/DisablingAutoResetOnSerialConnection/ (not sure if it's needed for the Nano, it was for my Mega)
> 11. Select Sketch > Upload using Programmer (normal Upload will not work)

## Thanks

Special Thanks go out to:

Nathan Heidt for writing the software and testing

Merlin Shaw (www.facebook.com/GeekIslandGaming) for ordering the first batch of prototypes and testing

![heatplate PCB front](https://github.com/DerSpatz/PCB-reflow-solder-heat-plate/blob/main/Heatplate_v1.1/renders/Heatplate_v1.1_front.png)
![heatplate PCB back](https://github.com/DerSpatz/PCB-reflow-solder-heat-plate/blob/main/Heatplate_v1.1/renders/Heatplate_v1.1_back.png)
