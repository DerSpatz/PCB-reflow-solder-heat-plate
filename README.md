# PCB reflow solder heat plate
This design is derived from the now obsolete v0.9, which turned out to be working just fine.

V1.0 uses only the analog temperature sensor, but there are two possible options to add digital temperature sensors to calibrate the heatbed in software (feature not yet included in software).

The firmware for the MCU can be found over at Nathan Heidt's GitHub: https://github.com/heidtn/PCB-reflow-solder-heat-plate/tree/main/Firmware
The firmware might need some small changes due to the small changes in the layout between v0.9 and v1.0.

Electrical safety has been considered in this build, inrush current and power draw are limited to safe values under normal conditions and there is a fuse included.

Board dimensions are 100 x 100 mm, and almost all parts can be sourced from LCSC.
This repository includes an interactive BOM with annotations and LCSC parts numbers.

Inspiration comes from https://github.com/AfterEarthLTD/Solder-Reflow-Plate

https://github.com/DerSpatz/PCB-reflow-solder-heat-plate/blob/main/Heatplate_v1.0/renders/Heatplate_v1.0_front.png
https://github.com/DerSpatz/PCB-reflow-solder-heat-plate/blob/main/Heatplate_v1.0/renders/Heatplate_v1.0_back.png
