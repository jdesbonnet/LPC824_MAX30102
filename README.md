# LPC824_MAX30102

A simple UART interface to MAX30102 SPO2 sensor mounted on a breakout board (part GY-MAX30102) using NXP LPC824 MCU.

## GY-MAX30102

Marked as "GY-MAX30100" on PCB. Cannot find schematic. PCB incorporates LDO and level converters. Can operate at 3.3V or 5V. PCB components

* "662K" 3.3V LDO
* "65K5" 1.8V LDO 
* "702" 60V N-channel enhancement mode MOSFET (as level convertor)

Datasheets:

http://www.mikrocontroller.net/attachment/193855/LM6206N3.pdf
http://www.s-manuals.com/pdf/datasheet/2/n/2n7002dw_panjit.pdf
