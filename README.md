# LPC824_MAX30102

A simple UART interface to one or more MAX30102 SPO2 sensor mounted on a breakout board 
(part GY-MAX30102) using NXP LPC824 MCU.

![overnight test run](https://github.com/jdesbonnet/PPG_Tools/blob/master/doc/sleep.png)

## 1 Aug 2017, version 2.2.2

* UART command 'M' (32 samples means), 'm' (no means)
* Continue to have reliablity issues when more than one sensor. Cause not determined yet.
 
## 31 July 2017, version 0.2.1

* Remove unnecessary register reads during sample read cycle.

## 25 July 2017, version 0.2.0

* More compact output record format with record identifier "$PPG" at start. 
Use of hex for data compaction (binary a step too far for the moment).

Over night recording stopped prematurely due (it seems) to cheap USB/UART
cable spontaneously disconnecting. 

## 24 July 2017, version 0.1.1
* Watch dog timer to recover from I2C bus errors. This is sub-optimum.

## 23 July 2017, version 0.1

* Support two MAX30102 sensors. Since the sensor has a fixed address they cannot share
the same I2C bus. Using I2C0 and I2C1. Up to 4 sensors can be supported this way.
* Known issue: can hang in an infinite loop while waiting for I2C (presumably due to 
bus error). Need to timeout I2C reads or implement watchdog timer to reset.
* Known issue: long sensor leads and high UART baud rate (460800bps) causing data
corruption in a significant number or records: use lower I2C bus clock and perhaps
run two I2C transfer in parallel. Also use shorter message format on UART and 
lower bit rate.
* Over night run with two MAX30102 sensors (thumb + toe) resulted in some good and
interesting data, but some work required to clean up due to above data corruption
issue.


## GY-MAX30102

Marked as "GY-MAX30100" on PCB. Cannot find schematic. PCB incorporates LDO and level converters. Can operate at 3.3V to 5V.
INT pin needs pull up resistor (either external or enabled within the MCU).

Breakout board components

* "662K" 3.3V LDO (presumably for LED driver)
* "65K5" 1.8V LDO  (for digital logic)
* "702" 60V N-channel enhancement mode MOSFET (as logic level convertor)

Using FET for level shifting:

AN97055 Bi-directional level shifter for I²C-bus and other systems.
https://cdn-shop.adafruit.com/datasheets/an97055.pdf

Datasheets:

http://www.mikrocontroller.net/attachment/193855/LM6206N3.pdf
http://www.s-manuals.com/pdf/datasheet/2/n/2n7002dw_panjit.pdf
https://datasheets.maximintegrated.com/en/ds/MAX30102.pdf

Papers:

[An Automatic Beat Detection Algorithm
for Pressure Signals](./doc/BeatDetection.pdf)


