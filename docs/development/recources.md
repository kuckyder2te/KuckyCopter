# Recources

## UART
GP8 and GP9 are Serial2 !!
Current COM4 115200bps

## Degugger
#define LOCAL_DEBUG = LOGGER ON<br>
#define LOCAL_DEBUG = LOGGER OFF<br>
#include "myLogger.h"

## Propeller rotation order
![propeller](../images/Motor_Rotation_order.jpg)

## CW red screw nut<br>
## CCW black screw nut<br>

## Position LEDs
![Position_LED](../images/Back-Steuerbord.jpg)
1 second ON<br>
9 seconds OFF<br>

## Battery state 
[LiPo description](https://fpvracing.ch/de/content/21-lipo-batterien)

|Battery voltage |analog|
|----------------|------|
|12,2V | 933|
|11,3V | 872|
|10,1V | 777|
| 9,5V | 730|
| 8,1V | 626|

## OneWire Library
 The oneWire library must be exchanged for the one_wire library. Modified timings so that it works when used on the Pi Pico.<br>
 look here:
 [Link](https://github.com/adamboardman/pico-onewire/tree/main)

