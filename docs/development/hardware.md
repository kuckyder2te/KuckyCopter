# Hardware
## Controller
Raspberry Pi Pico Entwicklung Bord TYPE-C High-Leistung Mikrocontroller-board RP2040<br>  128Mbit 4M 16MB Dual-core ARM Prozessor
![RP2040](../images/RP2040.jpg)

## PinOut
![RP2040DevBoardPins](../images/picoPins.jpg)  

## Circut Board
![Top](../images/Board_topview_1.jpg)
1.  Vin
2. Sonic front
3. NRF24L01
4. Sonic down
5. ESC´s
6. Thermosensor
7. State LED´s
8. Position LED´s

![Bottom](../images/Board_backside_1.png)

## IMU
MPU9250/MS5611 CJMCU-117 High Precision 9 Axis 10DOF Attitude Module SPI/IIC Communication
![IMU](../images/IMU.JPG)
![IMU2](../images/IMU_2.JPG)<br  >
**Attention: Pad NCS and PS must be connected**


Chip: MPU-9250<br> 
Power supply: 3-5v (internal low dropout regulator)<br> 
Communication: standard IIC / SPI communications protocol<br> 
Chip built 16bit AD converter, 16-bit data output<br> 
Gyroscopes range: ± 250 500 1000 2000 °/s<br> 
Acceleration range: ± 2 ± 4 ± 8 ± 16g<br> 
Magnetic field range: ± 4800uT<br> 

Module Model: GY-63-03<br> 
Name: MS5611 module (atmospheric pressure module)<br> 
Built-in 24bit AD converter chip<br> 
High quality Immersion Gold PCB, machine welding process to ensure quality<br> 
Use chip: MS5611-01BA03

|Funkt.|Pin |GPIO|
|------|----|----|
|  SDA | 6  | 4  |
|  SCL | 7  | 5  |

## Radio
**NRF24**
![NRF24](../images/NRF24.JPG)

|Funkt.|Pin |GPIO|Color|Stripe|
|------|----|----|-----|------|
| MISO | 21 | 16 |     |   1  | 
| CSN  | 22 | 17 |     |   2  |
| SLK  | 24 | 18 |     |   3  |
| MOSI | 25 | 19 |     |   4  |
| CE   | 26 | 20 |     |   5  |
| 3V3  |    |    | Red |      |
| GND  |    |    |Black|      |

Modell: E01-ML01DP5<br>
Schnitts telle: spi<br>
Leistung: 20dbm<br>
Abstand: 2100m
HF-Verbindungs stück: SMA-K
Frequenz: 2,4 GHz(2400MHz-2525MHz)
Feature: E01-ML01DP5 basiert auf original importierten nrf24l01p Form nordic in Norwegen. Und ausgestattet mit 20dbm Leistungsverstärker chip, der aus den USA importiert wird, wodurch die Sendeleistung 100mw (20dbm) erreicht, während die Empfangsempfindlichkeit um 10db erhöht wird. Diese Eigenschaften machen die Übertragungsdistanz 10 mal länger als nrf24l01p selbst. Die Anti-Interferenz-Abschirmung Abdeckung auf dem Modul macht die Anti-Interferenz-Leistung besser.

## Sonic
**HC-SR04 Ultrasonic Wave Detector, Distance Sensor**
![Sonic](../images/HCSR04.JPG)

1: Voltage: DC5V<br> 
2: Quiescent Current: <2mA<br> 
3: level output: the output of the high-5V<br> 
4: level: the end of 0V<br> 
5: Induction Angle: not more than 15 degrees<br> 
6: detection range: 2cm-450cm<br> 
7: High precision: up to 0.3cm

Panel wiring, VCC, trig 's (control side), the echo (receiving end), out (empty feet), GND
Note: TRIP-pin internal pull-up 10K resistor, down TRIP microcontroller IO port pin, and then to a 10us pulse signal.
OUT pin for this module as the switch output pin when the anti-theft module, ranging modules do not use this foot!
Note: The module should be inserted Fortunately, the circuit board re-energized, and avoid the high malfunction, if they have re-energized before they solve.
This module can be provided ranging program: C51, PIC18F877 microcontroller Yoshitatsu three MCU test reference.

**3 the module URF04 works:**<br> 
1. IO trigger ranging to at least 10us high signal;
2. module automatically sends eight 40kHz square wave, automatically detect whether a signal return;
3. a signal to return to a high IO output, high duration of the ultrasonic time from launch to return.
Test distance = (time high * speed of sound (340M / S)) / 2;

|Funkt. |Pin |GPIO|Color|
|-------|----|----|-----|
|Echo 1 | 25 | 21 |
|Trig 1 | 29 | 22 |
|Echo 2 | 5  | 3  |
|Trig 2 | 4  | 2  |

## ESC
**BLHeli Serie 30A electronic speed controller**
![ESC](../images/ESC.jpg)
![ESC1](../images/ILRIZ44N.jpg)

|-Funkt.-|Pin |GPIO|Color|  ESC - Motor  |
|--------|----|----|-----|---------------|
| ESC FL | 15 | 11 |  x  | 1-1 ; 2-2 ; 3-3 |
| ESC FR | 16 | 12 |  x  | 1-3 ; 2-2 ; 3-1 |
| ESC BR | 17 | 13 |  x  | 1-1 ; 2-2 ; 3-3 |
| ESC BL | 19 | 14 |  x  | 1-3 ; 2-2 ; 3-1 |

brown = GND<br>
red = +5V<br>
yellow = signal<br>

Kontinuierlicher Strom: 30a<br>
Berst strom: 30a<br>
Lixx Batterie: 2 ~ 4s<br>
Abmessungen: 52*26*7mm<br>
Bec Ausgang: 2a/5v<br>
Gewicht: 28g<br>

Basierend auf blheli Firmware, weiter optimiert für perfekte Antriebsleistung.<br>
Niederspannungsschutz, Überhitzungsschutz und Drosselklappen Signal verlustsschutz.<br>
Separate Stromversorgung für mcu und bec, Verbesserung der esc Fähigkeit, magnetische Störungen.<br>
Die Esc-Parameter können über die Programmkarte oder den Sender konfiguriert werden.<br>
Drosselbereich kann so konfiguriert werden, dass er mit verschiedenen Empfängern kompatibel ist.<br>
Ausgestattet mit integriertem linearen bec oder bec Schalter.<br>

## Temperatur
**DS18B20 temperatur sensor**
![Dallas](../images/Dallas.jpg)

|Funkt. |Pin |GPIO|Color|
|-------|----|----|-----|
|Temp.  | 9  | 6  |

## Alarm
**aktiver Summer kontinuierlicher Piepton 12*9,5mm 5V**
![Buzzer](../images/Buzzer.jpg)

|Funkt. |Pin |GPIO|
|-------|----|----|
| Alarm | 14 | 10 |