# Hardware
## Controller
Raspberry Pi Pico Entwicklung Bord TYPE-C High-Leistung Mikrocontroller-board RP2040<br>  128Mbit 4M 16MB Dual-core ARM Prozessor

![RP2040](../images/RP2040.jpg)

Dual-core Arm Cortex-M0+ processor, flexible clock running up to 133 MHz<br>
264kB on-chip SRAM<br>
2 × UART<br>
2 × SPI controllers<br>
2 × I2C controllers<br>
16 × PWM channels<br>
1 × USB 1.1 controller and PHY, with host and device support<br>
8 × Programmable I/O (PIO) state machines for custom peripheral support<br>
Operating temperature -40°C to +85°C<br>
Drag-and-drop programming using mass storage over USB<br>
Low-power sleep and dormant modes<br>
Temperature sensor<br>
Accelerated integer and floating-point libraries on-chip<br>

## PinOut
![RP2040DevBoardPins](../images/picoPins.jpg)  

## Circut Board
![Top](../images/board_top.JPG)
![Bottom](../images/board_bottom.JPG)

## IMU
MPU9250/MS5611 CJMCU-117 High Precision 9 Axis 10DOF Attitude Module SPI/IIC Communication

![IMU](../images/IMU.JPG)
![IMU2](../images/IMU_2.JPG)

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
Use chip: MS5611-01BA03<br>

|Funkt.|Pin |GPIO|
|------|----|----|
|  SDA | 6  | 4  |
|  SCL | 7  | 5  |

## Radio
**NRF24**
![NRF24](../images/NRF24L01_E01-ML01DP5.jpg)

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
Abstand: 2100m<br>
HF-Verbindungsstück: SMA-K<br>
Frequenz: 2,4 GHz(2400MHz-2525MHz)<br>
Supply voltage 2,0 to 3,6 VDC<br>
Communikation level 0,7VCC to 5V(VCC refers to the supply voltage)<br>
Operation range <br>
Transmitting power 20dBm (100mW)<br>
Air data rate can be configurate to 250k, 1M, 2Mbps<br>
Sleep current 1.0uA (nRF24L01P set as power.down)<br>
Transmitting current 130mA@ 20dBm<br>
Receiving current 20mA (CE=1)<br>
Transmitting lenght 32 bytes maximum for one package, 3 level FIFO<br>
Receiving lenght  32 bytes maximum for one package, 3 level FIFO<br>
Antenna type SMA-K (External thread hole)<br>
Operating temperature -40 ~ +85°<br>
Operating Humidity 10% ~ 90%<br>
Storage temperature -40 ~ +125°<br>

Feature: E01-ML01DP5 is based on original imported nrf24l01p form nordic in Norway. And equipped with 20dbm power amplifier chip imported from USA, which makes the transmitting power reach 100mW (20dbm) while increasing the receiving sensitivity by 10db. These features make the transmission distance 10 times longer than nrf24l01p itself. The anti-interference shielding cover on the module makes the anti-interference performance better.

![NRF24_2](../images/NRF24L01_E01-ML01DP5-dimension.jpg)

## Sonic
**HC-SR04 Ultrasonic Wave Detector, Distance Sensor**
![Sonic](../images/HCSR04.JPG)

1: Voltage: DC5V<br> 
2: Quiescent Current: <2mA<br> 
3: level output: the output of the high-5V<br> 
4: level: the end of 0V<br> 
5: Induction Angle: not more than 15 degrees<br> 
6: detection range: 2cm-450cm<br> 
7: High precision: up to 0.3cm<br>

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

Continuous stream: 30a<br>
Berst strom: 30a<br>
Lixx Batterie: 2 ~ 4s<br>
Dimensions: 52*26*7mm<br>
Bec Output: 2a/5v<br>
Weight: 28g<br>

Based on blheli firmware, further optimized for perfect drive performance.<br>
Low voltage protection, overheating protection and throttle signal loss protection.<br>
Separate power supply for mcu and bec, improve esc ability, magnetic interference.<br>
The Esc parameters can be configured via the program card or the transmitter.<br>
Throttle range can be configured to be compatible with different receivers.<br>
Equipped with integrated linear bec or bec switch.<br>

## Temperatur
**DS18B20 temperatur sensor**
![Dallas](../images/Dallas.jpg)

|Funkt. |Pin |GPIO|Color|
|-------|----|----|-----|
|Temp.  | 9  | 6  |

Working temperature -55 °c ~ 125 °c<br>
Power suply range 3,6 to 5,5V<br>

## Alarm
**aktiver Summer kontinuierlicher Piepton 12*9,5mm 5V**
![Buzzer](../images/Buzzer.jpg)

|Funkt. |Pin |GPIO|
|-------|----|----|
| Alarm | 14 | 10 |