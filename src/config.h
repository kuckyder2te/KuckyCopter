#pragma once

#define PIN_BT_TX 8
#define PIN_BT_RX 9
#define COM_SPEED 115200
#define BT_SPEED 9600

#define PIN_MOTOR_FL 11
#define PIN_MOTOR_FR 12
#define PIN_MOTOR_BL 14
#define PIN_MOTOR_BR 13

#define PIN_LED_STATE 4 // mainloop is running

#define AXIS_FPS 100

#define PID_OUTPUT_BITS 16
#define PID_OUTPUT_SIGNED true
#define PID_P_MIN 0.00390626 ///< The parameter P domain is [0.00390625 to 255] inclusive.
// #define PID_EEPROM_ADRRESS 50

#define PIN_RADIO_CE 20
#define PIN_RADIO_CSN 17
#define LED_RADIO 1

#define PIN_BATTERY 26
#define LED_PIN_ALERT 10

#define LED_POSITION 7

#define POWER_MIN 0
#define POWER_MAX 100
#define BASE_MOTOR_POWER 5 //< 5% minimal throttle in fly mode for preventing stop of the motors
#define PIN_ESC_ON 15
#define DUTYCYCLE_MIN 40000
#define DUTYCYCLE_MAX 80000

#define DOT_1 0.1    //  1 Nachkommastellen
#define DOT_2 0.01   //  2 Nachkommastellen
#define DOT_3 0.001  //  3 Nachkommastellen
#define DOT_4 0.0001 //  4 Nachkommastellen
