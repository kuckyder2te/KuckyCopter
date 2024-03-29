#pragma once
/*  File name: config.h
    Project name: KuckyCopter
    Date: 2022-05-31
    Author: Stephan Scholz / Wilhelm Kuckelsberg
    Description: Global defines
*/

/* Environment */
#define COM_SPEED 115200    //COM21
#define BT_SPEED  115200    //COM14
#define BT2_SPEED 115200    //COM4

#define PIN_LED_STATE 1  // flycontroller  is running <orange>
#define PIN_BATTERY   26   // Analog PIN for battery measurement
#define LED_PIN_ALERT 10 // PIN for acoustic and visual battery alert <red>
#define LED_POSITION  7   // PIN for the green and red position LED´s

#define DOT_1 0.1    //  1 decimal places for PID preferences
#define DOT_2 0.01   //  2       ""
#define DOT_3 0.001  //  3       ""
#define DOT_4 0.0001 //  4       ""

/* Menu position */
#define ROW_MENU 3 ///< First position for the main menue
#define COL_MENU 10
#define ROW_SELECT 24
#define COL_PID_VALUE 13
#define COL_CUR_PID_VAL 60      // Pos. freom the PID values from the EEPROM
#define ROW_STATE ROW_MENU + 46 // Position for state message
#define COL_STATE COL_MENU + 16

/* Motos and axis */
typedef enum
{
    primary = 0,
    secondary,
    yaw
} axisName;

#define PIN_MOTOR_FL 11 // ESC PIN´s
#define PIN_MOTOR_FR 12
#define PIN_MOTOR_BL 14
#define PIN_MOTOR_BR 13
#define POWER_MIN 10
#define POWER_MAX 100
#define BASE_MOTOR_POWER 10 //< 5% minimal throttle in fly mode for preventing stop of the motors
#define PIN_ESC_ON 15
#define DUTYCYCLE_MIN 40000 // ~ 1000 micro seconds
#define DUTYCYCLE_MAX 80000 // ~ 2000      ""

/* Sonics*/
#define PIN_ECHO_DOWN 21 // PIN for sonic down sensor
#define PIN_TRIGGER_DOWN 22
#define PIN_ECHO_FRONT 2 // PIN for sonic front sensor
#define PIN_TRIGGER_FRONT 3
#define NUMBER_OF_SLAVES 1 // Number of possible slave sonic sensors
#define MAX_DISTANCE 200   // max distance range 2 to 400cm
#define PIN_18B20 6        // PIN for the Dallas temperature sensor

typedef enum
{
    down = 0,
    front
} sonicName;

/* PID */
typedef enum
{
    kP = 0,
    kI,
    kD,
    eF
} pidCoefficient;

#define AXIS_FPS 100 // Frame rate taskmanager Axis

#define PID_OUTPUT_BITS 16
#define PID_OUTPUT_SIGNED true
#define PID_P_MIN 0.00390626 ///< The parameter P domain is [0.00390625 to 255] inclusive.

/* Radio*/
#define PIN_RADIO_CE 20
#define PIN_RADIO_CSN 17
#define LED_RADIO 0 // blue
