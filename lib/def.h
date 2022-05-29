/*  File name: def.h
    Project name: KuCo_xxx
    Date : 2022.04.21
    Author: Wilhelm Kuckelsberg

    Description:
    This file contains general definitions
*/    

#define CALIBRATION_LED 16  // Green
#define GYRO_LED 17 //blue
#define SONIC_LED 18  // Yellow
#define TASK_LED 19  // Red

//------ Environment begin -------
#define COM_SPEED 115200
#define BT_SPEED    9600
#define PUTTY_ROW			 45
#define PUTTY_COL  			124
//------ Environment end ---------

//------ PIN declarations --------

#define PIN_BT_TX        8
#define PIN_BT_RX        9

#define PIN_ECHO        21
#define PIN_TRIGGER     22

#define PIN_RADIO_CE    20
#define PIN_RADIO_CSN   17
#define PIN_RADIO_LED    0

#define PIN_BATTERY     26  // analog

#define PIN_ESC_ON      14
#define PIN_MOTOR_FL    10
#define PIN_MOTOR_FR    11
#define PIN_MOTOR_BL    12
#define PIN_MOTOR_BR    13

#define PIN_LED_RC       6
#define PIN_LED_STATE    7
#define PIN_LED_ALERT    8

#define PIN_DALLAS      28