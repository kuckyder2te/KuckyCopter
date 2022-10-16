/*  File name: model.h
    Project name: KuCo_Phantom 1
    Date: 2022-05-31
    Author: Wilhelm Kuckelsberg
    Description: Global constance
*/
#pragma once

#include "sensors.h"
#include "sonic.h"
#include "radio.h"
#include "performance.h"
#include "axisYaw.h"
#include "axisMotor.h"
#include "battery.h"
#include "newPID.h"

typedef enum
{
    arming_begin = 0, ///< When the Kuckycopter is first turned on, the arming starts.
    arming_busy,      /// ist das nötig?
    disablePID,
    standby,  ///< All motors on POWER_MIN
    prestart, ///< All motors on standby and ready to fly. (POWER_MIN)
    takeoff,  ///< The Quadrocopter takes off.
    set_pid,  ///< Fly without PID-Output = 0
    fly,      ///< Normal fly mode
    ground    ///< Kuckycopter stand on the ground
} flyState_e;

typedef enum
{
    primary = 0,
    secondary,
    yaw
} axisName_t; // bezeichnet die tatsächlichen Namen der 3 Achsen

typedef struct
{
    sensorData_t sensorData; // Data from imu and baro
    sonicData_t sonicData;
    performance_t performance;
    RC_interface_t RC_interface;
    AxisBase::axisData_t axisData[2];
    AxisBase::axisData_t yawData;
    yaw_t yaw;
    batteryData_t batteryData;
    pidData_t pidData[3];
    flyState_e flyState;
    pidData_TEST_t pidData_TEST[3];
} model_t;

//#undef _DEBUG_