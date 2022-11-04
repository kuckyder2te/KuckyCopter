/*  File name: model.h
    Project name: KuCo_Phantom 1
    Date: 2022-05-31
    Author: Wilhelm Kuckelsberg
    Description: Global constance
*/
#pragma once

//#include "sensors.h"
//#include "sonic.h"
//#include "radio.h"
//#include "performance.h"
//#include "axisYaw.h"
//#include "axisMotor.h"
//#include "battery.h"
//#include "newPID.h"  ???

typedef struct
{
    sensorData_t sensorData; // Data from imu and baro
    sonicData_t sonicData;
//    performance_t performance;
    RC_interface_t RC_interface;
    AxisBase::axisData_t axisData[2];
    AxisBase::axisData_t yawData;
    yaw_t yaw;
    batteryData_t batteryData;
//    pidData_t pidData[3];
} model_t;

//#undef _DEBUG_