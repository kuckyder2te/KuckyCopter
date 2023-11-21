/*  File name: model.h
    Project name: KuckyCopter 2
    Date: 2022-05-31
    Author: Stephan Scholz / Wilhelm Kuckelsberg
    Description: Global constance
*/
#pragma once

#include "sensors.h"
#include "sonics.h"
#include "radio.h"
#include "performance.h"
#include "axisYaw.h"
#include "axisMotor.h"
#include "battery.h"
#include "newPID.h" 

typedef struct
{
    sensorData_t sensorData; // Data from imu and baro
    sonicData_t sonicData;
    RC_interface_t RC_interface;
    AxisBase::axisData_t axisData[2];
    AxisBase::axisData_t yawData;
    yaw_t yaw;
    batteryData_t batteryData;
    uint32_t looptime;
} model_t;