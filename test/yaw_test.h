#pragma once

#include "..\src\config.h"
#include "..\lib\axisBase.h"
#include "..\lib\axisMotor.h" 
#include "..\lib\axisYaw.h"
#include "..\lib\newPID.h"
#include "..\lib\monitor.h"
#include "..\lib\model.h"

extern model_t model;

AxisMotor *axis[2];
AxisYaw  *axisyaw;
AxisBase::axisData_t axisData;

void test_setup(){

}

void test_loop(){

}

