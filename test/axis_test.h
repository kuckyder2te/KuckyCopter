#pragma once

#include "..\src\config.h"
#include "..\lib\axisBase.h"
#include "..\lib\axisMotor.h" //eigener testfall
#include "..\lib\newPID.h"

AxisMotor *axis[2];
AxisBase::axisData_t axisData;

void test_setup()
{

  LOGGER_VERBOSE("Enter....");
  axis[0] = new AxisMotor("axismotor_a");
  axis[1] = new AxisMotor("axismotor_b");
  axis[0]->setModel(&axisData);
  axis[1]->setModel(&axisData);

  for (uint8_t i = 0; i < 2; i++)
  {
    axis[i]->begin();
  }
}

void test_loop()
{
}
/*------------------------ end of axis test programm -------------------------------------------*/
