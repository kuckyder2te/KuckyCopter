#pragma once

#include "..\src\config.h"
#include "..\lib\flycontroller.h"
#include "..\lib\monitor.h"
#include "..\lib\model.h"

extern model_t model;

FlyController *flycontrol;
Monitor *monitor;
void test_setup()
{
  flycontrol = new FlyController("Flycontrol");
  monitor = new Monitor("monitor", Report_t::FLYCONTROL);
  monitor->setModel(&model);
}

void test_loop()
{
  flycontrol->update();
  monitor->update();
}