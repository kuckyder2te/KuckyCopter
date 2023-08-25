#pragma once

#include "..\src\config.h"
#include "..\lib\sensors.h"#
#include "..\lib\monitor.h"
#include "..\lib\model.h"

extern model_t model;

Sensor *sensor;
Monitor *monitor;
void test_setup()
{
  LOGGER_VERBOSE("Enter....");
  sensor = new Sensor("sensor");
  monitor = new Monitor("monitor", Report_t::SENSOR);
  monitor->setModel(&model);
  sensor->setModel(&model.sensorData)->begin();
}

void test_loop()
{
  monitor->update();
  sensor->enter();
}
/*------------------------ end of sensor test programm -------------------------------------------*/
