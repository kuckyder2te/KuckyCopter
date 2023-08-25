#pragma once

#include "..\src\config.h"
#include "..\lib\sonics.h"
#include "..\lib\monitor.h"
#include "..\lib\model.h"

extern model_t model;

Sonic *sonic;
Monitor *monitor;

void performance_test(uint16_t threshold)
{
  static unsigned long last_runtime = micros();
  uint16_t delta = micros() - last_runtime;
  if (delta > threshold)
    Serial.println(delta);
  last_runtime = micros();
}

void test_setup()
{
  LOGGER_VERBOSE("Enter....");
  sonic = new Sonic("sonic");
  monitor = new Monitor("monitor", Report_t::SONIC);
  monitor->setModel(&model);
  sonic->setModel(&model.sonicData);
  sonic->begin();
}

void test_loop()
{
  monitor->update();
  sonic->update();
  performance_test(60);
}
/*------------------------ end of sonic test programm -------------------------------------------*/