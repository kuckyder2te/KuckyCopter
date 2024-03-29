#pragma once
/* File name : sonic_test.h
	 Project name : KuckyCopter 2
	 Author: Stephan Scholz /  Wilhelm Kuckelsberg
	 Date : 2022-06-17
	 Description : Drohne
*/

#include "..\src\def.h"
#include "..\lib\sonics.h"
#include "..\lib\monitor.h"
#include "..\lib\model.h"
#include "..\lib\pico-onewire\one_wire.h"

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
  sonic->setModel(&model.sonicData)->begin();
  monitor = new Monitor("monitor", Report_t::SONIC);
  monitor->setModel(&model)->begin();
}

void test_loop()
{
  sonic->update();
  monitor->update();
  performance_test(60);
}
/*------------------------ end of sonic test programm -------------------------------------------*/