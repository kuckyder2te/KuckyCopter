#pragma once
/* File name : battery_test.h
	 Project name : KuckyCopter 2
	 Author: Stephan Scholz /  Wilhelm Kuckelsberg
	 Date : 2022-06-17
	 Description : Drohne
*/

#include "..\src\def.h"
#include "..\lib\battery.h"
#include "..\lib\monitor.h"
#include "..\lib\model.h"

extern model_t model;

Battery *battery;
Monitor *monitor;

void test_setup()
{
  LOGGER_VERBOSE("Enter....");
  battery = new Battery("barrery");
  monitor = new Monitor("monitor", Report_t::BATTERY);
  monitor->setModel(&model);
  battery->setModel(&model.batteryData);
  battery->begin();
  LOGGER_VERBOSE("leave....");
}

void test_loop()
{
  LOGGER_VERBOSE("Enter....");
  monitor->update();
  battery->update();
  LOGGER_VERBOSE("leave....");
}
/*------------------------ end of sonic test programm -------------------------------------------*/
