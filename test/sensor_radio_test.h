#pragma once

#include "..\src\config.h"
#include "..\lib\sonics.h"
#include "..\lib\monitor.h"
#include "..\lib\model.h"
#include "..\lib\battery.h"

extern model_t model;

Radio *radio;
Sensor *sensor;
Monitor *monitor;
Battery *battery;

void update_TX_payload()
{
  model.RC_interface.TX_payload.yaw = model.sensorData.yaw;
  model.RC_interface.TX_payload.pitch = model.sensorData.pitch;
  model.RC_interface.TX_payload.roll = model.sensorData.roll;
  model.RC_interface.TX_payload.altitude = model.sensorData.altitude;
  model.RC_interface.TX_payload.distance_down = model.sonicData.down_distance;
  model.RC_interface.TX_payload.distance_front = model.sonicData.front_distance;
  model.RC_interface.TX_payload.pressure = model.sensorData.pressure;
  model.RC_interface.TX_payload.temperature = model.sensorData.temperature_baro;
  model.RC_interface.TX_payload.battery = model.batteryData.battery_State;
}

void test_setup()
{
  LOGGER_VERBOSE("Enter....");
  radio = new Radio("radio");
  radio->setModel(&model.RC_interface)->begin();

  sensor = new Sensor("sensor");
  sensor->setModel(&model.sensorData)->begin();

  battery = new Battery("battery");
  battery->setModel(&model.batteryData)->begin();

  monitor = new Monitor("monitor", Report_t::RADIO_SENSOR);
  monitor->setModel(&model)->begin();
}
void test_loop()
{
  radio->update();
  sensor->enter();
  battery->update();
  monitor->update();
  update_TX_payload();
  delay(10);
}
/*------------------------ end of radio/sensor test progamm -------------------------------------*/