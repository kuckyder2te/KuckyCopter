#pragma once

#include "..\src\config.h"
#include "..\lib\radio.h"
#include "..\lib\monitor.h"
#include "..\lib\model.h"

extern model_t model;

Radio *radio;
Monitor *monitor;

void send_data_to_drohne()
{
  model.RC_interface.TX_payload.yaw = model.RC_interface.RX_payload.rcYaw;
  model.RC_interface.TX_payload.pitch = model.RC_interface.RX_payload.rcPitch;
  model.RC_interface.TX_payload.roll = model.RC_interface.RX_payload.rcRoll;
  model.RC_interface.TX_payload.altitude = model.RC_interface.RX_payload.rcThrottle;
  model.RC_interface.TX_payload.distance_down = (uint16_t)model.RC_interface.RX_payload.rcSwi1;
  model.RC_interface.TX_payload.distance_front = (uint16_t)model.RC_interface.RX_payload.rcSwi2;
  model.RC_interface.TX_payload.pressure = (float)model.RC_interface.RX_payload.rcSwi3;
  model.RC_interface.TX_payload.temperature = model.RC_interface.RX_payload.rcAltitudeBaroAdj;
  model.RC_interface.TX_payload.battery = model.RC_interface.RX_payload.rcAltitudeSonicAdj;
} /*------------------------ end of send_data_to_drohne -----------------------------------------*/

void test_setup()
{
  LOGGER_VERBOSE("Enter....");
  radio = new Radio("radio");
  radio->setModel(&model.RC_interface)->begin();
  monitor = new Monitor("monitor", Report_t::RADIO);
  monitor->setModel(&model)->begin();
}
void test_loop()
{
  radio->update();
  monitor->update();
  send_data_to_drohne();
  delay(100);
}
/*------------------------ end of radio test progamm -------------------------------------------*/
