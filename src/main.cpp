/*  File name : main.cpp
    Project name : KuckyCopter v2
    Date : 2022-04-23

    Description : Drohne
    Hardware : Raspberry Pi Pico 2020
               Gyro : MPU9250
               Baro : MS5911
               Radio : NRF24
               Sonic : HRS04
               COM : BT HC6 v1.05
*/

#include <Arduino.h>
#include <TaskManager.h>
#include <HardwareSerial.h>
// #include <Adafruit_Sensor.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_SPIDevice.h>
// #include <SPI.h>
#include "EEPROM.h"
#include "def.h"

#include "..\lib\sensors.h"
#include "..\lib\sonics.h"
#include "..\lib\temperature.h"
#include "..\lib\radio.h"
#include "..\lib\battery.h"
#include "..\lib\axisBase.h"
#include "..\lib\axisMotor.h"
#include "..\lib\axisYaw.h"
#include "..\lib\flyController.h"
#include "..\lib\performance.h"
#include "..\lib\configurator.h"
#include "..\lib\pos_LED.h"

#define LOCAL_DEBUG

#include "..\lib\myLogger.h"
#ifdef _MAIN
#define DISPLAY_DELAY 100
#endif
#include "..\lib\monitor.h"

model_t model;
int16_t Test;
HardwareSerial *MonitorOutput = &Serial; /// USB Pico
HardwareSerial *TestOutput = &Serial2;   /// Bluetooth Putty for PID adjust
HardwareSerial *DebugOutput = &Serial1;  /// Bluetooth CoolTerm for debuging

#ifdef _PID_ADJUST
PID_adjust *_pid_adjust;
#endif

void wireModel();
void base_setup();

#ifdef _MAIN
void main_setup()
{
  LOGGER_VERBOSE("Enter....");
  Tasks.add<AxisMotor>("axismotor_a")
      ->setModel(&model.axisData[axisName::primary])
      ->initMotorOrdered(PIN_MOTOR_FL)
      ->initMotorOrdered(PIN_MOTOR_BR)
      ->startFps(AXIS_FPS);
  Tasks.add<AxisMotor>("axismotor_b")
      ->setModel(&model.axisData[axisName::secondary])
      ->initMotorOrdered(PIN_MOTOR_FR)
      ->initMotorOrdered(PIN_MOTOR_BL)
      ->InvertRoll()
      ->startFps(AXIS_FPS);
  Tasks.add<AxisYaw>("axisyaw")
      ->setModel(&model.yawData, &model.yaw)
      ->setAxisOrdered(reinterpret_cast<AxisMotor *>(Tasks["axismotor_a"].get()))
      ->setAxisOrdered(reinterpret_cast<AxisMotor *>(Tasks["axismotor_b"].get()))
      ->startFps(AXIS_FPS);
  Tasks.add<FlyController>("flycontroller")
      ->init(&model) // He gets the complete model.
      ->setYawAxis(reinterpret_cast<AxisYaw *>(Tasks["axisyaw"].get()))
      ->startFps(10);
  Tasks.add<Sensor>("sensor")->setModel(&model.sensorData)->startFps(100);
  Tasks.add<Sonic>("sonic")->setModel(&model.sonicData)->startFps(10);
  //  Tasks.add<Battery>("battery")->setModel(&model.batteryData)->startFps(0.1);
  Tasks.add<Temperature>("temperature")->setModel(&model.temperatureData)->startFps(0.01); // One measurement every 100 seconds
  Tasks.add<Radio>("radio")->setModel(&model.RC_interface)->startFps(10);
  Tasks.add<POS_LED>("postion-led")->startFps(1);

#ifdef _SERIAL_STUDIO
  Tasks.add<Monitor>("Monitor")->setModel(&model)->startFps(10);
#endif

#ifdef _PID_ADJUST
  Tasks.add<PID_adjust>("pidadjust")
      ->setSerial(&Serial2)
      ->setModel(&model)
      ->addPID(reinterpret_cast<AxisBase *>(Tasks["axismotor_a"].get())->getPid(), "axismotor_a")
      ->addPID(reinterpret_cast<AxisBase *>(Tasks["axismotor_b"].get())->getPid(), "axismotor_b")
      ->addPID(reinterpret_cast<AxisBase *>(Tasks["axisyaw"].get())->getPid(), "axisyaw")
      ->startFps(100);
  _pid_adjust = reinterpret_cast<PID_adjust *>(Tasks["pidadjust"].get());
  _pid_adjust->display_Menu();
#endif

  LOGGER_NOTICE("Program is initialized");
  delay(100);
}
//-------------------------------------------------------------------------------------------------
void main_loop()
{
  LOGGER_VERBOSE("loop has begun");
  digitalWrite(LED_BUILTIN, HIGH);
  Tasks.update();
  LOGGER_VERBOSE("Loop completed successfully");
  digitalWrite(LED_BUILTIN, LOW);
}
//-------------------------------------------------------------------------------------------------
#elif _MOTOR
#include "..\test\motor_test.h"
#elif _AXIS_PRI
#include "..\test\axis_pri_test.h"
#elif _AXIS_SEC
#include "..\test\axis_sec_test.h"
#elif _ALL_AXIS
#include "..\test\all_axis_test.h"
#elif _YAW
#include "..\test\yaw_test.h"
#elif _SONIC
#include "..\test\sonic_test.h"
#elif _BATTERY
#include "..\test\battery_test.h"
#elif _SENSOR
#include "..\test\sensor_test.h"
#elif _RADIO
#include "..\test\radio_test.h"
#elif _RADIO_SENSOR
#include "..\test\sensor_radio_test.h"
#elif _PID
#include "..\test\pid_test.h"
#elif _FLYCONTROL
#include "..\test\flycontroller_test.h"
#endif
/*--------------------------- end of declarations -----------------------------------------------*/

void base_setup()
{
  delay(1000);
  model.emergencyStop = false;
  pinMode(PIN_ESC_ON, OUTPUT);
  digitalWrite(PIN_ESC_ON, HIGH); // MainPower für ESC´s ausgeschaltet,
                                  // will sagen, BC547 schaltet nicht durch, da die Basis HIGH ist

#ifndef _SERIAL1
  pinMode(PIN_LED_STATE, OUTPUT); // temp_debug Serial1
  digitalWrite(PIN_LED_STATE, LOW);
#endif

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  pinMode(LED_POSITION, OUTPUT);
  digitalWrite(LED_POSITION, HIGH);

  DebugOutput->begin(COM_SPEED);
  MonitorOutput->begin(COM_SPEED);
  TestOutput->begin(COM_SPEED);
  DebugOutput->println("Serial-1 COM10 OK");
  MonitorOutput->println("Serial-1 COM21 OK");
  TestOutput->println("BT COM4 OK ");

  TestOutput->print(__DATE__);
  TestOutput->print(" ");
  TestOutput->println(__TIME__);

  DebugOutput->print(__DATE__);
  DebugOutput->print(" ");
  DebugOutput->println(__TIME__);

  MonitorOutput->print(__DATE__);
  MonitorOutput->print(" ");
  MonitorOutput->println(__TIME__);

#ifdef _DEBUG_
  Logger::setOutputFunction(&localLogger);
  delay(50);
  Logger::setLogLevel(Logger::_DEBUG_); // Muss immer einen Wert in platformio.ini haben (SILENT)
#endif

  LOGGER_NOTICE("Program will initialized");
  wireModel();

  // TestOutput->println("********************************");
  // TestOutput->println("*       Kucky Copter 2         *");
  // TestOutput->println("*                              *");
  // TestOutput->print("*     ");
  // TestOutput->print(__DATE__);
  // TestOutput->print(" ");
  // TestOutput->print(__TIME__);
  // TestOutput->println("     *");
  // TestOutput->println("********************************");
  // TestOutput->flush();
  Wire.begin();

  EEPROM.begin(512);

  delay(2000);
  digitalWrite(LED_BUILTIN, LOW);
} /*------------------------ end of base setup --------------------------------------------------*/

void wireModel()
{
  model.axisData[axisName::primary].feedback = &model.sensorData.roll; // must be before setModel because of feedback Pointer
  model.axisData[axisName::secondary].feedback = &model.sensorData.pitch;
  model.yawData.feedback = &model.sensorData.yaw;


  model.axisData[axisName::primary].rcX = &model.RC_interface.RX_payload.rcRoll;
  model.axisData[axisName::primary].rcY = &model.RC_interface.RX_payload.rcPitch;
  model.axisData[axisName::secondary].rcX = &model.RC_interface.RX_payload.rcRoll;
  model.axisData[axisName::secondary].rcY = &model.RC_interface.RX_payload.rcPitch;
  model.yaw.rotationSpeed = &model.RC_interface.RX_payload.rcYaw;
  model.yaw.horz_Position = &model.sensorData.yaw;
  model.yaw.virtual_yaw = &model.sensorData.virtual_yaw;
}

void setup()
{
  base_setup();
#ifdef _MAIN
  main_setup();
#else
  test_setup();
#endif
}

void loop()
{
  unsigned long _lastLooptime = micros();
#ifdef _MAIN
  digitalWrite(LED_BUILTIN, LOW); // only for temp_debug
  main_loop();
  digitalWrite(LED_BUILTIN, HIGH);
#else
  test_loop();
#endif
  model.looptime = micros() - _lastLooptime;
  if (model.looptime > model.max_looptime)
  {
    model.max_looptime = model.looptime;
  }
  // Serial.println(model.max_looptime);
  // Serial.println(model.looptime);
} /*------------------------ end of standard setup and loop -------------------------------------*/