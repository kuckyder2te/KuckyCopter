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
//#include <Adafruit_Sensor.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_SPIDevice.h>
//#include <SPI.h>
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
#include "..\lib\PID_adjust.h"

#define LOCAL_DEBUG

#include "..\lib\myLogger.h"
#include "..\lib\monitor.h"

model_t model;

HardwareSerial *TestOutput = &Serial2;
HardwareSerial *DebugOutput = &Serial;

#ifdef _PID_ADJUST
  PID_adjust *_pid_adjust;
#endif

void base_setup();

#ifdef _MAIN
void main_setup()
{
  LOGGER_VERBOSE("Enter....");
  Tasks.add<AxisMotor>("axismotor_a")
      ->setModel(&model.axisData[0])
      ->initMotorOrdered(PIN_MOTOR_FL)
      ->initMotorOrdered(PIN_MOTOR_BR)
      ->startFps(AXIS_FPS);
  Tasks.add<AxisMotor>("axismotor_b")
      ->setModel(&model.axisData[1])
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
  Tasks.add<Sensor>("sensor")->setModel(&model.sensorData)->startFps(10);
  Tasks.add<Sonic>("sonic")->setModel(&model.sonicData)->startFps(10);
//  Tasks.add<Battery>("battery")->setModel(&model.batteryData)->startFps(0.1);
  Tasks.add<Temperature>("temperature")->setModel(&model.temperatureData)->startFps(0.01); //One measurement every 100 seconds
  Tasks.add<Radio>("radio")->setModel(&model.RC_interface)->startFps(10);

#ifdef SERIAL_STUDIO
  Tasks.add<Monitor>("Monitor")->setModel(&model)->startFps(0.1);
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
  Tasks["sensor"]->enter();
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
  pinMode(PIN_ESC_ON, OUTPUT);
  digitalWrite(PIN_ESC_ON, HIGH); // MainPower für ESC´s ausgeschaltet,
                                  // will sagen, BC547 schaltet nicht durch, da die Basis HIGH ist
  pinMode(PIN_LED_STATE, OUTPUT);
  digitalWrite(PIN_LED_STATE, LOW);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  pinMode(LED_POSITION, OUTPUT);
  digitalWrite(LED_POSITION, HIGH);

  DebugOutput->begin(COM_SPEED);
  TestOutput->begin(BT_SPEED);
  DebugOutput->println("Serial COM OK");
  TestOutput->println("BT COM OK ");
  TestOutput->print(__DATE__);
  TestOutput->print(" ");
  TestOutput->println(__TIME__);

#ifdef _DEBUG_
  Logger::setOutputFunction(&localLogger);
  delay(50);
  Logger::setLogLevel(Logger::_DEBUG_); // Muss immer einen Wert in platformio.ini haben (SILENT)
#endif

  LOGGER_NOTICE("Program will initialized");
  model.yaw.axisData[axisName::primary] = &model.axisData[axisName::primary]; // axisData wird mit yawData.axisData verknüpft
  model.yaw.axisData[axisName::secondary] = &model.axisData[axisName::secondary];

  TestOutput->println("********************************");
  TestOutput->println("*       Kucky Copter 2         *");
  TestOutput->println("*                              *");
  TestOutput->print("*     ");
  TestOutput->print(__DATE__);
  TestOutput->print(" ");
  TestOutput->print(__TIME__);
  TestOutput->println("     *");
  TestOutput->println("********************************");
  TestOutput->flush();
  Wire.begin();

  EEPROM.begin(512);

  delay(2000);
  digitalWrite(LED_BUILTIN, LOW);
} /*------------------------ end of base setup --------------------------------------------------*/

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
  digitalWrite(LED_BUILTIN, LOW);  // only for debug
  main_loop();
  digitalWrite(LED_BUILTIN, HIGH);
#else
  test_loop();
#endif
  model.looptime = micros()-_lastLooptime;
} /*------------------------ end of standard setup and loop -------------------------------------*/