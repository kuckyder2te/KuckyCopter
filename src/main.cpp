/*  File name : main.cpp
    Project name : KuCo_Phantom 1
    Date : 2022-04-23

    Description : Drohne
    Hardware : Raspberry Pi Pico 2020
               Gyro : MPU9250
               Baro : BMP280
               Radio : NRF24
               Sonic : HRS04
*/

#include <Arduino.h>
#include <TaskManager.h>
#include <HardwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_SPIDevice.h>

#include "..\lib\sensors.h"
#include "..\lib\sonic.h"
#include "..\lib\radio.h"
#include "..\lib\battery.h"
#include "..\lib\axisMotor.h"
#include "..\lib\axisYaw.h"
#include "..\lib\flyController.h"
#include "..\lib\myLogger.h"
#include "..\lib\performance.h"
#include "..\lib\model.h"
#include "..\lib\def.h"

#define PIN_BT_TX       8
#define PIN_BT_RX       9
#define COM_SPEED     9600
#define BT_SPEED      9600

model_t model;      /// Speicherplatz wird angelegt und instanziert
UART Serial2(PIN_BT_TX, PIN_BT_RX);

void setup() {
  LOGGER_NOTICE( "Program will initialized");
    model.performance.min_loop_time = 0xffff;
    model.yawData.axisData[0] = &model.axisData[0];  // axisData wird mit yawData.axisData verknüpft
    model.yawData.axisData[1] = &model.axisData[1];

    Serial.begin(COM_SPEED);
    Serial2.begin(BT_SPEED);
    Serial2.println("********************************");
    Serial2.println("*       KuCo Phantom 1         *");
    Serial2.println("*                              *");
    Serial2.print  ("*     ");Serial2.print(__DATE__);Serial2.print(" ");Serial2.print(__TIME__);Serial2.println("     *");
    Serial2.print  ("*    EEPROM PID Address   ");/*Serial2.print(PID_EEPROM_ADRRESS);*/Serial2.println("     *");
    Serial2.println("********************************");
    Serial2.flush();
    Wire.begin();

    delay(500);

#ifdef _DEBUG_
  Logger::setOutputFunction(&localLogger);
  Logger::setLogLevel(Logger::_DEBUG_);
#endif
  LOGGER_VERBOSE("Enter....");
    Tasks.add<AxisMotor>("axismotor_a")
      ->setModel(&model.axisData[0])
      ->setMotorPinOrdered(PIN_MOTOR_FL)
      ->setMotorPinOrdered(PIN_MOTOR_BR)
      ->startFps(_AXIS_FPS);
    Tasks.add<AxisMotor>("axismotor_b")
      ->setModel(&model.axisData[1])
      ->setMotorPinOrdered(PIN_MOTOR_FR)
      ->setMotorPinOrdered(PIN_MOTOR_BL)
      ->InvertRoll()
      ->startFps(_AXIS_FPS);
    Tasks.add<AxisYaw>("axisyaw")
      ->setModel(&model.yawData)
      ->setAxisOrdered(reinterpret_cast<AxisMotor*>(Tasks["axismotor_a"].get()))
      ->setAxisOrdered(reinterpret_cast<AxisMotor*>(Tasks["axismotor_b"].get()))
      ->startFps(_AXIS_FPS);
    Tasks.add<FlyController>("flycontroller")->startFps(100);
    Tasks.add<Sensor>("sensor")->setModel(&model.sensorData)->startFps(1); // Übergabe des models in das objekt Sensor
    Tasks.add<Sonic>("sonic")->setModel(&model.sonicData)->startFps(1);
    Tasks.add<Battery>("battery")->startFps(1);    
    Tasks.add<Radio>("radio")->startFps(1);
    LOGGER_NOTICE( "Program is initialized");
  LOGGER_VERBOSE("....leave"); 
}/*-------------------- end of setup ------------------------------------------*/

void loop() {
  LOGGER_VERBOSE("loop has begun");
  
  //  unsigned long enter = micros();
    Tasks.update();
    Tasks["sensor"]->enter();
    
  //  Serial.print("/*");Serial.print(model.sensorData.yaw);Serial.print(",");  /// eigenen monitor als Klasse erzeugen
  //                     Serial.print(model.sensorData.roll);Serial.print(",");
  //                     Serial.print(model.sensorData.pitch);Serial.print(",");Serial.println("*/");
  //  Serial.print(model.performance.min_loop_time);Serial.print(",");
  //  Serial.print(model.performance.max_loop_time);Serial.print(",");
  //  Serial.print(model.performance.last_loop_time);Serial.println("*/");
  //   delay(2000);
  //  model.performance.last_loop_time = micros() - enter;
  //  if(model.performance.last_loop_time > model.performance.max_loop_time)
  //    model.performance.max_loop_time = model.performance.last_loop_time;
  //  if(model.performance.last_loop_time < model.performance.min_loop_time)
  //    model.performance.min_loop_time = model.performance.last_loop_time;
  LOGGER_VERBOSE("Loop completed successfully");
}/*-------------------- end of loop ------------------------------------------*/