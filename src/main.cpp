/*  File name : main.cpp
    Project name : KuCo_Phantom 1
    Date : 2022-04-23

    Description : Drohne
    Hardware : Raspberry Pi Pico
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
#include "..\lib\pidController.h"
#include "..\lib\calibration.h"

#include "..\lib\myLogger.h"
#include "..\lib\performance.h"
#include "..\lib\model.h"
#include "..\lib\def.h"

model_t model;      /// Speicherplatz wird angelegt und instanziert
UART Serial2(PIN_BT_TX, PIN_BT_RX);

PidController pid_pri(model.pidData[axis_t::Primary]);  	    ///< Enumerations show the context better.
PidController pid_sec(model.pidData[axis_t::Secondary]);
PidController pid_yaw(model.pidData[axis_t::YawAxis]);

void setup() {
  LOGGER_NOTICE( "Program will initialized");
    model.performance.min_loop_time = 9999999999LL;
    
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
    Tasks.add<Sensor>("sensor")->setModel(&model.sensorData)->startFps(1); // Übergabe des models in das objekt Sensor
    Tasks.add<Sonic>("sonic")->setModel(&model.sonicData)->startFps(1);
    //Tasks.add<PidController>("pidController")->setModel(&model.pidData[3])->startFps(1);
    Tasks.add<Calibration>("calbration")->startFps(100);
    Tasks.add<Battery>("Battery")->startFps(1);    
    Tasks.add<Radio>("radio")->startFps(1);
    LOGGER_NOTICE( "Program is initialized");
  LOGGER_VERBOSE("....leave"); 
}

void loop() {
  LOGGER_VERBOSE("loop has begun");
  Serial.println("loop has begun Serial");
  
  //  unsigned long enter = micros();
    Tasks.update();
    Tasks["sensor"]->enter();
    
  //  Serial.print("/*");Serial.print(model.sensorData.yaw);Serial.print(",");  /// eigenen monitor als Klasse erzeugen
  //                     Serial.print(model.sensorData.roll);Serial.print(",");
  //                     Serial.print(model.sensorData.pitch);Serial.print(",");Serial.println("*/");
    // Serial.print(model.performance.min_loop_time);Serial.print(",");
    // Serial.print(model.performance.max_loop_time);Serial.print(",");
    // Serial.print(model.performance.last_loop_time);Serial.println("*/");
//   delay(2000);
    // model.performance.last_loop_time = micros() - enter;
    // if(model.performance.last_loop_time > model.performance.max_loop_time)
    //   model.performance.max_loop_time = model.performance.last_loop_time;
    // if(model.performance.last_loop_time < model.performance.min_loop_time)
    //   model.performance.min_loop_time = model.performance.last_loop_time;
  LOGGER_VERBOSE("Loop completed successfully");
}