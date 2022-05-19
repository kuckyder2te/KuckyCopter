/*  File name : main.cpp
    Project name : KuCo_xx
    Date : 2022-04-23

    Description : Drohne
    Hardware : Raspberry Pi Pico
               Gyro : MPU9250
               Baro : MS5611 / BMP280 for test
               Radio : NRF24
               Sonic : HRS04
*/

#include <Arduino.h>
#include <TaskManager.h>
#include <HardwareSerial.h>
//#include <SPI.h>
//#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_SPIDevice.h>
#include "..\lib\gyro.h"
#include "..\lib\baro.h"
#include "..\lib\sonic.h"
#include "..\lib\radio.h"
#include "..\lib\calibration.h"
#include "..\lib\myLogger.h"
#include "..\lib\performance.h"
#include "..\lib\gui.h"
#include "..\lib\model.h"
#include "..\lib\def.h"

model_t model;      /// Speicherplatz wird angelegt und instanziert
UART Serial2(PIN_BT_TX, PIN_BT_RX);

void setup() {
    model.performance.min_loop_time = 9999999999LL;
    
    Serial.begin(COM_SPEED);
    Serial2.begin(BT_SPEED);
    Serial.println("********************************");
    Serial.println("*       KuCo Phantom 1         *");
    Serial.println("*                              *");
    Serial.print  ("*     ");Serial.print(__DATE__);Serial.print(" ");Serial.print(__TIME__);Serial.println("     *");
    Serial.print  ("*  EEPROM PID Address   ");//Serial2.print(PID_EEPROM_ADRRESS);Serial2.println("     *");
    Serial.println("********************************");
    Serial.flush();

  //  delay(5000);
    Wire.begin();

#ifdef _DEBUG_
  Logger::setOutputFunction(&localLogger);
  Logger::setLogLevel(Logger::_DEBUG_);
#endif
  LOGGER_VERBOSE("Enter....");
    
    // Tasks.add<Baro>("baro")->setModel(&model.baroData)->startFps(1);
    // Tasks.add<Baro2>("baro2")->setModel(&model.baro2Data)->startFps(1);
    Tasks.add<Gyro>("gyro")->setModel(&model.gyroData)->startFps(1);   /// Übergabe des models in das objekt gyro
    Tasks.add<Sonic>("sonic")->setModel(&model.sonicData)->startFps(1);

    Tasks.add<Radio>("radio")->startFps(1);
    Tasks.add<Calibration>("calibration")->startFps(1);  
    Tasks.add<Gui>("gui")->port(Serial2)->startFps(1); 
    LOGGER_NOTICE( "Init Program");
  LOGGER_VERBOSE("....leave"); 
//  Serial2.println("setup");
}

void loop() {
//  Serial2.println("loop");
  
  //  unsigned long enter = micros();
    Tasks.update();
    Tasks["gyro"]->enter();
    
    // Serial.print("/*");Serial.print(model.gyroData.yaw);Serial.print(",");  /// eigenen monitor als Klasse erzeugen
    // Serial.print(model.gyroData.roll);Serial.print(",");
    // Serial.print(model.gyroData.pitch);Serial.print(",");
    // Serial.print(model.performance.min_loop_time);Serial.print(",");
    // Serial.print(model.performance.max_loop_time);Serial.print(",");
    // Serial.print(model.performance.last_loop_time);Serial.println("*/");
//   delay(2000);
    // model.performance.last_loop_time = micros() - enter;
    // if(model.performance.last_loop_time > model.performance.max_loop_time)
    //   model.performance.max_loop_time = model.performance.last_loop_time;
    // if(model.performance.last_loop_time < model.performance.min_loop_time)
    //   model.performance.min_loop_time = model.performance.last_loop_time;
}