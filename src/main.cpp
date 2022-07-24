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

//#define _DEBUG_ DEBUG
#include "..\lib\sensors.h"
//#define _DEBUG_ DEBUG
#include "..\lib\sonic.h"
#define _DEBUG_ DEBUG
#include "..\lib\radio.h"
#include "..\lib\battery.h"
//#define _DEBUG_ DEBUG
#include "..\lib\axisBase.h"
//#define _DEBUG_ DEBUG
#include "..\lib\axisMotor.h"
//#define _DEBUG_ DEBUG
#include "..\lib\axisYaw.h"
//#define _DEBUG_ DEBUG
#include "..\lib\flyController.h"
#include "..\lib\performance.h"
#include "..\lib\PID_adjust.h"
#include "..\lib\model.h"


#include "..\lib\myLogger.h"

#define PIN_BT_TX       8
#define PIN_BT_RX       9
#define COM_SPEED     115200
#define BT_SPEED      115200

#define PIN_MOTOR_FL    11
#define PIN_MOTOR_FR    12
#define PIN_MOTOR_BL    13
#define PIN_MOTOR_BR    14

#define AXIS_FPS 100

model_t model;      /// Speicherplatz wird angelegt und instanziert
UART Serial2(PIN_BT_TX, PIN_BT_RX);

PID_adjust *_pid_adjust;

void setup() {
    digitalWrite(PIN_ESC_ON, LOW);      // MainPower für ESC´s abgeschaltet
    Serial.begin(COM_SPEED);
    Serial2.begin(BT_SPEED);
    Logger::setOutputFunction(&localLogger);
    delay(50);
    Logger::setLogLevel(Logger::DEBUG);           // Muss immer einen Wert in platformio.ini haben (SILENT)
    
    LOGGER_NOTICE( "Program will initialized");
    model.performance.min_loop_time = 0xffff;
    model.yawData.axisData[0] = &model.axisData[0];  // axisData wird mit yawData.axisData verknüpft
    model.yawData.axisData[1] = &model.axisData[1];

    Serial2.println("********************************");
    Serial2.println("*       KuCo Phantom 1         *");
    Serial2.println("*                              *");
    Serial2.print  ("*     ");Serial2.print(__DATE__);Serial2.print(" ");Serial2.print(__TIME__);Serial2.println("     *");
    Serial2.print  ("*    EEPROM PID Address   ");/*Serial2.print(PID_EEPROM_ADRRESS);*/Serial2.println("     *");
    Serial2.println("********************************");
    Serial2.flush();
    Wire.begin();
    
    delay(500);

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
      ->setModel(&model.yawData)
      ->setAxisOrdered(reinterpret_cast<AxisMotor*>(Tasks["axismotor_a"].get()))
      ->setAxisOrdered(reinterpret_cast<AxisMotor*>(Tasks["axismotor_b"].get()))
      ->startFps(AXIS_FPS);
    Tasks.add<FlyController>("flycontroller")
      ->init(&model)    // bekommt das komplette Model, Master of Desater!!
      ->setYawAxis(reinterpret_cast<AxisYaw*>(Tasks["axisyaw"].get()))
      ->startFps(100);
    Tasks.add<Sensor>("sensor")->setModel(&model.sensorData)->startFps(1); // Übergabe des models in das objekt Sensor
    Tasks.add<Sonic>("sonic")->setModel(&model.sonicData)->startFps(1);
    Tasks.add<Battery>("battery")->setModel(&model.batteryData)->startFps(1);    
    Tasks.add<Radio>("radio")->startFps(10);

	  Tasks.add<PID_adjust>("pidadjust")
      ->setSerial(&Serial2)
      ->setModel(&model)
      ->addPID(reinterpret_cast<AxisBase*>(Tasks["axismotor_a"].get())->getPid(),"axismotor_a")
      ->addPID(reinterpret_cast<AxisBase*>(Tasks["axismotor_b"].get())->getPid(),"axismotor_b")
      ->addPID(reinterpret_cast<AxisBase*>(Tasks["axisyaw"].get())->getPid(),"axisyaw")
      ->startFps(100);
   _pid_adjust = reinterpret_cast<PID_adjust *>(Tasks["pidadjust"].get());
   _pid_adjust->display_Menu();

    LOGGER_NOTICE( "Program is initialized");
  LOGGER_VERBOSE("....leave"); 
}/*------------------------ end of setup ----------------------------------------------*/

void loop() {
  LOGGER_NOTICE("loop has begun");
  
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
}/*------------------------ end of loop -----------------------------------------------*/

#undef _DEBUG_