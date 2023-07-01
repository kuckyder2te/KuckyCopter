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
// #include <SPI.h>
// #include <Wire.h>
 #include <Adafruit_Sensor.h>
 #include <Adafruit_I2CDevice.h>
 #include <Adafruit_SPIDevice.h>
#include "EEPROM.h"

#define SERIAL_STUDIO

#include "..\lib\sensors.h"
#include "..\lib\sonic.h"
#include "..\lib\radio.h"
#include "..\lib\battery.h"
#include "..\lib\axisBase.h"
#include "..\lib\axisMotor.h"
#include "..\lib\axisYaw.h"
#include "..\lib\flyController.h"
#include "..\lib\performance.h"
#include "..\lib\PID_adjust.h"
//#include "..\lib\model.h"
#include "..\lib\myLogger.h"
#include "..\lib\monitor.h"

#define PIN_BT_TX 8
#define PIN_BT_RX 9
#define COM_SPEED 115200
#define BT_SPEED 115200

#define PIN_MOTOR_FL 11
#define PIN_MOTOR_FR 12
#define PIN_MOTOR_BL 13
#define PIN_MOTOR_BR 14

#define PIN_LED_STATE 0 // mainloop is running

#define AXIS_FPS 100

model_t model; /// Speicherplatz wird angelegt und instanziert
// UART Serial2(PIN_BT_TX, PIN_BT_RX);

#ifdef _PID_ADJUST
  PID_adjust *_pid_adjust;
#endif

void base_setup();
void motor_test_setup();
void main_setup();
void motor_test_loop();
void main_loop();

#ifdef _MAIN
  
#else
// Motor* motor[4];
  Motor m1(PIN_MOTOR_FL);    
  Motor m2(PIN_MOTOR_FR);    
  Motor m3(PIN_MOTOR_BL);    
  Motor m4(PIN_MOTOR_BR);    
#endif


void setup()
{
  base_setup();
#ifdef _MAIN
  main_setup();
  #else
  motor_test_setup();
  #endif
}


void loop()
{
  
  //  unsigned long enter = micros();
  #ifdef _MAIN
    main_loop();
  #else
    motor_test_loop();
  #endif
  
} /*------------------------ end of loop -----------------------------------------------*/

void base_setup(){
  pinMode(PIN_ESC_ON, OUTPUT);
  digitalWrite(PIN_ESC_ON, LOW); // MainPower für ESC´s abgeschaltet
  pinMode(PIN_LED_STATE, OUTPUT);
  digitalWrite(PIN_LED_STATE, LOW);

  Serial.begin(COM_SPEED);
  Serial2.begin(BT_SPEED);
  Serial.println("Serial COM OK");
  Serial2.println("BT COM OK ");
  Serial2.print(__DATE__);
  Serial2.print(" ");
  Serial2.println(__TIME__);

#ifdef _DEBUG_
  Logger::setOutputFunction(&localLogger);
  delay(50);
  Logger::setLogLevel(Logger::_DEBUG_); // Muss immer einen Wert in platformio.ini haben (SILENT)
#endif

  delay(5000);
  LOGGER_NOTICE("Program will initialized");
  // model.performance.min_loop_time = 0xffff;
  model.yaw.axisData[0] = &model.axisData[0]; // axisData wird mit yawData.axisData verknüpft
  model.yaw.axisData[1] = &model.axisData[1];

  Serial.println("********************************");
  Serial.println("*       KuCo Phantom 1         *");
  Serial.println("*                              *");
  Serial.print("*     ");
  Serial.print(__DATE__);
  Serial.print(" ");
  Serial.print(__TIME__);
  Serial.println("     *");
  Serial.print("*    EEPROM PID Address   "); /*Serial.print(PID_EEPROM_ADRRESS);*/
  Serial.println("     *");
  Serial.println("********************************");
  Serial.flush();
  Wire.begin();

  EEPROM.begin(512);
  
  delay(100);
}

void main_setup(){
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
      ->init(&model) // bekommt das komplette Model, Master of Desaster!!
      ->setYawAxis(reinterpret_cast<AxisYaw *>(Tasks["axisyaw"].get()))
      ->startFps(100);
  Tasks.add<Sensor>("sensor")->setModel(&model.sensorData)->startFps(10); // Übergabe des models in das objekt Sensor
//  Tasks.add<Sonic>("sonic")->setModel(&model.sonicData)->startFps(2); 
//  Tasks.add<Battery>("battery")->setModel(&model.batteryData)->startFps(1)

//  Tasks.add<Radio>("radio")->setModel(&model.RC_interface)->startFps(10);
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
} /*------------------------ end of setup ----------------------------------------------*/

void motor_test_setup(){
  LOGGER_VERBOSE("Enter....");
  //motor[0] = new Motor(PIN_MOTOR_FL);
  //motor[1] = new Motor(PIN_MOTOR_FR);
  // ....

  //for(uint8_t i = 0;i<4;i++) motor[i]->setup();
  m1.setup();
  m2.setup();
  m3.setup();
  m4.setup();
  m1.setMotorState(Motor::arming);
  m2.setMotorState(Motor::arming);
  m3.setMotorState(Motor::arming);
  m4.setMotorState(Motor::arming);
  
}

#define TEST_POWER 50
void motor_test_loop(){
  LOGGER_VERBOSE("loop has begun");
  static uint8_t POWER = 10;
  m1.update();      // array
  m2.update();
  m3.update();
  m4.update();
  if(m1.isArmed() && m4.isArmed()){
    if(Serial.available()){
      char key = Serial.read();
        switch(key){
          case '1':
          //case '2':
          //case '3':
          //case '4':
            Serial.println("M1");
            Serial.println(m1.getPower());
            //motor[key-48]->setMotorState(Motor::on);
            m1.setMotorState(Motor::on);
          break;
          case '2':
            Serial.println("M2");
            Serial.println(m2.getPower());
            m2.setMotorState(Motor::on);
          break;
          case '3':
            Serial.println("M3");
            Serial.println(m3.getPower());
            m3.setMotorState(Motor::on);
          break;
          case '4':
            Serial.println("M4");
            Serial.println(m4.getPower());
            m4.setMotorState(Motor::on);
          break;
          case '0':
            Serial.println("Motor off");
            m1.setMotorState(Motor::off);
            m2.setMotorState(Motor::off);
            m3.setMotorState(Motor::off);
            m4.setMotorState(Motor::off);
          break;
          case '+':
            POWER++;
            m1.setPower(POWER);
            m2.setPower(POWER);
            m3.setPower(POWER);
            m4.setPower(POWER);
            Serial.println(POWER);
          break;
          case '-':
            POWER--;
            m1.setPower(POWER);
            m2.setPower(POWER);
            m3.setPower(POWER);
            m4.setPower(POWER);
            Serial.println(POWER);
          break;

          default:
          ;
        };
    }else{
      Serial.println(m1.getMotorState());
      Serial.println(m2.getMotorState());
      Serial.println(m3.getMotorState());
      Serial.println(m4.getMotorState());
      Serial.println("------------------------");
    }
  }
  LOGGER_VERBOSE("Loop completed successfully");
}

void main_loop(){
  LOGGER_VERBOSE("loop has begun");
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
}
