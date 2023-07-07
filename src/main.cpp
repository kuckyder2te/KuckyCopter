/*  File name : main.cpp
    Project name : KuCo_Phantom 1
    Date : 2022-04-23

    Description : Drohne
    Hardware : Raspberry Pi Pico 2020
               Gyro : MPU9250
               Baro : MS5911
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

#define LOCAL_DEBUG

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

#ifdef _MAIN
void main_setup();  
void main_loop();
#elif _MOTOR
void motor_test_setup();
void motor_test_loop();
  Motor* motor[4]; 
#elif _AXIS
void axis_test_setup();
void axis_test_loop();
  AxisMotor* axis[2];
  axisData_t axisData;
#elif _SENSOR
void sensor_test_setup();
void sensor_test_loop();
  Sensor* sensor;
  Monitor* monitor;
#endif

/*--------------------------- end of declarations -----------------------------------------------*/
void setup()
{
  base_setup();
#ifdef _MAIN
  main_setup();
  #elif _MOTOR
  motor_test_setup();
  #elif _AXIS
  axis_test_setup();
  #elif _SENSOR
  sensor_test_setup();
#endif
}
/*--------------------------- end of standard setup ---------------------------------------------*/

void loop()
{
  //  unsigned long enter = micros();
  #ifdef _MAIN
    main_loop();
  #elif _MOTOR
    motor_test_loop();
  #elif _AXIS
    axis_test_loop();
  #elif _SENSOR
    sensor_test_loop();
  #endif
  
} /*------------------------ end of standard loop -----------------------------------------------*/

void base_setup(){
  pinMode(PIN_ESC_ON, OUTPUT);
  digitalWrite(PIN_ESC_ON, HIGH); // MainPower für ESC´s eingeschaltet,
                                  // will sagen, BC547 schaltet nicht durch, da die Basis HIGH ist
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
}/*------------------------ end of base setup ---------------------------------------------------*/
#ifdef _MAIN
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
} /*------------------------ end of setup -------------------------------------------------------*/
#endif

#ifdef _MOTOR
void motor_test_setup(){
  LOGGER_VERBOSE("Enter....");
  motor[0] = new Motor(PIN_MOTOR_FL);
  motor[1] = new Motor(PIN_MOTOR_FR);
  motor[2] = new Motor(PIN_MOTOR_BL);
  motor[3] = new Motor(PIN_MOTOR_BR);

  for(uint8_t i = 0; i<4; i++) {
    motor[i]->setup();
  }
  
} /*------------------------ end of motor_test_setup --------------------------------------------*/
#endif
#ifdef _AXIS
void axis_test_setup(){

  LOGGER_VERBOSE("Enter....");
  axis[0] = new AxisMotor("axismotor_a");
  axis[1] = new AxisMotor("axismotor_b");
  axis[0]->setModel(&axisData);
  axis[1]->setModel(&axisData);

  for(uint8_t i = 0; i < 2; i++){
    axis[i]->begin();
    //axis[i]->setState(AxisMotor::arming_start);
  }
} /*------------------------ end of axis_test_setup -------------------------------------------*/
#endif
#ifdef _SENSOR
void sensor_test_setup(){

  LOGGER_VERBOSE("Enter....");
  sensor = new Sensor("sensor");
  monitor = new Monitor("monitor");
  monitor->setModel(&model);
  sensor->setModel(&model.sensorData);
  sensor->begin();
} /*------------------------ end of sensor_test_setup -------------------------------------------*/
#endif

#ifdef _MOTOR
void motor_test_loop(){
  LOGGER_VERBOSE("loop has begun");
  static uint8_t test_power = BASE_MOTOR_POWER;

  for(uint8_t i = 0; i<4; i++)
    motor[i]->update();

  if(motor[0]->isArmed()&&motor[3]->isArmed()){
  
    if(Serial.available()){
      char key = Serial.read();
        switch(key){
          case '1':
          case '2':
          case '3':
          case '4': 
            LOGGER_NOTICE_FMT("Motor %i Power %i", key-'1', motor[key-'1']->getPower());
            motor[key-'1']->setMotorState(Motor::on);
          break;
          case '0':
            LOGGER_NOTICE("Motors off");
            for(uint8_t i = 0; i<4; i++)
              motor[i]->setMotorState(Motor::off);
          break;
          case '+':
            test_power++;
              for(uint8_t i = 0; i<4; i++){
                motor[i]->setPower(test_power);
                LOGGER_NOTICE_FMT("Motor %i Power %i", key-'1', motor[key-'1']->getPower());
              }
          break;
          case '-':
            test_power--;
               for(uint8_t i = 0; i<4; i++){
                motor[i]->setPower(test_power);   
                LOGGER_NOTICE_FMT("Motor %i Power %i", key-'1', motor[key-'1']->getPower());
              }
          break;
          case 'x':
            motor[0]->setMotorState(Motor::power_off);
          break;

          default:
          ;
        };
    }else{
      
      for(uint8_t i = 0; i<4; i++)
        motor[i]->getMotorState();
      LOGGER_NOTICE("------------------------");
    }
  }else{
    if(Serial.available()){
        char key = Serial.read();
        switch(key){
          case 'a':
            for(uint8_t i = 0; i<4; i++) {
              motor[i]->setMotorState(Motor::arming);
            }
          break;
        }
      }
  }
  LOGGER_VERBOSE("Loop completed successfully");
}
#endif
#ifdef _AXIS
void axis_test_loop(){

}
#endif

#ifdef _SENSOR
void sensor_test_loop(){
  monitor->update();
  sensor->update();
}
#endif

#ifdef _MAIN
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
#endif