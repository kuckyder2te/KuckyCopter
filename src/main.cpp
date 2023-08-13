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
#include <Adafruit_Sensor.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_SPIDevice.h>
#include "EEPROM.h"

#include "..\lib\sensors.h"
#include "..\lib\sonics.h"
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

model_t model;
// UART Serial2(PIN_BT_TX, PIN_BT_RX);

#ifdef _PID_ADJUST
PID_adjust *_pid_adjust;
#endif

void send_data_to_drohne();

void performance_test(uint16_t threshold)
{
  static unsigned long last_runtime = micros();
  uint16_t delta = micros() - last_runtime;
  if (delta > threshold)
    Serial.println(delta);
  last_runtime = micros();
}

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
      ->init(&model) // bekommt das komplette Model, Master of Desaster!!
      ->setYawAxis(reinterpret_cast<AxisYaw *>(Tasks["axisyaw"].get()))
      ->startFps(100);
  Tasks.add<Sensor>("sensor")->setModel(&model.sensorData)->startFps(10); // Übergabe des models in das objekt Sensor
  Tasks.add<Sonic>("sonic")->setModel(&model.sonicData)->startFps(2);
  //  Tasks.add<Battery>("battery")->setModel(&model.batteryData)->startFps(1)

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
//---------------------------------------------------------------------------------------------------------------------
void main_loop()
{
  LOGGER_VERBOSE("loop has begun");
  Tasks.update();
  Tasks["sensor"]->enter();
  LOGGER_VERBOSE("Loop completed successfully");
}
//---------------------------------------------------------------------------------------------------------------------
#elif _MOTOR
Motor *motor[4];

void motor_test_setup()
{
  LOGGER_VERBOSE("Enter....");
  motor[0] = new Motor(PIN_MOTOR_FL);
  motor[1] = new Motor(PIN_MOTOR_FR);
  motor[2] = new Motor(PIN_MOTOR_BL);
  motor[3] = new Motor(PIN_MOTOR_BR);

  for (uint8_t i = 0; i < 4; i++)
  {
    motor[i]->setup();
  }

  if (Serial.available())
  {
    if (!Serial.read())
    {
      Serial.println("----------- Motor setup menu -------------------");
      Serial.println("Key 1- 4 choose the motor.");
      Serial.println("Key 0 will stop all motors.");
      Serial.println("Key (+) or (-) increment or decrement the speed.");
      Serial.println("Key X set motorstate to off.");
      Serial.println("Press any key to contin.");
      Serial.println("------------------------------------------------");
    }
  }
}
//---------------------------------------------------------------------------------------
void motor_test_loop()
{
  LOGGER_VERBOSE("loop has begun");
  static uint8_t test_power = BASE_MOTOR_POWER;

  for (uint8_t i = 0; i < 4; i++)
    motor[i]->update();

  if (motor[0]->isArmed() && motor[3]->isArmed())
  {

    if (Serial.available())
    {
      char key = Serial.read(); // z.B. Key 1 ~ 49  ~ motor[key-'1'] = motor[0]
      switch (key)
      {
      case '1':
      case '2':
      case '3':
      case '4':
        LOGGER_NOTICE_FMT("Motor %i Power %i", key - '1', motor[key - '1']->getPower());
        motor[key - '1']->setMotorState(Motor::on);
        break;
      case '0':
        LOGGER_NOTICE("Motors off");
        for (uint8_t i = 0; i < 4; i++)
          motor[i]->setMotorState(Motor::off);
        break;
      case '+':
        test_power++;
        for (uint8_t i = 0; i < 4; i++)
        {
          motor[i]->setPower(test_power);
          LOGGER_NOTICE_FMT("Motor %i Power %i", key - '1', motor[key - '1']->getPower());
        }
        break;
      case '-':
        test_power--;
        for (uint8_t i = 0; i < 4; i++)
        {
          motor[i]->setPower(test_power);
          LOGGER_NOTICE_FMT("Motor %i Power %i", key - '1', motor[key - '1']->getPower());
        }
        break;
      case 'x':
        motor[0]->setMotorState(Motor::power_off);
        break;

      default:;
      };
    }
    else
    {

      for (uint8_t i = 0; i < 4; i++)
        motor[i]->getMotorState();
      LOGGER_NOTICE("------------------------");
    }
  }
  else
  {
    if (Serial.available())
    {
      char key = Serial.read();
      switch (key)
      {
      case 'a':
        for (uint8_t i = 0; i < 4; i++)
        {
          motor[i]->setMotorState(Motor::arming);
        }
        break;
      }
    }
  }
  LOGGER_VERBOSE("Loop completed successfully");
}
//---------------------------------------------------------------------------------------
#elif _SONIC
Sonic *sonic;
Monitor *monitor;
void sonic_test_setup()
{
  LOGGER_VERBOSE("Enter....");
  sonic = new Sonic("sonic");
  monitor = new Monitor("monitor", Report_t::SONIC);
  monitor->setModel(&model);
  sonic->setModel(&model.sonicData);
  sonic->begin();
}
//---------------------------------------------------------------------------------------------------------------------
void sonic_test_loop()
{
  monitor->update();
  sonic->update();
  performance_test(60);
}
//---------------------------------------------------------------------------------------------------------------------
#elif _AXIS
AxisMotor *axis[2];
axisData_t axisData;
void axis_test_setup()
{

  LOGGER_VERBOSE("Enter....");
  axis[0] = new AxisMotor("axismotor_a");
  axis[1] = new AxisMotor("axismotor_b");
  axis[0]->setModel(&axisData);
  axis[1]->setModel(&axisData);

  for (uint8_t i = 0; i < 2; i++)
  {
    axis[i]->begin();
  }
}
//---------------------------------------------------------------------------------------------------------------------
void axis_test_loop()
{
}
//---------------------------------------------------------------------------------------------------------------------
#elif _SENSOR
Sensor *sensor;
Monitor *monitor;
void sensor_test_setup()
{
  LOGGER_VERBOSE("Enter....");
  sensor = new Sensor("sensor");
  monitor = new Monitor("monitor", Report_t::SENSOR);
  monitor->setModel(&model);
  sensor->setModel(&model.sensorData);
  sensor->begin();
}
/*------------------------ end of sensor_test_setup -------------------------------------------*/
void sensor_test_loop()
{
  monitor->update();
  sensor->enter();
}
/*------------------------ end of sensor_test_loop -------------------------------------------*/
#elif _RADIO
Radio *radio;
Monitor *monitor;
void radio_test_setup()
{
  radio = new Radio("radio");
  radio->setModel(&model.RC_interface);
  radio->begin();
  monitor = new Monitor("monitor", Report_t::RADIO);
  monitor->setModel(&model);
}
void radio_test_loop()
{
  radio->update();
  monitor->update();

  send_data_to_drohne();
}

#elif _PID
NewPID *newPID[3];

void pid_test_setup()
{
}

void pid_test_loop()
{
}

#elif _FLYCONTROL
FlyController *flycontrol;
Monitor *monitor;
void flycontrol_test_setup()
{
  flycontrol = new FlyController("Flycontrol");
  monitor = new Monitor("monitor", Report_t::FLYCONTROL);
  monitor->setModel(&model);
}

void flycontrol_test_loop()
{
  flycontrol->update();
  monitor->update();
}

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
#elif _SONIC
  sonic_test_setup();
#elif _RADIO
  radio_test_setup();
#elif _PID
  pid_test_setup();
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
#elif _SONIC
  sonic_test_loop();
#elif _RADIO
  radio_test_loop();
#elif _PID
  pid_test_loop();
#endif

} /*------------------------ end of standard loop -----------------------------------------------*/

void base_setup()
{
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

  delay(1000);
  LOGGER_NOTICE("Program will initialized");
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
} /*------------------------ end of base setup --------------------------------------------------*/

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
