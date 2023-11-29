#pragma once
/* File name : all_axis_test.h
   Project name : KuckyCopter 2
   Author: Stephan Scholz /  Wilhelm Kuckelsberg
   Date : 2022-06-17
   Description : Drohne
*/

#include "..\src\config.h"
#include "..\lib\axisBase.h"
#include "..\lib\axisMotor.h"
#include "..\lib\monitor.h"
#include "..\lib\model.h"
#include "..\lib\newPID.h"
#include "..\lib\sensors.h"
// #include "..\lib\def.h"

#define LOCAL_DEBUG
#include "..\lib\myLogger.h"

AxisMotor *axisTest[2];
Monitor *monitor;
NewPID *newPid[2];
Sensor *sensor;
Motor *motor;

extern HardwareSerial *TestOutput;
extern model_t model;

bool menu = false;
bool recorded = false;

uint8_t _pidParameter = 0;

void print_pid_menu();

void print_main_menu()
{
  TestOutput->println("----------- Primary Axis Test setup menu -------");
  TestOutput->println("A for arming");
  TestOutput->println("S for current State");
  TestOutput->println("Key (+) or (-) increment or decrement Power.");
  TestOutput->println("G get resulting power.");
  TestOutput->println("I for invertRoll");
  TestOutput->println("D disable PID");
  TestOutput->println("E enable PID");
  TestOutput->println("St(o)p Motor");
  TestOutput->println("R for ready (Start Motors)");
  TestOutput->println("[SPACE] Power Off");
  TestOutput->println("P for PID Menu");
  TestOutput->println("? for this Menu");
  TestOutput->println("------------------------------------------------");
}

void main_gui(char key)
{
  static int16_t power = 0;
  switch (toupper(key))
  {
  case 'A':
    axisTest[axisName::primary]->setState(AxisMotor::state::arming_start);
    axisTest[axisName::secondary]->setState(AxisMotor::state::arming_start);
    break;
  case 'S':
    TestOutput->print("isStandby: ");
    TestOutput->println(axisTest[axisName::primary]->isStandby());
    TestOutput->print("isReady: ");
    TestOutput->println(axisTest[axisName::primary]->isReady());
    TestOutput->print("isDeactivatePID: ");
    TestOutput->println(axisTest[axisName::primary]->isDeactivatePID());
    TestOutput->print("isArmed: ");
    TestOutput->println(axisTest[axisName::primary]->isArmed());
    break;
  case 'G':
    TestOutput->println(motor->getResultingPower());
    break;

  case '+':
    power++;
    TestOutput->print("Power: ");
    TestOutput->println(power);
    axisTest[axisName::primary]->setPower(power);
    axisTest[axisName::secondary]->setPower(power);
    break;
  case '-':
    if (power > 0)
      power--;
    TestOutput->print("Power: ");
    TestOutput->println(power);
    axisTest[axisName::primary]->setPower(power);
    axisTest[axisName::secondary]->setPower(power);
    break;
  case 'I':
    TestOutput->println("Invert Roll");
    axisTest[axisName::primary]->InvertRoll();
    break;
  case 'O':
    TestOutput->println("Stop Motor");
    axisTest[axisName::primary]->setState(AxisMotor::state::standby);
    axisTest[axisName::secondary]->setState(AxisMotor::state::standby);
    break;
  case 'R':
    TestOutput->println("Motor Start");
    axisTest[axisName::primary]->setState(AxisMotor::state::ready);
    axisTest[axisName::secondary]->setState(AxisMotor::state::ready);
    break;
  case ' ':
    axisTest[axisName::primary]->setState(AxisMotor::state::off);
    axisTest[axisName::secondary]->setState(AxisMotor::state::off);
    break;
  case 'P':
    print_pid_menu();
    menu = true;
    break;
  case '?':
    print_main_menu();
    break;
  }
} /*------------------------- end of main_gui ---------------------------------------------------*/

void pid_gui(char key)
{
  static float temp;

  switch (toupper(key))
  {
  case 'L':
    TestOutput->print("Pitch Level: ");
    TestOutput->println(model.sensorData.pitch);
    TestOutput->print("Roll Level: ");
    TestOutput->println(model.sensorData.roll);
    TestOutput->print("SetPoint: ");
    TestOutput->println(model.axisData[axisName::primary].setpoint);
    break;
  case 'W':
    TestOutput->print("Error: ");
    TestOutput->println(model.axisData[axisName::primary].pidError);
    break;
  case 'S':
    TestOutput->print("P: ");
    TestOutput->println(newPid[axisName::primary]->getP(), 3);
    TestOutput->print("I: ");
    TestOutput->println(newPid[axisName::primary]->getI(), 3);
    TestOutput->print("D: ");
    TestOutput->println(newPid[axisName::primary]->getD(), 3);
    TestOutput->print("EF: ");
    TestOutput->println(newPid[axisName::primary]->getEF());
    TestOutput->print("ExecutionTime:");
    TestOutput->println(newPid[axisName::primary]->getExecutionTime());

    TestOutput->print("P: ");
    TestOutput->println(newPid[axisName::secondary]->getP(), 3);
    TestOutput->print("I: ");
    TestOutput->println(newPid[axisName::secondary]->getI(), 3);
    TestOutput->print("D: ");
    TestOutput->println(newPid[axisName::secondary]->getD(), 3);
    TestOutput->print("EF: ");
    TestOutput->println(newPid[axisName::secondary]->getEF());
    TestOutput->print("ExecutionTime:");
    TestOutput->println(newPid[axisName::secondary]->getExecutionTime());
    break;
  case 'R':
    TestOutput->println("Reset");
    newPid[axisName::primary]->initPID();
    newPid[axisName::secondary]->initPID();
    break;
  case 'O':
    TestOutput->print("Throttle is selected. Power= ");
    TestOutput->print(axisTest[axisName::primary]->getPower());
    _pidParameter = 5;
    break;
  case 'P':
    TestOutput->print("Parameter P is selected. P= ");
    TestOutput->print(newPid[axisName::primary]->getP());
    _pidParameter = 1;
    break;
  case 'I':
    TestOutput->println("Parameter I is selected. I= ");
    TestOutput->print(newPid[axisName::primary]->getI());
    _pidParameter = 2;
    break;
  case 'D':
    TestOutput->println("Parameter D is selected. D= ");
    TestOutput->print(newPid[axisName::primary]->getD());
    _pidParameter = 3;
    break;
  case 'F':
    TestOutput->println("Parameter eF is selected");
    _pidParameter = 4;
    break;
  case 'E':
    TestOutput->println("Enable PID");
    axisTest[axisName::primary]->setState(AxisMotor::state::enablePID);
    axisTest[axisName::secondary]->setState(AxisMotor::state::enablePID);
    break;
  case 'A':
    TestOutput->println("Disable PID");
    axisTest[axisName::primary]->setState(AxisMotor::state::disablePID);
    axisTest[axisName::secondary]->setState(AxisMotor::state::disablePID);
    break;
  case 'C':
    recorded = !recorded;
    break;
  case '8':
    TestOutput->print("rcX: ");
    TestOutput->println(++model.RC_interface.RX_payload.rcRoll); // erst erhöhen und dann schreiben
    break;
  case '2':
    TestOutput->print("rcX: ");
    TestOutput->println(--model.RC_interface.RX_payload.rcRoll);
    break;
  case '+':
    switch (_pidParameter)
    {
    case 1:
      temp = newPid[axisName::primary]->getP();
      newPid[axisName::primary]->setP(temp += DOT_3);
      newPid[axisName::secondary]->setP(temp);
      TestOutput->print("kP: ");
      TestOutput->println(temp, 4);
      break;
    case 2:
      temp = newPid[axisName::primary]->getI();
      newPid[axisName::primary]->setI(temp += DOT_3);
      newPid[axisName::secondary]->setI(temp);
      TestOutput->print("kI: ");
      TestOutput->println(temp, 4);
      break;
    case 3:
      temp = newPid[axisName::primary]->getD();
      newPid[axisName::primary]->setD(temp += DOT_3);
      newPid[axisName::secondary]->setD(temp);
      TestOutput->print("kD: ");
      TestOutput->println(temp, 4);
      break;
    case 4:
      temp = newPid[axisName::primary]->getEF();
      newPid[axisName::primary]->setEF(temp += 1);
      newPid[axisName::secondary]->setEF(temp);
      TestOutput->print("eF: ");
      TestOutput->println(temp, 4);
      break;
    case 5:
      temp = axisTest[axisName::primary]->getPower();
      axisTest[axisName::primary]->setPower(++temp);
      axisTest[axisName::secondary]->setPower(temp);
      TestOutput->print("Throttle = ");
      TestOutput->println(temp);
      break;
    } // end of switch
    newPid[axisName::primary]->saveParameters();
    newPid[axisName::secondary]->saveParameters();
    break;

  case '-':
    switch (_pidParameter)
    {
    case 1:
      temp = newPid[axisName::primary]->getP();
      newPid[axisName::primary]->setP(temp -= DOT_3);
      newPid[axisName::secondary]->setP(temp);
      TestOutput->print("kP: ");
      TestOutput->println(temp, 4);
      break;
    case 2:
      temp = newPid[axisName::primary]->getI();
      newPid[axisName::primary]->setI(temp -= DOT_3);
      newPid[axisName::secondary]->setI(temp);
      TestOutput->print("kI: ");
      TestOutput->println(temp, 4);
      break;
    case 3:
      temp = newPid[axisName::primary]->getD();
      newPid[axisName::primary]->setD(temp -= DOT_3);
      newPid[axisName::secondary]->setD(temp);
      TestOutput->print("kD: ");
      TestOutput->println(temp, 4);
      break;
    case 4:
      temp = newPid[axisName::primary]->getEF();
      newPid[axisName::primary]->setEF(temp -= 1);
      newPid[axisName::secondary]->setEF(temp);
      TestOutput->print("eF: ");
      TestOutput->println(temp, 4);
      break;
    case 5:
      temp = axisTest[axisName::primary]->getPower();
      axisTest[axisName::primary]->setPower(--temp);
      axisTest[axisName::secondary]->setPower(temp);
      TestOutput->print("Throttle = ");
      TestOutput->println(temp);
      break;
    }
    newPid[axisName::primary]->saveParameters();
    newPid[axisName::secondary]->saveParameters();
    break;

  case ' ':
    axisTest[axisName::primary]->setState(AxisMotor::state::off);
    axisTest[axisName::secondary]->setState(AxisMotor::state::off);
    break;
  case 'M':
    print_main_menu();
    menu = false;
    break;
  case '?':
    print_pid_menu();
    break;
  }
} /*------------------------- end of pid_gui ---------------------------------------------------*/

void print_pid_menu()
{
  TestOutput->println("----------- All Axis Test PID Menu ---------");
  TestOutput->println(" P - kP");
  TestOutput->println(" I - kI");
  TestOutput->println(" D - kD");
  TestOutput->println(" F - ef");
  TestOutput->println(" W - Show Error");
  TestOutput->println(" E - enable PID");
  TestOutput->println(" A - disable PID");
  TestOutput->println("(+) - increment PID coefftient");
  TestOutput->println("(-) - decrement PID coefftient");
  TestOutput->println(" 8 - increment setPoint");
  TestOutput->println(" 2 - decrement setPoint");
  TestOutput->println("[SPACE] Power Off");
  TestOutput->println(" R - Reset/Init PID");
  TestOutput->println(" O - for ready (Start Motors)");
  TestOutput->println(" S - Show PID-Values");
  TestOutput->println(" L - Show IMU Levels");
  TestOutput->println(" M for Main menu");
  TestOutput->println(" Re(c)ord on/off");
  TestOutput->println(" ? for this Menu");
  TestOutput->println("--------------------------------------------");
} /*------------------------- end of print_pid_menu ----------------------------------------------*/

void test_setup()
{
  delay(1000); // for switching terminal on
  LOGGER_VERBOSE("Enter....");

  sensor = new Sensor("Sensor");
  sensor->setModel(&model.sensorData)->begin();
  axisTest[axisName::primary] = new AxisMotor("Primary axismotor");
  axisTest[axisName::secondary] = new AxisMotor("Secondary axismotor");
  model.axisData[axisName::primary].feedback = &model.sensorData.roll; // must be before setModel because of feedback Pointer
  model.axisData[axisName::primary].rcX = &model.RC_interface.RX_payload.rcRoll;
  model.axisData[axisName::primary].rcY = &model.RC_interface.RX_payload.rcPitch;
  model.axisData[axisName::secondary].feedback = &model.sensorData.roll;
  model.axisData[axisName::secondary].rcX = &model.RC_interface.RX_payload.rcRoll;
  model.axisData[axisName::secondary].rcY = &model.RC_interface.RX_payload.rcPitch;
  axisTest[axisName::primary]->setModel(&model.axisData[axisName::primary])->begin();
  axisTest[axisName::secondary]->setModel(&model.axisData[axisName::secondary])->begin();
  axisTest[axisName::primary]->initMotorOrdered(PIN_MOTOR_FL)->initMotorOrdered(PIN_MOTOR_BR);
  axisTest[axisName::secondary]->initMotorOrdered(PIN_MOTOR_FR)->initMotorOrdered(PIN_MOTOR_BL);
  newPid[axisName::primary] = axisTest[axisName::primary]->getPid();
  newPid[axisName::secondary] = axisTest[axisName::secondary]->getPid();
  monitor = new Monitor("Monitor", Report_t::ALL_AXIS);
  monitor->setModel(&model)->begin();

  print_main_menu();
} /*------------------------- end of test_setup --------------------------------------------------*/

void test_loop()
{
  // TestOutput->println("loop");
  unsigned long _lastLooptime = micros();
  static unsigned long _lastMillis = millis();
  if (recorded && (millis() - _lastMillis > 100))
  {
    _lastMillis = millis();
    TestOutput->printf("/*%i,%i,%i,%i,%i,%.2f,%.2f,%.2f,%i,%i*/\r\n",
                       model.axisData[axisName::primary].power,
                       axisTest[axisName::primary]->getMotorPower(false),
                       axisTest[axisName::primary]->getMotorPower(true), // ???
                       model.sensorData.roll,
                       model.axisData[axisName::primary].pidError,
                       newPid[axisName::primary]->getP(),
                       newPid[axisName::primary]->getI(),
                       newPid[axisName::primary]->getD(),
                       model.axisData[axisName::primary].setpoint,
                       model.looptime);
  }
  if (TestOutput->available())
  {
    char key = TestOutput->read();
    switch (menu)
    {
    case false:
      main_gui(key);
      break;
    case true:
      pid_gui(key);
      break;
    }
  }
  model.axisData[axisName::primary].setpoint = model.RC_interface.RX_payload.rcRoll;
  model.axisData[axisName::secondary].setpoint = model.RC_interface.RX_payload.rcRoll;
  axisTest[axisName::primary]->update();
  axisTest[axisName::secondary]->update();
  monitor->update();
  sensor->enter();
  model.looptime = micros() - _lastLooptime;
}
/*------------------------ end of all_axis_test class -------------------------------------------*/
