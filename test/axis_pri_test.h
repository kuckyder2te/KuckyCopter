#pragma once

// #define EEPROM_OFFSET 300

#include "..\src\config.h"
#include "..\lib\axisBase.h"
#include "..\lib\axisMotor.h"
#include "..\lib\monitor.h"
#include "..\lib\model.h"
#include "..\lib\newPID.h"
#include "..\lib\sensors.h"
#include "..\lib\def.h"

AxisMotor *axis;
Monitor *monitor;
NewPID *newPid;
Sensor *sensor;

extern model_t model;

bool menu = false;

uint8_t _pidParameter = 0;
float testP = 0.1;
float testI = 0;
float testD = 0;
float testEF = 50;

void print_pid_menu();

void print_main_menu()
{
  Serial.println("----------- Primary Axis Test setup menu -------");
  Serial.println("A for arming");
  Serial.println("S for current State");
  Serial.println("Key (+) or (-) increment or decrement Power.");
  Serial.println("I for invertRoll");
  Serial.println("D disable PID");
  Serial.println("E enable PID");
  Serial.println("St(o)p Motor");
  Serial.println("R for ready (Start Motors)");
  Serial.println("[SPACE] Power Off");
  Serial.println("P for PID Menu");
  Serial.println("? for this Menu");
  Serial.println("------------------------------------------------");
}

void main_gui(char key)
{
  static int16_t power = 0;
  switch (toupper(key))
  {
  case 'A':
    axis->setState(AxisMotor::state::arming_start);
    break;
  case 'S':
    Serial.print("isStandby: ");
    Serial.println(axis->isStandby());
    Serial.print("isReady: ");
    Serial.println(axis->isReady());
    Serial.print("isDeactivatePID: ");
    Serial.println(axis->isDeactivatePID());
    Serial.print("isArmed: ");
    Serial.println(axis->isArmed());
    break;
  case '+':
    power++;
    Serial.print("Power: ");
    Serial.println(power);
    axis->setPower(power);
    break;
  case '-':
    if (power > 0)
      power--;
    Serial.print("Power: ");
    Serial.println(power);
    axis->setPower(power);
    break;
  case 'I':
    Serial.println("Invert Roll");
    axis->InvertRoll();
    break;
  case 'O':
    Serial.println("Stop Motor");
    axis->setState(AxisMotor::state::standby);
    break;
  case 'R':
    Serial.println("Motor Start");
    axis->setState(AxisMotor::state::ready);
    break;
  case ' ':
    axis->setState(AxisMotor::state::off);
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
  switch (toupper(key))
  {
      case 'L':
        Serial.print("Pitch Level: ");
        Serial.println(model.sensorData.pitch);
        Serial.print("Roll Level: ");
        Serial.println(model.sensorData.roll);
        Serial.print("SetPoint: ");
        Serial.println(model.axisData[axisName::primary].setpoint);
        break;
      case 'W':
        Serial.print("Error: ");
        Serial.println(model.axisData[0].pidError);
        Serial.print("Axis PidError: ");
        Serial.println(axis->getPidError());
        break;
      case 'S':
        Serial.print("P: ");
        Serial.println(newPid->getP(), 2);
        Serial.print("I: ");
        Serial.println(newPid->getI(), 2);
        Serial.print("D: ");
        Serial.println(newPid->getD(), 2);
        Serial.print("EF: ");
        Serial.println(newPid->getEF());
        Serial.print("ExecutionTime:");
        Serial.println(newPid->getExecutionTime());
        break;
      case 'R':
        Serial.println("Reset");
        newPid->initPID();
        break;
      case 'P':
        Serial.println("P");
        _pidParameter = 1;
        break;
      case 'I':
        Serial.println("I");
        _pidParameter = 2;
        break;
      case 'D':
        Serial.println("D");
        _pidParameter = 3;
        break;
      case 'F':
        Serial.println("F");
        _pidParameter = 4;
        break;
      case 'E':
        Serial.println("Enable PID");
        axis->setState(AxisMotor::state::enablePID);
        break;
      case 'A':
        Serial.println("Disable PID");
        axis->setState(AxisMotor::state::disablePID);
        break;
      case '8':
        Serial.print("rcX: ");
        Serial.println(++model.RC_interface.RX_payload.rcRoll); // erst erhöhen und dann schreiben
        break;
      case '2':
        Serial.print("rcX: ");
        Serial.println(--model.RC_interface.RX_payload.rcRoll);
        break;
        
      case '+':
      switch (_pidParameter)
      {
      case 1:
        testP += 0.001;
        newPid->setP(testP);
        Serial.print("P: ");
        Serial.println(testP, 2);
        break;

      case 2:
        testI += 0.0001;
        newPid->setI(testI);
        Serial.print("I: ");
        Serial.println(testI, 2);
        break;
      case 3:
        testD += 0.0001;
        newPid->setD(testD);
        Serial.print("D: ");
        Serial.println(testD), 2;
        break;
      case 4:
        testEF += 1;
        newPid->setEF(testEF);
        Serial.print("ExFreq: ");
        Serial.println(testEF, 0);
        break;
      }

    case '-':
      switch (_pidParameter)
      {
      case 1:
        testP -= 0.001;
        newPid->setP(testP);
        Serial.print("P: ");
        Serial.println(testP, 2);
        break;

      case 2:
        testI -= 0.0001;
        newPid->setI(testI);
        Serial.print("I: ");
        Serial.println(testI, 2);
        break;
      case 3:
        testD -= 0.0001;
        newPid->setD(testD);
        Serial.print("D: ");
        Serial.println(testD), 2;
        break;
      case 4:
        testEF -= 1;
        newPid->setEF(testEF);
        Serial.print("ExFreq: ");
        Serial.println(testEF, 0);
        break;
      }
      
      break;
      case ' ':
        axis->setState(AxisMotor::state::off);
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
  Serial.println("----------- Primary Axis Test PID Menu ---------");
  Serial.println(" P - kP");
  Serial.println(" I - kI");
  Serial.println(" D - kD");
  Serial.println(" F - ef");
  Serial.println(" W - Show Error");
  Serial.println(" E - enable PID");
  Serial.println(" A - disable PID");
  Serial.println("(+) - increment PID coefftient");
  Serial.println("(-) - decrement PID coefftient");
  Serial.println(" 8 - increment setPoint");
  Serial.println(" 2 - decrement setPoint");
  Serial.println("[SPACE] Power Off");
  Serial.println(" R - Reset/Init PID");
  Serial.println(" S - Show PID-Values");
  Serial.println(" L - Show IMU Levels");
  Serial.println(" M for Main menu");
  Serial.println(" ? for this Menu");
  Serial.println("------------------------------------------------");
} /*------------------------- end of print_pid_menu ----------------------------------------------*/

void test_setup()
{
  delay(2000); // for switching terminal on
  LOGGER_VERBOSE("Enter....");

  sensor = new Sensor("Sensor");
  sensor->setModel(&model.sensorData)->begin();
  axis = new AxisMotor("Primary axismotor");
  model.axisData[0].feedback = &model.sensorData.roll; // must be before setNodel because of feedback Pointer
  model.axisData[0].rcX = &model.RC_interface.RX_payload.rcRoll;
  model.axisData[0].rcY = &model.RC_interface.RX_payload.rcPitch;
  axis->setModel(&model.axisData[axisName::primary])->begin();
  axis->initMotorOrdered(PIN_MOTOR_FL)->initMotorOrdered(PIN_MOTOR_BR);
  newPid = axis->getPid();
  monitor = new Monitor("Monitor", Report_t::AXIS);
  monitor->setModel(&model)->begin();

  print_main_menu();
} /*------------------------- end of test_setup --------------------------------------------------*/

void test_loop()
{
  if (Serial.available())
  {
    char key = Serial.read();
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
  model.axisData[0].setpoint = model.RC_interface.RX_payload.rcRoll;
  axis->update();
  monitor->update();
  sensor->enter();
}
/*------------------------ end of axis pri test programm ----------------------------------------*/
