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
    axis->setState(AxisMotor::state::arming_start);
    break;
  case 'S':
    TestOutput->print("isStandby: ");
    TestOutput->println(axis->isStandby());
    TestOutput->print("isReady: ");
    TestOutput->println(axis->isReady());
    TestOutput->print("isDeactivatePID: ");
    TestOutput->println(axis->isDeactivatePID());
    TestOutput->print("isArmed: ");
    TestOutput->println(axis->isArmed());
    break;
  case 'G':
    TestOutput->println(motor->getResultingPower());
    break;
    
  case '+':
    power++;
    TestOutput->print("Power: ");
    TestOutput->println(power);
    axis->setPower(power);
    break;
  case '-':
    if (power > 0)
      power--;
    TestOutput->print("Power: ");
    TestOutput->println(power);
    axis->setPower(power);
    break;
  case 'I':
    TestOutput->println("Invert Roll");
    axis->InvertRoll();
    break;
  case 'O':
    TestOutput->println("Stop Motor");
    axis->setState(AxisMotor::state::standby);
    break;
  case 'R':
    TestOutput->println("Motor Start");
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
    TestOutput->println(newPid->getP(), 2);
    TestOutput->print("I: ");
    TestOutput->println(newPid->getI(), 2);
    TestOutput->print("D: ");
    TestOutput->println(newPid->getD(), 2);
    TestOutput->print("EF: ");
    TestOutput->println(newPid->getEF());
    TestOutput->print("ExecutionTime:");
    TestOutput->println(newPid->getExecutionTime());
    break;
  case 'R':
    TestOutput->println("Reset");
    newPid->initPID();
    break;
  case 'O':
    TestOutput->print("Throttle is selected. Power= ");TestOutput->print(axis->getPower());
    _pidParameter = 5;
    break;  
  case 'P':
    TestOutput->print("Parameter P is selected. P= ");TestOutput->print(newPid->getP());
    _pidParameter = 1;
    break;
  case 'I':
    TestOutput->println("Parameter I is selected. I= ");TestOutput->print(newPid->getI());
    _pidParameter = 2;
    break;
  case 'D':
    TestOutput->println("Parameter D is selected. D= ");TestOutput->print(newPid->getD());
    _pidParameter = 3;
    break;
  case 'F':
    TestOutput->println("Parameter eF is selected");
    _pidParameter = 4;
    break;
  case 'E':
    TestOutput->println("Enable PID");
    axis->setState(AxisMotor::state::enablePID);
    break;
  case 'A':
    TestOutput->println("Disable PID");
    axis->setState(AxisMotor::state::disablePID);
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
      temp = newPid->getP();
      newPid->setP(temp += DOT_1);
      TestOutput->print("kP: ");
      TestOutput->println(temp, 4);
      break;
    case 2:
      temp = newPid->getI();
      newPid->setI(temp += DOT_2);
      TestOutput->print("kI: ");
      TestOutput->println(temp, 4);
      break;
    case 3:
      temp = newPid->getD();
      newPid->setD(temp += DOT_2);
      TestOutput->print("kD: ");
      TestOutput->println(temp, 4);
      break;
    case 4:
      temp = newPid->getEF();
      newPid->setEF(temp += 1);
      TestOutput->print("eF: ");
      TestOutput->println(temp, 4);
      break;
    case 5:
      temp = axis->getPower();
      axis->setPower(++temp);
      TestOutput->print("Throttle = ");
      TestOutput->println(temp);
      break;
    }     // end of switch
    newPid->saveParameters();
    break;

  case '-':
    switch (_pidParameter)
    {
    case 1:
      temp = newPid->getP();
      newPid->setP(temp -= DOT_1);
      TestOutput->print("kP: ");
      TestOutput->println(temp, 4);
      break;
    case 2:
      temp = newPid->getI();
      newPid->setI(temp -= DOT_2);
      TestOutput->print("kI: ");
      TestOutput->println(temp, 4);
      break;
    case 3:
      temp = newPid->getD();
      newPid->setD(temp -= DOT_2);
      TestOutput->print("kD: ");
      TestOutput->println(temp, 4);
      break;
    case 4:
      temp = newPid->getEF();
      newPid->setEF(temp -= 1);
      TestOutput->print("eF: ");
      TestOutput->println(temp, 4);
      break;
    case 5:
      temp = axis->getPower();
      axis->setPower(--temp);
      TestOutput->print("Throttle = ");
      TestOutput->println(temp);
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
  TestOutput->println("----------- Primary Axis Test PID Menu ---------");
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
  TestOutput->println("------------------------------------------------");
} /*------------------------- end of print_pid_menu ----------------------------------------------*/

void test_setup()
{
  delay(1000); // for switching terminal on
  LOGGER_VERBOSE("Enter....");

  sensor = new Sensor("Sensor");
  sensor->setModel(&model.sensorData)->begin();
  axis = new AxisMotor("Primary axismotor");
  model.axisData[axisName::primary].feedback = &model.sensorData.roll; // must be before setModel because of feedback Pointer
  model.axisData[axisName::primary].rcX = &model.RC_interface.RX_payload.rcRoll;
  model.axisData[axisName::primary].rcY = &model.RC_interface.RX_payload.rcPitch;
  axis->setModel(&model.axisData[axisName::primary])->begin();
  axis->initMotorOrdered(PIN_MOTOR_FL)->initMotorOrdered(PIN_MOTOR_BR);
  newPid = axis->getPid();
  monitor = new Monitor("Monitor", Report_t::AXIS);
  monitor->setModel(&model)->begin();

  print_main_menu();
} /*------------------------- end of test_setup --------------------------------------------------*/

void test_loop()
{
  unsigned long _lastLooptime = micros();
  static unsigned long _lastMillis = millis();
  if(recorded&&(millis()-_lastMillis>100)){
    _lastMillis = millis();    
    TestOutput->print("/*");
    // Throttle
    TestOutput->print(model.axisData[axisName::primary].power);
    TestOutput->print(";");
    // Motor 1 Power
    TestOutput->print(axis->getMotorPower(false));
    TestOutput->print(";");
    // Motor 2 Power
    TestOutput->print(axis->getMotorPower(true));
    TestOutput->print(";");
    // IMU Roll
    TestOutput->print(model.sensorData.roll);
    TestOutput->print(";");
    // Error
    TestOutput->print(model.axisData[axisName::primary].pidError);
    TestOutput->print(";");
    // P
    TestOutput->print(newPid->getP());
    TestOutput->print(";");
    // I
    TestOutput->print(newPid->getI());
    TestOutput->print(";");
    // D
    TestOutput->print(newPid->getD());
    TestOutput->print(";");
    // LoopTime
    TestOutput->print(model.looptime);
    TestOutput->println("*/");
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
  model.axisData[0].setpoint = model.RC_interface.RX_payload.rcRoll;
  axis->update();
  monitor->update();
  sensor->enter();
  model.looptime = micros()-_lastLooptime;
}
/*------------------------ end of axis pri test programm ----------------------------------------*/
