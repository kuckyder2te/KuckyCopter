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

AxisMotor *axis1;
AxisMotor *axis2;
Monitor *monitor;
NewPID *newPid1;
NewPID *newPid2;
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
    axis1->setState(AxisMotor::state::arming_start);
    axis2->setState(AxisMotor::state::arming_start);
    break;
  case 'S':
    TestOutput->print("isStandby: ");
    TestOutput->println(axis1->isStandby());
    TestOutput->print("isReady: ");
    TestOutput->println(axis1->isReady());
    TestOutput->print("isDeactivatePID: ");
    TestOutput->println(axis1->isDeactivatePID());
    TestOutput->print("isArmed: ");
    TestOutput->println(axis1->isArmed());
    break;
  case 'G':
    TestOutput->println(motor->getResultingPower());
    break;
    
  case '+':
    power++;
    TestOutput->print("Power: ");
    TestOutput->println(power);
    axis1->setPower(power);
    axis2->setPower(power);
    break;
  case '-':
    if (power > 0)
      power--;
    TestOutput->print("Power: ");
    TestOutput->println(power);
    axis1->setPower(power);
    axis2->setPower(power);
    break;
  case 'I':
    TestOutput->println("Invert Roll");
    axis1->InvertRoll();
    break;
  case 'O':
    TestOutput->println("Stop Motor");
    axis1->setState(AxisMotor::state::standby);
    axis2->setState(AxisMotor::state::standby);
    break;
  case 'R':
    TestOutput->println("Motor Start");
    axis1->setState(AxisMotor::state::ready);
    axis2->setState(AxisMotor::state::ready);
    break;
  case ' ':
    axis1->setState(AxisMotor::state::off);
    axis2->setState(AxisMotor::state::off);
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
    TestOutput->println(newPid1->getP(), 2);
    TestOutput->print("I: ");
    TestOutput->println(newPid1->getI(), 2);
    TestOutput->print("D: ");
    TestOutput->println(newPid1->getD(), 2);
    TestOutput->print("EF: ");
    TestOutput->println(newPid1->getEF());
    TestOutput->print("ExecutionTime:");
    TestOutput->println(newPid1->getExecutionTime());

    TestOutput->print("P: ");
    TestOutput->println(newPid2->getP(), 2);
    TestOutput->print("I: ");
    TestOutput->println(newPid2->getI(), 2);
    TestOutput->print("D: ");
    TestOutput->println(newPid2->getD(), 2);
    TestOutput->print("EF: ");
    TestOutput->println(newPid2->getEF());
    TestOutput->print("ExecutionTime:");
    TestOutput->println(newPid2->getExecutionTime());
    break;
  case 'R':
    TestOutput->println("Reset");
    newPid1->initPID();
    newPid2->initPID();
    break;
  case 'O':
    TestOutput->print("Throttle is selected. Power= ");TestOutput->print(axis1->getPower());
    _pidParameter = 5;
    break;  
  case 'P':
    TestOutput->print("Parameter P is selected. P= ");TestOutput->print(newPid1->getP());
    _pidParameter = 1;
    break;
  case 'I':
    TestOutput->println("Parameter I is selected. I= ");TestOutput->print(newPid1->getI());
    _pidParameter = 2;
    break;
  case 'D':
    TestOutput->println("Parameter D is selected. D= ");TestOutput->print(newPid1->getD());
    _pidParameter = 3;
    break;
  case 'F':
    TestOutput->println("Parameter eF is selected");
    _pidParameter = 4;
    break;
  case 'E':
    TestOutput->println("Enable PID");
    axis1->setState(AxisMotor::state::enablePID);
    axis2->setState(AxisMotor::state::enablePID);
    break;
  case 'A':
    TestOutput->println("Disable PID");
    axis1->setState(AxisMotor::state::disablePID);
    axis2->setState(AxisMotor::state::disablePID);
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
      temp = newPid1->getP();
      newPid1->setP(temp += DOT_2);
      newPid2->setP(temp);
      TestOutput->print("kP: ");
      TestOutput->println(temp, 4);
      break;
    case 2:
      temp = newPid1->getI();
      newPid1->setI(temp += DOT_2);
      newPid2->setI(temp);
      TestOutput->print("kI: ");
      TestOutput->println(temp, 4);
      break;
    case 3:
      temp = newPid1->getD();
      newPid1->setD(temp += DOT_2);
      newPid2->setD(temp);
      TestOutput->print("kD: ");
      TestOutput->println(temp, 4);
      break;
    case 4:
      temp = newPid1->getEF();
      newPid1->setEF(temp += 1);
      newPid2->setEF(temp);
      TestOutput->print("eF: ");
      TestOutput->println(temp, 4);
      break;
    case 5:
      temp = axis1->getPower();
      axis1->setPower(++temp);
      axis2->setPower(temp);
      TestOutput->print("Throttle = ");
      TestOutput->println(temp);
      break;
    }     // end of switch
    newPid1->saveParameters();
    newPid2->saveParameters();
    break;

  case '-':
    switch (_pidParameter)
    {
    case 1:
      temp = newPid1->getP();
      newPid1->setP(temp -= DOT_2);
      newPid2->setP(temp);
      TestOutput->print("kP: ");
      TestOutput->println(temp, 4);
      break;
    case 2:
      temp = newPid1->getI();
      newPid1->setI(temp -= DOT_2);
      newPid2->setI(temp);
      TestOutput->print("kI: ");
      TestOutput->println(temp, 4);
      break;
    case 3:
      temp = newPid1->getD();
      newPid1->setD(temp -= DOT_2);
      newPid2->setD(temp);
      TestOutput->print("kD: ");
      TestOutput->println(temp, 4);
      break;
    case 4:
      temp = newPid1->getEF();
      newPid1->setEF(temp -= 1);
      newPid2->setEF(temp);
      TestOutput->print("eF: ");
      TestOutput->println(temp, 4);
      break;
    case 5:
      temp = axis1->getPower();
      axis1->setPower(--temp);
      axis2->setPower(temp);
      TestOutput->print("Throttle = ");
      TestOutput->println(temp);
      break;
    }
    break;

  case ' ':
    axis1->setState(AxisMotor::state::off);
    axis2->setState(AxisMotor::state::off);
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
  axis1 = new AxisMotor("Primary axismotor");
  axis2 = new AxisMotor("Secondary axismotor");
  model.axisData[axisName::primary].feedback = &model.sensorData.roll; // must be before setModel because of feedback Pointer
  model.axisData[axisName::primary].rcX = &model.RC_interface.RX_payload.rcRoll;
  model.axisData[axisName::primary].rcY = &model.RC_interface.RX_payload.rcPitch;
  model.axisData[axisName::secondary].feedback = &model.sensorData.roll; // must be before setModel because of feedback Pointer
  model.axisData[axisName::secondary].rcX = &model.RC_interface.RX_payload.rcRoll;
  model.axisData[axisName::secondary].rcY = &model.RC_interface.RX_payload.rcPitch;
  axis1->setModel(&model.axisData[axisName::primary])->begin();
  axis2->setModel(&model.axisData[axisName::secondary])->begin();
  axis1->initMotorOrdered(PIN_MOTOR_FL)->initMotorOrdered(PIN_MOTOR_BR);
  axis2->initMotorOrdered(PIN_MOTOR_FR)->initMotorOrdered(PIN_MOTOR_BL);
  newPid1 = axis1->getPid();
  newPid2 = axis2->getPid();
  monitor = new Monitor("Monitor", Report_t::ALL_AXIS);
  monitor->setModel(&model)->begin();

  print_main_menu();
} /*------------------------- end of test_setup --------------------------------------------------*/

void test_loop()
{
  //TestOutput->println("loop");
  unsigned long _lastLooptime = micros();
  static unsigned long _lastMillis = millis();
  if(recorded&&(millis()-_lastMillis>100)){
    _lastMillis = millis();    
    TestOutput->printf("/*%i,%i,%i,%i,%i,%.2f,%.2f,%.2f,%i,%i*/\r\n",
                    model.axisData[axisName::primary].power,
                    axis1->getMotorPower(false),
                    axis1->getMotorPower(true),
                    model.sensorData.roll,
                    model.axisData[axisName::primary].pidError,
                    newPid1->getP(),
                    newPid1->getI(),
                    newPid1->getD(),
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
  model.axisData[0].setpoint = model.RC_interface.RX_payload.rcRoll;
  model.axisData[1].setpoint = model.RC_interface.RX_payload.rcRoll;
  axis1->update();
  axis2->update();
  monitor->update();
  sensor->enter();
  model.looptime = micros()-_lastLooptime;
}
/*------------------------ end of axis pri test programm ----------------------------------------*/
