#pragma once

#include "..\src\config.h"
#include "..\lib\axisBase.h"
#include "..\lib\axisMotor.h"
#include "..\lib\monitor.h"
#include "..\lib\model.h"
#include "..\lib\newPID.h"

AxisMotor *axis;
Monitor *monitor;
NewPID *newPid;

extern model_t model;

bool menu = false;
void print_pid_menu();

void print_main_menu()
{
  Serial.println("----------- Axis Test setup menu -------------------");
  Serial.println("A for arming");
  Serial.println("S for current State");
  Serial.println("Key (+) or (-) increment or decrement Power.");
  Serial.println("I for invertRoll");
  Serial.println("D disable PID");
  Serial.println("E enable PID");
  Serial.println("St(o)p Motor");
  Serial.println("R for ready (Start Motors)");
  Serial.println("P for PID Menu");
  Serial.println("? for this Menu");
  Serial.println("------------------------------------------------");
}

void main_gui(char key)
{
  static int16_t power = 0;
  switch (key)
  {
  case 'A':
  case 'a':
    axis->setState(AxisMotor::motorState_e::arming_start);
    break;
  case 'S':
  case 's':
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
  case 'i':
    Serial.println("Invert Roll");
    axis->InvertRoll();
    break;
  case 'O':
  case 'o':
    Serial.println("Stop Motor");
    axis->setState(AxisMotor::motorState_e::standby);
    break;
  case 'R':
  case 'r':
    Serial.println("Motor Start");
    axis->setState(AxisMotor::motorState_e::ready);
    break;
  case 'P':
  case 'p':
    print_pid_menu();
    menu = true;
    break;
  case '?':
    print_main_menu();
    break;
  }
}

void pid_gui(char key)
{
  switch(key){
    case 'M':
    case 'm':
      print_main_menu();
      menu = false;
      break;
    case '?':
      print_pid_menu();
      break;
  }
}

void print_pid_menu()
{
  Serial.println("----------- Axis Test PID Menu -------------------");
  
  Serial.println("M for Main menu");
  Serial.println("? for this Menu");
  Serial.println("------------------------------------------------");
}


void test_setup()
{
  delay(5000); // for switching terminal on
  LOGGER_VERBOSE("Enter....");
  axis = new AxisMotor("axismotor");
  axis->setModel(&model.axisData[0])->begin();
  axis->initMotorOrdered(PIN_MOTOR_FL)->initMotorOrdered(PIN_MOTOR_BR);
  newPid = axis->getPid();
  newPid->initPID();
  newPid->setI(0);
  monitor = new Monitor("Monitor", Report_t::AXIS);
  monitor->setModel(&model)->begin();
  print_main_menu();
}

void test_loop()
{
  if (Serial.available())
  {
    char key = Serial.read();
    switch (menu)
    {
      case false: main_gui(key);
        break;
      case true: pid_gui(key);
        break;
    }
  }
  axis->update();
  monitor->update();
}
/*------------------------ end of axis test programm -------------------------------------------*/
