#pragma once

#include "..\src\config.h"
#include "..\lib\axisBase.h"
#include "..\lib\axisMotor.h"

Motor *motor[4];
void test_setup()
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
      Serial.println("Press any key to continue.");
      Serial.println("------------------------------------------------");
    }
  }
}

void test_loop()
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
/*------------------------ end of motor test programm -------------------------------------------*/
