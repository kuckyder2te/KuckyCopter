#pragma once

/*  File name : yaw_test.h
	Project name : KuckyCopter 2
	Author: Stephan Scholz /  Wilhelm Kuckelsberg
	Date : 2022-06-17
	Description : Drohne
*/
#include "..\src\config.h"
#include "..\lib\axisBase.h"
#include "..\lib\axisMotor.h"
#include "..\lib\axisYaw.h"
#include "..\lib\newPID.h"
#include "..\lib\monitor.h"
#include "..\lib\model.h"
#include "..\lib\sensors.h"

#define LOCAL_LOGGER
#include "..\lib\myLogger.h"

AxisMotor *axisTest[2];
Monitor *monitor;
NewPID *newPid[3];
Sensor *sensor;
Motor *motor;

extern HardwareSerial *TestOutput;
extern model_t model;

AxisYaw *axisyaw;
double rotationSpeed;  ///< Speed which the copter should turn
int16_t horz_Position; ///< Current YAW Position from Gyro

void main_gui(char key);
void print_main_menu();

void test_setup()
{
    LOGGER_NOTICE("Enter ....");
    //sensor = new Sensor("Sensor");
    //sensor->setModel(&model.sensorData)->begin();
    axisTest[axisName::primary] = new AxisMotor("Primary axismotor");
    axisTest[axisName::secondary] = new AxisMotor("Secondary axismotor");
    axisTest[axisName::primary]->setModel(&model.axisData[axisName::primary])->begin();
    axisTest[axisName::secondary]->setModel(&model.axisData[axisName::secondary])->begin();
    axisTest[axisName::primary]->initMotorOrdered(PIN_MOTOR_FL)->initMotorOrdered(PIN_MOTOR_BR);
    axisTest[axisName::secondary]->initMotorOrdered(PIN_MOTOR_FR)->initMotorOrdered(PIN_MOTOR_BL)->InvertRoll();

    axisyaw = new AxisYaw("Yaw");
    model.yaw.axisData[axisName::primary] = &model.axisData[axisName::primary];
    model.yaw.axisData[axisName::secondary] = &model.axisData[axisName::secondary];
    model.yaw.horz_Position = &horz_Position;
    model.yaw.rotationSpeed = &rotationSpeed;
    axisyaw->setModel(&model.yawData, &model.yaw)
        ->setAxisOrdered(axisTest[axisName::primary])
        ->setAxisOrdered(axisTest[axisName::secondary]);

    //   model.axisData[axisName::primary].feedback = &model.sensorData.roll; // must be before setModel because of feedback Pointer
    //   model.axisData[axisName::primary].rcX = &model.RC_interface.RX_payload.rcRoll;
    //   model.axisData[axisName::primary].rcY = &model.RC_interface.RX_payload.rcPitch;
    //   model.axisData[axisName::secondary].feedback = &model.sensorData.roll; // must be before setModel because of feedback Pointer
    //   model.axisData[axisName::secondary].rcX = &model.RC_interface.RX_payload.rcRoll;
    //   model.axisData[axisName::secondary].rcY = &model.RC_interface.RX_payload.rcPitch;

    newPid[axisName::primary] = axisTest[axisName::primary]->getPid();
    newPid[axisName::secondary] = axisTest[axisName::secondary]->getPid();
    newPid[axisName::yaw] = axisyaw->getPid();
    monitor = new Monitor("Monitor", Report_t::ALL_AXIS);
    monitor->setModel(&model)->begin();

    print_main_menu();
    LOGGER_NOTICE("....leave");
}

void test_loop()
{
    unsigned long _lastLooptime = micros();

    axisTest[axisName::primary]->update();
    axisTest[axisName::secondary]->update();
    axisyaw->update();
    monitor->update();
    //sensor->enter();
    if (TestOutput->available())
    {
        char key = TestOutput->read();
        main_gui(key);
    }

    model.looptime = micros() - _lastLooptime;
}

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
        axisyaw->setState(AxisYaw::state_e::arming_start);
        break;
    case 'S':
        TestOutput->print("isStandby: ");
        TestOutput->println(axisyaw->isStandby());
        TestOutput->print("isReady: ");
        TestOutput->println(axisyaw->isReady());
        TestOutput->print("isDeactivatePID: ");
        TestOutput->println(axisyaw->isDeactivatePID());
        TestOutput->print("isArmed: ");
        TestOutput->println(axisyaw->isArmed());
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
        axisTest[axisName::primary]->InvertRoll(); /// ??? nicht für pitch ???
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
        // print_pid_menu();
        // menu = true;
        break;
    case '?':
        print_main_menu();
        break;
    }
} /*------------------------- end of main_gui ---------------------------------------------------*/
