#pragma once
/*  File name : axisMotor.h
	Project name : KuCo_Phantom 1
	Author: Wilhelm Kuckelsberg
	Date : 2022-06-13
	Description : Drohne
*/

#include <Arduino.h>
#include "axisBase.h"
#include "motor.h"
#include "myLogger.h"

class AxisMotor : public AxisBase
{

public:
	typedef enum
	{
		first,
		second
	} motor_t;
	typedef enum
	{
		arming_start = 0,
		arming_busy,
		arming_end,
		disablePID,
		enablePID,
		standby,
		ready
	} motorState_e;
	
private:
	Motor *_motor[2];
	double _roll;
	bool _invertRoll;
	motorState_e _state;

public:
	AxisMotor(const String &name) : AxisBase(name)
	{
		LOGGER_NOTICE("Enter....");
		_invertRoll = false;
		_motor[motor_t::first] = NULL;
		_motor[motor_t::second] = NULL;
		_roll = 0;
		LOGGER_NOTICE("....leave");
	}

	virtual ~AxisMotor() {}

	AxisMotor *setModel(axisData_t *_model)
	{ // Rückgabe wert ist das eigene Objekt (this)
	  //_axis_data  wird aus dem Model in die Achse geschrieben
		LOGGER_NOTICE("Enter....");
		_axisData = _model;
		_state = standby;
		AxisBase::_sp = &_axisData->setpoint; /// _sp ist ein Pointer, der sich die Adresse des wertes aus &_axisDatat->setpoint holt
		AxisBase::_fb = _axisData->feedback;
		AxisBase::_error = &_axisData->pidError;
		loadPIDConfig();

		LOGGER_NOTICE("....leave");
		return this;
	} /*---------------------- setModel ------------------------------------------------*/

	AxisMotor *initMotorOrdered(uint8_t _pin)
	{
		LOGGER_VERBOSE("Enter....");
		LOGGER_NOTICE_FMT("Pin = %d", _pin);
		if (_motor[motor_t::first] == NULL)
		{
			LOGGER_NOTICE("Set first Motor");
			_motor[motor_t::first] = new Motor(_pin);
			_motor[motor_t::first]->setup();
		}
		else if (_motor[motor_t::second] == NULL)
		{
			LOGGER_NOTICE("Set second Motor");
			_motor[motor_t::second] = new Motor(_pin);
			_motor[motor_t::second]->setup();
		}
		else
		{
			LOGGER_FATAL("Too much Motors initialized!!");
		}
		LOGGER_VERBOSE("....leave");
		return this;
	} /*---------------------- setMotorPinOrdered -------------------------------------*/

	AxisMotor *InvertRoll()
	{
		_invertRoll = true;
		return this;
	} /*---------------------- InvertRoll ---------------------------------------------*/

	virtual void begin() override
	{
		LOGGER_VERBOSE("Enter....");
		LOGGER_NOTICE_FMT("%s", this->getName().c_str()); // Adresse von array of char
		AxisBase::begin();
	//	_axis_address++;
		LOGGER_VERBOSE("....leave");
	} /*-------------------------------- end of begin ---------------------------------*/

	virtual void update() override
	{
		LOGGER_VERBOSE("Enter....");
		AxisBase::update();

		LOGGER_NOTICE_FMT("Update %s ", this->getName().c_str());

		switch (_state)
		{
		case arming_start:
			LOGGER_VERBOSE("arming start");
			_motor[motor_t::first]->setMotorStates(Motor::arming);
			_motor[motor_t::second]->setMotorStates(Motor::arming);
			_motor[motor_t::first]->armingProcedure(false);
			_motor[motor_t::second]->armingProcedure(false);
			_state = arming_busy;
			break;

		case arming_busy:
			LOGGER_VERBOSE("arming_busy");
			break;

		case arming_end:
			LOGGER_VERBOSE("arming end");
			_motor[motor_t::first]->armingProcedure(true);
			_motor[motor_t::second]->armingProcedure(true);
			_motor[motor_t::first]->setMotorStates(Motor::off);
			_motor[motor_t::second]->setMotorStates(Motor::off);
			break;

		case disablePID:
			/* Deactivate the PID controller from the motor axes. Does it have to be that way?
			 * Look at module AxisYaw */
			LOGGER_VERBOSE("deactivate PID");
			AxisBase::_newPID->disablePID();
			break;

		case enablePID:
			/* Activate the PID controller from the MotorAxis with the current coefficients. */
			LOGGER_VERBOSE("activate PID");
			AxisBase::_newPID->enablePID();
			break;

		case standby:
			LOGGER_VERBOSE("standby");
			_motor[motor_t::first]->setMotorStates(Motor::off);
			_motor[motor_t::second]->setMotorStates(Motor::off);
			break;

		case ready:
			LOGGER_VERBOSE("ready");
			_motor[motor_t::first]->setMotorStates(Motor::on);
			_motor[motor_t::second]->setMotorStates(Motor::on);

			_roll = (_invertRoll ? (-(*_axisData->rcX)) : (*_axisData->rcX));

			_axisData->setpoint = (_roll + (*_axisData->rcY));

			_motor[motor_t::first]->setPower(_axisData->power - _axisData->pidError);
			_motor[motor_t::second]->setPower(_axisData->power + _axisData->pidError);
			LOGGER_NOTICE_FMT("AxisMotor SP:%d, Power:%d, Error:%d", _axisData->setpoint, _axisData->power, _axisData->pidError);
			break;
		} /* end of switch */

		_motor[motor_t::first]->updateState();
		_motor[motor_t::second]->updateState();
	} /*..................... end of update -------------------------------------------*/

	void setPower(int16_t _power)
	{
		if (_power < 0)
			_axisData->power = 0;
		else
			_axisData->power = _power;
	} /*--------------------- end of setPower -----------------------------------------*/

	void setState(motorState_e state)
	{
		_state = state;
		LOGGER_NOTICE_FMT("set AxisMotor State = %d", _state);
	} /*--------------------- end of setState -----------------------------------------*/

	boolean isArmed() const
	{
		LOGGER_VERBOSE("Enter....isArmed");
		return ((_motor[motor_t::first]->isMotorOff()) && (_motor[motor_t::second]->isMotorOff()));
	} /*---------------------- end of isArmed -----------------------------------------*/

	boolean isDeactivatePID()
	{
		LOGGER_VERBOSE("Enter....isDeactivatePID");
		return (_state == disablePID);
	} /*--------------------- end of isDeactivatePID ----------------------------------*/

	boolean isStandby() const
	{
		LOGGER_VERBOSE("Enter....isStandby");
		return (_state == standby);
	} /*--------------------- end of isStandby ----------------------------------------*/

	boolean isReady() const
	{
		LOGGER_VERBOSE("Enter....isReady");
		return (_state == ready);
	} /*---------------------- end of isReady -----------------------------------------*/

	// Motor** getMotor() {
	//  	return _motor;

	// } /*---------------------- end of getMotor -------------------------------------*/

}; /*.................................. end of AxisMotor class ------------------------*/

// #undef _DEBUG_