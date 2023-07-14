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

#define LOCAL_DEBUG
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
	motorState_e _state, _lastState;
	uint16_t _lastPower;

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
		begin();
		LOGGER_NOTICE("....leave");
		return this;
	} /*---------------------- setModel ------------------------------------------------*/

	AxisMotor *initMotorOrdered(uint8_t _pin)
	{
		LOGGER_NOTICE_FMT("Enter %s",this->getName().c_str());
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
		LOGGER_NOTICE_FMT("....leave %s",this->getName().c_str());
		return this;
	} /*---------------------- setMotorPinOrdered -------------------------------------*/

	AxisMotor *InvertRoll()
	{
		_invertRoll = true;
		return this;
	} /*---------------------- InvertRoll ---------------------------------------------*/

	virtual void begin() override
	{
		LOGGER_NOTICE_FMT("Enter %s",this->getName().c_str());
		LOGGER_NOTICE_FMT("%s", this->getName().c_str()); // Adresse von array of char
		AxisBase::begin();
		//..und weitere Configgeschichten
		LOGGER_VERBOSE("....leave");
	} /*-------------------------------- end of begin ---------------------------------*/

	virtual void update() override
	{
		LOGGER_VERBOSE("Enter....");
		AxisBase::update();

		LOGGER_VERBOSE_FMT("Update %s ", this->getName().c_str());

		_motor[motor_t::first]->update();
		_motor[motor_t::second]->update();	

		switch (_state)
		{
		case arming_start:
			LOGGER_NOTICE_FMT("arming start %s ", this->getName().c_str());
			_motor[motor_t::first]->setMotorState(Motor::arming);
			_motor[motor_t::second]->setMotorState(Motor::arming);
			_state = arming_busy;
			break;

		case arming_busy:
			LOGGER_NOTICE_FMT_CHK(_state,_lastState,"arming_busy %s ", this->getName().c_str());
			if((_motor[motor_t::first]->isArmed()) && 
				(_motor[motor_t::second]->isArmed()))
				{
					_state = arming_end;
				}
			break;

		case arming_end:
			LOGGER_NOTICE_FMT_CHK(_state,_lastState,"arming end %s ", this->getName().c_str());
			_motor[motor_t::first]->setMotorState(Motor::off);
			_motor[motor_t::second]->setMotorState(Motor::off);
			break;

		case disablePID:
			/* Deactivate the PID controller from the motor axes. Does it have to be that way?
			 * Look at module AxisYaw */
			LOGGER_NOTICE_FMT_CHK(_state,_lastState,"deactivate PID %s ", this->getName().c_str());
			AxisBase::_newPID->disablePID();
			break;

		case enablePID:
			/* Activate the PID controller from the MotorAxis with the current coefficients. */
			LOGGER_NOTICE_FMT_CHK(_state,_lastState,"activate PID %s ", this->getName().c_str());
			AxisBase::_newPID->enablePID();
			break;

		case standby:
			LOGGER_NOTICE_FMT_CHK(_state,_lastState,"standby %s ", this->getName().c_str());
			_motor[motor_t::first]->setMotorState(Motor::off);
			_motor[motor_t::second]->setMotorState(Motor::off);
			break;

		case ready:
			LOGGER_NOTICE_FMT_CHK(_state,_lastState,"ready %s ", this->getName().c_str());
			_motor[motor_t::first]->setMotorState(Motor::on);
			_motor[motor_t::second]->setMotorState(Motor::on);

			_roll = (_invertRoll ? (-(*_axisData->rcX)) : (*_axisData->rcX));

			_axisData->setpoint = (_roll + (*_axisData->rcY));

			_motor[motor_t::first]->setPower(_axisData->power - _axisData->pidError);
			_motor[motor_t::second]->setPower(_axisData->power + _axisData->pidError);
			LOGGER_NOTICE_FMT_CHK(_state,_lastState,"AxisMotor SP:%d, Power:%d, Error:%d", _axisData->setpoint, _axisData->power, _axisData->pidError);
			break;
		} /* end of switch */
	
	} /*..................... end of update -------------------------------------------*/

	void setState(motorState_e state)
	{
		//if(state!=_state)
		//	LOGGER_NOTICE_FMT("set AxisMotor State = %d", _state);		// Does not work. Somehow Update() is called
		_state = state;
	} /*--------------------- end of setState -----------------------------------------*/

	void setPower(int16_t _power)
	{
		LOGGER_NOTICE_FMT_CHK(_power,_lastPower,"set AxisMotor %s to Power = %d",this->getName().c_str(), _power);
		if (_power < 0)
			_axisData->power = 0;
		else
			_axisData->power = _power;
	} /*--------------------- end of setPower -----------------------------------------*/

	boolean isArmed() const
	{
		LOGGER_VERBOSE("Enter....isArmed");
		return ((_motor[motor_t::first]->isArmed()) && (_motor[motor_t::second]->isArmed()));
	} /*---------------------- end of isArmed -----------------------------------------*/

	boolean isDeactivatePID()
	{
		LOGGER_NOTICE_FMT_CHK(_state,_lastState,"Enter....isDeactivatePID %s ", this->getName().c_str());
		return (_state == disablePID);
	} /*--------------------- end of isDeactivatePID ----------------------------------*/

	boolean isStandby()
	{
		LOGGER_NOTICE_FMT_CHK(_state,_lastState,"Enter....isStandby %s ", this->getName().c_str());
		return (_state == standby);
	} /*--------------------- end of isStandby ----------------------------------------*/

	boolean isReady()
	{
		LOGGER_NOTICE_FMT_CHK(_state,_lastState,"Enter....isReady %s ", this->getName().c_str());
		return (_state == ready);
	} /*---------------------- end of isReady -----------------------------------------*/
}; /*.................................. end of axisMotor class ------------------------*/
