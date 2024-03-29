#pragma once
/*  File name : axisMotor.h
	Project name : KuckyCopter 2
	Authors: Stephan Scholz / Wilhelm Kuckelsberg
	Date : 2022-06-13
	Description : Drohne
*/

#include <Arduino.h>

#include "axisBase.h"
#include "motor.h"

//#define LOCAL_DEBUG
#include "myLogger.h"

class AxisMotor : public AxisBase
{
public:
	typedef enum
	{
		first,
		second
	} motor_t;
	
private:
	Motor *_motor[2];
	double _roll;
	bool _invertRoll;
	int16_t _lastPower;

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

	AxisMotor *setModel(axisData_t *_model)
	{ // Your own object is worth returning (this)
	  //_axis_data is written to the axis from the model
		LOGGER_NOTICE("Enter....");
		_axisData = _model;
		_state = standby;
		AxisBase::_sp = &_axisData->setpoint; 	// _sp is a pointer that gets the address  
												// of the value from &_axisData->setpoint
		AxisBase::_fb = _axisData->feedback;
		AxisBase::_error = &_axisData->pidError;
		LOGGER_NOTICE("....leave");
		return this;
	} /*---------------------- setModel ---------------------------------------------------------*/

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
	} /*---------------------- setMotorPinOrdered ----------------------------------------------*/

	AxisMotor *InvertRoll()
	{
		_invertRoll = true;
		return this;
	} /*---------------------- InvertRoll ------------------------------------------------------*/

	virtual void begin() override
	{
		LOGGER_NOTICE_FMT("Enter %s",this->getName().c_str());
		LOGGER_NOTICE_FMT("%s", this->getName().c_str()); // Address from array of char
		AxisBase::begin();
				//..and other configurations
		LOGGER_VERBOSE("....leave");
	} /*-------------------------------- end of begin ------------------------------------------*/

	virtual void update() override
	{
		LOGGER_NOTICE_FMT_CHK(_state,_lastState,"State Changed -> %d",_state);
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
			_motor[motor_t::first]->setMotorState(Motor::stop);
			_motor[motor_t::second]->setMotorState(Motor::stop);
			break;
		case disablePID:
			/* Deactivate the PID controller from the motor axes. Does it have to be that way?
			 * Look at module AxisYaw */
			LOGGER_NOTICE_FMT_CHK(_state,_lastState,"deactivate PID %s ", this->getName().c_str());
			_newPID->disablePID();
			break;
		case enablePID:
			/* Activate the PID controller from the MotorAxis with the current coefficients. */
			LOGGER_NOTICE_FMT_CHK(_state,_lastState,"activate PID %s ", this->getName().c_str());
			_newPID->clear();
			_newPID->enablePID();
			_state = ready;
			break;
		case standby:
			LOGGER_NOTICE_FMT_CHK(_state,_lastState,"standby %s ", this->getName().c_str());
			_motor[motor_t::first]->setMotorState(Motor::stop);
			_motor[motor_t::second]->setMotorState(Motor::stop);
			break;
		case ready:
			_motor[motor_t::first]->setMotorState(Motor::rotating);
			_motor[motor_t::second]->setMotorState(Motor::rotating);

			_roll = (_invertRoll ? (-(*_axisData->rcX)) : (*_axisData->rcX));

			_axisData->setpoint = (_roll + (*_axisData->rcY));

			// _motor[motor_t::first]->setPower(0);		// temp_debug
			// _motor[motor_t::second]->setPower(0);
			_motor[motor_t::first]->setPower(_axisData->power - _axisData->pidError);
			_motor[motor_t::second]->setPower(_axisData->power + _axisData->pidError);
				
			LOGGER_NOTICE_FMT_CHK(_state,_lastState,"ready %s, AxisMotor SP:%d, Power:%d, Error:%d", this->getName().c_str(), _axisData->setpoint, _axisData->power, _axisData->pidError);
			break;
		case off:
			LOGGER_NOTICE_FMT_CHK(_state,_lastState,"off %s ", this->getName().c_str());
			_axisData->setpoint = 0;
			_motor[motor_t::first]->setMotorState(Motor::power_off);
			_motor[motor_t::second]->setMotorState(Motor::power_off);
			break;
		} /* end of switch */
	
	} /*..................... end of update ----------------------------------------------------*/

	void setState(state_t state)
	{
		_state = state;
		LOGGER_NOTICE_FMT_CHK(_state,_lastState,"axisMotor state %i", _state);
	} /*--------------------- end of setState --------------------------------------------------*/

	void setPower(int16_t _power)
	{
		LOGGER_NOTICE_FMT_CHK(_power,_lastPower,"set %s to Power = %d",this->getName().c_str(), _power);
		if (_power < 0)
		{
			_axisData->power = 0;
		}
		else if (_power > POWER_MAX)
		{
			_axisData->power = POWER_MAX;
			LOGGER_WARNING_FMT("Power more the 100 %i",_power);
		}
		else {	
			_axisData->power = _power;
		}
	} /*--------------------- end of setPower ---------------------------------------------------*/

	int16_t getPower(){
		return _axisData->power;
	} /*--------------------- end of getPower ---------------------------------------------------*/

	uint32_t getMotorPower(bool motor){
		if(motor==false){
			return _motor[motor_t::first]->getPower();
		}else{
			return _motor[motor_t::second]->getPower();
		}
	} /*--------------------- end of getMotorPower ----------------------------------------------*/

	virtual boolean isArmed()
	{
		LOGGER_VERBOSE("Enter....isArmed");
		return ((_motor[motor_t::first]->isArmed()) && (_motor[motor_t::second]->isArmed()));
	} /*---------------------- end of isArmed ---------------------------------------------------*/
}; /*.................................. end of axisMotor class ----------------------------------*/
