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

	typedef struct
	{
		uint16_t power; /// power from YAW Axis
		int16_t pidError;
		int16_t setpoint;  ///< Memory for detuning the physical axis.
		int16_t *feedback; ///< Current value from the IMU
		double *rcX;	   ///< virtual axis. Corresponds to the ROLL axis.		///  zu int16_t konvertieren
		double *rcY;	   ///< virtual axis. Corresponds to the PITCH axis.
		pidData_t pidData;
	} axisData_t;

private:
	axisData_t *_axisData;
	Motor *_motor[2];
	double _roll;
	bool _invertRoll;
	motorState_e state;

public:
	AxisMotor(const String &name) : AxisBase(name)
	{
		LOGGER_VERBOSE("Enter....");
		_invertRoll = false;
		_motor[motor_t::first] = NULL;
		_motor[motor_t::second] = NULL;
		_roll = 0;
		LOGGER_VERBOSE("....leave");
	}

	virtual ~AxisMotor() {}

	AxisMotor *setModel(axisData_t *_model)
	{ // Rückgabe wert ist das eigene Objekt (this)
	  //_axis_data  wird aus dem Model in die Achse geschrieben
		LOGGER_VERBOSE("Enter....");
		_axisData = _model;
		state = standby;
		AxisBase::_sp = &_axisData->setpoint; /// _sp ist ein Pointer, der sich die Adresse des wertes aus &_axisDatat->setpoint holt
		AxisBase::_fb = _axisData->feedback;
		AxisBase::_error = &_axisData->pidError;
		LOGGER_VERBOSE("....leave");
		return this;
	}/*---------------------- setModel ------------------------------------------------*/

	AxisMotor *setMotorPinOrdered(uint8_t _pin)
	{
		LOGGER_VERBOSE("Enter....");
		LOGGER_NOTICE_FMT("PIN:%d", _pin);
		if (_motor[motor_t::first] == NULL)
		{
			LOGGER_NOTICE("Set first Motor");
			_motor[motor_t::first] = new Motor(_pin);
		}
		else if (_motor[motor_t::second] == NULL)
		{
			LOGGER_NOTICE("Set second Motor");
			_motor[motor_t::second] = new Motor(_pin);
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
		LOGGER_VERBOSE_FMT("%s", this->getName().c_str()); // Adresse von array of char
		AxisBase::begin();
		_motor[motor_t::first]->setup();
		_motor[motor_t::second]->setup();
		LOGGER_VERBOSE("....leave");
	} /*-------------------------------- end of begin ---------------------------------*/

	virtual void update() override
	{
		LOGGER_VERBOSE("Enter....");
		AxisBase::update();

		LOGGER_NOTICE_FMT("AxisMotor service AxisNo %d", _axis_address);

		switch (state)
		{
		case arming_start:
			LOGGER_NOTICE_FMT("AxisMotor arming start %d ", _axis_address);
			_motor[motor_t::first]->setMotorStates(Motor::arming);
			_motor[motor_t::second]->setMotorStates(Motor::arming);
			_motor[motor_t::first]->armingProcedure(false);
			_motor[motor_t::second]->armingProcedure(false);
			state = arming_busy;
			break;

		case arming_busy:
			LOGGER_NOTICE_FMT("AxisMotor arming_busy %s ", this->getName().c_str());
			break;

		case arming_end:
			LOGGER_NOTICE_FMT("AxisMotor arming end %s ", this->getName().c_str());
			_motor[motor_t::first]->armingProcedure(true);
			_motor[motor_t::second]->armingProcedure(true);
			_motor[motor_t::first]->setMotorStates(Motor::off);
			_motor[motor_t::second]->setMotorStates(Motor::off);
			break;

		case disablePID:
			/* Deactivate the PID controller from the motor axes. Does it have to be that way?
			 * Look at module AxisYaw */
			LOGGER_NOTICE_FMT("AxisMotor deactivate PID %d ", _axis_address);
			AxisBase::_newPID->disablePID();
			break;

		case enablePID:
			/* Activate the PID controller from the MotorAxis with the current coefficients. */
			LOGGER_NOTICE_FMT("AxisMotor activate PID %d ", _axis_address);
			AxisBase::_newPID->enablePID();
			break;

		case standby:
			LOGGER_NOTICE_FMT("AxisMotor standby %d ", _axis_address);
			_motor[motor_t::first]->setMotorStates(Motor::off);
			_motor[motor_t::second]->setMotorStates(Motor::off);
			break;

		case ready:
			LOGGER_NOTICE_FMT("AxisMotor ready %d ", _axis_address);
			_motor[motor_t::first]->setMotorStates(Motor::on);
			_motor[motor_t::second]->setMotorStates(Motor::on);

			_roll = (_invertRoll ? (-(*_axisData->rcX)) : (*_axisData->rcX));

			_axisData->setpoint = (_roll + (*_axisData->rcY));

			_motor[motor_t::first]->setPower(_axisData->power - _axisData->pidError);
			_motor[motor_t::second]->setPower(_axisData->power + _axisData->pidError);
			LOGGER_NOTICE_FMT("AxisMotor SP:%d, Power:%d, Error:%d, Axis:%d", _axisData->setpoint, _axisData->power, _axisData->pidError, _axis_address);
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
		state = state;
#ifdef AXISMOTOR_STATE
		LOG("set AxisMotor State = %d", _state);
		delay(DEBUG_DWELL_TIME);
#endif

	} /*--------------------- end of setState -----------------------------------------*/

	boolean isArmed() const
	{

		return ((_motor[motor_t::first]->isMotorOff()) && (_motor[motor_t::second]->isMotorOff()));

	} /*---------------------- end of isArmed -----------------------------------------*/
	
	boolean isDeactivatePID(){	
			LOGGER_VERBOSE("Enter....");
			return(state == disablePID);
	} /*--------------------- end of isDeactivatePID ----------------------------------*/

	boolean isStandby() const {
			LOGGER_VERBOSE("Enter....");
			return ( state == standby);
	} /*--------------------- end of isStandby ----------------------------------------*/

	boolean isReady() const {
			LOGGER_VERBOSE("Enter....");
			return ( state == ready);
	} /*---------------------- end of isReady -----------------------------------------*/

		// Motor** getMotor() {
		// 	return motor;

		// } /*---------------------- end of getMotor ---------------------------------*/
	
}; /*.................................. end of AxisMotor class ------------------------*/
