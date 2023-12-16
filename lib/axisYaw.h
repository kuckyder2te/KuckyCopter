#pragma once
/*  File name : axisYaw.h
	Project name : KuckyCopter 2
	Authors: Stephan Scholz /  Wilhelm Kuckelsberg
	Date : 2022-06-17
	Description : Drohne
*/

#include "AxisBase.h"
#include "AxisMotor.h"

//#define LOCAL_DEBUG
#include "myLogger.h"

#define YAW_SENSIBILITY 5
#define YAW_FINE_TUNING 0.1

typedef struct
{
	double rotationSpeed;	///< Speed which the copter should turn
	int16_t horz_Position; ///< Current YAW Position from Gyro
	AxisMotor::axisData_t *axisData[2];
} yaw_t;

class AxisYaw : public AxisBase
{

public:
	typedef enum
	{
		arming_start,
		arming_finished,
		disablePID,
		enablePID,
		ready
	} state_e;

private:
	AxisMotor *_axisMotor[2];
	yaw_t *_yaw;
	state_e _state, _lastState;
	int16_t _lastCompass;
	int16_t _virtualFeedback; ///< The calculated feedback due to the yaw rotation
	int16_t _virtualSetpoint;

public:
	AxisYaw(const String &name) : AxisBase(name)
	{											 
		_virtualSetpoint = 0;	// Setpoint is always 0 as this corresponds 
								// to the current position used in the middle position
		AxisBase::_sp = &_virtualSetpoint;		 
		AxisBase::_fb = &_virtualFeedback;		 
		AxisBase::_error = &_axisData->pidError; 

		_axisMotor[axisName::primary] = NULL;
		_axisMotor[axisName::secondary] = NULL;
		_state = disablePID;
		_lastCompass = 0;
	};

	AxisYaw *setModel(axisData_t *_model, yaw_t *yaw)
	{
		LOGGER_VERBOSE("Enter....");
		_axisData = _model;
		_yaw = yaw;
		begin();
		LOGGER_VERBOSE("....leave");
		return this;
	} /*------------------------------- end of setModel ----------------------------------------*/

	AxisYaw *setAxisOrdered(AxisMotor *_axis) // *_axis is the address
	{
		LOGGER_VERBOSE("Enter....");
		if (_axisMotor[axisName::primary] == NULL)
		{
			LOGGER_NOTICE_FMT("Set adress first Axis %p", _axis->getName().c_str());
			_axisMotor[axisName::primary] = _axis;
		}
		else if (_axisMotor[axisName::secondary] == NULL)
		{
			LOGGER_NOTICE_FMT("Set adress second Axis %p", _axis->getName().c_str());
			_axisMotor[axisName::secondary] = _axis;
		}
		else
		{
			LOGGER_FATAL("Too much Axis initialized!!");
		}
		LOGGER_VERBOSE("....leave");
		return this;
	} /*------------------------------- end of setAxisOrdered ----------------------------------*/

	virtual void begin()
	{
		AxisBase::begin();
		digitalWrite(PIN_ESC_ON, HIGH);
	} /*------------------------------- end of virtual begin -----------------------------------*/

	virtual void update()
	{
		AxisBase::update();

		LOGGER_NOTICE("Update axisYaw");

		switch (_state)
		{
		case arming_start:
			LOGGER_NOTICE_FMT_CHK(_state, _lastState, "Enter arming start %d", _state);
			_axisMotor[axisName::primary]->setState(AxisMotor::state::arming_start);
			_axisMotor[axisName::secondary]->setState(AxisMotor::state::arming_start);
			_state = arming_finished;
			LOGGER_VERBOSE("....leave arming_start");
			break;

		case arming_finished:
			LOGGER_NOTICE_FMT_CHK(_state, _lastState, "Enter arming finished %d", _state);
			if (_axisMotor[axisName::primary]->isArmed() && _axisMotor[axisName::secondary]->isArmed())
			{
				LOGGER_NOTICE("All motors armed");
				_state = ready;
			}
			LOGGER_VERBOSE("....leave");
			break;

		case disablePID:
			/* Disables the YawAxis PID controller and initiates deactivation for the motor axes. */
			LOGGER_NOTICE_FMT_CHK(_state, _lastState, "Enter disablePID state %d", _state);
			_newPID->disablePID();
			_axisMotor[axisName::primary]->setState(AxisMotor::disablePID);
			_axisMotor[axisName::secondary]->setState(AxisMotor::disablePID);
			LOGGER_VERBOSE("....leave");
			break;

		case enablePID:
			/* Enables the YawAxis PID controller and initiates activation for the motor axes. */
			LOGGER_NOTICE_FMT_CHK(_state, _lastState, "Enter enablePID state %d", _state);
			_newPID->enablePID();
			_yaw->horz_Position = 0;
			_axisMotor[axisName::primary]->setState(AxisMotor::enablePID);
			_axisMotor[axisName::secondary]->setState(AxisMotor::enablePID);
			_lastCompass = *_axisData->feedback; ///< Becomes necessary, so that after the start the Copter does not turn.
			LOGGER_VERBOSE("....leave");
			break;

		case ready:
			//LOGGER_NOTICE_FMT_CHK(_state, _lastState, "Enter ready state %d", _state);
			LOGGER_NOTICE_FMT("Enter ready state %d", _state);
			_axisMotor[axisName::primary]->setState(AxisMotor::ready);
			_axisMotor[axisName::secondary]->setState(AxisMotor::ready);

			// _axisData->power = 10;		// Test
			// *_yaw->rotationSpeed = 100; // Test

			if ((_yaw->rotationSpeed > YAW_SENSIBILITY) || (_yaw->rotationSpeed < (-YAW_SENSIBILITY)))
			{ ///< YAW Joystick is not moved....
				// _yaw->axisData[axisName::primary]->power = _axisData->power - _yaw->rotationSpeed * YAW_FINE_TUNING;
				// _yaw->axisData[axisName::secondary]->power = _axisData->power + _yaw->rotationSpeed * YAW_FINE_TUNING;
				_axisMotor[axisName::primary]->setPower(_axisData->power - _yaw->rotationSpeed * YAW_FINE_TUNING);
				_axisMotor[axisName::secondary]->setPower(_axisData->power + _yaw->rotationSpeed * YAW_FINE_TUNING);
				_yaw->horz_Position = 0;
			}
			else
			{											 ///< YAW  PID controller is active  .... Yaw joystick is in middle position
				_virtualFeedback = _yaw->horz_Position; /// *_fb = into the PID controller
				// _yaw->axisData[axisName::primary]->power = _axisData->power - _axisData->pidError;
				// _yaw->axisData[axisName::secondary]->power = _axisData->power + _axisData->pidError;
				_axisMotor[axisName::primary]->setPower(_axisData->power - _axisData->pidError); // yawError comes from the PID controller.
				_axisMotor[axisName::secondary]->setPower(_axisData->power + _axisData->pidError);
			}
			LOGGER_VERBOSE("....leave");
			break;
		default:;
		} /* end of switch */
	}/*--------------------- end of virtual service ---------------------------------------------*/

	void setState(state_e state)
	{
		LOGGER_NOTICE_FMT("set axisYaw state = %d", _state);
		_state = state;
		LOGGER_NOTICE("...leave  setState");
	} /*---------------------- end of setState --------------------------------------------------*/

	virtual boolean isArmed()
	{
		LOGGER_VERBOSE_FMT("State %d ", _state);
		return (_axisMotor[axisName::primary]->isArmed() && _axisMotor[axisName::secondary]->isArmed());
	} /*---------------------- end of isArmed ---------------------------------------------------*/

	virtual boolean isDeactivatePID()
	{
		bool state = ((_state == disablePID) && _axisMotor[axisName::primary]->isDeactivatePID() 
						&& _axisMotor[axisName::secondary]->isDeactivatePID());
		LOGGER_NOTICE_FMT("Yaw isDeactivatePID %d", state);
		return (state);
	} /*---------------------- end of isDeactivatePID -------------------------------------------*/

	virtual boolean isReady()
	{
		LOGGER_NOTICE_FMT("axisYaw is ready state %d", _state);
		LOGGER_NOTICE_FMT("axisMotor isReady pri state %d", (_axisMotor[axisName::primary]->isReady()));
		LOGGER_NOTICE_FMT("axisMotor isReady sec state %d", (_axisMotor[axisName::secondary]->isReady()));
		//bool ready = ((_state == ready)) && (_axisMotor[axisName::primary]->isReady()) && (_axisMotor[axisName::secondary]->isReady());
		bool ready = ((_axisMotor[axisName::primary]->isReady()) && (_axisMotor[axisName::secondary]->isReady()));
		LOGGER_NOTICE_FMT("getYawAxisState %d ", ready);
		return ready;
	} /*---------------------- end of isReady ---------------------------------------------------*/

}; /* ------------------------ end of axisYaw Class ---------------------------------------------*/