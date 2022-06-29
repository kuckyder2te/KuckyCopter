#pragma once
/*  File name : axisYaw.h
	Project name : KuCo_Phantom 1
	Author: Wilhelm Kuckelsberg
	Date : 2022-06-17
	Description : Drohne
*/

#include "AxisBase.h"
#include "AxisMotor.h"
#include "myLogger.h"

#define YAW_SENSIBILITY 5
#define YAW_FINE_TUNING 0.1
typedef struct
{
	uint16_t throttle;		///< Corresponds to the "rcThrottle" from rcData
	int16_t yawError;		///< The deviation calculated by the PID.
	int16_t setpoint;		///< Provided in the model only for debugging purposes as it is used directly for the motor axes.
	int16_t *feedback;		///< " ""
	double *rotationSpeed;	///< Speed which the copter should turn
	int16_t *horz_Position; ///< Current YAW Position from Gyro
	AxisMotor::axisData_t *axisData[2];
} yawData_t;

typedef enum
{
	Primary,
	Secondary,
	YawAxis
} axis_t;

class AxisYaw : public AxisBase
{

public:
	typedef enum
	{
		arming_start,
		arming_power_on,
		arming_finished,
		disablePID,
		enablePID,
		ready
	} state_e;

private:
	AxisMotor *_axisMotor[2];
	yawData_t *_yawData;
	state_e _state;
	int16_t _lastCompass;
	int16_t _virtualFeedback; ///< The calculated feedback due to the yaw rotation
	int16_t _virtualSetpoint;

public:
	AxisYaw(const String &name) : AxisBase(name)
	{											/// Hier war der Fehler, ich muss das Basisobjekt weiterleiten
		_virtualSetpoint = 0;					/// SP ist immer 0 da dies der aktuellen Position entspricht die in der Mittelstellung verwendet wird
		AxisBase::_sp = &_virtualSetpoint;		// -Zuweisen des Modells zu den PID Parametern �ber Zeiger
		AxisBase::_fb = &_virtualFeedback;		// -Ist notwendig um AxisBase.Service den PID error f�r beliebige Achsen berechnen z lassen
		AxisBase::_error = &_yawData->yawError; // -

		_axisMotor[axis_t::Primary] = NULL;
		_axisMotor[axis_t::Secondary] = NULL;
		_state = disablePID;
		_lastCompass = 0;
	};

	virtual ~AxisYaw(){};

	AxisYaw *setModel(yawData_t *_model)
	{ // Rückgabe wert ist das eigene Objekt (this)
		LOGGER_VERBOSE("Enter....");
		_yawData = _model;
		LOGGER_VERBOSE("....leave");
		return this;
	} /*------------------------------- end of setModel ------------------------------*/
	
	AxisYaw *setAxisOrdered(AxisMotor *_axis)
	{
		LOGGER_VERBOSE("Enter....");
		if (_axisMotor[axis_t::Primary] == NULL)
		{
			LOGGER_NOTICE("Set first Axis");
			_axisMotor[axis_t::Primary] = _axis;
		}
		else if (_axisMotor[axis_t::Secondary] == NULL)
		{
			LOGGER_NOTICE("Set second Axis");
			_axisMotor[axis_t::Secondary] = _axis;
		}
		else
		{
			LOGGER_FATAL("Too much Axis initialized!!");
		}
		LOGGER_VERBOSE("....leave");
		return this;
	} /*------------------------------- end of setAxisOrdered -------------------------*/

	virtual void begin()
	{
		AxisBase::begin();
	} /*------------------------------- end of virtual begin --------------------------*/

	virtual void update()
	{
		AxisBase::update();

		switch (_state)
		{

		case arming_start:
			/* The arming procedure will start. */
			LOGGER_VERBOSE("Enter....");
			_axisMotor[axis_t::Primary]->setState(AxisMotor::motorState_e::arming_start);
			_axisMotor[axis_t::Secondary]->setState(AxisMotor::motorState_e::arming_start);
			_state = arming_power_on;
			LOGGER_VERBOSE("....leave");
			break;

		case arming_power_on:
			/* All ESCs will connected with the main power. */
			LOGGER_VERBOSE("Enter....");
			digitalWrite(PIN_ESC_ON, HIGH);
			delay(20);
			_state = arming_finished;
			LOGGER_VERBOSE("....leave");
			break;

		case arming_finished:
			/* Arming procedure is finished */
			LOGGER_VERBOSE("Enter....");
			_axisMotor[axis_t::Primary]->setState(AxisMotor::arming_end);
			_axisMotor[axis_t::Secondary]->setState(AxisMotor::arming_end);
			LOGGER_VERBOSE("....leave");
			break;

		case disablePID:
			/* Disables the YawAxis PID controller and initiates deactivation for the motor axes. */
			LOGGER_VERBOSE("Enter....");
			_newPID->disablePID();
			//			_yawData->axisData[0]->state = motor_state_e::disablePID;
			//			_yawData->axisData[1]->state = motor_state_e::disablePID;
			_axisMotor[axis_t::Primary]->setState(AxisMotor::disablePID);
			_axisMotor[axis_t::Secondary]->setState(AxisMotor::disablePID);
			LOGGER_VERBOSE("....leave");
			break;

		case enablePID:
			/* Enables the YawAxis PID controller and initiates activation for the motor axes. */
			LOGGER_VERBOSE("Enter....");
			_newPID->enablePID();
			*_yawData->horz_Position = 0;
			//		_yawData->axisData[0]->state = motor_state_e::enablePID;
			//		_yawData->axisData[1]->state = motor_state_e::enablePID;
			_axisMotor[axis_t::Primary]->setState(AxisMotor::enablePID);
			_axisMotor[axis_t::Secondary]->setState(AxisMotor::enablePID);
			_lastCompass = *_yawData->feedback; ///< Becomes necessary, so that after the start the Copter does not turn.
			LOGGER_VERBOSE("....leave");
			break;

		case ready:
			LOGGER_VERBOSE("Enter....");
			_axisMotor[axis_t::Primary]->setState(AxisMotor::ready);
			_axisMotor[axis_t::Secondary]->setState(AxisMotor::ready);
			if ((*_yawData->rotationSpeed > YAW_SENSIBILITY) || (*_yawData->rotationSpeed < (-YAW_SENSIBILITY)))
			{ ///< YAW Joystick is not moved....
				_yawData->axisData[0]->power = _yawData->throttle - *_yawData->rotationSpeed * YAW_FINE_TUNING;
				_yawData->axisData[1]->power = _yawData->throttle + *_yawData->rotationSpeed * YAW_FINE_TUNING;
				_axisMotor[axis_t::Primary]->setPower(_yawData->throttle - *_yawData->rotationSpeed * YAW_FINE_TUNING);
				_axisMotor[axis_t::Secondary]->setPower(_yawData->throttle + *_yawData->rotationSpeed * YAW_FINE_TUNING);
				*_yawData->horz_Position = 0;
			}
			else
			{												 ///< YAW  PID controller is active  .... Yaw joystick is in middle position
				_virtualFeedback = *_yawData->horz_Position; /// *_fb = into the PID controller
				_yawData->axisData[0]->power = _yawData->throttle - _yawData->yawError;
				_yawData->axisData[1]->power = _yawData->throttle + _yawData->yawError;
				_axisMotor[axis_t::Primary]->setPower(_yawData->throttle - _yawData->yawError); // yawError comes from the PID controller.
				_axisMotor[axis_t::Secondary]->setPower(_yawData->throttle + _yawData->yawError);
			}
			LOGGER_VERBOSE("....leave");
			break;
		default:;
		} /* end of switch */
	} /*--------------------- end of virtual service ----------------------------------*/

	void test(){
		LOGGER_NOTICE("Dies ist ein Testaufruf");
	}  /*---------------------- end of test -------------------------------------------*/

	void setState(state_e state)
	{
		LOGGER_NOTICE_FMT("set YawAxis State = %d", _state);
		_state = state;
		LOGGER_NOTICE("Leave  setState");
	} /*---------------------- end of setState ----------------------------------------*/

	boolean isArmed()
	{
		LOGGER_NOTICE_FMT("get YawAxis State %d ", _state);
	//	return true;
		return ((_state == arming_finished)) && (_axisMotor[axis_t::Primary]->isArmed()) && (_axisMotor[axis_t::Secondary]->isArmed());
		
	} /*---------------------- end of isArmed -----------------------------------------*/

	boolean isDeactivatePID()
	{
		return ((_state == disablePID)) && (_axisMotor[axis_t::Primary]->isDeactivatePID()) && (_axisMotor[axis_t::Secondary]->isDeactivatePID());
		LOGGER_NOTICE_FMT("Yaw isDeactivatePID ", _state);
	} /*---------------------- end of isDeactivatePID ---------------------------------*/

	boolean isReady()
	{
		return ((_state == ready))  && (_axisMotor[axis_t::Primary]->isReady()) && (_axisMotor[axis_t::Secondary]->isReady());
		LOGGER_NOTICE_FMT("getYawAxisState %d ", _state);
	} /*---------------------- end of isReady -----------------------------------------*/
}; /* ------------------------ end of AxisYaw Class -----------------------------------*/
