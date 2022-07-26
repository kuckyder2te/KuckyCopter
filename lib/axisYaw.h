#pragma once
/*  File name : axisYaw.h
	Project name : KuCo_Phantom 1
	Author: Wilhelm Kuckelsberg
	Date : 2022-06-17
	Description : Drohne
*/

#include "AxisBase.h"
#include "AxisMotor.h"

//#define LOCAL_DEBUG
#include "myLogger.h"

#include "def.h"

#define YAW_SENSIBILITY 5
#define YAW_FINE_TUNING 0.1

typedef struct
{
	double *rotationSpeed;	///< Speed which the copter should turn
 	int16_t *horz_Position; ///< Current YAW Position from Gyro
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
	{											/// Hier war der Fehler, ich muss das Basisobjekt weiterleiten
		_virtualSetpoint = 0;					/// SP ist immer 0 da dies der aktuellen Position entspricht die in der Mittelstellung verwendet wird
		AxisBase::_sp = &_virtualSetpoint;		// -Zuweisen des Modells zu den PID Parametern �ber Zeiger
		AxisBase::_fb = &_virtualFeedback;		// -Ist notwendig um AxisBase.Service den PID error f�r beliebige Achsen berechnen z lassen
		AxisBase::_error = &_axisData->pidError; // -

		_axisMotor[axisName_e::primary] = NULL;
		_axisMotor[axisName_e::secondary] = NULL;
		_state = disablePID;
		_lastCompass = 0;
	};

	virtual ~AxisYaw(){};

	AxisYaw *setModel(axisData_t *_model, yaw_t *yaw)
	{ // Rückgabe wert ist das eigene Objekt (this)
		LOGGER_VERBOSE("Enter....");
		_axisData = _model;
		_yaw = yaw;
		begin();
		LOGGER_VERBOSE("....leave");
		return this;
	} /*------------------------------- end of setModel -------------------------------*/
	
	AxisYaw *setAxisOrdered(AxisMotor *_axis)	// *_axis ist die Adresse
	{
		LOGGER_VERBOSE("Enter....");
		if (_axisMotor[axisName_e::primary] == NULL)
		{
			LOGGER_NOTICE_FMT("Set adress first Axis %p", _axis->getName().c_str());
			_axisMotor[axisName_e::primary] = _axis;
		}
		else if (_axisMotor[axisName_e::secondary] == NULL)
		{
			LOGGER_NOTICE_FMT("Set adress second Axis %p", _axis->getName().c_str());
			_axisMotor[axisName_e::secondary] = _axis;
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
			LOGGER_NOTICE_FMT_CHK(_state,_lastState,"Enter arming_start State %d",_state);
			digitalWrite(PIN_ESC_ON, HIGH);
			delay(20);
			_axisMotor[axisName_e::primary]->setState(AxisMotor::motorState_e::arming_start);
			_axisMotor[axisName_e::secondary]->setState(AxisMotor::motorState_e::arming_start);
			_state = arming_finished;
			LOGGER_VERBOSE("....leave arming_start");
			break;

		case arming_finished:
			/* Arming procedure is finished */
			LOGGER_NOTICE_FMT_CHK(_state,_lastState,"Enter arming_finished State %d",_state);
			if(_axisMotor[axisName_e::primary]->isArmed() && _axisMotor[axisName_e::secondary]->isArmed()){
				LOGGER_NOTICE("All Motors armed");
				_state = ready;
			}
			// _axisMotor[axisName_e::primary]->setState(AxisMotor::arming_end);
			// _axisMotor[axisName_e::secondary]->setState(AxisMotor::arming_end);
			LOGGER_VERBOSE("....leave");
			break;

		case disablePID:
			/* Disables the YawAxis PID controller and initiates deactivation for the motor axes. */
			LOGGER_NOTICE_FMT_CHK(_state,_lastState,"Enter disablePID State %d",_state);
			_newPID->disablePID();
			//			_yawData->axisData[0]->state = motor_state_e::disablePID;
			//			_yawData->axisData[1]->state = motor_state_e::disablePID;
			_axisMotor[axisName_e::primary]->setState(AxisMotor::disablePID);
			_axisMotor[axisName_e::secondary]->setState(AxisMotor::disablePID);
			LOGGER_VERBOSE("....leave");
			break;

		case enablePID:
			/* Enables the YawAxis PID controller and initiates activation for the motor axes. */
			LOGGER_NOTICE_FMT_CHK(_state,_lastState,"Enter enablePID State %d",_state);
			_newPID->enablePID();
			*_yaw->horz_Position = 0;
			//		_yawData->axisData[0]->state = motor_state_e::enablePID;
			//		_yawData->axisData[1]->state = motor_state_e::enablePID;
			_axisMotor[axisName_e::primary]->setState(AxisMotor::enablePID);
			_axisMotor[axisName_e::secondary]->setState(AxisMotor::enablePID);
			_lastCompass = *_axisData->feedback; ///< Becomes necessary, so that after the start the Copter does not turn.
			LOGGER_VERBOSE("....leave");
			break;

		case ready:
			LOGGER_NOTICE_FMT_CHK(_state,_lastState,"Enter ready State %d",_state);
			
			_axisMotor[axisName_e::primary]->setState(AxisMotor::ready);
			_axisMotor[axisName_e::secondary]->setState(AxisMotor::ready);

			_axisData->power = 10;		// Test

			if ((*_yaw->rotationSpeed > YAW_SENSIBILITY) || (*_yaw->rotationSpeed < (-YAW_SENSIBILITY)))
			{ ///< YAW Joystick is not moved....
				_yaw->axisData[0]->power = _axisData->power - *_yaw->rotationSpeed * YAW_FINE_TUNING;
				_yaw->axisData[1]->power = _axisData->power + *_yaw->rotationSpeed * YAW_FINE_TUNING;
				_axisMotor[axisName_e::primary]->setPower(_axisData->power - *_yaw->rotationSpeed * YAW_FINE_TUNING);
				_axisMotor[axisName_e::secondary]->setPower(_axisData->power + *_yaw->rotationSpeed * YAW_FINE_TUNING);
				*_yaw->horz_Position = 0;
			}
			else
			{	///< YAW  PID controller is active  .... Yaw joystick is in middle position
				_virtualFeedback = *_yaw->horz_Position; /// *_fb = into the PID controller
				_yaw->axisData[0]->power = _axisData->power - _axisData->pidError;
				_yaw->axisData[1]->power = _axisData->power + _axisData->pidError;
				_axisMotor[axisName_e::primary]->setPower(_axisData->power - _axisData->pidError); // yawError comes from the PID controller.
				_axisMotor[axisName_e::secondary]->setPower(_axisData->power + _axisData->pidError);
			}
			LOGGER_VERBOSE("....leave");
			break;
		default:;
		} /* end of switch */
	} /*--------------------- end of virtual service ----------------------------------*/

	void setState(state_e state)
	{
		LOGGER_NOTICE_FMT("set YawAxis State = %d", _state);
		_state = state;
		LOGGER_NOTICE("Leave  setState");
	} /*---------------------- end of setState ----------------------------------------*/

	boolean isArmed()
	{
		LOGGER_VERBOSE_FMT("State %d ", _state);
		return (_axisMotor[axisName_e::primary]->isArmed() && _axisMotor[axisName_e::secondary]->isArmed());		
	} /*---------------------- end of isArmed -----------------------------------------*/

	boolean isDeactivatePID()
	{
		return ((_state == disablePID)) && (_axisMotor[axisName_e::primary]->isDeactivatePID()) 
										&& (_axisMotor[axisName_e::secondary]->isDeactivatePID());
		LOGGER_NOTICE_FMT("Yaw isDeactivatePID %d", _state);
	} /*---------------------- end of isDeactivatePID ---------------------------------*/

	boolean isReady()
	{
		LOGGER_NOTICE_FMT("getYawAxisState %d ", _state);
		return ((_state == ready))  && (_axisMotor[axisName_e::primary]->isReady()) 
									&& (_axisMotor[axisName_e::secondary]->isReady());
	} /*---------------------- end of isReady -----------------------------------------*/
}; /* ------------------------ end of AxisYaw Class -----------------------------------*/