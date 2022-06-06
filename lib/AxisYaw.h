/*
 * AxisYaw.h
 *
 *  Created on: 25.12.2018
 *      Author: willy
 */

#ifndef AXISYAW_H_
#define AXISYAW_H_

#include <log4arduino.h>
#include "AxisBase.h"
#include "AxisMotor.h"
#include "Config.h"

namespace modules {

typedef struct {
	uint16_t   throttle;			///< Corresponds to the "rcThrottle" from rcData
	int16_t    yawError;			///< The deviation calculated by the PID.
	int16_t    setpoint;			///< Provided in the model only for debugging purposes as it is used directly for the motor axes.
	int16_t*   feedback;			///< " ""
	double*    rotationSpeed;		///< Speed which the copter should turn
	int16_t*   horz_Position;		///< Current YAW Position from Gyro
	axisData_t axisData[2];
} yawData_t;

class AxisYaw: public AxisBase {

public:
	typedef enum{
		arming_start,
		arming_power_on,
		arming_finished,
		disablePID,
		enablePID,
		ready
	}state_e;

private:
	AxisMotor* 	_motorAxis[2];
	yawData_t* 	_yawData;
	state_e		_state;
	int16_t     _lastCompass;
	int16_t 	_virtualFeedback;	///< The calculated feedback due to the yaw rotation
	int16_t		_virtualSetpoint;
public:

	AxisYaw(Scheduler &manager, ProcPriority pr, uint32_t period,
			AxisMotor* priAxis, AxisMotor* secAxis, MyPid* pid,
			yawData_t* yawData) : _yawData(yawData),
			AxisBase(manager, pr, period, pid) {			/// Hier war der Fehler, ich muss das Basisobjekt weiterleiten
			_virtualSetpoint = 0;								/// SP ist immer 0 da dies der aktuellen Position entspricht die in der Mittelstellung verwendet wird
			AxisBase::_sp    = &_virtualSetpoint;				// -Zuweisen des Modells zu den PID Parametern �ber Zeiger
			AxisBase::_fb    = &_virtualFeedback;			// -Ist notwendig um AxisBase.Service den PID error f�r beliebige Achsen berechnen z lassen
			AxisBase::_error = &_yawData->yawError;			// -
			_motorAxis[motor_t::first]  = priAxis;
			_motorAxis[motor_t::second] = secAxis;
			_state 			 = disablePID;
			_lastCompass	 = 0;
	}
	;
	virtual void setup() {
		AxisBase::setup();
	}/* end of virtual setup */

	virtual void service() {
		AxisBase::service();

		switch(_state){

			case arming_start:
				/* The arming procedure will start. */
				#ifdef AXISYAW_STATE
				LOGS("AxisYAW arming_start");
				delay(DEBUG_DWELL_TIME);
				#endif
				_motorAxis[axis_t::Primary]->setState(modules::AxisMotor::arming_start);
				_motorAxis[axis_t::Secondary]->setState(modules::AxisMotor::arming_start);
				_state = arming_power_on;
				break;

			case arming_power_on:
				/* All ESC�s will connected with the main power. */
				#ifdef AXISYAW_STATE
				LOGS("AxisYAW arming_power_on");
				delay(DEBUG_DWELL_TIME);
				#endif
//				Serial3.println("ESC_ON");
				digitalWrite(PIN_ESC_ON, HIGH);
				delay(20);
				_state = arming_finished;
				break;

			case arming_finished:
				/* Arming procedure is finished */
				#ifdef AXISYAW_STATE
				LOGS("Axis YAW arming_finished");
				delay(DEBUG_DWELL_TIME);
				#endif
				_motorAxis[axis_t::Primary]->setState(modules::AxisMotor::arming_end);
				_motorAxis[axis_t::Secondary]->setState(modules::AxisMotor::arming_end);
				break;

			case disablePID:
				#ifdef AXISYAW_STATE
				/* Disables the YawAxis PID controller and initiates deactivation for the motor axes. */
				LOGS("AxisYaw deactivatePID");
				delay(DEBUG_DWELL_TIME);
				#endif
				_pid->disablePID();
				_motorAxis[axis_t::Primary]->setState(modules::AxisMotor::disablePID);
				_motorAxis[axis_t::Secondary]->setState(modules::AxisMotor::disablePID);
				break;

			case enablePID:
				/* Enables the YawAxis PID controller and initiates activation for the motor axes. */
				#ifdef AXISYAW_STATE
				LOGS("AxisYaw enablePID");
				delay(DEBUG_DWELL_TIME);
				#endif
				_pid->enablePID();
				*_yawData->horz_Position = 0;
				_motorAxis[axis_t::Primary]->setState(modules::AxisMotor::enablePID);
				_motorAxis[axis_t::Secondary]->setState(modules::AxisMotor::enablePID);
				_lastCompass = *_yawData->feedback;	///< Becomes necessary, so that after the start the Copter does not turn.
				break;

			case ready:

				#ifdef AXISYAW_STATE
				LOGS("AxisYaw ready");
				delay(DEBUG_DWELL_TIME);
				#endif
				_motorAxis[axis_t::Primary]->setState(modules::AxisMotor::ready);
				_motorAxis[axis_t::Secondary]->setState(modules::AxisMotor::ready);

				 if((*_yawData->rotationSpeed > YAW_SENSIBILITY)||(*_yawData->rotationSpeed < (-YAW_SENSIBILITY))){ ///< YAW Joystick is not moved....

					 _motorAxis[axis_t::Primary]->setPower(_yawData->throttle - *_yawData->rotationSpeed*YAW_FINE_TUNING);
					 _motorAxis[axis_t::Secondary]->setPower(_yawData->throttle + *_yawData->rotationSpeed*YAW_FINE_TUNING);
					 *_yawData->horz_Position = 0;
				 }else{					 ///< YAW  PID controller is active  .... Yaw joystick is in middle position
					 _virtualFeedback = *_yawData->horz_Position;				/// *_fb = into the PID controller
					 _motorAxis[axis_t::Primary]->setPower(_yawData->throttle - _yawData->yawError);		//yawError comes from the PID controller.
					 _motorAxis[axis_t::Secondary]->setPower(_yawData->throttle + _yawData->yawError);
				 }
				 break;
		default:
			;
		} /* end of switch */

	}/* end of virtual service ------------------------------------------------------------------*/

	void setState(state_e state) {
		_state = state;
		#ifdef AXISYAW_STATE
		LOG("set YawAxis State = %d", _state);
		delay(DEBUG_DWELL_TIME);
		#endif
	}//---------------------- end of setState -----------------------------------------------------

	boolean isArmed() {
		#ifdef AXISYAW_STATE
		LOG("get YawAxis State %d ", _state);
		delay(DEBUG_DWELL_TIME);
		#endif
		return ((_state == arming_finished)
				&&(_motorAxis[axis_t::Primary]->isArmed())
				&&(_motorAxis[axis_t::Secondary]->isArmed())
				);

	}//---------------------- end of isArmed ------------------------------------------------------

	boolean isDeactivatePID(){
		#ifdef AXISYAW_STATE
		LOG("Yaw isDeactivatePID ", _state);
		delay(DEBUG_DWELL_TIME);
		#endif
		return((_state == disablePID)
				&& (_motorAxis[axis_t::Primary]->isDeactivatePID())
				&& (_motorAxis[axis_t::Secondary]->isDeactivatePID())
				);
	}//---------------------- end of isDeactivatePID ----------------------------------------------

	boolean isReady() {
		#ifdef AXISYAW_STATE
		LOG("getYawAxisState %d ", _state);
		delay(DEBUG_DWELL_TIME);
		#endif
		return ((_state == ready)
				&&(_motorAxis[axis_t::Primary]->isReady())
				&&(_motorAxis[axis_t::Secondary]->isReady())
				);

	}//---------------------- end of isReady ------------------------------------------------------

	virtual void onEnable(){
		LOG("%s enable",__PRETTY_FUNCTION__);
	}
	virtual void onDisable(){
		LOG("%s disable",__PRETTY_FUNCTION__);
	}
};
} /* end of namespace modules */
#endif /* AXISYAW_H_ */
/* -------------------------- end of AxisYaw Class ----------------------------------------------*/
