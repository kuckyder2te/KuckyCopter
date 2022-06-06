/*
 * AxisMotor.h
 *
 *  Created on: 25.12.2018
 *      Author: willy
 */

#ifndef AXISMOTOR_H_
#define AXISMOTOR_H_

#include <log4arduino.h>
#include "AxisBase.h"
#include "AxisMotor.h"
#include "Motor.h"

namespace modules {

extern Motor motor;

typedef struct {
	uint16_t power;			/// power from YAW Axis
	int16_t  pidError;
	int16_t  setpoint;	    ///< Memory for detuning the physical axis.
	int16_t* feedback;		///< Current value from the IMU
	double*  rcX;			///< virtual axis. Corresponds to the ROLL axis.		///  zu int16_t konvertieren
	double*  rcY;			///< virtual axis. Corresponds to the PITCH axis.
} axisData_t;

class AxisMotor: public AxisBase {
public:

typedef enum{
	arming_start = 0,
	arming_busy,
	arming_end,
	disablePID,
	enablePID,
	standby,
	ready
}state_e;

protected:

private:
		   axisData_t* _axisData;
	   Motor* 	   _motor[2];
	   bool 	   _invertRoll;
	   double 	   _roll;
	   state_e     _state;
static uint8_t     _instance;
	   uint8_t     _motoraxis_address;	///< Gives everyone axis a title


public:
	AxisMotor(Scheduler &manager, ProcPriority pr, uint32_t period,
			Motor* priMotor, Motor* secMotor, MyPid* pid, axisData_t* axisData,
			bool invertRoll) : _axisData(axisData), _invertRoll(invertRoll),
	AxisBase(manager, pr, period, pid) {
		AxisBase::_sp = &_axisData->setpoint;		/// _sp ist ein Pointer, der sich die Adresse des wertes aus &_axisDatat->setpoint holt
		AxisBase::_fb = _axisData->feedback;
		AxisBase::_error = &_axisData->pidError;
		_motor[motor_t::first]  = priMotor;
		_motor[motor_t::second] = secMotor;
		_roll 			  = 0;
		_state 			  = standby;
		_motoraxis_address 	 = _instance++;
	};

	virtual void setup() {
		AxisBase::setup();
		_motor[motor_t::first]->setup();
		_motor[motor_t::second]->setup();
		#ifdef AXISMOTOR_STATE
		LOG("AxisMotor setup %d", _motoraxis_address);
		delay(DEBUG_DWELL_TIME);
		#endif
	} /* end of virtual setup */

	virtual void service() {
		AxisBase::service();
		#ifdef AXISMOTOR_STATE
		LOG("AxisMotor service AxisNo %d", _motoraxis_address);
		delay(DEBUG_DWELL_TIME);
		#endif
		switch( _state){

			case arming_start:
			//	Serial.println("Axis Motor arming_start");
				#ifdef AXISMOTOR_STATE
				LOG("AxisMotor arming start %d ", _motoraxis_address);
				delay(DEBUG_DWELL_TIME);
				#endif
				_motor[motor_t::first]->setMotorStates(modules::Motor::arming);
				_motor[motor_t::second]->setMotorStates(modules::Motor::arming);
				_motor[motor_t::first]->armingProcedure(false);
				_motor[motor_t::second]->armingProcedure(false);
				_state = arming_busy;
				break;

			case arming_busy:
			//	Serial.println("Axis Motor arming_busy");
				#ifdef AXISMOTOR_STATE
				LOG("AxisMotor arming_busy %d ", _motoraxis_address);
				delay(DEBUG_DWELL_TIME);
				#endif
				break;

			case arming_end:
			//	Serial.println("Axis Motor arming_end");
				#ifdef AXISMOTOR_STATE
				LOG("AxisMotor arming end %d ", _motoraxis_address);
				delay(DEBUG_DWELL_TIME);
				#endif
				_motor[motor_t::first]->armingProcedure(true);
				_motor[motor_t::second]->armingProcedure(true);
				_motor[motor_t::first]->setMotorStates(modules::Motor::off);
				_motor[motor_t::second]->setMotorStates(modules::Motor::off);
				break;

			case disablePID:
			//	Serial.println("Axis Motor disablePID");
				/* Deactivate the PID controller from the motor axes. Does it have to be that way?
				 * Look at module AxisYaw */
				#ifdef AXISMOTOR_STATE
				LOG("AxisMotor deactivate PID %d ", _motoraxis_address);
				delay(DEBUG_DWELL_TIME);
				#endif
				_pid->disablePID();
				break;

			case enablePID:
			//	Serial.println("Axis Motor enablePID");
				/* Activate the PID controller from the MotorAxis with the current coefficients. */
				#ifdef AXISMOTOR_STATE
				LOG("AxisMotor activate PID %d ", _motoraxis_address);
				delay(DEBUG_DWELL_TIME);
				#endif
				_pid->enablePID();
				break;

			case standby:
			//	Serial.println("AxisMotor standby");
				#ifdef AXISMOTOR_STATE
				LOG("AxisMotor standby %d ", _motoraxis_address);
				delay(DEBUG_DWELL_TIME);
				#endif
				_motor[motor_t::first]->setMotorStates(modules::Motor::off);
				_motor[motor_t::second]->setMotorStates(modules::Motor::off);
				break;
																			/// neuer status	standby _motor[MOTOR_PRI]->setMotorStates(modules::Motor::on);
			case ready:														/// neuer Namen z.B Motoren arbeiten
			//	Serial.println("Motor ready");
				#ifdef AXISMOTOR_STATE
				LOG("AxisMotor ready %d ", _motoraxis_address);
				delay(DEBUG_DWELL_TIME);
				#endif
				_motor[motor_t::first]->setMotorStates(modules::Motor::on);
				_motor[motor_t::second]->setMotorStates(modules::Motor::on);

				_roll = (_invertRoll?(-(*_axisData->rcX)):(*_axisData->rcX));

				_axisData->setpoint = (_roll + (*_axisData->rcY));

				_motor[motor_t::first]->setPower(_axisData->power  - _axisData->pidError);
				_motor[motor_t::second]->setPower(_axisData->power + _axisData->pidError);
				#ifdef AXISMOTOR_STATE
				LOG("AxisMotor S:%d, P:%d, E:%d, No:%D", _axisData->setpoint, _axisData->power, _axisData->pidError, _motoraxis_address);
				delay(DEBUG_DWELL_TIME);
				#endif
				break;
		} /* end of switch */

		_motor[motor_t::first]->updateState();
		_motor[motor_t::second]->updateState();

	} /* end of virtual service */

	void setPower(int16_t _power){
		if(_power < 0)
			_axisData->power = 0;
		else
			_axisData->power = _power;

	}//---------------------- end of setPower -----------------------------------------------------

	void setState(state_e state) {
		_state = state;
		#ifdef AXISMOTOR_STATE
		LOG("set AxisMotor State = %d",  _state);
		delay(DEBUG_DWELL_TIME);
		#endif

	}//---------------------- end of setState -----------------------------------------------------

	boolean isArmed() const {

		return ((_motor[motor_t::first]->isMotorOff())
			  &&(_motor[motor_t::second]->isMotorOff()));

	}//---------------------- end of isArmed ------------------------------------------------------

	boolean isDeactivatePID(){
		#ifdef AXISMOTOR_STATE
		LOG("AxisMotor isDeactivatePID %d", _motoraxis_address);
		delay(DEBUG_DWELL_TIME);
		#endif
		return( _state == disablePID);

	}//---------------------- end of isDeactivatePID ----------------------------------------------

	boolean isStandby() const {
		return ( _state == standby);

	}//---------------------- end of isStandby ----------------------------------------------------

	boolean isReady() const {
		return ( _state == ready);

	}//---------------------- end of isReady ------------------------------------------------------

	Motor** getMotor() {
		return _motor;

	}//---------------------- end of getMotor -----------------------------------------------------

	virtual void onEnable(){
		LOG("%s enable",__PRETTY_FUNCTION__);
	}
	virtual void onDisable(){
		LOG("%s disable",__PRETTY_FUNCTION__);
	}
};
} /* namespace modules */

#endif /* AXISMOTOR_H_ */

/* -------------------------- end of AxisMotor Class --------------------------------------------*/
