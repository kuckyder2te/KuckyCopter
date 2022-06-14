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

typedef struct {
	uint16_t power;			/// power from YAW Axis
	int16_t  pidError;
	int16_t  setpoint;	    ///< Memory for detuning the physical axis.
	int16_t* feedback;		///< Current value from the IMU
	double*  rcX;			///< virtual axis. Corresponds to the ROLL axis.		///  zu int16_t konvertieren
	double*  rcY;			///< virtual axis. Corresponds to the PITCH axis.
} axisData_t;

typedef enum{
	arming_start = 0,
	arming_busy,
	arming_end,
	disablePID,
	enablePID,
	standby,
	ready
}state_e;

//extern Motor motor;

class AxisMotor : public AxisBase {

private:
	axisData_t *_axisData;
//	Motor* 	   _motor[2];
	bool 	   _invertRoll;
	double 	   _roll;
	state_e     _state;
    //static uint8_t _instance;
	uint8_t     _motoraxis_address;	///< Gives everyone axis a title    


public:
    AxisMotor(const String& name)  : AxisBase(name) {
		AxisBase::_sp = &_axisData->setpoint;		/// _sp ist ein Pointer, der sich die Adresse des wertes aus &_axisDatat->setpoint holt
		AxisBase::_fb = _axisData->feedback;
		AxisBase::_error = &_axisData->pidError;
		// _motor[motor_t::first]  = priMotor;
		// _motor[motor_t::second] = secMotor;
		_roll 			  = 0;
		_state 			  = standby;
		_motoraxis_address 	 = AxisBase::_instance++;
    }

	virtual ~AxisMotor() {}

   AxisMotor* setModel(axisData_t* _model){    // Rückgabe wert ist das eigene Objekt (this)
        LOGGER_VERBOSE("Enter....");
        _axisData = _model;
        LOGGER_VERBOSE("....leave");
    return this;
	}

 
    /*---------------------------------------------------------------------------------*/
    virtual void begin() override {
    LOGGER_NOTICE("BaseAxis initialized");
     AxisBase::begin();              

    LOGGER_NOTICE("End init BaseAxis");
    
	// _motor[motor_t::first]->setup();
	// _motor[motor_t::second]->setup();
		
	LOGGER_NOTICE_FMT("AxisMotor setup %d", _motoraxis_address);	
    }/*-------------------------------- end of begin ----------------------------------*/


    virtual void update() override {

		AxisBase::update();         
		
		LOGGER_NOTICE_FMT("AxisMotor service AxisNo %d", _motoraxis_address);
		
		switch( _state){

			case arming_start:
			
				LOGGER_NOTICE_FMT("AxisMotor arming start %d ", _motoraxis_address);
			
				// _motor[motor_t::first]->setMotorStates(modules::Motor::arming);
				// _motor[motor_t::second]->setMotorStates(modules::Motor::arming);
				// _motor[motor_t::first]->armingProcedure(false);
				// _motor[motor_t::second]->armingProcedure(false);
				_state = arming_busy;
				break;

			case arming_busy:
			
				LOGGER_NOTICE_FMT("AxisMotor arming_busy %d ", _motoraxis_address);
			
				break;

			case arming_end:
			
				LOGGER_NOTICE_FMT("AxisMotor arming end %d ", _motoraxis_address);
			
				// _motor[motor_t::first]->armingProcedure(true);
				// _motor[motor_t::second]->armingProcedure(true);
				// _motor[motor_t::first]->setMotorStates(modules::Motor::off);
				// _motor[motor_t::second]->setMotorStates(modules::Motor::off);
				 break;

			case disablePID:
			//	Serial.println("Axis Motor disablePID");
				/* Deactivate the PID controller from the motor axes. Does it have to be that way?
				 * Look at module AxisYaw */
				
				LOGGER_NOTICE_FMT("AxisMotor deactivate PID %d ", _motoraxis_address);
				
				AxisBase::_newPID->disablePID();

                break;

			case enablePID:
			
				/* Activate the PID controller from the MotorAxis with the current coefficients. */
				
				LOGGER_NOTICE_FMT("AxisMotor activate PID %d ", _motoraxis_address);
				
				AxisBase::_newPID->enablePID();
				break;

			case standby:
			
				LOGGER_NOTICE_FMT("AxisMotor standby %d ", _motoraxis_address);
				
				// _motor[motor_t::first]->setMotorStates(modules::Motor::off);
				// _motor[motor_t::second]->setMotorStates(modules::Motor::off);
				break;
																			/// neuer status	standby _motor[MOTOR_PRI]->setMotorStates(modules::Motor::on);
			case ready:														/// neuer Namen z.B Motoren arbeiten
			
				LOGGER_NOTICE_FMT("AxisMotor ready %d ", _motoraxis_address);
			
				// _motor[motor_t::first]->setMotorStates(modules::Motor::on);
				// _motor[motor_t::second]->setMotorStates(modules::Motor::on);

				_roll = (_invertRoll?(-(*_axisData->rcX)):(*_axisData->rcX));

				_axisData->setpoint = (_roll + (*_axisData->rcY));

				// _motor[motor_t::first]->setPower(_axisData->power  - _axisData->pidError);
				// _motor[motor_t::second]->setPower(_axisData->power + _axisData->pidError);
			
				LOGGER_NOTICE_FMT("AxisMotor S:%d, P:%d, E:%d, No:%D", _axisData->setpoint, _axisData->power, _axisData->pidError, _motoraxis_address);
			
				break;
		} /* end of switch */

		// _motor[motor_t::first]->updateState();
		// _motor[motor_t::second]->updateState();
    }/*................................... end of update ------------------------------*/

    // optional (you can remove this method)
    // virtual void exit() override {
    // }

    // optional (you can remove this method)
    // virtual void idle() override {
    // }

    // optional (you can remove this method)
    // virtual void reset() override {
    // }
};/*................................... end of MotorAxis.h class ----------------------*/
