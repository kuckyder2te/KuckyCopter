#pragma once
/*  File name : motor.h
	Author: WilhelmKuckelsberg
    Project name : KuCo_Phantom 1
    Date : 2022-06-14

    Description : 
*/

#include <Arduino.h>
#include <PWMServo.h>
#include "def.h"
#include "myLogger.h"

#define ARM_MIN  1000 // 
#define ARM_MAX  2000 //
#define POWER_MIN  0 //
#define POWER_MAX  100 // 
#define BASE_MOTOR_POWER 10	//< 10% minimal throttle in fly mode for preventing stop of the motors

class Motor {

public:
	typedef enum {
		arming = 0, 
		off,
		on
	} states_e;

protected:
			uint8_t  _pin;
			PWMServo _motor;
			uint16_t _power;
			int16_t  _maxPower;
			states_e _states;
	static  uint8_t  _instance;
			uint8_t  _motor_address;	///< Gives everyone axis a title

 public:
	Motor(uint8_t pin) :_pin(pin){
		_power = POWER_MIN;
		_maxPower = POWER_MAX;
		_states = off;
	//	_motor_address  = _instance++;
	};

	void virtual setup() {
		LOGGER_VERBOSE("Enter....");
		_motor.attach(_pin, ARM_MIN, ARM_MAX);
		delay(20);
		LOGGER_NOTICE_FMT("Motor setup Pin%d, Motor%d", _pin, _motor_address);
		LOGGER_VERBOSE("....leave");
	} /* end of virtual setup */

	void updateState() {
	LOGGER_VERBOSE("Enter....");
		uint16_t resultingPower;

			switch (_states) {
			case arming:
			LOGGER_NOTICE_FMT("Motor arming %d ", _motor_address);
				break;

			case off:	
			LOGGER_NOTICE_FMT("Motor off %d ", _motor_address);
				_power = 0;
				_motor.write(ARM_MIN);
				break;

			case on:
			LOGGER_NOTICE_FMT("Motor on %d ", _motor_address);
				resultingPower = _power;
				if (resultingPower < BASE_MOTOR_POWER) {
					resultingPower = BASE_MOTOR_POWER;
				}
				_motor.write((resultingPower * 10) + ARM_MIN);
				break;
		}															
	LOGGER_VERBOSE("....leave");
	}//-------------------------- end of updateState ----------------------------------------------

	/* Motor is attached with a PIN and set the power to 2000 ms.
	 * After this set the power to 1000 ms and the arming is finished.*/ 

	void armingProcedure(bool step) {
	LOGGER_VERBOSE("Enter....");
		if (_states == arming) {
			if (!step) {
				LOGGER_NOTICE_FMT("Arming Procedure max: %d",ARM_MAX);
				_motor.write(ARM_MAX);		///< Arming begin
			} else {
				_motor.write(ARM_MIN);		///< Arming end
				LOGGER_NOTICE_FMT("Arming Procedure: min %d",ARM_MIN);
			}
		}
	LOGGER_VERBOSE("....leave");
	}//-------------------------- end of armingProcedure ---------------------------------------------------

	uint16_t getPower() const {
	LOGGER_VERBOSE("Enter....");
		return _power;
	LOGGER_VERBOSE("....leave");
	}//-------------------------- end of getPower -------------------------------------------------

	uint16_t setPower(int16_t power) {
	LOGGER_VERBOSE("Enter....");
		if (power < 0) {
			_power = 0;
		} else if (power > _maxPower) {
			_power = _maxPower;
		} else {
			_power = power;
		}
		 return _power;
	LOGGER_VERBOSE("....leave");
	}//-------------------------- end of setPower -------------------------------------------------

	uint8_t getMaxPower() const {
	LOGGER_VERBOSE("Enter....");
		return _maxPower;
	LOGGER_VERBOSE("....leave");
	}//-------------------------- end of getMaxPower ----------------------------------------------

	void setMaxPower(uint8_t maxPower) {
	LOGGER_VERBOSE("Enter....");
		if (maxPower > 100)
			_maxPower = 100;
		else
			_maxPower = maxPower;
	LOGGER_VERBOSE("....leave");
	}//-------------------------- end of setMaxPower ----------------------------------------------

	bool isMotorOff() {
	LOGGER_VERBOSE("Enter....");
		return (_states == off);
	LOGGER_VERBOSE("....leave");
	}//-------------------------- end of isMotorOn ------------------------------------------------

	void setMotorStates(states_e state) {
	LOGGER_VERBOSE("Enter....");
		_states = state;
	LOGGER_VERBOSE("....leave");
	}//-------------------------- end of setMotorStates -------------------------------------------
};/*--------------------------- end of Motor class ------------------------------------------------*/
