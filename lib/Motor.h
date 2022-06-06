/*
 * Motor.h
 *
 *  Created on: 25.12.2018
 *      Author: willy
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include <Arduino.h>
#include <log4arduino.h>
#include <Config.h>
#include <Servo.h>

namespace modules {

class Motor {

public:
	typedef enum {
		arming = 0, off, on
	} states_e;

protected:
			uint8_t  _pin;
			Servo 	 _motor;
			uint16_t _power;
			int16_t  _maxPower;
			states_e _states;
	static  uint8_t  _instance;
			uint8_t  _motor_address;	///< Gives everyone axis a title

 public:
	Motor(uint8_t pin) :_pin(pin){
		_power = 0;
		_maxPower = POWER_MAX;
		_states = off;
		_motor_address  = _instance++;
	};

	void virtual setup() {
		_motor.attach(_pin, ARM_MIN, ARM_MAX);
		delay(20);
		LOG("Motor setup %d, %d", _pin, _motor_address);
		delay(DEBUG_DWELL_TIME);
	} /* end of virtual setup */

	void Motor::updateState() {
	uint16_t resultingPower;

		switch (_states) {
		case arming:
			#ifdef MOTOR_STATE
			LOG("Motor arming %d ", _motor_address);
			delay(DEBUG_DWELL_TIME);
			#endif
			break;

		case off:
			#ifdef MOTOR_STATE
			LOG("Motor off %d ", _motor_address);
			delay(DEBUG_DWELL_TIME);
			#endif
			_power = 0;
			_motor.writeMicroseconds(ARM_MIN);
			break;

		case on:
			#ifdef MOTOR_STATE
			LOG("Motor on %d ", _motor_address);
			delay(DEBUG_DWELL_TIME);
			#endif
			resultingPower = _power;

			if (resultingPower < BASE_MOTOR_POWER) {
				resultingPower = BASE_MOTOR_POWER;
			}
			_motor.writeMicroseconds((resultingPower * 10) + ARM_MIN);
			break;
		}

	} //-------------------------- end of updateState ----------------------------------------------

	 /* Motor is attached with a PIN and set the power to 2000 ms.
	  * After this set the power to 1000 ms and the arming is finished.	 */

	void Motor::armingProcedure(bool step) {
		if (_states == arming) {
			if (!step) {
				LOG("Arming Procedure max: %d",ARM_MAX);
				_motor.writeMicroseconds(ARM_MAX);		///< Arming begin
			} else {
				_motor.writeMicroseconds(ARM_MIN);		///< Arming end
				LOG("Arming Procedure: min %d",ARM_MIN);
			}
		}

	}//-------------------------- end of arming ---------------------------------------------------

	uint16_t Motor::getPower() const {
		return _power;

	}//-------------------------- end of getPower -------------------------------------------------

	uint16_t Motor::setPower(int16_t power) {

			if (power < 0) {
				_power = 0;
			} else if (power > _maxPower) {
				_power = _maxPower;
			} else {
				_power = power;
			}
		return _power;

	}//-------------------------- end of setPower -------------------------------------------------

	uint8_t Motor::getMaxPower() const {
		return _maxPower;

	}//-------------------------- end of getMaxPower ----------------------------------------------

	void Motor::setMaxPower(uint8_t maxPower) {
		if (maxPower > 100)
			_maxPower = 100;
		else
			_maxPower = maxPower;

	}//-------------------------- end of setMaxPower ----------------------------------------------

	bool isMotorOff() {
		return (_states == off);

	}//-------------------------- end of isMotorOn ------------------------------------------------

	void setMotorStates(states_e state) {
		_states = state;

	}//-------------------------- end of setMotorStates -------------------------------------------
};
} /* end of namespace modules */

#endif /* MOTOR_H_ */
/*--------------------------- end of Motor class ------------------------------------------------*/
