#pragma once
/*  File name : motor.h
	Author: WilhelmKuckelsberg
    Project name : KuCo_Phantom 1
    Date : 2022-06-14

    Description : 
*/

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include "def.h"

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates


class Motor {

public:
	typedef enum {
		arming = 0, off, on
	} states_e;

protected:
			uint8_t  _pin;
//			Servo 	 _motor;
			uint16_t _power;
			int16_t  _maxPower;
			states_e _states;
	static  uint8_t  _instance;
			uint8_t  _motor_address;	///< Gives everyone axis a title

 public:
	Motor(uint8_t pin) :_pin(pin){
		// _power = 0;
		// _maxPower = POWER_MAX;
		// _states = off;
		// _motor_address  = _instance++;
	};

	void virtual setup() {
		// _motor.attach(_pin, ARM_MIN, ARM_MAX);
		// delay(20);
		// LOGGER_NOTICE_FMT("Motor setup %d, %d", _pin, _motor_address);
	
	} /* end of virtual setup */

	void updateState() {
	// uint16_t resultingPower;

	// 	switch (_states) {
	// 	case arming:
	// 		#ifdef MOTOR_STATE
	// 		LOGGER_NOTICE_FMT("Motor arming %d ", _motor_address);
	// 		delay(DEBUG_DWELL_TIME);
	// 		#endif
	// 		break;

	// 	case off:
	// 		
	// 		LOGGER_NOTICE_FMT("Motor off %d ", _motor_address);
	// 		
	// 		_power = 0;
	// 		_motor.writeMicroseconds(ARM_MIN);
	// 		break;

	// 	case on:
	// 		
	// 		LOGGER_NOTICE_FMT("Motor on %d ", _motor_address);
	// 		
	// 		resultingPower = _power;

	// 		if (resultingPower < BASE_MOTOR_POWER) {
	// 			resultingPower = BASE_MOTOR_POWER;
	// 		}
	// 		_motor.writeMicroseconds((resultingPower * 10) + ARM_MIN);
	// 		break;
	// 	}

	} //-------------------------- end of updateState ----------------------------------------------

	 /* Motor is attached with a PIN and set the power to 2000 ms.
	  * After this set the power to 1000 ms and the arming is finished.	 */

	void armingProcedure(bool step) {
		// if (_states == arming) {
		// 	if (!step) {
		// 		LOGGER_NOTICE_FMT("Arming Procedure max: %d",ARM_MAX);
		// 		_motor.writeMicroseconds(ARM_MAX);		///< Arming begin
		// 	} else {
		// 		_motor.writeMicroseconds(ARM_MIN);		///< Arming end
		// 		LOGGER_NOTICE_FMT("Arming Procedure: min %d",ARM_MIN);
		// 	}
		// }

	}//-------------------------- end of arming ---------------------------------------------------

	uint16_t getPower() const {
		return _power;

	}//-------------------------- end of getPower -------------------------------------------------

	uint16_t setPower(int16_t power) {

		// 	if (power < 0) {
		// 		_power = 0;
		// 	} else if (power > _maxPower) {
		// 		_power = _maxPower;
		// 	} else {
		// 		_power = power;
		// 	}
		 return _power;

	}//-------------------------- end of setPower -------------------------------------------------

	uint8_t getMaxPower() const {
		return _maxPower;

	}//-------------------------- end of getMaxPower ----------------------------------------------

	void setMaxPower(uint8_t maxPower) {
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
};/*--------------------------- end of Motor class ------------------------------------------------*/
