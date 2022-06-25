#pragma once
/*  File name : motor.h
	Author: WilhelmKuckelsberg
	Project name : KuCo_Phantom 1
	Date : 2022-06-14

	Description :
*/

#include <Arduino.h>
#include <RP2040_PWM.h>
#include "def.h"
#include "myLogger.h"

#define ARM_MIN 1000		//
#define ARM_MAX 2000		//
#define POWER_MIN 0			//
#define POWER_MAX 100		//
#define BASE_MOTOR_POWER 10 //< 10% minimal throttle in fly mode for preventing stop of the motors
#define PIN_ESC_ON 14

#define _PWM_LOGLEVEL_ 3

#if (defined(ARDUINO_NANO_RP2040_CONNECT) || defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_ADAFRUIT_FEATHER_RP2040) || \
	 defined(ARDUINO_GENERIC_RP2040)) &&                                                                                       \
	defined(ARDUINO_ARCH_MBED)

#if (_PWM_LOGLEVEL_ > 3)
#warning USING_MBED_RP2040_PWM
#endif

#elif (defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_ADAFRUIT_FEATHER_RP2040) || \
	   defined(ARDUINO_GENERIC_RP2040)) &&                                                                               \
	!defined(ARDUINO_ARCH_MBED)

#if (_PWM_LOGLEVEL_ > 3)
#warning USING_RP2040_PWM
#endif
#else
#error This code is intended to run on the RP2040 mbed_nano, mbed_rp2040 or arduino-pico platform! Please check your Tools->Board setting.
#endif
class Motor
{
	float frequency;
	float dutyCycle;

public:
	typedef enum
	{
		arming = 0,
		off,
		on
	} motorstate_e;

protected:
	uint8_t _pin;
	RP2040_PWM *_motor; // warum pointer?
	uint16_t _power;
	int16_t _maxPower;
	motorstate_e _motorstate;
	static uint8_t _instance;
	uint8_t _motor_address; ///< Gives everyone axis a title

public:
	Motor(uint8_t pin) : _pin(pin)
	{
		_power = 0;
		_maxPower = 180;
		_motorstate = off;
		//	_motor_address  = _instance++;
	};

	void setup()
	{
		LOGGER_VERBOSE("Enter....");
		LOGGER_VERBOSE("Not implemented yet");
		LOGGER_NOTICE("Starting PWM_DynamicDutyCycle on ");
		LOGGER_NOTICE(BOARD_NAME);
		LOGGER_NOTICE(RP2040_PWM_VERSION);

		frequency = 1000;
		_motor = new RP2040_PWM(_pin, frequency, 50);

		if (_motor)
		{
			_motor->setPWM();
		}

		//_motor.attach(_pin, ARM_MIN, ARM_MAX);
		delay(20);
		LOGGER_NOTICE_FMT("Motor setup Pin=%d, Motor=%d", _pin, _motor_address);
		LOGGER_VERBOSE("....leave");
	} /*------------------------------- end of setup ----------------------------------*/

	void updateState()
	{
		LOGGER_VERBOSE("Enter....");
		uint16_t resultingPower;

		switch (_motorstate)
		{
		case arming:
			LOGGER_NOTICE_FMT("Motor arming %d ", _motor_address);
			break;

		case off:
			LOGGER_NOTICE_FMT("Motor off %d ", _motor_address);
			_power = 0;
			LOGGER_VERBOSE("Not implemented yet");
			
			break;

		case on:
			LOGGER_NOTICE_FMT("Motor on %d ", _motor_address);
			resultingPower = _power;
			if (resultingPower < BASE_MOTOR_POWER) {
				resultingPower = BASE_MOTOR_POWER;
			}
			LOGGER_VERBOSE("Not implemented yet");
			// _motor.write((resultingPower * 10) + 0);
			break;
		}
		LOGGER_VERBOSE("....leave");
	} //-------------------------- end of updateState ----------------------------------------------

	/* Motor is attached with a PIN and set the power to 2000 ms.
	 * After this set the power to 1000 ms and the arming is finished.*/

	void armingProcedure(bool step)
	{
		LOGGER_VERBOSE("Enter....");
		if (_motorstate == arming)
		{
			if (!step)
			{
				LOGGER_NOTICE_FMT("Arming begin max: %d", ARM_MAX);
				LOGGER_VERBOSE("Not implemented yet");
				//	_motor.write(180);		///< Arming begin
			}
			else
			{
				//	_motor.write(0);		///< Arming end
				LOGGER_NOTICE_FMT("Arming fineshed min %d", ARM_MIN);
				LOGGER_VERBOSE("Not implemented yet");
			}
		}
		LOGGER_VERBOSE("....leave");
	} //-------------------------- end of armingProcedure ---------------------------------------------------

	uint16_t getPower() const
	{
		LOGGER_VERBOSE("Enter....");
		return _power;
		LOGGER_VERBOSE("....leave");
	} //-------------------------- end of getPower -------------------------------------------------

	uint16_t setPower(int16_t power)
	{
		LOGGER_VERBOSE("Enter....");
		if (power < 0)
		{
			_power = 0;
		}
		else if (power > _maxPower)
		{
			_power = _maxPower;
		}
		else
		{
			_power = power;
		}
		return _power;
		LOGGER_VERBOSE("....leave");
	} //-------------------------- end of setPower -------------------------------------------------

	uint8_t getMaxPower() const
	{
		LOGGER_VERBOSE("Enter....");
		return _maxPower;
		LOGGER_VERBOSE("....leave");
	} //-------------------------- end of getMaxPower -----------------------------------

	void setMaxPower(uint8_t maxPower)
	{
		LOGGER_VERBOSE("Enter....");
		if (maxPower > 100)
			_maxPower = 100;
		else
			_maxPower = maxPower;
		LOGGER_VERBOSE("....leave");
	} //-------------------------- end of setMaxPower -----------------------------------

	bool isMotorOff()
	{
		LOGGER_VERBOSE("Enter....");
		return (_motorstate == off);
		LOGGER_VERBOSE("....leave");
	} //-------------------------- end of isMotorOn -------------------------------------

	void setMotorStates(motorstate_e state)
	{
		LOGGER_VERBOSE("Enter....");
		_motorstate = state;
		LOGGER_VERBOSE("....leave");
	} //-------------------------- end of setMotorStates --------------------------------
};	  /*--------------------------- end of Motor class --------------------------------*/
