#pragma once
/*  File name : motor.h
	Author: WilhelmKuckelsberg
	Project name : KuCo_Phantom 1
	Date : 2022-06-14

	Description :
*/

#include <Arduino.h>
#include <RP2040_PWM.h>

#define LOCAL_DEBUG
#include "myLogger.h"

/*
#define _PWM_LOGLEVEL_ 0

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
*/

#define POWER_MIN 0			//
#define POWER_MAX 100		//
#define BASE_MOTOR_POWER 10 //< 10% minimal throttle in fly mode for preventing stop of the motors
#define PIN_ESC_ON 15
#define DUTYCYCLE_MIN 40000
#define DUTYCYCLE_MAX 80000

class Motor
{
private:
	float frequency;
	float dutyCycle;
	bool _isArmed;
	
public:
	typedef enum
	{
		arming = 0,
		finished,
		off,
		on
	} motorstate_e;

protected:
	uint8_t _pin;
	RP2040_PWM *_motor;
	uint16_t _power;
	int16_t _maxPower;
	motorstate_e _motorState;
	uint16_t lastPower;

//	uint8_t _motor_address; ///< Gives everyone axis a title

public:
	Motor(uint8_t pin) : _pin(pin)
	{
		_power = 0;
		_maxPower = 100; // %
		_motorState = off;
		_isArmed = false;
		LOGGER_NOTICE_FMT("PIN = %d", _pin);
	}; /*--------------------------------------------------------------*/

	void setup()
	{
		LOGGER_VERBOSE("Enter....");

		frequency = 400.0f;   // Ist das hier richtig?? oder besser als #define

		_motor = new RP2040_PWM(_pin, frequency, DUTYCYCLE_MAX/1000); // 2mS

		if (_motor)
		{
		//	digitalWrite(PIN_ESC_ON, HIGH); // Mail Power für die ESC´s eingeschaltet
			LOGGER_NOTICE_FMT("_motor Pin = %d", _pin);
			_motor->setPWM();
		}

		if (!_motor->setPWM_Int(_pin, frequency, DUTYCYCLE_MAX/1000))
		{
			LOGGER_FATAL_FMT("PWM-Pin %d not known", _pin);
		};

		delay(20);

		LOGGER_VERBOSE("....leave");
	} /*------------------------------- end of setup ----------------------------------*/

	void update()
	{
		LOGGER_VERBOSE("Enter....");
		uint16_t resultingPower;

		switch (_motorState)
		{
		case arming:
			LOGGER_NOTICE_FMT("arming begin %d", _pin);
			_motor->setPWM_Int(_pin, frequency, DUTYCYCLE_MAX); // ESC auf max stellen
			delay(1000);		//kurze Wartezeit
			digitalWrite(PIN_ESC_ON, HIGH);	// ESC´s einschalten der PIN wird 4 mal auf HIGH gesetzt
			delay(1000);		//kurze Wartezeit für DEBUG
			_motorState = finished;
			break;

		case finished:
			LOGGER_NOTICE("Arming is fineshed");
			_motor->setPWM_Int(_pin, frequency, DUTYCYCLE_MIN); // ESC auf min stellen
																// und dann auf die Melodie warten
			_isArmed = true;
			break;

		case off:
			LOGGER_NOTICE_FMT("Motor off %d", _pin);
			_power = 0;
			break;

		case on:
			_power = 10; // Test Value
			resultingPower = map(_power, 0, 100, DUTYCYCLE_MIN, DUTYCYCLE_MAX);
			if (resultingPower < BASE_MOTOR_POWER)
			{
				resultingPower = BASE_MOTOR_POWER;
			}

			LOGGER_NOTICE_FMT_CHK(resultingPower, lastPower, "RC Throttle %d ResultingPower %d PIN %d ", _power, resultingPower, _pin);
			break;
		}

		_motor->setPWM_Int(_pin, frequency, resultingPower);

		LOGGER_VERBOSE("....leave");
	} /*-------------------------- end of updateState ---------------------------------*/

	/* Motor is attached with a PIN and set the power to 2000 ms.
	   After this set the power to 1000 ms and the arming is finished.*/

	// void armingProcedure(bool step)
	// {
	// 	LOGGER_VERBOSE("Enter....");
	// 	if (_motorState == arming)
	// 	{
	// 		if (!step)
	// 		{
	// 			LOGGER_NOTICE("Arming begin max.");
	// 			_motor->setPWM_Int(_pin, frequency, DUTYCYCLE_MAX);
	// 		}
	// 		else
	// 		{
	// 			_motor->setPWM_Int(_pin, frequency, DUTYCYCLE_MIN);
	// 			LOGGER_NOTICE("Arming fineshed min.");
	// 		}
	// 	}
	// 	LOGGER_VERBOSE("....leave");
	// } /*-------------------------- end of armingProcedure -----------------------------*/

	uint16_t getPower() const
	{
		LOGGER_VERBOSE("Enter....");
		return _power;
		LOGGER_VERBOSE("....leave");
	} /*-------------------------- end of getPower ------------------------------------*/

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
			LOGGER_NOTICE_FMT("setPower %d ", _power);
		}
		return _power;
		LOGGER_VERBOSE("....leave");
	} /*-------------------------- end of setPower ------------------------------------*/

	uint8_t getMaxPower() const
	{
		LOGGER_VERBOSE("Enter....");
		return _maxPower;
		LOGGER_VERBOSE("....leave");
	} /*-------------------------- end of getMaxPower ---------------------------------*/

	void setMaxPower(uint8_t maxPower)
	{
		LOGGER_VERBOSE("Enter....");
		if (maxPower > 100)
			_maxPower = 100;
		else
			_maxPower = maxPower;
		LOGGER_VERBOSE("....leave");
	} /*-------------------------- end of setMaxPower ---------------------------------*/

	bool isArmed(){
		return _isArmed;
	}
	
	bool isMotorOff()
	{
		LOGGER_VERBOSE("Enter....");
		return (_motorState == off);
		LOGGER_VERBOSE("....leave");
	} /*-------------------------- end of isMotorOff ----------------------------------*/

	motorstate_e getMotorState()
	{
		LOGGER_VERBOSE("Enter....");
		return _motorState;
	} /*-------------------------- end of getMotorState ------------------------------*/

	void setMotorState(motorstate_e state)
	{
		LOGGER_VERBOSE("Enter....");
		_motorState = state;
		LOGGER_VERBOSE("....leave");
	} /*-------------------------- end of setMotorStates ------------------------------*/
};	  /*--------------------------- end of Motor class --------------------------------*/
