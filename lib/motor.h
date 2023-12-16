#pragma once
/*  File name : motor.h
	Author: Stephan Scholz / WilhelmKuckelsberg
	Project name : KuCo_Phantom 1
	Date : 2022-06-14

	Description :
*/

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

#include <Arduino.h>
#include <RP2040_PWM.h>

//#define LOCAL_DEBUG
#include "myLogger.h"
#include "def.h"

class Motor
{
private:
	float frequency;
	float dutyCycle;
	bool _isArmed;
	unsigned long _lastMillis;
	
public:
	typedef enum
	{
		arming = 0,
		power_on,
		power_off,
		busy,
		finished,
		stop,
		rotating
	} motorstate_e;

protected:
	uint8_t _pin;
	RP2040_PWM *_motor;
	uint32_t resultingPower, _resultingPower, _lastResultingPower;
	int16_t _power,_lastPower,_maxPower;
	motorstate_e _motorState,_lastMotorState;

public:
	Motor(uint8_t pin) : _pin(pin)
	{
		_power = 0;
		_maxPower = 100; // %
		_motorState = stop;
		_isArmed = false;
		LOGGER_NOTICE_FMT("PIN = %d", _pin);
	}; /*--------------------------------------------------------------*/

	void setup()
	{
		LOGGER_VERBOSE("Enter....");

		frequency = 400.0f;   // Ist das hier richtig?? oder besser als #define
		_motor = nullptr;
		_motor = new RP2040_PWM(_pin, frequency, DUTYCYCLE_MAX/1000); // 2mS

		if (_motor)
		{
			LOGGER_NOTICE_FMT("_motor Pin = %d", _pin);
			_motor->setPWM();
		}else{
			LOGGER_FATAL_FMT("PWM-Pin %d not known", _pin);
		};

		delay(20);

		LOGGER_VERBOSE("....leave");
	} /*------------------------------- end of setup ----------------------------------*/

	void update()
	{
		LOGGER_VERBOSE("Enter....");
		if(_motorState!=_lastMotorState){
			LOGGER_VERBOSE_FMT("*******New State to Update - Pin %d ********",_pin);
		}
		switch (_motorState)
		{
		case arming:
			LOGGER_NOTICE("arming begin");
			resultingPower = DUTYCYCLE_MAX;
			_motorState = power_on;
			break;
		case power_on:
			delay(20);
			digitalWrite(PIN_ESC_ON, LOW);	// ESC´s switch on (BC547)
			_motorState = busy;
			_lastMillis = millis();
			break;
		case power_off:
			digitalWrite(PIN_ESC_ON, HIGH);	// ESC´s turn off (BC547)
			_isArmed = false;
			break;
		case busy:
			LOGGER_NOTICE_CHK(_motorState,_lastMotorState,"Arming is busy");
			if(millis() - _lastMillis > 2000){
				_motorState = finished;
				resultingPower = DUTYCYCLE_MIN;
			//	LOGGER_NOTICE_FMT_CHK(resultingPower,_resultingPower,"busy - resultingPower %i", resultingPower);
				_lastMillis = millis();
			}
			break;
		case finished:
			LOGGER_NOTICE_CHK(_motorState,_lastMotorState,"Arming is finished");
			if(millis() - _lastMillis > 1000){
				_isArmed = true;
				_lastMillis = millis();
				_motorState = stop;
			}
			resultingPower = DUTYCYCLE_MIN;
			//LOGGER_NOTICE_FMT_CHK(resultingPower,_resultingPower,"finished - resultingPower %i", _resultingPower);
			break;

		case stop:
			LOGGER_NOTICE_CHK(_motorState,_lastMotorState,"Motor off");
			_power = 0;
			resultingPower = DUTYCYCLE_MIN;
			//LOGGER_NOTICE_FMT_CHK(resultingPower,_resultingPower,"stop - resultingPower %i", _resultingPower);
			break;

		case rotating:
			LOGGER_NOTICE_CHK(_motorState,_lastMotorState,"Motor on");
			resultingPower = map(_power, 0, 100, DUTYCYCLE_MIN, DUTYCYCLE_MAX);
			if (resultingPower < map(BASE_MOTOR_POWER, 0, 100, DUTYCYCLE_MIN, DUTYCYCLE_MAX))
			{
				resultingPower = map(BASE_MOTOR_POWER, 0, 100, DUTYCYCLE_MIN, DUTYCYCLE_MAX);
			}
			LOGGER_NOTICE_FMT_CHK(resultingPower, _resultingPower, "RC Throttle %d ResultingPower %d", _power, resultingPower);
			LOGGER_NOTICE_FMT_CHK(resultingPower,_resultingPower,"rotating - resultingPower %i", resultingPower);
			break;
		}
		if(_lastResultingPower!=resultingPower){
			LOGGER_NOTICE_FMT("resultingPower = %d - Pin: %d",resultingPower,_pin);
			_motor->setPWM_Int(_pin, frequency, resultingPower);
			_lastResultingPower= resultingPower;
		}

		if((digitalRead(PIN_ESC_ON)==HIGH) && (_motorState!=power_on)){
			_motorState= power_off;
		}
		LOGGER_VERBOSE("....leave");
	} /*-------------------------- end of updateState ----------------------------------------------*/

	uint16_t getPower() const
	{
		LOGGER_VERBOSE("Enter....");
		return _power;	
		LOGGER_VERBOSE("....leave");
	} /*-------------------------- end of getPower -------------------------------------------------*/

	String getResultingPower() const
	{
		char buffer1[20];
		char buffer2[20];
		LOGGER_VERBOSE("Enter....");
		String One = "Resultingpower ";
		String ResultingPower = "Resultingpower " + _resultingPower;
		String Pin = " on Pin " +_pin;
		String Output =ResultingPower +Pin;
		return Output;	
		LOGGER_VERBOSE("....leave");
	} /*-------------------------- end of getResultingPower --------------------------------------*/

	uint16_t setPower(int16_t power)
	{
		LOGGER_VERBOSE("Enter....");
		if (power < 0)
		{
			_power = 0;
			LOGGER_NOTICE_FMT("Power = %d < 0 - Pin: %d",power,_pin);
		}
		else if (power > _maxPower)
		{
			_power = _maxPower;
			LOGGER_NOTICE_FMT("Power = %d over maxPower - Pin: %d",power,_pin);
		}
		else
		{
			_power = power;			
		}
		LOGGER_NOTICE_FMT_CHK(_power,_lastPower,"setPower %d - Pin: %d", _power,_pin);
		LOGGER_VERBOSE("....leave");
		return _power;
	} /*-------------------------- end of setPower ---------------------------------------------*/
	uint16_t getPower(){
		return _power;
	}
	uint8_t getMaxPower() const
	{
		LOGGER_VERBOSE("Enter....");
		return _maxPower;
		LOGGER_VERBOSE("....leave");
	} /*-------------------------- end of getMaxPower ------------------------------------------*/

	void setMaxPower(uint8_t maxPower)
	{
		LOGGER_VERBOSE("Enter....");
		if (maxPower > 100)
			_maxPower = 100;
		else
			_maxPower = maxPower;
		LOGGER_VERBOSE("....leave");
	} /*-------------------------- end of setMaxPower ------------------------------------------*/

	bool isArmed(){
		return _isArmed;
	} /*-------------------------- end of isArmed ----------------------------------------------*/
	
	bool isMotorOff()
	{
		LOGGER_VERBOSE("Enter....");
		return (_motorState == stop);
		LOGGER_VERBOSE("....leave");
	} /*-------------------------- end of isMotorOff -------------------------------------------*/

	motorstate_e getMotorState()
	{
		LOGGER_VERBOSE("Enter....");
		return _motorState;
	} /*-------------------------- end of getMotorState ---------------------------------------*/

	void setMotorState(motorstate_e state)
	{
		LOGGER_VERBOSE("Enter....");
		_motorState = state;
		LOGGER_VERBOSE("....leave");
	} /*-------------------------- end of setMotorStates ---------------------------------------*/
	
};	  /*--------------------------- end of Motor class -----------------------------------------*/
