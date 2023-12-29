/* 		File name: newPID.h
 *  	Created: 2022-06-13
 *      Author: Stephan Scholz / Wilhelm Kuckelsberg
 * 		Project: Phantom 1
 *      https://github.com/mike-matera/FastPID
 *
 *      The parameter P domain is [0.00390625 to 255] inclusive.
 *      Last PID parameter
 *      		P		I		D
 *      Pri		0,14	0,18	0,102
 *      Sec		0,14	0,18	0,102
 *      Yaw		0,01	0		0
 */
#pragma once

#include <FastPID.h>

#define LOCAL_DEBUG
#include "myLogger.h"

#include "EEPROM.h"
#include "def.h"

#ifndef EEPROM_OFFSET
	#define EEPROM_OFFSET 0
#endif

typedef struct
{
	float pidCoefficient[4]; // 16 bytes
	uint8_t output_bits;	 // 2			""
	bool output_signed;		 // 1   		""
	bool modified;			 // 1   must be set if the parameters were changed manually
} pidData_t;

static pidData_t initPid = {{0.10f, 0.15f, 0.08f, 50.0f}, 8, false, false};

class NewPID : public FastPID
{

private:
	pidData_t _pidData;
	bool _isEnabled;
	String _ParentName;
	uint16_t _eepromAddress;
	float temp;

protected:
	float RC_SP;
	float FB;

public:
	NewPID(String name, uint16_t eepromAddress)
	{
		LOGGER_VERBOSE("Enter....");
		_ParentName = name;
		_eepromAddress = eepromAddress;
		_isEnabled = false;
		this->setOutputRange(-100, 100);
		this->setOutputConfig(PID_OUTPUT_BITS, PID_OUTPUT_SIGNED);
		loadParameters();
		if (eepromAddress == 0){
			LOGGER_NOTICE("Init EEPROM");
			EEPROM.begin(512);
		}
		LOGGER_VERBOSE("....leave");
	} /*-------------------------------- end of constructor ------------------------------------*/

	void saveParameters()
	{
	LOGGER_NOTICE("Enter....");
	
		saveParameters(&_pidData);

	LOGGER_NOTICE("....leave");
	} /*-------------------------------- end of saveParameters ---------------------------------*/

	void saveParameters(pidData_t *data)
	{
	LOGGER_NOTICE("Enter....");

		LOGGER_NOTICE_FMT("sizeof pidData_t = %i addr = %i", sizeof(pidData_t), _eepromAddress);
		uint8_t *current = reinterpret_cast<uint8_t *>(data);

		for (uint8_t i = 0; i < sizeof(pidData_t); i++)
		{
			EEPROM.write(EEPROM_OFFSET + _eepromAddress + i, *(current + i)); // Pointer arethmetic
			LOGGER_NOTICE_FMT("addr = %i current %i", EEPROM_OFFSET + _eepromAddress, (*(current+i)));
		}

		LOGGER_NOTICE_FMT("_pidData kP = %.2f", _pidData.pidCoefficient[pidCoefficient::kP]);
		LOGGER_NOTICE_FMT("_pidData kI = %.2f", _pidData.pidCoefficient[pidCoefficient::kI]);
		LOGGER_NOTICE_FMT("_pidData kD = %.2f", _pidData.pidCoefficient[pidCoefficient::kD]);
		LOGGER_NOTICE_FMT("_pidData eF = %.2f", _pidData.pidCoefficient[pidCoefficient::eF]);

		if (EEPROM.commit())
		{
			LOGGER_NOTICE("EEPROM successfully committed");
		}
		else
		{
			LOGGER_FATAL("ERROR! EEPROM commit failed");
		}

	LOGGER_NOTICE("....leave");
	} /*-------------------------------- end of saveParameters ---------------------------------*/

	void loadParameters()
	{
	LOGGER_NOTICE("Enter....");

		LOGGER_NOTICE_FMT("sizeof pidData_t = %i addr = %i", sizeof(pidData_t), _eepromAddress);

		uint8_t *current = reinterpret_cast<uint8_t *>(&_pidData); // current zeigt auf die gleiche Speicherstelle wie _pidData
																   // der datentyp _pidData wird in eine uint8 typ geändert
		
		for (uint8_t i = 0; i < sizeof(pidData_t); i++)
		{
			*(current + i) = EEPROM.read((EEPROM_OFFSET + _eepromAddress + i));
			LOGGER_NOTICE_FMT("i = %i ADDR = %i", (uint8_t) * (current + i), EEPROM_OFFSET + _eepromAddress);
		}

		LOGGER_NOTICE_FMT("_pidData kP = %.2f", _pidData.pidCoefficient[pidCoefficient::kP]);
		LOGGER_NOTICE_FMT("_pidData kI = %.2f", _pidData.pidCoefficient[pidCoefficient::kI]);
		LOGGER_NOTICE_FMT("_pidData kD = %.2f", _pidData.pidCoefficient[pidCoefficient::kD]);
		LOGGER_NOTICE_FMT("_pidData eF = %.2f", _pidData.pidCoefficient[pidCoefficient::eF]);

	LOGGER_NOTICE("....leave");
	} /*-------------------------------- end of loadParameters ----------------------------------*/

	void initPID(){
		LOGGER_NOTICE("Enter..");
		saveParameters(&initPid);
		//saveParameters();
		loadParameters();
		LOGGER_NOTICE(".. leave");
	} /*-------------------------------- end of initPID -----------------------------------------*/

	void disablePID()
	{
	LOGGER_NOTICE("Enter....");

		LOGGER_NOTICE_FMT("Disabled PID controller %s ", _ParentName.c_str());
		this->setCoefficients(PID_P_MIN, 0.0, 0.0, 1000);
	 	_isEnabled = false;

	LOGGER_NOTICE("....leave");	
	} /*-------------------------------- end of deactivatePID ----------------------------------*/

	void enablePID()
	{
	LOGGER_NOTICE("Enter....");

		/* This function has 2 tasks.
		 * 1. The PID parameters are uploaded from the PID adjustment.
		 * 2. The PID parameters are activated. */
		LOGGER_NOTICE("enablePID");
		this->setCoefficients(_pidData.pidCoefficient[pidCoefficient::kP],
							  _pidData.pidCoefficient[pidCoefficient::kI],
							  _pidData.pidCoefficient[pidCoefficient::kD],
							  _pidData.pidCoefficient[pidCoefficient::eF]);

		_isEnabled = true;

	LOGGER_NOTICE("....leave");
	} /*-------------------------------- end of activatePID ------------------------------------*/
	
	// Methode Overlayed for real disable of PID execution
	int16_t step(int16_t sp, int16_t fb){
		//LOGGER_NOTICE("Enter");
		if(_isEnabled){
			return FastPID::step(sp, fb);
		}
		return 0;
	}

	void setP(float p)
	{
	LOGGER_NOTICE("Enter....");

		LOGGER_NOTICE_FMT("setP: %f ", p);
		if (p <= PID_P_MIN)
		{
			p = PID_P_MIN;
		}

		_pidData.pidCoefficient[pidCoefficient::kP] = p;

		LOGGER_NOTICE_FMT(" _pidCoeff.kP: %.3f", _pidData.pidCoefficient[pidCoefficient::kP]);

		if (_isEnabled)
		{
			enablePID();
		}

	LOGGER_NOTICE("....leave");
	} /*-------------------------------- end of setP -------------------------------------------*/

	void setI(float i)
	{
	LOGGER_NOTICE("Enter....");

		LOGGER_NOTICE_FMT("setI: %f", i);

		_pidData.pidCoefficient[pidCoefficient::kI] = i;

		LOGGER_NOTICE_FMT("_pidCoeff.kI: %.3f", _pidData.pidCoefficient[pidCoefficient::kI]);

		if (_isEnabled)
		{
			enablePID();
		}

	LOGGER_NOTICE("....leave");
	} /*-------------------------------- end of setI -------------------------------------------*/

	void setD(float d)
	{
	LOGGER_NOTICE("Enter....");

		_pidData.pidCoefficient[pidCoefficient::kD] = d;

		LOGGER_NOTICE_FMT("_pidCoeff.kD: %.3f", _pidData.pidCoefficient[pidCoefficient::kD]);

		if (_isEnabled)
		{
			enablePID();
		}

	LOGGER_NOTICE("....leave");
	} /*-------------------------------- end of setD -------------------------------------------*/

	void setEF(float ef)
	{
	LOGGER_NOTICE("Enter....");
		_pidData.pidCoefficient[pidCoefficient::eF] = ef;

		LOGGER_NOTICE_FMT("_pidCoeff.EF: %.3f", _pidData.pidCoefficient[pidCoefficient::eF]);

		if (_isEnabled)
		{
			enablePID();
		}
	LOGGER_NOTICE("....leave");
	} /*-------------------------------- end of setEF -------------------------------------------*/

	float getExecutionTime()
	///< Convert frequency to millis
	{
		LOGGER_NOTICE_FMT("PID getExecutionTime %.3f", (1 / _pidData.pidCoefficient[pidCoefficient::eF]) * 1000);
		return ((1.0 / (float)_pidData.pidCoefficient[pidCoefficient::eF]) * 1000);

	} /*-------------------------------- end of getExecutionTime -------------------------------*/

	float getP() const
	{
	LOGGER_NOTICE("Enter....");

		LOGGER_NOTICE_FMT("_pidCoeff KP: %.3f", _pidData.pidCoefficient[pidCoefficient::kP]);
		return _pidData.pidCoefficient[pidCoefficient::kP];

	LOGGER_NOTICE("....leave");
	} /*-------------------------------- end of getP -------------------------------------------*/

	float getI() const
	{
	LOGGER_NOTICE("Enter....");

		LOGGER_NOTICE_FMT("_pidCoeff KI: %.3f", _pidData.pidCoefficient[pidCoefficient::kI]);
		return _pidData.pidCoefficient[pidCoefficient::kI];

	LOGGER_NOTICE("....leave");
	} /*-------------------------------- end of getI -------------------------------------------*/

	float getD() const
	{
	LOGGER_NOTICE("Enter....");

		LOGGER_NOTICE_FMT("_pidCoeff KD: %.3f", _pidData.pidCoefficient[pidCoefficient::kD]);
		return _pidData.pidCoefficient[pidCoefficient::kD];

	LOGGER_NOTICE("....leave");
	} /*-------------------------------- end of getD -------------------------------------------*/

	float getEF() const
	{
	LOGGER_NOTICE("Enter....");

		LOGGER_NOTICE_FMT("_pidCoeff.EF: %.3f", _pidData.pidCoefficient[pidCoefficient::eF]);
		return _pidData.pidCoefficient[pidCoefficient::eF];

	LOGGER_NOTICE("....leave");
	} /*-------------------------------- end of getEF ------------------------------------------*/
	void printPidValues(){
		LOGGER_NOTICE_FMT("Axis: %s ", _ParentName.c_str());
		getP();
		getI();
		getD();
		getEF();
	}

}; /*--------------------------- end of newPID class -------------------------------------------*/

