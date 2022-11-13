/* 		File name: newPID.h
 *  	Created: 2022-06-13
 *      Author: willy
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

#define LOCAL_DEBUG		// enable = debug this class  /  disable no debug
#include "myLogger.h"

#include "EEPROM.h"
#include "def.h"

#define PID_FREQUENCY 50 ///< PID parameter
// #define PID_SEC_FREQUENCY 51
// #define PID_YAW_FREQUENCY 52

#define PID_OUTPUT_BITS 16
#define PID_OUTPUT_SIGNED false
#define PID_P_MIN 0.00390626 ///< The parameter P domain is [0.00390625 to 255] inclusive.
#define PID_EEPROM_ADRRESS 50

typedef struct
{
	float pidCoefficient[3]; // 12 bytes
//	uint8_t output_bits;	 // 2			""
///	bool output_signed;		 // 1   		""
//	bool modified;			 // 1   muss gesetzt werden wenn die Parameter manuell geändert wurden
} pidData_t;

//static pidData_t initPid = {{1.11f, 2.22f, 3.33f, 50.0f}, 8, false, false};

class NewPID : public FastPID
{

private:
	pidData_t _pidData;
	bool _isEnabled;
	String _ParentName;
	uint16_t _eepromAddress;

protected:
	float RC_SP;
	float FB;

public:
	NewPID(String name, uint16_t eepromAddress)
	{
		_ParentName = name;
		_eepromAddress = eepromAddress;
		_isEnabled = false;
		this->setOutputRange(-100, 100);
		this->setOutputConfig(PID_OUTPUT_BITS, PID_OUTPUT_SIGNED);
		loadParameters();
	} /*-------------------------------- end of constructor ---------------------------*/

	void saveParameters()
	{
	LOGGER_NOTICE("Enter....");
	
		saveParameters(&_pidData);

	LOGGER_VERBOSE("....leave");
	} /*-------------------------------- end of saveParameters ------------------------*/

	void saveParameters(pidData_t *data)
	{
	LOGGER_VERBOSE("Enter....");

		LOGGER_NOTICE_FMT("sizeof pidData_t = %i addr = %i", sizeof(pidData_t), _eepromAddress);
		uint8_t *current = reinterpret_cast<uint8_t *>(data);

		for (uint8_t i = 0; i < sizeof(pidData_t); i++)
		{
			EEPROM.write(_eepromAddress + i, *(current + i)); // Pointer arethmetic
			LOGGER_NOTICE_FMT("Addr = %i current %i", _eepromAddress+1, (*(current+i)));
		}

		if (EEPROM.commit())
		{
			LOGGER_NOTICE("EEPROM successfully committed");
		}
		else
		{
			LOGGER_FATAL("ERROR! EEPROM commit failed");
		}

	LOGGER_VERBOSE("....leave");
	} /*-------------------------------- end of saveParameters ------------------------*/

	void loadParameters()
	{
	LOGGER_VERBOSE("Enter....");

		LOGGER_NOTICE_FMT("sizeof pidData_t = %i addr = %i", sizeof(pidData_t), _eepromAddress);
		// delay(10000);
		// LOGGER_NOTICE("Stop");
		uint8_t *current = reinterpret_cast<uint8_t *>(&_pidData); // current zeigt auf die gleiche Speicherstelle wie _pidData
																   // der datentyp _pidData wird in eine uint8 typ geändert
		

		for (uint8_t i = 0; i < sizeof(pidData_t); i++)
		{
			*(current + i) = EEPROM.read((_eepromAddress + i));
	//		LOGGER_NOTICE_FMT("i = %i", (uint8_t) * (current + i));
		}

		LOGGER_NOTICE_FMT("_pidData kP = %.2f", _pidData.pidCoefficient[pidCoeffi_e::kP]);
		LOGGER_NOTICE_FMT("_pidData kI = %.2f", _pidData.pidCoefficient[pidCoeffi_e::kI]);
		LOGGER_NOTICE_FMT("_pidData kD = %.2f", _pidData.pidCoefficient[pidCoeffi_e::kD]);

	LOGGER_VERBOSE("....leave");
	} /*-------------------------------- end of loadParameters ------------------------*/

	void disablePID()
	{
	LOGGER_VERBOSE("Enter....");

		// LOGGER_NOTICE_FMT("Disabled PID controller %s ", _ParentName.c_str());
		// this->setCoefficients(PID_P_MIN, 0.0, 0.0, PID_FREQUENCY);

		// _isEnabled = false;

	LOGGER_VERBOSE("....leave");	
	} /*-------------------------------- end of deactivatePID -------------------------*/

	void enablePID()
	{
	LOGGER_VERBOSE("Enter....");

		/* This function has 2 tasks.
		 * 1. The PID parameters are uploaded from the PID adjustment.
		 * 2. The PID parameters are activated. */
		LOGGER_NOTICE("enablePID");
		this->setCoefficients(_pidData.pidCoefficient[pidCoeffi_e::kP],
							  _pidData.pidCoefficient[pidCoeffi_e::kI],
							  _pidData.pidCoefficient[pidCoeffi_e::kD],
							  PID_FREQUENCY);

		_isEnabled = true;

	LOGGER_VERBOSE("....leave");
	} /*-------------------------------- end of activatePID ---------------------------*/

	// void putPID_Data_into_EEPROM()
	// {
	// LOGGER_NOTICE("Enter....");
		
	// LOGGER_VERBOSE("....leave");
	// } /*-------------------------------- end of putPID_Data_into_EEPROM ---------------*/

	// void getPID_Data_from_EEPROM()
	// {
	// LOGGER_NOTICE("Enter....");
	
	// LOGGER_VERBOSE("....leave");
	// } /*-------------------------------- end of getPID_Data_into_EEPROM ---------------*/

	void setP(float p)
	{
	LOGGER_VERBOSE("Enter....");

		LOGGER_NOTICE_FMT("setP: %f ", p);
		if (p <= PID_P_MIN)
		{
			p = PID_P_MIN;
		}

		_pidData.pidCoefficient[pidCoeffi_e::kP] = p;

		LOGGER_NOTICE_FMT(" _pidCoeff.kP: %.3f", _pidData.pidCoefficient[pidCoeffi_e::kP]);

		if (_isEnabled)
		{
			enablePID();
		}

	LOGGER_VERBOSE("....leave");
	} /*-------------------------------- end of setP ----------------------------------*/

	void setI(float i)
	{
	LOGGER_VERBOSE("Enter....");

		LOGGER_NOTICE_FMT("setI: %f", i);

		_pidData.pidCoefficient[pidCoeffi_e::kI] = i;

		LOGGER_NOTICE_FMT("_pidCoeff.kI: %.3f", _pidData.pidCoefficient[pidCoeffi_e::kI]);

		if (_isEnabled)
		{
			enablePID();
		}

	LOGGER_VERBOSE("....leave");
	} /*-------------------------------- end of setI ----------------------------------*/

	void setD(float d)
	{
	LOGGER_VERBOSE("Enter....");

		_pidData.pidCoefficient[pidCoeffi_e::kD] = d;

		LOGGER_NOTICE_FMT("_pidCoeff.kD: %.3f", _pidData.pidCoefficient[pidCoeffi_e::kD]);

		if (_isEnabled)
		{
			enablePID();
		}

	LOGGER_VERBOSE("....leave");
	} /*-------------------------------- end of setD ----------------------------------*/

	void setEF(float ef)
	{
	LOGGER_VERBOSE("Enter....");
		_pidData.pidCoefficient[pidCoeffi_e::eF] = ef;

		LOGGER_NOTICE_FMT("_pidCoeff.EF: %.3f", _pidData.pidCoefficient[pidCoeffi_e::eF]);

		if (_isEnabled)
		{
			enablePID();
		}//

	LOGGER_VERBOSE("....leave");
	} /*-------------------------------- end of setEF ----------------------------------*/

	float getExecutionTime()
	{
	LOGGER_VERBOSE("Enter....");

	//	LOGGER_NOTICE_FMT("PID getExecutionTime %.3f", (1 / _pidData.pidCoefficient[pidCoeffi_e::eF]) * 1000);
		return ((1.0 / (float)_pidData.pidCoefficient[pidCoeffi_e::eF]) * 1000);
		///< Convert frequency to millis

		
		LOGGER_VERBOSE("....leave");
	} /*-------------------------------- end of getExecutionTime ----------------------*/

	// void updateEEPROM(void)
	// {
	// LOGGER_VERBOSE("Enter....");

	// 	//	EEPROM.read(addr, _pidParameter);
	// 	enablePID();

	// LOGGER_VERBOSE("....leave");
	// } /*-------------------------------- end of updateEEPROM --------------------------*/
	//
	// void readEEPROM(void)
	// {
	// LOGGER_VERBOSE("Enter....");

	// 	//	EEPROM.write(addr, _pidParameter);
	// 	enablePID();

	// LOGGER_VERBOSE("....leave");
	// } /*-------------------------------- end of readEEPROM ----------------------------*/

	float getP() const
	{
	LOGGER_VERBOSE("Enter....");

		LOGGER_NOTICE_FMT("_pidCoeff KP: %.3f", _pidData.pidCoefficient[pidCoeffi_e::kP]);
		return _pidData.pidCoefficient[pidCoeffi_e::kP];

	LOGGER_VERBOSE("....leave");
	} /*-------------------------------- end of getP ----------------------------------*/

	float getI() const
	{
	LOGGER_VERBOSE("Enter....");

		LOGGER_NOTICE_FMT("_pidCoeff KI: %.3f", _pidData.pidCoefficient[pidCoeffi_e::kI]);
		return _pidData.pidCoefficient[pidCoeffi_e::kI];

	LOGGER_VERBOSE("....leave");
	} /*-------------------------------- end of getI ----------------------------------*/

	float getD() const
	{
	LOGGER_VERBOSE("Enter....");

		LOGGER_NOTICE_FMT("_pidCoeff KD: %.3f", _pidData.pidCoefficient[pidCoeffi_e::kD]);
		return _pidData.pidCoefficient[pidCoeffi_e::kD];

	LOGGER_VERBOSE("....leave");
	} /*-------------------------------- end of getD ----------------------------------*/

	float getEF() const
	{
	LOGGER_VERBOSE("Enter....");

		LOGGER_NOTICE_FMT("_pidCoeff.EF: %.3f", _pidData.pidCoefficient[pidCoeffi_e::eF]);
		return _pidData.pidCoefficient[pidCoeffi_e::eF];

	LOGGER_VERBOSE("....leave");
	} /*-------------------------------- end of getEF ---------------------------------*/

}; /*--------------------------- end of newPID class ----------------------------------*/

