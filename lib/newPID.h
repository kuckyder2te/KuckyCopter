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

#include "myLogger.h"
#include "EEPROM.h"
#include "def.h"

#define PID_FREQUENCY      50			///< PID parameter
#define PID_OUTPUT_BITS	   16
#define PID_OUTPUT_SIGNED  flase	
#define PID_P_MIN			0.00390626	///< The parameter P domain is [0.00390625 to 255] inclusive.
#define PID_EEPROM_ADRRESS 50

typedef struct
{
	float pidCoefficient[4];  // 12 bytes
	uint8_t output_bits;	  // 2			""
	bool output_signed;		  // 1   		""
	bool modified; 			  // 1   muss gesetzt werden wenn die Parameter manuell geändert wurden
} pidData_t;

static pidData_t initPid ={{1.11f, 2.22f, 3.33f, 50.0f}, 8, false, false};
									
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
	NewPID(String name, uint16_t eepromAddress){
		_ParentName = name;
		_eepromAddress = eepromAddress;
		_isEnabled = false;
		this->setOutputRange(-100, 100);
		this->setOutputConfig(16, true);
		loadParameters();
		disablePID();
	} /*-------------------------------- end of constructor ---------------------------*/	

	void saveParameters(){	
		saveParameters(&_pidData);
	} /*-------------------------------- end of saveParameters ------------------------*/

	void saveParameters(pidData_t* data){	

		LOGGER_WARNING_FMT("SavePara - sizeof pidData_t = %i addr = %i", sizeof(pidData_t), _eepromAddress);
		uint8_t* current = reinterpret_cast<uint8_t*>(data);

		for(uint8_t i=0; i<sizeof(pidData_t); i++){		
			EEPROM.write(_eepromAddress+i,*(current+i));						//Pointer arethmetic
		//	LOGGER_WARNING_FMT("Addr = %i current %i", _eepromAddress+1, (*(current+i)));
		}

		if (EEPROM.commit()) {
			LOGGER_WARNING("EEPROM successfully committed");
		} else {
			LOGGER_WARNING("ERROR! EEPROM commit failed");
		}
	//	delay(2000);
	} /*-------------------------------- end of saveParameters ------------------------*/

	void loadParameters(){

		LOGGER_WARNING_FMT("LoadPara - sizeof pidData_t = %i addr = %i", sizeof(pidData_t), _eepromAddress);

		uint8_t* current = reinterpret_cast<uint8_t*>(&_pidData);   // current zeigt auf die gleiche Speicherstelle wie _pidData
																   // der datentyp _pidData wird in eine uint8 typ geändert
			for(uint8_t i=0; i<sizeof(pidData_t); i++){
				*(current+i) = EEPROM.read((_eepromAddress+i));
				LOGGER_WARNING_FMT("i = %i",(uint8_t)*(current+i));
			}

		LOGGER_WARNING_FMT("_pidData.kP = %.2f", _pidData.pidCoefficient[pidCoeffi_e::kP]);
		LOGGER_WARNING_FMT("_pidData.kI = %.2f", _pidData.pidCoefficient[pidCoeffi_e::kI]);
		LOGGER_WARNING_FMT("_pidData.kD = %.2f", _pidData.pidCoefficient[pidCoeffi_e::kD]);
		LOGGER_WARNING_FMT("_pidData.exFreq = %.1f", _pidData.pidCoefficient[pidCoeffi_e::eF]);
		LOGGER_WARNING_FMT("_pidData.Output bits = %i", _pidData.output_bits);
		LOGGER_WARNING_FMT("_pidData.output signed = %i", _pidData.output_signed);

	//	delay(2000);
	} /*-------------------------------- end of loadParameters ------------------------*/

	void disablePID()
	{
		LOGGER_NOTICE_FMT("Disabled PID controller %s ", _ParentName.c_str());
			this->setCoefficients(PID_P_MIN,0.0,0.0,PID_FREQUENCY);

		_isEnabled = false;
	} /*-------------------------------- end of deactivatePID -------------------------*/

	void enablePID()
	{
		/* This function has 2 tasks.
		 * 1. The PID parameters are uploaded from the PID adjustment.
		 * 2. The PID parameters are activated. */
			LOGGER_NOTICE("enablePID");
			this->setCoefficients(_pidData.pidCoefficient[pidCoeffi_e::kP],
								  _pidData.pidCoefficient[pidCoeffi_e::kI],
								  _pidData.pidCoefficient[pidCoeffi_e::kD],
								  _pidData.pidCoefficient[pidCoeffi_e::eF]);

			_isEnabled = true;
	} /*-------------------------------- end of activatePID ---------------------------*/

	void putPID_Data_into_EEPROM(){
	
		LOGGER_VERBOSE("Enter....");

		LOGGER_VERBOSE("....leave");

	} /*-------------------------------- end of putPID_Data_into_EEPROM ---------------*/

	void getPID_Data_into_EEPROM(){
	
		LOGGER_VERBOSE("Enter....");

		LOGGER_VERBOSE("....leave");

	} /*-------------------------------- end of getPID_Data_into_EEPROM ---------------*/

	void setP(float p)
	{		
		LOGGER_WARNING_FMT("setP: %f ", p);
		if (p <= PID_P_MIN){
		 	p = PID_P_MIN;
		}

		_pidData.pidCoefficient[pidCoeffi_e::kP] = p;

		LOGGER_WARNING_FMT(" _pidCoeff.kP: %.3f", _pidData.pidCoefficient[pidCoeffi_e::kP]);

		if(_isEnabled){
			enablePID();
		}
		delay(100);
	} /*-------------------------------- end of setP ----------------------------------*/

	void setI(float i)
	{
		LOGGER_NOTICE_FMT("setI: %f", i);

		_pidData.pidCoefficient[pidCoeffi_e::kI] = i;

		LOGGER_WARNING_FMT("_pidCoeff.kI: %.3f", _pidData.pidCoefficient[pidCoeffi_e::kI]);

		if(_isEnabled)
			enablePID();
	} /*-------------------------------- end of setI ----------------------------------*/

	void setD(float d)
	{
		_pidData.pidCoefficient[pidCoeffi_e::kI] = d;

		LOGGER_WARNING_FMT("_pidCoeff.kD: %.3f", _pidData.pidCoefficient[pidCoeffi_e::kD]);	

		if(_isEnabled)
			enablePID();
	} /*-------------------------------- end of setD ----------------------------------*/

	void setEF(float ef)
	{
		_pidData.pidCoefficient[pidCoeffi_e::eF] = ef;

		LOGGER_WARNING_FMT("_pidCoeff.EF: %.3f", _pidData.pidCoefficient[pidCoeffi_e::eF]);	
		
		if(_isEnabled)
			enablePID();
	} /*-------------------------------- end of setEF ----------------------------------*/

	float getExecutionTime()
	{
		LOGGER_NOTICE_FMT("PID getExecutionTime %.3f", (1 / _pidData.pidCoefficient[pidCoeffi_e::eF]) * 1000);
		return  ((1.0/(float)_pidData.pidCoefficient[pidCoeffi_e::eF])*1000);
		///< Convert frequency to millis
	} /*-------------------------------- end of getExecutionTime ----------------------*/

	void updateEEPROM(void)
	{
	//	EEPROM.read(addr, _pidParameter);
		enablePID();
	} /*-------------------------------- end of updateEEPROM --------------------------*/

	void readEEPROM(void)
	{
	//	EEPROM.write(addr, _pidParameter);
		enablePID();
	} /*-------------------------------- end of readEEPROM ----------------------------*/

	float getP() const
	{
		LOGGER_WARNING_FMT("_pidCoeff.KP: %.3f", _pidData.pidCoefficient[pidCoeffi_e::kP]);
		return _pidData.pidCoefficient[pidCoeffi_e::kP];
	} /*-------------------------------- end of getP ----------------------------------*/

	float getI() const
	{
		LOGGER_WARNING_FMT("_pidCoeff.KI: %.3f", _pidData.pidCoefficient[pidCoeffi_e::kI]);
		return _pidData.pidCoefficient[pidCoeffi_e::kI];
	} /*-------------------------------- end of getI ----------------------------------*/

	float getD() const
		{LOGGER_WARNING_FMT("_pidCoeff.KP: %.3f", _pidData.pidCoefficient[pidCoeffi_e::eF]);
		return _pidData.pidCoefficient[pidCoeffi_e::kD];
	} /*-------------------------------- end of getD ----------------------------------*/

	float getEF() const
	{
		LOGGER_WARNING_FMT("_pidCoeff.EF: %.3f", _pidData.pidCoefficient[pidCoeffi_e::eF]);
		return _pidData.pidCoefficient[pidCoeffi_e::eF];
	} /*-------------------------------- end of getEF ---------------------------------*/

};/*--------------------------- end of MyPid class ------------------------------------*/

//#undef _DEBUG_
