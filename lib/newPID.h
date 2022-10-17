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
#include "..\..\EEPROM\EEPROM.h"   // Refernce to framework  1.. is Root 2.. ins framework
//#include "..\lib\model.h"

#define PID_FREQUENCY      50			///< PID parameter
#define PID_P_MIN			0.00390626	///< The parameter P domain is [0.00390625 to 255] inclusive.
#define PID_EEPROM_ADRRESS 50

typedef struct
{
	float pidCoefficient[3];  // 12 bytes
	float executionFrequency; // 4
	int output_bits;		  // 4
	bool output_signed;		  // 1   zusammen 21 Byte
} pidData_t;

typedef struct
{
	float kP;
	float kI;
	float kD;
	float exFreq;
	bool modified; // muss gesetzt werden wenn die Parameter manuell geändert wurden
} pidData_TEST_t;

typedef struct
{
	float kP;
	float kI;
	float kD;
	float exFreq;
	bool modified; // muss gesetzt werden wenn die Parameter manuell geändert wurden
} pidParameter_t;

static pidParameter_t initPid = {1.11, 2.22, 3.33, 50, false};

class NewPID : public FastPID
{ 

private:	
	pidParameter_t _pidParameter;
//	pidData_TEST_t _pidData_TEST;
	pidData_t pidData;
	bool _isEnabled;
	String _ParentName;

protected:
	float RC_SP;
	float FB;

public:
	NewPID(String name){
		_ParentName = name;
		_isEnabled = false;
		this->setOutputRange(-100, 100);
		this->setOutputConfig(16, true);
		_pidParameter.kP = 0;
		_pidParameter.exFreq = 0;
		disablePID();
	} /*-------------------------------- end of constructor ---------------------------*/

	void init(uint8_t instance)
	{
		static int count = 0;
		Serial2.print("NewPID init ");Serial2.println(count);

		uint8_t start = instance * sizeof(pidParameter_t);
		uint8_t size = sizeof(pidParameter_t);
		LOGGER_WARNING_FMT("Startadresse %i Instance %i Size %i", start, instance, size);
	//	loadParameters(start);
		saveParameters(start, &initPid);
		//if(!_pidParameter.modified){
		// 	saveParameters(start, &initPid);
		// 	// for(int i=start;i<(sizeof(pidData_t)+start);i++){
		// 	// 	EEPROM.write(i, instance);
		// 	// }
		// }
		count++;

	//	float test = pidData[0].pid
	//	Serial2.print("test ");Serial2.println(test);

		delay(10000);
	} /*-------------------------------- end of int -----------------------------------*/	

	void saveParameters(uint16_t addr, pidParameter_t* data){	
		static int count = 0;
		Serial2.print("saveParameters ");Serial2.println(count);

		uint8_t* current = reinterpret_cast<uint8_t*>(data);

		for(uint8_t i=0; i<sizeof(pidParameter_t); i++){
			LOGGER_WARNING_FMT("i = %i", i);
			EEPROM.write(addr+i,*(current+i));						//Pointer arethmetic
		}
		    if (EEPROM.commit()) {
      			LOGGER_WARNING("EEPROM successfully committed");
    		} else {
	      		LOGGER_WARNING("ERROR! EEPROM commit failed");
    		}
		count++;
	} /*-------------------------------- end of saveParameters ------------------------*/

	void loadParameters(int addr){
		static int count = 0;
		Serial2.print("loadParameters ");Serial2.println(count);

		LOGGER_WARNING_FMT("sizeof pidData_t = %i", sizeof(pidParameter_t));
		for (int i = 0; i < 80; i++) {
    		Serial2.println(EEPROM.read(i));		
		}

		uint8_t* current = reinterpret_cast<uint8_t*>(&_pidParameter);
			for(uint8_t i=0; i<sizeof(pidParameter_t); i++){
				*(current+i) = EEPROM.read((addr+i));
				LOGGER_WARNING_FMT("i = %i",*(current+i));
			}		
		LOGGER_WARNING_FMT("_pidParameter.kP = %f", (float)_pidParameter.kP);
		LOGGER_WARNING_FMT("_pidParameter.kI = %f", (float)_pidParameter.kI);
		LOGGER_WARNING_FMT("_pidParameter.kD = %f", (float)_pidParameter.kD);
		LOGGER_WARNING_FMT("_pidParameter.exFreq = %f", (float)_pidParameter.exFreq);
		count++;
	} /*-------------------------------- end of loadParameters ------------------------*/

	void disablePID()
	{
		LOGGER_NOTICE_FMT("Disabled PID controller %s ", _ParentName.c_str());
			this->setCoefficients(_pidParameter.kP,0.0,0.0,_pidParameter.exFreq);
			_isEnabled = false;
	} /*-------------------------------- end of deactivatePID -------------------------*/

	void enablePID()
	{
		/* This function has 2 tasks.
		 * 1. The PID parameters are uploaded from the PID adjustment.
		 * 2. The PID parameters are activated. */
			LOGGER_NOTICE("enablePID");
			this->setCoefficients(_pidParameter.kP,_pidParameter.kI,_pidParameter.kD,_pidParameter.exFreq);
		_isEnabled = true;
	} /*-------------------------------- end of activatePID ---------------------------*/

	void setP(float p)
	{
		LOGGER_NOTICE_FMT("setP: %f", p);
		_pidParameter.kP = p;
		if (_pidParameter.kP <= PID_P_MIN){
			_pidParameter.kP = PID_P_MIN;
		}
		LOGGER_NOTICE_FMT("_pidParameter.kP: %f", _pidParameter.kP);
		if(_isEnabled){
			enablePID();

		}
	} /*-------------------------------- end of setP ----------------------------------*/

	void setI(float i)
	{
		LOGGER_NOTICE_FMT("setI: %f", i);
		_pidParameter.kI = i;
		if (_pidParameter.kI <= 0){
			_pidParameter.kI = 0;
		}
		LOGGER_NOTICE_FMT("_pidParameter.kI: %f", _pidParameter.kI);
		if(_isEnabled)
			enablePID();
	} /*-------------------------------- end of setI ----------------------------------*/

	void setD(float d)
	{
		LOGGER_NOTICE_FMT("setD: %f", d);
		_pidParameter.kD = d;
		if (_pidParameter.kD <= 0){
			_pidParameter.kD = 0;
		}
		LOGGER_NOTICE_FMT("_pidParameter.kD: %f", _pidParameter.kD);		
		if(_isEnabled)
			enablePID();
	} /*-------------------------------- end of setD ----------------------------------*/

	void setExecutionFrequency(uint8_t ef)
	{
		LOGGER_NOTICE_FMT("setExecutionFrequency: %d", ef);
		_pidParameter.exFreq = ef;
		if(_isEnabled)
			enablePID();
	} /*-------------------------------- end of setExecutionFrequency -----------------*/

	uint8_t getExecutionTime()
	{
		LOGGER_NOTICE_FMT("PID getExecutionTime %f", (1 / _pidParameter.exFreq) * 1000);
		return ((1.0/(float)_pidParameter.exFreq)*1000);
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
		return _pidParameter.kP;
	} /*-------------------------------- end of getP ----------------------------------*/

	float getI() const
	{
		return _pidParameter.kI;
	} /*-------------------------------- end of getI ----------------------------------*/

	float getD() const
	{
		return _pidParameter.kD;
	} /*-------------------------------- end of getD ----------------------------------*/

	float getExFreq() const
	{
		return _pidParameter.exFreq;
	} /*-------------------------------- end of getExTime -----------------------------*/
};/*--------------------------- end of MyPid class ------------------------------------*/

//#undef _DEBUG_
