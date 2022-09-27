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

#define PID_FREQUENCY      50			///< PID parameter
#define PID_P_MIN			0.00390626	///< The parameter P domain is [0.00390625 to 255] inclusive.
#define PID_EEPROM_ADRRESS 50

typedef struct
{
	float pidCoefficient[3];	// 12 bytes
	float executionFrequency;	// 4
	int output_bits;			// 4	
	bool output_signed;			// 1   zusammen 21 Byte
} pidData_t;

class NewPID : public FastPID
{ 

private:
	typedef struct{
		float kP;
		float kI;
		float kD;
		float exFreq;
	}pidParameter_t;

	pidParameter_t _pidParameter;
	bool _isEnabled;
	String _ParentName;
	int addr;

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

	void init()
	{
		EEPROM.begin(512);	
		addr = 0;
	} /*-------------------------------- end of int -----------------------------------*/	

	void saveParameters(uint16_t addr, pidData_t* data){	
		uint8_t* current = reinterpret_cast<uint8_t*>(data);

		for(uint8_t i=0; i<sizeof(pidData_t); i++){
			//Serial.print("i = ");Serial.println(i);
			EEPROM.write(addr+i,*(current+i));						//Pointer arethmetic
		}
	} /*-------------------------------- end of saveParameters ------------------------*/

	void loadParameters(int addr){

		uint8_t value;
//		Serial.print("sizeof(pidData_t = ");Serial.println(sizeof(pidData_t));
		uint8_t* current = reinterpret_cast<uint8_t*>(&_pidParameter);
			for(uint8_t i=0; i<sizeof(pidData_t); i++){
			//	Serial.print("i = ");Serial.println(*(current+i));
			//	*(current+i) = EEPROM.read((int)(addr+i));
			}		
	//	Serial.println(_pidParameter.kP);
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
		if(_isEnabled){
			enablePID();
		}
	} /*-------------------------------- end of setP ----------------------------------*/

	void setI(float i)
	{
		LOGGER_NOTICE_FMT("setI: %f", i);
		_pidParameter.kI = i;
		if (_pidParameter.kI <= 0)
			_pidParameter.kI = 0;

		if(_isEnabled)
			enablePID();
	} /*-------------------------------- end of setI ----------------------------------*/

	void setD(float d)
	{
		LOGGER_NOTICE_FMT("setD: %f", d);
		_pidParameter.kD = d;
		if (_pidParameter.kD <= 0)
			_pidParameter.kD = 0;

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
