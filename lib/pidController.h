#pragma once
/*
 *  File name: model.h
 *	Project name: KuCo_xxx
 *  Date: 2022-005-28 (29.12.2018)
 *  Author: Wilhelm Kuckelsberg
 *  Quelle:  https://github.com/mike-matera/FastPID  
 *
 *      The parameter P domain is [0.00390625 to 255] inclusive.
 *
 *      Last PID parameter
 *
 *      		P		I		D
 *      Pri		0,14	0,18	0,102
 *      Sec		0,14	0,18	0,102
 *      Yaw		0,01	0		0
 *
 */

#include <FastPID.h>
#include <TaskManager.h>
#include <extEEPROM.h>
#include "..\lib\def.h"
//#include "..\lib\model.h"			
#include "myLogger.h"

//------ PID configuration begin --------------------
// #define PID_FREQUENCY        50				///< PID parameter
#define PID_P_MIN			  0.00390626	///< The parameter P domain is [0.00390625 to 255] inclusive.
// #define PID_EEPROM_ADRRESS	 50
// #define PID_ACTIVE_AT		  9
// #define YAW_SENSIBILITY		  5
// #define YAW_FINE_TUNING		  0.1
//------ PID configuration end ----------------------

#ifndef PID_EEPROM_ADRRESS
#define PID_EEPROM_ADRRESS 50
#endif

typedef struct {
	float pidCoefficient[3];
	float executionFrequency;
	int   output_bits;
	bool  output_signed;
} pidData_t;

typedef enum {
	kP,
	kI,
	kD
}coeffizient_t;

class PIDcontroller: public FastPID {			/// MyPid is an FastPit
private:
	float 	  	_kP;
	float       _kI;
	float       _kD;
	float		_exFreq;
	uint8_t     _EEPROM_startAddress;
	uint8_t 	_pidInstance;

protected:
	pidData_t&     _pidData;				/// MyPid has a PidData
	static uint8_t _instance;
	float          RC_SP;
	float          FB;

public:
	PIDcontroller(pidData_t& pidData) : _pidData(pidData),
		FastPID() {	// Call the standard constructor of the base class.
			setOutputRange(-100, 100);
		//	_EEPROM_startAddress = (_instance * sizeof(pidData_t))+PID_EEPROM_ADRRESS;  ///< calculated the the EEPROM address
		//	EEPROM.get(_EEPROM_startAddress, _pidData);	///< Reads the data from the EEPROM directly by the start.
			_pidInstance =_instance++;
			setOutputConfig(16, true);
			disablePID();
		};

	void disablePID(){
	//	Serial.println("MyPID disablePID");
		#ifdef PID_STATE
		LOG("Disabled PID controller %d ", _pidInstance);
		delay(100);
		#endif

		FastPID::setCoefficients(PID_P_MIN, 0.0, 0.0, getExecutionTime());

		#ifdef PID_VALUES
		LOGS(_pidData.pidCoefficient[coeffizient_t::kP]);
		LOGS(_pidData.pidCoefficient[coeffizient_t::kI]);
		LOGS(_pidData.pidCoefficient[coeffizient_t::kD]);
		delay(100);
		#endif

	}//-------------------------------- end of deactivatePID --------------------------------------

	void enablePID(){
		/* This function has 2 tasks.
		 * 1. The PID parameters are uploaded from the PID adjustment.
		 * 2. The PID parameters are activated. */
	//	Serial.println("MyPID enablePID");
		#ifdef PID_STATE
		LOG("Enable PID controller %d ", _pidInstance);
		delay(100);
		#endif

 		FastPID::setCoefficients(_pidData.pidCoefficient[coeffizient_t::kP],
								 _pidData.pidCoefficient[coeffizient_t::kI],
								 _pidData.pidCoefficient[coeffizient_t::kD],
								 getExecutionTime());

		#ifdef PID_VALUES
		LOGS(_pidData.pidCoefficient[coeffizient_t::kP]);
		LOGS(_pidData.pidCoefficient[coeffizient_t::kI]);
		LOGS(_pidData.pidCoefficient[coeffizient_t::kD]);
		delay(100);
		#endif

		#ifdef PID_VALUES_SERIAL
		Serial3.print(_pidData.pidCoefficient[coeffizient_t::kP], 4);
		Serial3.print(" | ");
		Serial3.print(_pidData.pidCoefficient[coeffizient_t::kI], 4);
		Serial3.print(" | ");
		Serial3.println(_pidData.pidCoefficient[coeffizient_t::kD], 4);
		delay(100);
		#endif

}//-------------------------------- end of activatePID --------------------------------------------

	void setP( float p){
		#ifdef PID_STATE
		LOG("setP: %d",p);
		#endif
		_kP = p;
		if(_kP <= PID_P_MIN)
			_kP = PID_P_MIN;

		_pidData.pidCoefficient[coeffizient_t::kP]=_kP;
		Serial3.print("kP ");Serial3.println(_kP);

		enablePID();

	}//-------------------------------- end of setP -----------------------------------------------

	void setI( float i){
		#ifdef PID_STATE
		LOG("setI: %d",i);
		#endif
		_kI = i;
		if(_kI <= 0)
			_kI = 0;

		_pidData.pidCoefficient[coeffizient_t::kI]=_kI;
		Serial3.print("kI ");Serial3.println(_kI);

		enablePID();

	}//-------------------------------- end of setI -----------------------------------------------

	void setD( float d){
		#ifdef PID_STATE
		LOG("setD: %d",d);
		#endif
		_kD = d;
		if(_kD <= 0)
			_kD = 0;

		_pidData.pidCoefficient[coeffizient_t::kD]=_kD;
		Serial3.print("kD ");Serial3.println(_kD);

		enablePID();

	}//-------------------------------- end of setD -----------------------------------------------
	void setExecutionFrequency(uint8_t ef){
		#ifdef PID_STATE
		LOG("setExecutionFrequency: %d", ef);
		#endif

		_pidData.executionFrequency = ef;
		Serial3.print("Frequenz ");Serial3.println(ef);
		
		enablePID();

	}//-------------------------------- end of setExecutionFrequency ------------------------------
	uint8_t getExecutionTime(){
		#ifdef PID_STATE
		LOG("PID getExecutionTime %d", (1/_pidData.executionFrequency)*1000);
		#endif

		return ((1.0/(float)_pidData.executionFrequency)*1000);
		///< Convert frequency to millis

	}//-------------------------------- end of getExecutionTime -----------------------------------
	void updateEEPROM(void){
	//	EEPROM.put(_EEPROM_startAddress, _pidData);
		enablePID();

	}//-------------------------------- end of updateEEPROM ---------------------------------------
	void readEEPROM(void){
	//	EEPROM.get(_EEPROM_startAddress,  _pidData);
		enablePID();

	}//-------------------------------- end of readEEPROM -----------------------------------------
	float getP() const {
		return _kP;

	}//-------------------------------- end of getP -----------------------------------------------
	float getI() const {
		return _kI;

	}//-------------------------------- end of getI -----------------------------------------------
	float getD() const {
		return _kD;

	}//-------------------------------- end of getD -----------------------------------------------
	float getExFreq() const {
		return _pidData.executionFrequency;

	}//-------------------------------- end of getExTime ------------------------------------------

};	/*--------------------------------- end of pidController class ------------------------------*/


