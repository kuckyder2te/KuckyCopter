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
#include <extEEPROM.h>
#include <TaskManager.h>
#include "..\lib\def.h"
//#include "..\lib\model.h"			
#include "myLogger.h"

#ifndef PID_EEPROM_ADRRESS
#define PID_EEPROM_ADRRESS 50
#endif

//------ PID configuration begin --------------------
// #define PID_FREQUENCY        50				///< PID parameter
#define PID_P_MIN			  0.00390626	///< The parameter P domain is [0.00390625 to 255] inclusive.
// #define PID_EEPROM_ADRRESS	 50
// #define PID_ACTIVE_AT		  9
// #define YAW_SENSIBILITY		  5
// #define YAW_FINE_TUNING		  0.1
//------ PID configuration end ----------------------

typedef struct {
	float pidCoeff_P;
    float pidCoeff_I;
    float pidCoeff_D;
	float executionFrequency;
	int   output_bits;
	bool  output_signed;
} pidData_t;




// typedef enum {
// 	kP,
// 	kI,
// 	kD
// }coeffizient_t;

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
	 uint8_t _instance;
	float          RC_SP;
	float          FB;

public:
	PIDcontroller(pidData_t& pidData) : _pidData(pidData), FastPID() {	// Call the standard constructor of the base class.
			setOutputRange(-100, 100);
			// _EEPROM_startAddress = (_instance * sizeof(pidData_t))+PID_EEPROM_ADRRESS;  ///< calculated the the EEPROM address
			// EEPROM.get(_EEPROM_startAddress, _pidData);	///< Reads the data from the EEPROM directly by the start.
			_pidInstance =_instance++;
			setOutputConfig(16, true);
		//			disablePID();
		};

    virtual ~PIDcontroller() {}

    // PIDcontroller* setModel(pidData_t* _model){    // Rückgabe wert ist das eigene Objekt (this)
    // LOGGER_VERBOSE("Enter....");
    // _pidData = _model;
    // LOGGER_VERBOSE("....leave");
    // return this;

    // virtual void begin() override {
    //  	LOGGER_VERBOSE("Enter...");
		
	// 	LOGGER_VERBOSE("...Leave");  
    // }

    // virtual void update() override {
	// 	LOGGER_VERBOSE("Enter....");
	//     LOGGER_VERBOSE("...Leave");  
    // }

	void disablePID(){

		LOGGER_WARNING_FMT("Disabled PID controller %d ", _pidInstance);
		delay(100);

		FastPID::setCoefficients(PID_P_MIN, 0.0, 0.0, getExecutionTime());

		LOGGER_WARNING_FMT("PID P = %f", _pidData.pidCoeff_P);
		LOGGER_WARNING_FMT("PID I = %f", _pidData.pidCoeff_I);
		LOGGER_WARNING_FMT("PID D = %f", _pidData.pidCoeff_D);
		delay(100);

	}//-------------------------------- end of deactivatePID --------------------------------------

	void enablePID(){
		/* This function has 2 tasks.
		 * 1. The PID parameters are uploaded from the PID adjustment.
		 * 2. The PID parameters are activated. */
	
		LOGGER_WARNING_FMT("Enable PID controller %d ", _pidInstance);
		delay(100);
		
 		FastPID::setCoefficients(_pidData.pidCoeff_P,
								 _pidData.pidCoeff_I,
								 _pidData.pidCoeff_D,
								 getExecutionTime());

		LOGGER_WARNING_FMT("PID P = %f", _pidData.pidCoeff_P);
		LOGGER_WARNING_FMT("PID I = %f", _pidData.pidCoeff_I);
		LOGGER_WARNING_FMT("PID D = %f", _pidData.pidCoeff_D);
		delay(100);
		
	}//-------------------------------- end of activatePID --------------------------------------------

	void setP( float p){	
	//	LOGGER_WARNING_FMT("set P: %f",p);
		_kP = p;
		if(_kP <= PID_P_MIN)
			_kP = PID_P_MIN;

		_pidData.pidCoeff_P=_kP;
        LOGGER_WARNING_FMT("set P: %f",_kP);
		enablePID();
	}//-------------------------------- end of setP -----------------------------------------------

	void setI( float i){
	//	LOGGER_WARNING_FMT("set I: %f",i);
		_kI = i;
		if(_kI <= 0)
			_kI = 0;

		_pidData.pidCoeff_I=_kI;
		LOGGER_WARNING_FMT("set I: %f",_kI);
		enablePID();
	}//-------------------------------- end of setI -----------------------------------------------

	void setD( float d){
    //    LOGGER_WARNING_FMT("set D: %f",_kD);
		_kD = d;
		if(_kD <= 0)
			_kD = 0;

		_pidData.pidCoeff_D=_kD;
		LOGGER_WARNING_FMT("set D: %f",_kD);
		enablePID();
	}//-------------------------------- end of setD -----------------------------------------------

	void setExecutionFrequency(uint8_t ef){
		LOGGER_WARNING_FMT("setExecutionFrequency: %d", ef);

		_pidData.executionFrequency = ef;
		
		enablePID();
	}//-------------------------------- end of setExecutionFrequency ------------------------------
	
	uint8_t getExecutionTime(){
		LOGGER_WARNING_FMT("PID getExecutionTime %d", (1/_pidData.executionFrequency)*1000);

		return ((1.0/(float)_pidData.executionFrequency)*1000);
		///< Convert frequency to millis
	}//-------------------------------- end of getExecutionTime -----------------------------------
	
	void updateEEPROM(void){
	//	EEPROM.put(_EEPROM_startAddress, _pidData);
		enablePID();
	}//-------------------------------- end of updateEEPROM ---------------------------------------
	
	void readEEPROM(void){
		// EEPROM.get(_EEPROM_startAddress,  _pidData);
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
};
/*--------------------------- end of pidController class ------------------------------------------------*/
