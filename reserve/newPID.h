#pragma once
/*  File name : pidNew.h
	Autor: Wilhelm Kuckelsberg
	Date: 2022-05-31 (2021.05.24)
	Description: PID Regler wird eingestellt
*/

#include <TaskManager.h>
#include <FastPID.h>

#include ".\EEPROM\EEPROM.h"
#include "myLogger.h"
#include "def.h"



typedef struct {
	float pidCoefficient[3];
	float executionFrequency;
	int   output_bits;
	bool  output_signed;
} pidData_t;


typedef enum {
	kP,
	kI,
	kD,
    eF
}coeffizient_t;

class NewPID {

private:
	float 	  	_kP;
	float       _kI;
	float       _kD;
	float		_exFreq;
	uint8_t     _EEPROM_startAddress;
	uint8_t 	_pidInstance;    

protected:
    FastPID *_fastPID;
	EEPROMClass *_eeprom;
    pidData_t &_pidData;
    static uint8_t _instance;			// static entfernt
	float  RC_SP;
	float  FB;
    
public:

    NewPID((pidData_t &_pidData) : _pidData(pidData)) {
    LOGGER_VERBOSE("Enter....");
	
    	_fastPID->setOutputRange(-100, 100);
//		_EEPROM_startAddress = (_instance * sizeof(pidData_t))+PID_EEPROM_ADRRESS;  ///< calculated the the EEPROM address
//		EEPROM.get(_EEPROM_startAddress, _pidData);	///< Reads the data from the EEPROM directly by the start.
		_pidInstance =_instance++;
		LOGGER_WARNING_FMT("Instance %d ", _pidInstance);
		_fastPID->setOutputConfig(16, true);
		disablePID();
    LOGGER_VERBOSE("....leave");         
    }

    virtual ~NewPID() {}
   
 
    void disablePID(){
    LOGGER_WARNING_FMT("Disabled PID controller %d ", _pidInstance);

    _fastPID->setCoefficients(PID_P_MIN, 0.0, 0.0, getExecutionTime());

    // LOGGER_WARNING_FMT("PID Coeff. P = %f", _pidData->pidCoefficient[coeffizient_t::kP]);
    // LOGGER_WARNING_FMT("PID Coeff. I = %f", _pidData->pidCoefficient[coeffizient_t::kI]);
    // LOGGER_WARNING_FMT("PID Coeff. D = %f", _pidData->pidCoefficient[coeffizient_t::kD]);
	}//-------------------------------- end of disablePID() -----------------------------

	void enablePID(){
		/* This function has 2 tasks.
		 * 1. The PID parameters are uploaded from the PID adjustment.
		 * 2. The PID parameters are activated. */
		LOGGER_WARNING_FMT("Enable PID controller %d ", _pidInstance);

 		// _fastPID->setCoefficients(_pidData->pidCoefficient[coeffizient_t::kP],
		// 						  _pidData->pidCoefficient[coeffizient_t::kI],
		// 						  _pidData->pidCoefficient[coeffizient_t::kD],
		// 						  getExecutionTime());
	
		// LOGGER_WARNING_FMT("PID Coeff. P = %f", _pidData->pidCoefficient[coeffizient_t::kP]);
		// LOGGER_WARNING_FMT("PID Coeff. I = %f", _pidData->pidCoefficient[coeffizient_t::kI]);
		// LOGGER_WARNING_FMT("PID Coeff. D = %f", _pidData->pidCoefficient[coeffizient_t::kD]);
	}//-------------------------------- end of activatePID ------------------------------

	void setP( float _kP){
	//	_kP = p;
		if(_kP <= PID_P_MIN)
			_kP = PID_P_MIN;

	//	_pidData->pidCoefficient[coeffizient_t::kP]=_kP;
		LOGGER_WARNING_FMT("kp = %f", _kP);
	//	enablePID();
	}//-------------------------------- end of setP -------------------------------------

	void setI( float _kI){
	//	_kI = i;
		if(_kI <= 0)
			_kI = 0;

	//	_pidData->pidCoefficient[coeffizient_t::kI]=_kI;
		LOGGER_WARNING_FMT("kI = %f", _kI);
		enablePID();
	}//-------------------------------- end of setI -------------------------------------

	void setD( float _kD){
	//	_kD = d;
		if(_kD <= 0)
			_kD = 0;

		// _pidData->pidCoefficient[coeffizient_t::kD]=_kD;
		LOGGER_WARNING_FMT("kD = %f", _kD);
		enablePID();
	}//-------------------------------- end of setD -------------------------------------
	
	void setExecutionFrequency(uint8_t _ef){
		// _pidData->executionFrequency = _ef;
		LOGGER_WARNING_FMT("setExecutionFrequency: %d", _ef);
		enablePID();
	}//-------------------------------- end of setExecutionFrequency --------------------
	uint8_t getExecutionTime(){
	//	LOGGER_WARNING_FMT("PID getExecutionTime %d", (1/_pidData->executionFrequency)*1000);
	//< Convert frequency to millis
	//	return ((1.0/(float)_pidData->executionFrequency)*1000);	
	}//-------------------------------- end of getExecutionTime -------------------------
	
	void updateEEPROM(void){
	//	EEPROM.put(_EEPROM_startAddress, _pidData);
		enablePID();
	}//-------------------------------- end of updateEEPROM -----------------------------
	
	void readEEPROM(void){
	//	EEPROM.get(_EEPROM_startAddress,  _pidData);
		enablePID();
	}//-------------------------------- end of readEEPROM -------------------------------
	
	float getP() const {
		return _kP;
	}//-------------------------------- end of getP -------------------------------------
	
	float getI() const {
		return _kI;
	}//-------------------------------- end of getI -------------------------------------
	
	float getD() const {
		return _kD;
	}//-------------------------------- end of getD -------------------------------------
	
	// float getExFreq() const {
	//  	return _pidData->executionFrequency;
	// }//-------------------------------- end of getExFreq --------------------------------
};/*--------------------------- end of pidNew.h class ----------------------------*/