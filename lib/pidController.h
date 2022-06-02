#pragma once
/*  File name : pidController.h
	Autor: Wilhelm Kuckelsberg
	Date: 2022-05-31 (2021.05.24)
	Description: PID werden eingestellt
*/

#include <TaskManager.h>
#include <FastPID.h>

#include "..\lib\myLogger.h"
#include "..\lib\def.h"

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

class PidController: public FastPID {			/// MyPid is an FastPit
private:
	float 	  	_kP;
	float       _kI;
	float       _kD;
	float		_exFreq;
	uint8_t     _EEPROM_startAddress;
	uint8_t 	_pidInstance;

protected:
	pidData_t&     _pidData;				/// MyPid has a PidData
	uint8_t _instance;			// static entfernt
	float          RC_SP;
	float          FB;

public:
	PidController(pidData_t& pidData) : _pidData(pidData),
		FastPID() {	// Call the standard constructor of the base class.
			setOutputRange(-100, 100);
	//		_EEPROM_startAddress = (_instance * sizeof(pidData_t))+PID_EEPROM_ADRRESS;  ///< calculated the the EEPROM address
	//		EEPROM.get(_EEPROM_startAddress, _pidData);	///< Reads the data from the EEPROM directly by the start.
			_pidInstance =_instance++;
			setOutputConfig(16, true);
			disablePID();
		};

	void disablePID(){
		LOGGER_WARNING_FMT("Disabled PID controller %d ", _pidInstance);

		FastPID::setCoefficients(PID_P_MIN, 0.0, 0.0, getExecutionTime());

		LOGGER_WARNING_FMT("PID Coeff. P = %f", _pidData.pidCoefficient[coeffizient_t::kP]);
		LOGGER_WARNING_FMT("PID Coeff. I = %f", _pidData.pidCoefficient[coeffizient_t::kI]);
		LOGGER_WARNING_FMT("PID Coeff. D = %f", _pidData.pidCoefficient[coeffizient_t::kD]);
	}//-------------------------------- end of disablePID() -----------------------------

	void enablePID(){
		/* This function has 2 tasks.
		 * 1. The PID parameters are uploaded from the PID adjustment.
		 * 2. The PID parameters are activated. */
		LOGGER_WARNING_FMT("Enable PID controller %d ", _pidInstance);

 		FastPID::setCoefficients(_pidData.pidCoefficient[coeffizient_t::kP],
								 _pidData.pidCoefficient[coeffizient_t::kI],
								 _pidData.pidCoefficient[coeffizient_t::kD],
								 getExecutionTime());
	
		LOGGER_WARNING_FMT("PID Coeff. P = %f", _pidData.pidCoefficient[coeffizient_t::kP]);
		LOGGER_WARNING_FMT("PID Coeff. I = %f", _pidData.pidCoefficient[coeffizient_t::kI]);
		LOGGER_WARNING_FMT("PID Coeff. D = %f", _pidData.pidCoefficient[coeffizient_t::kD]);
	}//-------------------------------- end of activatePID ------------------------------

	void setP( float p){
		_kP = p;
		if(_kP <= PID_P_MIN)
			_kP = PID_P_MIN;

		_pidData.pidCoefficient[coeffizient_t::kP]=_kP;
		LOGGER_WARNING_FMT("kp = %f", _kP);
		enablePID();
	}//-------------------------------- end of setP -------------------------------------

	void setI( float i){
		_kI = i;
		if(_kI <= 0)
			_kI = 0;

		_pidData.pidCoefficient[coeffizient_t::kI]=_kI;
		LOGGER_WARNING_FMT("kI = %f", _kI);
		enablePID();
	}//-------------------------------- end of setI -------------------------------------

	void setD( float d){
		_kD = d;
		if(_kD <= 0)
			_kD = 0;

		_pidData.pidCoefficient[coeffizient_t::kD]=_kD;
		LOGGER_WARNING_FMT("kD = %f", _kD);
		enablePID();
	}//-------------------------------- end of setD -------------------------------------
	
	void setExecutionFrequency(uint8_t ef){
		_pidData.executionFrequency = ef;
		LOGGER_WARNING_FMT("setExecutionFrequency: %d", ef);
		enablePID();
	}//-------------------------------- end of setExecutionFrequency --------------------
	uint8_t getExecutionTime(){
		#ifdef PID_STATE
		LOG("PID getExecutionTime %d", (1/_pidData.executionFrequency)*1000);
		#endif
	//< Convert frequency to millis
		return ((1.0/(float)_pidData.executionFrequency)*1000);	
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
	
	float getExFreq() const {
		return _pidData.executionFrequency;
	}//-------------------------------- end of getExFreq --------------------------------
};
/*--------------------------- end of pidController.h class ----------------------------*/