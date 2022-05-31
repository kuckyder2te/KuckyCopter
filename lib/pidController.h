#pragma once


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

class PidController : public Task::Base {

    	float 	  	_kP;
	float       _kI;
	float       _kD;
	float		_exFreq;
	uint8_t     _EEPROM_startAddress;
	uint8_t 	_pidInstance;
 

protected:
    FastPID *_fastPID;
	pidData_t*     _pidData;
    uint8_t _instance;
	float          RC_SP;
	float          FB;


public:
    PidController(const String& name) : Task::Base(name) {
    LOGGER_VERBOSE("Enter....");
        _fastPID->setOutputRange(-100, 100);
		_pidInstance =_instance++;
		_fastPID->setOutputConfig(16, true);
		disablePID();
    LOGGER_VERBOSE("....leave");
    }



    virtual ~PidController() {}

        PidController* setModel(pidData_t* _model){    // Rückgabe wert ist das eigene Objekt (this)
        LOGGER_VERBOSE("Enter....");
        _pidData = _model;
        LOGGER_VERBOSE("....leave");
        return this;
    }
  
    virtual void begin() override {
    LOGGER_VERBOSE("Enter....");
        LOGGER_NOTICE("FastPID initialized");
        _fastPID = new FastPID();  // Adresse in Variable speichern
        LOGGER_NOTICE("End init FastPID");
        
    LOGGER_VERBOSE("....leave");   
    }
    // virtual void enter() override {
    // }
    virtual void update() override {

    }
    
 	void disablePID(){
		LOGGER_WARNING_FMT("Disabled PID controller %d ", _pidInstance);
		delay(100);
		
		_fastPID->setCoefficients(PID_P_MIN, 0.0, 0.0, getExecutionTime());

		LOGGER_WARNING_FMT("PID coeff P = %f ", _pidData->pidCoefficient[coeffizient_t::kP]);
		LOGGER_WARNING_FMT("PID coeff I = %f ", _pidData->pidCoefficient[coeffizient_t::kI]);
		LOGGER_WARNING_FMT("PID coeff D = %f ", _pidData->pidCoefficient[coeffizient_t::kD]);
		delay(100);
	}//-------------------------------- end of disablePID --------------------------------------

	void enablePID(){
	/* This function has 2 tasks.
	* 1. The PID parameters are uploaded from the PID adjustment.
	* 2. The PID parameters are activated. */

		LOGGER_WARNING_FMT("Enable PID controller %d ", _pidInstance);
		delay(100);

 		_fastPID->setCoefficients(_pidData->pidCoefficient[coeffizient_t::kP],
								 _pidData->pidCoefficient[coeffizient_t::kI],
								 _pidData->pidCoefficient[coeffizient_t::kD],
								 getExecutionTime());

		LOGGER_WARNING_FMT("PID coeff P = %f", _pidData->pidCoefficient[coeffizient_t::kP]);
		LOGGER_WARNING_FMT("PID coeff I = %f", _pidData->pidCoefficient[coeffizient_t::kI]);
		LOGGER_WARNING_FMT("PID coeff D = %f", _pidData->pidCoefficient[coeffizient_t::kD]);
		delay(100);
	}//-------------------------------- end of enablePID --------------------------------------------

	void setP( float p){
		_kP = p;
		if(_kP <= PID_P_MIN)
			_kP = PID_P_MIN;

		_pidData->pidCoefficient[coeffizient_t::kP]=_kP;
		LOGGER_WARNING_FMT("Set P = %f - kP = %f" ,p, _kP);
		enablePID();
	}//-------------------------------- end of setP -----------------------------------------------

	void setI( float i){
		_kI = i;
		if(_kI <= 0)
			_kI = 0;

		_pidData->pidCoefficient[coeffizient_t::kI]=_kI;
		LOGGER_WARNING_FMT("Set I = %f - kI = %f" ,i, _kI);
		enablePID();
	}//-------------------------------- end of setI -----------------------------------------------

	void setD( float d){
		_kD = d;
		if(_kD <= 0)
			_kD = 0;

		_pidData->pidCoefficient[coeffizient_t::kD]=_kD;
		LOGGER_WARNING_FMT("Set D = %f - kD = %f", d, _kD);
		enablePID();
	}//-------------------------------- end of setD -----------------------------------------------
	
	void setExecutionFrequency(uint8_t ef){
		_pidData->executionFrequency = ef;
		LOGGER_WARNING_FMT("EXE Freq. = %d", ef);		
		enablePID();
	}//-------------------------------- end of setExecutionFrequency ------------------------------
	
	uint8_t getExecutionTime(){
		LOGGER_WARNING_FMT("PID getExecutionTime %d", (1/_pidData->executionFrequency)*1000);
		return ((1.0/(float)_pidData->executionFrequency)*1000);
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
		return _pidData->executionFrequency;
	}//-------------------------------- end of getExFreq ------------------------------------------

};  /*---------------------- end of pidController.h -------------*/