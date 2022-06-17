#pragma once
/*  File name : axisBase.h
    Project name : KuCo_Phantom 1
    Date : 2022-06-16

    Description : 
*/

//#include <Arduino.h>
#include <TaskManager.h>
#include "newPID.h"
#include "def.h"
#include "myLogger.h"

typedef struct{
	int16_t* error;
}baseData_t;

#define	kP
#define	kI
#define	kD

class AxisBase : public Task::Base {

private:
	float 	  	_kP;
	float       _kI;
	float       _kD;
	float		_exFreq;
	uint8_t     _EEPROM_startAddress;
	uint8_t 	_pidInstance;

protected:
    NewPID        *_newPID;
    baseData_t    *_baseData;
     static       uint8_t 	_instance;      ///< static entfernt
    uint8_t  		_axis_address;	///< Gives everyone axis a title
    int16_t* 		_sp;
    int16_t* 		_fb;
    int16_t* 		_error;
    uint32_t 		_lastMillis;
    uint32_t		_temp;

public:
    AxisBase(const String& name) : Task::Base(name) {
	       _newPID = new NewPID();  // Adresse in Variable speichern
        _lastMillis 	 = millis();
        _error			 = 0;
        _sp				 = 0;
        _fb				 = 0;
    //    _axis_address 	 = _instance++;
    }

    virtual ~AxisBase() {}

        AxisBase* setModel(baseData_t* _model){    // Rückgabe wert ist das eigene Objekt (this)
        LOGGER_VERBOSE("Enter....");
        _baseData = _model;
        LOGGER_VERBOSE("....leave");
        return this;
    }

    virtual void begin() override {
        LOGGER_NOTICE("New PID initialized");
     
        _newPID->setExecutionFrequency(PID_FREQUENCY);
        LOGGER_NOTICE("End init New PID");
    }

    virtual void update() override {
    /* _sp Position of the joysticks.
        * _fb Position of the drohne.  */

			// if(millis()-_lastMillis >= _newPID->getExecutionTime()){	/// muss hier so sein? wird in der service loop immer wieder aufgerufen
			//  	*_error = _newPID->step(*_sp, *_fb);					///< Calculate PID error

			//  	_lastMillis = millis();
			// }       
    }
};/*----------------------------------- end of axisBase class -------------------------*/