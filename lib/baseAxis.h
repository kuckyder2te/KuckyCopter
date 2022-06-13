#pragma once
/*  File name : baseAxis.h
    Project name : KuCo_Phantom 1
    Date : 2022-06-16

    Description : 
*/

//#include <Arduino.h>
#include <TaskManager.h>
//#include <FastPID.h>
//#include "newPID.h"
#include "def.h"
#include "myLogger.h"



typedef struct{
	int16_t* error;
}baseData_t;

// #define	kPcoeff,
// 	kI,
// 	kD
// coeffizient_t;

#define	kP
#define	kI
#define	kD

class BaseAxis : public Task::Base {

private:
	float 	  	_kP;
	float       _kI;
	float       _kD;
	float		_exFreq;
	uint8_t     _EEPROM_startAddress;
	uint8_t 	_pidInstance;

protected:
//    NewPID        *_newPID;
    baseData_t    *_baseData;
//    static uint8_t 	_instance;
    uint8_t  		_axis_address;	///< Gives everyone axis a title
    int16_t* 		_sp;
    int16_t* 		_fb;
    int16_t* 		_error;
    uint32_t 		_lastMillis;
    uint32_t		_temp;

public:
    BaseAxis(const String& name) : Task::Base(name) {
		// _pid->setExecutionFrequency(PID_FREQUENCY);
        _lastMillis 	 = millis();
        _error			 = 0;
        _sp				 = 0;
        _fb				 = 0;
    //    _axis_address 	 = _instance++;
    }

    virtual ~BaseAxis() {
    }

    // optional (you can remove this method)
    // virtual void begin() override {
    // }

    // optional (you can remove this method)
    // virtual void enter() override {
    // }

    virtual void update() override {
 			/* _sp Position of the joysticks.
			 * _fb Position of the drohne.  */

			// if(millis()-_lastMillis >= _newPID->getExecutionTime()){	/// muss hier so sein? wird in der service loop immer wieder aufgerufen
			// 	*_error = _newPID->step(*_sp, *_fb);					///< Calculate PID error

			// 	_lastMillis = millis();
			// }       
    }

    // optional (you can remove this method)
    // virtual void exit() override {
    // }

    // optional (you can remove this method)
    // virtual void idle() override {
    // }

    // optional (you can remove this method)
    // virtual void reset() override {
    // }
};
