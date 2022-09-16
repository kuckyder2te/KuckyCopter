#pragma once
/*  File name : axisBase.h
    Project name : KuCo_Phantom 1
    Author: Wilhelm Kuckelsberg
    Date : 2022-06-16

    Description : 
*/

#include <TaskManager.h>

#include "newPID.h"
#include "myLogger.h"


class AxisBase : public Task::Base {

private:
static uint8_t 	_instanceCounter;      ///< static entfernt

protected:
    NewPID      *_newPID;
//    axisData_t  *_axisData;
 //   uint8_t _instance;
    uint8_t  	_axis_address;	///< Gives everyone axis a title
    int16_t* 	_sp;
    int16_t* 	_fb;
    int16_t* 	_error;
    uint32_t 	_lastMillis;

public:
    AxisBase(const String& name) : Task::Base(name) {
	    _newPID = new NewPID(this->getName()); // Adresse in Variable speichern
        _axis_address = sizeof(pidData_t) * _instanceCounter++;
        _lastMillis 	 = millis();
        _error			 = 0;
        _sp				 = 0;
        _fb				 = 0;
    }

    virtual ~AxisBase() {}

    //     AxisBase* setModel(axisData_t* _model){    // Rückgabe wert ist das eigene Objekt (this)
    //     LOGGER_VERBOSE("Enter....");
    //     _axisData = _model;
    //     LOGGER_VERBOSE("....leave");
    //     return this;
    // } /*----------------------------------- end of setModel ---------------------------*/

    NewPID* getPid(){
        return _newPID;
    } /*----------------------------------- end of getPid -----------------------------*/

    void savePIDConfig(){
     //   _newPID->saveParameters(_axis_address,_axisData-> )
    }
    
    virtual void begin() override
    {
        LOGGER_NOTICE("New PID initialized");   
        _newPID->setExecutionFrequency(PID_FREQUENCY);
        LOGGER_NOTICE("End init New PID");
    } /*----------------------------------- end of begin ------------------------------*/

    virtual void update() override {
    /* _sp Position of the joysticks.
       _fb Position of the drohne.  */

        if(millis()-_lastMillis >= _newPID->getExecutionTime()){	/// muss hier so sein? wird in der service loop immer wieder aufgerufen
            *_error = _newPID->step(*_sp, *_fb);					///< Calculate PID error
            _lastMillis = millis();
        }       
    } /*----------------------------------- end of update -----------------------------*/
};/*----------------------------------- end of axisBase class -------------------------*/


uint8_t AxisBase::_instanceCounter = 0;   //https://stackoverflow.com/questions/5391973/undefined-reference-to-static-const-int
// #undef _DEBUG_