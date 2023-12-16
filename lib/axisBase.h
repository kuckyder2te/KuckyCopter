#pragma once
/*  File name : axisBase.h
    Project name : KuckyCopter 2
    Authors: Stephan Scholz / Wilhelm Kuckelsberg
    Date : 2022-06-16

    Description :
*/

#include <TaskManager.h>

#define LOCAL_DEBUG
#include "myLogger.h"

#include "newPID.h"

class AxisBase : public Task::Base
{

public:
    typedef struct
    {
        uint16_t power; /// power from YAW Axis
        int16_t pidError;
        int16_t setpoint;  ///< Memory for detuning the physical axis.
        int16_t *feedback; ///< Current value from the IMU
        int8_t *rcX;       ///< virtual axis. Corresponds to the ROLL axis.		///  zu int16_t konvertieren
        int8_t *rcY;       ///< virtual axis. Corresponds to the PITCH axis.
    } axisData_t;

    typedef enum
	{
		arming_start = 0,
		arming_busy,
		arming_end,
		disablePID,
		enablePID,
		standby,
		ready,
		off
	} state;

private:
    static uint8_t _instanceCounter;

protected:
    NewPID *_newPID;
    uint8_t eepromAddress; ///< Gives everyone axis a title
    int16_t *_sp;
    int16_t *_fb;
    int16_t *_error;
    uint32_t _lastMillis;
    axisData_t *_axisData;
    state _state, _lastState;

public:
    /// @brief Defines the base axis
    /// @param name Task name
    AxisBase(const String &name) : Task::Base(name)
    {
        eepromAddress = sizeof(pidData_t) * _instanceCounter++;
        _newPID = new NewPID(name, eepromAddress); 

        _lastMillis = millis();
        _error = 0;
        _sp = 0;
        _fb = 0;
        //   loadPIDConfig();
    }

    virtual boolean isArmed() = 0;

    NewPID *getPid()
    {
        return _newPID;
    } /*----------------------------------- end of getPid -------------------------------------*/

    void savePIDConfig()
    {
        LOGGER_VERBOSE("Enter....");

        _newPID->saveParameters();

        LOGGER_VERBOSE("....leave");
    } /*----------------------------------- end of savePIDConfig -------------------------------*/

    void loadPIDConfig()
    {
        LOGGER_VERBOSE("Enter....");

        _newPID->loadParameters();

        LOGGER_VERBOSE("....leave");
    } /*----------------------------------- end of loadPIDConfig -------------------------------*/

    virtual void begin() override
    {
        LOGGER_VERBOSE("Enter....");

        LOGGER_VERBOSE("....leave");
    } /*----------------------------------- end of begin ----------------------------------------*/

    virtual void update() override
    {
        LOGGER_VERBOSE("Enter....");
        /* _sp Position of the joysticks.
           _fb Position of the drohne.  */

        if (millis() - _lastMillis >= _newPID->getExecutionTime())
        {              
            int16_t err = _newPID->step(*_sp, *_fb);                        
            *_error = err;
            _lastMillis = millis();
        }
        LOGGER_VERBOSE("....leave");
    } /*----------------------------------- end of update -------------------------------------*/
    int16_t getPidError(){
        return *_error;
    } /*--------------------- end of getPidError ------------------------------------------------*/

    virtual boolean isStandby()
	{
		LOGGER_NOTICE_FMT_CHK(_state,_lastState,"Enter....isStandby %s ", this->getName().c_str());
		return (_state == standby);
	} /*--------------------- end of isStandby --------------------------------------------------*/
    
	virtual boolean isDeactivatePID()
	{
		LOGGER_NOTICE_FMT_CHK(_state,_lastState,"Enter....isDeactivatePID %s ", this->getName().c_str());
		return (_state == disablePID);
	} /*--------------------- end of isDeactivatePID --------------------------------------------*/

	virtual boolean isReady()
	{
		LOGGER_NOTICE_FMT_CHK(_state,_lastState,"axisbase....isReady %s ", this->getName().c_str());
		return (_state == ready);
	} /*---------------------- end of isReady ---------------------------------------------------*/
};    /*----------------------------------- end of axisBase class -------------------------------*/

uint8_t AxisBase::_instanceCounter = 0; 
// https://stackoverflow.com/questions/5391973/undefined-reference-to-static-const-int