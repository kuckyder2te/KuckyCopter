#pragma once
/*  File name : axisBase.h
    Project name : KuCo_Phantom 1
    Authors: Stephan cholz / Wilhelm Kuckelsberg
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
        double *rcX;       ///< virtual axis. Corresponds to the ROLL axis.		///  zu int16_t konvertieren
        double *rcY;       ///< virtual axis. Corresponds to the PITCH axis.
    } axisData_t;

private:
    static uint8_t _instanceCounter; ///< static entfernt ??

protected:
    NewPID *_newPID;
    uint8_t eepromAddress; ///< Gives everyone axis a title
    int16_t *_sp;
    int16_t *_fb;
    int16_t *_error;
    uint32_t _lastMillis;
    axisData_t *_axisData;

public:
    /// @brief Defindet the base axis
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

        LOGGER_NOTICE("New PID initialized");
        _newPID->setEF(PID_FREQUENCY);

        LOGGER_VERBOSE("....leave");
    } /*----------------------------------- end of begin ----------------------------------------*/

    virtual void update() override
    {
        LOGGER_VERBOSE("Enter....");

        /* _sp Position of the joysticks.
           _fb Position of the drohne.  */

        if (millis() - _lastMillis >= _newPID->getExecutionTime())
        {                                        /// muss hier so sein? wird in der service loop immer wieder aufgerufen
            *_error = _newPID->step(*_sp, *_fb); ///< Calculate PID error
            _lastMillis = millis();
        }

        LOGGER_VERBOSE("....leave");
    } /*----------------------------------- end of update -------------------------------------*/
};    /*----------------------------------- end of axisBase class -----------------------------*/

uint8_t AxisBase::_instanceCounter = 0; // https://stackoverflow.com/questions/5391973/undefined-reference-to-static-const-int