#pragma once
/*  File name :
    Project name : KuCo_Phantom 1
    Author: Wilhelm Kuckelsberg
    Date : 2022-06-19

    Description : Drohne

*/

#include <Arduino.h>
#include <TaskManager.h>
#include "myLogger.h"

typedef enum
{
    fly_arming = 0, ///< When the Kuckycopter is first turned on, the arming starts.
    fly_arming_busy,
    fly_disablePID,
    fly_standby,  ///< All motors on POWER_MIN
    fly_prestart, ///< All motors on standby and ready to fly. (POWER_MIN)
    fly_takeoff,  ///< The Quadrocopter takes off.
    fly_set_pid,  ///< Fly without PID-Output = 0
    fly_fly,      ///< Normal fly mode
    fly_ground    ///< Kuckycopter stand on the ground
} flyState_e;

class FlyController : public Task::Base
{

private:
    flyState_e *_flyState;
    

public:
    FlyController(const String &name)
        : Task::Base(name)
    {
    }

    virtual ~FlyController() {}

    FlyController *setModel(flyState_e *_model)
    { // Rückgabe wert ist das eigene Objekt (this)
      //_axis_data  wird aus dem Model i9n die Achsed geschriene
        LOGGER_VERBOSE("Enter....");
        _flyState = _model;

        LOGGER_VERBOSE("....leave");
        return this;
    }

    virtual void begin() override
    {
        LOGGER_VERBOSE("Enter....");

        LOGGER_VERBOSE("....leave");
    }

    // optional (you can remove this method)
    // virtual void enter() override {
    // }

    virtual void update() override
    {

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
}; /*------------------------------ end of flyController -------------------*/
