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
#include "axisYaw.h"

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

#define PIN_LED_STATE 7
#define POWER_LIFT_UP 60 ///< The KuckyCopter will start, if throttle > 60
#define DOWN_TIME 2000   ///< Time to turn off the engines (in Microseconds).

class FlyController : public Task::Base
{

private:
    flyState_e _flyState;
    AxisYaw *_axisYaw;

public:
    FlyController(const String &name)
        : Task::Base(name)
    {
    }

    virtual ~FlyController() {}

    // FlyController *setModel(flyState_e *_model)
    // { // Rückgabe wert ist das eigene Objekt (this)
    //   //_axis_data  wird aus dem Model i9n die Achsed geschriene
    //     LOGGER_VERBOSE("Enter....");
    //     _flyState = _model;

    //     LOGGER_VERBOSE("....leave");
    //     return this;
    // }

    virtual void begin() override
    {
        LOGGER_VERBOSE("Enter....");
        _flyState = fly_arming;
        LOGGER_VERBOSE("....leave");
    }

    virtual void update() override
    {
        static uint16_t downTime = 0;
        switch (_flyState)
        {

        case fly_arming:
            /* This is only setting, for the first start from the airplane.
             * Main Power ON/ OFF or option switch. */

            LOGGER_NOTICE("arming start");
//            LOGGER_NOTICE("not implemented yet");

            _axisYaw->setState(AxisYaw::arming_start);
            _flyState = fly_arming_busy;

            break;

        case AxisMotor::arming_busy:
            LOGGER_NOTICE("arming is finished");
//            LOGGER_NOTICE("not implemented yet");
            if (_axisYaw->isArmed())
            {
                _flyState = fly_disablePID; /// -> goes to "disablePID"
            }
            break;

        case fly_disablePID:
            /* Deactivate the PID controller from all axis, and report the state. */
            LOGGER_NOTICE("disablePID");
//            LOGGER_NOTICE("not implemented yet");
            _axisYaw->setState(AxisYaw::disablePID);
            _flyState = fly_standby;
            LOGGER_NOTICE("disable PID is finished");
            break;

        case AxisMotor::standby:
            /* Make sure the throttle lever is set to 0 and RC is connected. */
            //_model.interface.isconnect = true;
            //_model.interface.payload.rcThrottle = 1;
            LOGGER_NOTICE("standby");
            LOGGER_NOTICE("not implemented yet");
            // if (_model.interface.isconnect && (_model.interface.payload.rcThrottle <= POWER_MIN))
            // {
            //     _flyState = fly_prestart;
            // }
            // else
            // {
            //     _model.yawData.throttle = 0;
            //     _flyState = fly_standby;
            // }

            // //	uint16_t battery = analogRead(A1);
            LOGGER_NOTICE("standby fineshed");
            break;

        case fly_prestart:
            LOGGER_NOTICE("prestart");
//            LOGGER_NOTICE("not implemented yet");
            /* Checked if all axes are OK. */

            _axisYaw->setState(AxisYaw::ready);
            if (_axisYaw->isReady())
            {
                _flyState = fly_takeoff;
            }
            else
            {
                _flyState = fly_prestart; /// new 23.05.21
            }
            break;

            // case fly_takeoff:
            //     LOGGER_NOTICE("take off");

            //     /* Throttle greater than POWER_LIFT_UP and RC is connected, go to the next state. */

            //     _model.yawData.throttle = _model.interface.payload.rcThrottle;
            //     if (_model.interface.isconnect && (_model.interface.payload.rcThrottle < POWER_LIFT_UP))
            //     {
            //         _flyState = fly_set_pid;
            //     }
            //     else
            //     {
            //         _flyState = fly_takeoff;
            //     }
            //     LOGGER_NOTICE("take off fineshed");
            //     break;

        case fly_set_pid:

            /* If everything is checked, the PID controller is activated. */

            LOGGER_NOTICE("FlyController set_pid");
            LOGGER_NOTICE("not implemented yet");

            // _model.yawData.throttle = _model.interface.payload.rcThrottle;
            // if (_model.interface.isconnect && (_model.interface.payload.rcThrottle >= POWER_LIFT_UP))
            // {
            //     axisYaw->setState(AxisYaw::enablePID);
            //     *_model.yawData.horz_Position = 0; ///< Reset YAW Position before lift off
            //     _flyState = fly_fly;
            // }
            // else
            // {
            //     _flyState = fly_set_pid; /// new 23.05.21
            // }
            break;

        case fly_fly:
            /* If the power is less than POWER_LIFT_UP and the altitude is less than PID_ACTIVE_AT, the status is set to ground. */

            LOGGER_NOTICE("fly");
            LOGGER_NOTICE("not implemented yet");

            // _model.yawData.throttle = _model.interface.payload.rcThrottle;
            // //			if ((_model.interface.payload.rcThrottle <= POWER_LIFT_UP) || (_model.usData.distance[US::usNo_e::down] < PID_ACTIVE_AT)) {
            // if ((_model.interface.payload.rcThrottle <= POWER_LIFT_UP))
            // {
            //     _flyState = fly_ground;
            // }
            // else
            // {
            //     axisYaw->setState(AxisYaw::ready);
            //     downTime = millis(); ///< save the time for state ground
            // }
            //			uint8_t battery = analogRead(A1);
            //			Serial.println(battery);

            break;

        case fly_ground:
            /* If the quadrocopter is on the ground for more than DOWN_TIME, disable the engines. */
            LOGGER_NOTICE("ground");
            LOGGER_NOTICE("not implemented yet");

            if (millis() - downTime >= DOWN_TIME)
            {
                _flyState = fly_fly;
            }
            else
            {
                _flyState = fly_disablePID;
                //                _model.yawData.throttle = 0;
            }
            break;

        default:
            _flyState = fly_disablePID;
            break;

        } /* end of switch flyState */
    } /*------------------------------- end of update ---------------------------------*/
}; /*---------------------------------- end of flyController --------------------------*/
