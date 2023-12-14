#pragma once
/*  File name :
    Project name : KuckyCopter 2
    Authors: Stephan Scholz / Wilhelm Kuckelsberg
    Date : 2022-06-19

    Description : Drohne
*/

#include <Arduino.h>
#include <TaskManager.h>

#define LOCAL_DEBUG
#include "myLogger.h"

#include "radio.h"
#include "sonics.h"
#include "model.h"

// #define POWER_LIFT_UP 60 ///< The KuckyCopter will start, if throttle > 60
#define POWER_LIFT_UP 10 /// Test Value
#define DOWN_TIME 2000   ///< Time to turn off the engines (in Microseconds).
#define PID_ACTIVE_AT 9  ///< PID aktiviert ab einer Höhe von 9cm

class FlyController : public Task::Base
{
public:
private:
    AxisYaw *_axisYaw;
    model_t *_model;

    typedef enum
    {
        arming_begin = 0,   ///< When the Kuckycopter is first turned on, the arming starts.
        arming_busy,        ///< ist das nötig?
        disablePID,
        standby,            ///< All motors on POWER_MIN
        prestart,           ///< All motors on standby and ready to fly. (POWER_MIN)
        takeoff,            ///< The Quadrocopter takes off.
        set_pid,            ///< Fly without PID-Output = 0
        fly,                ///< Normal fly mode
        ground              ///< Kuckycopter stand on the ground
    } flyState_e;
    flyState_e flyState, Debug_flyState;

    void updateModel()
    {
        _model->RC_interface.TX_payload.yaw = _model->sensorData.yaw;
        _model->RC_interface.TX_payload.pitch = _model->sensorData.pitch;
        _model->RC_interface.TX_payload.roll = _model->sensorData.roll;
        _model->RC_interface.TX_payload.altitude = _model->sensorData.altitude;
        _model->RC_interface.TX_payload.pressure = _model->sensorData.pressure;
        _model->RC_interface.TX_payload.temperature = _model->sensorData.temperature_baro;
        _model->RC_interface.TX_payload.distance_down = _model->sonicData.down_distance;
    }

public:
    FlyController(const String &name)
        : Task::Base(name)
    {
    }

    FlyController *init(model_t *model)
    { 
        LOGGER_VERBOSE("Enter....");
        _model = model;
        flyState = arming_begin;
        LOGGER_VERBOSE("....leave");
        return this;
    }

    FlyController *setYawAxis(AxisYaw *axisYaw)
    {
        _axisYaw = axisYaw;
        return this;
    }

    virtual void begin() override // Wird nach dem Erzeugen das Tasks automatisch aufgerufen. _model ist hier noch nicht bekannt
    {
    }

    virtual void update() override
    {
        static uint16_t downTime = 0;
        switch (flyState)
        {

        case arming_begin:
            /* This is only setting, for the first start from the airplane.
             * Main Power ON/OFF or option-switch. */
            LOGGER_NOTICE("arming begin");
            _axisYaw->setState(AxisYaw::state_e::arming_start);
            flyState = arming_busy;
            break;

        case arming_busy:
            LOGGER_VERBOSE("arming busy");
            if (_axisYaw->isArmed())
            {
                flyState = disablePID;
                LOGGER_NOTICE("arming is fineshed");
            }
            break;

        case disablePID:
            /* Deactivate the PID controller from all axis. */
            LOGGER_VERBOSE("disablePID");
            _axisYaw->setState(AxisYaw::state_e::disablePID);
            flyState = standby;
            LOGGER_NOTICE("disable PID is finished");
            break;

        case standby:
            LOGGER_VERBOSE("standby");
            /* Make sure the throttle lever is set to 0 and RC is connected. */
            // _model->RC_interface.isconnect = true;           // Just to test if Flycontroller is running.
            // _model->RC_interface.RX_payload.rcThrottle = 1;   //          ""

            if (_model->RC_interface.isconnect && (_model->RC_interface.RX_payload.rcThrottle >= POWER_MIN))
            {
                flyState = prestart;
                LOGGER_NOTICE_CHK(flyState, Debug_flyState, "standby is fineshed");
            }
            else
            {
                _model->yawData.power = 0;
                flyState = standby;
                LOGGER_NOTICE_CHK(flyState, Debug_flyState, "standby is held");
            }
            break;

        case prestart:
            LOGGER_VERBOSE("prestart");
            /* Checked if all axes are OK. */
            _axisYaw->setState(AxisYaw::ready);
            if (_axisYaw->isReady())
            {
                flyState = takeoff;
                LOGGER_NOTICE("prestart is fineshed");
            }
            else
            {
                flyState = prestart;
                LOGGER_NOTICE("prestart is held");
            }
            break;

        case takeoff:
            LOGGER_VERBOSE("take off");
            /* Throttle greater than POWER_LIFT_UP and RC is connected, go to the next state. */
            //_radio->RC_interface->isconnect = true;           // Just to test if Flycontroller is running.
            _model->yawData.power = _model->RC_interface.RX_payload.rcThrottle;
            //_radio->RC_interface->RX_payload.rcThrottle = 1;  // Just to test if Flycontroller is running.
            if (_model->RC_interface.isconnect && (_model->RC_interface.RX_payload.rcThrottle < POWER_LIFT_UP))
            {
                flyState = set_pid;
                LOGGER_NOTICE("take off is fineshed");
            }
            else
            {
                flyState = takeoff;
                LOGGER_NOTICE("take off is held");
            }
            break;

        case set_pid:
            /* If everything is checked, the PID controller is activated. */
            LOGGER_VERBOSE("set pid");
            _model->RC_interface.isconnect = true; // Just to test if Flycontroller is running.
            _model->yawData.power = _model->RC_interface.RX_payload.rcThrottle;
            _model->RC_interface.RX_payload.rcThrottle = 65; // Just to test if Flycontroller is running. Debug
            if (_model->RC_interface.isconnect && (_model->RC_interface.RX_payload.rcThrottle >= POWER_LIFT_UP))
            {
                _axisYaw->setState(AxisYaw::enablePID);
                _model->yaw.horz_Position = 0; ///< Reset YAW Position before lift off
                flyState = fly;
                LOGGER_NOTICE("set pid is fineshed");
            }
            else
            {
                flyState = set_pid;
                LOGGER_NOTICE("set pid is held");
            }
            break;

        case fly:
            /* If the power is less than POWER_LIFT_UP and the altitude is less than PID_ACTIVE_AT, 
            the status is set to ground. */
            LOGGER_VERBOSE("fly");
            //_model->sonicData.down_distance = 10;          // Just to test if Flycontroller is running.

            _model->RC_interface.RX_payload.rcThrottle = 100;

            _model->yawData.power = _model->RC_interface.RX_payload.rcThrottle;
            //_model->yawData.throttle = 50;                  // Just to test if Flycontroller is running.

            if ((_model->yawData.power <= POWER_LIFT_UP) || (_model->sonicData.down_distance < PID_ACTIVE_AT))
            {
                flyState = ground;
                LOGGER_NOTICE("fly is fineshed");
            }
            else
            {
                _axisYaw->setState(AxisYaw::ready);
                downTime = millis(); ///< save the time for state ground
                LOGGER_NOTICE("fly is held");
            }
            break;

        case ground:
            /* If the quadrocopter is on the ground for more than DOWN_TIME, disable the engines. */
            LOGGER_VERBOSE("ground");
            if (millis() - downTime >= DOWN_TIME)
            {
                flyState = fly;
                LOGGER_NOTICE("Flystate is activ");
            }
            else
            {
                flyState = disablePID;
                _model->yawData.power = 0;
                LOGGER_NOTICE("Drone is on the ground.");
            }
            break;

        default:
            flyState = disablePID;
            break;

        } /* end of switch flyState */
        updateModel();
    } /*------------------------------- end of update ------------------------------------------*/
};/*---------------------------------- end of flyController ------------------------------------*/