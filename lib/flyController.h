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
#include "radio.h"
#include "sonic.h"
#include "model.h"

#define PIN_LED_STATE 7
#define POWER_LIFT_UP 60 ///< The KuckyCopter will start, if throttle > 60
#define DOWN_TIME 2000   ///< Time to turn off the engines (in Microseconds).
#define PID_ACTIVE_AT 9     ///< PID aktiviert ab einer Höhe von 9cm
class FlyController : public Task::Base
{
public:
    
private:
    AxisYaw *_axisYaw;
    model_t *_model;

public:
    FlyController(const String &name)
        : Task::Base(name)
    {
    }

    virtual ~FlyController() {}

    FlyController *init(model_t *model){ // Rückgabe wert ist das eigene Objekt (this)
                                         //_axis_data  wird aus dem Model in die Achsen geschrieben
        LOGGER_VERBOSE("Enter....");
        _model = model;
        _model->flyState = arming_begin;

        LOGGER_VERBOSE("....leave");
        return this;
    }

    FlyController *setYawAxis(AxisYaw *axisYaw){
        _axisYaw = axisYaw;
        return this;
    }

    virtual void begin() override   // Wird nach dem Erzeugen das Tasks automatisch aufgerufen. _model ist hier noch nicht bekannt
    {
    }

    virtual void update() override
    {
        static uint16_t downTime = 0;
        switch (_model->flyState)
        {

        case arming_begin:
            /* This is only setting, for the first start from the airplane.
             * Main Power ON/OFF or option-switch. */
            LOGGER_VERBOSE("arming begin");
            _axisYaw->setState(AxisYaw::arming_start);     /// hier bleibt es hängen
            _model->flyState = arming_busy;
            LOGGER_VERBOSE("arming begin is fineshed");
            break;

        case arming_busy:
            LOGGER_VERBOSE("arming busy");
            if (_axisYaw->isArmed())
            {
                _model->flyState = disablePID; 
                LOGGER_VERBOSE("arming busy is fineshed");
            }
            break;

        case disablePID:
            /* Deactivate the PID controller from all axis. */
            LOGGER_VERBOSE("disablePID");
            _axisYaw->setState(AxisYaw::state_e::disablePID);
            _model->flyState = standby;
            LOGGER_VERBOSE("disable PID is finished");
            break;

        case standby:
            LOGGER_VERBOSE("standby");
            /* Make sure the throttle lever is set to 0 and RC is connected. */
            _model->rcInterface.isconnect = true;          // nur zum testen, ob Flycontroller
            _model->rcInterface.payload.rcThrottle = 1;    // durchläuft
            
            if (_model->rcInterface.isconnect && (_model->rcInterface.payload.rcThrottle >= POWER_MIN))
            {
                _model->flyState = prestart;
                LOGGER_VERBOSE("standby is fineshed");
            }
            else
            {
                _model->yawData.throttle = 0;
                _model->flyState = standby;
            }          
            break;

        case prestart:
            LOGGER_VERBOSE("prestart");
            /* Checked if all axes are OK. */
            _axisYaw->setState(AxisYaw::ready);
            if (_axisYaw->isReady())
            {
                _model->flyState = takeoff;
                LOGGER_VERBOSE("prestart is fineshed");
            }
            else
            {
                _model->flyState = prestart; 
            }
            break;

            case takeoff:
                LOGGER_VERBOSE("take off");
                /* Throttle greater than POWER_LIFT_UP and RC is connected, go to the next state. */
                //_radio->rcInterface->isconnect = true;          // nur zum testen, ob Flycontroller           
                 _model->yawData.throttle = _model->rcInterface.payload.rcThrottle;
                //_radio->rcInterface->payload.rcThrottle = 1;    // durchläuft
                if (_model->rcInterface.isconnect && (_model->rcInterface.payload.rcThrottle < POWER_LIFT_UP))
                {
                    _model->flyState = set_pid;
                }
                else
                {
                    _model->flyState = takeoff;
                }
                LOGGER_VERBOSE("take off is fineshed");
                break;

        case set_pid:
            /* If everything is checked, the PID controller is activated. */
            LOGGER_VERBOSE("set pid");
            _model->rcInterface.isconnect = true;          // nur zum testen, ob Flycontroller           
            _model->yawData.throttle = _model->rcInterface.payload.rcThrottle;
            _model->rcInterface.payload.rcThrottle = 65;    // durchläuft
            if (_model->rcInterface.isconnect && (_model->rcInterface.payload.rcThrottle >= POWER_LIFT_UP))
            {
                _axisYaw->setState(AxisYaw::enablePID);
                _model->yawData.horz_Position = 0; ///< Reset YAW Position before lift off
                _model->flyState = fly;
                LOGGER_VERBOSE("set pid is fineshed");
            }
            else
            {
                _model->flyState = set_pid; /// new 23.05.21
            }
            break;

        case fly:
            /* If the power is less than POWER_LIFT_UP and the altitude is less than PID_ACTIVE_AT, the status is set to ground. */
            LOGGER_VERBOSE("fly");
            //_model->sonicData.distance = 10;          // nur zum testen, ob Flycontroller                      
            _model->yawData.throttle= _model->rcInterface.payload.rcThrottle;
           //_model->yawData.throttle = 50;             // durchläuft

            if ((_model->rcInterface.payload.rcThrottle <= POWER_LIFT_UP) || (_model->sonicData.distance < PID_ACTIVE_AT))
            //if ((_model->rcInterface.payload.rcThrottle <= POWER_LIFT_UP))
            {
                _model->flyState = ground;
                LOGGER_NOTICE("fly is fineshed");
            }
            else
            {
                _axisYaw->setState(AxisYaw::ready);
                downTime = millis(); ///< save the time for state ground
            }
            break;

        case ground:
            /* If the quadrocopter is on the ground for more than DOWN_TIME, disable the engines. */
            LOGGER_VERBOSE("ground");
            if (millis() - downTime >= DOWN_TIME)
            {
                _model->flyState = fly;
                LOGGER_VERBOSE("ground fineshed");
            }
            else
            {
                _model->flyState = disablePID;
                _model->yawData.throttle = 0;
            }
            break;

        default:
            _model->flyState = disablePID;
            break;

        } /* end of switch flyState */
    } /*------------------------------- end of update ---------------------------------*/
}; /*---------------------------------- end of flyController --------------------------*/
//#undef _DEBUG_