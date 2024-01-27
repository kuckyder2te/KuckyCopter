#pragma once
/*  File name :
    Project name : KuckyCopter 2
    Authors: Stephan Scholz / Wilhelm Kuckelsberg
    Date : 2022-06-19

    Description : Flight controls.
*/

#include <Arduino.h>
#include <TaskManager.h>

//#define LOCAL_DEBUG
#include "myLogger.h"

#include "radio.h"
#include "sonics.h"
#include "model.h"

#define POWER_LIFT_UP 25 ///< Test Value
#define DOWN_TIME 2000  ///< Time to turn off the engines (in Microseconds).
#define PID_ACTIVE_AT 9 ///< PID aktiviert ab einer Höhe von 9cm

class FlyController : public Task::Base
{
private:
    AxisYaw *_axisYaw;
    model_t *_model;

    typedef enum
    {
        arming_begin = 0, ///< When the Kuckycopter is first turned on, the arming starts.
        arming_busy,      ///< Arming is in progress
        disablePID,       ///< disable the PID controlle
        standby,          ///< All motors on POWER_MIN
        prestart,         ///< All motors on standby and ready to fly. (POWER_MIN)
        takeoff,          ///< The Quadrocopter takes off.
        set_pid,          ///< Fly without PID-Output = 0
        fly,              ///< Normal fly mode
        ground            ///< Kuckycopter stand on the ground
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
    } /*------------------------------- end of updateModel --------------------------------------*/

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

    void printPidErrors(){
        LOGGER_NOTICE_FMT("YAW Error: %d",_model->yawData.pidError);
        LOGGER_NOTICE_FMT("Primary Axis Error: %d",_model->axisData[axisName::primary].pidError);
        LOGGER_NOTICE_FMT("Primary Axis SetPoint: %d",_model->axisData[axisName::primary].setpoint);
        LOGGER_NOTICE_FMT("Primary Axis Feedback: %d",*_model->axisData[axisName::primary].feedback);
        LOGGER_NOTICE_FMT("Secondary Axis Error: %d",_model->axisData[axisName::secondary].pidError);
        LOGGER_NOTICE_FMT("Secondary Axis SetPoint: %d",_model->axisData[axisName::secondary].setpoint);
        LOGGER_NOTICE_FMT("Secondary Axis Feedback: %d",*_model->axisData[axisName::secondary].feedback);
    }

    void resetYawPosition(){
        static int32_t debug_yaw;
        LOGGER_NOTICE_FMT_CHK((int32_t)_model->sensorData.yaw,debug_yaw,"Set YAW Position to %d",(int32_t)_model->sensorData.yaw);
        _model->yawData.setpoint = _model->sensorData.yaw;
    }

    virtual void begin() override // _model is not yet known here
    {
    } /*------------------------------- end of begin --------------------------------------------*/

    virtual void update() override
    {
        static uint16_t downTime = 0;
        switch (flyState)
        {
        case arming_begin:
            /* This is only setting, for the first start from the airplane.
             * Main Power ON/OFF or option-switch. */
            LOGGER_NOTICE("arming begin -> arming_busy");
            _axisYaw->setState(AxisYaw::state_e::arming_start);
            flyState = arming_busy;
            break;

        case arming_busy:
            LOGGER_VERBOSE("arming busy");
            if (_axisYaw->isArmed())
            {
                flyState = disablePID;
                LOGGER_NOTICE("arming is finished -> disablePID");
                if(!_model->RC_interface.TX_payload.isInitialized){
                    LOGGER_NOTICE("************** Please init RC!! **************");
                }
            }
            break;

        case disablePID:
            /* Deactivate the PID controller from all axis. */
            LOGGER_VERBOSE("disablePID");
            _axisYaw->setState(AxisYaw::state_e::disablePID);
            flyState = standby;
            LOGGER_NOTICE("disable PID is finished -> standby");
            break;

        case standby:
            LOGGER_VERBOSE("standby");
            /* Make sure the throttle lever is set to 0 and RC is connected. */
            if (_model->RC_interface.TX_payload.isInitialized &&_model->RC_interface.isconnect && (getThrottle() >= POWER_MIN))
            {
                flyState = prestart;
                LOGGER_NOTICE_CHK(flyState, Debug_flyState, "standby is finished -> prestart");
            }
            else
            {
                _model->yawData.power = 0;
                _axisYaw->setState(AxisYaw::standby);
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
                LOGGER_NOTICE("prestart is finished -> takeoff");
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
            
            _model->yawData.power = getThrottle();
            //_radio->RC_interface->RX_payload.rcThrottle = 1;  // Just to test if Flycontroller is running.
            if (_model->RC_interface.isconnect && (_model->RC_interface.RX_payload.rcThrottle < POWER_LIFT_UP))
            {
                flyState = set_pid;
                LOGGER_NOTICE_CHK(flyState,Debug_flyState,"take off is finished -> set_pid");
            }
            else
            {
                flyState = takeoff;
                _axisYaw->setState(AxisYaw::standby);
                LOGGER_NOTICE_CHK(flyState,Debug_flyState,"take off is held");
            }
            break;

        case set_pid:
            /* If everything is checked, the PID controller is activated. */
            LOGGER_VERBOSE("set pid");
            _model->yawData.power = getThrottle();
            resetYawPosition();
            
            if(_model->RC_interface.isconnect && (_model->yawData.power > POWER_LIFT_UP))
            {
                _axisYaw->setState(AxisYaw::enablePID);
                _model->yaw.horz_Position = 0; ///< Reset YAW Position before lift off
                resetYawPosition();
                printPidErrors();           // Got no execution time
                flyState = fly;
                LOGGER_NOTICE_CHK(flyState,Debug_flyState,"PID setting is finished -> fly");
            }
            else
            {
                flyState = set_pid;
                LOGGER_NOTICE_CHK(flyState,Debug_flyState,"PID setting is idle");
            }
            break;

        case fly:
            /* If the power is less than POWER_LIFT_UP and the altitude is less than PID_ACTIVE_AT,
            the status is set to ground. */
            LOGGER_VERBOSE("fly");
            //_model->sonicData.down_distance = 10;   // temp_debug
            //_model->RC_interface.RX_payload.rcThrottle = 0;   // temp_debug
            
            _model->yawData.power = getThrottle();

            if ((_model->yawData.power < POWER_LIFT_UP) && (_model->sonicData.down_distance < PID_ACTIVE_AT))
            {
                flyState = ground;
                resetYawPosition();
                LOGGER_NOTICE_CHK(flyState,Debug_flyState,"fly is finished -> ground");
            }
            else
            {
                _axisYaw->setState(AxisYaw::ready);
                downTime = millis(); ///< save the time for state ground
                LOGGER_NOTICE_CHK(flyState,Debug_flyState,"fly is held");
            }
            break;

        case ground:
            /* If the quadrocopter is on the ground for more than DOWN_TIME, disable the engines. */
            LOGGER_VERBOSE("ground");
            if (millis() - downTime <= DOWN_TIME)
            {
                flyState = fly;
                LOGGER_NOTICE_CHK(flyState,Debug_flyState,"Flystate is activ -> fly");
            }
            else
            {
                flyState = standby;
                _model->RC_interface.TX_payload.isInitialized = false;
                _model->yawData.power = 0;
                LOGGER_NOTICE_CHK(flyState,Debug_flyState,"Drohne is on the ground -> disablePID");
            }
            printPidErrors();
            break;

        default:
            flyState = disablePID;
            break;

        } /* end of switch flyState */
        updateModel();
    } /*------------------------------- end of update -------------------------------------------*/
    int16_t getThrottle(){
            int16_t throttle;
            throttle = _model->RC_interface.RX_payload.rcThrottle;
            if(throttle<POWER_MIN){
                throttle = POWER_MIN;
            }
            if(throttle>POWER_MAX){
                LOGGER_WARNING_FMT("MAX Power overflow: %d !!!",throttle);
                throttle = POWER_MAX;
            }
            return throttle;
        }
};    /*---------------------------------- end of flyController ---------------------------------*/