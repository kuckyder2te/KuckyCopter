#pragma once
/*  File name :
    Project name : KuCo_Phantom 1
    Author: Stephan Scholz / Wilhelm Kuckelsberg
    Date : 2022-

    Description : Displays the values for debugging

*/

#include <Arduino.h>
#include <TaskManager.h>
#include "model.h"

// #define LOCAL_DEBUG
#include "myLogger.h"

#define DISPLAY_DELAY 1000
typedef enum
{
    MOTOR,
    AXIS,
    SENSOR,
    RADIO,
    SONIC,
    FLYCONTROL,
    DEFAULT
} Report_t;

class Monitor : public Task::Base
{
private:
    model_t *_model;
    unsigned long _lastMillis;
    uint16_t _display_delay;
    Report_t _report;

public:
    Monitor(const String &name, Report_t report = Report_t::DEFAULT, uint16_t delay = DISPLAY_DELAY)
        : Task::Base(name)
    {
        _lastMillis = millis();
        _report = report;
        _display_delay = delay;
    }

    virtual ~Monitor() {}

    Monitor *setModel(model_t *_model)
    {
        LOGGER_VERBOSE("Enter....");
        _model = _model;
        LOGGER_VERBOSE("....leave");
        return this;
    } /*--------------------- end of setModel ---------------------------------------------------*/

    virtual void update() override
    {
        if (millis() - _lastMillis > _display_delay)
        {
            _lastMillis = millis();

            switch (_report)
            {
            case MOTOR:

                break;
            case SENSOR:
                Serial.printf("/*%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%i*/\r\n",
                    _model->sensorData.yaw,
                    _model->sensorData.pitch,
                    _model->sensorData.roll,
                    _model->sensorData.pressure,
                    _model->sensorData.temperature_baro,
                    _model->sonicData.down_distance,
                    _model->RC_interface.isconnect);
                break;
            case RADIO:
                Serial.printf("/*%i,%i,%i,%i,%i,%i,%i,%i,%i,%i*/\r\n",
                    _model->RC_interface.isconnect,
                    _model->RC_interface.RX_payload.rcThrottle,
                    _model->RC_interface.RX_payload.rcYaw,
                    _model->RC_interface.RX_payload.rcPitch,
                    _model->RC_interface.RX_payload.rcRoll,
                    _model->RC_interface.RX_payload.rcSwi1, // Switch is not activ
                    _model->RC_interface.RX_payload.rcSwi2,
                    _model->RC_interface.RX_payload.rcSwi3,
                    _model->RC_interface.RX_payload.rcAltitudeBaroAdj,
                    _model->RC_interface.RX_payload.rcAltitudeSonicAdj);
                break;
            case SONIC:
                Serial.printf("/*%i,%i*/\r\n",
                    _model->sonicData.down_distance,
                    _model->sonicData.front_distance);
                break;
            default:
                Serial.println("Default");
                break;
            }
        }
    }
};
/* -------------------------- end of monitor class ----------------------------------------------*/
