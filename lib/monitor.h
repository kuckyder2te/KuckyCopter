#pragma once
/*  File name : monitor.h
    Project name : KuCo_Phantom 2
    Author: Stephan Scholz / Wilhelm Kuckelsberg
    Date : 2022-

    Description : Displays the values for debugging

*/

#include <Arduino.h>
#include <TaskManager.h>
#include "model.h"

//#define LOCAL_DEBUG
#include "myLogger.h"

#ifndef DISPLAY_DELAY
#define DISPLAY_DELAY 1000
#endif
extern HardwareSerial *MonitorOutput;
typedef enum
{
    MOTOR,
    AXIS,
    ALL_AXIS,
    SENSOR,
    RADIO,
    RADIO_SENSOR,
    SONIC,
    BATTERY,
    FLYCONTROL,
    DEFAULT
} Report_t;

char strBuf[100];

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

    Monitor *setModel(model_t *model)
    {
        LOGGER_VERBOSE("Enter....");
        _model = model;
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
                MonitorOutput->println("SENSOR");
                MonitorOutput->printf("/*%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%i*/\r\n",
                    _model->sensorData.yaw,
                    _model->sensorData.pitch,
                    _model->sensorData.roll,
                    _model->sensorData.pressure,
                    _model->sensorData.temperature_baro,
                    _model->sonicData.down_distance,
                    _model->RC_interface.isconnect);
                break;
            case RADIO:
            MonitorOutput->println("DROHNE RADIO");
                MonitorOutput->printf("/*%i,%i,%i,%i,%i,%i,%i,%i,%i,%i*/\r\n",
                    _model->RC_interface.isconnect,
                    _model->RC_interface.RX_payload.rcThrottle,
                    _model->RC_interface.RX_payload.rcYaw,
                    _model->RC_interface.RX_payload.rcPitch,
                    _model->RC_interface.RX_payload.rcRoll,
                    _model->RC_interface.RX_payload.rcSwi1,
                    _model->RC_interface.RX_payload.rcSwi2,
                    _model->RC_interface.RX_payload.rcSwi3,
                    _model->RC_interface.RX_payload.rcAltitudeBaroAdj,
                    _model->RC_interface.RX_payload.rcAltitudeSonicAdj);                   
                break;
            case RADIO_SENSOR:
            MonitorOutput->println("DROHNE RADIO_SENSOR");
                sprintf(strBuf,"/*%d,%d,%d,%d,%d,%d,%d,%d,%d,%d*/",
                    _model->RC_interface.isconnect,
                    _model->RC_interface.RX_payload.rcThrottle,
                    _model->RC_interface.RX_payload.rcYaw,
                    _model->RC_interface.RX_payload.rcPitch,
                    _model->RC_interface.RX_payload.rcRoll,
                    _model->RC_interface.RX_payload.rcSwi1,
                    _model->RC_interface.RX_payload.rcSwi2,
                    _model->RC_interface.RX_payload.rcSwi3,
                    _model->RC_interface.RX_payload.rcAltitudeBaroAdj,
                    _model->RC_interface.RX_payload.rcAltitudeSonicAdj);
                MonitorOutput->println(strBuf);         
                break;
            case SONIC:
            MonitorOutput->println("SONIC");
                sprintf(strBuf,"/*%i,%i,%.2f*/",
                    "Sonic distance/cm = ",
                    _model->sonicData.down_distance,
                    _model->sonicData.front_distance,
                    _model->temperatureData.temperature);
                MonitorOutput->println(strBuf);
                break;
            case AXIS:
                break;
            case ALL_AXIS:              
                break;
            default:
                MonitorOutput->println("y.p.r.pr,temp,dis,con,A_err,B_err,Y_err,Hor_pos,A_Pwr,B_Pwr");
                MonitorOutput->printf("/*%d,%d,%d,%.2f,%.2f,%d,%d,%d,%d,%d,%d,%d,%d*/\r\n",
                    _model->sensorData.yaw,
                    _model->sensorData.pitch,
                    _model->sensorData.roll,
                    _model->sensorData.pressure,
                    _model->sensorData.temperature_baro,
                    _model->sonicData.down_distance,
                    _model->RC_interface.isconnect,
                    _model->axisData[0].pidError,
                    _model->axisData[1].pidError,
                    _model->yawData.pidError,
                    *_model->yaw.horz_Position,
                    _model->axisData[0].power,
                    _model->axisData[1].power
                    );
                break;
            }
        }
    } /*--------------------- end of update -----------------------------------------------------*/
};/* -------------------------- end of monitor class --------------------------------------------*/
