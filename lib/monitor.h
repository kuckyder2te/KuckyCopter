#pragma once
/*  File name : 
    Project name : KuCo_Phantom 1
    Author: Wilhelm Kuckelsberg
    Date : 2022-

    Description : Drohne
 
*/

#include <Arduino.h>
#include <TaskManager.h>
#include "model.h"

//#define LOCAL_DEBUG
#include "myLogger.h"

#define DISPLAY_DELAY 1000


class Monitor : public Task::Base {
private:
    model_t *_model;
    unsigned long _lastMillis;
public:
    Monitor(const String& name)
    : Task::Base(name) {
        _lastMillis = millis();
    }

    virtual ~Monitor() {}


    Monitor *setModel(model_t *_mod)
    { // Rückgabe wert ist das eigene Objekt (this)
        LOGGER_VERBOSE("Enter....");
        _model = _mod;
        LOGGER_VERBOSE("....leave");
        return this;
    } /*--------------------- end of setModel -----------------------------------------*/


    // optional (you can remove this method)
    // virtual void begin() override {
    // }

    virtual void update() override {
        if(millis()-_lastMillis < DISPLAY_DELAY){
            _lastMillis = millis();
    /*
    IMU-> Yaw,Pitch,Roll
    Sonic-> closeRange
    Radio-> 
    Baro->
    Motor
    */

        Serial.printf("/*%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%i*/\r\n",
            _model->sensorData.yaw,
            _model->sensorData.pitch,
            _model->sensorData.roll,
            _model->sensorData.pressure,
            _model->sensorData.temperature_baro,
            _model->sonicData.closeRange,
            _model->RC_interface.isconnect
            );
        }
    }
};
