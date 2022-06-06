#pragma once
/*  File name: sonic.h
 *	Project name: KuCo_Phantom 1
 *  Date: 2022-05-28
 *  Author: Wilhelm Kuckelsberg
 *  Description: Niederflug Messung
 */

#include <Arduino.h>
#include <TaskManager.h>
#include <HCSR04.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "..\lib\def.h"

OneWire oneWire(PIN_DALLAS);
DallasTemperature sens_temperature(&oneWire);

DeviceAddress insideThermometer;
typedef struct{
    float temperature;
    double distance;
//    float coreTemperature;
}sonicData_t;
class Sonic : public Task::Base {
    bool b;         // Klassenvariable

protected:
    UltraSonicDistanceSensor *_distanceSensor;  
    sonicData_t *_sonicData;
        
public:
    Sonic(const String& name) : Task::Base(name) ,b(false){
    
    }

    virtual ~Sonic() {}

    Sonic* setModel(sonicData_t* _model){    // Rückgabe wert ist das eigene Objekt (this)
    LOGGER_VERBOSE("Enter....");
        _sonicData = _model;
    LOGGER_VERBOSE("....leave");
        return this;
    }

    virtual void begin() override {
        LOGGER_VERBOSE("Enter....");
     //   sens_temperature.requestTemperatures();   // wird nicht ausgeführt, auch nicht im Testcode
        _distanceSensor = new UltraSonicDistanceSensor(PIN_TRIGGER, PIN_ECHO);
    //    coreTemperature = analogReadTemp();
        LOGGER_VERBOSE("....leave");
    }

    virtual void enter() override {
        LOGGER_VERBOSE("Enter....");
     //   _sonicData->temperature = sens_temperature.getTempCByIndex(0);
          _sonicData->distance = _distanceSensor->measureDistanceCm();
        LOGGER_VERBOSE("....leave");
    }

    virtual void update() override {
        LOGGER_VERBOSE("Enter....");
     //   LOGGER_NOTICE_FMT("Temperature: %.2f *C", _sonicData->temperature);
        LOGGER_NOTICE_FMT("Distance: %.2f cm", _distanceSensor->measureDistanceCm());
        LOGGER_VERBOSE("....leave");
    }
};/*----------------------------------- end of sonic.h class ----------------------------*/