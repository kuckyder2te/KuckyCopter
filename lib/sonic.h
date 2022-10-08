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
//#include <OneWire.h>
//#include <DallasTemperature.h>
#include "myLogger.h"

#define PIN_ECHO 21
#define PIN_TRIGGER 22
#define PIN_DALLAS 28

//OneWire oneWire(PIN_DALLAS);
//DallasTemperature sens_temperature(&oneWire);

//DeviceAddress insideThermometer;
typedef struct
{
    float temperature;
    double distance;
    //    float coreTemperature;
} sonicData_t;
class Sonic : public Task::Base
{
    bool b; // Klassenvariable

public:
    sonicData_t *_sonicData;

protected:
    UltraSonicDistanceSensor *_hrsr04;

public:
    Sonic(const String &name) : Task::Base(name), b(false)
    {
    }

    virtual ~Sonic() {}

    Sonic *setModel(sonicData_t *_model)
    { // Rückgabe wert ist das eigene Objekt (this)
        LOGGER_VERBOSE("Enter....");
        _sonicData = _model;
        LOGGER_VERBOSE("....leave");
        return this;
    } /*--------------------- end of setModel -----------------------------------------*/

    virtual void begin() override
    {
        LOGGER_VERBOSE("Enter....");
        _hrsr04 = new UltraSonicDistanceSensor(PIN_TRIGGER, PIN_ECHO);
        LOGGER_VERBOSE("....leave");
    } /*--------------------- end og begin --------------------------------------------*/

    virtual void enter() override
    {
        LOGGER_VERBOSE("Enter....");
        _sonicData->distance = _hrsr04->measureDistanceCm();
        LOGGER_VERBOSE("....leave");
    } /*--------------------- end og enter --------------------------------------------*/

    virtual void update() override
    {
        LOGGER_VERBOSE("Enter....");
        //   LOGGER_NOTICE_FMT("Temperature: %.2f *C", _sonicData->temperature);
        LOGGER_WARNING_FMT("Distance: %.2f cm", _hrsr04->measureDistanceCm());
        LOGGER_VERBOSE("....leave");
    } /*--------------------- end og update -------------------------------------------*/

};/*----------------------------------- end of sonic.h class --------------------------*/

//#undef _DEBUG_