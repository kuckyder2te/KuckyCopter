#pragma once
/*  File name: sonic.h
 *	Project name: KuCo_Phantom 1
 *  Date: 2022-05-28
 *  Author: Wilhelm Kuckelsberg
 *  Description: Niederflug 
 *  HCSR04 max. Distanz 400cm
 *  
 *  Impulsdauer 100cm   2,9137 mSec * 2 = 5,8275 mSec
 *              200cm   5,8275            11,655 mSec        
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

#define MAX_DISTANCE 200
#define MAX_TIME_OUT 11655

//OneWire oneWire(PIN_DALLAS);
//DallasTemperature sens_temperature(&oneWire);

typedef struct
{
    float temperature;
    double distance;
} sonicData_t;
class Sonic : public Task::Base
{
    bool b; // Klassenvariable

public:
    sonicData_t *_sonicData;

protected:
    UltraSonicDistanceSensor *_hcrs04;

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
        _hcrs04 = new UltraSonicDistanceSensor(PIN_TRIGGER, PIN_ECHO, MAX_DISTANCE, MAX_TIME_OUT);
        LOGGER_WARNING_FMT("File %s : HCSR04 lib version: %s", __FILE__, HCSR_LIB_VERSION);
        LOGGER_VERBOSE("....leave");
    } /*--------------------- end og begin --------------------------------------------*/

    virtual void enter() override
    {
        LOGGER_VERBOSE("Enter....");
        _sonicData->distance = _hcrs04->measureDistanceCm();
        LOGGER_VERBOSE("....leave");
    } /*--------------------- end og enter --------------------------------------------*/

    virtual void update() override
    {
        LOGGER_VERBOSE("Enter....");
        //   LOGGER_NOTICE_FMT("Temperature: %.2f *C", _sonicData->temperature);
        LOGGER_WARNING_FMT("Distance: %.2f cm", _hcrs04->measureDistanceCm());
        LOGGER_VERBOSE("....leave");
    } /*--------------------- end og update -------------------------------------------*/

};/*----------------------------------- end of sonic.h class --------------------------*/

//#undef _DEBUG_