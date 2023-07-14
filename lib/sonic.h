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
#include <HC_SR04.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define LOCAL_DEBUG
#include "myLogger.h"

#define PIN_ECHO_1 21 // Sonic down
#define PIN_TRIGGER_1 22
#define PIN_ECHO_2 3 // Sonic front
#define PIN_TRIGGER_2 1

#define PIN_DHT 6
#define DHTTYPE DHT22

#define MAX_DISTANCE 200
HC_SR04_BASE *slaves[] = {new HC_SR04<PIN_ECHO_2>(PIN_TRIGGER_2)};
HC_SR04<PIN_ECHO_1> sensorSonic(PIN_TRIGGER_1, slaves, 1); // sensor with echo and trigger pin
DHT dht(PIN_DHT, DHTTYPE);

typedef struct
{
    float temperature;
    float humidity;
    float closeRange, _closeRange; // Entfernung, Temperatur kompensiert
    float closeRange_raw;          // Entfernung ohne Kompensation
    float speedOfSoundInCmPerMicroSec;
    float down_distance;
    float front_distance;
} sonicData_t;

class Sonic : public Task::Base
{

public:
    sonicData_t *_sonicData;
    sonicData_t __sonicData;

protected:
public:
    Sonic(const String &name) : Task::Base(name){}

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
        dht.begin();
        sensorSonic.beginAsync();
        for (int i = 0; i < sensorSonic.getNumberOfSensors(); i++)
            if (!sensorSonic.isInterruptSupported(i))
                //LOGGER_FATAL_FMT("Sensor, %i: *FAILED Interrupt!", i);
                //LOGGER_FATAL(String(i).c_str());
                Serial.println(i);
                
        sensorSonic.startAsync(200000);
        LOGGER_VERBOSE("....leave");
    } /*--------------------- end of begin --------------------------------------------*/

    virtual void update() override
    {
        LOGGER_VERBOSE("Enter....");
        
        if (sensorSonic.isFinished())
        {
            _sonicData->down_distance = sensorSonic.getDist_cm(0);
            _sonicData->front_distance = sensorSonic.getDist_cm(1);
            sensorSonic.startAsync(200000);
        }
        
        LOGGER_VERBOSE("....leave");
    } /*--------------------- end of update -------------------------------------------*/

}; /*----------------------------------- end of sonic.h class -------------------------*/
