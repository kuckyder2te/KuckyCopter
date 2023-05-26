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

//#define LOCAL_DEBUG
#include "myLogger.h"

#define PIN_ECHO 21
#define PIN_TRIGGER 22
#define PIN_DHT 6
#define DHTTYPE DHT22

#define MAX_DISTANCE 200

HC_SR04<PIN_ECHO> sensorSonic(PIN_TRIGGER); // sensor with echo and trigger pin
DHT dht(PIN_DHT, DHTTYPE);

typedef struct
{
    float temperature;
    float humidity;
    float closeRange, _closeRange; // Entfernung, Temperatur kompensiert
    float closeRange_raw;        // Entfernung ohne Kompensation
    float speedOfSoundInCmPerMicroSec;
} sonicData_t;

class Sonic : public Task::Base{

public:
    sonicData_t *_sonicData;
    sonicData_t __sonicData;

protected:

public:
    Sonic(const String &name) : Task::Base(name) //, b(false)
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

        dht.begin();

        sensorSonic.beginAsync();
        sensorSonic.startAsync(100000); // start first measurement

        LOGGER_VERBOSE("....leave");
    } /*--------------------- end of begin --------------------------------------------*/

    virtual void enter() override
    {
        LOGGER_VERBOSE("Enter....");
        LOGGER_VERBOSE("....leave");
    } /*--------------------- end of enter --------------------------------------------*/

    virtual void update() override
    {
        LOGGER_VERBOSE("Enter....");

        if (sensorSonic.isFinished())
        {
            _sonicData->closeRange = sensorSonic.getDist_cm();
            sensorSonic.startAsync(100000);
            #ifdef SERIAL_STUDIO
                LOGGER_NOTICE_FMT_CHK(_sonicData->closeRange, __sonicData.closeRange, "Altitude: %.2f", _sonicData->closeRange);
            #endif
        }
            _sonicData->humidity = dht.readHumidity();
            _sonicData->temperature = dht.readTemperature();
 
            // Check if any reads failed and exit early (to try again).
            if (isnan(_sonicData->humidity) || isnan(_sonicData->temperature)) {
                LOGGER_FATAL("Failed to read from DHT sensor!");
                return;
            }
            _sonicData->speedOfSoundInCmPerMicroSec = 0.03313 + (0.0000606 * _sonicData->temperature); // Cair ≈ (331.3 + 0.606 ⋅ ϑ) m/s

            #ifndef SERIAL_STUDIO
                LOGGER_NOTICE_FMT_CHK(_sonicData->speedOfSoundInCmPerMicroSec, __sonicData.speedOfSoundInCmPerMicroSec,"SoundSpeed: %.2f", _sonicData->speedOfSoundInCmPerMicroSec);
                LOGGER_NOTICE_FMT_CHK(_sonicData->humidity, __sonicData.humidity, "Humidity: %.2f Temperature: %.2f", _sonicData->humidity, _sonicData->temperature);
            #endif

        LOGGER_VERBOSE("....leave");
    } /*--------------------- end of update -------------------------------------------*/

}; /*----------------------------------- end of sonic.h class -------------------------*/
