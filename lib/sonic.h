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
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include "myLogger.h"

#define PIN_ECHO 21
#define PIN_TRIGGER 22
#define PIN_DHT 28
#define DHTTYPE DHT22

#define MAX_DISTANCE 200
#define MAX_TIME_OUT 11655

typedef struct
{
    float temperature;
    float humidity;
    double distance;
} sonicData_t;
class Sonic : public Task::Base
{
    bool b; // Klassenvariable
    uint32_t delayMS;

public:
    sonicData_t *_sonicData;

protected:
    UltraSonicDistanceSensor *_hcrs04;
    DHT_Unified *_dht;

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
        _dht = new DHT_Unified(PIN_DHT, DHTTYPE);

        _dht->begin();
        sensor_t sensor;
        _dht->temperature().getSensor(&sensor);
        LOGGER_WARNING("Temperature Sensor");
        LOGGER_WARNING_FMT("Sensor Type: %c", sensor.name);
        LOGGER_WARNING_FMT("Driver Ver: %i", sensor.version);
        LOGGER_WARNING_FMT("Unique ID: %i", sensor.version);
        LOGGER_WARNING_FMT("Min  %f max. value %f ", sensor.min_value, sensor.max_value);
        LOGGER_WARNING_FMT("Resolution:  %f", sensor.resolution);

        // Print humidity sensor details.
        _dht->humidity().getSensor(&sensor);
        LOGGER_WARNING("Humidity Sensor");
        LOGGER_WARNING_FMT("Sensor Type: %c", sensor.name);
        LOGGER_WARNING_FMT("Driver Ver: %i", sensor.version);
        LOGGER_WARNING_FMT("Unique ID: %i", sensor.version);
        LOGGER_WARNING_FMT("Min  %f max. value %f ", sensor.min_value, sensor.max_value);
        LOGGER_WARNING_FMT("Resolution:  %f", sensor.resolution);

        // Set delay between sensor readings based on sensor details.
        delayMS = sensor.min_delay / 1000;  // ~2000
        Serial2.println(delayMS);
//        delay(5000);
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
       
        LOGGER_WARNING_FMT("Distance: %.2f cm", _hcrs04->measureDistanceCm());

        // Delay between measurements.
    //    delay(delayMS);
        // Get temperature event and print its value.
        sensors_event_t event;
        _dht->temperature().getEvent(&event);
        if (isnan(event.temperature))
        {
            LOGGER_FATAL("Error reading temperature!");
        }
        else
        {
            _sonicData->temperature = event.temperature;
            LOGGER_WARNING_FMT("Temperature: %.2f%s", event.temperature, "*C");
        }
        // Get humidity event and print its value.
        _dht->humidity().getEvent(&event);
        if (isnan(event.relative_humidity))
        {
            LOGGER_FATAL("Error reading humidity!");
        }
        else
        {
            _sonicData->humidity = event.relative_humidity;
            LOGGER_WARNING_FMT("Humidity: %.2f%c", event.relative_humidity, '%');
        }
        LOGGER_VERBOSE("....leave");
    } /*--------------------- end og update -------------------------------------------*/

}; /*----------------------------------- end of sonic.h class --------------------------*/

//#undef _DEBUG_