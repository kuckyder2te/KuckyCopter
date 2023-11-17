#pragma once
/*  File name: sonic.h
 *	Project name: KuCo_Phantom 1
 *  Date: 2022-05-28
 *  Author: Stephan Scholz / Wilhelm Kuckelsberg
 *  Description: Low flight and obstacle detection
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

#define PIN_ECHO_1      21 // Sonic down
#define PIN_TRIGGER_1   22
#define PIN_ECHO_2       3 // Sonic front
#define PIN_TRIGGER_2    2
#define NUMBER_OF_SLAVES 1 // Number of possible slave sonic sensors

#define PIN_DHT 6
#define DHTTYPE DHT22

#define MAX_DISTANCE 200 // distance range 2 to 400cm

DHT dht(PIN_DHT, DHTTYPE);

char buffer[128];

typedef struct
{
    uint16_t temperature;
    uint16_t humidity;
    //float speedOfSoundInCmPerMicroSec;
    uint16_t down_distance;
    uint16_t front_distance;
} sonicData_t;

class Sonic : public Task::Base
{
private:
    HC_SR04_BASE *slave;
    HC_SR04_BASE *sonic;
    HC_SR04_BASE *slaves[NUMBER_OF_SLAVES];

public:
    sonicData_t *_sonicData;
    sonicData_t __sonicData;    // makes the logger readable "...CHK..."

protected:
public:
    Sonic(const String &name) : Task::Base(name)
    {
        slave = new HC_SR04<PIN_ECHO_2>(PIN_TRIGGER_2); // This is a slave sensor, in this case only on
        slaves[0] = slave;
        sonic = new HC_SR04<PIN_ECHO_1>(PIN_TRIGGER_1, slaves, NUMBER_OF_SLAVES); // Master sensor with echo and trigger pin
    }

    Sonic *setModel(sonicData_t *_model)
    {
        LOGGER_VERBOSE("Enter....");
        _sonicData = _model;
        LOGGER_VERBOSE("....leave");
        return this;
    } /*--------------------- end of setModel ---------------------------------------------------*/

    virtual void begin() override
    {
        LOGGER_VERBOSE("Enter....");
        dht.begin();
        sonic->beginAsync();
        for (int i = 0; i < sonic->getNumberOfSensors(); i++)
            if (!sonic->isInterruptSupported(i))
            {
                LOGGER_FATAL_FMT("Sensor, %i: *FAILED Interrupt!", i);
                LOGGER_FATAL(String(i).c_str());
                Serial.println(i);
            }
        sonic->startAsync(200000);
        LOGGER_VERBOSE("....leave");
    } /*--------------------- end of begin ------------------------------------------------------*/

    virtual void update() override
    {
        LOGGER_VERBOSE("Enter....");

        if (sonic->isFinished())
        {
            _sonicData->down_distance = sonic->getDist_cm(0);
            _sonicData->front_distance = sonic->getDist_cm(1);
            sonic->startAsync(200000);
        }
        LOGGER_VERBOSE("....leave");
    } /*--------------------- end of update -----------------------------------------------------*/

};/*----------------------------------- end of sonic.h class ------------------------------------*/
