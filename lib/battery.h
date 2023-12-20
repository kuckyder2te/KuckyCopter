#pragma once
/*  File name : battery.h
    Project name : KuckyCopter
    Author: Stephan Scholz / Wilhelm Kuckelsberg
    Date : 2022-11-21
    Description : Check battery condition.
*/
#include <Arduino.h>
#include <TaskManager.h>

#include <stdio.h>
// #include "pico/stdlib.h"
#include "hardware/adc.h"

//#define LOCAL_DEBUG
#include "myLogger.h"

#include "def.h"

/* 12,2V    933
   11,3V    872
   10,1V    777
    9,5V    730
    8,1V    626
*/

typedef struct
{
    int battery_State;
} batteryData_t;

class Battery : public Task::Base
{

protected:
    batteryData_t *_batteryData;

public:
    Battery(const String &name)
        : Task::Base(name)
    {
        LOGGER_VERBOSE("Enter public....");
        LOGGER_VERBOSE("....leave");
    }

    Battery *setModel(batteryData_t *_model)
    {
        LOGGER_VERBOSE("Enter setModel....");
        _batteryData = _model;
        LOGGER_VERBOSE("....leave");
        return this;
        LOGGER_VERBOSE("....leave");
    } /*------------------------ end of configuration -------------------------------------------*/

    virtual void begin()
    {
        LOGGER_VERBOSE("Enter begin....");
        pinMode(LED_PIN_ALERT, OUTPUT);
        digitalWrite(LED_PIN_ALERT, LOW);
        LOGGER_VERBOSE("....leave");
    } /*------------------------ end of begin ---------------------------------------------------*/

    virtual void update() override
    {
        static uint32_t lastMillis = millis();
        static bool state;

        LOGGER_VERBOSE("Enter update....");

        _batteryData->battery_State = analogRead(PIN_BATTERY);

        // LOGGER_NOTICE_FMT("Battery state = %i", _batteryData->battery_State);

        if (_batteryData->battery_State > 860)
        {
            LOGGER_NOTICE_FMT("Battery is full = %i", _batteryData->battery_State);
            digitalWrite(LED_PIN_ALERT, LOW);           
        }

        else if (_batteryData->battery_State > 780 || _batteryData->battery_State >= 859)
        {
            if (millis() - lastMillis > 1000)
            {
                LOGGER_NOTICE_FMT("Battery is OK = %i", _batteryData->battery_State);
                state = !state;
                digitalWrite(LED_PIN_ALERT, state);
                lastMillis = millis();
            }
        }

        else if (_batteryData->battery_State < 779)
        {
            if (millis() - lastMillis > 100)
            {
                LOGGER_NOTICE_FMT("Battery is OK = %i", _batteryData->battery_State);
                state = !state;
                digitalWrite(LED_PIN_ALERT, state);
                lastMillis = millis();
            }
        }
        LOGGER_VERBOSE("....leave");
    } /*------------------------ end of update ---------------------------------------------------*/

}; /*--------------------------------- end of class battery.h -----------------------------------*/