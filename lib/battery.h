#pragma once
/*  File name : battery.h
    Project name : KuCo_Phantom 1
    Author: Stephan Scholz / Wilhelm Kuckelsberg
    Date : 2022-06-13
    Description : Check battery condition.
*/
#include <Arduino.h>
#include <TaskManager.h>

// #define LOCAL_DEBUG
#include "myLogger.h"

#include "config.h"

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

    virtual ~Battery() {}

    Battery *setModel(batteryData_t *_model)
    {
        LOGGER_VERBOSE("Enter setModel....");

        _batteryData = _model;
        LOGGER_VERBOSE("....leave");
        return this;

        LOGGER_VERBOSE("....leave");
    }

    virtual void begin()
    {
        LOGGER_VERBOSE("Enter begin....");

        pinMode(LED_PIN_ALERT, OUTPUT);
        digitalWrite(LED_PIN_ALERT, LOW);

        LOGGER_VERBOSE("....leave");
    }

    virtual void update() override
    {
        LOGGER_VERBOSE("Enter update....");

        _batteryData->battery_State = analogRead(PIN_BATTERY);
        LOGGER_NOTICE_FMT("Battery state = %i", _batteryData->battery_State);

        LOGGER_VERBOSE("....leave");
    }
}; /*--------------------------------- end of class battery.h -----------------------------------*/