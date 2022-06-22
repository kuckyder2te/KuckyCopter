#pragma once
/*  File name : battery.h
    Project name : KuCo_Phantom 1
    Author: Wilhelm Kuckelsberg
    Date : 2022-06-13
    Description : Kontrolle des Ladezustands
*/
#include <Arduino.h>
#include <TaskManager.h>

#define PIN_BATTERY   26 // analog
#define PIN_LED_ALERT 8

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
    }

    virtual ~Battery()
    {
    }

    Battery *setModel(batteryData_t *_model)
    { // Rückgabe wert ist das eigene Objekt (this)
        LOGGER_VERBOSE("Enter....");
        _batteryData = _model;
        LOGGER_VERBOSE("....leave");
        return this;
    }

    virtual void begin()
    {
        pinMode(PIN_LED_ALERT, OUTPUT);
        digitalWrite(PIN_LED_ALERT, LOW);
    }

    virtual void update() override
    {
        LOGGER_VERBOSE("Enter....");

        //   _batteryData->battery_State = analogRead(PIN_BATTERY);
        //   LOGGER_NOTICE_FMT("Battery state = %i", _batteryData->battery_State);

        LOGGER_VERBOSE("....leave");
    }
}; /*--------------------------------- end of class battery.h --------------------------*/