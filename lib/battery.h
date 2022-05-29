#pragma once

#include <Arduino.h>
#include <TaskManager.h>
#include "..\lib\def.h"

typedef struct{
    int battery_State;
}batteryData_t;

class Battery : public Task::Base {

protected:
    batteryData_t *_batteryData;
    
public:
    Battery(const String& name)
    : Task::Base(name) {
    }

    virtual ~Battery() {
    }

    Battery* setModel(batteryData_t* _model){    // Rückgabe wert ist das eigene Objekt (this)
    LOGGER_VERBOSE("Enter....");
        _batteryData = _model;
    LOGGER_VERBOSE("....leave");
        return this;
    }

    virtual void begin(){}

    virtual void update() override {
    LOGGER_VERBOSE("Enter....");

     //   _batteryData->battery_State = analogRead(PIN_BATTERY);
     //   LOGGER_NOTICE_FMT("Battery state = %i", _batteryData->battery_State);

    LOGGER_VERBOSE("....leave");    
    }
};  /*--------------------------------- end of class battery.h --------------------------*/