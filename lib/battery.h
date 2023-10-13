#pragma once
/*  File name : battery.h
    Project name : KuCo_Phantom 1
    Author: Stephan Scholz / Wilhelm Kuckelsberg
    Date : 2022-06-13
    Description : Check battery condition.
*/
#include <Arduino.h>
#include <TaskManager.h>

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"

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

            stdio_init_all();
            adc_init();
            adc_gpio_init(PIN_BATTERY);
            adc_select_input(0);
            pinMode(LED_PIN_ALERT, OUTPUT);
            digitalWrite(LED_PIN_ALERT, LOW);

        LOGGER_VERBOSE("....leave");
    }

    virtual void update() override
    {
        LOGGER_VERBOSE("Enter update....");

            //_batteryData->battery_State = analogRead(PIN_BATTERY);
            uint16_t result = adc_read();
            float conversion_factor = 3.3f / (1 << 12);
            _batteryData->battery_State = result * conversion_factor;

        LOGGER_NOTICE_FMT("Battery state = %i", _batteryData->battery_State);

        LOGGER_VERBOSE("....leave");
    }
}; /*--------------------------------- end of class battery.h -----------------------------------*/