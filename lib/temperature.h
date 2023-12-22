#pragma once
/*  File name: temperature.h
 *	Project name: KuckyCopter 2
 *  Date: 2023-10-12
 *  Author: Wilhelm Kuckelsberg
 *  Description: 
 *  https://github.com/adamboardman/pico-onewire
 */

#include <Arduino.h>
#include <TaskManager.h>
#include <HC_SR04.h>
#include "./pico-onewire/one_wire.h"

#include "def.h"

//#define LOCAL_DEBUG
#include "myLogger.h"

typedef struct
{
    float temperature; // Externel (outdoor) temperature
} temperatureData_t;

class Temperature : public Task::Base
{
private:
    rom_address_t address{};
    One_wire *_one_wire;

public:
    temperatureData_t *_temperatureData;
    temperatureData_t __temperatureData;

protected:
public:
    Temperature(const String &name) : Task::Base(name)
    {
        _one_wire = new One_wire(PIN_18B20);
        _one_wire->init();
    }

    Temperature *setModel(temperatureData_t *_model)
    {
        LOGGER_VERBOSE("Enter....");
        _temperatureData = _model;
        LOGGER_VERBOSE("....leave");
        return this;
    } /*--------------------- end of setModel ---------------------------------------------------*/

    virtual void begin() override
    {
        LOGGER_VERBOSE("Enter....");

        LOGGER_VERBOSE("....leave");
    } /*--------------------- end of begin ------------------------------------------------------*/

    virtual void update() override
    {
        LOGGER_VERBOSE("Enter....");

        _one_wire->single_device_read_rom(address);
        // printf("Device Address: %02x%02x%02x%02x%02x%02x%02x%02x\n", address.rom[0], address.rom[1], address.rom[2], 
        //                                                              address.rom[3], address.rom[4], address.rom[5], 
        //                                                              address.rom[6], address.rom[7]);
        _one_wire->convert_temperature(address, true, false);
        //Serial.println(_one_wire->temperature(address),4);
        // printf("Temperature: %3.1foC\n", _one_wire->temperature(address));
         sleep_ms(1000); // was soll das ???

        float temp = _one_wire->temperature(address);
        LOGGER_NOTICE_FMT("Temperatur = %.2f", temp);
        //Serial.println(temp,4);
        _temperatureData->temperature = temp;

        LOGGER_VERBOSE("....leave");
    } /*--------------------- end of update -----------------------------------------------------*/
}; /*----------------------------------- end of temperature.h class -----------------------------*/
