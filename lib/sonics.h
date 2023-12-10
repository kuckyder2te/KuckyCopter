#pragma once
/*  File name: sonic.h
 *	Project name: KuckyCopter
 *  Date: 2022-05-28
 *  Author: Stephan Scholz / Wilhelm Kuckelsberg
 *  Description: Low flight and obstacle detection
 *  HCSR04 max. Distanz 400cm
 *
 *  Impulsdauer 100cm   2,9137 mSec * 2 = 5,8275 mSec
 *              200cm   5,8275            11,655 mSec
 *
 * @param maxDistanceCm  Maximum distance sensor can measure, defaults to 4m for HC-SR04.
 *                       You might want to set this value if you are using different sensor than HC-SR04
 *                       or if you don't care about distances larger than whatever you will set it to
 *                       (therefore reducing time it takes for a single measurement).
 * @param maxTimeoutMicroSec  Single measurement will never take longer than whatever value you provide here.
 *   You might want to do this in order to ensure your program is never blocked for longer than some specific time,
 *   since measurements are blocking.
 *   By default, there is no limit on time (only on distance). By defining timeout, you are again limiting the distance.
 */

#include <Arduino.h>
#include <TaskManager.h>
#include <HC_SR04.h>
// #include <Adafruit_Sensor.h>
//#include "..\lib\pico-onewire\pico_pi_mocks.h"
#include "./pico-onewire/one_wire.h"

#include "def.h"

#define LOCAL_DEBUG
#include "myLogger.h"

//One_wire one_wire(PIN_18B20);
// rom_address_t address{};

typedef struct
{
    float temperature; // Internal (outdoor) temperature
    uint16_t down_distance;
    uint16_t front_distance;
} sonicData_t;

class Sonic : public Task::Base
{
private:
    HC_SR04_BASE *slave;
    HC_SR04_BASE *sonic;
    HC_SR04_BASE *slaves[NUMBER_OF_SLAVES];
    rom_address_t address{};
    One_wire *_one_wire;

    unsigned short maxDistanceCm;
    unsigned long maxTimeoutMicroSec;

public:
    sonicData_t *_sonicData;
    sonicData_t __sonicData; // makes the logger readable "...CHK..."

protected:
public:
    Sonic(const String &name) : Task::Base(name)
    {
        // This is a slave sensor, in this case only on
        slave = new HC_SR04<PIN_ECHO_FRONT>(PIN_TRIGGER_FRONT);
        slaves[0] = slave;
        // Master sensor with echo and trigger pin
        sonic = new HC_SR04<PIN_ECHO_DOWN>(PIN_TRIGGER_DOWN, slaves, NUMBER_OF_SLAVES);

        _one_wire = new One_wire(PIN_18B20);
        _one_wire->init();
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

        sonic->beginAsync();
        for (int i = 0; i < sonic->getNumberOfSensors(); i++)
        {
            if (!sonic->isInterruptSupported(i))
            {
                LOGGER_FATAL_FMT("Sensor, %i: *FAILED Interrupt!", i);
                LOGGER_FATAL(String(i).c_str());
            }
        }
        sonic->startAsync(200000);

        LOGGER_VERBOSE("....leave");
    } /*--------------------- end of begin ------------------------------------------------------*/

    virtual void update() override
    {
        LOGGER_VERBOSE("Enter....");
        if (sonic->isFinished())
        {
            _sonicData->down_distance = sonic->getDist_cm(sonicName::front);
            _sonicData->front_distance = sonic->getDist_cm(sonicName::down);
            sonic->startAsync(200000);
        }

        _one_wire->single_device_read_rom(address);
        // printf("Device Address: %02x%02x%02x%02x%02x%02x%02x%02x\n", address.rom[0], address.rom[1], address.rom[2], address.rom[3], address.rom[4], address.rom[5], address.rom[6], address.rom[7]);
        _one_wire->convert_temperature(address, true, false);
        Serial.println(_one_wire->temperature(address),4);
        // printf("Temperature: %3.1foC\n", _one_wire->temperature(address));
        //sleep_ms(1000);

        _sonicData->temperature = _one_wire->temperature(address);

        LOGGER_VERBOSE("....leave");
    } /*--------------------- end of update -----------------------------------------------------*/

    void temperature_compensssion(float temperature)
    {
        unsigned long maxDistanceDurationMicroSec;
        float speedOfSoundInCmPerMicroSec = 0.03313 + 0.0000606 * temperature; // Cair ≈ (331.3 + 0.606 ⋅ ϑ) m/s

        maxDistanceDurationMicroSec = 2.5 * maxDistanceCm / speedOfSoundInCmPerMicroSec;
        if (maxTimeoutMicroSec > 0)
        {
            maxDistanceDurationMicroSec = min(maxDistanceDurationMicroSec, maxTimeoutMicroSec);
        }
    }
}; /*----------------------------------- end of sonic.h class -----------------------------------*/
