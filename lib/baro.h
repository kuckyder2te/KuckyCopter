#pragma once
/*  File name : baro2.h
    Project name : KuCo_xx
    Date : 2022-05-15

    Description : Drohne
    Hardware : Baro : MS5611 / BMP280 for test
               
*/

#include <TaskManager.h>
#include <Adafruit_BMP280.h>

typedef struct{
    double baseline;
    double pressure;
    double altitude;
    double read_altitude;
    double sealevel;
    double temperature;
}baroData_t;

class Baro : public Task::Base {

protected:
    Adafruit_BMP280 *_baro;
    baroData_t *_baroData;

public:
    Baro(const String& name)
    : Task::Base(name) {
    }

    virtual ~Baro() {}

        Baro* setModel(baroData_t* _model){    // Rückgabe wert ist das eigene Objekt (this)
    LOGGER_VERBOSE("Enter....");
    _baroData = _model;
    LOGGER_VERBOSE("....leave");
    return this;
    }

    virtual void begin() override {
        if (!_baro->begin()) {
            LOGGER_FATAL("Could not find a valid BMP280 sensor");
        while (1) delay(10);
        }
    /* Default settings from datasheet. */
    _baro->setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                        Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                        Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                        Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                        Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
    }

    virtual void enter() override {
        if (_baro->takeForcedMeasurement()) {
            _baroData->temperature = _baro->readTemperature();
            _baroData->pressure = _baro->readPressure();
            _baroData->altitude = _baro->readAltitude(1013.25); /* Adjusted to local forecast! */
        } else {
        LOGGER_FATAL("Forced measurement failed!");    
        }
    }
    virtual void update() override {
        LOGGER_NOTICE_FMT("Temperature =  %f *C", _baroData->temperature);
        LOGGER_NOTICE_FMT("Pressure =  %f Pa", _baroData->pressure);
        LOGGER_NOTICE_FMT("Approx altitude =  %f m", _baroData->altitude);    
    }
};
