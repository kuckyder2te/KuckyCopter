#pragma once
/*  File name: sensors.h
 *	Project name: KuCo_Phantom 1
 *  Date: 2022-05-28
 *  Author: Wilhelm Kuckelsberg
 *  Description: Lage und Höhen Position
 * 
 *  Description: My location: Germany, 51373 Leverkusen Sperlingsweg 9, roof
 * 	double myRealAltitude = 61;
 */

#include <Arduino.h>
#include <TaskManager.h>
// #include <Adafruit_I2CDevice.h>
// #include <Adafruit_SPIDevice.h>
// #include <Adafruit_Sensor.h>
#include <MPU9250_asukiaaa.h>
#include <MS5611.h>

//#define LOCAL_DEBUG
#include "myLogger.h"

typedef struct
{
    float pitch;
    float roll;
    float yaw;
    double compass;  
    float _pressure; 
    float _altitude;
    float _temperature_baro;
    float _seaLevel;
} sensorData_t;

#define HOME_ALTITUDE 61

class Sensor : public Task::Base
{

private:
    uint32_t start, stop, count, result;
    float _aX, _aY, _aZ, _aSqrt, _gX, _gY, _gZ, _mDirection, _mX, _mY, _mZ; // MPU9520 values for filter

protected:
    MPU9250_asukiaaa *_mpu9250; // Speicherplatz reserviert
    MS5611 *_ms5611;
    sensorData_t *_sensorData;

public:
    Sensor(const String &name) : Task::Base(name)
    {
        LOGGER_VERBOSE("Enter....");
        LOGGER_VERBOSE("....leave");
    }

    virtual ~Sensor() {}

    Sensor *setModel(sensorData_t *_model)
    { // Rückgabe wert ist das eigene Objekt (this)
        LOGGER_VERBOSE("Enter....");
        _sensorData = _model;
        LOGGER_VERBOSE("....leave");
        return this;
    }

    virtual void begin() override
    {
        LOGGER_VERBOSE("Enter....");
        Wire.begin();
        LOGGER_NOTICE("MPU9250 initialized");
            _mpu9250 = new MPU9250_asukiaaa(); // Adresse in Variable speichern
        LOGGER_NOTICE("End init MPU9250");

        uint8_t sensorId;
        int result;

        result = _mpu9250->readId(&sensorId);
        if (result == 0) {
            LOGGER_WARNING_FMT("sensorId: %i", sensorId);
        } else {
            LOGGER_FATAL_FMT("Cannot read sensorId %i", result);
        }
        LOGGER_WARNING_FMT("File %s : MPU lib version: %s", __FILE__, MPU9250_LIB_VERSION);

        _mpu9250->beginAccel();
        _mpu9250->beginGyro();
        _mpu9250->beginMag();

        // You can set your own offset for mag values
        // _mpu9250->magXOffset = -50;
        // _mpu9250->magYOffset = -55;
        // _mpu9250->magZOffset = -10;

        LOGGER_NOTICE("MS5611 initialized");
            _ms5611 = new MS5611();
        LOGGER_NOTICE("End init MS5611");

        LOGGER_WARNING_FMT("File %s : MS561 lib vesion: %s", __FILE__, MS5611_LIB_VERSION);

        if (_ms5611->begin() == true)
            {
                LOGGER_NOTICE("MS5611 found.");
            }
            else
            {
                LOGGER_FATAL("MS5611 not found. halt.");
                while (1);
            }

            _ms5611->setOversampling(OSR_ULTRA_HIGH);

        LOGGER_WARNING_FMT("Oversampling = %i", _ms5611->getOversampling());
        delay(500);
        LOGGER_VERBOSE("....leave");
    } /* ------------------ end of begin ----------------------------------------------*/

    virtual void enter() override
    {

        LOGGER_VERBOSE("Enter....");

        if (_mpu9250->accelUpdate() == 0)
        {
            LOGGER_NOTICE("Update accelerometer successful");
            _aX = _mpu9250->accelX();
            _aY = _mpu9250->accelY();
            _aZ = _mpu9250->accelZ();
            _aSqrt = _mpu9250->accelSqrt();
        }
        else
        {
            LOGGER_FATAL("Accelerometer update failed!");
        }

        if (_mpu9250->gyroUpdate() == 0)
        {
            LOGGER_NOTICE("Update gyrometer successful");
            _gX = _mpu9250->gyroX();
            _gY = _mpu9250->gyroY();
            _gZ = _mpu9250->gyroZ();
        }
        else
        {
            LOGGER_FATAL("Gyrometer update failed!");
        }

        if (_mpu9250->magUpdate() == 0)
        {
            LOGGER_NOTICE("Update magnetometer successful");
            _mX = _mpu9250->magX();
            _mY = _mpu9250->magY();
            _mZ = _mpu9250->magZ();
            _mDirection = _mpu9250->magHorizDirection();
        }
        else
        {
            LOGGER_FATAL("Magnetometer update failed!");
        }

        start = micros();
        result = _ms5611->read();   // uses default OSR_ULTRA_LOW  (fastest)
        stop = micros();

        _sensorData->_seaLevel = getSeaLevel(_ms5611->getPressure(), HOME_ALTITUDE);  // Warum immer hier

        _sensorData->_temperature_baro = _ms5611->getTemperature();
        _sensorData->_pressure = _ms5611->getPressure();
        _sensorData->_altitude = getAltitude(_sensorData->_pressure, _sensorData->_seaLevel);

        LOGGER_VERBOSE("....leave");
    } /* ------------------ end of enter ---------------------------------------------*/

    virtual void update() override
    {
    LOGGER_VERBOSE("Enter....");

        LOGGER_NOTICE_FMT("accelX:  %.2f", _aX);
        LOGGER_NOTICE_FMT("accelY %.2f", _aY);
        LOGGER_NOTICE_FMT("accelZ %.2f", _aZ);
        LOGGER_NOTICE_FMT("accelSqrt %.2f", _aSqrt);

        LOGGER_NOTICE_FMT("gyroX:  %.2f", _gX);
        LOGGER_NOTICE_FMT("gyroY %.2f", _gY);
        LOGGER_NOTICE_FMT("gyroZ %.2f", _gZ);

        LOGGER_NOTICE_FMT("magX:  %.2f", _mX);
        LOGGER_NOTICE_FMT("magY %.2f", _mY);
        LOGGER_NOTICE_FMT("magZ %.2f", _mZ);
        LOGGER_NOTICE_FMT("horizontalDirection %.2f", _mDirection);

        LOGGER_NOTICE_FMT("Sea level =  %.2fPa", _sensorData->_seaLevel);
        LOGGER_NOTICE_FMT("Pressure =  %.2fPa", _sensorData->_pressure);
        LOGGER_NOTICE_FMT("Temperature =  %.2f*C", _sensorData->_temperature_baro);   
        LOGGER_NOTICE_FMT("Approx altitude =  %.4fm", _sensorData->_altitude);

    LOGGER_VERBOSE("....leave");
    } /*------------------------------- end of update ---------------------------------*/

    float getAltitude(double press, double seaLevel) {
        //return (1.0f - pow(press/101325.0f, 0.190295f)) * 4433000.0f;
        //return ((pow((seaLevel / press), 1/5.257) - 1.0) * (temp + 273.15)) / 0.0065;
        return (44330.0f * (1.0f - pow((double)press / (double)seaLevel, 0.1902949f)));
    } /*------------------------------- end of getAltitude ----------------------------*/

    	// Calculate sea level from Pressure given on specific altitude
	double getSeaLevel(double pressure, double altitude)
	{
		return ((double)pressure / pow(1.0f - ((double)altitude / 44330.0f), 5.255f));
	}//-------------------------------- end of getSeaLevel --------------------------------------	

};    /*----------------------------------- end of sensor.h class ---------------------*/

//#undef _DEBUG_