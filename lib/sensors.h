#pragma once

/*  File name: pidController.h
 *	Project name: KuCo_Phantom 1
 *  Date: 2022-005-28
 *  Author: Wilhelm Kuckelsberg
 */
 
#include <Arduino.h>
#include <TaskManager.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_SPIDevice.h>
#include <Adafruit_Sensor.h>
#include <MPU9250_asukiaaa.h>
#include <Adafruit_BMP280.h>
#include "..\lib\def.h"
#include "myLogger.h"

typedef struct {
    float  pitch;
    float  roll;
    float  yaw;
	double   compass;			///< Real compass direction
    float _pressure;        //  BMP280 values
    float _altitude;
    float _temperature_baro;
}sensorData_t;

    
class Sensor : public Task::Base {
    //Klasssenvarialblen
    float _aX, _aY, _aZ, _aSqrt, _gX, _gY, _gZ, _mDirection, _mX, _mY, _mZ; // MPU9520 values

private:
 
protected:     
    MPU9250_asukiaaa *_mpu9250;     // Speicherplatz reserviert
    Adafruit_BMP280 *_bmp280;
    sensorData_t *_sensorData;      

public:
    Sensor(const String& name) : Task::Base(name) {
        LOGGER_VERBOSE("Enter....");
        // pinMode(GYRO_LED, OUTPUT);
        // digitalWrite(GYRO_LED, LOW);
        LOGGER_VERBOSE("....leave");         
    }

    virtual ~Sensor() {}

    Sensor* setModel(sensorData_t* _model){    // Rückgabe wert ist das eigene Objekt (this)
        LOGGER_VERBOSE("Enter....");
        _sensorData = _model;
        LOGGER_VERBOSE("....leave");
        return this;
    }

    virtual void begin() override {
    LOGGER_VERBOSE("Enter....");
        Wire.begin();
        LOGGER_NOTICE("MPU9250 initialized");
        _mpu9250 = new MPU9250_asukiaaa();  // Adresse in Variable speichern
        LOGGER_NOTICE("End init MPU9250");

        LOGGER_NOTICE("BMP280 initialized");
        _bmp280 = new Adafruit_BMP280(); // Adresse in Variable// Adresse in Variable speichern
        LOGGER_NOTICE("End init BMP280");

            _bmp280->begin(0x76);
            _mpu9250->beginAccel();
            _mpu9250->beginGyro();
            _mpu9250->beginMag();

            // You can set your own offset for mag values
            // _mpu9250->magXOffset = -50;
            // _mpu9250->magYOffset = -55;
            // _mpu9250->magZOffset = -10;

        LOGGER_VERBOSE("....leave");   
    }   /* ------------------ end of begin --------------------------------------------*/

    virtual void enter() override {


    LOGGER_VERBOSE("Enter....");

        if (_mpu9250->accelUpdate() == 0) {
            LOGGER_NOTICE("Update accelerometer successful");
            _aX = _mpu9250->accelX();
            _aY = _mpu9250->accelY();
            _aZ = _mpu9250->accelZ();
            _aSqrt = _mpu9250->accelSqrt();
        } else {
            LOGGER_FATAL("Accelerometer update failed!");
        }

        if (_mpu9250->gyroUpdate() == 0) {
            LOGGER_NOTICE("Update gyrometer successful"); 
            _gX = _mpu9250->gyroX();
            _gY = _mpu9250->gyroY();
            _gZ = _mpu9250->gyroZ();
        } else {
            LOGGER_FATAL("Gyrometer update failed!");
        }

        if (_mpu9250->magUpdate() == 0) {
            LOGGER_NOTICE("Update magnetometer successful");
            _mX = _mpu9250->magX();
            _mY = _mpu9250->magY();
        //    _mZ = _mpu9250->magZ();
            _mDirection = _mpu9250->magHorizDirection();
        } else {
            LOGGER_FATAL("Magnetometer update failed!");
        }

//        if (_bmp280->takeForcedMeasurement()) {
            LOGGER_NOTICE("ForcedMeasurement successful");
            _sensorData->_temperature_baro = _bmp280->readTemperature();
            _sensorData->_pressure = _bmp280->readPressure();
            _sensorData->_altitude = _bmp280->readAltitude(1013.25); /* Adjusted to local forecast! */
        // } else {
        // LOGGER_FATAL("Forced measurement failed!");    
        // }  

    LOGGER_VERBOSE("....leave"); 
    }   /* ------------------ end of enter -------------------------------------------*/

    virtual void update() override {
    LOGGER_VERBOSE("Enter...."); 
        //Check thresholds, Warnings if too stall
 
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

        LOGGER_NOTICE_FMT("Temperature =  %.2f*C", _sensorData->_temperature_baro);
        LOGGER_NOTICE_FMT("Pressure =  %.2fPa", _sensorData->_pressure);
        LOGGER_NOTICE_FMT("Approx altitude =  %.2fm", _sensorData->_altitude);

    LOGGER_VERBOSE("....leave");   
    }   /*------------------------------- end of update -------------------------------*/
};  /*----------------------------------- end of sensor.h class -----------------------*/
