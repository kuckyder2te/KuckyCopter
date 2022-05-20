#pragma once

#include <Arduino.h>
#include <TaskManager.h>
#include <MPU9250.h>
#include <Adafruit_BMP280.h>
#include "..\lib\def.h"
#include "myLogger.h"

typedef struct {
    float  pitch;
    float  roll;
    float  yaw;
	double   compass;			///< Real compass direction
	int16_t  horz_Position;
	float	 angularVelocity;	///< Angular velocity
	int16_t  compassDiff;		///< Difference between two compass values
	short 	 vectorOld[3];
	//bool	 isMove;
//    float getAccelX_mss;
}sensorData_t;

class Sensor : public Task::Base {
    bool b;         // Klassenvariable
    int status;
    
    float _accelX_mss, _accelY_mss, _accelZ_mss;  //   MPU9250 values
    float _gyroX_rads, _gyroY_rads, _gyroZ_rads;  
    float _magX_uT, _magY_uT, _magZ_uT;  
    float _temperature_C;

    float _pressure;        //  BMP280 values
    float _altitude;
    float _temperature_baro;

private:
    // MPU9250Setting setting;
 
protected:
	MPU9250  *_mpu9250;         // Speicherplatz reserviert
    Adafruit_BMP280 *_bmp280;
    sensorData_t *_sensorData;      

public:
    Sensor(const String& name) : Task::Base(name) ,b(false){
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
        _mpu9250 = new MPU9250(Wire, 0x68);  // Adresse in Variable speichern
        LOGGER_NOTICE("Start init MPU");

            // start communication with IMU 
            status = _mpu9250->begin();
            if (status < 0) {
                LOGGER_FATAL("IMU initialization unsuccessful");
                LOGGER_FATAL("Check IMU wiring or try cycling power");
                LOGGER_FATAL_FMT("Status: %i", status);
                while(1) {}
            }
            // setting the accelerometer full scale range to +/-8G 
            _mpu9250->setAccelRange(MPU9250::ACCEL_RANGE_8G);
            // setting the gyroscope full scale range to +/-500 deg/s
            _mpu9250->setGyroRange(MPU9250::GYRO_RANGE_500DPS);
            // setting DLPF bandwidth to 20 Hz
            _mpu9250->setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
            // setting SRD to 19 for a 50 Hz update rate
            _mpu9250->setSrd(19);
        LOGGER_NOTICE("End init MPU");

        LOGGER_NOTICE("BMP280 initialized");
        _bmp280 = new Adafruit_BMP280();  // Adresse in Variable speichern

        LOGGER_NOTICE("Start init BMP280");
        if (!_bmp280->begin()) {
            LOGGER_FATAL("Could not find a valid BMP280 sensor");
        while (1) delay(10);
        }
        /* Default settings from datasheet. */
        _bmp280->setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                        Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                        Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                        Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                        Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
        LOGGER_NOTICE("End init BMP280");
    LOGGER_VERBOSE("....leave");   
    }

    virtual void enter() override {
    LOGGER_VERBOSE("Enter....");
        _mpu9250->readSensor();

        _accelX_mss = _mpu9250->getAccelX_mss();
        _accelY_mss = _mpu9250->getAccelY_mss();
        _accelZ_mss = _mpu9250->getAccelZ_mss();

        _gyroX_rads = _mpu9250->getGyroX_rads();
        _gyroY_rads = _mpu9250->getGyroY_rads();
        _gyroZ_rads = _mpu9250->getGyroZ_rads();

        _magX_uT = _mpu9250->getMagX_uT();
        _magY_uT = _mpu9250->getMagY_uT();
        _magZ_uT = _mpu9250->getMagZ_uT();

        _temperature_C = _mpu9250->getTemperature_C();

        if (_bmp280->takeForcedMeasurement()) {
            _temperature_baro = _bmp280->readTemperature();
            _pressure = _bmp280->readPressure();
            _altitude = _bmp280->readAltitude(1013.25); /* Adjusted to local forecast! */
        } else {
        LOGGER_FATAL("Forced measurement failed!");    
        }  
    LOGGER_VERBOSE("....leave"); 
    }

    virtual void update() override {
    LOGGER_VERBOSE("Enter...."); 
        //Check thresholds, Warnings if too stall
        LOGGER_NOTICE_FMT("getAccelX_mss %f", _accelX_mss);
        LOGGER_NOTICE_FMT("getAccelY_mss %f", _accelY_mss);
        LOGGER_NOTICE_FMT("getAccelZ_mss %f", _accelZ_mss);
        LOGGER_NOTICE_FMT("getGyroX_rads %f", _gyroX_rads);
        LOGGER_NOTICE_FMT("getGyroY_rads %f", _gyroY_rads);
        LOGGER_NOTICE_FMT("getGyroZ_rads %f", _gyroZ_rads);
        LOGGER_NOTICE_FMT("getMagX_uT %f", _magX_uT);
        LOGGER_NOTICE_FMT("getMagY_uT %f", _magY_uT);
        LOGGER_NOTICE_FMT("getMagZ_uT %f", _magZ_uT);
        LOGGER_NOTICE_FMT("Temperatur Gyro %f", _temperature_C); 

        LOGGER_NOTICE_FMT("Temperature =  %f*C", _temperature_baro);
        LOGGER_NOTICE_FMT("Pressure =  %fPa", _pressure);
        LOGGER_NOTICE_FMT("Approx altitude =  %fm", _altitude);

    LOGGER_VERBOSE("....leave");   
    }
//    String getMonitor(){     
//        char buf[20];
//        //sprintf(buf,"/d,/d,/d",_gyroData.roll,_gyroData.pitch,_gyroData.yaw);
//        return buf;
//    }
};  /*----------------------------------- end of sensor.h class -----------------------*/
