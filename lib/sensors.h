#pragma once
/*  File name: sensors.h
 *	Project name: KuCo_Phantom 1
 *  Date: 2022-05-28
 *  Author: Wilhelm Kuckelsberg
 *  Description: Lage und Höhen Position
 *
 *  Description: My location: Germany, 51373 Leverkusen Sperlingsweg 9, roof
 * 	double myRealAltitude = 61;
 *
 *  Decimal degrees = Degrees + (Minutes/60) + (Seconds/3600)
 * 51,.033f; // Declination at Leverkusen, Germany is 51 degrees 1 minutes and  59.9988 seconds on 2022-12-21
 */

#include <Arduino.h>
#include <TaskManager.h>
// #include <Adafruit_I2CDevice.h>
// #include <Adafruit_SPIDevice.h>
// #include <Adafruit_Sensor.h>
#include <MPU9250.h>
#include <MS5611.h>

//#define LOCAL_DEBUG
#include "myLogger.h"

typedef struct
{
    float pitch;
    float roll;
    float yaw;
    double compass;
    float pressure;
    float altitude;
    float temperature_baro;
    float _seaLevel;
} sensorData_t;

#define HOME_ALTITUDE 61

class Sensor : public Task::Base
{

private:
    uint32_t start, stop, count;
    MPU9250Setting setting;
    uint16_t _updateCounter;

protected:
    MPU9250 _mpu9250; // Speicherplatz reserviert
    MS5611 _ms5611;
    sensorData_t *_sensorData;
    sensorData_t __sensorData;

public:
    Sensor(const String &name) : Task::Base(name)
    {
        LOGGER_VERBOSE("Enter....");
        LOGGER_VERBOSE("....leave");
    }

    virtual ~Sensor() {}

    Sensor *setModel(sensorData_t *_model)
    { 
        LOGGER_VERBOSE("Enter....");
        _sensorData = _model;
        LOGGER_VERBOSE("....leave");
        return this;
    } /* ------------------ end of setModel ---------------------------------------------*/

    virtual void begin() override
    {
        LOGGER_VERBOSE("Enter....");
        Wire.begin();
        LOGGER_NOTICE("MPU9250 initialized");
 
        setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
        setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
        setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
        setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
        setting.gyro_fchoice = 0x03;
        setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
        setting.accel_fchoice = 0x01;
        setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

        _mpu9250.setMagneticDeclination(51);
        if (!_mpu9250.setup(0x68))
        { // change to your own address
            while (1)
            {
                LOGGER_FATAL("MPU connection failed.");
                delay(5000);
            }
        }
        LOGGER_NOTICE("End init MPU9250");

        LOGGER_NOTICE("MS5611 initialized");
    //    _ms5611 = new MS5611();
        LOGGER_NOTICE("End init MS5611");

//        LOGGER_WARNING_FMT("File %s : MS5611 lib vesion: %s", __FILE__, MS5611_LIB_VERSION);

        if (_ms5611.begin() == true)
        {
            LOGGER_NOTICE("MS5611 found.");
        }
        else
        {
            LOGGER_FATAL("MS5611 not found. halt.");
            while (1) ;
        }

        _ms5611.setOversampling(OSR_STANDARD);

        LOGGER_WARNING_FMT("Oversampling = %i", _ms5611.getOversampling());
        delay(500);
        LOGGER_VERBOSE("....leave");
    } /* ------------------ end of begin ----------------------------------------------*/

    virtual void update() override
    {
        LOGGER_VERBOSE("Enter....");
        LOGGER_VERBOSE("....leave");
    } /* ------------------ end of update ---------------------------------------------*/

    virtual void enter() override
    {
        LOGGER_VERBOSE("Enter....");
        LOGGER_NOTICE_FMT("Wire %d",Wire.availableForWrite());
        if (_mpu9250.update()){
            LOGGER_NOTICE("_mpu9250.update");

            _sensorData->yaw = _mpu9250.getYaw();
            _sensorData->pitch = _mpu9250.getPitch();
            _sensorData->roll = _mpu9250.getRoll();

            #ifdef _SERIAL_STUDIO
                // Serial2.printf("/*%.2f,%.2f,%.2f*/\r\n",_mpu9250.getYaw(), _mpu9250.getPitch(), _mpu9250.getRoll());
            #else
                LOGGER_NOTICE_FMT("Yaw: %0.2f Pitch: %0.2f Roll: %0.2f",_mpu9250.getYaw(),_mpu9250.getPitch(),_mpu9250.getRoll());
                LOGGER_NOTICE_FMT_CHK(_sensorData->yaw, __sensorData.yaw, "Yaw = %0.2f", _sensorData->yaw);
                LOGGER_NOTICE_FMT_CHK(_sensorData->pitch, __sensorData.pitch, "Pitch = %0.2f", _sensorData->pitch);
                LOGGER_NOTICE_FMT_CHK(_sensorData->roll, __sensorData.yaw, "Roll = %0.2f", _sensorData->yaw);
                LOGGER_NOTICE_FMT_CHK(_sensorData->compass, __sensorData.compass, "compass = %0.2f", _sensorData->compass);                   
            #endif    
            LOGGER_NOTICE("_mpu9250 leave");
 
        }else{
            LOGGER_NOTICE("_ms5611.read");
            _ms5611.read(); // uses default OSR_ULTRA_LOW  (fastest)
            _sensorData->_seaLevel = getSeaLevel(_ms5611.getPressure(), HOME_ALTITUDE); // Warum immer hier
            _sensorData->temperature_baro = _ms5611.getTemperature();
            _sensorData->pressure = _ms5611.getPressure();
            _sensorData->altitude = getAltitude(_sensorData->pressure, _sensorData->_seaLevel);

            #ifdef _SERIAL_STUDIO
                /* Ausgabe von Rohdaten*/
                // Serial2.printf("/*%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f*/\r\n",_aX, _aY, _aZ, _aSqrt, _gX, _gY, _gZ, _mDirection, _mX, _mY, _mZ);
                // Serial2.printf("/*%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f*/\r\n",
                //                    _aX, _aY, _aZ, _gX, _gY, _gZ, _mX, _mY, _mZ);
            #else
                LOGGER_NOTICE_FMT_CHK(_sensorData->altitude, __sensorData.altitude, "Altitude = %0.2f", _sensorData->altitude);
                LOGGER_NOTICE_FMT_CHK(_sensorData->pressure, __sensorData.pressure, "Pressure = %0.2f", _sensorData->pressure);
                LOGGER_NOTICE_FMT_CHK(_sensorData->_seaLevel, __sensorData._seaLevel, "SeaLevel = %0.2f", _sensorData->_seaLevel);
                LOGGER_NOTICE_FMT_CHK(_sensorData->temperature_baro, __sensorData.temperature_baro, "Temperature_baro = %0.2f", _sensorData->temperature_baro);
            #endif

            LOGGER_NOTICE("_ms5611 leave");
        }
        LOGGER_VERBOSE("....leave");
    } /*------------------------------- end of enter ---------------------------------*/

    float getAltitude(double press, double seaLevel)
    {
        // return (1.0f - pow(press/101325.0f, 0.190295f)) * 4433000.0f;
        // return ((pow((seaLevel / press), 1/5.257) - 1.0) * (temp + 273.15)) / 0.0065;
        return (44330.0f * (1.0f - pow((double)press / (double)seaLevel, 0.1902949f)));
    } /*------------------------------- end of getAltitude ----------------------------*/

    // Calculate sea level from Pressure given on specific altitude
    double getSeaLevel(double pressure, double altitude)
    {
        return ((double)pressure / pow(1.0f - ((double)altitude / 44330.0f), 5.255f));
    } //-------------------------------- end of getSeaLevel ---------------------------------------
};    /*----------------------------------- end of sensor.h class ---------------------*/
