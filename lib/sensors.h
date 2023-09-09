#pragma once
/*  File name: sensors.h
 *	Project name: KuCo_Phantom 1
 *  Date: 2022-05-28
 *  Author: Stephan Scholz / Wilhelm Kuckelsberg
 *  Description: Lage und Höhen Position
 *
 *  Description: My location: Germany, 51373 Leverkusen Sperlingsweg 9, roof
 * 	double myRealAltitude = 61;
 *
 *  Decimal degrees = Degrees + (Minutes/60) + (Seconds/3600)
 * 51,.033f; // Declination at Leverkusen, Germany is 51 degrees 1 minutes and  59.9988 seconds on 2022-12-21
 * Breitengrad: 51.048 / Längengrad: 6.9942
*   Adresse: Sperlingsweg 9, 51373 Leverkusen, Deutschland
Höhe 55m von Strasse zum Dach + ca. 6m
 */

#include <Arduino.h>
#include <TaskManager.h>
#include <MPU9250.h>
#include <MS5611.h>

//#define LOCAL_DEBUG
#include "myLogger.h"

typedef struct
{
    int16_t pitch;
    int16_t roll;
    float yaw;
    double compass;
    float pressure;
    float altitude;
    float temperature_baro;
    float seaLevel;
} sensorData_t;

#define HOME_ALTITUDE 61

class Sensor : public Task::Base
{

private:
    MPU9250Setting setting;
    uint16_t _updateCounter;

protected:
    MPU9250 _mpu9250; // Speicherplatz reserviert
    MS5611 _ms5611;
    sensorData_t *_sensorData;
    sensorData_t __sensorData;      // for better reading

public:
    Sensor(const String &name) : Task::Base(name)
    {
        LOGGER_VERBOSE("Enter....");
        LOGGER_VERBOSE("....leave");
    }

    Sensor *setModel(sensorData_t *_model)
    {
        LOGGER_VERBOSE("Enter....");
        _sensorData = _model;
        LOGGER_VERBOSE("....leave");
        return this;
    } /* ------------------ end of setModel ----------------------------------------------------*/

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
        {
            while (1)
            {
                LOGGER_FATAL("MPU connection failed.");
                delay(5000);
            }
        }
        LOGGER_NOTICE("End init MPU9250");

        if (_ms5611.begin() == true)
        {
            LOGGER_NOTICE("MS5611 found.");
        }
        else
        {
            LOGGER_FATAL("MS5611 not found. halt.");
            while (1)
                ;
        }

        _ms5611.setOversampling(OSR_STANDARD);

        LOGGER_NOTICE_FMT("Oversampling = %i", _ms5611.getOversampling());
        delay(100 );
        LOGGER_VERBOSE("....leave");
    } /* ------------------ end of begin --------------------------------------------------------*/

    virtual void update() override
    {
        LOGGER_VERBOSE("Enter....");
        LOGGER_VERBOSE("....leave");
    } /* ------------------ end of update -------------------------------------------------------*/

    virtual void enter() override
    {
        LOGGER_VERBOSE("Enter....");
        //    LOGGER_NOTICE_FMT("Wire %d", Wire.availableForWrite());
        if (_mpu9250.update())
        {
            LOGGER_VERBOSE("_mpu9250.update");

            _sensorData->yaw = _mpu9250.getYaw();
            _sensorData->pitch = _mpu9250.getPitch()*100;       // For more range
            _sensorData->roll = _mpu9250.getRoll()*100;         // For more range

            display_imu_data();

            LOGGER_VERBOSE("_mpu9250 leave");
        }
        else
        {
            LOGGER_VERBOSE("_ms5611.read");
            _ms5611.read();                                                             // uses default OSR_ULTRA_LOW  (fastest)
        //    _sensorData->seaLevel = getSeaLevel(_ms5611.getPressure(), HOME_ALTITUDE); 
            _sensorData->temperature_baro = _ms5611.getTemperature();
            _sensorData->pressure = _ms5611.getPressure();
            _sensorData->altitude = getAltitude(_ms5611.getPressure(), (getSeaLevel(_ms5611.getPressure(), HOME_ALTITUDE)));

            display_baro_data();

            LOGGER_VERBOSE("_ms5611 leave");
        }
        LOGGER_VERBOSE("....leave");
    } /*------------------------------- end of enter --------------------------------------------*/

    float getAltitude(double press, double seaLevel)
    {
        // return (1.0f - pow(press/101325.0f, 0.190295f)) * 4433000.0f;
         return ((pow((seaLevel / press), 1/5.257) - 1.0) * ( _ms5611.getTemperature() + 273.15)) / 0.0065;
        //  return (44330.0f * (1.0f - pow((double)press / (double)seaLevel, 0.1902949f)));
    } /*------------------------------- end of getAltitude --------------------------------------*/

    // Calculate sea level from Pressure given on specific altitude
    double getSeaLevel(double pressure, double altitude)
    {
        return ((double)pressure / pow(1.0f - ((double)altitude / 44330.0f), 5.255f));
    } /*-------------------------------- end of getSeaLevel -------------------------------------*/

    void display_imu_data()
    {

        LOGGER_NOTICE_FMT_CHK(_sensorData->yaw, __sensorData.yaw, "Yaw = %0.2f", _sensorData->yaw);
        LOGGER_NOTICE_FMT_CHK(_sensorData->pitch, __sensorData.pitch, "Pitch = %0.2f", _sensorData->pitch);
        LOGGER_NOTICE_FMT_CHK(_sensorData->roll, __sensorData.yaw, "Roll = %0.2f", _sensorData->roll);
        LOGGER_NOTICE_FMT_CHK(_sensorData->compass, __sensorData.compass, "compass = %0.2f", _sensorData->compass);
    } /*-------------------------------- end of display_imu_data --------------------------------*/

    void display_baro_data()
    {
        LOGGER_NOTICE_FMT_CHK(_sensorData->altitude, __sensorData.altitude, "Altitude = %0.2f", _sensorData->altitude);
        LOGGER_NOTICE_FMT_CHK(_sensorData->pressure, __sensorData.pressure, "Pressure = %0.2f", _sensorData->pressure);
        LOGGER_NOTICE_FMT_CHK(_sensorData->temperature_baro, __sensorData.temperature_baro, "Temperature_baro = %0.2f", _sensorData->temperature_baro);
    } /*-------------------------------- end of display_baro_data -------------------------------*/

}; /*----------------------------------- end of sensor.h class ----------------------------------*/
