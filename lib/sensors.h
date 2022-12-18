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

#define LOCAL_DEBUG
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
    float __aX, __aY, __aZ, __aSqrt, __gX, __gY, __gZ, __mDirection, __mX, __mY, __mZ;
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
    float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
    float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
    float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
    float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
    float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

protected:
    MPU9250_asukiaaa *_mpu9250; // Speicherplatz reserviert
    MS5611 *_ms5611;
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
            //LOGGER_NOTICE("Update accelerometer successful");
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
            //LOGGER_NOTICE("Update gyrometer successful");
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
            //LOGGER_NOTICE("Update magnetometer successful");
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

        // LOGGER_NOTICE_FMT_CHK(_aX, __aX, "accelX:  %.2f", _aX);
        // LOGGER_NOTICE_FMT_CHK(_aY, __aY,"accelY %.2f", _aY);
        // LOGGER_NOTICE_FMT_CHK(_aZ, __aZ,"accelZ %.2f", _aZ);
        // LOGGER_NOTICE_FMT_CHK(_aSqrt, __aSqrt,"accelSqrt %.2f", _aSqrt);

        // LOGGER_NOTICE_FMT_CHK(_gX, __gX,"gyroX:  %.2f", _gX);
        // LOGGER_NOTICE_FMT_CHK(_gY, __gY,"gyroY %.2f", _gY);
        // LOGGER_NOTICE_FMT_CHK(_gZ, __gZ,"gyroZ %.2f", _gZ);

        // LOGGER_NOTICE_FMT_CHK(_mX, __mX,"magX:  %.2f", _mX);
        // LOGGER_NOTICE_FMT_CHK(_mY, __mY,"magY %.2f", _mY);
        // LOGGER_NOTICE_FMT_CHK(_mZ, __mZ,"magZ %.2f", _mZ);
        // LOGGER_NOTICE_FMT_CHK(_mDirection,__mDirection,"horizontalDirection %.2f", _mDirection);

        // LOGGER_NOTICE_FMT_CHK(_sensorData->_seaLevel,__sensorData._seaLevel,"Sea level =  %.2fPa", _sensorData->_seaLevel);
        // LOGGER_NOTICE_FMT_CHK(_sensorData->_pressure,__sensorData._pressure,"Pressure =  %.2fPa", _sensorData->_pressure);
        // LOGGER_NOTICE_FMT_CHK(_sensorData->_temperature_baro, __sensorData._temperature_baro,"Temperature =  %.2f*C", _sensorData->_temperature_baro);   
        // LOGGER_NOTICE_FMT_CHK(_sensorData->_altitude, __sensorData._altitude,"Approx altitude =  %.4fm", _sensorData->_altitude);

        MadgwickQuaternionUpdate(_aX, _aY, _aZ, _gX, _gY, _gZ, _mX, _mY, _mZ);

        _sensorData->yaw = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
        _sensorData->pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
        _sensorData->roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
        _sensorData->pitch *= 180.0f / PI;
        _sensorData->yaw   *= 180.0f / PI; 
        _sensorData->yaw   -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
        _sensorData->roll  *= 180.0f / PI;

    /* Ausgabe von Rohdaten*/
    //Serial2.printf("/*%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f*/\r\n",_aX, _aY, _aZ, _aSqrt, _gX, _gY, _gZ, _mDirection, _mX, _mY, _mZ);

    /* Augabe von Arbeitsdaten*/
    Serial2.printf("/*%.2f,%.2f,%.2f,%.2f,%.2f*/\r\n",_sensorData->_pressure,_sensorData->_altitude,_sensorData->yaw,_sensorData->pitch,_sensorData->roll);

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


    private:

    void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
        {
            float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
            float norm;
            float hx, hy, _2bx, _2bz;
            float s1, s2, s3, s4;
            float qDot1, qDot2, qDot3, qDot4;

            // Auxiliary variables to avoid repeated arithmetic
            float _2q1mx;
            float _2q1my;
            float _2q1mz;
            float _2q2mx;
            float _4bx;
            float _4bz;
            float _2q1 = 2.0f * q1;
            float _2q2 = 2.0f * q2;
            float _2q3 = 2.0f * q3;
            float _2q4 = 2.0f * q4;
            float _2q1q3 = 2.0f * q1 * q3;
            float _2q3q4 = 2.0f * q3 * q4;
            float q1q1 = q1 * q1;
            float q1q2 = q1 * q2;
            float q1q3 = q1 * q3;
            float q1q4 = q1 * q4;
            float q2q2 = q2 * q2;
            float q2q3 = q2 * q3;
            float q2q4 = q2 * q4;
            float q3q3 = q3 * q3;
            float q3q4 = q3 * q4;
            float q4q4 = q4 * q4;

            // Normalise accelerometer measurement
            norm = sqrtf(ax * ax + ay * ay + az * az);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            ax *= norm;
            ay *= norm;
            az *= norm;

            // Normalise magnetometer measurement
            norm = sqrtf(mx * mx + my * my + mz * mz);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            mx *= norm;
            my *= norm;
            mz *= norm;

            // Reference direction of Earth's magnetic field
            _2q1mx = 2.0f * q1 * mx;
            _2q1my = 2.0f * q1 * my;
            _2q1mz = 2.0f * q1 * mz;
            _2q2mx = 2.0f * q2 * mx;
            hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
            hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
            _2bx = sqrtf(hx * hx + hy * hy);
            _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
            _4bx = 2.0f * _2bx;
            _4bz = 2.0f * _2bz;

            // Gradient decent algorithm corrective step
            s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
            norm = 1.0f/norm;
            s1 *= norm;
            s2 *= norm;
            s3 *= norm;
            s4 *= norm;

            // Compute rate of change of quaternion
            qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
            qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
            qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
            qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

            // Integrate to yield quaternion
            q1 += qDot1 * deltat;
            q2 += qDot2 * deltat;
            q3 += qDot3 * deltat;
            q4 += qDot4 * deltat;
            norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
            norm = 1.0f/norm;
            q[0] = q1 * norm;
            q[1] = q2 * norm;
            q[2] = q3 * norm;
            q[3] = q4 * norm;

        }

};    /*----------------------------------- end of sensor.h class ---------------------*/

//#undef _DEBUG_