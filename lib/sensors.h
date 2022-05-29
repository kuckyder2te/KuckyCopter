#pragma once

/*  File name: pidController.h
 *	Project name: KuCo_Phantom 1
 *  Date: 2022-005-28
 *  Author: Wilhelm Kuckelsberg
 * 
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
 
    /* Values for the "MadgwickQuaternionUpdate" Filter */
    /*
    float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
    // global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
    float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
    float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
    // There is a tradeoff in the beta parameter between accuracy and response speed.
    // In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
    // However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
    // Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
    // By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
    // I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
    // the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
    // In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
    float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
    float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
    float pitch, yaw, roll;
    */

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

     //   MadgwickQuaternionUpdate(_aX, _aY, _aZ, _gX*PI/180.0f, _gY*PI/180.0f, _gZ*PI/180.0f,  _mY,  _mX, _mZ);

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

    // void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
    // {
    // LOGGER_VERBOSE("Enter....");     
    //     float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
    //     float norm;
    //     float hx, hy, _2bx, _2bz;
    //     float s1, s2, s3, s4;
    //     float qDot1, qDot2, qDot3, qDot4;

    //     // Auxiliary variables to avoid repeated arithmetic
    //     float _2q1mx;
    //     float _2q1my;
    //     float _2q1mz;
    //     float _2q2mx;
    //     float _4bx;
    //     float _4bz;
    //     float _2q1 = 2.0f * q1;
    //     float _2q2 = 2.0f * q2;
    //     float _2q3 = 2.0f * q3;
    //     float _2q4 = 2.0f * q4;
    //     float _2q1q3 = 2.0f * q1 * q3;
    //     float _2q3q4 = 2.0f * q3 * q4;
    //     float q1q1 = q1 * q1;
    //     float q1q2 = q1 * q2;
    //     float q1q3 = q1 * q3;
    //     float q1q4 = q1 * q4;
    //     float q2q2 = q2 * q2;
    //     float q2q3 = q2 * q3;
    //     float q2q4 = q2 * q4;
    //     float q3q3 = q3 * q3;
    //     float q3q4 = q3 * q4;
    //     float q4q4 = q4 * q4;

    //     // Normalise accelerometer measurement
    //     norm = sqrtf(ax * ax + ay * ay + az * az);
    //     if (norm == 0.0f) return; // handle NaN
    //     norm = 1.0f/norm;
    //     ax *= norm;
    //     ay *= norm;
    //     az *= norm;

    //     // Normalise magnetometer measurement
    //     norm = sqrtf(mx * mx + my * my + mz * mz);
    //     if (norm == 0.0f) return; // handle NaN
    //     norm = 1.0f/norm;
    //     mx *= norm;
    //     my *= norm;
    //     mz *= norm;

    //     // Reference direction of Earth's magnetic field
    //     _2q1mx = 2.0f * q1 * mx;
    //     _2q1my = 2.0f * q1 * my;
    //     _2q1mz = 2.0f * q1 * mz;
    //     _2q2mx = 2.0f * q2 * mx;
    //     hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    //     hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    //     _2bx = sqrtf(hx * hx + hy * hy);
    //     _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    //     _4bx = 2.0f * _2bx;
    //     _4bz = 2.0f * _2bz;

    //     // Gradient decent algorithm corrective step
    //     s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    //     s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    //     s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    //     s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    //     norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
    //     norm = 1.0f/norm;
    //     s1 *= norm;
    //     s2 *= norm;
    //     s3 *= norm;
    //     s4 *= norm;

    //     // Compute rate of change of quaternion
    //     qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
    //     qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
    //     qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
    //     qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

    //     // Integrate to yield quaternion
    //     q1 += qDot1 * deltat;
    //     q2 += qDot2 * deltat;
    //     q3 += qDot3 * deltat;
    //     q4 += qDot4 * deltat;
    //     norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    //     norm = 1.0f/norm;
    //     q[0] = q1 * norm;
    //     q[1] = q2 * norm;
    //     q[2] = q3 * norm;
    //     q[3] = q4 * norm;


    //     // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
    //     // In this coordinate system, the positive z-axis is down toward Earth. 
    //     // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
    //     // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
    //     // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
    //     // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
    //     // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
    //     // applied in the correct order which for this configuration is yaw, pitch, and then roll.
    //     // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.

    //     yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]); 
    //     pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    //     roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    //     pitch *= 180.0f / PI;
    //     yaw   -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    //     roll  *= 180.0f / PI;

    //     LOGGER_NOTICE_FMT("Yaw %.2f", yaw);
    //     LOGGER_NOTICE_FMT("Pitch %.2f", pitch);
    //     LOGGER_NOTICE_FMT("Roll %.2f", roll);

    // LOGGER_VERBOSE("....leave");
    
    // } /* ---------------------------- end of MadgwickQuaternionUpdate filter ----------*/
    
};  /*----------------------------------- end of sensor.h class -----------------------*/
