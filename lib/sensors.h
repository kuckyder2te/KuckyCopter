#pragma once
/*  File name: sensors.h
 *	Project name: KuckyCopter 2
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
 * Höhe 55m von Strasse zum Dach + ca. 6m
 *
 * https://github.com/har-in-air/ESP8266_MPU9250_MS5611_VARIO?tab=readme-ov-file
 */

#include <Arduino.h>
#include <TaskManager.h>
#include <Wire.h>
#include <MPU9250.h>
#include <MS5611.h>
#include "def.h"

// #define LOCAL_DEBUG
#include "myLogger.h"

#include "..\lib\putty_out.h"

#define MIN_SENSOR_DELAY 1000
typedef struct
{
    int16_t pitch;
    int16_t roll;
    int16_t yaw;
    int16_t virtual_yaw;
    double compass;
    float pressure;
    float altitude;
    float temperature_baro;
    float seaLevel;
} sensorData_t;

#define HOME_ALTITUDE 61

const uint8_t EEPROM_SIZE = 1 + sizeof(float) * 3 * 4;

class Sensor : public Task::Base
{
    enum EEP_ADDR
    {
        EEP_CALIB_FLAG = 0x64,
        EEP_ACC_BIAS = 0x65,
        EEP_GYRO_BIAS = 0x71,
        EEP_MAG_BIAS = 0x7D,
        EEP_MAG_SCALE = 0x89
    };

private:
    MPU9250Setting setting;
    uint8_t _updateCounter;
    int16_t _lastYaw;
    PUTTY_out *_putty_out;
    HardwareSerial *_serial;

public:
    bool startCalibration;
    float acc_bias_x, acc_bias_y, acc_bias_z;
    float gyro_bias_x, gyro_bias_y, gyro_bias_z;
    float mag_bias_x, mag_bias_y, mag_bias_z;
    float mag_scale_x, mag_scale_Y, mag_scale_z;

protected:
    MPU9250 _mpu9250; // Speicherplatz reserviert
    MS5611 _ms5611;
    sensorData_t *_sensorData;
    sensorData_t __sensorData; // for better reading

public:
    Sensor(const String &name) : Task::Base(name)
    {
        LOGGER_VERBOSE("Enter....");
        LOGGER_VERBOSE("....leave");
    }

    Sensor *setSerial(HardwareSerial *serial)
    {
        _serial = serial;
        _putty_out = new PUTTY_out(*serial);
        return this;
    } /* -------------------- end of Config *setSerial ----------------------------*/

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
        Wire.setClock(400000); // For 3k3 pullup resistors
        // Wire.setWireTimeout(3000, true); //timeout value in uSec  https://github.com/jrowberg/i2cdevlib/issues/519#issuecomment-752023021
        Wire.begin();
        LOGGER_NOTICE("MPU9250 will be initialized");

        setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
        setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
        setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
        setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
        setting.gyro_fchoice = 0x03;
        setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
        setting.accel_fchoice = 0x01;
        setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

        _mpu9250.setMagneticDeclination(51);

        // uint8_t ERR = _mpu9250.get_i2c_error(); // temp_debug
        // if (ERR >= 7)
        // { // to avoid stickbreaker-i2c branch's error code
        //     Serial.print("I2C ERROR CODE : ");
        //     Serial.println(ERR);
        // }

        if (!_mpu9250.setup(0x68))
        {
            while (1)
            {
                LOGGER_FATAL("MPU connection failed.");
                delay(10);
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
        delay(100);
        LOGGER_VERBOSE("....leave");
    } /* ------------------ end of begin --------------------------------------------------------*/

    virtual void update() override
    {
        static uint32_t lastMillis1 = 0;
        uint8_t time = 0;

        LOGGER_VERBOSE("Enter....");
        if (!startCalibration)
        {
            LOGGER_NOTICE_CHK("IMU calibration is false");
            if (_mpu9250.update())
            {
                LOGGER_VERBOSE("_mpu9250.update");
                _sensorData->yaw = _mpu9250.getYaw();
                _sensorData->pitch = _mpu9250.getPitch();
                _sensorData->roll = _mpu9250.getRoll();
                int16_t deltaYaw = _sensorData->yaw - _lastYaw;
                if (abs(deltaYaw) >= 100)
                {
                    if (_lastYaw < 0)
                    {
                        deltaYaw += 180;
                    }
                    else if (_lastYaw > 0)
                    {
                        deltaYaw -= 180;
                    }
                    else
                    { // _lastYaw == 0 case
                        deltaYaw -= _sensorData->yaw + 180;
                    }
                }
                else
                {
                    _sensorData->virtual_yaw += deltaYaw;
                }
                _lastYaw = _sensorData->yaw;
                display_imu_data();

                LOGGER_VERBOSE("_mpu9250 leave");
            }
            if (++_updateCounter % 10 == 0) // Only executed every 10 runs.
            {
                LOGGER_VERBOSE("_ms5611.read");
                _ms5611.read(); // uses default OSR_ULTRA_LOW  (fastest)
                //    _sensorData->seaLevel = getSeaLevel(_ms5611.getPressure(), HOME_ALTITUDE);
                _sensorData->temperature_baro = _ms5611.getTemperature();
                _sensorData->pressure = _ms5611.getPressure();
                _sensorData->altitude = getAltitude(_ms5611.getPressure(), (getSeaLevel(_ms5611.getPressure(), HOME_ALTITUDE)));

                display_baro_data();

                LOGGER_VERBOSE("_ms5611 leave");
            }
        }
        else
        {
            LOGGER_NOTICE_CHK("IMU calibration is true");
            uint8_t row_add = 0;

            _putty_out->print(ROW_SELECT, COL_MENU, YELLOW, "Accel Gyro calibration will start in ca. 5 sec.");
            _putty_out->print(ROW_SELECT + (row_add += 1), COL_MENU, YELLOW, "Please leave the device still on the flat plane.");

            _mpu9250.verbose(true);

            // if (millis() - lastMillis1 > 5000)
            // {
            //     lastMillis1 = millis();
            //     break;
            // }

            //    delay(5000);
            _mpu9250.calibrateAccelGyro();

            _putty_out->print(ROW_SELECT + (row_add += 2), COL_MENU, YELLOW, "Mag calibration will start in 5sec.");
            _putty_out->print(ROW_SELECT + (row_add += 1), COL_MENU, YELLOW, "Please Wave device in a figure eight until done.");
            delay(5000);
            _mpu9250.calibrateMag();

            Serial1.println("Print calibration");
            print_calibration();
            _mpu9250.verbose(false);

            // save to eeprom
            Serial1.println("Save calibration");
            saveCalibration();

            // load from eeprom
            Serial1.println("Load calibration");
            loadCalibration();

            start_Calibration(false);
            row_add = 0;
        }
        LOGGER_VERBOSE("....leave");
    } /* ------------------ end of update -------------------------------------------------------*/

    MPU9250 *getIMU()
    {
        return &_mpu9250;
    } /* ------------------ end of getIMU -------------------------------------------------------*/

    float getAltitude(double press, double seaLevel)
    {
        // return (1.0f - pow(press/101325.0f, 0.190295f)) * 4433000.0f;
        return ((pow((seaLevel / press), 1 / 5.257) - 1.0) * (_ms5611.getTemperature() + 273.15)) / 0.0065;
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

    void start_Calibration(bool x)
    {
        startCalibration = x;
    } /*-------------------------------- start_Calibration -----------------------------------------*/

    void writeByte(int address, byte value)
    {
        EEPROM.put(address, value);
    } /*-------------------------------- end of writeByte ---------------------------------- -----*/

    void writeFloat(int address, float value)
    {
        EEPROM.put(address, value);
    } /*-------------------------------- end of writeFloat --------------------------------------*/

    byte readByte(int address)
    {
        byte valueIn = 0;
        EEPROM.get(address, valueIn);
        Serial1.print("Value = ");
        Serial1.println(valueIn);
        return valueIn;
    } /*-------------------------------- end of readByte ---------------------------------------*/

    float readFloat(int address)
    {
        float valueIn = 0;
        EEPROM.get(address, valueIn);
        Serial1.print("Value = ");
        Serial1.println(valueIn);
        return valueIn;
    } /*-------------------------------- end of readFloat ---------------------------------------*/

    void clearCalibration()
    {
        writeByte(EEP_CALIB_FLAG, 0);
    } /*-------------------------------- end of clearCalibration --------------------------------*/

    bool isCalibrated()
    {
        return (readByte(EEP_CALIB_FLAG) == 0x01);
    } /*-------------------------------- end of isCalibrated ------------------------------------*/

    void saveCalibration()
    {
        Serial1.println("Write calibrated parameters to EEPROM");
        writeByte(EEP_CALIB_FLAG, 1);
        writeFloat(EEP_ACC_BIAS + 0, _mpu9250.getAccBias(0));
        writeFloat(EEP_ACC_BIAS + 4, _mpu9250.getAccBias(1));
        writeFloat(EEP_ACC_BIAS + 8, _mpu9250.getAccBias(2));
        writeFloat(EEP_GYRO_BIAS + 0, _mpu9250.getGyroBias(0));
        writeFloat(EEP_GYRO_BIAS + 4, _mpu9250.getGyroBias(1));
        writeFloat(EEP_GYRO_BIAS + 8, _mpu9250.getGyroBias(2));
        writeFloat(EEP_MAG_BIAS + 0, _mpu9250.getMagBias(0));
        writeFloat(EEP_MAG_BIAS + 4, _mpu9250.getMagBias(1));
        writeFloat(EEP_MAG_BIAS + 8, _mpu9250.getMagBias(2));
        writeFloat(EEP_MAG_SCALE + 0, _mpu9250.getMagScale(0));
        writeFloat(EEP_MAG_SCALE + 4, _mpu9250.getMagScale(1));
        writeFloat(EEP_MAG_SCALE + 8, _mpu9250.getMagScale(2));

#if defined(ESP_PLATFORM) || defined(ESP8266)
        EEPROM.commit();
#endif
    } /*-------------------------------- end of saveCalibration ---------------------------------*/

    void loadCalibration()
    {
        Serial1.println("Load calibrated parameters from EEPROM");
        if (isCalibrated())
        {
            Serial1.println("calibrated? : YES");
            Serial1.println("load calibrated values");
            _mpu9250.setAccBias(
                readFloat(EEP_ACC_BIAS + 0),
                readFloat(EEP_ACC_BIAS + 4),
                readFloat(EEP_ACC_BIAS + 8));
            _mpu9250.setGyroBias(
                readFloat(EEP_GYRO_BIAS + 0),
                readFloat(EEP_GYRO_BIAS + 4),
                readFloat(EEP_GYRO_BIAS + 8));
            _mpu9250.setMagBias(
                readFloat(EEP_MAG_BIAS + 0),
                readFloat(EEP_MAG_BIAS + 4),
                readFloat(EEP_MAG_BIAS + 8));
            _mpu9250.setMagScale(
                readFloat(EEP_MAG_SCALE + 0),
                readFloat(EEP_MAG_SCALE + 4),
                readFloat(EEP_MAG_SCALE + 8));
        }
        else
        {
            Serial1.println("calibrated? : NO");
            Serial1.println("load default values");
            _mpu9250.setAccBias(0., 0., 0.);
            _mpu9250.setGyroBias(0., 0., 0.);
            _mpu9250.setMagBias(0., 0., 0.);
            _mpu9250.setMagScale(1., 1., 1.);
        }
    } /*-------------------------------- end of loadCalibration ---------------------------------*/

    void print_calibration()
    {
        uint8_t row_add = 0;
        _putty_out->print(ROW_SELECT + (row_add += 6), COL_MENU + 15, RED, "< calibration parameters >");
        _putty_out->print(ROW_SELECT + (row_add += 2), COL_MENU + 4, YELLOW, "accel bias   gyro bias   mag bias  mag scale");
        _putty_out->print(ROW_SELECT + (row_add += 1), COL_MENU + 4, YELLOW, "   [g]        [deg/s]      [mG]             ");
        _putty_out->print(ROW_SELECT + (row_add += 2), COL_MENU + 2, YELLOW, "X");
        _putty_out->print(ROW_SELECT + (row_add += 2), COL_MENU + 2, YELLOW, "Y");
        _putty_out->print(ROW_SELECT + (row_add += 2), COL_MENU + 2, YELLOW, "Z");

        _putty_out->print(ROW_SELECT + (row_add -= 4), COL_MENU + 7, CYAN, 3, (_mpu9250.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY));
        _putty_out->print(ROW_SELECT + row_add, COL_MENU + 19, CYAN, 3, (_mpu9250.getGyroBiasX() * 1000.f / (float)MPU9250::CALIB_GYRO_SENSITIVITY));
        _putty_out->print(ROW_SELECT + row_add, COL_MENU + 31, CYAN, 3, (_mpu9250.getMagBiasX()));
        _putty_out->print(ROW_SELECT + row_add, COL_MENU + 41, CYAN, 3, (_mpu9250.getMagScaleX()));

        _putty_out->print(ROW_SELECT + (row_add += 2), COL_MENU + 7, CYAN, 3, (_mpu9250.getAccBiasY() * 1000.f / MPU9250::CALIB_ACCEL_SENSITIVITY));
        _putty_out->print(ROW_SELECT + row_add, COL_MENU + 19, CYAN, 3, (_mpu9250.getGyroBiasY() * 1000.f / MPU9250::CALIB_GYRO_SENSITIVITY));
        _putty_out->print(ROW_SELECT + row_add, COL_MENU + 31, CYAN, 3, (_mpu9250.getMagBiasY()));
        _putty_out->print(ROW_SELECT + row_add, COL_MENU + 41, CYAN, 3, (_mpu9250.getMagScaleY()));

        _putty_out->print(ROW_SELECT + (row_add += 2), COL_MENU + 7, CYAN, 3, (_mpu9250.getAccBiasZ() * 1000.f / MPU9250::CALIB_ACCEL_SENSITIVITY));
        _putty_out->print(ROW_SELECT + row_add, COL_MENU + 19, CYAN, 3, (_mpu9250.getGyroBiasZ() * 1000.f / MPU9250::CALIB_GYRO_SENSITIVITY));
        _putty_out->print(ROW_SELECT + row_add, COL_MENU + 31, CYAN, 3, (_mpu9250.getMagBiasZ()));
        _putty_out->print(ROW_SELECT + row_add, COL_MENU + 41, CYAN, 3, (_mpu9250.getMagScaleZ()));
        row_add = 0;
    } /*-------------------------------- end of printCalibration --------------------------------*/

    // void printBytes()
    // {
    //     for (size_t i = 0; i < EEPROM_SIZE; ++i)
    //         Serial1.println(readByte(i), HEX);
    // } /*-------------------------------- end of printBytes --------------------------------------*/

    // void setupEEPROM()
    // {
    //     Serial1.println("EEPROM start");

    //     if (!isCalibrated())
    //     {
    //         Serial1.println("Need Calibration!!");
    //     }
    //     Serial1.println("EEPROM calibration value is : ");
    //     print_calibrate_values();
    //     Serial1.println("Loaded calibration value is : ");
    //     loadCalibration();
    // } /*-------------------------------- end of setupEEPROM -------------------------------------*/
}; /*----------------------------------- end of sensor.h class -------------------------------*/
