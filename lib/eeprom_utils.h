#include <EEPROM.h>
#include "MPU9250.h"

const uint8_t EEPROM_SIZE = 1 + sizeof(float) * 3 * 4;
extern MPU9250 mpu;

enum EEP_ADDR {
    // EEP_CALIB_FLAG = 0x00 + 100,    // 0 + 100
    // EEP_ACC_BIAS = 0x01 + 100,      // 1 + 100
    // EEP_GYRO_BIAS = 0x0D + 100,     // 13 + 100
    // EEP_MAG_BIAS = 0x19 + 100,      // 25 + 100
    // EEP_MAG_SCALE = 0x25 + 100      // 37 + 100

    EEP_CALIB_FLAG = 0x64,    
    EEP_ACC_BIAS = 0x65,      
    EEP_GYRO_BIAS = 0x71, 
    EEP_MAG_BIAS = 0x7D, 
    EEP_MAG_SCALE = 0x89 
};

void writeByte(int address, byte value) {
    EEPROM.put(address, value);
}

void writeFloat(int address, float value) {
    EEPROM.put(address, value);
}

byte readByte(int address) {
    byte valueIn = 0;
    EEPROM.get(address, valueIn);
    return valueIn;
}

float readFloat(int address) {
    float valueIn = 0;
    EEPROM.get(address, valueIn);
    return valueIn;
}

void clearCalibration() {
    writeByte(EEP_CALIB_FLAG, 0);
}

bool isCalibrated() {
    return (readByte(EEP_CALIB_FLAG) == 0x01);
}

void saveCalibration() {
    Serial1.println("Write calibrated parameters to EEPROM");
    writeByte(EEP_CALIB_FLAG, 1);
    writeFloat(EEP_ACC_BIAS + 0, mpu.getAccBias(0));
    writeFloat(EEP_ACC_BIAS + 4, mpu.getAccBias(1));
    writeFloat(EEP_ACC_BIAS + 8, mpu.getAccBias(2));
    writeFloat(EEP_GYRO_BIAS + 0, mpu.getGyroBias(0));
    writeFloat(EEP_GYRO_BIAS + 4, mpu.getGyroBias(1));
    writeFloat(EEP_GYRO_BIAS + 8, mpu.getGyroBias(2));
    writeFloat(EEP_MAG_BIAS + 0, mpu.getMagBias(0));
    writeFloat(EEP_MAG_BIAS + 4, mpu.getMagBias(1));
    writeFloat(EEP_MAG_BIAS + 8, mpu.getMagBias(2));
    writeFloat(EEP_MAG_SCALE + 0, mpu.getMagScale(0));
    writeFloat(EEP_MAG_SCALE + 4, mpu.getMagScale(1));
    writeFloat(EEP_MAG_SCALE + 8, mpu.getMagScale(2));
#if defined(ESP_PLATFORM) || defined(ESP8266)
    EEPROM.commit();
#endif
}

void loadCalibration() {
    Serial1.println("Load calibrated parameters from EEPROM");
    if (isCalibrated()) {
        Serial1.println("calibrated? : YES");
        Serial1.println("load calibrated values");
        mpu.setAccBias(
            readFloat(EEP_ACC_BIAS + 0),
            readFloat(EEP_ACC_BIAS + 4),
            readFloat(EEP_ACC_BIAS + 8));
        mpu.setGyroBias(
            readFloat(EEP_GYRO_BIAS + 0),
            readFloat(EEP_GYRO_BIAS + 4),
            readFloat(EEP_GYRO_BIAS + 8));
        mpu.setMagBias(
            readFloat(EEP_MAG_BIAS + 0),
            readFloat(EEP_MAG_BIAS + 4),
            readFloat(EEP_MAG_BIAS + 8));
        mpu.setMagScale(
            readFloat(EEP_MAG_SCALE + 0),
            readFloat(EEP_MAG_SCALE + 4),
            readFloat(EEP_MAG_SCALE + 8));
    } else {
        Serial1.println("calibrated? : NO");
        Serial1.println("load default values");
        mpu.setAccBias(0., 0., 0.);
        mpu.setGyroBias(0., 0., 0.);
        mpu.setMagBias(0., 0., 0.);
        mpu.setMagScale(1., 1., 1.);
    }
}

void printCalibration() {
    Serial1.println("< calibration parameters >");
    Serial1.print("calibrated? : ");
    Serial1.println(readByte(EEP_CALIB_FLAG) ? "YES" : "NO");
    Serial1.print("acc bias x  : ");
    Serial1.println(readFloat(EEP_ACC_BIAS + 0) * 1000.f / MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial1.print("acc bias y  : ");
    Serial1.println(readFloat(EEP_ACC_BIAS + 4) * 1000.f / MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial1.print("acc bias z  : ");
    Serial1.println(readFloat(EEP_ACC_BIAS + 8) * 1000.f / MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial1.print("gyro bias x : ");
    Serial1.println(readFloat(EEP_GYRO_BIAS + 0) / MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial1.print("gyro bias y : ");
    Serial1.println(readFloat(EEP_GYRO_BIAS + 4) / MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial1.print("gyro bias z : ");
    Serial1.println(readFloat(EEP_GYRO_BIAS + 8) / MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial1.print("mag bias x  : ");
    Serial1.println(readFloat(EEP_MAG_BIAS + 0));
    Serial1.print("mag bias y  : ");
    Serial1.println(readFloat(EEP_MAG_BIAS + 4));
    Serial1.print("mag bias z  : ");
    Serial1.println(readFloat(EEP_MAG_BIAS + 8));
    Serial1.print("mag scale x : ");
    Serial1.println(readFloat(EEP_MAG_SCALE + 0));
    Serial1.print("mag scale y : ");
    Serial1.println(readFloat(EEP_MAG_SCALE + 4));
    Serial1.print("mag scale z : ");
    Serial1.println(readFloat(EEP_MAG_SCALE + 8));
}

void printBytes() {
    for (size_t i = 0; i < EEPROM_SIZE; ++i)
        Serial1.println(readByte(i), HEX);
}

void setupEEPROM() {
    Serial1.println("EEPROM start");

    if (!isCalibrated()) {
        Serial1.println("Need Calibration!!");
    }
    Serial1.println("EEPROM calibration value is : ");
    printCalibration();
    Serial1.println("Loaded calibration value is : ");
    loadCalibration();
}
