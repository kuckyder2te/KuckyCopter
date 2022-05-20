#pragma once

#include <Arduino.h>
#include <TaskManager.h>
#include <MPU9250.h>
#include "..\lib\def.h"
#include "myLogger.h"

typedef struct {
	//int16_t  imuOutput[3];		///< 0->primary, 1->secondary, 2->YAW
    float  pitch;
    float  roll;
    float  yaw;
	double   compass;			///< Real compass direction
	int16_t  horz_Position;
	float	 angularVelocity;	///< Angular velocity
	int16_t  compassDiff;		///< Difference between two compass values
	short 	 vectorOld[3];
	//bool	 isMove;
    float getAccelX_mss;
}gyroData_t;

class Gyro : public Task::Base {
    bool b;         // Klassenvariable
    int status;

private:
    // MPU9250Setting setting;
 
protected:
	MPU9250  *_mpu9250;         // Pointer auf die platz freigemacht
    gyroData_t *_gyroData;      

public:
    Gyro(const String& name) : Task::Base(name) ,b(false){
        LOGGER_VERBOSE("Enter....");
        // pinMode(GYRO_LED, OUTPUT);
        // digitalWrite(GYRO_LED, LOW);
        LOGGER_VERBOSE("....leave");         
    }

    virtual ~Gyro() {}

    Gyro* setModel(gyroData_t* _model){    // Rückgabe wert ist das eigene Objekt (this)
        LOGGER_VERBOSE("Enter....");
        _gyroData = _model;
        LOGGER_VERBOSE("....leave");
        return this;
    }

    virtual void begin() override {
        LOGGER_VERBOSE("Enter....");
        Wire.begin();
        LOGGER_NOTICE("Wire initialized");
        _mpu9250 = new MPU9250(Wire, 0x68);  // Adresse in Variable speichern
        LOGGER_NOTICE("Start init MPU");

            // start communication with IMU 
            status = _mpu9250->begin();
            if (status < 0) {
                LOGGER_FATAL("IMU initialization unsuccessful");
                LOGGER_FATAL("Check IMU wiring or try cycling power");
                LOGGER_NOTICE_FMT("Status: %i", status);
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
        LOGGER_VERBOSE("....leave");   
     }

    virtual void enter() override {
        LOGGER_VERBOSE("Enter....");
         _mpu9250->readSensor();

  // display the data
        _gyroData->getAccelX_mss = _mpu9250->getAccelX_mss();
    
         //   Serial.print(IMU.getAccelX_mss(),6);
        LOGGER_VERBOSE("....leave"); 
    }

    virtual void update() override {
        LOGGER_VERBOSE("Enter...."); 
        //Check thresholds
        //Warnings if too stall
        // LOGGER_NOTICE_FMT("roll =%.2f,pitch =,%.2f,yaw =%.2f", _gyroData->roll, _gyroData->pitch, _gyroData->yaw);
        // LOGGER_VERBOSE("....leave"); 
   }
//    String getMonitor(){     
//        char buf[20];
//        //sprintf(buf,"/d,/d,/d",_gyroData.roll,_gyroData.pitch,_gyroData.yaw);
//        return buf;
//    }
   
    //  virtual void idle() override {
    //  }  
};  /*--------------------------------- end of gyro class ----------------------------*/
