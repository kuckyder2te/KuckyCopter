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
}gyroData_t;

class Gyro : public Task::Base {
    bool b;         // Klassenvariable

private:
    MPU9250Setting setting;
 
protected:
	MPU9250  *_mpu9250;         // Pointer auf die platz freigemacht
    gyroData_t *_gyroData;      

public:
    Gyro(const String& name) : Task::Base(name) ,b(false){
        LOGGER_VERBOSE("Enter....");
        pinMode(GYRO_LED, OUTPUT);
        digitalWrite(GYRO_LED, LOW);
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
        _mpu9250 = new MPU9250();  // Adresse in Variable speichern
        LOGGER_NOTICE("Start init MPU");

        if (!_mpu9250->setup(0x68, setting)) {  // change to your own address
            while (1) {
            LOGGER_FATAL("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
            }
        }       
        setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
        setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
        setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
        setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
        setting.gyro_fchoice = 0x03;
        setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
        setting.accel_fchoice = 0x01;
        setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ; 
		LOGGER_NOTICE("End init MPU");  
        LOGGER_VERBOSE("....leave");   
     }

    virtual void enter() override {
        LOGGER_VERBOSE("Enter....");
        if(_mpu9250->update()){  
             _gyroData->roll = _mpu9250->getRoll();
             _gyroData->pitch = _mpu9250->getPitch();
             _gyroData->yaw = _mpu9250->getYaw();            
         }
        LOGGER_VERBOSE("....leave"); 
    }

    virtual void update() override {
        LOGGER_VERBOSE("Enter...."); 
        //Check thresholds
        //Warnings if too stall
        LOGGER_NOTICE_FMT("roll =%.2f,pitch =,%.2f,yaw =%.2f", _gyroData->roll, _gyroData->pitch, _gyroData->yaw);
        LOGGER_VERBOSE("....leave"); 
   }
//    String getMonitor(){     
//        char buf[20];
//        //sprintf(buf,"/d,/d,/d",_gyroData.roll,_gyroData.pitch,_gyroData.yaw);
//        return buf;
//    }
   
    //  virtual void idle() override {
    //  }  
};
