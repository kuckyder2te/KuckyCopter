#pragma once


#include <TaskManager.h>
#include <FastPID.h>

#include "..\lib\myLogger.h"
#include "..\lib\def.h"

typedef struct {
	float pidCoefficient[3];
	float executionFrequency;
	int   output_bits;
	bool  output_signed;
} pidData_t;

class PidController : public Task::Base {

protected:
    FastPID *_fastPID;
	pidData_t*     _pidData;


public:
    PidController(const String& name) : Task::Base(name) {
       LOGGER_VERBOSE("Enter....");
       LOGGER_VERBOSE("....leave");
    }

				/// MyPid has a PidData

    virtual ~PidController() {}

        PidController* setModel(pidData_t* _model){    // Rückgabe wert ist das eigene Objekt (this)
        LOGGER_VERBOSE("Enter....");
        _pidData = _model;
        LOGGER_VERBOSE("....leave");
        return this;
    }
  
    virtual void begin() override {
    LOGGER_VERBOSE("Enter....");
        LOGGER_NOTICE("FastPID initialized");
        _fastPID = new FastPID();  // Adresse in Variable speichern
        LOGGER_NOTICE("End init FastPID");
    LOGGER_VERBOSE("....leave");   
    }

   
    // virtual void enter() override {
    // }

    virtual void update() override {

    }

    
    // virtual void exit() override {
    // }

   
    // virtual void idle() override {
    // }

   
    // virtual void reset() override {
    // }
};  /*---------------------- end of pidController.h -------------*/