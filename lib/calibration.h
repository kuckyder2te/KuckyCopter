#pragma once
/*  File name : calibration.h
	Autor: Wilhelm Kuckelsberg
	Date: 2021.xx.xx
	Description: The PID values can be configured via the terminal.
*/

#include <Arduino.h>
#include <TaskManager.h>
#include "..\lib\def.h"
#include "..\lib\gui.h"
//#include "..\lib\pidController.h" //new
//#include "..\lib\model.h" //new

#include "myLogger.h"
typedef enum{               // Enumarations for menu
	pri_P = 11,
	pri_I = 21,
	pri_D = 31,
	sec_P = 12,
	sec_I = 22,
	sec_D = 32,
	yaw_P = 13,
	yaw_I = 23,
	yaw_D = 33,
	pri_ef = 41,
	sec_ef = 42,
	yaw_ef = 43
}pidTyp_t;

#define	axis_pri  1
#define	axis_sec  2
#define	axis_yaw  3

#define	kP  10
#define	kI  20
#define	kD  30
#define	eF  40

// typedef enum{
// 	P = 0,
// 	I,
// 	D
// }pid;

// typedef struct {
// 	float pidCoefficient[3];
// 	float executionFrequency;
// 	int   output_bits;
// 	bool  output_signed;
// } pidData_t;

typedef enum{
	Primary,
	Secondary,
	YawAxis
}axis_t;

typedef enum{
	P = 0,
	I,
	D
}pid;

//extern PIDcontroller pid_pri;
//extern modules::MyPid myPID_sec;
//extern modules::MyPid myPID_yaw;


class Calibration : public Task::Base {
    bool b;         // Klassenvariable
    uint8_t _itemAxis;
    uint8_t _itemCoefficient;
	uint8_t _pidType;
    float   _newFactor = 0.1;	///< Multiplication factor for the PID coefficients, default setting.
	double  _factor;
	float _X_kP_value;
	float _X_kI_value;
	float _X_kD_value;
	float _Y_kP_value;
	float _Y_kI_value;
	float _Y_kD_value;
	float _Z_kP_value;
	float _Z_kI_value;
	float _Z_kD_value;

protected:
	Gui *_gui;
//	pidData_t *_pidData;
	//model_t	*_model;

public:
    Calibration(const String& name) : Task::Base(name){
		LOGGER_VERBOSE("Enter....");
        // pinMode(GYRO_LED, OUTPUT);
        // digitalWrite(GYRO_LED, LOW);
        LOGGER_VERBOSE("....leave");    
    }

    // virtual ~Calibration() {}

	//     Calibration* setModel(pidData_t* _model){    // Rückgabe wert ist das eigene Objekt (this)
    //     LOGGER_VERBOSE("Enter....");
    //     _pidData = _model;
    //     LOGGER_VERBOSE("....leave");
    //     return this;
    // }


    virtual void begin() override {
     	LOGGER_VERBOSE("Enter...");
			// _X_kP_value = 0;
			// LOGGER_WARNING_FMT(" set _X_kP_value = %d", _X_kP_value);
		LOGGER_VERBOSE("...Leave");  
    }

    // optional (you can remove this method)
    // virtual void enter() override {
    // }

    virtual void update() override {
		LOGGER_VERBOSE("Enter....");
    
        if(Serial2.available()>0){

            char key = Serial2.read();
			switch(key){
				case 'x':	
					LOGGER_WARNING_FMT(" primaty axis is select = %d", axis_pri);												///< Choose the axes
					setItemAxis(axis_pri);
					break;
				case 'y':
					LOGGER_WARNING_FMT(" secundary axis is select = %d", axis_sec);	
					setItemAxis(axis_sec);
					break;
				case 'z':
					LOGGER_WARNING_FMT(" YAW axis is select = %d", axis_yaw);	
					setItemAxis(axis_yaw);
					break;

				case 'p':													///< Choose the PID Coefficient				
					LOGGER_WARNING_FMT(" kP is select = %d", kP);
					setItemCoefficient(kP);
					break;
				case 'i':
					LOGGER_WARNING_FMT(" kI is select =%d", kI);
					setItemCoefficient(kI);					
					break;
				case 'd':
					LOGGER_WARNING_FMT(" kD is select =%d", kD);
					setItemCoefficient(kD);
					break;
				case 'e':
					LOGGER_WARNING_FMT(" eF is select = %d", eF);
					setItemCoefficient(eF);
					break;

				case '+':
					LOGGER_WARNING("Up");
					coefficient_Up();			///< Coefficient increment
					break;
				case '-':
					LOGGER_WARNING("Down");
					coefficient_Down();			///< Coefficient decrement
					break;

				case '0':						///< Choose the decimal places  0 to 0,001
					setDecimalPlaces(0);
					break;
				case '1':
					setDecimalPlaces(1);
					break;
				case '2':
					setDecimalPlaces(2);
					break;
				case '3':
					setDecimalPlaces(3);
					break;

				case 's':							///< Saved all coefficients into the EEPROM
	//				myPID_pri.updateEEPROM();
	//				myPID_sec.updateEEPROM();
	//				myPID_yaw.updateEEPROM();
					// displayPIDcoefficients();
					break;
				case 'r':							///< Reads all coefficients from the EEPROM
	//				myPID_pri.readEEPROM();
	//				myPID_sec.readEEPROM();
	//				myPID_yaw.readEEPROM();
			 	break;
				case 'a':							///< Set all PID parameters to 0
	//  			myPID_pri.setP(PID_P_MIN);
	// 				myPID_pri.setI(0);
	// 				myPID_pri.setD(0);
	// 				myPID_pri.setExecutionFrequency(50);
	// 				myPID_sec.setP(PID_P_MIN);
	// 				myPID_sec.setI(0);
	// 				myPID_sec.setD(0);
	// 				myPID_sec.setExecutionFrequency(50);
	// 				myPID_yaw.setP(PID_P_MIN);
	// 				myPID_yaw.setI(0);
	// 				myPID_yaw.setD(0);
	// 				myPID_yaw.setExecutionFrequency(50);
	// 
					displayPIDcoefficients();
					break;

				case 'g':							///< get factory default
	//  			myPID_pri.setP(0.2);
	// 				myPID_pri.setI(0.041);
	// 				myPID_pri.setD(0.1);
	// 				myPID_pri.setExecutionFrequency(50);
	// 				myPID_sec.setP(0.2);
	// 				myPID_sec.setI(0.041);
	// 				myPID_sec.setD(0.1);
	// 				myPID_sec.setExecutionFrequency(50);
	// 				myPID_yaw.setP(0.2);
	// 				myPID_yaw.setI(0.01);
	// 				myPID_yaw.setD(0);
	// 				myPID_yaw.setExecutionFrequency(50);
	// 
					displayPIDcoefficients();
					break;

				case 'c':	///< Copies the primary values to the secondary axis
	// 				myPID_sec.setP(myPID_pri.getP());
	// 				myPID_sec.setI(myPID_pri.getI());
	// 				myPID_sec.setD(myPID_pri.getD());				
					break;

				case 'm':
					LOGGER_WARNING("m was pressed");	
					display_Menu();
					displayPIDcoefficients();
					break;

				default:{
				}
			}	/* end of switch(key) */
        }	/* end of if Serial2 available */
		
		LOGGER_VERBOSE("....leave");    
    }// end of update ---------------

    void setItemAxis(uint8_t itemAxis) {
	 /* Selects the axis, according to the keyboard input.
	 * Key X = primary axis (1)
	 * Key Y = primary axis (2)
	 * Key Z = primary axis (3)	 */
		_itemAxis = itemAxis;
	//	LOGGER_WARNING_FMT("itemAxis = %d", _itemAxis);
	} 
	
	void setItemCoefficient(uint8_t itemCoefficient) {
	 /* Selects the coefficient, according to the keyboard input.
	 * Key P = coefficient P (10)
	 * Key I = coefficient I (20)
	 * Key D = coefficient D (30)
	 * Key E = ExecutingFrequency (40)	 */
		_itemCoefficient = itemCoefficient;
	//	LOGGER_WARNING_FMT("itemCoefficient = %d", _itemCoefficient);
	}	/*----------------------------- end of setItemCoefficient ---------------------*/ 

	void setDecimalPlaces(uint8_t dot){

		switch(dot){
			case 0:
				_newFactor = 1;
				break;
			case 1:
				_newFactor = 0.1;
				break;
			case 2:
				_newFactor = 0.01;
				break;
			case 3:		
				_newFactor = 0.001;
				break;
		}// end of switch

		LOGGER_WARNING_FMT("New Factor = %f", _newFactor);

	} 	/*----------------------------- end of setDecimalPlaces -----------------------*/

	 /* Set the "PID Type",
	  * e.g _itemAxis = 1 and _itemCoefficient = 20 ~ _pidType 21
	  * Will say, it select the parameter for secondary axis and coefficient 'i'
	  */
	uint8_t getPidType(bool up) {   /// const deleted

		_pidType = _itemAxis + _itemCoefficient;
	//	LOGGER_WARNING_FMT("PID Type = %d", _pidType);

		if(_pidType < 40){			///< P, I and D

			if(up)
				_factor = _newFactor;
			else
				_factor = _newFactor *-1;

		}
		if(_pidType >= 40){		///< Executingfrequency only
			if(up)
				_factor = 1;
			else
				_factor = -1;
		}   
		return _pidType;
    }
    /* Increment the coefficient according the PID Type. Shown "getPidType". */
	void coefficient_Up() {
		select(getPidType(true));
	}

	/* decrement the coefficient according the PID Type. Shown "getPidType". */
	void coefficient_Down() {
		select(getPidType(false));
    }

    /* Here you sets the real coefficient into the model, and print it into the GUI. +/- _factor
	   like this: Pri. P = 2.20 
	*/
	void select(uint8_t type) {
		LOGGER_WARNING_FMT("Select %d", type);

		switch(type){

		case pidTyp_t::pri_P:
		//	LOGGER_WARNING("Select");
			_X_kP_value += _factor;
			LOGGER_WARNING_FMT("_X_kP_value = %f", _X_kP_value);

		//	pid_pri.setP(((_model.pidData[axis_t::Primary].pidCoefficient[pid::P]) += _factor));
			break;
		case pidTyp_t::pri_I:
			_X_kI_value += _factor;
			LOGGER_WARNING_FMT("_X_kI_value = %f", _X_kI_value);
//			myPID_pri.setI(((_model.pidData[axis_t::Primary].pidCoefficient[pid::I]) += _factor));
			break;
		case pidTyp_t::pri_D:
			_X_kD_value += _factor;
			LOGGER_WARNING_FMT("_X_kD_value = %f", _X_kD_value);
//			myPID_pri.setD(((_model.pidData[axis_t::Primary].pidCoefficient[pid::D]) += _factor));
			break;

		case pidTyp_t::sec_P:
			_Y_kP_value += _factor;
			LOGGER_WARNING_FMT("_Y_kP_value = %f", _Y_kP_value);
//			myPID_sec.setP(((_model.pidData[axis_t::Secondary].pidCoefficient[pid::P]) += _factor));
			break;
		case pidTyp_t::sec_I:
			_Y_kI_value += _factor;
			LOGGER_WARNING_FMT("_Y_kI_value = %f", _Y_kI_value);
//			myPID_sec.setI(((_model.pidData[axis_t::Secondary].pidCoefficient[pid::I]) += _factor));
			break;
		case pidTyp_t::sec_D:
			_Y_kD_value += _factor;
			LOGGER_WARNING_FMT("_Y_kD_value = %f", _Y_kD_value);
//			myPID_sec.setD(((_model.pidData[axis_t::Secondary].pidCoefficient[pid::D]) += _factor));
			break;

		case pidTyp_t::yaw_P:
			_Z_kP_value += _factor;
			LOGGER_WARNING_FMT("_Z_kP_value = %f", _Z_kP_value);
//			myPID_yaw.setP(((_model.pidData[axis_t::YawAxis].pidCoefficient[pid::P]) += _factor));
			break;
		case pidTyp_t::yaw_I:
			_Z_kI_value += _factor;
			LOGGER_WARNING_FMT("_Z_kI_value = %f", _Z_kI_value);
//			myPID_yaw.setI(((_model.pidData[axis_t::YawAxis].pidCoefficient[pid::I]) += _factor));
			break;
		case pidTyp_t::yaw_D:
			_Z_kD_value += _factor;
			LOGGER_WARNING_FMT("_Z_kD_value = %f", _Z_kD_value);
//			myPID_yaw.setD(((_model.pidData[axis_t::YawAxis].pidCoefficient[pid::D]) += _factor));
			break;

		case pidTyp_t::pri_ef:
//			myPID_pri.setExecutionFrequency(((_model.pidData[axis_t::Primary].executionFrequency) += _factor));
			break;
		case pidTyp_t::sec_ef:
//			myPID_sec.setExecutionFrequency(((_model.pidData[axis_t::Secondary].executionFrequency) += _factor));
			break;
		case pidTyp_t::yaw_ef:
//			myPID_yaw.setExecutionFrequency(((_model.pidData[axis_t::YawAxis].executionFrequency) += _factor));
			break;
		} /* end of switch */
	}// end of select

	void display_Menu() {
	LOGGER_WARNING("Enter....");

		LOGGER_WARNING("-----------Menu for PID configuration (BT)-------------");
		LOGGER_WARNING("---(X) choose the primary");
		LOGGER_WARNING("---(Y)           secondary");
		LOGGER_WARNING("---(Z)            YAW axis");
		LOGGER_WARNING("--- P, I or D select the coefficient");
		LOGGER_WARNING("---(0),(1),(2)or(3)select the accurarcy");
		LOGGER_WARNING("---(E) choose the execution frequency");
		LOGGER_WARNING("---(+) increment according to the value");
		LOGGER_WARNING("---(-) decrement      ''");
		LOGGER_WARNING("---(S) saves all coefficient into the EEPROM");
		LOGGER_WARNING("---(R) reads all coefficients from the EEPROM");
		LOGGER_WARNING("---(C) Copies the primary values to the secondary axis");
		LOGGER_WARNING("---(A) all values are set to 0 in the EEPROM.");
		LOGGER_WARNING("---(G) get factory defaults");
		LOGGER_WARNING("---(M) display this menu");
		LOGGER_WARNING("--------------------------------------------------------");


	LOGGER_WARNING("....leeave");
} //-------------------------- end of display_Menu ------------------------------------------------
/*
 * Displayed all PID coefficients from the _model */

void displayPIDcoefficients() {

/* 	_gui->gray();
 	_gui->print(ROW_PID, COL_PID, "Current PID coefficients in the EEPROM");
 	_gui->yellow();
 
 	_gui->print(ROW_PID+2, COL_PID,    c_pri_p);_gui->print(ROW_PID+2, COL_PID+9,  2, _model.pidData[axis_t::Primary].pidCoefficient[pid::P]);
	_gui->print(ROW_PID+2, COL_PID+16, c_pri_i);_gui->print(ROW_PID+2, COL_PID+20, 3, _model.pidData[axis_t::Primary].pidCoefficient[pid::I]);
	_gui->print(ROW_PID+2, COL_PID+27, c_pri_d);_gui->print(ROW_PID+2, COL_PID+31, 3, _model.pidData[axis_t::Primary].pidCoefficient[pid::D]);
	_gui->print(ROW_PID+2, COL_PID+40, c_ef);   _gui->print(ROW_PID+2, COL_PID+52, 0, _model.pidData[axis_t::Primary].executionFrequency);

	_gui->print(ROW_PID+3, COL_PID,    c_sec_p);_gui->print(ROW_PID+3, COL_PID+9,  2, _model.pidData[axis_t::Secondary].pidCoefficient[pid::P]);
											  _gui->print(ROW_PID+3, COL_PID+20, 3, _model.pidData[axis_t::Secondary].pidCoefficient[pid::I]);
											  _gui->print(ROW_PID+3, COL_PID+31, 3, _model.pidData[axis_t::Secondary].pidCoefficient[pid::D]);
											  _gui->print(ROW_PID+3, COL_PID+52, 0, _model.pidData[axis_t::Secondary].executionFrequency);

	_gui->print(ROW_PID+4, COL_PID,    c_yaw_p);_gui->print(ROW_PID+4, COL_PID+9,  2, _model.pidData[axis_t::YawAxis].pidCoefficient[P]);
											  _gui->print(ROW_PID+4, COL_PID+20, 3, _model.pidData[axis_t::YawAxis].pidCoefficient[pid::I]);
											  _gui->print(ROW_PID+4, COL_PID+31, 3, _model.pidData[axis_t::YawAxis].pidCoefficient[pid::D]);
											  _gui->print(ROW_PID+4, COL_PID+52, 0, _model.pidData[axis_t::YawAxis].executionFrequency);
 */
} //-------------------------- end of displayPIDcoefficients --------------------------------------

};