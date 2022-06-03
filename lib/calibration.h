#pragma once
/*  File name : calibration.h
	Autor: Wilhelm Kuckelsberg
	Date: 2022-05-31 (2021.05.24)
	Description: The PID values can be configured online via the terminal.
*/

#include <Arduino.h>
#include <TaskManager.h>
#include <extEEPROM.h>
#include "..\lib\pidController.h"
#include "..\lib\def.h"
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

#define	kP  10
#define	kI  20
#define	kD  30
#define	eF  40

typedef enum{
	axis_pri = 1,
	axis_sec = 2,
	axis_yaw = 3
}itemAxis_t;

typedef enum{
	Primary = 0,
	Secondary,
	YawAxis
}axis_t;

typedef enum{
	P = 0,
	I,
	D
}pid;

class Calibration : public Task::Base {
    bool b;         // Klassenvariable
    uint8_t _itemAxis;
    uint8_t _itemCoefficient;
	uint8_t _pidType;
    float   _newFactor = 0.1;	///< Multiplication factor for the PID coefficients, default setting.
	double  _factor;

	float _x_kP=0;
	float _x_kI=0;
	float _x_kD=0;
	float _x_eF=0;
	float _y_kP=0;
	float _y_kI=0;
	float _y_kD=0;
	float _y_eF=0;
	float _z_kP=0;
	float _z_kI=0;
	float _z_kD=0;
	float _z_eF=0;

protected:
	PidController *_pid_pri;
	PidController *_pid_sec;
	PidController *_pid_yaw;

public:
    Calibration(const String& name) : Task::Base(name){
		LOGGER_VERBOSE("Enter....");
 
        LOGGER_VERBOSE("....leave");    
    }

	virtual ~Calibration() {}

    virtual void begin() override {
     	LOGGER_VERBOSE("Enter...");

		LOGGER_VERBOSE("...Leave");  
    }

    // virtual void enter() override {
    // }

    virtual void update() override {
		LOGGER_VERBOSE("Enter....");
    
        if(Serial2.available()>0){

            char key = Serial2.read();
			switch(key){
				case 'x':	
//					LOGGER_WARNING_FMT("primaty axis is select = %d", itemAxis_t::axis_pri);												///< Choose the axes
					setItemAxis(itemAxis_t::axis_pri);
					break;
				case 'y':
//					LOGGER_WARNING_FMT("secundary axis is select = %d", axis_sec);	
					setItemAxis(itemAxis_t::axis_sec);
					break;
				case 'z':
//					LOGGER_WARNING_FMT("YAW axis is select = %d", itemAxis_t::axis_yaw);	
					setItemAxis(itemAxis_t::axis_yaw);
					break;

				case 'p':													///< Choose the PID Coefficient				
//					LOGGER_WARNING_FMT("kP is select = %d", kP);
					setItemCoefficient(kP);
					break;
				case 'i':
//					LOGGER_WARNING_FMT("kI is select =%d", kI);
					setItemCoefficient(kI);					
					break;
				case 'd':
//					LOGGER_WARNING_FMT("kD is select =%d", kD);
					setItemCoefficient(kD);
					break;
				case 'e':
//					LOGGER_WARNING_FMT("eF is select = %d", eF);
					setItemCoefficient(eF);
					break;

				case '+':
					LOGGER_WARNING("+");
					coefficient_Up();			///< Coefficient increment
					break;
				case '-':
					LOGGER_WARNING("-");
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
	//				pid_pri.updateEEPROM();
	//				pid_sec.updateEEPROM();
	//				pid_yaw.updateEEPROM();
					displayPIDcoefficients();
					break;
				case 'r':							///< Reads all coefficients from the EEPROM
	//				pid_pri.readEEPROM();
	//				pid_sec.readEEPROM();
	//				pid_yaw.readEEPROM();
			 	break;
				case 'a':							///< Set all PID parameters to 0
	 				 _pid_pri->setP(PID_P_MIN);
					// pid_pri.setI(0);
					// pid_pri.setD(0);
					// pid_pri.setExecutionFrequency(50);
					// pid_sec.setP(PID_P_MIN);
					// pid_sec.setI(0);
					// pid_sec.setD(0);
					// pid_sec.setExecutionFrequency(50);
					// pid_yaw.setP(PID_P_MIN);
					// pid_yaw.setI(0);
					// pid_yaw.setD(0);
					// pid_yaw.setExecutionFrequency(50);	
					displayPIDcoefficients();
					break;

				case 'g':	//< get factory default
	 				// pid_pri.setP(0.2);
					// pid_pri.setI(0.041);
					// pid_pri.setD(0.1);
					// pid_pri.setExecutionFrequency(50);
					// pid_sec.setP(0.2);
					// pid_sec.setI(0.041);
					// pid_sec.setD(0.1);
					// pid_sec.setExecutionFrequency(50);
					// pid_yaw.setP(0.2);
					// pid_yaw.setI(0.01);
					// pid_yaw.setD(0);
					// pid_yaw.setExecutionFrequency(50);	
					displayPIDcoefficients();
					break;

				case 'c':	//< Copies the primary values to the secondary axis
					// pid_sec.setP(pid_pri.getP());
					// pid_sec.setI(pid_pri.getI());
					// pid_sec.setD(pid_pri.getD());				
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
		LOGGER_WARNING_FMT("itemAxis = %d", _itemAxis);
	}	/*----------------------------- end of setItemAxis ----------------------------*/ 
	
	void setItemCoefficient(uint8_t itemCoefficient) {
	 /* Selects the coefficient, according to the keyboard input.
	 * Key P = coefficient P (10)
	 * Key I = coefficient I (20)
	 * Key D = coefficient D (30)
	 * Key E = ExecutingFrequency (40)	 */
		_itemCoefficient = itemCoefficient;
		LOGGER_WARNING_FMT("itemCoefficient = %d", _itemCoefficient);
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
	}/*----------------------------- end of setDecimalPlaces --------------------------*/

	 /* Set the "PID Type",
	  * e.g _itemAxis = 1 and _itemCoefficient = 20 ~ _pidType 21
	  * Will say, it select the parameter for secondary axis and coefficient 'i'
	  */
	uint8_t getPidType(bool up) {   /// const deleted

		_pidType = _itemAxis + _itemCoefficient;
//		LOGGER_WARNING_FMT("PID Type = %d", _pidType);

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
    }/*----------------------------- end of getPidType --------------------------------*/

    /* Increment the coefficient according the PID Type. Shown "getPidType". */
	void coefficient_Up() {
		select(getPidType(true));
	}/*----------------------------- end of coefficient_Up ----------------------------*/

	/* decrement the coefficient according the PID Type. Shown "getPidType". */
	void coefficient_Down() {
		select(getPidType(false));
    }/*----------------------------- end of coefficient_Down --------------------------*/

	bool checkValue(float a){
		if(a >= 0)
			return true;
		else
			return false;
	}/*----------------------------- end of checkValue --------------------------------*/

    /* Here you sets the real coefficient into the model, and print it into the GUI. +/- _factor
	   like this: Pri. P = 2.20 
	*/
	void select(uint8_t type) {
	LOGGER_WARNING_FMT("Select %d", type);

		switch(type){

		case pidTyp_t::pri_P:
			_x_kP += _factor;
			if(checkValue(_x_kP)){
				LOGGER_WARNING_FMT("X kP = %f", _x_kP);
		//		pid_pri.setP(((_model->pidData[axis_t::Primary].pidCoefficient[pid::P]) += _factor));
	//			_pid_pri->setP(_x_kP);
			}
			break;
		case pidTyp_t::pri_I:
			_x_kI += _factor;
			if(checkValue(_x_kI)){
				LOGGER_WARNING_FMT("X kI = %f", _x_kI);
	//			pid_pri.setI(((_model->pidData[axis_t::Primary].pidCoefficient[pid::I]) += _factor));
	//			_pid_pri->setI(_x_kI);
			}
			break;
		case pidTyp_t::pri_D:
			_x_kD += _factor;
			if(checkValue(_x_kD)){
				LOGGER_WARNING_FMT("X kD = %f", _x_kD);
	//			pid_pri.setD(((_model->pidData[axis_t::Primary].pidCoefficient[pid::D]) += _factor));
	//			_pid_pri->setD(_x_kD);
			}
			break;

		case pidTyp_t::sec_P:
			_y_kP += _factor;
			if(checkValue(_y_kP)){
				LOGGER_WARNING_FMT("Y kP = %f", _y_kP);
	//			pid_sec.setP(((_model->pidData[axis_t::Secondary].pidCoefficient[pid::P]) += _factor));
	//			_pid_sec->setP(_y_kP);
			}
			break;

		case pidTyp_t::sec_I:
			_y_kI += _factor;
			if(checkValue(_y_kI)){
				LOGGER_WARNING_FMT("Y kI = %f", _y_kI);
	//			pid_sec.setI(((_model->pidData[axis_t::Secondary].pidCoefficient[pid::I]) += _factor));
	//			_pid_sec->setI(_y_kI);
			}
			break;

		case pidTyp_t::sec_D:
			_y_kD += _factor;
			if(checkValue(_y_kD)){
				LOGGER_WARNING_FMT("Y kD = %f", _y_kD);
	//			pid_sec.setD(((_model->pidData[axis_t::Secondary].pidCoefficient[pid::D]) += _factor));
	//			_pid_sec->setD(_y_kD);	
			}
			break;

		case pidTyp_t::yaw_P:
			_z_kP += _factor;
			if(checkValue(_z_kP)){
				LOGGER_WARNING_FMT("Z kP = %f", _z_kP);
	//			pid_yaw.setP(((_model->pidData[axis_t::YawAxis].pidCoefficient[pid::P]) += _factor));
	//			_pid_yaw->setD(_z_kP);
			}
			break;

		case pidTyp_t::yaw_I:
			_z_kI += _factor;
			if(checkValue(_z_kI)){
				LOGGER_WARNING_FMT("Z kI = %f", _z_kI);
	//			pid_yaw.setI(((_model->pidData[axis_t::YawAxis].pidCoefficient[pid::I]) += _factor));
	//			_pid_yaw->setI(_z_kI);
			}
			break;
		case pidTyp_t::yaw_D:
			_z_kD += _factor;
			if(checkValue(_z_kD)){
				LOGGER_WARNING_FMT("Z kD = %f", _z_kD);
	//			pid_yaw.setD(((_model->pidData[axis_t::YawAxis].pidCoefficient[pid::D]) += _factor));
	//			_pid_yaw->setD(_z_kD);
			}
			break;

		case pidTyp_t::pri_ef:
			_x_eF += _factor;
			if(checkValue(_x_eF)){
				LOGGER_WARNING_FMT("x eF = %f", _x_eF);
	//			_pid_pri->setExecutionFrequency(((_model->pidData[axis_t::Primary].executionFrequency) += _factor));
	//			_pid_pri->setExecutionFrequency(_x_eF);
			}
			break;
		case pidTyp_t::sec_ef:
			_y_eF += _factor;
			if(checkValue(_y_eF)){
				LOGGER_WARNING_FMT("y eF = %f", _y_eF);
	//			pid_sec.setExecutionFrequency(((_model->pidData[axis_t::Secondary].executionFrequency) += _factor));
	//			_pid_sec->setExecutionFrequency(_y_eF);
			}
			break;
		case pidTyp_t::yaw_ef:
			_z_eF += _factor;
			if(checkValue(_z_eF)){
				LOGGER_WARNING_FMT("z eF = %f", _z_eF);
	//			pid_yaw.setExecutionFrequency(((_model->pidData[axis_t::YawAxis].executionFrequency) += _factor));
	//			_pid_yaw->setExecutionFrequency(_z_eF);
			}
			break;
		} /* end of switch */
	}/*----------------------------- end of select ------------------------------------*/

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
} //-------------------------- end of display_Menu --------------------------------------

/* Displayed all PID coefficients from the _model */
void displayPIDcoefficients() {

 	LOGGER_WARNING("Current PID coefficients in the EEPROM");
	// Serial2.println(); 
 	// LOGGER_WARNING_FMT("Pri P = %f", _model->pidData[axis_t::Primary].pidCoefficient[pid::P]);
	// LOGGER_WARNING_FMT("    I = %f", _model->pidData[axis_t::Primary].pidCoefficient[pid::I]);
	// LOGGER_WARNING_FMT("    D = %f", _model->pidData[axis_t::Primary].pidCoefficient[pid::D]);
	// LOGGER_WARNING_FMT("   eF = %d", _model->pidData[axis_t::Primary].executionFrequency);
	// Serial2.println();
	// LOGGER_WARNING_FMT("Sec P = %f", _model->pidData[axis_t::Secondary].pidCoefficient[pid::P]);
	// LOGGER_WARNING_FMT("    I = %f", _model->pidData[axis_t::Secondary].pidCoefficient[pid::I]);
	// LOGGER_WARNING_FMT("    D = %f", _model->pidData[axis_t::Secondary].pidCoefficient[pid::D]);
	// LOGGER_WARNING_FMT("   eF = %d", _model->pidData[axis_t::Secondary].executionFrequency);
	// Serial2.println();
	// LOGGER_WARNING_FMT("Yaw P = %f", _model->pidData[axis_t::YawAxis].pidCoefficient[P]);
	// LOGGER_WARNING_FMT("    I = %f", _model->pidData[axis_t::YawAxis].pidCoefficient[pid::I]);
	// LOGGER_WARNING_FMT("    D = %f", _model->pidData[axis_t::YawAxis].pidCoefficient[pid::D]);
	// LOGGER_WARNING_FMT("   eF = %d", _model->pidData[axis_t::YawAxis].executionFrequency);
 
} //-------------------------- end of displayPIDcoefficients ----------------------------
};	/*------------------------ end of calibration.h class -----------------------------*/