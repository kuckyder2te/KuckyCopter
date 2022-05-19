#pragma once

#include <Arduino.h>
#include <TaskManager.h>
#include "..\lib\def.h"
#include "..\lib\gui.h"

#define ROW_MENU 	 2		///< First position for the main menue
#define COL_MENU     8

#define ROW_SELECT 	20		///< First position for select PID type
#define COL_SELECT   8

#define ROW_COEFF	25		///< First position for new coefficients
#define COL_COEFF	 8

#define ROW_PID     31		///< First position for current coefficients
#define COL_PID      8


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

typedef enum{
	axis_pri = 1,
	axis_sec = 2,
	axis_yaw = 3
}itemAxis_t;

typedef enum{
	kP = 10,
	kI = 20,
	kD = 30,
	eF = 40
}itemCoeff_t;

typedef enum{
	P = 0,
	I,
	D
}pid;

//extern Gui gui;

class Calibration : public Task::Base {
    bool b;         // Klassenvariable
    uint8_t _itemAxis;
    uint8_t _itemCoefficient;
	uint8_t _pidType;
    float   _newFactor = 0.1;	///< Multiplication factor for the PID coefficients, default setting.
	double  _factor;

	const char* c_pri_select = "Primary axis is select";		///< Strings for menu and informations
	const char* c_sec_select = "Secondary axis is select";
	const char* c_yaw_select = "YAW axis is select";
	const char* c_p_select = "Coefficient P is select";
	const char* c_i_select = "Coefficient I is select";
	const char* c_d_select = "Coefficient D is select";
	const char* c_ef_select = "Exec. frequency is select";
	const char* c_whitespace = "                           ";
	const char* c_pri_p = "Pri. P =";
	const char* c_pri_i = "I =";
	const char* c_pri_d = "D =";
	const char* c_sec_p = "Sec.";
	const char* c_sec_i = "I =";
	const char* c_sec_d = "D =";
	const char* c_yaw_p = "YAW";
	const char* c_yaw_i = "I =";
	const char* c_yaw_d = "D =";
	const char* c_ef = "Exec. Hz. =";	

protected:
	Gui *_gui;

public:
    Calibration(const String& name) : Task::Base(name) ,b(false){
        pinMode(CALIBRATION_LED, OUTPUT);
        digitalWrite(CALIBRATION_LED, LOW);
    }

    virtual ~Calibration() {}

    virtual void begin() override {
     	LOGGER_VERBOSE("Enter...");
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
				case 'x':													///< Choose the axes
					setItemAxis(itemAxis_t::axis_pri);
					_gui->print(45, 8, c_whitespace);						///< Clears the string "Illegal button was pressed"
					_gui->clearPart(ROW_SELECT, COL_SELECT, c_whitespace);	///< Clears the current line
					_gui->print    (ROW_SELECT, COL_SELECT, c_pri_select);	///< Print the selected axis
					break;
				case 'y':
					setItemAxis(itemAxis_t::axis_sec);
					_gui->print(45, 8, c_whitespace);
					_gui->clearPart(ROW_SELECT, COL_SELECT, c_whitespace);
					_gui->print    (ROW_SELECT, COL_SELECT, c_sec_select);
					break;
				case 'z':
					setItemAxis(itemAxis_t::axis_yaw);
					_gui->print(45, 8, c_whitespace);
					_gui->clearPart(ROW_SELECT, COL_SELECT, c_whitespace);
					_gui->print	 (ROW_SELECT, COL_SELECT, c_yaw_select);
					break;

				case 'p':													///< Choose the PID Coefficient
					setItemCoefficient(itemCoeff_t::kP);
					_gui->print(45, 8, c_whitespace);
					_gui->clearPart(ROW_SELECT+2, COL_SELECT, c_whitespace);
					_gui->print    (ROW_SELECT+2, COL_SELECT, c_p_select);	///< Print the selected coefficient
					break;
				case 'i':
					setItemCoefficient(itemCoeff_t::kI);
					_gui->print(45, 8, c_whitespace);
					_gui->clearPart(ROW_SELECT+2, COL_SELECT, c_whitespace);
					_gui->print    (ROW_SELECT+2, COL_SELECT, c_i_select);
					break;
				case 'd':
					setItemCoefficient(itemCoeff_t::kD);
					_gui->print(45, 8, c_whitespace);
					_gui->clearPart(ROW_SELECT+2, COL_SELECT, c_whitespace);
					_gui->print    (ROW_SELECT+2, COL_SELECT, c_d_select);
					break;
				case 'e':
					setItemCoefficient(itemCoeff_t::eF);
					_gui->print(45, 8, c_whitespace);
					_gui->clearPart(ROW_SELECT+2, COL_SELECT, c_whitespace);
					_gui->print    (ROW_SELECT+2, COL_SELECT, c_ef_select);
					break;

				case '+':
					_gui->print(45, 8, c_whitespace);
					coefficient_Up();			///< Coefficient increment
					break;
				case '-':
					_gui->print(45, 8, c_whitespace);
					coefficient_Down();			///< Coefficient decrement
					break;

				case '0':						///< Choose the decimal places  0 to 0,001
	//				setDecimalPlaces(0);
					_gui->print(45, 8, c_whitespace);
					_gui->clearPart(ROW_SELECT+1, COL_SELECT, c_whitespace);
					_gui->print    (ROW_SELECT+1, COL_SELECT, "Accuracy = 1,0");
					break;
				case '1':
	//				setDecimalPlaces(1);
					_gui->print(45, 8, c_whitespace);
					_gui->clearPart(ROW_SELECT+1, COL_SELECT, c_whitespace);
					_gui->print    (ROW_SELECT+1, COL_SELECT, "Accuracy = 0,1");
					break;
				case '2':
	//				setDecimalPlaces(2);
					_gui->print(45, 8, c_whitespace);
					_gui->clearPart(ROW_SELECT+1, COL_SELECT, c_whitespace);
					_gui->print    (ROW_SELECT+1, COL_SELECT, "Accuracy = 0,01");
					break;
				case '3':
	//				setDecimalPlaces(3);
					_gui->print(45, 8, c_whitespace);
					_gui->clearPart(ROW_SELECT+1, COL_SELECT, c_whitespace);
					_gui->print    (ROW_SELECT+1, COL_SELECT, "Accuracy = 0,001");
					break;

				case 's':							///< Saved all coefficients into the EEPROM
	//				myPID_pri.updateEEPROM();
	//				myPID_sec.updateEEPROM();
	//				myPID_yaw.updateEEPROM();
					_gui->red();
					_gui->print(45, 8, "PID data was backed up");
					_gui->yellow();
					displayPIDcoefficients();
					break;
				case 'r':							///< Reads all coefficients from the EEPROM
	//				myPID_pri.readEEPROM();
	//				myPID_sec.readEEPROM();
	//				myPID_yaw.readEEPROM();
					_gui->red();
					_gui->print(45, 8, "PID data was read out");
					_gui->yellow();
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
	//				
					break;

				case 'm':
	//				display_Menu();
					displayPIDcoefficients();
					break;

				default:{
					_gui->red();
					_gui->print(45, 8, "Illegal button was pressed");
					_gui->yellow();
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
		LOGGER_NOTICE("setItemAxis");
		LOGGER_NOTICE_FMT("itemAxis = %f", itemAxis);
	//	Serial2.print("itemAxis = ");Serial2.println(itemAxis);
	} 
	
	void setItemCoefficient(uint8_t itemCoefficient) {
	 /* Selects the coefficient, according to the keyboard input.
	 * Key P = coefficient P (10)
	 * Key I = coefficient I (20)
	 * Key D = coefficient D (30)
	 * Key E = ExecutingFrequency (40)	 */
		_itemCoefficient = itemCoefficient;
		LOGGER_NOTICE_FMT("itemCoefficient = %f", itemCoefficient);
	//	Serial2.print("itemCoefficient = ");Serial2.println(itemCoefficient);
	} 
	 /* Set the "PID Type",
	  * e.g _itemAxis = 1 and _itemCoefficient = 20 ~ _pidType 21
	  * Will say, it select the parameter for secondary axis and coefficient 'i'
	  */

	uint8_t getPidType(bool up) {   /// const deleted

		_pidType = _itemAxis + _itemCoefficient;

		if(_pidType < 40){			///< P, I and D

//			Serial2.print("New factor = ");Serial2.println(_newFactor, 3);

//			if(_pidType == pidTyp_t::pri_P || _pidType == pidTyp_t::sec_P || _pidType == pidTyp_t::yaw_P)
				if(up)
					_factor = _newFactor;
				else
					_factor = _newFactor *-1;
//			else
//				if(up)
//					_factor = FACTOR_ID;
//				else
//					_factor = FACTOR_ID *-1;
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

		switch(type){

		case pidTyp_t::pri_P:
//			myPID_pri.setP(((_model.pidData[axis_t::Primary].pidCoefficient[pid::P]) += _factor));
			break;
		case pidTyp_t::pri_I:
//			myPID_pri.setI(((_model.pidData[axis_t::Primary].pidCoefficient[pid::I]) += _factor));
			break;
		case pidTyp_t::pri_D:
//			myPID_pri.setD(((_model.pidData[axis_t::Primary].pidCoefficient[pid::D]) += _factor));
			break;

		case pidTyp_t::sec_P:
//			myPID_sec.setP(((_model.pidData[axis_t::Secondary].pidCoefficient[pid::P]) += _factor));
			break;
		case pidTyp_t::sec_I:
//			myPID_sec.setI(((_model.pidData[axis_t::Secondary].pidCoefficient[pid::I]) += _factor));
			break;
		case pidTyp_t::sec_D:
//			myPID_sec.setD(((_model.pidData[axis_t::Secondary].pidCoefficient[pid::D]) += _factor));
			break;

		case pidTyp_t::yaw_P:
//			myPID_yaw.setP(((_model.pidData[axis_t::YawAxis].pidCoefficient[pid::P]) += _factor));
			break;
		case pidTyp_t::yaw_I:
//			myPID_yaw.setI(((_model.pidData[axis_t::YawAxis].pidCoefficient[pid::I]) += _factor));
			break;
		case pidTyp_t::yaw_D:
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

	_gui->clear();
	_gui->gray();
	_gui->print(ROW_MENU,   COL_MENU, "-----------Menu for PID configuration (BT)-----------");
	_gui->yellow();
	_gui->print(ROW_MENU+2, COL_MENU, "(X) choose the primary");
	_gui->print(ROW_MENU+3, COL_MENU, "(Y)           secondary");
	_gui->print(ROW_MENU+4, COL_MENU, "(Z)            YAW axis");
	_gui->print(ROW_MENU+5, COL_MENU, " P, I or D select the coefficient");
	_gui->print(ROW_MENU+6, COL_MENU, "(0),(1),(2)or(3)select the accurarcy");
	_gui->print(ROW_MENU+7, COL_MENU, "(E) choose the execution frequency");
	_gui->print(ROW_MENU+8, COL_MENU, "(+) increment according to the value");
	_gui->print(ROW_MENU+9, COL_MENU, "(-) decrement      ''");
	_gui->print(ROW_MENU+10, COL_MENU,"(S) saves all coefficient into the EEPROM");
	_gui->print(ROW_MENU+11, COL_MENU,"(R) reads all coefficients from the EEPROM");
	_gui->print(ROW_MENU+12, COL_MENU,"(C) Copies the primary values to the secondary axis");
	_gui->print(ROW_MENU+13, COL_MENU,"(A) all values are set to 0 in the EEPROM.");
	_gui->print(ROW_MENU+14, COL_MENU,"(G) get factory defaults");
	_gui->print(ROW_MENU+15,COL_MENU, "(M) display the menu");
	_gui->gray();
	_gui->print(ROW_MENU+18,COL_MENU, "-----------------------------------------------------");
	_gui->yellow();

} //-------------------------- end of display_Menu ------------------------------------------------
/*
 * Displayed all PID coefficients from the _model */

void displayPIDcoefficients() {

	_gui->gray();
	_gui->print(ROW_PID, COL_PID, "Current PID coefficients in the EEPROM");
	_gui->yellow();
/* 	_gui->print(ROW_PID+2, COL_PID,    c_pri_p);_gui->print(ROW_PID+2, COL_PID+9,  2, _model.pidData[axis_t::Primary].pidCoefficient[pid::P]);
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