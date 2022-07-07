#pragma once
/*  File name : PID_Calbration.h
	Project name : KuCo_Phantom 1
	Author: Wilhelm Kuckelsberg
	Date : 2022-07-02
	Description : Einstellen de PID Regler via Bluetooth
				  und die maximalen Flughöhen
*/

#include <TaskManager.h>
#include <Arduino.h>
#include <Config.h>
#include <HardwareSerial.h>
#include "..\lib\myLogger.h"
//#include <EEPROM.h>
#include "..\lib\GUI.h"
//#include "MyPid.h"
//#include "Model.h"

//#ifdef _PID_CALIBRATION

//#define FACTOR_P    0.01		///< change to 0,01 resolution via putty
//#define FACTOR_ID   0.001

// Font Courier New bold 12 point

#define ROW_MENU 3 ///< First position for the main menue
#define COL_MENU 10

#define ROW_SELECT 26 ///< First position for select PID type
#define COL_SELECT 20

#define ROW_COEFF 28 ///< First position for new coefficients
#define COL_COEFF 20

#define ROW_PID 33 ///< First position for current coefficients
#define COL_PID 20

#define ROW_OUTPUT 5
#define COL_OUTPUT 70
#define COL_OUTPUT_VALUE 88

#define ROW_ILLEGAL 40 // Position for errors
#define COL_ILLEGAL 20

// extern modules::MyPid myPID_pri;
// extern modules::MyPid myPID_sec;
// extern modules::MyPid myPID_yaw;

// extern GUI gui;

typedef enum
{
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
} pidTyp_t;

typedef enum
{
	axis_pri = 1,
	axis_sec = 2,
	axis_yaw = 3
} itemAxis_t;

typedef enum
{
	xkP = 10,
	xkI = 20,
	xkD = 30,
	xeF = 40
} itemCoeff_t;

typedef enum
{
	P = 0,
	I,
	D
} pid;

class PID_adjust : public Task::Base
{

	//	model_t		   &_model;
	HardwareSerial *_serial;

	uint8_t _itemAxis;
	uint8_t _itemCoefficient;
	uint8_t _pidType;
	uint8_t _dotPlaces = 3; ///< Decimal places.
	float _newAddOn = 0.1;	///< Multiplication factor for the PID coefficients, default setting.
	double _addOn;

	const char *c_pri_select = "Primary axis is select"; ///< Strings for menu and informations
	const char *c_sec_select = "Secondary axis is select";
	const char *c_yaw_select = "YAW axis is select";
	const char *c_ef_select = "Exec. frequency is select";

	const char *c_p_select = "Coefficient P = ";
	const char *c_i_select = "Coefficient I = ";
	const char *c_d_select = "Coefficient D = ";

	const char *c_whitespace = "                           ";
	// const char *c_equal_sign = " = ";
	//  const char *c_pri_i = "I =";
	//  const char *c_pri_d = "D =";
	const char *c_primary_p = "Primary axis   P = ";
	const char *c_primary_i = "               I = ";
	const char *c_primary_d = "               D = ";
	const char *c_secondary_p = "Secondary axis P = ";
	const char *c_secondary_i = "               I = ";
	const char *c_secondary_d = "               D = ";
	const char *c_yaw_p = "YAW axis       P = ";
	const char *c_yaw_i = "               I = ";
	const char *c_yaw_d = "               D = ";
	const char *c_ef_pri = "Exec. freq. Pri  = ";
	const char *c_ef_sec = "            Sec. = ";
	const char *c_ef_yaw = "            YAW  = ";

	// const char *c_sec_p = "Sec.";
	// const char *c_sec_i = "I =";
	// const char *c_sec_d = "D =";
	// const char *c_yaw_p = "YAW";
	// const char *c_yaw_i = "I =";
	// const char *c_yaw_d = "D =";
	// const char *c_ef = "Exec. Hz. =";

	float _x_kP = 0;
	float _x_kI = 0;
	float _x_kD = 0;
	float _x_eF = 0;
	float _y_kP = 0;
	float _y_kI = 0;
	float _y_kD = 0;
	float _y_eF = 0;
	float _z_kP = 0;
	float _z_kI = 0;
	float _z_kD = 0;
	float _z_eF = 0;

	GUI *_gui;

public:
	PID_adjust(const String &name)
		: Task::Base(name)
	{
	}

	virtual ~PID_adjust()
	{
	}

	PID_adjust *setSerial(HardwareSerial *serial)
	{
		_serial = serial;
		_gui = new GUI(*serial);
		return this;
	}

	virtual void begin() override
	{
		//	LOGGER_VERBOSE("Enter....");
			display_Menu();
		//	LOGGER_VERBOSE("....leave");
	}

	virtual void update() override
	{
		//	LOGGER_VERBOSE("Enter....");

		if (_serial->available() > 0) // Hier werden die gedrückten keys abgefragt
		{
			char key = _serial->read();
			switch (key)
			{
			case 'x': ///< Choose the axes
				setItemAxis(itemAxis_t::axis_pri);
				_gui->yellow();
				_gui->print(ROW_ILLEGAL, COL_ILLEGAL, c_whitespace);	   ///< Clears the string "Illegal button was pressed"
				_gui->clearPart(ROW_SELECT, COL_SELECT + 5, c_whitespace); ///< Clears the current line
				_gui->print(ROW_SELECT, COL_SELECT + 5, c_pri_select);	   ///< Print the selected axis
				break;
			case 'y':
				setItemAxis(itemAxis_t::axis_sec);
				_gui->yellow();
				_gui->print(ROW_ILLEGAL, COL_ILLEGAL, c_whitespace);
				_gui->clearPart(ROW_SELECT, COL_SELECT + 5, c_whitespace);
				_gui->print(ROW_SELECT, COL_SELECT + 5, c_sec_select);
				break;
			case 'z':
				setItemAxis(itemAxis_t::axis_yaw);
				_gui->yellow();
				_gui->print(ROW_ILLEGAL, COL_ILLEGAL, c_whitespace);
				_gui->clearPart(ROW_SELECT, COL_SELECT + 5, c_whitespace);
				_gui->print(ROW_SELECT, COL_SELECT + 5, c_yaw_select);
				break;

			case 'p': ///< Choose the PID parameter
				setItemCoefficient(itemCoeff_t::xkP);
				_gui->yellow();
				_gui->print(ROW_ILLEGAL, COL_ILLEGAL, c_whitespace);
				_gui->clearPart(ROW_SELECT + 2, COL_SELECT + 10, c_whitespace);
				_gui->print(ROW_SELECT + 2, COL_SELECT + 10, c_p_select); ///< Print the selected coefficient
				break;

			case 'i':
				setItemCoefficient(itemCoeff_t::xkI);
				_gui->yellow();
				_gui->print(ROW_ILLEGAL, COL_ILLEGAL, c_whitespace);
				_gui->clearPart(ROW_SELECT + 3, COL_SELECT + 10, c_whitespace);
				_gui->print(ROW_SELECT + 3, COL_SELECT + 10, c_i_select);
				break;

			case 'd':
				setItemCoefficient(itemCoeff_t::xkD);
				_gui->yellow();
				_gui->print(ROW_ILLEGAL, COL_ILLEGAL, c_whitespace);
				_gui->clearPart(ROW_SELECT + 4, COL_SELECT + 10, c_whitespace);
				_gui->print(ROW_SELECT + 4, COL_SELECT + 10, c_d_select);
				break;

			case 'e':
				setItemCoefficient(itemCoeff_t::xeF);
				_gui->yellow();
				_gui->print(ROW_ILLEGAL, COL_ILLEGAL, c_whitespace);
				_gui->clearPart(ROW_SELECT, COL_SELECT + 5, c_whitespace);
				_gui->print(ROW_SELECT, COL_SELECT + 5, c_ef_select);
				break;

			case '+':
				_gui->yellow();
				_gui->print(ROW_ILLEGAL, COL_ILLEGAL, c_whitespace);
				coefficient_Up(); ///< Coefficient increment
				break;

			case '-':
				_gui->yellow();
				_gui->print(ROW_ILLEGAL, COL_ILLEGAL, c_whitespace);
				coefficient_Down(); ///< Coefficient decrement
				break;

			case '0': ///< Choose the decimal places  0 to 0,001
				setDecimalPlaces(0);
				_gui->print(ROW_ILLEGAL, COL_ILLEGAL, c_whitespace);
				_gui->clearPart(ROW_SELECT + 1, COL_SELECT, c_whitespace);
				_gui->print(ROW_SELECT + 1, COL_SELECT, "Accuracy = 1,0");
				break;
			case '1':
				setDecimalPlaces(1);
				_gui->print(ROW_ILLEGAL, COL_ILLEGAL, c_whitespace);
				_gui->clearPart(ROW_SELECT + 1, COL_SELECT, c_whitespace);
				_gui->print(ROW_SELECT + 1, COL_SELECT, "Accuracy = 0,1");
				break;
			case '2':
				setDecimalPlaces(2);
				_gui->print(ROW_ILLEGAL, COL_ILLEGAL, c_whitespace);
				_gui->clearPart(ROW_SELECT + 1, COL_SELECT, c_whitespace);
				_gui->print(ROW_SELECT + 1, COL_SELECT, "Accuracy = 0,01");
				break;
			case '3':
				setDecimalPlaces(3);
				_gui->print(ROW_ILLEGAL, COL_ILLEGAL, c_whitespace);
				_gui->clearPart(ROW_ILLEGAL + 1, COL_SELECT, c_whitespace);
				_gui->print(ROW_SELECT + 1, COL_SELECT, "Accuracy = 0,001");
				break;

			case 's': ///< Saved all coefficients into the EEPROM
				// myPID_pri.updateEEPROM();
				// myPID_sec.updateEEPROM();
				// myPID_yaw.updateEEPROM();
				_gui->red();
				_gui->print(45, 8, "PID data was backed up");
				_gui->yellow();
				displayPIDcoefficients();
				break;
			case 'r': ///< Reads all coefficients from the EEPROM
				// myPID_pri.readEEPROM();
				// myPID_sec.readEEPROM();
				// myPID_yaw.readEEPROM();
				_gui->red();
				_gui->print(45, 8, "PID data was read out");
				_gui->yellow();
				break;
			case 'a': ///< Set all PID parameters to 0
				// myPID_pri.setP(PID_P_MIN);
				// myPID_pri.setI(0);
				// myPID_pri.setD(0);
				// myPID_pri.setExecutionFrequency(50);
				// myPID_sec.setP(PID_P_MIN);
				// myPID_sec.setI(0);
				// myPID_sec.setD(0);
				// myPID_sec.setExecutionFrequency(50);
				// myPID_yaw.setP(PID_P_MIN);
				// myPID_yaw.setI(0);
				// myPID_yaw.setD(0);
				// myPID_yaw.setExecutionFrequency(50);
				displayPIDcoefficients();
				break;

			case 'g': ///< get factory default
				// myPID_pri.setP(0.2);
				// myPID_pri.setI(0.041);
				// myPID_pri.setD(0.1);
				// myPID_pri.setExecutionFrequency(50);
				// myPID_sec.setP(0.2);
				// myPID_sec.setI(0.041);
				// myPID_sec.setD(0.1);
				// myPID_sec.setExecutionFrequency(50);
				// myPID_yaw.setP(0.2);
				// myPID_yaw.setI(0.01);
				// myPID_yaw.setD(0);
				// myPID_yaw.setExecutionFrequency(50);
				displayPIDcoefficients();
				break;

			case 'c': ///< Copies the primary values to the secondary axis
				// myPID_sec.setP(myPID_pri.getP());
				// myPID_sec.setI(myPID_pri.getI());
				// myPID_sec.setD(myPID_pri.getD());
				break;

			case 'm':
				display_Menu();
				displayPIDcoefficients();
				break;

			case 'h':
				// LOGGER_NOTICE("not implemented");
				break;

			case 'n':
				// LOGGER_NOTICE("not implemented");
				break;

			default:
			{
				_gui->red();
				_gui->print(ROW_ILLEGAL, COL_ILLEGAL, "Illegal button was pressed");
				_gui->yellow();
			}
			} /* end of switch(key) */
		}	  /* end of _serial.available */
			  //	LOGGER_VERBOSE("....leave");
	}		  /* -------------------- end of update -------------------------------------------*/

	void display_Menu()
	{
		//	LOGGER_VERBOSE("Enter....");
		_gui->clear();
		_gui->clear();
		_gui->gray();
		_gui->print(ROW_MENU, COL_MENU, "-----------Menu for PID configuration (BT)-----------");
		_gui->yellow();
		_gui->print(ROW_MENU + 2, COL_MENU, "(X) choose the primary");
		_gui->print(ROW_MENU + 3, COL_MENU, "(Y)           secondary");
		_gui->print(ROW_MENU + 4, COL_MENU, "(Z)            YAW axis");
		_gui->print(ROW_MENU + 5, COL_MENU, " P, I or D select the coefficient");
		_gui->print(ROW_MENU + 6, COL_MENU, "(0),(1),(2)or(3)select the accurarcy");
		_gui->print(ROW_MENU + 7, COL_MENU, "(E) choose the execution frequency");
		_gui->print(ROW_MENU + 8, COL_MENU, "(+) increment according to the value");
		_gui->print(ROW_MENU + 9, COL_MENU, "(-) decrement      ''");
		_gui->print(ROW_MENU + 10, COL_MENU, "(S) saves all coefficient into the EEPROM");
		_gui->print(ROW_MENU + 11, COL_MENU, "(R) reads all coefficients from the EEPROM");
		_gui->print(ROW_MENU + 12, COL_MENU, "(C) Copies the primary values to the secondary axis");
		_gui->print(ROW_MENU + 13, COL_MENU, "(A) all values are set to 0 in the EEPROM.");
		_gui->print(ROW_MENU + 14, COL_MENU, "(G) get factory defaults");
		_gui->print(ROW_MENU + 15, COL_MENU, "(H) set the maximal altitude");
		_gui->print(ROW_MENU + 16, COL_MENU, "(N) set the maximal near field altitude");
		_gui->print(ROW_MENU + 17, COL_MENU, "(M) display the menu");
		_gui->gray();
		_gui->print(ROW_MENU + 19, COL_MENU, "-----------------------------------------------------");
		_gui->yellow();
		//	LOGGER_VERBOSE("....leave");
	} /*-------------------------- end of display_Menu --------------------------------*/

	void displayPIDcoefficients()
	{
		_gui->gray();
		_gui->print(ROW_PID, COL_PID, "Current PID coefficients in the EEPROM");
		_gui->yellow();
		// _gui->print(ROW_PID+2, COL_PID,    c_equal_sign);_gui->print(ROW_PID+2, COL_PID+9,  2, _model.pidData[axis_t::Primary].pidCoefficient[pid::P]);
		// _gui->print(ROW_PID+2, COL_PID+16, c_pri_i);_gui->print(ROW_PID+2, COL_PID+20, 3, _model.pidData[axis_t::Primary].pidCoefficient[pid::I]);
		// _gui->print(ROW_PID+2, COL_PID+27, c_pri_d);_gui->print(ROW_PID+2, COL_PID+31, 3, _model.pidData[axis_t::Primary].pidCoefficient[pid::D]);
		// _gui->print(ROW_PID+2, COL_PID+40, c_ef);   _gui->print(ROW_PID+2, COL_PID+52, 0, _model.pidData[axis_t::Primary].executionFrequency);

		// _gui->print(ROW_PID+3, COL_PID,    c_sec_p);_gui->print(ROW_PID+3, COL_PID+9,  2, _model.pidData[axis_t::Secondary].pidCoefficient[pid::P]);
		// 										  _gui->print(ROW_PID+3, COL_PID+20, 3, _model.pidData[axis_t::Secondary].pidCoefficient[pid::I]);
		// 										  _gui->print(ROW_PID+3, COL_PID+31, 3, _model.pidData[axis_t::Secondary].pidCoefficient[pid::D]);
		// 										  _gui->print(ROW_PID+3, COL_PID+52, 0, _model.pidData[axis_t::Secondary].executionFrequency);

		// _gui->print(ROW_PID+4, COL_PID,    c_yaw_p);_gui->print(ROW_PID+4, COL_PID+9,  2, _model.pidData[axis_t::YawAxis].pidCoefficient[P]);
		// 										  _gui->print(ROW_PID+4, COL_PID+20, 3, _model.pidData[axis_t::YawAxis].pidCoefficient[pid::I]);
		// 										  _gui->print(ROW_PID+4, COL_PID+31, 3, _model.pidData[axis_t::YawAxis].pidCoefficient[pid::D]);
		// 										  _gui->print(ROW_PID+4, COL_PID+52, 0, _model.pidData[axis_t::YawAxis].executionFrequency);

	} /*--------------------- end of displayPIDcoefficients ---------------------------*/

	void displayPIDcoefficientsTemp() ///  only Template for the positions
	{
		_gui->blue();
		_gui->print(ROW_OUTPUT, COL_OUTPUT, "Temp PID coefficients");
		_gui->print(ROW_OUTPUT + 1, COL_OUTPUT, "   in the EEPROM");
		_gui->gray();
		_gui->print(ROW_OUTPUT + 3, COL_OUTPUT, c_primary_p);
		_gui->print(ROW_OUTPUT + 3, COL_OUTPUT_VALUE, 2, _x_kP);
		_gui->print(ROW_OUTPUT + 4, COL_OUTPUT, c_primary_i);
		_gui->print(ROW_OUTPUT + 4, COL_OUTPUT_VALUE, 2, _x_kI);
		_gui->print(ROW_OUTPUT + 5, COL_OUTPUT, c_primary_d);
		_gui->print(ROW_OUTPUT + 5, COL_OUTPUT_VALUE, 2, _x_kD);
		_gui->print(ROW_OUTPUT + 7, COL_OUTPUT, c_secondary_p);
		_gui->print(ROW_OUTPUT + 7, COL_OUTPUT_VALUE, 2, _y_kP);
		_gui->print(ROW_OUTPUT + 8, COL_OUTPUT, c_secondary_i);
		_gui->print(ROW_OUTPUT + 8, COL_OUTPUT_VALUE, 2, _y_kI);
		_gui->print(ROW_OUTPUT + 9, COL_OUTPUT, c_secondary_d);
		_gui->print(ROW_OUTPUT + 9, COL_OUTPUT_VALUE, 2, _y_kD);
		_gui->print(ROW_OUTPUT + 11, COL_OUTPUT, c_yaw_p);
		_gui->print(ROW_OUTPUT + 11, COL_OUTPUT_VALUE, 2, _z_kP);
		_gui->print(ROW_OUTPUT + 12, COL_OUTPUT, c_yaw_i);
		_gui->print(ROW_OUTPUT + 12, COL_OUTPUT_VALUE, 2, _z_kI);
		_gui->print(ROW_OUTPUT + 13, COL_OUTPUT, c_yaw_d);
		_gui->print(ROW_OUTPUT + 13, COL_OUTPUT_VALUE, 2, _z_kD);
		_gui->print(ROW_OUTPUT + 15, COL_OUTPUT, c_ef_pri);
		_gui->print(ROW_OUTPUT + 15, COL_OUTPUT_VALUE, 2, _x_eF);
		_gui->print(ROW_OUTPUT + 16, COL_OUTPUT, c_ef_sec);
		_gui->print(ROW_OUTPUT + 16, COL_OUTPUT_VALUE, 2, _y_eF);
		_gui->print(ROW_OUTPUT + 17, COL_OUTPUT, c_ef_yaw);
		_gui->print(ROW_OUTPUT + 17, COL_OUTPUT_VALUE, 2, _z_eF);

		// _gui->print(ROW_PID+2, COL_PID+16, c_pri_i);_gui->print(ROW_PID+2, COL_PID+20, 3, _model.pidData[axis_t::Primary].pidCoefficient[pid::I]);
		// _gui->print(ROW_PID+2, COL_PID+27, c_pri_d);_gui->print(ROW_PID+2, COL_PID+31, 3, _model.pidData[axis_t::Primary].pidCoefficient[pid::D]);
		// _gui->print(ROW_PID+2, COL_PID+40, c_ef);   _gui->print(ROW_PID+2, COL_PID+52, 0, _model.pidData[axis_t::Primary].executionFrequency);

		// _gui->print(ROW_PID+3, COL_PID,    c_sec_p);_gui->print(ROW_PID+3, COL_PID+9,  2, _model.pidData[axis_t::Secondary].pidCoefficient[pid::P]);
		// 										  _gui->print(ROW_PID+3, COL_PID+20, 3, _model.pidData[axis_t::Secondary].pidCoefficient[pid::I]);
		// 										  _gui->print(ROW_PID+3, COL_PID+31, 3, _model.pidData[axis_t::Secondary].pidCoefficient[pid::D]);
		// 										  _gui->print(ROW_PID+3, COL_PID+52, 0, _model.pidData[axis_t::Secondary].executionFrequency);

		// _gui->print(ROW_PID+4, COL_PID,    c_yaw_p);_gui->print(ROW_PID+4, COL_PID+9,  2, _model.pidData[axis_t::YawAxis].pidCoefficient[P]);
		// 										  _gui->print(ROW_PID+4, COL_PID+20, 3, _model.pidData[axis_t::YawAxis].pidCoefficient[pid::I]);
		// 										  _gui->print(ROW_PID+4, COL_PID+31, 3, _model.pidData[axis_t::YawAxis].pidCoefficient[pid::D]);
		// 										  _gui->print(ROW_PID+4, COL_PID+52, 0, _model.pidData[axis_t::YawAxis].executionFrequency);

	} /*--------------------- end of displayPIDcoefficients ---------------------------*/
	void setItemAxis(uint8_t itemAxis)
	{
		/* Selects the axis, according to the keyboard input.
		 * Key X = primary axis (1)
		 * Key Y = primary axis (2)
		 * Key Z = primary axis (3)	 */
		_itemAxis = itemAxis;
		//	LOGGER_WARNING_FMT("itemAxis = %d", _itemAxis);
	} /*----------------------------- end of setItemAxis ------------------------------*/

	void setItemCoefficient(uint8_t itemCoefficient)
	{
		/* Selects the coefficient, according to the keyboard input.
		 * Key P = coefficient P (10)
		 * Key I = coefficient I (20)
		 * Key D = coefficient D (30)
		 * Key E = ExecutingFrequency (40)	 */
		_itemCoefficient = itemCoefficient;
		//	LOGGER_WARNING_FMT("itemCoefficient = %d", _itemCoefficient);
	} /*----------------------------- end of setItemCoefficient -----------------------*/

	void setDecimalPlaces(uint8_t dot)
	{

		switch (dot)
		{
		case 0:
			_newAddOn = 1;
			break;
		case 1:
			_newAddOn = 0.1;
			break;
		case 2:
			_newAddOn = 0.01;
			break;
		case 3:
			_newAddOn = 0.001;
			break;
		} // end of switch
		  //	LOGGER_WARNING_FMT("New Factor = %f", _newAddOn);
	}	  /*----------------------------- end of setDecimalPlaces -------------------------*/

	/* Set the "PID Type",
	 * e.g _itemAxis = 1 and _itemCoefficient = 20 ~ _pidType 21
	 * Will say, it select the parameter for secondary axis and coefficient 'i'
	 */
	uint8_t getPidType(bool up)
	{ /// const deleted

		_pidType = _itemAxis + _itemCoefficient;
		//	LOGGER_WARNING_FMT("PID Type = %d", _pidType);

		if (_pidType < 40)
		{ ///< P, I and D

			if (up)
				_addOn = _newAddOn;
			else
				_addOn = _newAddOn * -1;
		}
		if (_pidType >= 40)
		{ ///< Executingfrequency only
			if (up)
				_addOn = 1;
			else
				_addOn = -1;
		}
		return _pidType;
	} /*----------------------------- end of getPidType -------------------------------*/

	/* Increment the coefficient according the PID Type. Shown "getPidType". */
	void coefficient_Up()
	{
		select(getPidType(true));
	} /*----------------------------- end of coefficient_Up ---------------------------*/

	/* decrement the coefficient according the PID Type. Shown "getPidType". */
	void coefficient_Down()
	{
		select(getPidType(false));
	} /*----------------------------- end of coefficient_Down -------------------------*/

	bool checkValue(float a) // It should be ensured that no negative values are passed.
	{
		if (a >= 0)
			return true;
		else
			return false;
	} /*----------------------------- end of checkValue -------------------------------*/

	/* Here you sets the real coefficient into the model, and print it into the GUI. +/- _addOn
	   like this: Pri. P = 2.20
	*/
	void select(uint8_t type) // put the coefficient into the right PID type
	{
		//	LOGGER_WARNING_FMT("Select %d", type);

		switch (type)
		{

		case pidTyp_t::pri_P:
			_x_kP += _addOn;
			if (checkValue(_x_kP))
			{
				// LOGGER_WARNING_FMT("X Axis kP = %f", _x_kP);
				_gui->print(ROW_SELECT + 2, COL_SELECT + 26, _dotPlaces, _x_kP);
				displayPIDcoefficientsTemp();
				// pid_pri.setP(((_model->pidData[axis_t::Primary].pidCoefficient[pid::P]) += _addOn));
				//_pid_pri->setP(_x_kP);
			}
			break;
		case pidTyp_t::pri_I:
			_x_kI += _addOn;
			if (checkValue(_x_kI))
			{
				// LOGGER_WARNING_FMT("X Axis kI = %f", _x_kI);
				_gui->print(ROW_SELECT + 3, COL_SELECT + 26, _dotPlaces, _x_kI);
				displayPIDcoefficientsTemp();
				// pid_pri.setI(((_model->pidData[axis_t::Primary].pidCoefficient[pid::I]) += _addOn));
				//_pid_pri->setI(_x_kI);
			}
			break;
		case pidTyp_t::pri_D:
			_x_kD += _addOn;
			if (checkValue(_x_kD))
			{
				// LOGGER_WARNING_FMT("X Axis kD = %f", _x_kD);
				_gui->print(ROW_SELECT + 4, COL_SELECT + 26, _dotPlaces, _x_kD);
				displayPIDcoefficientsTemp();
				// pid_pri.setD(((_model->pidData[axis_t::Primary].pidCoefficient[pid::D]) += _addOn));
				//_pid_pri->setD(_x_kD);
			}
			break;

		case pidTyp_t::sec_P:
			_y_kP += _addOn;
			if (checkValue(_y_kP))
			{
				// LOGGER_WARNING_FMT("Y Axis kP = %f", _y_kP);
				_gui->print(ROW_SELECT + 2, COL_SELECT + 26, _dotPlaces, _y_kP);
				displayPIDcoefficientsTemp();
				// pid_sec.setP(((_model->pidData[axis_t::Secondary].pidCoefficient[pid::P]) += _addOn));
				//_pid_sec->setP(_y_kP);
			}
			break;

		case pidTyp_t::sec_I:
			_y_kI += _addOn;
			if (checkValue(_y_kI))
			{
				//			LOGGER_WARNING_FMT("Y Axis kI = %f", _y_kI);
				_gui->print(ROW_SELECT + 3, COL_SELECT + 26, _dotPlaces, _y_kI);
				displayPIDcoefficientsTemp();
				//			pid_sec.setI(((_model->pidData[axis_t::Secondary].pidCoefficient[pid::I]) += _addOn));
				//			_pid_sec->setI(_y_kI);
			}
			break;

		case pidTyp_t::sec_D:
			_y_kD += _addOn;
			if (checkValue(_y_kD))
			{
				// LOGGER_WARNING_FMT("Y Axis kD = %f", _y_kD);
				_gui->print(ROW_SELECT + 4, COL_SELECT + 26, _dotPlaces, _y_kD);
				displayPIDcoefficientsTemp();
				// pid_sec.setD(((_model->pidData[axis_t::Secondary].pidCoefficient[pid::D]) += _addOn));
				//_pid_sec->setD(_y_kD);
			}
			break;

		case pidTyp_t::yaw_P:
			_z_kP += _addOn;
			if (checkValue(_z_kP))
			{
				// LOGGER_WARNING_FMT("Z Axis kP = %f", _z_kP);
				_gui->print(ROW_SELECT + 2, COL_SELECT + 26, _dotPlaces, _z_kP);
				displayPIDcoefficientsTemp();
				// pid_yaw.setP(((_model->pidData[axis_t::YawAxis].pidCoefficient[pid::P]) += _addOn));
				//_pid_yaw->setD(_z_kP);
			}
			break;

		case pidTyp_t::yaw_I:
			_z_kI += _addOn;
			if (checkValue(_z_kI))
			{
				// LOGGER_WARNING_FMT("Z Axis kI = %f", _z_kI);
				_gui->print(ROW_SELECT + 3, COL_SELECT + 26, _dotPlaces, _z_kI);
				displayPIDcoefficientsTemp();
				// pid_yaw.setI(((_model->pidData[axis_t::YawAxis].pidCoefficient[pid::I]) += _addOn));
				//_pid_yaw->setI(_z_kI);
			}
			break;

		case pidTyp_t::yaw_D:
			_z_kD += _addOn;
			if (checkValue(_z_kD))
			{
				// LOGGER_WARNING_FMT("Z Axis kD = %f", _z_kD);
				_gui->print(ROW_SELECT + 4, COL_SELECT + 26, _dotPlaces, _z_kD);
				displayPIDcoefficientsTemp();
				// pid_yaw.setD(((_model->pidData[axis_t::YawAxis].pidCoefficient[pid::D]) += _addOn));
				//_pid_yaw->setD(_z_kD);
			}
			break;

		case pidTyp_t::pri_ef:
			_x_eF += _addOn;
			if (checkValue(_x_eF))
			{
				// LOGGER_WARNING_FMT("X Axis eF = %f", _x_eF);
				_gui->print(ROW_SELECT + 4, COL_SELECT + 26, _dotPlaces, _x_eF);
				displayPIDcoefficientsTemp();
				//_pid_pri->setExecutionFrequency(((_model->pidData[axis_t::Primary].executionFrequency) += _addOn));
				//_pid_pri->setExecutionFrequency(_x_eF);
			}
			break;

		case pidTyp_t::sec_ef:
			_y_eF += _addOn;
			if (checkValue(_y_eF))
			{
				// LOGGER_WARNING_FMT("Y Axis eF = %f", _y_eF);
				_gui->print(ROW_SELECT + 4, COL_SELECT + 26, _dotPlaces, _y_eF);
				displayPIDcoefficientsTemp();
				// pid_sec.setExecutionFrequency(((_model->pidData[axis_t::Secondary].executionFrequency) += _addOn));
				//_pid_sec->setExecutionFrequency(_y_eF);
			}
			break;
		case pidTyp_t::yaw_ef:
			_z_eF += _addOn;
			if (checkValue(_z_eF))
			{
				// LOGGER_WARNING_FMT("Z Axis eF = %f", _z_eF);
				_gui->print(ROW_SELECT + 4, COL_SELECT + 26, _dotPlaces, _z_eF);
				displayPIDcoefficientsTemp();
				// pid_yaw.setExecutionFrequency(((_model->pidData[axis_t::YawAxis].executionFrequency) += _addOn));
				//_pid_yaw->setExecutionFrequency(_z_eF);
			}
			break;
		} /* end of switch */
	}	  /*----------------------------- end of select ------------------------------------*/
};		  /*------------------------- end of PID_calibration class ----------------------------*/

//#endif
