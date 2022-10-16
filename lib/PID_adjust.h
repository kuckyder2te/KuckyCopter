/*  File name : PID_Calbration.h
	Project name : KuCo_Phantom 1
	Author: Wilhelm Kuckelsberg
	Date : 2022-07-02
	Description : Einstellen de PID Regler via Bluetooth
				  und die maximalen Flughöhen
*/

#pragma once

#include <TaskManager.h>
#include <Arduino.h>
#include <HardwareSerial.h>

#include "..\lib\myLogger.h"
#include "..\lib\putty_out.h"
#include "dictionary.h"
#include "..\lib\model.h"

#define PID_NUM 3

#ifdef _PID_ADJUST

class PID_adjust : public Task::Base
{
private:
	uint8_t _pidCount;
	uint8_t _itemAxis;
	uint8_t _itemCoefficient;
	uint8_t _pidType;
	uint8_t _dotPlaces = 3; ///< Decimal places.
	float _newAddOn = 0.1;	///< Multiplication factor for the PID coefficients, default setting.
	double _addOn;

	typedef enum
	{
		axis_pri = 1,
		axis_sec = 2,
		axis_yaw = 3
	} itemAxis_Number_t; // Basis Nummer der Achse

	typedef enum
	{
		offset_P = 10,
		offset_I = 20,
		offset_D = 30,
		offset_EF = 40
	} itemCoefficient_t; // zu addierende Zahl zur Basis

	typedef enum		// Pid_typ Primary P to YAW ef
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
	} pidTyp_t; // itemAxis_Number_t + itemCoefficient_t eergibt den "pidTyp" für die Funktion select()

	float pri_kP_value = 0.14;		// Initialsation to 0 ??  Stephan
	float pri_kI_value = 0.18;
	float pri_kD_value = 0.102;
	float pri_EF_value = 0;

	float sec_kP_value = 0.14;
	float sec_kI_value = 0.18;
	float sec_kD_value = 0.102;
	float sec_EF_value = 0;

	float yaw_kP_value = 0.01;
	float yaw_kI_value = 0;
	float yaw_kD_value = 0;
	float yaw_EF_value = 0;

	typedef struct
	{
		NewPID *_pid;
		String _name;
	} namedPid_t;

	typedef enum
	{
		kP = 0,
		kI,
		kD
	} pidCoeff_t;

	model_t *_model; 
	HardwareSerial *_serial;
	namedPid_t _newPID[PID_NUM + 1]; // Refactoring zu einer dynamischen Liste mit CPP Templates (Stephan)
	PUTTY_out *_putty_out;
	Dictionary *_dict;

public:
	PID_adjust(const String &name)
		: Task::Base(name)
	{
		_pidCount = 0;
		_newPID[PID_NUM]._pid = nullptr;
	}

	virtual ~PID_adjust()
	{
	}

	PID_adjust *setModel( model_t *model)	// const damit nur gelesen werden kann
    { // Rückgabe wert ist das eigene Objekt (this)
        LOGGER_VERBOSE("Enter....");
        _model = model;
		// _model->interface.isconnect = true;		// darf nicht funktionieren da const und nur gelesen werden soll
        LOGGER_VERBOSE("....leave");
        return this;
    } /* -------------------- end of PID_adjust *setModel -----------------------------*/	

	PID_adjust *setSerial(HardwareSerial *serial)
	{
		_serial = serial;
		_putty_out = new PUTTY_out(*serial);
		return this;
	} /* -------------------- end of PID_adjust *setSerial ----------------------------*/ 

	PID_adjust *addPID(NewPID *pid, String name)
	{
		_newPID[_pidCount]._pid = pid;
		_newPID[_pidCount]._name = name;
		_pidCount++;
		uint8_t i = 0;
		while (_newPID[i]._pid != nullptr)
		{
			LOGGER_WARNING_FMT("PID: %s initialized! %d", _newPID[i]._name.c_str(), _pidCount);
			i++;
		}
		return this;
	} /* -------------------- end of PID_adjust *addPID -------------------------------*/

	virtual void begin() override
	{
		LOGGER_VERBOSE("Enter....");
		_dict = new (Dictionary);
		LOGGER_VERBOSE("....leave");
	} /* -------------------- end of begin --------------------------------------------*/

	virtual void update() override
	{
		LOGGER_VERBOSE("Enter....");

		if (_serial->available() > 0) // Hier werden die gedrückten keys abgefragt
		{
			LOGGER_VERBOSE("Enter....");
			char key = _serial->read();
			switch (key)
			{
			case 'x': ///< Choose the axes
				setItemAxis(itemAxis_Number_t::axis_pri);
				_putty_out->yellow();
				_putty_out->print(ROW_ILLEGAL, COL_ILLEGAL, _dict->c_whitespace);		///< Clears the string "Illegal button was pressed"
				_putty_out->clearPart(ROW_SELECT, COL_SELECT + 5, _dict->c_whitespace); ///< Clears the current line
				_putty_out->print(ROW_SELECT, COL_SELECT + 5, _dict->c_pri_select);		///< Print the selected axis
				break;
			case 'y':
				setItemAxis(itemAxis_Number_t::axis_sec);
				_putty_out->yellow();
				_putty_out->print(ROW_ILLEGAL, COL_ILLEGAL, _dict->c_whitespace);
				_putty_out->clearPart(ROW_SELECT, COL_SELECT + 5, _dict->c_whitespace);
				_putty_out->print(ROW_SELECT, COL_SELECT + 5, _dict->c_sec_select);
				break;
			case 'z':
				setItemAxis(itemAxis_Number_t::axis_yaw);
				_putty_out->yellow();
				_putty_out->print(ROW_ILLEGAL, COL_ILLEGAL, _dict->c_whitespace);
				_putty_out->clearPart(ROW_SELECT, COL_SELECT + 5, _dict->c_whitespace);
				_putty_out->print(ROW_SELECT, COL_SELECT + 5, _dict->c_yaw_select);
				break;

			case 'p': ///< Choose the PID parameter
				setItemCoefficient(itemCoefficient_t::offset_P);
				_putty_out->yellow();
				_putty_out->print(ROW_ILLEGAL, COL_ILLEGAL, _dict->c_whitespace);
				_putty_out->clearPart(ROW_SELECT + 1, COL_SELECT + 10, _dict->c_whitespace);
				_putty_out->print(ROW_SELECT + 1, COL_SELECT + 10, _dict->c_p_select); ///< Print the selected coefficient
				break;

			case 'i':
				setItemCoefficient(itemCoefficient_t::offset_I);
				_putty_out->yellow();
				_putty_out->print(ROW_ILLEGAL, COL_ILLEGAL, _dict->c_whitespace);
				_putty_out->clearPart(ROW_SELECT + 2, COL_SELECT + 10, _dict->c_whitespace);
				_putty_out->print(ROW_SELECT + 2, COL_SELECT + 10, _dict->c_i_select);
				break;

			case 'd':
				setItemCoefficient(itemCoefficient_t::offset_D);
				_putty_out->yellow();
				_putty_out->print(ROW_ILLEGAL, COL_ILLEGAL, _dict->c_whitespace);
				_putty_out->clearPart(ROW_SELECT + 3, COL_SELECT + 10, _dict->c_whitespace);
				_putty_out->print(ROW_SELECT + 3, COL_SELECT + 10, _dict->c_d_select);
				break;

			case 'e':
				setItemCoefficient(itemCoefficient_t::offset_EF);
				_putty_out->yellow();
				_putty_out->print(ROW_ILLEGAL, COL_ILLEGAL, _dict->c_whitespace);
				_putty_out->clearPart(ROW_SELECT + 6, COL_SELECT + 5, _dict->c_whitespace);
				_putty_out->print(ROW_SELECT + 6, COL_SELECT + 5, _dict->c_ef_select);
				break;

			case '+':
				_putty_out->yellow();
				_putty_out->print(ROW_ILLEGAL, COL_ILLEGAL, _dict->c_whitespace);
				coefficient_Up(); ///< Coefficient increment
				break;

			case '-':
				_putty_out->yellow();
				_putty_out->print(ROW_ILLEGAL, COL_ILLEGAL, _dict->c_whitespace);
				coefficient_Down(); ///< Coefficient decrement
				break;

			case '0': ///< Choose the decimal places  0 to 0,001
				setDecimalPlaces(0);
				_putty_out->yellow();
				_putty_out->print(ROW_ILLEGAL, COL_ILLEGAL, _dict->c_whitespace);
				_putty_out->clearPart(ROW_SELECT + ROW_ACCURAY_ADD, COL_SELECT, _dict->c_whitespace);
				_putty_out->print(ROW_SELECT + ROW_ACCURAY_ADD, COL_SELECT, _dict->c_accuracy10);
				break;

			case '1':
				setDecimalPlaces(1);
				_putty_out->yellow();
				_putty_out->print(ROW_ILLEGAL, COL_ILLEGAL, _dict->c_whitespace);
				_putty_out->clearPart(ROW_SELECT + ROW_ACCURAY_ADD, COL_SELECT, _dict->c_whitespace);
				_putty_out->print(ROW_SELECT + ROW_ACCURAY_ADD, COL_SELECT, _dict->c_accuracy01);
				break;

			case '2':
				setDecimalPlaces(2);
				_putty_out->yellow();
				_putty_out->print(ROW_ILLEGAL, COL_ILLEGAL, _dict->c_whitespace);
				_putty_out->clearPart(ROW_SELECT + ROW_ACCURAY_ADD, COL_SELECT, _dict->c_whitespace);
				_putty_out->print(ROW_SELECT + ROW_ACCURAY_ADD, COL_SELECT, _dict->c_accuracy001);
				break;

			case '3':
				setDecimalPlaces(3);
				_putty_out->yellow();
				_putty_out->print(ROW_ILLEGAL, COL_ILLEGAL, _dict->c_whitespace);
				_putty_out->clearPart(ROW_ILLEGAL + ROW_ACCURAY_ADD, COL_SELECT, _dict->c_whitespace);
				_putty_out->print(ROW_SELECT + ROW_ACCURAY_ADD, COL_SELECT, _dict->c_accuracy0001);
				break;

			case 's': ///< Saved all coefficients into the EEPROM
				// myPID_pri.updateEEPROM();
				// myPID_sec.updateEEPROM();
				// myPID_yaw.updateEEPROM();
				_putty_out->red();
				_putty_out->print(45, 8, "PID data was backed up");
				_putty_out->yellow();
				displayPIDcoefficients();
				break;

			case 'r': ///< Reads all coefficients from the EEPROM
				// myPID_pri.readEEPROM();
				// myPID_sec.readEEPROM();
				// myPID_yaw.readEEPROM();
				_putty_out->red();
				_putty_out->print(45, 8, "PID data was read out");
				_putty_out->yellow();
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
				LOGGER_NOTICE(" Altitude is not implemented");
				break;

			case 'n':
				LOGGER_NOTICE(" Near ground is not implemented");
				break;

			default:
			{
				_putty_out->red();
				_putty_out->print(ROW_ILLEGAL, COL_ILLEGAL, "Illegal button was pressed");
				_putty_out->yellow();
			}
			} /* end of switch(key) */
			LOGGER_VERBOSE("....leave");
		} /* end of _serial.available */
		LOGGER_VERBOSE("....leave");
	} /* -------------------- end of update -------------------------------------------*/

	void setItemAxis(uint8_t itemAxis)
	{
		/* Selects the axis, according to the keyboard input.
		 * Key X = primary axis (1)
		 * Key Y = primary axis (2)
		 * Key Z = primary axis (3)	 */
		_itemAxis = itemAxis;
		LOGGER_NOTICE_FMT("itemAxis = %d", _itemAxis);
	} /*----------------------------- end of setItemAxis ------------------------------*/

	void setItemCoefficient(uint8_t itemCoefficient)
	{
		/* Selects the coefficient, according to the keyboard input.
		 * Key P = coefficient P (10)
		 * Key I = coefficient I (20)
		 * Key D = coefficient D (30)
		 * Key E = ExecutingFrequency (40)	 */
		_itemCoefficient = itemCoefficient;
		LOGGER_NOTICE_FMT("itemCoefficient = %d", _itemCoefficient);
	} /*----------------------------- end of setItemCoefficient -----------------------*/

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

	/* Here you sets the real coefficient into the model, and print it into the GUI. +/- _addOn
	   like this: Pri. P = 2.20
	*/
	void select(uint8_t type) // put the coefficient into the right PID type
	{
		LOGGER_NOTICE_FMT("Select %d", type);

		switch (type)
		{

		case pidTyp_t::pri_P:
			pri_kP_value += _addOn;
			if (checkValue(pri_kP_value))
			{
			LOGGER_WARNING_FMT("X Axis kP = %f", pri_kP_value);
				_putty_out->print(ROW_SELECT + 1, COL_SELECT + 26, _dotPlaces, pri_kP_value);
				displayPIDcoefficients();
				_newPID[axisName_t::primary]._pid->setP(pri_kP_value);  
				_model->pidData_TEST[axisName_t::primary].kD = pri_kP_value;
			LOGGER_WARNING("X Axis kP.... leave");
			}
			break;
		case pidTyp_t::pri_I:
			pri_kI_value += _addOn;
			if (checkValue(pri_kI_value))
			{
				LOGGER_NOTICE_FMT("X Axis kI = %f", pri_kI_value);
				_putty_out->print(ROW_SELECT + 2, COL_SELECT + 26, _dotPlaces, pri_kI_value);
				displayPIDcoefficients();
				_newPID[axisName_t::primary]._pid->setI(pri_kI_value);
			}
			break;
		case pidTyp_t::pri_D:
			pri_kD_value += _addOn;
			if (checkValue(pri_kD_value))
			{
				LOGGER_NOTICE_FMT("X Axis kD = %f", pri_kD_value);
				_putty_out->print(ROW_SELECT + 3, COL_SELECT + 26, _dotPlaces, pri_kD_value);
				displayPIDcoefficients();
				_newPID[axisName_t::primary]._pid->setD(pri_kD_value);
			}
			break;

		case pidTyp_t::sec_P:
			sec_kP_value += _addOn;
			if (checkValue(sec_kP_value))
			{
				LOGGER_NOTICE_FMT("Y Axis kP = %f", sec_kP_value);
				_putty_out->print(ROW_SELECT + 1, COL_SELECT + 26, _dotPlaces, sec_kP_value);
				displayPIDcoefficients();
				_newPID[axisName_t::secondary]._pid->setP(sec_kP_value);
			}
			break;

		case pidTyp_t::sec_I:
			sec_kI_value += _addOn;
			if (checkValue(sec_kI_value))
			{
				LOGGER_NOTICE_FMT("Y Axis kI = %f", sec_kI_value);
				_putty_out->print(ROW_SELECT + 2, COL_SELECT + 26, _dotPlaces, sec_kI_value);
				displayPIDcoefficients();
				_newPID[axisName_t::secondary]._pid->setI(sec_kI_value);
			}
			break;

		case pidTyp_t::sec_D:
			sec_kD_value += _addOn;
			if (checkValue(sec_kD_value))
			{
				LOGGER_NOTICE_FMT("Y Axis kD = %f", sec_kD_value);
				_putty_out->print(ROW_SELECT + 3, COL_SELECT + 26, _dotPlaces, sec_kD_value);
				displayPIDcoefficients();
				_newPID[axisName_t::secondary]._pid->setD(sec_kD_value);
			}
			break;

		case pidTyp_t::yaw_P:
			yaw_kP_value += _addOn;
			if (checkValue(yaw_kP_value))
			{
				LOGGER_NOTICE_FMT("Z Axis kP = %f", yaw_kP_value);
				_putty_out->print(ROW_SELECT + 1, COL_SELECT + 26, _dotPlaces, yaw_kP_value);
				displayPIDcoefficients();
				_newPID[axisName_t::yaw]._pid->setP(yaw_kP_value);
			}
			break;

		case pidTyp_t::yaw_I:
			yaw_kI_value += _addOn;
			if (checkValue(yaw_kI_value))
			{
				LOGGER_NOTICE_FMT("Z Axis kI = %f", yaw_kI_value);
				_putty_out->print(ROW_SELECT + 2, COL_SELECT + 26, _dotPlaces, yaw_kI_value);
				displayPIDcoefficients();
				_newPID[axisName_t::yaw]._pid->setI(yaw_kI_value);
			}
			break;

		case pidTyp_t::yaw_D:
			yaw_kD_value += _addOn;
			if (checkValue(yaw_kD_value))
			{
				LOGGER_NOTICE_FMT("Z Axis kD = %f", yaw_kD_value);
				_putty_out->print(ROW_SELECT + 3, COL_SELECT + 26, _dotPlaces, yaw_kD_value);
				displayPIDcoefficients();
				_newPID[axisName_t::yaw]._pid->setD(yaw_kD_value);
			}
			break;

		case pidTyp_t::pri_ef:
			pri_EF_value += _addOn;
			if (checkValue(pri_EF_value))
			{
				LOGGER_NOTICE_FMT("X Axis eF = %f", pri_EF_value);
				_putty_out->print(ROW_SELECT + 6, COL_SELECT + 32, _dotPlaces, pri_EF_value);
				displayPIDcoefficients();
				_newPID[axisName_t::primary]._pid->setExecutionFrequency(pri_EF_value);
			}
			break;

		case pidTyp_t::sec_ef:
			sec_EF_value += _addOn;
			if (checkValue(sec_EF_value))
			{
				LOGGER_NOTICE_FMT("Y Axis eF = %f", sec_EF_value);
				_putty_out->print(ROW_SELECT + 6, COL_SELECT + 32, _dotPlaces, sec_EF_value);
				displayPIDcoefficients();
				_newPID[axisName_t::secondary]._pid->setExecutionFrequency(sec_EF_value);
			}
			break;
		case pidTyp_t::yaw_ef:
			yaw_EF_value += _addOn;
			if (checkValue(yaw_EF_value))
			{
				LOGGER_NOTICE_FMT("Z Axis eF = %f", yaw_EF_value);
				_putty_out->print(ROW_SELECT + 6, COL_SELECT + 32, _dotPlaces, yaw_EF_value);
				displayPIDcoefficients();
				_newPID[axisName_t::yaw]._pid->setExecutionFrequency(yaw_EF_value);
			}
			break;
		} /* end of switch */
	}	  /*----------------------------- end of select -------------------------------*/

		void display_Menu()
	{
		LOGGER_VERBOSE("Enter....");
		_putty_out->clear();
		_putty_out->clear();
		_putty_out->gray();
		_putty_out->print(ROW_MENU, COL_MENU, "-----------Menu for PID configuration (BT)-----------");
		_putty_out->yellow();
		_putty_out->print(ROW_MENU + 2, COL_MENU, "(X) choose the primary");
		_putty_out->print(ROW_MENU + 3, COL_MENU, "(Y)           secondary");
		_putty_out->print(ROW_MENU + 4, COL_MENU, "(Z)            YAW axis");
		_putty_out->print(ROW_MENU + 5, COL_MENU, " P, I or D select the coefficient");
		_putty_out->print(ROW_MENU + 6, COL_MENU, "(0),(1),(2)or(3)select the accurarcy");
		_putty_out->print(ROW_MENU + 7, COL_MENU, "(E) choose the execution frequency");
		_putty_out->print(ROW_MENU + 8, COL_MENU, "(+) increment according to the value");
		_putty_out->print(ROW_MENU + 9, COL_MENU, "(-) decrement      ''");
		_putty_out->print(ROW_MENU + 10, COL_MENU, "(S) saves all coefficient into the EEPROM");
		_putty_out->print(ROW_MENU + 11, COL_MENU, "(R) reads all coefficients from the EEPROM");
		_putty_out->print(ROW_MENU + 12, COL_MENU, "(C) Copies the primary values to the secondary axis");
		_putty_out->print(ROW_MENU + 13, COL_MENU, "(A) all values are set to 0 in the EEPROM.");
		_putty_out->print(ROW_MENU + 14, COL_MENU, "(G) get factory defaults");
		_putty_out->print(ROW_MENU + 15, COL_MENU, "(H) set the maximal altitude");
		_putty_out->print(ROW_MENU + 16, COL_MENU, "(N) set the maximal near field altitude");
		_putty_out->print(ROW_MENU + 17, COL_MENU, "(M) display the menu");
		_putty_out->gray();
		_putty_out->print(ROW_MENU + 19, COL_MENU, "-----------------------------------------------------");
		_putty_out->yellow();
		_putty_out->print(ROW_SELECT + ROW_ACCURAY_ADD, COL_SELECT, "Accuracy = 1,0");
		LOGGER_VERBOSE("....leave");
	} /*-------------------------- end of display_Menu --------------------------------*/

	void displayPIDcoefficients() ///  only Template for the positions
	{
		_putty_out->blue();
		_putty_out->print(ROW_OUTPUT, COL_OUTPUT, "Temp PID coefficients");
		_putty_out->print(ROW_OUTPUT + 1, COL_OUTPUT, "   in the EEPROM");
		_putty_out->gray();
		_putty_out->print(ROW_OUTPUT + 3, COL_OUTPUT, _dict->c_primary_p);
		_putty_out->print(ROW_OUTPUT + 3, COL_OUTPUT_VALUE, 3, pri_kP_value);
		_putty_out->print(ROW_OUTPUT + 4, COL_OUTPUT, _dict->c_primary_i);
		_putty_out->print(ROW_OUTPUT + 4, COL_OUTPUT_VALUE, 3, pri_kI_value);
		_putty_out->print(ROW_OUTPUT + 5, COL_OUTPUT, _dict->c_primary_d);
		_putty_out->print(ROW_OUTPUT + 5, COL_OUTPUT_VALUE, 3, pri_kD_value);
		_putty_out->print(ROW_OUTPUT + 7, COL_OUTPUT, _dict->c_secondary_p);
		_putty_out->print(ROW_OUTPUT + 7, COL_OUTPUT_VALUE, 3, sec_kP_value);
		_putty_out->print(ROW_OUTPUT + 8, COL_OUTPUT, _dict->c_secondary_i);
		_putty_out->print(ROW_OUTPUT + 8, COL_OUTPUT_VALUE, 3, sec_kI_value);
		_putty_out->print(ROW_OUTPUT + 9, COL_OUTPUT, _dict->c_secondary_d);
		_putty_out->print(ROW_OUTPUT + 9, COL_OUTPUT_VALUE, 3, sec_kD_value);
		_putty_out->print(ROW_OUTPUT + 11, COL_OUTPUT, _dict->c_yaw_p);
		_putty_out->print(ROW_OUTPUT + 11, COL_OUTPUT_VALUE, 3, yaw_kP_value);
		_putty_out->print(ROW_OUTPUT + 12, COL_OUTPUT, _dict->c_yaw_i);
		_putty_out->print(ROW_OUTPUT + 12, COL_OUTPUT_VALUE, 3, yaw_kI_value);
		_putty_out->print(ROW_OUTPUT + 13, COL_OUTPUT, _dict->c_yaw_d);
		_putty_out->print(ROW_OUTPUT + 13, COL_OUTPUT_VALUE, 3, yaw_kD_value);
		_putty_out->print(ROW_OUTPUT + 15, COL_OUTPUT, _dict->c_ef_pri);
		_putty_out->print(ROW_OUTPUT + 15, COL_OUTPUT_VALUE, 1, pri_EF_value);
		_putty_out->print(ROW_OUTPUT + 16, COL_OUTPUT, _dict->c_ef_sec);
		_putty_out->print(ROW_OUTPUT + 16, COL_OUTPUT_VALUE, 1, sec_EF_value);
		_putty_out->print(ROW_OUTPUT + 17, COL_OUTPUT, _dict->c_ef_yaw);
		_putty_out->print(ROW_OUTPUT + 17, COL_OUTPUT_VALUE, 1, yaw_EF_value);

		// _putty_out->print(ROW_PID+2, COL_PID+16, c_pri_i);_putty_out->print(ROW_PID+2, COL_PID+20, 3, _model.pidData[axis_t::Primary].pidCoefficient[pidCoeff_t::I]);
		// _putty_out->print(ROW_PID+2, COL_PID+27, c_pri_d);_putty_out->print(ROW_PID+2, COL_PID+31, 3, _model.pidData[axis_t::Primary].pidCoefficient[pidCoeff_t::D]);
		// _putty_out->print(ROW_PID+2, COL_PID+40, c_ef);   _putty_out->print(ROW_PID+2, COL_PID+52, 0, _model.pidData[axis_t::Primary].executionFrequency);

		// _putty_out->print(ROW_PID+3, COL_PID,    c_sec_p);_putty_out->print(ROW_PID+3, COL_PID+9,  2, _model.pidData[axis_t::Secondary].pidCoefficient[pidCoeff_t::P]);
		// 										  _putty_out->print(ROW_PID+3, COL_PID+20, 3, _model.pidData[axis_t::Secondary].pidCoefficient[pidCoeff_t::I]);
		// 										  _putty_out->print(ROW_PID+3, COL_PID+31, 3, _model.pidData[axis_t::Secondary].pidCoefficient[pidCoeff_t::D]);
		// 										  _putty_out->print(ROW_PID+3, COL_PID+52, 0, _model.pidData[axis_t::Secondary].executionFrequency);

		// _putty_out->print(ROW_PID+4, COL_PID,    c_yaw_p);_putty_out->print(ROW_PID+4, COL_PID+9,  2, _model.pidData[axis_t::YawAxis].pidCoefficient[P]);
		// 										  _putty_out->print(ROW_PID+4, COL_PID+20, 3, _model.pidData[axis_t::YawAxis].pidCoefficient[pidCoeff_t::I]);
		// 										  _putty_out->print(ROW_PID+4, COL_PID+31, 3, _model.pidData[axis_t::YawAxis].pidCoefficient[pidCoeff_t::D]);
		// 										  _putty_out->print(ROW_PID+4, COL_PID+52, 0, _model.pidData[axis_t::YawAxis].executionFrequency);

	} /*--------------------- end of displayPIDcoefficients ---------------------------*/

	bool checkValue(float a) // It should be ensured that no negative values are passed.
	{
		if (a >= 0)
			return true;
		else
			return false;
	} /*----------------------------- end of checkValue -------------------------------*/

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
		LOGGER_NOTICE_FMT("New Factor = %f", _newAddOn);
	} /*----------------------------- end of setDecimalPlaces -------------------------*/

};		  /*------------------------- end of PID_adjust class -------------------------*/

//#undef _DEBUG_

#endif