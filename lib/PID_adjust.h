/*  File name : PID_Calbration.h
	Project name : KuckyCopter 2
	Author: Stephan Scholz / Wilhelm Kuckelsberg
	Date : 2022-07-02
	Description : Einstellen de PID Regler via Bluetooth
				  und die maximalen Flughöhen
*/

#pragma once

#include <TaskManager.h>
#include <Arduino.h>
#include <HardwareSerial.h>

// #define LOCAL_DEBUG
#include "..\lib\myLogger.h"

#include "..\lib\putty_out.h"
#include "dictionary.h"
#include "..\lib\model.h"

#ifdef _PID_ADJUST

#define PID_NUM 3

#define ROW_MENU 3 ///< First position for the main menue
#define COL_MENU 10
#define ROW_SELECT ROW_MENU + 24 ///< First position for select PID type
#define COL_SELECT 5
#define ROW_COEFF 28 ///< First position for new coefficients
#define COL_COEFF 20
#define ROW_PID 33 ///< First position for current coefficients
#define COL_PID 20
#define ROW_OUTPUT ROW_MENU + 22
#define COL_OUTPUT 50
#define COL_OUTPUT_VALUE 69
#define ROW_ACCURAY ROW_MENU + 17
#define ROW_STATE ROW_MENU + 46 // Position for state message
#define COL_STATE COL_MENU + 16

class PID_adjust : public Task::Base
{
private:
	uint8_t _pidCount;
	uint8_t _itemAxis;
	uint8_t _itemCoefficient;
	uint8_t _pidType;
	uint8_t _dotPlaces = 3; ///< Decimal places.
	float _newAddOn = 0.01; ///< Multiplication factor for the PID coefficients, default setting.
	double _addOn;
	uint8_t _maxPID;

	typedef enum
	{
		axis_pri = 1,
		axis_sec = 2,
		axis_yaw = 3
	} itemAxis_Number_t; // Base number of the axis

	typedef enum
	{
		offset_P = 10,
		offset_I = 20,
		offset_D = 30,
		offset_EF = 40
	} itemCoefficient_t; // number to be added to the base

	typedef enum // Pid_typ Primary P to YAW ef
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
	} pidTyp_t;						/* itemAxis_Number_t + itemCoefficient_t returns
									   the "pidType" for the select() function
									*/
	float pri_kP_value = PID_P_MIN; // Start values for menu control
	float pri_kI_value = 0.0;
	float pri_kD_value = 0.0;
	float pri_EF_value = 0.0;

	float sec_kP_value = PID_P_MIN;
	float sec_kI_value = 0.0;
	float sec_kD_value = 0.0;
	float sec_EF_value = 0.0;

	float yaw_kP_value = PID_P_MIN;
	float yaw_kI_value = 0.0;
	float yaw_kD_value = 0.0;
	float yaw_EF_value = 0.0;

	typedef struct
	{
		NewPID *_pid;
		String _name;
	} namedPid_t;

	model_t *_model;
	HardwareSerial *_serial;
	namedPid_t _namedPID[PID_NUM]; // Refactoring zu einer dynamischen Liste mit CPP Templates (Stephan)
	PUTTY_out *_putty_out;
	Dictionary *_dict;

public:
	PID_adjust(const String &name)
		: Task::Base(name)
	{
		_pidCount = 0;
		_maxPID = 0;
	}

	virtual ~PID_adjust() {}

	PID_adjust *setModel(model_t *model)
	{

		LOGGER_VERBOSE("Enter....");
		_model = model;
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
		LOGGER_NOTICE("Enter ...");
		_namedPID[_pidCount]._pid = pid;
		_namedPID[_pidCount]._name = name;
		for (uint8_t i = 0; i < _pidCount + 1; i++)
		{
			LOGGER_NOTICE_FMT("PID: %s initialized! %d", _namedPID[i]._name.c_str(), i + 1);
		}
		_pidCount++;
		LOGGER_NOTICE("... leave");
		return this;
	} /* -------------------- end of PID_adjust *addPID -----------------------------------------*/

	virtual void begin() override
	{
		LOGGER_VERBOSE("Enter....");
		_dict = new (Dictionary);
		LOGGER_VERBOSE("....leave");
	} /* -------------------- end of begin ------------------------------------------------------*/

	virtual void update() override
	{
		LOGGER_VERBOSE("Enter....");

		if (_serial->available() > 0)
		{
			char key = _serial->read();
			switch (toupper(key))
			{
			case 'X': ///< Choose the axes
				setItemAxis(itemAxis_Number_t::axis_pri);
				_putty_out->yellow();
				clearStateLine();
				_putty_out->clearPart(ROW_MENU + 21, COL_SELECT + 5, _dict->c_whitespace);	///< Clears the current line
				_putty_out->print(ROW_MENU + 21, COL_SELECT + 5, _dict->c_axis_pri_select); ///< Print the selected axis
				break;
			case 'Y':
				setItemAxis(itemAxis_Number_t::axis_sec);
				_putty_out->yellow();
				clearStateLine();
				_putty_out->clearPart(ROW_MENU + 26, COL_SELECT + 5, _dict->c_whitespace);
				_putty_out->print(ROW_MENU + 26, COL_SELECT + 5, _dict->c_axis_sec_select);
				break;
			case 'Z':
				setItemAxis(itemAxis_Number_t::axis_yaw);
				_putty_out->yellow();
				clearStateLine();
				_putty_out->clearPart(ROW_MENU + 31, COL_SELECT + 5, _dict->c_whitespace);
				_putty_out->print(ROW_MENU + 31, COL_SELECT + 5, _dict->c_axis_yaw_select);
				break;

			case 'P': ///< Choose the PID parameter
				setItemCoefficient(itemCoefficient_t::offset_P);
				_putty_out->yellow();
				clearStateLine();
				_putty_out->clearPart(ROW_MENU + 22 + ((_itemAxis - 1) * 5), COL_SELECT + 10, _dict->c_whitespace);
				_putty_out->print(ROW_MENU + 22 + ((_itemAxis - 1) * 5), COL_SELECT + 10, _dict->c_p_coeff); ///< Print the selected coefficient
				break;

			case 'I':
				setItemCoefficient(itemCoefficient_t::offset_I);
				_putty_out->yellow();
				clearStateLine();
				_putty_out->clearPart(ROW_MENU + 23 + ((_itemAxis - 1) * 5), COL_SELECT + 10, _dict->c_whitespace);
				_putty_out->print(ROW_MENU + 23 + ((_itemAxis - 1) * 5), COL_SELECT + 10, _dict->c_i_coeff);
				break;

			case 'D':
				setItemCoefficient(itemCoefficient_t::offset_D);
				_putty_out->yellow();
				clearStateLine();
				_putty_out->clearPart(ROW_MENU + 24 + ((_itemAxis - 1) * 5), COL_SELECT + 10, _dict->c_whitespace);
				_putty_out->print(ROW_MENU + 24 + ((_itemAxis - 1) * 5), COL_SELECT + 10, _dict->c_d_coeff);
				break;

			case 'E':
				setItemCoefficient(itemCoefficient_t::offset_EF);
				_putty_out->yellow();
				clearStateLine();
				_putty_out->clearPart(ROW_MENU + 25 + ((_itemAxis - 1) * 5), COL_SELECT + 10, _dict->c_whitespace);
				_putty_out->print(ROW_MENU + 25 + ((_itemAxis - 1) * 5), COL_SELECT + 10, _dict->c_ef_coeff);
				break;

			case '+':
				_putty_out->yellow();
				clearStateLine();
				coefficient_Up(); ///< Coefficient increment
				break;

			case '-':
				_putty_out->yellow();
				clearStateLine();
				coefficient_Down(); ///< Coefficient decrement
				break;

			case '0': ///< Choose the decimal places  0 to 0,001
				setDecimalPlaces(0);
				clearStateLine();
				_putty_out->cyan();
				_putty_out->print(ROW_MENU + 7, COL_MENU + 47, 3, _newAddOn);
				break;

			case '1':
				setDecimalPlaces(1);
				clearStateLine();
				_putty_out->cyan();
				_putty_out->print(ROW_MENU + 7, COL_MENU + 47, 3, _newAddOn);
				break;

			case '2':
				setDecimalPlaces(2);
				clearStateLine();
				_putty_out->cyan();
				_putty_out->print(ROW_MENU + 7, COL_MENU + 47, 3, _newAddOn);
				break;

			case '3':
				setDecimalPlaces(3);
				_putty_out->cyan();
				_putty_out->print(ROW_MENU + 7, COL_MENU + 47, 3, _newAddOn);
				break;
			case '5':
				setDecimalPlaces(5);
				clearStateLine();
				_putty_out->cyan();
				_putty_out->print(ROW_MENU + 7, COL_MENU + 47, 3, _newAddOn);
				break;

			case 'S': ///< Saved all coefficients into the EEPROM
				for (uint8_t i = 0; i < 3; i++)
				{
					_namedPID[i]._pid->saveParameters();
				}

				_putty_out->red();
				_putty_out->print(ROW_MENU + 9, COL_MENU + 47, "Done");
				_putty_out->print(ROW_MENU + 10, COL_MENU + 47, "    ");
				displayPIDcoefficients();
				break;

			case 'R': ///< Reads all coefficients from the EEPROM
				for (uint8_t i = 0; i < 3; i++)
				{
					_namedPID[i]._pid->loadParameters();
				}

				_putty_out->red();
				_putty_out->print(ROW_MENU + 10, COL_MENU + 47, "Done");
				_putty_out->print(ROW_MENU + 9, COL_MENU + 47, "    ");
				displayPIDcoefficients();
				break;

			case 'A': ///< Set all PID parameters to 0
				_namedPID[axisName::primary]._pid->setP(PID_P_MIN);
				_namedPID[axisName::primary]._pid->setI(0);
				_namedPID[axisName::primary]._pid->setD(0);
				_namedPID[axisName::primary]._pid->setEF(50);
				_namedPID[axisName::secondary]._pid->setP(PID_P_MIN);
				_namedPID[axisName::secondary]._pid->setI(0);
				_namedPID[axisName::secondary]._pid->setD(0);
				_namedPID[axisName::secondary]._pid->setEF(50);
				_namedPID[axisName::yaw]._pid->setP(PID_P_MIN);
				_namedPID[axisName::yaw]._pid->setI(0);
				_namedPID[axisName::yaw]._pid->setD(0);
				_namedPID[axisName::yaw]._pid->setEF(50);
				displayPIDcoefficients();
				break;

			case 'G': ///< get factory default
				_namedPID[axisName::primary]._pid->setP(0.14);
				_namedPID[axisName::primary]._pid->setI(0.18);
				_namedPID[axisName::primary]._pid->setD(0.102);
				_namedPID[axisName::primary]._pid->setEF(50);
				_namedPID[axisName::secondary]._pid->setP(0.14);
				_namedPID[axisName::secondary]._pid->setI(0.18);
				_namedPID[axisName::secondary]._pid->setD(0.102);
				_namedPID[axisName::secondary]._pid->setEF(50);
				_namedPID[axisName::yaw]._pid->setP(0.01);
				_namedPID[axisName::yaw]._pid->setI(0);
				_namedPID[axisName::yaw]._pid->setD(0);
				_namedPID[axisName::yaw]._pid->setEF(50);
				displayPIDcoefficients();
				break;

			case 'C': ///< Copies the primary values to the secondary axis
				_putty_out->red();
				_putty_out->print(ROW_STATE, COL_STATE, " 'C' is not implemented");
				_putty_out->yellow();
				break;

			case 'M':
				display_Menu();
				displayPIDcoefficients();
				break;

			case 'H':
				_putty_out->yellow();
				_putty_out->print(ROW_MENU + 38, COL_MENU + 10, "max. altitude via baro      m");
				_putty_out->cyan();
				_putty_out->print(ROW_MENU + 38, COL_MENU + 33, "1000");
				break;

			case 'N':
				_putty_out->yellow();
				_putty_out->print(ROW_MENU + 40, COL_MENU + 10, "max. altitude via down sonic     cm");
				_putty_out->cyan();
				_putty_out->print(ROW_MENU + 40, COL_MENU + 39, "400");
				break;

			case 'F':
				_putty_out->yellow();
				_putty_out->print(ROW_MENU + 42, COL_MENU + 10, "max. distance to horizontal object     cm");
				_putty_out->cyan();
				_putty_out->print(ROW_MENU + 42, COL_MENU + 45, "400");
				break;

			default:
			{
				_putty_out->red();
				_putty_out->print(ROW_STATE, COL_STATE, "Illegal button was pressed");
				_putty_out->yellow();
			}
			} /* end of switch(key) */
		}	  /* end of _serial.available */
		LOGGER_VERBOSE("....leave");
	} /* -------------------- end of update ----------------------------------------------------*/

	void clearStateLine()
	{
		_putty_out->print(ROW_MENU + 46, COL_STATE, _dict->c_whitespace);
	} /* -------------------- end of clearStateLine ----------------------------------------------------*/
	/* Clears the string "Illegal button was pressed" */
	void setItemAxis(uint8_t itemAxis)
	{
		/* Selects the axis, according to the keyboard input.
		 * Key X = primary axis (1)
		 * Key Y = primary axis (2)
		 * Key Z = primary axis (3)	 */
		_itemAxis = itemAxis;
		LOGGER_NOTICE_FMT("itemAxis = %d", _itemAxis);
	} /*----------------------------- end of setItemAxis ---------------------------------------*/

	void setItemCoefficient(uint8_t itemCoefficient)
	{
		/* Selects the coefficient, according to the keyboard input.
		 * Key P = coefficient P (10)
		 * Key I = coefficient I (20)
		 * Key D = coefficient D (30)
		 * Key E = ExecutingFrequency (40)	 */
		_itemCoefficient = itemCoefficient;
		LOGGER_NOTICE_FMT("itemCoefficient = %d", _itemCoefficient);
	} /*----------------------------- end of setItemCoefficient --------------------------------*/

	/* Set the "PID Type",
	 * e.g _itemAxis = 1 and _itemCoefficient = 20 ~ _pidType 21
	 * Will say, it select the parameter for secondary axis and coefficient 'i'
	 */
	uint8_t getPidType(bool up)
	{ /// const deleted

		_pidType = _itemAxis + _itemCoefficient;
		//	LOGGER_NOTICE_FMT("PID Type = %d", _pidType);

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
	} /*----------------------------- end of getPidType ----------------------------------------*/

	/* Increment the coefficient according the PID Type. Shown "getPidType". */
	void coefficient_Up()
	{
		select(getPidType(true));
	} /*----------------------------- end of coefficient_Up ------------------------------------*/

	/* decrement the coefficient according the PID Type. Shown "getPidType". */
	void coefficient_Down()
	{
		select(getPidType(false));
	} /*----------------------------- end of coefficient_Down ----------------------------------*/

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
				LOGGER_NOTICE_FMT("X Axis kP = %f", pri_kP_value);
				_putty_out->cyan();
				_putty_out->print(ROW_MENU + 22, COL_SELECT + 27, _dotPlaces, pri_kP_value);
				_namedPID[axisName::primary]._pid->setP(pri_kP_value);
				displayPIDcoefficients();
			}
			break;

		case pidTyp_t::pri_I:
			pri_kI_value += _addOn;
			if (checkValue(pri_kI_value))
			{
				LOGGER_NOTICE_FMT("X Axis kI = %f", pri_kI_value);
				_putty_out->cyan();
				_putty_out->print(ROW_MENU + 23, COL_SELECT + 27, _dotPlaces, pri_kI_value);
				_namedPID[axisName::primary]._pid->setI(pri_kI_value);
				displayPIDcoefficients();
			}
			break;

		case pidTyp_t::pri_D:
			pri_kD_value += _addOn;
			if (checkValue(pri_kD_value))
			{
				LOGGER_NOTICE_FMT("X Axis kD = %f", pri_kD_value);
				_putty_out->cyan();
				_putty_out->print(ROW_MENU + 24, COL_SELECT + 27, _dotPlaces, pri_kD_value);
				_namedPID[axisName::primary]._pid->setD(pri_kD_value);
				displayPIDcoefficients();
			}
			break;

		case pidTyp_t::pri_ef:
			pri_EF_value += _addOn;
			if (checkValue(pri_EF_value))
			{
				LOGGER_WARNING_FMT("X Axis eF = %f", pri_EF_value);
				_putty_out->cyan();
				_putty_out->print(ROW_MENU + 25, COL_SELECT + 27, _dotPlaces, pri_EF_value);
				_namedPID[axisName::primary]._pid->setEF(pri_EF_value);
				displayPIDcoefficients();
			}
			break;

		case pidTyp_t::sec_P:
			sec_kP_value += _addOn;
			if (checkValue(sec_kP_value))
			{
				LOGGER_NOTICE_FMT("Y Axis kP = %f", sec_kP_value);
				_putty_out->cyan();
				_putty_out->print(ROW_MENU + 22 + ((_itemAxis - 1) * 5), COL_SELECT + 27, _dotPlaces, sec_kP_value);
				_namedPID[axisName::secondary]._pid->setP(sec_kP_value);
				displayPIDcoefficients();
			}
			break;

		case pidTyp_t::sec_I:
			sec_kI_value += _addOn;
			if (checkValue(sec_kI_value))
			{
				LOGGER_NOTICE_FMT("Y Axis kI = %f", sec_kI_value);
				_putty_out->cyan();
				_putty_out->print(ROW_MENU + 23 + ((_itemAxis - 1) * 5), COL_SELECT + 27, _dotPlaces, sec_kI_value);
				_namedPID[axisName::secondary]._pid->setI(sec_kI_value);
				displayPIDcoefficients();
			}
			break;

		case pidTyp_t::sec_D:
			sec_kD_value += _addOn;
			if (checkValue(sec_kD_value))
			{
				LOGGER_NOTICE_FMT("Y Axis kD = %f", sec_kD_value);
				_putty_out->cyan();
				_putty_out->print(ROW_MENU + 24 + ((_itemAxis - 1) * 5), COL_SELECT + 27, _dotPlaces, sec_kD_value);
				_namedPID[axisName::secondary]._pid->setD(sec_kD_value);
				displayPIDcoefficients();
			}
			break;

		case pidTyp_t::sec_ef:
			sec_EF_value += _addOn;
			if (checkValue(sec_EF_value))
			{
				LOGGER_WARNING_FMT("Y Axis eF = %f", sec_EF_value);
				_putty_out->cyan();
				_putty_out->print(ROW_MENU + 25 + ((_itemAxis - 1) * 5), COL_SELECT + 27, _dotPlaces, sec_EF_value);
				_namedPID[axisName::secondary]._pid->setEF(sec_EF_value);
				displayPIDcoefficients();
			}
			break;

		case pidTyp_t::yaw_P:
			yaw_kP_value += _addOn;
			if (checkValue(yaw_kP_value))
			{
				LOGGER_NOTICE_FMT("Z Axis kP = %f", yaw_kP_value);
				_putty_out->cyan();
				_putty_out->print(ROW_MENU + 22 + ((_itemAxis - 1) * 5), COL_SELECT + 27, _dotPlaces, yaw_kP_value);
				_namedPID[axisName::yaw]._pid->setP(yaw_kP_value);
				displayPIDcoefficients();
			}
			break;

		case pidTyp_t::yaw_I:
			yaw_kI_value += _addOn;
			if (checkValue(yaw_kI_value))
			{
				LOGGER_NOTICE_FMT("Z Axis kI = %f", yaw_kI_value);
				_putty_out->cyan();
				_putty_out->print(ROW_MENU + 23 + ((_itemAxis - 1) * 5), COL_SELECT + 27, _dotPlaces, yaw_kI_value);
				_namedPID[axisName::yaw]._pid->setI(yaw_kI_value);
				displayPIDcoefficients();
			}
			break;

		case pidTyp_t::yaw_D:
			yaw_kD_value += _addOn;
			if (checkValue(yaw_kD_value))
			{
				LOGGER_NOTICE_FMT("Z Axis kD = %f", yaw_kD_value);
				_putty_out->cyan();
				_putty_out->print(ROW_MENU + 24 + ((_itemAxis - 1) * 5), COL_SELECT + 27, _dotPlaces, yaw_kD_value);
				_namedPID[axisName::yaw]._pid->setD(yaw_kD_value);
				displayPIDcoefficients();
			}
			break;

		case pidTyp_t::yaw_ef:
			yaw_EF_value += _addOn;
			if (checkValue(yaw_EF_value))
			{
				LOGGER_WARNING_FMT("Z Axis eF = %f", yaw_EF_value);
				_putty_out->cyan();
				_putty_out->print(ROW_MENU + 25 + ((_itemAxis - 1) * 5), COL_SELECT + 27, _dotPlaces, yaw_EF_value);
				_namedPID[axisName::yaw]._pid->setEF(yaw_EF_value);
				displayPIDcoefficients();
			}
			break;
		} /* end of switch */
	}	  /*----------------------------- end of select ----------------------------------------*/

	void display_Menu()
	{
		uint8_t row_add = -1;
		LOGGER_VERBOSE("Enter....");
		_putty_out->clear();
		_putty_out->gray();
		_putty_out->print(ROW_MENU + (row_add += 1), COL_MENU, "-------- Online configurater for KuckyCopter (BT) --------");
		_putty_out->yellow();
		_putty_out->print(ROW_MENU + (row_add += 2), COL_MENU, "(X) choose the primary");
		_putty_out->print(ROW_MENU + (row_add += 1), COL_MENU, "(Y)           secondary");
		_putty_out->print(ROW_MENU + (row_add += 1), COL_MENU, "(Z)            YAW axis");
		_putty_out->print(ROW_MENU + (row_add += 1), COL_MENU, " P, I, D or E select the coefficient)");
		_putty_out->print(ROW_MENU + (row_add += 1), COL_MENU, "(0),(1),(2),(3) or (5) select the accurarcy");
		_putty_out->print(ROW_MENU + (row_add += 1), COL_MENU, "(+) increment according to the value");
		_putty_out->print(ROW_MENU + (row_add += 1), COL_MENU, "(-) decrement      ''");
		_putty_out->print(ROW_MENU + (row_add += 1), COL_MENU, "(S) saves all coefficient into the EEPROM");
		_putty_out->print(ROW_MENU + (row_add += 1), COL_MENU, "(R) reads all coefficients from the EEPROM");
		_putty_out->print(ROW_MENU + (row_add += 1), COL_MENU, "(C) Copies the primary values to the secondary axis");
		_putty_out->print(ROW_MENU + (row_add += 1), COL_MENU, "(A) all values are set to 0 in the EEPROM.");
		_putty_out->print(ROW_MENU + (row_add += 1), COL_MENU, "(G) get factory defaults");
		_putty_out->print(ROW_MENU + (row_add += 1), COL_MENU, "(H) set the maximal altitude");
		_putty_out->print(ROW_MENU + (row_add += 1), COL_MENU, "(N) set the maximal near field altitude");
		_putty_out->print(ROW_MENU + (row_add += 1), COL_MENU, "(F) set the maximal distance to horizontal object.");
		_putty_out->print(ROW_MENU + (row_add += 1), COL_MENU, "(M) display the menu");
		_putty_out->gray();
		_putty_out->print(ROW_MENU + (row_add += 2), COL_MENU, "---------------------------------------------------------");

		_putty_out->cyan();
		_putty_out->print(ROW_MENU + 7, COL_MENU + 47, 3, _newAddOn);
		_putty_out->gray();
		_putty_out->print(ROW_STATE, COL_MENU, "State message : ");
		clearStateLine();

		LOGGER_VERBOSE("....leave");
	} /*-------------------------- end of display_Menu -----------------------------------------*/

	void displayPIDcoefficients()
	{
		_putty_out->blue();
		_putty_out->print(ROW_MENU + 21, COL_MENU + 32, "..in EEPROM");

		_putty_out->gray();
		_putty_out->print(ROW_MENU + 22, COL_MENU + 36, 3, _namedPID[axisName::primary]._pid->getP());
		_putty_out->print(ROW_MENU + 23, COL_MENU + 36, 3, _namedPID[axisName::primary]._pid->getI());
		_putty_out->print(ROW_MENU + 24, COL_MENU + 36, 3, _namedPID[axisName::primary]._pid->getD());
		_putty_out->print(ROW_MENU + 25, COL_MENU + 36, 3, _namedPID[axisName::primary]._pid->getEF());
		_putty_out->print(ROW_MENU + 27, COL_MENU + 36, 3, _namedPID[axisName::secondary]._pid->getP());
		_putty_out->print(ROW_MENU + 28, COL_MENU + 36, 3, _namedPID[axisName::secondary]._pid->getI());
		_putty_out->print(ROW_MENU + 29, COL_MENU + 36, 3, _namedPID[axisName::secondary]._pid->getD());
		_putty_out->print(ROW_MENU + 30, COL_MENU + 36, 3, _namedPID[axisName::secondary]._pid->getEF());
		_putty_out->print(ROW_MENU + 32, COL_MENU + 36, 3, _namedPID[axisName::yaw]._pid->getP()); //  hier stimmt was nicht mit der Ausgabe ??
		_putty_out->print(ROW_MENU + 33, COL_MENU + 36, 3, _namedPID[axisName::yaw]._pid->getI());
		_putty_out->print(ROW_MENU + 34, COL_MENU + 36, 3, _namedPID[axisName::yaw]._pid->getD());
		_putty_out->print(ROW_MENU + 35, COL_MENU + 36, 1, _namedPID[axisName::yaw]._pid->getEF());
	} /*--------------------- end of displayPIDcoefficients -------------------------------------*/

	bool checkValue(float a) // It should be ensured that no negative values are passed.
	{
		if (a >= 0)
			return true;
		else
			return false;
	} /*----------------------------- end of checkValue -----------------------------------------*/

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
		case 5:
			_newAddOn = 5;
			break;
		} // end of switch
		LOGGER_NOTICE_FMT("New Factor = %f", _newAddOn);
	} /*----------------------------- end of setDecimalPlaces -----------------------------------*/

}; /*------------------------- end of PID_adjust class ------------------------------------------*/
#endif