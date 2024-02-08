/*  File name : PID_Calbration.h
	Project name : KuCo_Phantom 1
	Author: Stephan Scholz / Wilhelm Kuckelsberg
	Date : 2022-07-02
	Description : Einstellen de PID Regler via Bluetooth
				  und die maximalen Flughöhen
*/

#pragma once

#include <TaskManager.h>
#include <Arduino.h>
#include <HardwareSerial.h>

#define LOCAL_DEBUG
#include "..\lib\myLogger.h"

#include "..\lib\putty_out.h"
#include "dictionary.h"
#include "..\lib\model.h"

#ifdef _PID_ADJUST

#define PID_NUM 3

#define ROW_MENU 3 ///< First position for the main menue
#define COL_MENU 10
#define ROW_STATE ROW_MENU + 46 // Position for state message
#define COL_STATE COL_MENU + 16

class PID_adjust : public Task::Base
{
private:
	uint8_t _pidCount;
	uint8_t level1;
	uint8_t level2;
	uint8_t _pidType;
	uint8_t _dotPlaces = 3; ///< Decimal places.
	float _newAddOn = 0.1;	///< Multiplication factor for the PID coefficients, default setting.
	double _addOn;
	uint8_t _maxPID;
	uint8_t _option;

	typedef enum
	{
		axis_pri = 1,
		axis_sec = 2,
		axis_yaw = 3,
	} level1_t; // Base number of the axis

	typedef enum
	{
		offset_P = 10,
		offset_I = 20,
		offset_D = 30,
		offset_EF = 40
	} level2_t; // number to be added to the base

	typedef enum // Pid_typ Primary P to YAW ef
	{
		pri_P =  11,
		pri_I =  21,
		pri_D =  31,
		pri_EF = 41,

		sec_P =  12,
		sec_I =  22,
		sec_D =  32,
		sec_EF = 42,

		yaw_P =  13,
		yaw_I =  23,
		yaw_D =  33,
		yaw_EF = 43,
	} pidTyp_t;						/* level1_t + level2_t returns
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
				set_level1(level1_t::axis_pri);
				_putty_out->yellow();
				clearStateLine();
				_putty_out->clearPart(ROW_MENU + 21, COL_MENU, _dict->c_whitespace);	///< Clears the current line
				_putty_out->print(ROW_MENU + 21, COL_MENU, _dict->c_axis_pri_select); ///< Print the selected axis
				LOGGER_NOTICE_FMT("Level 1 =  %i", level1);
				break;

			case 'Y':
				set_level1(level1_t::axis_sec);
				_putty_out->yellow();
				clearStateLine();
				_putty_out->clearPart(ROW_MENU + 26, COL_MENU, _dict->c_whitespace);
				_putty_out->print(ROW_MENU + 26, COL_MENU, _dict->c_axis_sec_select);
				LOGGER_NOTICE_FMT("Level 1 =  %i", level1);
				break;

			case 'Z':
				set_level1(level1_t::axis_yaw);
				_putty_out->yellow();
				clearStateLine();
				_putty_out->clearPart(ROW_MENU + 31, COL_MENU, _dict->c_whitespace);
				_putty_out->print(ROW_MENU + 31, COL_MENU, _dict->c_axis_yaw_select);
				LOGGER_NOTICE_FMT("Level 1 =  %i", level1);
				break;

			case 'P': ///< Choose the PID parameter
				set_level2(level2_t::offset_P);
				_option = level1 + level2;
				LOGGER_NOTICE_FMT("Option %i", _option);

				switch (_option)
				{
				case pri_P:
					_putty_out->red();
					_putty_out->print(ROW_MENU + 22, COL_MENU + 22, 3, _namedPID[axisName::primary]._pid->getP());
					_putty_out->print(ROW_MENU + 22, COL_MENU + 28, _dict->c_current);
					clear_current_eeprom(22);
					break;
				case sec_P:
					_putty_out->red();
					_putty_out->print(ROW_MENU + 27, COL_MENU + 22, 3, _namedPID[axisName::secondary]._pid->getP());
					_putty_out->print(ROW_MENU + 27, COL_MENU + 28, _dict->c_current);
					clear_current_eeprom(27);

					break;
				case yaw_P:
					_putty_out->red();
					_putty_out->print(ROW_MENU + 32, COL_MENU + 22, 3, _namedPID[axisName::yaw]._pid->getP());
					_putty_out->print(ROW_MENU + 32, COL_MENU + 28, _dict->c_current);
					clear_current_eeprom(32);

					break;
				}

				_putty_out->yellow();
				clearStateLine();
				_putty_out->clearPart(ROW_MENU + 22 + ((level1 - 1) * 5), COL_MENU + 5, _dict->c_whitespace);
				_putty_out->print(ROW_MENU + 22 + ((level1 - 1) * 5), COL_MENU + 5, _dict->c_p_coeff); ///< Print the selected coefficient
				break;

			case 'I':
				set_level2(level2_t::offset_I);
				_option = level1 + level2;

				LOGGER_NOTICE_FMT("Option %i", _option);

				switch (_option)
				{
				case pri_I:
					_putty_out->red();
					_putty_out->print(ROW_MENU + 23, COL_MENU + 22, 3, _namedPID[axisName::primary]._pid->getI());
					_putty_out->print(ROW_MENU + 23, COL_MENU + 28, _dict->c_current);
					clear_current_eeprom(23);
					break;
				case sec_I:
					_putty_out->red();
					_putty_out->print(ROW_MENU + 28, COL_MENU + 22, 3, _namedPID[axisName::secondary]._pid->getI());
					_putty_out->print(ROW_MENU + 28, COL_MENU + 28, _dict->c_current);
					clear_current_eeprom(28);
					break;
				case yaw_I:
					_putty_out->red();
					_putty_out->print(ROW_MENU + 33, COL_MENU + 22, 3, _namedPID[axisName::yaw]._pid->getI());
					_putty_out->print(ROW_MENU + 33, COL_MENU + 28, _dict->c_current);
					clear_current_eeprom(33);
					break;
				}

				_putty_out->yellow();
				clearStateLine();
				_putty_out->clearPart(ROW_MENU + 23 + ((level1 - 1) * 5), COL_MENU + 5, _dict->c_whitespace);
				_putty_out->print(ROW_MENU + 23 + ((level1 - 1) * 5), COL_MENU + 5, _dict->c_i_coeff);
				break;

			case 'D':
				set_level2(level2_t::offset_D);
				_option = level1 + level2;
				LOGGER_FATAL_FMT("Option %i", _option);

				switch (_option)
				{
				case pri_D:
					_putty_out->red();
					_putty_out->print(ROW_MENU + 24, COL_MENU + 22, 3, _namedPID[axisName::primary]._pid->getD());
					_putty_out->print(ROW_MENU + 24, COL_MENU + 28, _dict->c_current);
					clear_current_eeprom(24);
					break;
				case sec_D:
					_putty_out->red();
					_putty_out->print(ROW_MENU + 29, COL_MENU + 22, 3, _namedPID[axisName::secondary]._pid->getD());
					_putty_out->print(ROW_MENU + 29, COL_MENU + 28, _dict->c_current);
					clear_current_eeprom(29);
					break;
				case yaw_D:
					_putty_out->red();
					_putty_out->print(ROW_MENU + 34, COL_MENU + 22, 3, _namedPID[axisName::yaw]._pid->getD());
					_putty_out->print(ROW_MENU + 34, COL_MENU + 28, _dict->c_current);
					clear_current_eeprom(34);
					break;
				}
				_putty_out->yellow();
				clearStateLine();
				_putty_out->clearPart(ROW_MENU + 24 + ((level1 - 1) * 5), COL_MENU + 5, _dict->c_whitespace);
				_putty_out->print(ROW_MENU + 24 + ((level1 - 1) * 5), COL_MENU + 5, _dict->c_d_coeff);
				break;

			case 'E':
				set_level2(level2_t::offset_EF);
				_option = level1 + level2;
				LOGGER_FATAL_FMT("Option %i", _option);

				switch (_option)
				{
				case pri_EF:
					_putty_out->red();
					_putty_out->print(ROW_MENU + 25, COL_MENU + 22, 3, _namedPID[axisName::primary]._pid->getEF());
					_putty_out->print(ROW_MENU + 25, COL_MENU + 28, _dict->c_current);
					clear_current_eeprom(25);
					break;
				case sec_EF:
					_putty_out->red();
					_putty_out->print(ROW_MENU + 30, COL_MENU + 22, 3, _namedPID[axisName::secondary]._pid->getEF());
					_putty_out->print(ROW_MENU + 30, COL_MENU + 28, _dict->c_current);
					clear_current_eeprom(30);
					break;
				case yaw_EF:
					_putty_out->red();
					_putty_out->print(ROW_MENU + 35, COL_MENU + 22, 3, _namedPID[axisName::yaw]._pid->getEF());
					_putty_out->print(ROW_MENU + 35, COL_MENU + 28, _dict->c_current);
					clear_current_eeprom(35);
					break;
				}

				_putty_out->yellow();
				clearStateLine();
				_putty_out->clearPart(ROW_MENU + 25 + ((level1 - 1) * 5), COL_MENU + 5, _dict->c_whitespace);
				_putty_out->print(ROW_MENU + 25 + ((level1 - 1) * 5), COL_MENU + 5, _dict->c_ef_coeff);
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
				clearStateLine();
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
				break;

			case 'R': ///< Reads all coefficients from the EEPROM
				for (uint8_t i = 0; i < 3; i++)
				{
					_namedPID[i]._pid->loadParameters();
				}

				_putty_out->red();
				_putty_out->print(ROW_MENU + 10, COL_MENU + 47, "Done");
				_putty_out->print(ROW_MENU + 9, COL_MENU + 47, "    ");
				break;

			case 'V': ///< Set all PID parameters to 0
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
				break;

			case 'G': ///< get factory default
				_namedPID[axisName::primary]._pid->setP(0.1);
				_namedPID[axisName::primary]._pid->setI(0.15);
				_namedPID[axisName::primary]._pid->setD(0.08);
				_namedPID[axisName::primary]._pid->setEF(50);
				_namedPID[axisName::secondary]._pid->setP(0.1);
				_namedPID[axisName::secondary]._pid->setI(0.15);
				_namedPID[axisName::secondary]._pid->setD(0.08);
				_namedPID[axisName::secondary]._pid->setEF(50);
				_namedPID[axisName::yaw]._pid->setP(0.01);
				_namedPID[axisName::yaw]._pid->setI(0);
				_namedPID[axisName::yaw]._pid->setD(0);
				_namedPID[axisName::yaw]._pid->setEF(50);
				break;

			case 'C': ///< Copies the primary values to the secondary axis
				_putty_out->red();
				_namedPID[axisName::secondary]._pid->setP(_namedPID[axisName::primary]._pid->getP());
				_namedPID[axisName::secondary]._pid->setI(_namedPID[axisName::primary]._pid->getI());
				_namedPID[axisName::secondary]._pid->setD(_namedPID[axisName::primary]._pid->getD());
				_namedPID[axisName::secondary]._pid->setEF(_namedPID[axisName::primary]._pid->getEF());
				_putty_out->yellow();
				break;

			case 'M':
				display_Menu();
				break;

			default:
			{
				_putty_out->red();
				_putty_out->print(ROW_STATE - 1, COL_STATE, "Illegal button");
				_putty_out->print(ROW_STATE, COL_STATE, "was pressed");
				_putty_out->yellow();
			}
			} /* end of switch(key) */
		}	  /* end of _serial.available */
		LOGGER_VERBOSE("....leave");
	} /* -------------------- end of update -----------------------------------------------------*/

	/* Clears the current PID coefficent" */
	void clear_current_eeprom(uint8_t x)
	{
		uint32_t lastMillis = millis();
		delay(1000);
		_putty_out->print(ROW_MENU + x, COL_MENU + 28, "         "); // 9 spaces
		LOGGER_NOTICE_FMT("clear current line =  %i", x);
	} /* -------------------- end of clear_current_eeprom ---------------------------------------*/

	/* Clears the string "Illegal button was pressed" */
	void clearStateLine()
	{
		_putty_out->print(ROW_MENU + 46, COL_STATE, _dict->c_whitespace);
	} /* -------------------- end of clearStateLine ---------------------------------------------*/

	void set_level1(uint8_t itemAxis)
	{
		/* Selects the axis, according to the keyboard input.
		 * Key X = primary axis (1)
		 * Key Y = primary axis (2)
		 * Key Z = primary axis (*3)	 */
		level1 = itemAxis;
		LOGGER_NOTICE_FMT("Level 1 = %d", level1);
	} /*----------------------------- end of set_level1 -----------------------------------------*/

	void set_level2(uint8_t itemCoefficient)
	{
		/* Selects the coefficient, according to the keyboard input.
		 * Key P = coefficient P (10)
		 * Key I = coefficient I (20)
		 * Key D = coefficient D (30)
		 * Key E = ExecutingFrequency (40)	 */
		level2 = itemCoefficient;
		LOGGER_NOTICE_FMT("Level 2 = %d", level2);
	} /*----------------------------- end of set_level2 ----------------------------------------*/

	/* Set the "PID Type",
	 * e.g level1 = 1 and level2 = 20 ~ _pidType 21
	 * Will say, it select the parameter for secondary axis and coefficient 'i'
	 */
	uint8_t getPidType(bool up)
	{
		_pidType = level1 + level2;
		LOGGER_NOTICE_FMT("PID Type Input = %d", _pidType);

		if (_pidType < 40)
		{ ///< P, I and D
			if (up)
				_addOn = _newAddOn;
			else
				_addOn = _newAddOn * -1;
		}
		if ((_pidType >= 40) && (_pidType < 44))
		{ ///< Executingfrequency only
			if (up)
				_addOn = 5;
			else
				_addOn = -5;
		}

		LOGGER_NOTICE_FMT("PID Type = %d", _pidType);
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
				_putty_out->print(ROW_MENU + 22, COL_MENU + 22, _dotPlaces, pri_kP_value);
				_namedPID[axisName::primary]._pid->setP(pri_kP_value);
			}
			break;

		case pidTyp_t::pri_I:
			pri_kI_value += _addOn;
			if (checkValue(pri_kI_value))
			{
				LOGGER_NOTICE_FMT("X Axis kI = %f", pri_kI_value);
				_putty_out->cyan();
				_putty_out->print(ROW_MENU + 23, COL_MENU + 27, _dotPlaces, pri_kI_value);
				_namedPID[axisName::primary]._pid->setI(pri_kI_value);
			}
			break;

		case pidTyp_t::pri_D:
			pri_kD_value += _addOn;
			if (checkValue(pri_kD_value))
			{
				LOGGER_NOTICE_FMT("X Axis kD = %f", pri_kD_value);
				_putty_out->cyan();
				_putty_out->print(ROW_MENU + 24, COL_MENU + 27, _dotPlaces, pri_kD_value);
				_namedPID[axisName::primary]._pid->setD(pri_kD_value);
			}
			break;

		case pidTyp_t::pri_EF:
			pri_EF_value += _addOn;
			if (checkValue(pri_EF_value))
			{
				LOGGER_WARNING_FMT("X Axis eF = %f", pri_EF_value);
				_putty_out->cyan();
				_putty_out->print(ROW_MENU + 25, COL_MENU + 232, _dotPlaces, pri_EF_value);
				_namedPID[axisName::primary]._pid->setEF(pri_EF_value);
			}
			break;

		case pidTyp_t::sec_P:
			sec_kP_value += _addOn;
			if (checkValue(sec_kP_value))
			{
				LOGGER_NOTICE_FMT("Y Axis kP = %f", sec_kP_value);
				_putty_out->cyan();
				_putty_out->print(ROW_MENU + 22 + ((level1 - 1) * 5), COL_MENU + 22, _dotPlaces, sec_kP_value);
				_namedPID[axisName::secondary]._pid->setP(sec_kP_value);
			}
			break;

		case pidTyp_t::sec_I:
			sec_kI_value += _addOn;
			if (checkValue(sec_kI_value))
			{
				LOGGER_NOTICE_FMT("Y Axis kI = %f", sec_kI_value);
				_putty_out->cyan();
				_putty_out->print(ROW_MENU + 23 + ((level1 - 1) * 5), COL_MENU + 22, _dotPlaces, sec_kI_value);
				_namedPID[axisName::secondary]._pid->setI(sec_kI_value);
			}
			break;

		case pidTyp_t::sec_D:
			sec_kD_value += _addOn;
			if (checkValue(sec_kD_value))
			{
				LOGGER_NOTICE_FMT("Y Axis kD = %f", sec_kD_value);
				_putty_out->cyan();
				_putty_out->print(ROW_MENU + 24 + ((level1 - 1) * 5), COL_MENU + 22, _dotPlaces, sec_kD_value);
				_namedPID[axisName::secondary]._pid->setD(sec_kD_value);
			}
			break;

		case pidTyp_t::sec_EF:
			sec_EF_value += _addOn;
			if (checkValue(sec_EF_value))
			{
				LOGGER_WARNING_FMT("Y Axis eF = %f", sec_EF_value);
				_putty_out->cyan();
				_putty_out->print(ROW_MENU + 25 + ((level1 - 1) * 5), COL_MENU + 22, _dotPlaces, sec_EF_value);
				_namedPID[axisName::secondary]._pid->setEF(sec_EF_value);
			}
			break;

		case pidTyp_t::yaw_P:
			yaw_kP_value += _addOn;
			if (checkValue(yaw_kP_value))
			{
				LOGGER_NOTICE_FMT("Z Axis kP = %f", yaw_kP_value);
				_putty_out->cyan();
				_putty_out->print(ROW_MENU + 22 + ((level1 - 1) * 5), COL_MENU + 22, _dotPlaces, yaw_kP_value);
				_namedPID[axisName::yaw]._pid->setP(yaw_kP_value);
			}
			break;

		case pidTyp_t::yaw_I:
			yaw_kI_value += _addOn;
			if (checkValue(yaw_kI_value))
			{
				LOGGER_NOTICE_FMT("Z Axis kI = %f", yaw_kI_value);
				_putty_out->cyan();
				_putty_out->print(ROW_MENU + 23 + ((level1 - 1) * 5), COL_MENU + 22, _dotPlaces, yaw_kI_value);
				_namedPID[axisName::yaw]._pid->setI(yaw_kI_value);
			}
			break;

		case pidTyp_t::yaw_D:
			yaw_kD_value += _addOn;
			if (checkValue(yaw_kD_value))
			{
				LOGGER_NOTICE_FMT("Z Axis kD = %f", yaw_kD_value);
				_putty_out->cyan();
				_putty_out->print(ROW_MENU + 24 + ((level1 - 1) * 5), COL_MENU + 22, _dotPlaces, yaw_kD_value);
				_namedPID[axisName::yaw]._pid->setD(yaw_kD_value);
			}
			break;

		case pidTyp_t::yaw_EF:
			yaw_EF_value += _addOn;
			if (checkValue(yaw_EF_value))
			{
				LOGGER_WARNING_FMT("Z Axis eF = %f", yaw_EF_value);
				_putty_out->cyan();
				_putty_out->print(ROW_MENU + 25 + ((level1 - 1) * 5), COL_MENU + 22, _dotPlaces, yaw_EF_value);
				_namedPID[axisName::yaw]._pid->setEF(yaw_EF_value);
			}
			break;

		} /* end of switch */
	} /*----------------------------- end of select ---------------------------------------------*/

	void display_Menu()
	{
		uint8_t row_add = -1;
		LOGGER_VERBOSE("Enter....");
		_putty_out->clear();
		_putty_out->gray();
		_putty_out->print(ROW_MENU + (row_add += 1), COL_MENU, "------ Online PID configurater for KuckyCopter (BT) -----");
		_putty_out->yellow();
		_putty_out->print(ROW_MENU + (row_add += 2), COL_MENU, "(X) choose the primary");
		_putty_out->print(ROW_MENU + (row_add += 1), COL_MENU, "(Y)           secondary");
		_putty_out->print(ROW_MENU + (row_add += 1), COL_MENU, "(Z)            YAW axis");
		_putty_out->print(ROW_MENU + (row_add += 1), COL_MENU, " P, I, D or E select the coefficient");
		_putty_out->print(ROW_MENU + (row_add += 1), COL_MENU, "(0),(1),(2),(3) or (5) select the accurarcy");
		_putty_out->print(ROW_MENU + (row_add += 1), COL_MENU, "(+) increment according to the value");
		_putty_out->print(ROW_MENU + (row_add += 1), COL_MENU, "(-) decrement      ''");
		_putty_out->print(ROW_MENU + (row_add += 1), COL_MENU, "(S) saves all coefficient into the EEPROM");
		_putty_out->print(ROW_MENU + (row_add += 1), COL_MENU, "(R) reads all coefficients from the EEPROM");
		_putty_out->print(ROW_MENU + (row_add += 1), COL_MENU, "(C) Copies the primary values to the secondary axis");
		_putty_out->print(ROW_MENU + (row_add += 1), COL_MENU, "(V) all values are set to 0 in the EEPROM.");
		_putty_out->print(ROW_MENU + (row_add += 1), COL_MENU, "(G) get factory defaults");
		_putty_out->print(ROW_MENU + (row_add += 1), COL_MENU, "(M) display the menu");
		_putty_out->gray();
		_putty_out->print(ROW_MENU + (row_add += 2), COL_MENU, "---------------------------------------------------------");

		_putty_out->cyan();
		_putty_out->print(ROW_MENU + 7, COL_MENU + 47, 3, _newAddOn);
		_putty_out->gray();
		_putty_out->print(ROW_STATE - 1, COL_MENU, "State message : ");
		_putty_out->print(ROW_STATE - 1, COL_MENU + 45, "Last compile ");

		_putty_out->cyan();
		_putty_out->print(ROW_STATE - 1, COL_MENU + 50, _dict->c_date);
		_putty_out->print(ROW_STATE, COL_MENU + 50, _dict->c_time);
		clearStateLine();

		LOGGER_VERBOSE("....leave");
	} /*-------------------------- end of display_Menu -----------------------------------------*/

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