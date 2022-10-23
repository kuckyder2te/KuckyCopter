/* 		File name: newPID.h
 *  	Created: 2022-06-13
 *      Author: willy
 * 		Project: Phantom 1
 *      https://github.com/mike-matera/FastPID
 *
 *      The parameter P domain is [0.00390625 to 255] inclusive.
 *      Last PID parameter
 *      		P		I		D
 *      Pri		0,14	0,18	0,102
 *      Sec		0,14	0,18	0,102
 *      Yaw		0,01	0		0
 */
#pragma once

#include <FastPID.h>
#include "myLogger.h"
#include "..\..\EEPROM\EEPROM.h"   // Refernce to framework  1.. is Root 2.. ins framework
//#include "..\lib\model.h"
#include "def.h"

#define PID_FREQUENCY      50			///< PID parameter
#define PID_P_MIN			0.00390626	///< The parameter P domain is [0.00390625 to 255] inclusive.
#define PID_EEPROM_ADRRESS 50

typedef struct
{
	float pidCoefficient[3];  // 12 bytes
	float executionFrequency; // 4		besser via #define ??
	uint8_t output_bits;	  // 2			""
	bool output_signed;		  // 1   		""
	bool modified; 			  // 1   muss gesetzt werden wenn die Parameter manuell geändert wurden
} pidData_t;

// typedef struct
// {
// 	float kP;
// 	float kI;
// 	float kD;
// 	float exFreq;
// 	bool modified; // muss gesetzt werden wenn die Parameter manuell geändert wurden
// } pidData_TEST_t;

// typedef struct
// {
// 	float kP;
// 	float kI;
// 	float kD;
// 	bool modified; // muss gesetzt werden wenn die Parameter manuell geändert wurden
// } pidCoeffi_e;

//static pidData_t initPid[] ={1.11, 2.22, 3.33, 50, 8, false, false};
									
class NewPID : public FastPID
{ 

private:	
//  pidCoeffi_e _pidCoeffi;
//	pidData_TEST_t _pidData_TEST;
	pidData_t pidData[3];
	bool _isEnabled;
	String _ParentName;

protected:
	float RC_SP;
	float FB;

public:
	NewPID(String name){
		_ParentName = name;
		_isEnabled = false;
		this->setOutputRange(-100, 100);
		this->setOutputConfig(16, true);

		// pidData[0].pidCoefficient[pidCoeffi_e::kP] = 1.11;
		// pidData[0].pidCoefficient[pidCoeffi_e::kI] = 2.22;
		// pidData[0].pidCoefficient[pidCoeffi_e::kD] = 3.33;
		// pidData[0].executionFrequency = 50;
		// pidData[0].output_bits = 8;		  // default lt. FastPID
		// pidData[0].output_signed = false;	  // default lt. FastPID 

		// pidData[1].pidCoefficient[pidCoeffi_e::kP] = 0.11;
		// pidData[1].pidCoefficient[pidCoeffi_e::kI] = 0.22;
		// pidData[1].pidCoefficient[pidCoeffi_e::kD] = 0.33;
		// pidData[1].executionFrequency = 60;
		// pidData[1].output_bits = 8;		  // default lt. FastPID
		// pidData[1].output_signed = false;	  // default lt. FastPID 

		// pidData[2].pidCoefficient[pidCoeffi_e::kP] = 5.55;
		// pidData[2].pidCoefficient[pidCoeffi_e::kI] = 8.88;
		// pidData[2].pidCoefficient[pidCoeffi_e::kD] = 6.66;
		// pidData[2].executionFrequency = 10;
		// pidData[2].output_bits = 8;		  // default lt. FastPID
		// pidData[2].output_signed = false;	  // default lt. FastPID 

		// pidData[0].pidCoefficient[pidCoeffi_e::kP] = 5.11;
		// pidData[0].pidCoefficient[pidCoeffi_e::kI] = 2.22;
		// pidData[0].pidCoefficient[pidCoeffi_e::kD] = 3.33;
		// pidData[0].executionFrequency = 50;
		// pidData[0].output_bits = 8;		  // default lt. FastPID
		// pidData[0].output_signed = false;	  // default lt. FastPID 

		// pidData[1].pidCoefficient[pidCoeffi_e::kP] = 5.11;
		// pidData[1].pidCoefficient[pidCoeffi_e::kI] = 0.22;
		// pidData[1].pidCoefficient[pidCoeffi_e::kD] = 0.33;
		// pidData[1].executionFrequency = 60;
		// pidData[1].output_bits = 8;		  // default lt. FastPID
		// pidData[1].output_signed = false;	  // default lt. FastPID 

		// pidData[2].pidCoefficient[pidCoeffi_e::kP] = 10.55;
		// pidData[2].pidCoefficient[pidCoeffi_e::kI] = 8.88;
		// pidData[2].pidCoefficient[pidCoeffi_e::kD] = 6.66;
		// pidData[2].executionFrequency = 10;
		// pidData[2].output_bits = 8;		  // default lt. FastPID
		// pidData[2].output_signed = false;	  // default lt. FastPID 

		disablePID();
	} /*-------------------------------- end of constructor ---------------------------*/

	void init(uint8_t instance)
	{
		static int count = 0;
		
		LOGGER_WARNING_FMT("NewPID init = instance = %i count = %i", instance, count);
		uint8_t start = instance * sizeof(pidData_t);
		uint8_t size = sizeof(pidData_t);
		LOGGER_WARNING_FMT("Startadresse %i Instance %i Size %i", start, instance, size);
//		loadParameters(start, instance);
		
		saveParameters(start, &pidData[instance]);
		//if(!_pidParameter.modified){
		// 	saveParameters(start, &initPid);
		// 	// for(int i=start;i<(sizeof(pidData_t)+start);i++){
		// 	// 	EEPROM.write(i, instance);
		// 	// }
		// }
		count++;

	//	delay(2000);
	} /*-------------------------------- end of int -----------------------------------*/	

	void saveParameters(uint16_t addr, pidData_t* data){	
		static int count = 0;

		LOGGER_WARNING_FMT("SavePara - sizeof pidData_t = %i addr = %i count = %i", sizeof(pidData_t), addr, count);
		uint8_t* current = reinterpret_cast<uint8_t*>(data);

		for(uint8_t i=0; i<sizeof(pidData_t); i++){
		//	LOGGER_WARNING_FMT("i = %i", i);
			EEPROM.write(addr+i,*(current+i));						//Pointer arethmetic
			Serial2.println(*(current+i));
		//	LOGGER_WARNING_FMT("Addr = %i current %i", addr+1, (*(current+i)));
		}

		if (EEPROM.commit()) {
			LOGGER_WARNING("EEPROM successfully committed");
		} else {
			LOGGER_WARNING("ERROR! EEPROM commit failed");
		}
		count++;
	//	delay(2000);
	} /*-------------------------------- end of saveParameters ------------------------*/

	void loadParameters(int addr, int instance){
		static int count = 0;

		LOGGER_WARNING_FMT("LoadPara - sizeof pidData_t = %i addr = %i Instance = %i count = %i", sizeof(pidData_t), addr, instance, count);
		// for (int i = 0; i < sizeof(pidData_t); i++) {
    	// 	Serial2.println(EEPROM.read(i));		
		// }

		uint8_t* current = reinterpret_cast<uint8_t*>(&pidData[instance-1]);   // -1 ???

			for(uint8_t i=0; i<sizeof(pidData_t); i++){
				*(current+i) = EEPROM.read((addr+i));
				LOGGER_WARNING_FMT("i = %i",(uint8_t)*(current+i));
			}

		// LOGGER_WARNING_FMT("_pidData.kP = %.2f", pidData[instance].pidCoefficient[pidCoeffi_e::kP]);
		// LOGGER_WARNING_FMT("_pidData.kI = %.2f", pidData[instance].pidCoefficient[pidCoeffi_e::kI]);
		// LOGGER_WARNING_FMT("_pidData.kD = %.2f", pidData[instance].pidCoefficient[pidCoeffi_e::kD]);
		// LOGGER_WARNING_FMT("_pidData.exFreq = %.1f", pidData[instance].executionFrequency);
		// LOGGER_WARNING_FMT("_pidData.Output bits = %i", pidData[instance].output_bits);
		// LOGGER_WARNING_FMT("_pidData.output signed = %i", pidData[instance].output_signed);
		count++;
	//	delay(2000);
	} /*-------------------------------- end of loadParameters ------------------------*/

	void disablePID()
	{
		LOGGER_NOTICE_FMT("Disabled PID controller %s ", _ParentName.c_str());
	//		this->setCoefficients(_pidParameter.kP,0.0,0.0,_pidParameter.exFreq);
			for(uint8_t i = 0; i < 3; i++){
				this->setCoefficients(pidData[i].pidCoefficient[pidCoeffi_e::kP], 0.0, 0.0, PID_FREQUENCY);
			}
			_isEnabled = false;
	} /*-------------------------------- end of deactivatePID -------------------------*/

	void enablePID()
	{
		/* This function has 2 tasks.
		 * 1. The PID parameters are uploaded from the PID adjustment.
		 * 2. The PID parameters are activated. */
			LOGGER_NOTICE("enablePID");
	//		this->setCoefficients(_pidParameter.kP,_pidParameter.kI,_pidParameter.kD,_pidParameter.exFreq);
			for(uint8_t i = 0; i < 3; i++){
				this->setCoefficients(pidData[axisName_e::primary].pidCoefficient[pidCoeffi_e::kP], 
									  pidData[axisName_e::secondary].pidCoefficient[pidCoeffi_e::kI],
									  pidData[axisName_e::yaw].pidCoefficient[pidCoeffi_e::kD],
									  PID_FREQUENCY);
			}
		_isEnabled = true;
	} /*-------------------------------- end of activatePID ---------------------------*/

	void setP(float p, uint8_t axis)
	{
		float last_P0, last_P1, last_P2;
		LOGGER_WARNING_FMT("setP: %f axis %i", p, axis);

		if (p <= PID_P_MIN){
		 	p = PID_P_MIN;
		}

		switch(axis){

			case (axisName_e::primary):
				p = p + last_P0;
				pidData[axisName_e::primary].pidCoefficient[pidCoeffi_e::kP] = p;
				last_P0 = p;
				break;

			case (axisName_e::secondary):
				p = p + last_P1;
				pidData[axisName_e::secondary].pidCoefficient[pidCoeffi_e::kP] = p;
				last_P1 = p;
				break;
				
			case (axisName_e::yaw):
				p = p + last_P2;
				pidData[axisName_e::yaw].pidCoefficient[pidCoeffi_e::kP] = p;
				last_P2 = p;
				break;
		} // end of switch

		LOGGER_WARNING_FMT("Axis %i _pidCoeff.kP: %f", axis, pidData[axisName_e::primary].pidCoefficient[pidCoeffi_e::kP]);
		LOGGER_WARNING_FMT("Axis %i _pidCoeff.kP: %f", axis, pidData[axisName_e::secondary].pidCoefficient[pidCoeffi_e::kP]);
		LOGGER_WARNING_FMT("Axis %i _pidCoeff.kP: %f", axis, pidData[axisName_e::yaw].pidCoefficient[pidCoeffi_e::kP]);

		if(_isEnabled){
			enablePID();
		}
		delay(100);
	} /*-------------------------------- end of setP ----------------------------------*/

	void setI(float i, uint8_t axis)
	{
		float last_I0, last_I1, last_I2;
		LOGGER_NOTICE_FMT("setI: %f", i);

		switch(axis){

			case (axisName_e::primary):
				i = i + last_I0;
				pidData[axisName_e::primary].pidCoefficient[pidCoeffi_e::kI] = i;
				last_I0 = i;
				break;

			case (axisName_e::secondary):
				i = i + last_I1;
				pidData[axisName_e::secondary].pidCoefficient[pidCoeffi_e::kI] = i;
				last_I1 = i;
				break;
				
			case (axisName_e::yaw):
				i = i + last_I2;
				pidData[axisName_e::yaw].pidCoefficient[pidCoeffi_e::kI] = i;
				last_I2 = i;
				break;
		} // end of switch

		LOGGER_WARNING_FMT("Axis %i _pidCoeff.kP: %f", axis, pidData[axisName_e::primary].pidCoefficient[pidCoeffi_e::kI]);
		LOGGER_WARNING_FMT("Axis %i _pidCoeff.kP: %f", axis, pidData[axisName_e::secondary].pidCoefficient[pidCoeffi_e::kI]);
		LOGGER_WARNING_FMT("Axis %i _pidCoeff.kP: %f", axis, pidData[axisName_e::yaw].pidCoefficient[pidCoeffi_e::kI]);
		if(_isEnabled)
			enablePID();
	} /*-------------------------------- end of setI ----------------------------------*/

	void setD(float d, uint8_t axis)
	{
		float last_D0, last_D1, last_D2;
		LOGGER_NOTICE_FMT("setD: %f", d);
		switch(axis){

			case (axisName_e::primary):
				d = d + last_D0;
				pidData[axisName_e::primary].pidCoefficient[pidCoeffi_e::kI] = d;
				last_D0 = d;
				break;

			case (axisName_e::secondary):
				d = d + last_D1;
				pidData[axisName_e::secondary].pidCoefficient[pidCoeffi_e::kI] = d;
				last_D1 = d;
				break;
				
			case (axisName_e::yaw):
				d = d + last_D2;
				pidData[axisName_e::yaw].pidCoefficient[pidCoeffi_e::kI] = d;
				last_D2 = d;
				break;
		}  // end of switch

		LOGGER_WARNING_FMT("Axis %i _pidCoeff.kP: %f", axis, pidData[axisName_e::primary].pidCoefficient[pidCoeffi_e::kD]);
		LOGGER_WARNING_FMT("Axis %i _pidCoeff.kP: %f", axis, pidData[axisName_e::secondary].pidCoefficient[pidCoeffi_e::kD]);
		LOGGER_WARNING_FMT("Axis %i _pidCoeff.kP: %f", axis, pidData[axisName_e::yaw].pidCoefficient[pidCoeffi_e::kD]);	
		if(_isEnabled)
			enablePID();
	} /*-------------------------------- end of setD ----------------------------------*/

	void setExecutionFrequency(uint8_t ef)
	{
		LOGGER_NOTICE_FMT("setExecutionFrequency: %d", ef);
	//	_pidParameter.exFreq = ef;
		if(_isEnabled)
			enablePID();
	} /*-------------------------------- end of setExecutionFrequency -----------------*/

	uint8_t getExecutionTime()
	{
		//LOGGER_NOTICE_FMT("PID getExecutionTime %f", (1 / _pidParameter.exFreq) * 1000);
		return 0; // ((1.0/(float)_pidParameter.exFreq)*1000);
		///< Convert frequency to millis
	} /*-------------------------------- end of getExecutionTime ----------------------*/

	void updateEEPROM(void)
	{
	//	EEPROM.read(addr, _pidParameter);
		enablePID();
	} /*-------------------------------- end of updateEEPROM --------------------------*/

	void readEEPROM(void)
	{
	//	EEPROM.write(addr, _pidParameter);
		enablePID();
	} /*-------------------------------- end of readEEPROM ----------------------------*/

	float getP() const
	{
		return 0;
	} /*-------------------------------- end of getP ----------------------------------*/

	float getI() const
	{
		return 0;
	} /*-------------------------------- end of getI ----------------------------------*/

	float getD() const
	{
		return 0;
	} /*-------------------------------- end of getD ----------------------------------*/

	float getExFreq() const
	{
		return 0;
	} /*-------------------------------- end of getExTime -----------------------------*/
};/*--------------------------- end of MyPid class ------------------------------------*/

//#undef _DEBUG_
