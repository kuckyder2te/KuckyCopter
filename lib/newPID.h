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
// 	float exFreq;
// 	bool modified; // muss gesetzt werden wenn die Parameter manuell geändert wurden
// } pidParameter_t;

//static pidData_t initPid[] ={1.11, 2.22, 3.33, 50, 8, false, false};
									
class NewPID : public FastPID
{ 

private:	
//	pidParameter_t _pidParameter;
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
		pidData[0].pidCoefficient[pidCoeff_t::kP] = 1.11;
		pidData[0].pidCoefficient[pidCoeff_t::kI] = 2.22;
		pidData[0].pidCoefficient[pidCoeff_t::kD] = 3.33;
		pidData[0].executionFrequency = 50;
		pidData[0].output_bits = 8;		  // default lt. FastPID
		pidData[0].output_signed = false;	  // default lt. FastPID 

		pidData[1].pidCoefficient[pidCoeff_t::kP] = 0.11;
		pidData[1].pidCoefficient[pidCoeff_t::kI] = 0.22;
		pidData[1].pidCoefficient[pidCoeff_t::kD] = 0.33;
		pidData[1].executionFrequency = 60;
		pidData[1].output_bits = 8;		  // default lt. FastPID
		pidData[1].output_signed = false;	  // default lt. FastPID 

		pidData[2].pidCoefficient[pidCoeff_t::kP] = 5.55;
		pidData[2].pidCoefficient[pidCoeff_t::kI] = 8.88;
		pidData[2].pidCoefficient[pidCoeff_t::kD] = 6.66;
		pidData[2].executionFrequency = 10;
		pidData[2].output_bits = 8;		  // default lt. FastPID
		pidData[2].output_signed = false;	  // default lt. FastPID 

		disablePID();
	} /*-------------------------------- end of constructor ---------------------------*/

	void init(uint8_t instance)
	{
		static int count = 0;
		Serial2.print("NewPID init ");Serial2.println(count);

		uint8_t start = instance * sizeof(pidData_t);
		uint8_t size = sizeof(pidData_t);
		LOGGER_WARNING_FMT("Startadresse %i Instance %i Size %i", start, instance, size);
		loadParameters(start, instance);
		//saveParameters(start, &initPid[3][5]);
//		saveParameters(start, &pidData[count]);
		//if(!_pidParameter.modified){
		// 	saveParameters(start, &initPid);
		// 	// for(int i=start;i<(sizeof(pidData_t)+start);i++){
		// 	// 	EEPROM.write(i, instance);
		// 	// }
		// }
		count++;

		delay(2000);
	} /*-------------------------------- end of int -----------------------------------*/	

	void saveParameters(uint16_t addr, pidData_t* data){	
		static int count = 0;
		Serial2.print("saveParameters ");Serial2.println(count);

		uint8_t* current = reinterpret_cast<uint8_t*>(data);

		for(uint8_t i=0; i<sizeof(pidData_t); i++){
		//	LOGGER_WARNING_FMT("i = %i", i);
			EEPROM.write(addr+i,*(current+i));						//Pointer arethmetic
			Serial2.println(*(current+i));
		}

		if (EEPROM.commit()) {
			LOGGER_WARNING("EEPROM successfully committed");
		} else {
			LOGGER_WARNING("ERROR! EEPROM commit failed");
		}
		count++;
		delay(2000);
	} /*-------------------------------- end of saveParameters ------------------------*/

	void loadParameters(int addr, int instance){
		static int count = 0;
		Serial2.print("loadParameters ");Serial2.println(count);

		LOGGER_WARNING_FMT("sizeof pidData_t = %i addr = %i Instance = %i", sizeof(pidData_t), addr, instance);
		// for (int i = 0; i < sizeof(pidData_t); i++) {
    	// 	Serial2.println(EEPROM.read(i));		
		// }

		uint8_t* current = reinterpret_cast<uint8_t*>(&pidData[instance-1]);   // -1 ???

			for(uint8_t i=0; i<sizeof(pidData_t); i++){
				*(current+i) = EEPROM.read((addr+i));
				LOGGER_WARNING_FMT("i = %i",(uint8_t)*(current+i));
			}

		LOGGER_WARNING_FMT("_pidData.kP = %.2f", pidData[instance].pidCoefficient[pidCoeff_t::kP]);
		LOGGER_WARNING_FMT("_pidData.kI = %.2f", pidData[instance].pidCoefficient[pidCoeff_t::kI]);
		LOGGER_WARNING_FMT("_pidData.kD = %.2f", pidData[instance].pidCoefficient[pidCoeff_t::kD]);
		LOGGER_WARNING_FMT("_pidData.exFreq = %.1f", pidData[instance].executionFrequency);
		LOGGER_WARNING_FMT("_pidData.Output bits = %i", pidData[instance].output_bits);
		LOGGER_WARNING_FMT("_pidData.output signed = %i", pidData[instance].output_signed);
		count++;
		delay(2000);
	} /*-------------------------------- end of loadParameters ------------------------*/

	void disablePID()
	{
		LOGGER_NOTICE_FMT("Disabled PID controller %s ", _ParentName.c_str());
	//		this->setCoefficients(_pidParameter.kP,0.0,0.0,_pidParameter.exFreq);
			_isEnabled = false;
	} /*-------------------------------- end of deactivatePID -------------------------*/

	void enablePID()
	{
		/* This function has 2 tasks.
		 * 1. The PID parameters are uploaded from the PID adjustment.
		 * 2. The PID parameters are activated. */
			LOGGER_NOTICE("enablePID");
	//		this->setCoefficients(_pidParameter.kP,_pidParameter.kI,_pidParameter.kD,_pidParameter.exFreq);
		_isEnabled = true;
	} /*-------------------------------- end of activatePID ---------------------------*/

	void setP(float p)
	{
		LOGGER_NOTICE_FMT("setP: %f", p);
		// _pidParameter.kP = p;
		// if (_pidParameter.kP <= PID_P_MIN){
		// 	_pidParameter.kP = PID_P_MIN;
		// }
		//LOGGER_NOTICE_FMT("_pidParameter.kP: %f", _pidParameter.kP);
		if(_isEnabled){
			enablePID();
		}
	} /*-------------------------------- end of setP ----------------------------------*/

	void setI(float i)
	{
		LOGGER_NOTICE_FMT("setI: %f", i);
		// _pidParameter.kI = i;
		// if (_pidParameter.kI <= 0){
		// 	_pidParameter.kI = 0;
		// }
		//LOGGER_NOTICE_FMT("_pidParameter.kI: %f", _pidParameter.kI);
		if(_isEnabled)
			enablePID();
	} /*-------------------------------- end of setI ----------------------------------*/

	void setD(float d)
	{
		LOGGER_NOTICE_FMT("setD: %f", d);
		// _pidParameter.kD = d;
		// if (_pidParameter.kD <= 0){
		// 	_pidParameter.kD = 0;
		// }
		// LOGGER_NOTICE_FMT("_pidParameter.kD: %f", _pidParameter.kD);		
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
