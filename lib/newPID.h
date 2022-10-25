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
#include "def.h"

#define PID_FREQUENCY      50			///< PID parameter
#define PID_OUTPUT_BITS	   16
#define PID_OUTPUT_SIGNED  flase	
#define PID_P_MIN			0.00390626	///< The parameter P domain is [0.00390625 to 255] inclusive.
#define PID_EEPROM_ADRRESS 50

typedef struct
{
	float pidCoefficient[4];  // 12 bytes
	uint8_t output_bits;	  // 2			""
	bool output_signed;		  // 1   		""
	bool modified; 			  // 1   muss gesetzt werden wenn die Parameter manuell geändert wurden
} pidData_t;

static pidData_t initPid ={{1.11f, 2.22f, 3.33f, 50.0f}, 8, false, false};
									
class NewPID : public FastPID
{ 

private:	
	pidData_t _pidData;
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

		disablePID();
	} /*-------------------------------- end of constructor ---------------------------*/

	void init(uint8_t instance)
	{		
		LOGGER_WARNING_FMT("NewPID init = instance = %i", instance);
		uint8_t start = instance * sizeof(pidData_t);
		uint8_t size = sizeof(pidData_t);
		LOGGER_WARNING_FMT("Startadresse %i Size %i", start, size);		
		
		//saveParameters(start, &_pidData[instance]);
		//saveParameters(start, &initPid);
		loadParameters(start);

		//if(!_pidData.modified){
		// 	saveParameters(start, &initPid);
		// 	// for(int i=start;i<(sizeof(pidData_t)+start);i++){
		// 	// 	EEPROM.write(i, instance);
		// 	// }
		// }

	//	delay(2000);
	} /*-------------------------------- end of int -----------------------------------*/	

	void saveParameters(uint16_t addr){	
		saveParameters(addr, &_pidData);
	} /*-------------------------------- end of saveParameters ------------------------*/

	void saveParameters(uint16_t addr, pidData_t* data){	

		LOGGER_WARNING_FMT("SavePara - sizeof pidData_t = %i addr = %i", sizeof(pidData_t), addr);
		uint8_t* current = reinterpret_cast<uint8_t*>(data);

		for(uint8_t i=0; i<sizeof(pidData_t); i++){
		//	LOGGER_WARNING_FMT("i = %i", i);
			EEPROM.write(addr+i,*(current+i));						//Pointer arethmetic
		//	LOGGER_WARNING_FMT("Addr = %i current %i", addr+1, (*(current+i)));
		}

		if (EEPROM.commit()) {
			LOGGER_WARNING("EEPROM successfully committed");
		} else {
			LOGGER_WARNING("ERROR! EEPROM commit failed");
		}
	//	delay(2000);
	} /*-------------------------------- end of saveParameters ------------------------*/

	void loadParameters(int addr){

		LOGGER_WARNING_FMT("LoadPara - sizeof pidData_t = %i addr = %i", sizeof(pidData_t), addr);
		// for (int i = 0; i < sizeof(pidData_t); i++) {
    	// 	Serial2.println(EEPROM.read(i));		
		// }

		uint8_t* current = reinterpret_cast<uint8_t*>(&_pidData);   // current zeigt auf die gleiche Speicherstelle wie _pidData
																   // der datentyp _pidData wird in eine uint8 typ geändert
			for(uint8_t i=0; i<sizeof(pidData_t); i++){
				*(current+i) = EEPROM.read((addr+i));
				LOGGER_WARNING_FMT("i = %i",(uint8_t)*(current+i));
			}

		LOGGER_WARNING_FMT("_pidData.kP = %.2f", _pidData.pidCoefficient[pidCoeffi_e::kP]);
		LOGGER_WARNING_FMT("_pidData.kI = %.2f", _pidData.pidCoefficient[pidCoeffi_e::kI]);
		LOGGER_WARNING_FMT("_pidData.kD = %.2f", _pidData.pidCoefficient[pidCoeffi_e::kD]);
		LOGGER_WARNING_FMT("_pidData.exFreq = %.1f", _pidData.pidCoefficient[pidCoeffi_e::eF]);
		LOGGER_WARNING_FMT("_pidData.Output bits = %i", _pidData.output_bits);
		LOGGER_WARNING_FMT("_pidData.output signed = %i", _pidData.output_signed);

	//	delay(2000);
	} /*-------------------------------- end of loadParameters ------------------------*/

	void disablePID()
	{
		LOGGER_NOTICE_FMT("Disabled PID controller %s ", _ParentName.c_str());
			this->setCoefficients(PID_P_MIN,0.0,0.0,PID_FREQUENCY);

		_isEnabled = false;
	} /*-------------------------------- end of deactivatePID -------------------------*/

	void enablePID()
	{
		/* This function has 2 tasks.
		 * 1. The PID parameters are uploaded from the PID adjustment.
		 * 2. The PID parameters are activated. */
			LOGGER_NOTICE("enablePID");
			this->setCoefficients(_pidData.pidCoefficient[pidCoeffi_e::kP],
								  _pidData.pidCoefficient[pidCoeffi_e::kI],
								  _pidData.pidCoefficient[pidCoeffi_e::kD],
								  _pidData.pidCoefficient[pidCoeffi_e::eF]);

			_isEnabled = true;
	 } /*-------------------------------- end of activatePID ---------------------------*/

	void setP(float p)
	{		
		LOGGER_WARNING_FMT("setP: %f ", p);
		if (p <= PID_P_MIN){
		 	p = PID_P_MIN;
		}

		_pidData.pidCoefficient[pidCoeffi_e::kP] = p;

		LOGGER_WARNING_FMT(" _pidCoeff.kP: %f", _pidData.pidCoefficient[pidCoeffi_e::kP]);

		if(_isEnabled){
			enablePID();
		}
		delay(100);
	} /*-------------------------------- end of setP ----------------------------------*/

	void setI(float i)
	{
		LOGGER_NOTICE_FMT("setI: %f", i);

		_pidData.pidCoefficient[pidCoeffi_e::kI] = i;

		LOGGER_WARNING_FMT("_pidCoeff.kP: %f", _pidData.pidCoefficient[pidCoeffi_e::kI]);

		if(_isEnabled)
			enablePID();
	} /*-------------------------------- end of setI ----------------------------------*/

	void setD(float d)
	{
		_pidData.pidCoefficient[pidCoeffi_e::kI] = d;

		LOGGER_WARNING_FMT("_pidCoeff.kP: %f", _pidData.pidCoefficient[pidCoeffi_e::kD]);	

		if(_isEnabled)
			enablePID();
	} /*-------------------------------- end of setD ----------------------------------*/

	void setEF(float ef)
	{
		_pidData.pidCoefficient[pidCoeffi_e::kI] = ef;

		LOGGER_WARNING_FMT("_pidCoeff.kP: %f", _pidData.pidCoefficient[pidCoeffi_e::eF]);	
		
		if(_isEnabled)
			enablePID();
	} /*-------------------------------- end of setEF ----------------------------------*/

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
