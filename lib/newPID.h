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

#include "def.h"
#include <FastPID.h>
#include "myLogger.h"

typedef struct {
	float pidCoefficient[3];
	float executionFrequency;
	int   output_bits;
	bool  output_signed;
} pidData_t;

typedef enum {
	kP,
	kI,
	kD
}coeffizient_t;

class NewPID {			/// MyPid is an FastPit

private:
	float 	  	_kP;
	float       _kI;
	float       _kD;
	float		_exFreq;
	uint8_t     _EEPROM_startAddress;
	uint8_t 	_pidInstance;

protected:
	pidData_t     *_pidData;				/// MyPid has a PidData
	static uint8_t _instance;
	float          RC_SP;
	float          FB;

public:
		 	// setOutputRange(-100, 100);
			// _pidInstance =_instance++;
			// setOutputConfig(16, true);
			// disablePID();
		

	void disablePID(){

		LOGGER_NOTICE_FMT("Disabled PID controller %d ", _pidInstance);

//		FastPID::setCoefficients(PID_P_MIN, 0.0, 0.0, getExecutionTime());

	}//-------------------------------- end of deactivatePID --------------------------------------

	void enablePID(){
		/* This function has 2 tasks.
		 * 1. The PID parameters are uploaded from the PID adjustment.
		 * 2. The PID parameters are activated. */
	

}//-------------------------------- end of activatePID --------------------------------------------

	void setP( float p){
		
		LOGGER_NOTICE_FMT("setP: %d",p);
		
		_kP = p;
		if(_kP <= PID_P_MIN)
			_kP = PID_P_MIN;

//		_pidData.pidCoefficient[coeffizient_t::kP]=_kP;


		enablePID();

	}//-------------------------------- end of setP -----------------------------------------------

	void setI( float i){
		
		LOGGER_NOTICE_FMT("setI: %d",i);
		
		_kI = i;
		if(_kI <= 0)
			_kI = 0;

	//	_pidData.pidCoefficient[coeffizient_t::kI]=_kI;
	

		enablePID();

	}//-------------------------------- end of setI -----------------------------------------------

	void setD( float d){
		
		LOGGER_NOTICE_FMT("setD: %d",d);
		
		_kD = d;
		if(_kD <= 0)
			_kD = 0;

	//	_pidData.pidCoefficient[coeffizient_t::kD]=_kD;
	

		enablePID();

	}//-------------------------------- end of setD -----------------------------------------------
	void setExecutionFrequency(uint8_t ef){
		
		LOGGER_WARNING_FMT("setExecutionFrequency: %d", ef);
		

	//	_pidData.executionFrequency = ef;
	
		
		enablePID();

	}//-------------------------------- end of setExecutionFrequency ------------------------------
	uint8_t getExecutionTime(){
		
		LOGGER_WARNING_FMT("PID getExecutionTime %d", (1/_pidData->executionFrequency)*1000);
		

	//	return ((1.0/(float)_pidData.executionFrequency)*1000);
		///< Convert frequency to millis

	}//-------------------------------- end of getExecutionTime -----------------------------------
	void updateEEPROM(void){
	//	EEPROM.put(_EEPROM_startAddress, _pidData);
		enablePID();

	}//-------------------------------- end of updateEEPROM ---------------------------------------
	void readEEPROM(void){
	
		enablePID();

	}//-------------------------------- end of readEEPROM -----------------------------------------
	float getP() const {
		return _kP;

	}//-------------------------------- end of getP -----------------------------------------------
	float getI() const {
		return _kI;

	}//-------------------------------- end of getI -----------------------------------------------
	float getD() const {
		return _kD;

	}//-------------------------------- end of getD -----------------------------------------------
	float getExFreq() const {
	//	return _pidData.executionFrequency;

	}//-------------------------------- end of getExTime ------------------------------------------

};

/*--------------------------- end of MyPid class ------------------------------------------------*/
