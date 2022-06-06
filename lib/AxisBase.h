/*
 * AxisBase.h
 *
 *  Created on: 25.12.2018
 *      Author: willy
 */

#ifndef AXISBASE_H_
#define AXISBASE_H_

#include <log4arduino.h>
#include <Arduino.h>
#include <ProcessScheduler/Process.h>
#include "MyPid.h"
#include "Config.h"

namespace modules {

typedef struct{
	int16_t* error;
}baseData_t;

class AxisBase: public Process {

	protected:
		MyPid* 	 		_pid;
		baseData_t*		_baseData;
		static uint8_t 	_instance;
		uint8_t  		_axis_address;	///< Gives everyone axis a title
		int16_t* 		_sp;
		int16_t* 		_fb;
		int16_t* 		_error;
		uint32_t 		_lastMillis;
		uint32_t		_temp;

	public:
		AxisBase(Scheduler &manager, ProcPriority pr, uint32_t period, MyPid* pid): _pid(pid), Process(manager, pr, period){

			 /* If a meaningful value is stored in the EEPROM, the following line must be deactivated.
			 * Factory default. */

			_pid->setExecutionFrequency(PID_FREQUENCY);
			_lastMillis 	 = millis();
			_error			 = 0;
			_sp				 = 0;
			_fb				 = 0;
			_axis_address 	 = _instance++;
		};

		virtual void setup(){

		} /* end of virtual setup */

		virtual void service(){
			/* _sp Position of the joysticks.
			 * _fb Position of the drohne.  */

			if(millis()-_lastMillis >= _pid->getExecutionTime()){	/// muss hier so sein? wird in der service loop immer wieder aufgerufen
				*_error = _pid->step(*_sp, *_fb);					///< Calculate PID error
#ifdef AXISBASE
LOG("Axis Base - Axis %d %s %d %s %d %s %d", _axis_address, " error ",(int16_t)*_error,
										" SP ", (int16_t)*_sp," FB ", (int16_t)*_fb);
#endif

				_lastMillis = millis();
			}
		}

		virtual void onEnable(){
			LOG("%s enable",__PRETTY_FUNCTION__);
		}
		virtual void onDisable(){
			LOG("%s disable",__PRETTY_FUNCTION__);
		}
};
} /* end of namespace modules */
#endif /* AXISBASE_H_ */
/* -------------------------- end of AxisBase Class ---------------------------------------------*/
