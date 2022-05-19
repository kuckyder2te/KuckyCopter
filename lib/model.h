/*  File name: model.h
	Project name: KuCo_xxx
    Date: 2022-04.20
    Author: Wilhelm Kuckelsberg

*/

//#include "..\lib\gyro.h"		// müssten die nicht includiert werden?
//#include "..\lib\baro.h"
//#include "..\lib\sonic.h"
#include "performance.h"

typedef enum {
	arming = 0,		///< When the Kuckycopter is first turned on, the arming starts.
	arming_busy,
	disablePID,
	standby,		///< All motors on POWER_MIN
	prestart,		///< All motors on standby and ready to fly. (POWER_MIN)
	takeoff,		///< The Quadrocopter takes off.
	set_pid,		///< Fly without PID-Output = 0
	fly,			///< Normal fly mode
	ground			///< Kuckycopter stand on the ground
} flyState_e;


typedef struct {
	gyroData_t  gyroData;		///< Sensors
	sonicData_t  sonicData;
	performance_t performance;
	baroData_t	baroData;
		interface_t interface;

/* 	processes::baroData_t  baroData;
	processes::usData_t    usData;
	processes::interface_t interface;
	modules::pidData_t 	   pidData[3];		///< PID coefficient
	modules::yawData_t	   yawData;
	modules::baseData_t	   baseData;
	flyState_e 	 		   flyState; */
} model_t;