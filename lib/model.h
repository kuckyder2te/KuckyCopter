/*  File name: model.h
	Project name: KuCo_xxx
    Date: 2022-04.20
    Author: Wilhelm Kuckelsberg

*/

//#include "..\lib\gyro.h"		// müssten die nicht includiert werden?
//#include "..\lib\baro.h"	
//#include "..\lib\sonic.h"
#include "..\lib\pidController.h"	// warum muss die inkludiert werden siehe Zeile 33
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
	sensorData_t sensorData;	// Data from imu and baro
	// gyroData_t gyroData;
	// baroData_t baroData;
	sonicData_t  sonicData;
	performance_t performance;
	interface_t interface;
	pidData_t pidData[3];

/* 	processes::baroData_t  baroData;
	processes::usData_t    usData;
	processes::interface_t interface;
	modules::pidData_t 	   pidData[3];		///< PID coefficient
	modules::yawData_t	   yawData;
	modules::baseData_t	   baseData;
	flyState_e 	 		   flyState; */
} model_t;