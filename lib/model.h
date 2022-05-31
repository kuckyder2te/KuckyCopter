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
typedef struct {
	sensorData_t sensorData;	// Data from imu and baro
	// gyroData_t gyroData;
	// baroData_t baroData;
	sonicData_t  sonicData;
	performance_t performance;
	interface_t interface;
	pidData_t pidData[3];
} model_t;