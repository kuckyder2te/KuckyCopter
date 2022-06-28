/*  File name: model.h
	Project name: KuCo_Phantom 1
    Date: 2022-05-31
    Author: Wilhelm Kuckelsberg
	Description: Global constance
*/

#include "performance.h"
//#include "axisBase.h" // auskommentziert 2022-06-25
// #include "AxisYaw.h"
//#include "motorAxis.h"
// #include "Motor.h"
typedef struct {
	sensorData_t sensorData;	// Data from imu and baro
	sonicData_t  sonicData;
	performance_t performance;
	interface_t interface;
//	baseData_t baseData;
	AxisMotor::axisData_t axisData[2];
	yawData_t  yawData;
	batteryData_t batteryData;
//	pidData_t pidData[3];
//	flyState_e flyState;
} model_t;