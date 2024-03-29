
/*  File name : dictionary.h
	Project name : KuCo_Phantom 2
	Author: Wilhelm Kuckelsberg
	Date : 2022-07-18
	Description : Constants for PID_adjust
				  
*/
#pragma once

class Dictionary {

public:

    const char *c_axis_pri_select = "Primary axis is select"; 	// choose the axis and Exe. frequency
	const char *c_axis_sec_select = "Secondary axis is select";
	const char *c_axis_yaw_select = "YAW axis is select";
	const char *c_axis_ef_select  = "Ex. Freq. is select";

	const char *c_axis_main_ef_select = "Set the value for Exe. Freq.";
	const char *c_axis_pri_ef_select  = "          pri axis = ";
	const char *c_axis_sec_ef_select  = "          sec axis = ";
	const char *c_axis_yaw_ef_select  = "          yaw axis = ";

	const char *c_ef_primary =   "Value pri. =";
	const char *c_ef_secondary = "Value sec. =";
	const char *c_ef_yawZ =      "Value yaw  =";  // das große 'Z' ist nur eine schnelle Lösung

	const char *c_p_coeff =  "Coefficient P = ";
	const char *c_i_coeff =  "Coefficient I = ";
	const char *c_d_coeff =  "Coefficient D = ";
	const char *c_ef_coeff = "Coefficient eF = ";

	const char *c_accuracy =   "Accuracy = 1.0  ";
	const char *c_accuracy01 =   "Accuracy = 0.1  ";
	const char *c_accuracy001 =  "Accuracy = 0.01 ";
	const char *c_accuracy0001 = "Accuracy = 0.001";

	const char *c_whitespace = " ";
	const char *c_primary_p = "Primary axis   P = ";
	const char *c_primary_i = "               I = ";
	const char *c_primary_d = "               D = ";
	const char *c_secondary_p = "Secondary axis P = ";
	const char *c_secondary_i = "               I = ";
	const char *c_secondary_d = "               D = ";
	const char *c_yaw_p = "YAW axis       P = ";
	const char *c_yaw_i = "               I = ";
	const char *c_yaw_d = "               D = ";
	const char *c_ef_pri = "Exec. freq. Pri  = ";
	const char *c_ef_sec = "            Sec. = ";
	const char *c_ef_yaw = "            YAW  = ";

	const char *c_current = "<-current";

	const char *c_date = __DATE__;
	const char *c_time = __TIME__;
};