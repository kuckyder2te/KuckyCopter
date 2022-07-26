
/*  File name : dictionary.h
	Project name : KuCo_Phantom 1
	Author: Wilhelm Kuckelsberg
	Date : 2022-07-18
	Description : Constants for PID_adjust
				  
*/
#pragma once

class Dictionary {

public:

    const char *c_pri_select = "Primary axis is select"; ///< Strings for menu and informations
	const char *c_sec_select = "Secondary axis is select";
	const char *c_yaw_select = "YAW axis is select";
	const char *c_ef_primary =    "Exec. frequency primary  =  ";
	const char *c_ef_secondary =  "Exec. frequency secondary = ";
	const char *c_ef_yaw_ =       "Exec. frequency yaw    =    ";

	const char *c_p_select = "Coefficient P = ";
	const char *c_i_select = "Coefficient I = ";
	const char *c_d_select = "Coefficient D = ";

	const char *c_accuracy =   "Accuracy = 1.0  ";
	const char *c_accuracy01 =   "Accuracy = 0.1  ";
	const char *c_accuracy001 =  "Accuracy = 0.01 ";
	const char *c_accuracy0001 = "Accuracy = 0.001";

	const char *c_whitespace = "                           ";
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
};