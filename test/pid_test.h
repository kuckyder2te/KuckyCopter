#pragma once
/*  File name : pid_test.h
	Project name : KuckyCopter 2
	Author: Stephan Scholz /  Wilhelm Kuckelsberg
	Date : 2022-06-17
	Description : Drohne
*/

#include "..\src\config.h"
#include "..\lib\newPID.h"
#include "..\lib\monitor.h"
#include "..\lib\model.h"

extern model_t model;

NewPID *newPID[3];

void test_setup()
{
}

void test_loop()
{
}
/*------------------------ end of PID test progamm ----------------------------------------------*/