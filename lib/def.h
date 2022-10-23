/*  File name: def.h
    Project name: KuCo_Phantom 1
    Date: 2022-05-31
    Author: Wilhelm Kuckelsberg
    Description: Global names
*/
#pragma once

typedef enum
{
    primary = 0,
    secondary,
    yaw
} axisName_e; // bezeichnet die tatsächlichen Namen der 3 Achsen

typedef enum
{
    kP = 0,
    kI,
    kD
} pidCoeffi_e;
