/*
library name:  Indoor Positioning System 	
written by: 	 Park	 
Date Written:  4 August 2020	 
Last Modified: 11 August 2020 by Park	
Description: 	 Indoor Positioning System library.	 
References:    
               - 		

*/


#ifndef _IPS_HAL_H
#define _IPS_HAL_H

//Header Files
#include "stm32f4xx_hal.h"  //depending on your board
#include <stdio.h>
#include <math.h>			//Pow() 

#define ANGLE        'a'
#define DISTANCE     'd' 

#define GYRO_STOP    1
#define GYRO_START   2

#define ACCEL_STOP   3
#define ACCEL_START  4

#define P_STOP       3 // GYRO_STOP * ACCEL_STOP   = 3
#define P_LINEAR     4 // GYRO_STOP * ACCEL_START  = 4
#define P_ROTATION   6 // GYRO_START * ACCEL_STOP = 6
#define P_NONLINEAR  8 // GYRO_START * ACCEL_START = 8


// 1. Get the heading_mag
float getAzimuth(float *mag);
// 2. Get the heading_gyro
float getYaw(float *gyro, float bias_gyro, float dt);
// 3. Get the distance_accel
float getDistance(float *accel, float dt);
// 4. Get pattern 
int getPattern(float *gyro, float *accel, float dt);  
// 5. Get optimized angle and distance by pattern 
float getPatternData(float *gyro, float *accel, float dt);

#endif /* _IPS_HAL_H */

