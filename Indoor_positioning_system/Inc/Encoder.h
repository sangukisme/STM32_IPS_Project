/*
library name:  Encoder 	
written by: 	 Park	 
Date Written:  4 August 2020	 
Last Modified: 29 September 2020 by Park	
Description: 	 Encoder library.	 
References:    
               - https://wiki.dfrobot.com/Micro_DC_Motor_with_Encoder-SJ01_SKU__FIT0450		

*/

#ifndef _Encoder_H
#define _Encoder_H


#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

#define PI 3.14159265
#define rad (PI / 180)
#define wheel_radius 3
#define robot_radius 5

void pidInit(float kp, float ki, float kd, float setpoint, float sampletime);

bool pidValue(int countLF, int countLB, int countRF, int countRB, uint8_t flag);

void pidCompute(float *output_L, float *output_R, float kp);

void pidSetPWM(float output_L, float output_R, uint8_t flag);

void encoderLocate(float *angle_dt, float *distance_dt);



#endif


