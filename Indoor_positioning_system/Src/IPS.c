/*
library name:  Indoor Positioning System 	
written by: 	 Park	 
Date Written:  4 August 2020	 
Last Modified: 11 August 2020 by Park	
Description: 	 Indoor Positioning System library.	 
References:    
               - 
							 
*/


#include "IPS.h"

#define PI 3.14159265
static double rad = PI / 180;

// 1. Get the heading_mag
float getAzimuth(float *mag)
{
	double x, y;
	double ret;
	float Azimuth = 0;
  
	// Calcurate the heading_mag 
	// heading_mag(k) = Azimuth
	x = (double) mag[0];
	y = (double) mag[1];
	ret = atan2(y,x);		
	Azimuth = ret / rad;              
	
	return Azimuth;
}

// 2. Get the heading_gyro 
float getYaw(float *gyro, float bias_gyro, float dt)
{
	float Yaw = 0;
	static float gyro_last = 0;
	float gyro_avg = 0;
	
	gyro_avg = (gyro[2] + gyro_last) / 2;
	gyro_last = gyro[2];

	if ( dt > 1.0f ) dt = 0; // 자이로의 측정 간격은 보통 20~30ms이고, 데이터 송신시 100~200ms이다.
												   // ESP 충돌시 발생하는 delay는 무려 12초 정도로 초기화 직후 측정하는 것을 방지하기 위함  
	
	// Calcurate the heading_gyro 
	// heading_gyro(k) = heading_gyro(k-1) + Yaw		      
  Yaw = (gyro_avg - bias_gyro) * dt; 		      

	return Yaw;
} 

// 3. Get the distance_accel
float getDistance(float *accel, float dt)
{
	double x, y;
	float distance = 0;
	float SVM_accel = 0;
	
	// Calcurate the distance_accel 
	// 6 : 가속도-관성 비례 상수 @@(나중에 테스트 하면서 업데이트 하기!)
	x = (double) accel[0];
	y = (double) accel[1];
	SVM_accel = sqrt(pow(x, 2) + pow(y, 2)) * 9.8; 
	distance  = 0.5 * SVM_accel * dt * dt * 100 * 6;   
	
	return distance;
}

// 4. Get pattern of robot movement
int getPattern(float *gyro, float *accel, float dt)
{
	static float Stop_time[2] = {0,};						
	int pattern_state[2] = {0,};
	int pattern = 0;
	float SVM_accel = 0;
		
	// Get gyro pattern
  // GYRO Threshold = 0.0 ~ 2.0
  // Time Threshold = 0.1[s]	
	if (gyro[2] < 2.0f && gyro[2] > 0.0f )   // STOP Value : 0.8 ~ 1.1 
	{			
		Stop_time[0] += dt;
		if (Stop_time[0] > 0.1f)
		{
			pattern_state[0] = GYRO_STOP;        
		}
		else
		{
			pattern_state[0] = GYRO_START;
		}
	}
	else
	{
		Stop_time[0] = 0;
		pattern_state[0] = GYRO_START;
	}
		
	// Get accel pattern 
	// ACCEL Threshold = 0.01
	// Time  Threshold = 0.1[s]
	SVM_accel = sqrt(pow(accel[0], 2) + pow(accel[1], 2));  // STOP Value : 0.000 ~ 0.005
	if (SVM_accel < 0.01f)
	{
		Stop_time[1] += dt;
		if (Stop_time[1] > 0.1f)
		{
			pattern_state[1] = ACCEL_STOP;        
		}
		else
		{
			pattern_state[1] = ACCEL_START;
		}
	}
	else
	{
		Stop_time[1] = 0;
		pattern_state[1] = ACCEL_START;
	}  
	
	// pattern = P_GYRO * P_ACCEL 
	// GYRO_STOP  * ACCEL_STOP  = 3 : P_STOP
	// GYRO_STOP  * ACCEL_START = 4 : P_LINEAR
	// GYRO_START * ACCEL_STOP  = 6 : P_ROTATION
	// GYRO_START * ACCEL_START = 8 : P_NONLINEAR
	pattern = pattern_state[0] * pattern_state[1];

	return pattern;	
}

// 5. Get optimized angle and distance by pattern 
float getPatternData(float *gyro, float *accel, float dt)
{
	int pattern;
	static uint32_t count = 0;
	static float sum_gyro = 0;
	static float bias_gyro = 0;
	float data = 0;
	
	// Get Pattern  
	pattern = getPattern(gyro, accel, dt);  
	
	switch (pattern)
	{
		case P_STOP:
					count++;
					sum_gyro += gyro[2];
					bias_gyro = sum_gyro / count;
					//data = getYaw(gyro, bias_gyro, dt);
					break;
		case P_LINEAR:
					count++;
					sum_gyro += gyro[2];
					bias_gyro = sum_gyro / count;
					//data = getYaw(gyro, bias_gyro, dt);					
					break;
		case P_ROTATION:
					count = 0;
					sum_gyro = 0;
					data = getYaw(gyro, bias_gyro, dt);
					break;
		case P_NONLINEAR:
					count = 0;
					sum_gyro = 0;
					data = getYaw(gyro, bias_gyro, dt);
					break;
		default :
					break;
	}	
	return data;
}

