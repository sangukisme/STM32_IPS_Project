/*
library name:  Encoder 	
written by: 	 Park	 
Date Written:  4 August 2020	 
Last Modified: 29 September 2020 by Park	
Description: 	 Encoder library.	 
References:    
               - https://wiki.dfrobot.com/Micro_DC_Motor_with_Encoder-SJ01_SKU__FIT0450		

*/

#include "Encoder.h"

static float Kp;
static float Kp_L;   
static float Kp_R;   
static float Ki;
static float Kd;
static float Setpoint;
static float SampleTime;
static int PWM_Max;
static int PWM_Min;
static uint32_t PWM;
	
static float dt;
static float CountL_dt;
static float CountR_dt;
static float angle_L_v;
static float angle_R_v;
static float L_v;
static float R_v;

static float flag_L;
static float flag_R;
static int count_L;
static int count_R;
static float errSum_L;
static float errSum_R;
static float dErr_L;
static float dErr_R;

void pidInit(float kp, float ki, float kd, float setpoint, float sampletime)
{
	Kp = kp;
	Kp_L = kp;
	Kp_R = kp;
	Ki = ki;
	Kd = kd;
	Setpoint = setpoint;
  SampleTime = sampletime;  
  
	/** PWM Set Table (바닥 상태에 따라 다르다! 최대 100이상 차이)
	num		Set   PWM 
	1			50    1000
	2			60    1050
	3			70    1100
	4			80    1150
	n			A	  1100 + 3 * (A - 50)
	**/
	//PWM = 1100 + 5 * (setpoint - 50);
	PWM = 1050;
  TIM3->CCR2 = PWM;
  TIM3->CCR3 = PWM;
	PWM_Max = 1400;
  PWM_Min = 900;
}

bool pidValue(int countLF, int countLB, int countRF, int countRB, uint8_t flag)
{
	static int tempL[2] = {0,};
	static int tempR[2] = {0,};
	float CountLF_dt = 0;
	float CountLB_dt = 0;
	float CountRF_dt = 0;
  float	CountRB_dt = 0;
	static uint32_t tickLast = 0;	
	static float dt_sum = 0;
	uint32_t dt_ms = 0;
	float dt_s = 0;
	
 	if (tickLast != 0) 
	{
		dt_ms = HAL_GetTick() - tickLast;
		dt_s = (float)dt_ms / 1000.f;
		dt_sum += dt_s;
	}
	tickLast = HAL_GetTick();
	
	if (dt_sum > SampleTime)
	{
		dt = dt_sum;
		dt_sum = 0;	
		
		if (tempL[0] != 0) CountLF_dt = countLF - tempL[0];    
		if (tempL[1] != 0) CountLB_dt = countLB - tempL[1];   
		tempL[0] = countLF;
		tempL[1] = countLB;
		if (CountLF_dt >= CountLB_dt) CountL_dt = CountLF_dt;
    else 													CountL_dt = -1 * CountLB_dt;		
		
    if (tempR[0] != 0) CountRF_dt = countRF - tempR[0];    
		if (tempR[1] != 0) CountRB_dt = countRB - tempR[1];   
		tempR[0] = countRF;
		tempR[1] = countRB;
		if (CountRF_dt >= CountRB_dt) CountR_dt = CountRF_dt;
		else												  CountR_dt = -1 * CountRB_dt;
		
		if (flag == 6)    // 6 : STATE_STOP
		{
			flag_L = 0;     // PID제어의 ID는 안정화 진입후 사용! (초기 error값은 P만으로 제어)
			flag_R = 0;     // PID제어의 ID는 안정화 진입후 사용! (초기 error값은 P만으로 제어)
			errSum_L = 0;   // I제어 값 초기화
			errSum_R = 0; 	// I제어 값 초기화
			dErr_L = 0;			// D제어 값 초기화						
			dErr_R = 0;			// D제어 값 초기화		
			count_L = 0;    // I제어 스택 초기화
			count_R = 0; 	  // I제어 스택 초기화
			Kp_L = Kp;  		// P제어 게인 값 초기화
			Kp_R = Kp;      // P제어 게인 값 초기화	
			TIM3->CCR2 = PWM; // 항상 동일한 속도로 출발
			TIM3->CCR3 = PWM; // 항상 동일한 속도로 출발
		}
		return true;
	}	
	return false;
}

void pidCompute(float *output_L, float *output_R, float kp)
{
	float input = 0;
  float error = 0;
	static float lastErr_L = 0;
	static float lastErr_R = 0;
	
	/** output L compute **/
	angle_L_v = ( CountL_dt / 1.07 ) / dt;
	L_v = wheel_radius * angle_L_v * rad;
	input = fabs(L_v);
	error = Setpoint - input;
	if ( fabs(error) <= 3 )
	{
		flag_L = 1;
		Kp_L = kp;
	}
	if (flag_L)
	{
		count_L++;
		if(count_L > 500) errSum_L = 0;
		errSum_L += (error * dt);
		dErr_L = (error - lastErr_L); 
	}
	*output_L = (Kp_L * error) + (Ki * errSum_L) + (Kd * dErr_L);
	lastErr_L = error;
	
	/** output R compute **/
	angle_R_v = ( CountR_dt / 1.07 ) / dt;
	R_v = wheel_radius * angle_R_v * rad;
	input = fabs(R_v);
	error = Setpoint - input;
	if ( fabs(error) <= 3 )
	{
		flag_R = 1;
		Kp_R = kp;
	}
	if (flag_R)
	{
		count_R++;
		if(count_R > 500) errSum_R = 0;
		errSum_R += (error * dt);
		dErr_R = (error - lastErr_R); 
	}
	*output_R = (Kp_R * error) + (Ki * errSum_R) + (Kd * dErr_R);
	lastErr_R = error;
}

void pidSetPWM(float output_L, float output_R, uint8_t flag)
{
	if(fabs(output_L) > 5) output_L = 5 * (output_L / fabs(output_L));  // 한번에 제어할 수 있는 최대 PWM값은 5
	if(fabs(output_R) > 5) output_R = 5 * (output_R / fabs(output_R));  // 한번에 제어할 수 있는 최대 PWM값은 5
	
	// PWM Control 	
	if (flag == 4) output_L = 0; 			// 4 : STATE_LEFT
	else if (flag == 5) output_R = 0; // 5 : STATE_RIGHT
	else if (flag == 0 || flag == 6 || flag == 7)  return; // 0 : STATE_READY, 6 : STATE_STOP, 7 : STATE_ERR
	
	if (TIM3->CCR2 + output_L > PWM_Max)			TIM3->CCR2 = PWM_Max;
	else if (TIM3->CCR2 + output_L < PWM_Min) TIM3->CCR2 = PWM_Min;    		
	else 																			TIM3->CCR2 += (int)output_L;

	if (TIM3->CCR3 + output_R > PWM_Max) 			TIM3->CCR3 = PWM_Max;
	else if (TIM3->CCR3 + output_R < PWM_Min) TIM3->CCR3 = PWM_Min;    		
	else 																			TIM3->CCR3 += (int)output_R;	
}

// Caculate Position 
void encoderLocate(float *angle_dt, float *distance_dt)
{
	float robot_v = (L_v + R_v) / 2;
	float robot_angle_v = ((R_v - L_v) / (2 * robot_radius)) / rad;
	
	if (dt > 1.0f) dt = 0; // 엔코더의 측정 간격은 보통 20~30ms이고, 데이터 송신시 100~200ms이다.
												 // ESP 충돌시 발생하는 delay는 무려 12초 정도로 초기화 직후 측정하는 것을 방지하기 위함
	
	*distance_dt = robot_v * dt;
	*angle_dt = robot_angle_v * dt; 
}
