/*
library name:  MPU9250 9 axis module 	
written by: 	 Park	 
Date Written:  4 April 2020	 
Last Modified: 11 August 2020 by Park	
Description: 	 MPU9250 Module Basic Functions Device Driver library that use HAL libraries.	 
References:    
               - SparkFun_MPU-9250_Breakout_Arduino_Library: https://github.com/sparkfun/SparkFun_MPU-9250_Breakout_Arduino_Library			
               - MPU9250 Registers map: https://invensense.tdk.com/wp-content/uploads/2017/11/RM-MPU-9250A-00-v1.6.pdf  

*/


#ifndef _MPU9250_H
#define _MPU9250_H


//Header Files
#include "stm32f4xx_hal.h"  //dpending on your board
#include "stdio.h"
#include <string.h>
#include <stdbool.h>	//Boolean
#include <math.h>			//Pow() 

#define SERIAL_DEBUG true

//Magnetometer Registers
#define WHO_AM_I_AK8963  0x00 // (AKA WIA) should return 0x48
#define INFO             0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L    0x03  // data
#define AK8963_XOUT_H    0x04
#define AK8963_YOUT_L    0x05
#define AK8963_YOUT_H    0x06
#define AK8963_ZOUT_L    0x07
#define AK8963_ZOUT_H    0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define SELF_TEST_X_GYRO 0x00
#define SELF_TEST_Y_GYRO 0x01
#define SELF_TEST_Z_GYRO 0x02

/*#define X_FINE_GAIN      0x03 // [7:0] fine gain
#define Y_FINE_GAIN      0x04
#define Z_FINE_GAIN      0x05
#define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC   0x07
#define YA_OFFSET_H      0x08
#define YA_OFFSET_L_TC   0x09
#define ZA_OFFSET_H      0x0A
#define ZA_OFFSET_L_TC   0x0B */

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A       0x10

#define XG_OFFSET_H       0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L       0x14
#define YG_OFFSET_H       0x15
#define YG_OFFSET_L       0x16
#define ZG_OFFSET_H       0x17
#define ZG_OFFSET_L       0x18
#define SMPLRT_DIV        0x19
#define CONFIG            0x1A
#define GYRO_CONFIG       0x1B
#define ACCEL_CONFIG      0x1C
#define ACCEL_CONFIG2     0x1D
#define LP_ACCEL_ODR      0x1E
#define WOM_THR           0x1F

// Duration counter threshold for motion interrupt generation, 1 kHz rate,
// LSB = 1 ms
#define MOT_DUR           0x20
// Zero-motion detection threshold bits [7:0]
#define ZMOT_THR          0x21
// Duration counter threshold for zero motion interrupt generation, 16 Hz rate,
// LSB = 64 ms
#define ZRMOT_DUR         0x22

#define FIFO_EN            0x23
#define I2C_MST_CTRL       0x24
#define I2C_SLV0_ADDR      0x25
#define I2C_SLV0_REG       0x26
#define I2C_SLV0_CTRL      0x27
#define I2C_SLV1_ADDR      0x28
#define I2C_SLV1_REG       0x29
#define I2C_SLV1_CTRL      0x2A
#define I2C_SLV2_ADDR      0x2B
#define I2C_SLV2_REG       0x2C
#define I2C_SLV2_CTRL      0x2D
#define I2C_SLV3_ADDR      0x2E
#define I2C_SLV3_REG       0x2F
#define I2C_SLV3_CTRL      0x30
#define I2C_SLV4_ADDR      0x31
#define I2C_SLV4_REG       0x32
#define I2C_SLV4_DO        0x33
#define I2C_SLV4_CTRL      0x34
#define I2C_SLV4_DI        0x35
#define I2C_MST_STATUS     0x36
#define INT_PIN_CFG        0x37
#define INT_ENABLE         0x38
#define DMP_INT_STATUS     0x39  // Check DMP interrupt
#define INT_STATUS         0x3A
#define ACCEL_XOUT_H       0x3B
#define ACCEL_XOUT_L       0x3C
#define ACCEL_YOUT_H       0x3D
#define ACCEL_YOUT_L       0x3E
#define ACCEL_ZOUT_H       0x3F
#define ACCEL_ZOUT_L       0x40
#define TEMP_OUT_H         0x41
#define TEMP_OUT_L         0x42
#define GYRO_XOUT_H        0x43
#define GYRO_XOUT_L        0x44
#define GYRO_YOUT_H        0x45
#define GYRO_YOUT_L        0x46
#define GYRO_ZOUT_H        0x47
#define GYRO_ZOUT_L        0x48
#define EXT_SENS_DATA_00   0x49
#define EXT_SENS_DATA_01   0x4A
#define EXT_SENS_DATA_02   0x4B
#define EXT_SENS_DATA_03   0x4C
#define EXT_SENS_DATA_04   0x4D
#define EXT_SENS_DATA_05   0x4E
#define EXT_SENS_DATA_06   0x4F
#define EXT_SENS_DATA_07   0x50
#define EXT_SENS_DATA_08   0x51
#define EXT_SENS_DATA_09   0x52
#define EXT_SENS_DATA_10   0x53
#define EXT_SENS_DATA_11   0x54
#define EXT_SENS_DATA_12   0x55
#define EXT_SENS_DATA_13   0x56
#define EXT_SENS_DATA_14   0x57
#define EXT_SENS_DATA_15   0x58
#define EXT_SENS_DATA_16   0x59
#define EXT_SENS_DATA_17   0x5A
#define EXT_SENS_DATA_18   0x5B
#define EXT_SENS_DATA_19   0x5C
#define EXT_SENS_DATA_20   0x5D
#define EXT_SENS_DATA_21   0x5E
#define EXT_SENS_DATA_22   0x5F
#define EXT_SENS_DATA_23   0x60
#define MOT_DETECT_STATUS  0x61
#define I2C_SLV0_DO        0x63
#define I2C_SLV1_DO        0x64
#define I2C_SLV2_DO        0x65
#define I2C_SLV3_DO        0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL    0x69
#define USER_CTRL          0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1         0x6B  // Device defaults to the SLEEP mode
#define PWR_MGMT_2         0x6C
#define DMP_BANK           0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT         0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG            0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1          0x70
#define DMP_REG_2          0x71
#define FIFO_COUNTH        0x72
#define FIFO_COUNTL        0x73
#define FIFO_R_W           0x74
#define WHO_AM_I_MPU9250   0x75  // Should return 0x71
#define XA_OFFSET_H        0x77
#define XA_OFFSET_L        0x78
#define YA_OFFSET_H        0x7A
#define YA_OFFSET_L        0x7B
#define ZA_OFFSET_H        0x7D
#define ZA_OFFSET_L        0x7E

// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
// The previous preprocessor directives were sensitive to the location that the user defined AD1
// Now simply define MPU9250_ADDRESS as one of the two following depending on your application
#define MPU9250_ADDRESS_AD1 0x69  // Device address when ADO = 1
#define MPU9250_ADDRESS_AD0 0x68  // Device address when ADO = 0
#define AK8963_ADDRESS  0x0C   // Address of magnetometer
#define READ_FLAG 0x80


typedef struct
{
	uint8_t ClockSource;
	uint8_t CONFIG_DLPF;     
	uint8_t ACCEL_DLPF;      // A_DLPF_CFG set, ACCEL_CONFIG_REG2 bit[1:0]
	uint8_t Ascale; 
	uint8_t Gscale;
	bool 		Sleep_Mode_Bit;
}MPU_ConfigTypeDef;

enum PM_CLKSEL_ENUM
{
	Internal_20MHz_1 	= 0x00,
	Auto_best_clk_1		= 0x01,  // Auto selects the best available clock source ? PLL if ready, else use the Internal oscillator
	Auto_best_clk_2		= 0x02,  // Auto selects the best available clock source ? PLL if ready, else use the Internal oscillator
	Auto_best_clk_3		= 0x03,  // Auto selects the best available clock source ? PLL if ready, else use the Internal oscillator
	Auto_best_clk_4		= 0x04,  // Auto selects the best available clock source ? PLL if ready, else use the Internal oscillator
	Auto_best_clk_5		= 0x05,  // Auto selects the best available clock source ? PLL if ready, else use the Internal oscillator
	Internal_20MHz_2	= 0x06,
	Stop_clk          = 0x07,  // Stops the clock and keeps timing generator in reset
};

enum DLPF_CFG_ENUM
{
	DLPF_250G_4000T_Hz    = 0x00,
	DLPF_184G_188T_Hz    = 0x01,
	DLPF_92G_98T_Hz 	  = 0x02,
	DLPF_41G_42T_Hz 	  = 0x03,
	DLPF_20G_20T_Hz 	  = 0x04,
	DLPF_10G_10T_Hz 		= 0x05,
	DLPF_5G_5T_Hz 			= 0x06
};

enum ACCEL_DLPF_CFG_ENUM
{
	DLPF_218A_Hz_1  = 0x00,
	DLPF_218A_Hz_2  = 0x01,
	DLPF_99A_Hz 	  = 0x02,
	DLPF_44A_Hz 	  = 0x03,
	DLPF_21A_Hz 	  = 0x04,
	DLPF_10A_Hz 		= 0x05,
	DLPF_5A_Hz 			= 0x06
};

enum accel_FullScale
{
	AFS_2G = 0,
	AFS_4G,
	AFS_8G,
	AFS_16G
};

enum gyro_FullScale 
{
	GFS_250DPS = 0,
	GFS_500DPS,
	GFS_1000DPS,
	GFS_2000DPS
};

typedef struct
{
	uint8_t Mscale;
	uint8_t Mmode;
}AK8963_ConfigTypeDef;

enum M_scale 
{
	MFS_14BITS = 0, // 0.6 mG per LSB
	MFS_16BITS      // 0.15 mG per LSB
};

enum M_MODE 
{
	M_8HZ = 0x02,  // 8 Hz update
	M_100HZ = 0x06 // 100 Hz continuous magnetometer
};

// Raw Data for gyro, accelerometer, and magnetometer
typedef struct
{
	int16_t accelCount[3];   
  int16_t gyroCount[3];    
  int16_t magCount[3]; 
	int8_t rawReady;
}RawData_Def;			
 
// Function Prototype
// 1. i2c Handler
void MPU9250_Init(I2C_HandleTypeDef *I2Chnd); 
// 2. Self test (Only used in development phase)
int MPU9250SelfTest(void);   
int AK8963SelfTest(void);    
void getMagFactory(void);  
// 3. Get bias of Accel and Gyro
void calibrateMPU9250(void);              
// 4. MPU9250 Initialaztion Configuration 
void MPU9250_Config(MPU_ConfigTypeDef *config);
// 5. AK8963 Initialaztion Configuration
void AK8963_Config(AK8963_ConfigTypeDef *magConfig);
// 6. Get bias of Mag (Get Hard iron)
void magCalMPU9250(uint8_t Mmode);
// 7. Get raw data
void readRawData(RawData_Def *rawData);
void readMagRawData(RawData_Def *magRawData);
// 8. Get calibrated data and dt
void readCaliData(float *acc, float *gyro, float *dt, float *bias_acc);
void readDMAData(uint8_t *rawdata, float *acc, float *gyro, float *dt, float *bias_acc);
void readMagCaliData(float *mag);
// 9. Get accel bias
void accelCalibrate(float *bias_acc);

#endif /* _MPU9250_H */
