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

//Header files
#include "MPU9250.h"

//Library Variable
// 1. I2C Handle 
static I2C_HandleTypeDef i2cHandler;
// 2. Bias corrections for gyro, accelerometer, and magnetometer
static float accBias[3], gyroBias[3], magBias[3], magScale[3];
// 3. Factory mag calibration and mag bias
static float magfactory[3];
// 4. Scale resolutions per LSB for the sensors
static float aRes, gRes, mRes;


// 1. i2c Handler
// Copy I2C CubeMX handle to local library
void MPU9250_Init(I2C_HandleTypeDef *I2Chnd)
{
	memcpy(&i2cHandler, I2Chnd, sizeof(*I2Chnd));
}

// 2. Self test (Only used in development phase)
// Accelerometer and gyroscope self test; check calibration wrt factory settings
// Should return percent deviation from factory trim values, +/- 14 or less
// deviation is a pass.
int MPU9250SelfTest(void)
{
	uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
  uint8_t selfTest[6];
  int32_t aAvg[3] = {0}, gAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
  float factoryTrim[6];
  uint8_t FS = GFS_250DPS;
	uint8_t Dev_ADDR = MPU9250_ADDRESS_AD0 << 1 ;
	uint8_t WriteData;
	
	float testData[6];
	uint8_t result = 0;
	uint8_t pass_count = 0;
	
	WriteData = 0x00;
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, SMPLRT_DIV, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);      // Set sample rate to 1 kHz (only used for 1kHz internal sampling.)
	WriteData = 0x02;
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, CONFIG, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);          // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	WriteData = FS<<3;
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);     // Set full scale range for the gyro to 250 dps
	WriteData = 0x02;
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, ACCEL_CONFIG2, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);   // Set accelerometer rate to 1 kHz and bandwidth to 99 Hz
	WriteData = FS<<3;
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);    // Set full scale range for the accelerometer to 2 g

	// Get average current values of gyro and acclerometer
	// Warning! : Do not move the target during measurement
	for( int ii = 0; ii < 200; ii++)   
	{  
    HAL_I2C_Mem_Read(&i2cHandler, Dev_ADDR, ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, rawData, 6, 100);      // Read the six raw data registers into data array  
    aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
    aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  
		HAL_I2C_Mem_Read(&i2cHandler, Dev_ADDR, GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, rawData, 6, 100);      // Read the six raw data registers sequentially into data array  
    gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
    gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  }
	
	// Get average of 200 values and store as average current readings
  for (int ii =0; ii < 3; ii++)
  {
    aAvg[ii] /= 200;
    gAvg[ii] /= 200;
  }
	
	// Configure the accelerometer for self-test
  // Enable self test on all three axes and set accelerometer range to +/- 2 g
  WriteData = 0xE0;
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);    
  // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  WriteData = 0xE0;
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);    
  HAL_Delay(25);  // Delay a while to let the device stabilize

	// Get average self-test values of gyro and acclerometer
	// Warning! : Do not move the target during measurement.
  for (int ii = 0; ii < 200; ii++)
  {
    // Read the six raw data registers into data array
		HAL_I2C_Mem_Read(&i2cHandler, Dev_ADDR, ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, rawData, 6, 100);      
    // Turn the MSB and LSB into a signed 16-bit value
    aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
    aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    // Read the six raw data registers sequentially into data array
		HAL_I2C_Mem_Read(&i2cHandler, Dev_ADDR, GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, rawData, 6, 100);      
    // Turn the MSB and LSB into a signed 16-bit value
    gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
    gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }
	
	// Get average of 200 values and store as average self-test readings
  for (int ii =0; ii < 3; ii++)
  {
    aSTAvg[ii] /= 200;
    gSTAvg[ii] /= 200;
  }
	
	// Configure the gyro and accelerometer for normal operation
	WriteData = 0x00;
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);    
  WriteData = 0x00;
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);    
  HAL_Delay(25);  // Delay a while to let the device stabilize

	// Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
  // X,Y,Z-axis accel self-test results
	HAL_I2C_Mem_Read(&i2cHandler, Dev_ADDR, SELF_TEST_X_ACCEL, I2C_MEMADD_SIZE_8BIT, &selfTest[0], 3, 100);      
  // X,Y,Z-axis gyro self-test results
	HAL_I2C_Mem_Read(&i2cHandler, Dev_ADDR, SELF_TEST_X_GYRO, I2C_MEMADD_SIZE_8BIT, &selfTest[3], 3, 100);      
  
	// Retrieve factory self-test value from self-test code reads
  // FT[Xa] factory trim calculation
  factoryTrim[0] = (float)(2620/1<<FS)*(pow(1.01 ,((double)selfTest[0] - 1.0) ));
  // FT[Ya] factory trim calculation
  factoryTrim[1] = (float)(2620/1<<FS)*(pow(1.01 ,((double)selfTest[1] - 1.0) ));
  // FT[Za] factory trim calculation
  factoryTrim[2] = (float)(2620/1<<FS)*(pow(1.01 ,((double)selfTest[2] - 1.0) ));
  // FT[Xg] factory trim calculation
  factoryTrim[3] = (float)(2620/1<<FS)*(pow(1.01 ,((double)selfTest[3] - 1.0) ));
  // FT[Yg] factory trim calculation
  factoryTrim[4] = (float)(2620/1<<FS)*(pow(1.01 ,((double)selfTest[4] - 1.0) ));
  // FT[Zg] factory trim calculation
  factoryTrim[5] = (float)(2620/1<<FS)*(pow(1.01 ,((double)selfTest[5] - 1.0) ));
	
	
	// Report results as a ratio of (STR - FT)/FT; the change from Factory Trim
  // of the Self-Test Response
  // To get percent, must multiply by 100
  for (int i = 0; i < 3; i++)
  {
    // Report percent differences
    testData[i] = ((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i];
    // Report percent differences
    testData[i+3] = ((float)(gSTAvg[i] - gAvg[i])) / factoryTrim[i+3];
		// report Self-Test is passing if all the following criteria are fulfilled.
		// If the pass_count is 6, it's the passing. 
		if(testData[i] > 0.5f && testData[i+3] < 1.5f)  pass_count++;
		if(testData[i+3] > 0.5f)                        pass_count++; 						
  }
	
	if(pass_count == 6) 	result = 1;  // The result is 1, Pass  
  else                  result = 0;  // The result is 0, fail
	
	return result;
}

int AK8963SelfTest(void)
{
	RawData_Def magRaw;
	
	uint8_t Dev_ADDR = AK8963_ADDRESS << 1 ;
	uint8_t rawData[7];
	uint8_t WriteData;
	uint8_t readData = 0;
	
	float testData[3];
	uint8_t pass_count = 0;
	uint8_t result = 0;
	
	//--------------------------------------------------------------------------
	WriteData = 0x02;    
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, INT_PIN_CFG, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);  // Bypass Enable
	HAL_Delay(100);
	//--------------------------------------------------------------------------
   
	//--------------------------------------------------------------------------
	getMagFactory();
	//--------------------------------------------------------------------------

	WriteData = 0x00;
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, AK8963_CNTL, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);  // Power down magnetometer
	HAL_Delay(10);
	WriteData = 0x40;
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, AK8963_ASTC, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);  // Generate magnetic field for self-test
	HAL_Delay(10);
	WriteData = 0x18;
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, AK8963_CNTL, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);  // Enter self-test mode , 16bit output
	HAL_Delay(10);

	// Check if data is ready or not by polling the DRDY bit of the ST1 register. 
	// When the data is ready, proceed to next step.
	HAL_I2C_Mem_Read(&i2cHandler, Dev_ADDR, AK8963_ST1, I2C_MEMADD_SIZE_8BIT, &readData, 1, 100);
	while((readData & 0x01) == 0x00)
	{
		HAL_I2C_Mem_Read(&i2cHandler, Dev_ADDR, AK8963_ST1, I2C_MEMADD_SIZE_8BIT, &readData, 1, 100);
	}

	// Read the six raw data and ST2 registers sequentially into data array
	HAL_I2C_Mem_Read(&i2cHandler, Dev_ADDR, AK8963_XOUT_L, I2C_MEMADD_SIZE_8BIT, &rawData[0], 7, 100);
	uint8_t c = rawData[6]; // End data read by reading ST2 register
	// Check if magnetic sensor overflow set, if not then report data
	if (!(c & 0x08))
	{
		// Turn the MSB and LSB into a signed 16-bit value
		magRaw.magCount[0] = ((int16_t)rawData[1] << 8) | rawData[0];
		// Data stored as little Endian
		magRaw.magCount[1] = ((int16_t)rawData[3] << 8) | rawData[2];
		magRaw.magCount[2] = ((int16_t)rawData[5] << 8) | rawData[4];
	}
	
	// sensitivity adjustment of measurement data.
	testData[0] = magRaw.magCount[0] * magfactory[0];
	testData[1] = magRaw.magCount[1] * magfactory[1];
	testData[2] = magRaw.magCount[2] * magfactory[2];
	
	
	WriteData = 0x00;
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, AK8963_ASTC, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);  // magnetic field off
	HAL_Delay(10);
	WriteData = 0x00;
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, AK8963_CNTL, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);  // Power down magnetometer
	HAL_Delay(10);

	//--------------------------------------------------------------------------
	WriteData = 0x00;    
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, INT_PIN_CFG, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);  // Bypass Enable off
	HAL_Delay(100);
	//--------------------------------------------------------------------------

	// report Self-Test is passing if all the following criteria are fulfilled.
	// If the pass_count is 3, it's the passing. 
	if(testData[0] >= -200.f && testData[0] <= 200.f)	  pass_count++;
	if(testData[1] >= -200.f && testData[1] <= 200.f)   pass_count++;	
	if(testData[2] >= -3200.f && testData[2] <= -800.f) pass_count++; 
	
	if(pass_count == 3) 	result = 1;  // The result is 1, Pass  
  else                  result = 0;  // The result is 0, fail
	
	return result;
}

void getMagFactory(void)
{
	uint8_t Dev_ADDR = AK8963_ADDRESS << 1 ;
	uint8_t WriteData;
	
	// First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here
  // TODO: Test this!! Likely doesn't work
  WriteData = 0x00;
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, AK8963_CNTL, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);  // Power down magnetometer
  HAL_Delay(10);
  WriteData = 0x1F;
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, AK8963_CNTL, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);  // Enter Fuse ROM access mode , 16bit output
  HAL_Delay(10);
	
	// Read the x-, y-, and z-axis calibration values
  HAL_I2C_Mem_Read(&i2cHandler, Dev_ADDR, AK8963_ASAX, I2C_MEMADD_SIZE_8BIT, rawData, 3, 100);
  
	// Return x-axis sensitivity adjustment values, etc.
  magfactory[0] =  (float)(rawData[0] - 128)/256 + 1;
  magfactory[1] =  (float)(rawData[1] - 128)/256 + 1;
  magfactory[2] =  (float)(rawData[2] - 128)/256 + 1;
  WriteData = 0x00;
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, AK8963_CNTL, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);  // Power down magnetometer
  HAL_Delay(10);
}

// 3. Get bias of Accel and Gyro
void calibrateMPU9250(void)
{
	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

	uint8_t Dev_ADDR = MPU9250_ADDRESS_AD0 << 1 ;
	uint8_t WriteData;

	// reset device
  // Write a one to bit 7 reset bit; toggle reset device
	WriteData = READ_FLAG;
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);
  HAL_Delay(100);
	
	// get stable time source; Auto select clock source to be PLL gyroscope
  // reference if ready else use the internal oscillator, bits 2:0 = 001
	WriteData = 0x01;
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);
  WriteData = 0x00;
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, PWR_MGMT_2, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);
  HAL_Delay(200);
	
	// Configure device for bias calculation
  // Disable all interrupts
	WriteData = 0x00;
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, INT_ENABLE, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);
  // Disable FIFO
	WriteData = 0x00;
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, FIFO_EN, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);
  // Turn on internal clock source
	WriteData = 0x00;
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);
  // Disable I2C master
	WriteData = 0x00;
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, I2C_MST_CTRL, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);
  // Disable FIFO and I2C master modes
	WriteData = 0x00;
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, USER_CTRL, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);
  // Reset FIFO and DMP
	WriteData = 0x0C;
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, USER_CTRL, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);
  HAL_Delay(15);
	
	// Configure MPU9250 gyro and accelerometer for bias calculation
  // Set low-pass filter to gyro 188 Hz, accel 218 Hz
	WriteData = 0x01;
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, CONFIG, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);
  WriteData = 0x01;
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, ACCEL_CONFIG2, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);
	// Set sample rate to 1 kHz
	WriteData = 0x00;
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, SMPLRT_DIV, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);
  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  WriteData = 0x00;
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);
  // Set accelerometer full-scale to 2 g, maximum sensitivity
  WriteData = 0x00;
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);
	
	uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384; // = 16384 LSB/g
	
	// Configure FIFO to capture accelerometer and gyro data for bias calculation
  WriteData = 0x40;
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, USER_CTRL, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);   // Enable FIFO
  // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in
  // MPU-9150)
	WriteData = 0x78;
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, FIFO_EN, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);   
  HAL_Delay(40);  // accumulate 40 samples in 40 milliseconds = 480 bytes
	
	// At end of sample accumulation, turn off FIFO sensor read
  // Disable gyro and accelerometer sensors for FIFO
  WriteData = 0x00;
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, FIFO_EN, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);  
  // Read FIFO sample count
	HAL_I2C_Mem_Read(&i2cHandler, Dev_ADDR, FIFO_COUNTH, I2C_MEMADD_SIZE_8BIT, &data[0], 2, 100);
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  // How many sets of full gyro and accelerometer data for averaging
  packet_count = fifo_count/12;
	
	 for (ii = 0; ii < packet_count; ii++)
  {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    // Read data for averaging
		HAL_I2C_Mem_Read(&i2cHandler, Dev_ADDR, FIFO_R_W, I2C_MEMADD_SIZE_8BIT, &data[0], 12, 100);
    // Form signed 16-bit integer for each sample in FIFO
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  );
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  );
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  );
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  );
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  );
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]);

    // Sum individual signed 16-bit biases to get accumulated signed 32-bit
    // biases.
    accel_bias[0] += (int32_t) accel_temp[0];
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
  }
	
	// Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
	accel_bias[0] /= (int32_t) packet_count;
  accel_bias[1] /= (int32_t) packet_count;
  accel_bias[2] /= (int32_t) packet_count;
  gyro_bias[0]  /= (int32_t) packet_count;
  gyro_bias[1]  /= (int32_t) packet_count;
  gyro_bias[2]  /= (int32_t) packet_count;
	
	// Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
  if (accel_bias[2] > 0L)
  {
    accel_bias[2] -= (int32_t) accelsensitivity;
  }
  else
  {
    accel_bias[2] += (int32_t) accelsensitivity;
  }
	
	// Construct the gyro biases for push to the hardware gyro bias registers,
  // which are reset to zero upon device startup.
  // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input
  // format.
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF;
  // Biases are additive, so change sign on calculated average gyro biases
  data[1] = (-gyro_bias[0]/4)       & 0xFF;
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;
	
	// Push gyro biases to hardware registers
	WriteData = data[0];
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, XG_OFFSET_H, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);   // Enable FIFO
  WriteData = data[1];
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, XG_OFFSET_L, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);   // Enable FIFO
  WriteData = data[2];
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, YG_OFFSET_H, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);   // Enable FIFO
  WriteData = data[3];
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, YG_OFFSET_L, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);   // Enable FIFO
  WriteData = data[4];
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, ZG_OFFSET_H, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);   // Enable FIFO
  WriteData = data[5];
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, ZG_OFFSET_L, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);   // Enable FIFO
  
	// Output scaled gyro biases for display in the main program
  gyroBias[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
  gyroBias[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  gyroBias[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

  // Construct the accelerometer biases for push to the hardware accelerometer
  // bias registers. These registers contain factory trim values which must be
  // added to the calculated accelerometer biases; on boot up these registers
  // will hold non-zero values. In addition, bit 0 of the lower byte must be
  // preserved since it is used for temperature compensation calculations.
  // Accelerometer bias registers expect bias input as 2048 LSB per g, so that
  // the accelerometer biases calculated above must be divided by 8.
	
	// A place to hold the factory accelerometer trim biases
  int32_t accel_bias_reg[3] = {0, 0, 0};
  // Read factory accelerometer trim values
	HAL_I2C_Mem_Read(&i2cHandler, Dev_ADDR, XA_OFFSET_H, I2C_MEMADD_SIZE_8BIT, &data[0], 2, 100);
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  HAL_I2C_Mem_Read(&i2cHandler, Dev_ADDR, YA_OFFSET_H, I2C_MEMADD_SIZE_8BIT, &data[0], 2, 100);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  HAL_I2C_Mem_Read(&i2cHandler, Dev_ADDR, ZA_OFFSET_H, I2C_MEMADD_SIZE_8BIT, &data[0], 2, 100);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
	
	// Define mask for temperature compensation bit 0 of lower byte of
  // accelerometer bias registers
  uint32_t mask = 1uL;
  // Define array to hold mask bit for each accelerometer bias axis
  uint8_t mask_bit[3] = {0, 0, 0};
	
	for (ii = 0; ii < 3; ii++)
  {
    // If temperature compensation bit is set, record that fact in mask_bit
    if ((accel_bias_reg[ii] & mask))
    {
      mask_bit[ii] = 0x01;
    }
  }
	
	// Construct total accelerometer bias, including calculated average
  // accelerometer bias from above
  // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g
  // (16 g full scale)
  accel_bias_reg[0] -= (accel_bias[0]/8);
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  // preserve temperature compensation bit when writing back to accelerometer
  // bias registers
  data[1] = data[1] | mask_bit[0];
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  // Preserve temperature compensation bit when writing back to accelerometer
  // bias registers
  data[3] = data[3] | mask_bit[1];
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  // Preserve temperature compensation bit when writing back to accelerometer
  // bias registers
  data[5] = data[5] | mask_bit[2];
	
	// Apparently this is not working for the acceleration biases in the MPU-9250
  // Are we handling the temperature correction bit properly?
  // Push accelerometer biases to hardware registers
	WriteData = data[0];
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, XA_OFFSET_H, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);   // Enable FIFO
  WriteData = data[1];
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, XA_OFFSET_L, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);   // Enable FIFO
  WriteData = data[2];
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, YA_OFFSET_H, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);   // Enable FIFO
  WriteData = data[3];
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, YA_OFFSET_L, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);   // Enable FIFO
  WriteData = data[4];
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, ZA_OFFSET_H, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);   // Enable FIFO
  WriteData = data[5];
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, ZA_OFFSET_L, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);   // Enable FIFO

  // Output scaled accelerometer biases for display in the main program
  accBias[0] = (float)accel_bias[0]/(float)accelsensitivity;
  accBias[1] = (float)accel_bias[1]/(float)accelsensitivity;
  accBias[2] = (float)accel_bias[2]/(float)accelsensitivity;
	
}

// 4. MPU9250 Initialaztion Configuration 
void MPU9250_Config(MPU_ConfigTypeDef *config)
{
	uint8_t Dev_ADDR = MPU9250_ADDRESS_AD0 << 1 ;
	uint8_t WriteData;
	
	// wake up device
  // Clear sleep mode bit (6), enable all sensors
	WriteData = 0x00;
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);  
  HAL_Delay(100); // Wait for all registers to reset
	
	// Get stable time source
  // Auto select clock source to be PLL gyroscope reference if ready else
  WriteData = config->ClockSource; // (Auto_best_clk_1 = 0x01)
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);  
  HAL_Delay(200);
	
	// Configure Gyro and Thermometer
  // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz,
  // respectively;
  // minimum delay time for this setting is 5.9 ms, which means sensor fusion
  // update rates cannot be higher than 1 / 0.0059 = 170 Hz
  // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
  // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!),
  // 8 kHz, or 1 kHz
  WriteData = config->CONFIG_DLPF;    // (DLPF_44A_Hz = 0x03)
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, CONFIG, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);  
	
	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  // Use a 200 Hz rate; a rate consistent with the filter update rate
  // determined inset in CONFIG above.
  WriteData = 0x04;    
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, SMPLRT_DIV, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);  
	
	// Set gyroscope full scale range
  // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are
  // left-shifted into positions 4:3
	// get current GYRO_CONFIG register value
  uint8_t c; 
	HAL_I2C_Mem_Read(&i2cHandler, Dev_ADDR, GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &c, 1, 100);
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x03; // Clear Fchoice bits [1:0]        @ modified by Park ('0x02 -> 0x03') @ 
  c = c & ~0x18; // Clear GFS bits [4:3]
  c = c | config->Gscale << 3; // Set full scale range for the gyro (GFS_250DPS = 0x00)
  // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of
  // GYRO_CONFIG
  // c =| 0x00;
  // Write new GYRO_CONFIG value to register
	WriteData = c;    
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);  
	
	// Set accelerometer full-scale range configuration
  // Get current ACCEL_CONFIG register value
  HAL_I2C_Mem_Read(&i2cHandler, Dev_ADDR, ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &c, 1, 100);
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | config->Ascale << 3; // Set full scale range for the accelerometer (AFS_2G 0x00)
  // Write new ACCEL_CONFIG register value
  WriteData = c;    
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);  
	
	// Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by
  // choosing 1 for accel_fchoice_b bit [3]; in this case the bandwidth is
  // 1.13 kHz
  // Get current ACCEL_CONFIG2 register value
  HAL_I2C_Mem_Read(&i2cHandler, Dev_ADDR, ACCEL_CONFIG2, I2C_MEMADD_SIZE_8BIT, &c, 1, 100);
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  c = c | config->ACCEL_DLPF;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  // Write new ACCEL_CONFIG2 register value
  WriteData = c;    
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, ACCEL_CONFIG2, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);  
  // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
  // but all these rates are further reduced by a factor of 5 to 200 Hz because
  // of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH
  // until interrupt cleared, clear on read of INT_STATUS, and enable
  // I2C_BYPASS_EN so additional chips can join the I2C bus and all can be
  // controlled by the Myboard(stm32f4xx) as master.
  WriteData = 0x22;    
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, INT_PIN_CFG, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);  
  // Enable data ready (bit 0) interrupt
  WriteData = 0x01;    
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, INT_ENABLE, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);  
  HAL_Delay(100);
	
	// Set the Accelerometer and Gyroscope Scaling Factor
	// Accelerometer Scaling Factor
	switch (config->Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that
    // 2-bit value:
    case AFS_2G:
      aRes = 2.0f / 32768.0f;
      break;
    case AFS_4G:
      aRes = 4.0f / 32768.0f;
      break;
    case AFS_8G:
      aRes = 8.0f / 32768.0f;
      break;
    case AFS_16G:
      aRes = 16.0f / 32768.0f;
      break;
  }
	// Gyroscope Scaling Factor
	switch (config->Gscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that
    // 2-bit value:
    case GFS_250DPS:
      gRes = 250.0f / 32768.0f;
      break;
    case GFS_500DPS:
      gRes = 500.0f / 32768.0f;
      break;
    case GFS_1000DPS:
      gRes = 1000.0f / 32768.0f;
      break;
    case GFS_2000DPS:
      gRes = 2000.0f / 32768.0f;
      break;
  }
}

// 5. AK8963 Initialaztion Configuration
void AK8963_Config(AK8963_ConfigTypeDef *magConfig)
{
	uint8_t Dev_ADDR = AK8963_ADDRESS << 1 ;
	uint8_t WriteData;
	
	// Configure the magnetometer for continuous read and highest resolution.
  // Set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL
  // register, and enable continuous mode data acquisition Mmode (bits [3:0]),
  // 0010 for 8 Hz and 0110 for 100 Hz sample rates.
	
	// Set magnetometer data resolution and sample ODR
  WriteData = magConfig->Mscale << 4 | magConfig->Mmode;
	HAL_I2C_Mem_Write(&i2cHandler, Dev_ADDR, AK8963_CNTL, I2C_MEMADD_SIZE_8BIT, &WriteData, 1, 100);  
  HAL_Delay(10);
	
	// Gyroscope Scaling Factor
	switch (magConfig->Mscale)
  {
    // Possible magnetometer scales (and their register bit settings) are:
    // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
      mRes = 10.0f * 4912.0f / 8190.0f; // Proper scale to return milliGauss
      break;
    case MFS_16BITS:
      mRes = 10.0f * 4912.0f / 32760.0f; // Proper scale to return milliGauss
      break;
  }
}

// 6. Get bias of Mag (Get Hard iron)
// Function which accumulates gyro and accelerometer data after device
// initialization. It calculates the average of the at-rest readings and then
// loads the resulting offsets into accelerometer and gyro bias registers.
void magCalMPU9250(uint8_t Mmode)
{
	uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3]  = {0, 0, 0},
          mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3]  = {-32768, -32768, -32768},		// Wrote out decimal (signed) values to remove a conversion warning
          mag_min[3]  = {32767, 32767, 32767};
					
  RawData_Def mag_temp;				
	
	printf("Mag Calibration: Wave device in a figure 8 until done!\r\n");
  printf("4 seconds to get ready followed by 15 seconds of sampling\r\n");
	HAL_Delay(4000);          
					
	// shoot for ~fifteen seconds of mag data
  // at 8 Hz ODR, new mag data is available every 125 ms
	if (Mmode == M_8HZ)
  {
    sample_count = 128;
  }
  // at 100 Hz ODR, new mag data is available every 10 ms
  if (Mmode == M_100HZ)
  {
    sample_count = 1500;
  }
	
	for (ii = 0; ii < sample_count; ii++)
  {
    readMagRawData(&mag_temp);  // Read the mag data
		
		printf("tempMx:%d tempMy:%d tempMz:%d\r\n", mag_temp.magCount[0], mag_temp.magCount[1], mag_temp.magCount[2]);

    for (int jj = 0; jj < 3; jj++)
    {
      if (mag_temp.magCount[jj] > mag_max[jj])
      {
        mag_max[jj] = mag_temp.magCount[jj];
      }
      if (mag_temp.magCount[jj] < mag_min[jj])
      {
        mag_min[jj] = mag_temp.magCount[jj];
      }
    }

    if (Mmode == M_8HZ)
    {
      HAL_Delay(135); // At 8 Hz ODR, new mag data is available every 125 ms
    }
    if (Mmode == M_100HZ)
    {
      HAL_Delay(12);  // At 100 Hz ODR, new mag data is available every 10 ms
    }
  }
	
  printf("magX max, min: %d , %d\r\n", mag_max[0], mag_min[0]);
	printf("magY max, min: %d , %d\r\n", mag_max[1], mag_min[1]);
	printf("magZ max, min: %d , %d\r\n", mag_max[2], mag_min[2]);
	
	// Get hard iron correction
  // Get 'average' x mag bias in counts
  mag_bias[0]  = (mag_max[0] + mag_min[0]) / 2;
  // Get 'average' y mag bias in counts
  mag_bias[1]  = (mag_max[1] + mag_min[1]) / 2;
  // Get 'average' z mag bias in counts
  mag_bias[2]  = (mag_max[2] + mag_min[2]) / 2;
	
	// Save mag biases in G for main program
  magBias[0] = (float)mag_bias[0];
  magBias[1] = (float)mag_bias[1];
  magBias[2] = (float)mag_bias[2];

	// Get soft iron correction estimate
  // Get average x axis max chord length in counts
  mag_scale[0]  = (mag_max[0] - mag_min[0]) / 2;
  // Get average y axis max chord length in counts
  mag_scale[1]  = (mag_max[1] - mag_min[1]) / 2;
  // Get average z axis max chord length in counts
  mag_scale[2]  = (mag_max[2] - mag_min[2]) / 2;
	
	float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
  avg_rad /= 3.0f;

  magScale[0] = avg_rad / ((float)mag_scale[0]);
  magScale[1] = avg_rad / ((float)mag_scale[1]);
  magScale[2] = avg_rad / ((float)mag_scale[2]);
}

// 7. Get raw data
void readRawData(RawData_Def *rawData)
{
	uint8_t rawdata[14];
	uint8_t Dev_ADDR = MPU9250_ADDRESS_AD0 << 1;
	uint8_t ReadData;
	
	// Wait for data ready bit to be set
	HAL_I2C_Mem_Read(&i2cHandler, Dev_ADDR, INT_STATUS, I2C_MEMADD_SIZE_8BIT, &ReadData, 1, 10);   
	if(ReadData & 0x01)
	{
		// Read the Accel, Gyro raw data registers into data array
		HAL_I2C_Mem_Read(&i2cHandler, Dev_ADDR, ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, rawdata, 14, 1000);
		
		// Turn the MSB and LSB into a signed 16-bit value
		rawData->accelCount[0] = ((int16_t)rawdata[0] << 8) | rawdata[1] ;
		rawData->accelCount[1] = ((int16_t)rawdata[2] << 8) | rawdata[3] ;
		rawData->accelCount[2] = ((int16_t)rawdata[4] << 8) | rawdata[5] ;
		
		// Turn the MSB and LSB into a signed 16-bit value
		rawData->gyroCount[0] = ((int16_t)rawdata[8] << 8) | rawdata[9] ;
		rawData->gyroCount[1] = ((int16_t)rawdata[10] << 8) | rawdata[11] ;
		rawData->gyroCount[2] = ((int16_t)rawdata[12] << 8) | rawdata[13] ;	
		
		rawData->rawReady = 1;		
	}	
}

void readMagRawData(RawData_Def *magRawData)
{
	// x/y/z gyro register data, ST2 register stored here, must read ST2 at end
  // of data acquisition
  uint8_t rawData[7];
	uint8_t Dev_ADDR = AK8963_ADDRESS << 1 ;
	
	// Read the six raw data and ST2 registers sequentially into data array
	HAL_I2C_Mem_Read(&i2cHandler, Dev_ADDR, AK8963_XOUT_L, I2C_MEMADD_SIZE_8BIT, &rawData[0], 7, 100);
	uint8_t c = rawData[6]; // End data read by reading ST2 register
	// Check if magnetic sensor overflow set, if not then report data
	if (!(c & 0x08))
	{
		// Turn the MSB and LSB into a signed 16-bit value
		magRawData->magCount[0] = ((int16_t)rawData[1] << 8) | rawData[0];
		// Data stored as little Endian
		magRawData->magCount[1] = ((int16_t)rawData[3] << 8) | rawData[2];
		magRawData->magCount[2] = ((int16_t)rawData[5] << 8) | rawData[4];
	}		
}

// 8. Get calibrated data and dt
void readCaliData(float *acc, float *gyro, float *dt, float *bias_acc)
{
	RawData_Def rawData;
	static uint32_t tickLast = 0;
	static uint32_t dt_ms = 0;
	
	readRawData(&rawData);
	
	if(rawData.rawReady)
	{
		acc[0] = (float)rawData.accelCount[0] * aRes - bias_acc[0]; // - accBias[0];
		acc[1] = (float)rawData.accelCount[1] * aRes - bias_acc[1]; // - accBias[1];
		acc[2] = (float)rawData.accelCount[2] * aRes - bias_acc[2]; // - accBias[2];

		acc[0] = (-1.0f)*acc[0];
		acc[1] = (-1.0f)*acc[1];
		acc[2] = (-1.0f)*acc[2];

		gyro[0] = (float)rawData.gyroCount[0] * gRes;  // - gyroBias[0];
		gyro[1] = (float)rawData.gyroCount[1] * gRes;  // - gyroBias[1];
		gyro[2] = (float)rawData.gyroCount[2] * gRes;  // - gyroBias[2];
		
		rawData.rawReady = 0;
		
		// Calculates the time interval between data and data  
		if (tickLast != 0) 
		{
			dt_ms = HAL_GetTick() - tickLast;
			*dt = (float)dt_ms / 1000.f;
		}
		tickLast = HAL_GetTick();
	}
}

void readDMAData(uint8_t *rawdata, float *acc, float *gyro, float *dt, float *bias_acc)
{
	RawData_Def rawData;
	static uint32_t tickLast = 0;
	static uint32_t dt_ms = 0;
	
	rawData.accelCount[0] = ((int16_t)rawdata[0] << 8) | rawdata[1] ;
	rawData.accelCount[1] = ((int16_t)rawdata[2] << 8) | rawdata[3] ;
	rawData.accelCount[2] = ((int16_t)rawdata[4] << 8) | rawdata[5] ;
	
	// Turn the MSB and LSB into a signed 16-bit value
	rawData.gyroCount[0] = ((int16_t)rawdata[8] << 8) | rawdata[9] ;
	rawData.gyroCount[1] = ((int16_t)rawdata[10] << 8) | rawdata[11] ;
	rawData.gyroCount[2] = ((int16_t)rawdata[12] << 8) | rawdata[13] ;	
		
	acc[0] = (float)rawData.accelCount[0] * aRes - bias_acc[0]; // - accBias[0];
	acc[1] = (float)rawData.accelCount[1] * aRes - bias_acc[1]; // - accBias[1];
	acc[2] = (float)rawData.accelCount[2] * aRes - bias_acc[2]; // - accBias[2];

	acc[0] = (-1.0f)*acc[0];
	acc[1] = (-1.0f)*acc[1];
	acc[2] = (-1.0f)*acc[2];

	gyro[0] = (float)rawData.gyroCount[0] * gRes;  // - gyroBias[0];
	gyro[1] = (float)rawData.gyroCount[1] * gRes;  // - gyroBias[1];
	gyro[2] = (float)rawData.gyroCount[2] * gRes;  // - gyroBias[2];
	
	// Calculates the time interval between data and data  
	if (tickLast != 0) 
	{
		dt_ms = HAL_GetTick() - tickLast;
		*dt = (float)dt_ms / 1000.f;
	}
	tickLast = HAL_GetTick();	
}

void readMagCaliData(float *mag)
{
	RawData_Def magRaw;
	readMagRawData(&magRaw);
	
	// 미리 측정해 둔 지자기 Hard iron 값 --------------------
	//   Max Min  중간값
	// X 222 -169 26.5 
	// Y 293 -45  124
	// 다른 값으로도 해보기
	magBias[0] = 17.f;   // 새로 구한 값(지금까지 중에서 가장 정확)
  magBias[1] = 158.f;   
	// ------------------------------------------------
	
	mag[0] = ((float)magRaw.magCount[0] - magBias[0]) * mRes * magfactory[0];
	mag[1] = ((float)magRaw.magCount[1] - magBias[1]) * mRes * magfactory[1];
	mag[2] = ((float)magRaw.magCount[2] - magBias[2]) * mRes * magfactory[2];
	
//	magCaliData->mx *= mBias->magScale[0];
//	magCaliData->my *= mBias->magScale[1];
//	magCaliData->mz *= mBias->magScale[2];
	
	mag[0] = (-1.0f)*mag[0];
	mag[1] = (-1.0f)*mag[1];
	mag[2] = (-1.0f)*mag[2];
}

// 9. Get accel bias
void accelCalibrate(float *bias_acc)
{
	RawData_Def rawData;
	float bias_temp[3] = {0,};
	int set_count = 40;
	
	for (int i = 0; i < set_count; i++)
	{
		readRawData(&rawData);
		
		bias_temp[0] += rawData.accelCount[0];
		bias_temp[1] += rawData.accelCount[1];
		bias_temp[2] += rawData.accelCount[2];
		
	}
	
	bias_acc[0] = bias_temp[0] / set_count;
	bias_acc[1] = bias_temp[1] / set_count;
	bias_acc[2] = bias_temp[2] / set_count;
  
	if (bias_acc[2]  > 0L)
  {
    bias_acc[2]  -= (int32_t) 16384; // = 16384 LSB/g
  }
  else
  {
    bias_acc[2]  += (int32_t) 16384; // = 16384 LSB/g
  }
	
	bias_acc[0] *= aRes;
	bias_acc[1] *= aRes;
	bias_acc[2] *= aRes;
	
}
