#ifndef __MPU6050_H__
#define __MPU6050_H__

#include <stdint.h>
#include <stddef.h>
#include <assert.h>

#include "main.h"

typedef enum{
	MPU6050_Status_Ok = 0x00,
	MPU6050_Status_Error,
	MPU6050_Status_DeviceNC,
	MPU6050_Status_DeviceInvalid,
	MPU6050_Status_Timeout
}MPU6050_Status;

typedef enum{
	MPU6050_Device_0 = 0x00,	/*!< AD0 pin is set to low */
	MPU6050_Device_1 = 0x01		/*!< AD0 pin is set to high */
}MPU6050_Device;

typedef enum{
	MPU6050_Accelerometer_2g 	= 0x00,
	MPU6050_Accelerometer_4g	= 0x01,
	MPU6050_Accelerometer_8g 	= 0x02,
	MPU6050_Accelerometer_16g	= 0x03
}MPU6050_Accelerometer;

typedef enum{
	MPU6050_Gyroscope_250s 		= 0x00,
	MPU6050_Gyroscope_500s 		= 0x01,
	MPU6050_Gyroscope_1000s 	= 0x02,
	MPU6050_Gyroscope_2000s 	= 0x03
}MPU6050_Gyroscope;

typedef enum{
	MPU6050_SampleRate_100Hz 	= 79,
	MPU6050_SampleRate_125Hz 	= 63,
	MPU6050_SampleRate_250Hz 	= 31,
	MPU6050_SampleRate_500Hz 	= 15,
	MPU6050_SampleRate_1KHz 	= 7,
	MPU6050_SampleRate_2KHz		= 3,
	MPU6050_SampleRate_4KHz		= 1,
	MPU6050_SampleRate_8KHz		= 0
}MPU6050_SampleRate;

typedef enum{
	MPU6050_DLPF_0 = 0,
	MPU6050_DLPF_1 = 1,
	MPU6050_DLPF_2 = 2,
	MPU6050_DLPF_3 = 3,
	MPU6050_DLPF_4 = 4,
	MPU6050_DLPF_5 = 5,
	MPU6050_DLPF_6 = 6
}MPU6050_DLPF;

typedef enum{
	MPU6050_InterruptFlag_FIFO_OFLOW_EN = 0x10,
	MPU6050_InterruptFlag_I2C_MST_INT_EN = 0x08,
	MPU6050_InterruptFlag_DATA_RDY_EN = 0x01
}MPU6050_InterruptFlag;

typedef struct
{
	MPU6050_Device device;
	MPU6050_SampleRate sample_rate;
	MPU6050_Accelerometer accelerometer;
	MPU6050_Gyroscope gyroscope;
	MPU6050_DLPF dlpf;
}MPU6050_Configuraton;

typedef struct
{
	I2C_TypeDef*i2c_handler;
	uint8_t dev_address;
	MPU6050_Device device;
	MPU6050_Configuraton config;
	MPU6050_Status status;
}mpu6050_t;


typedef struct {
	float x;
	float y;
	float z;
}mpu6050_accel_output;


typedef struct {
	float x;
	float y;
	float z;
}mpu6050_gyro_output;

typedef struct{
	mpu6050_accel_output accel;
	mpu6050_gyro_output gyro;
	float temp;
}mpu6050_outputs;


MPU6050_Status mpu6050_get_status(mpu6050_t *hmpu);
void mpu6050_reset_state(mpu6050_t *hmpu);

MPU6050_Status mpu6050_init(mpu6050_t *hmpu, I2C_TypeDef*hi2c,  MPU6050_Configuraton configuration);
MPU6050_Status mpu6050_start(mpu6050_t *hmpu);

MPU6050_Status mpu6050_set_sample_rate(mpu6050_t *hmpu, MPU6050_SampleRate sampleRate);
MPU6050_Status mpu6050_get_sample_rate(mpu6050_t *hmpu, MPU6050_SampleRate* out_sampleRate);

MPU6050_Status mpu6050_set_interrupt(mpu6050_t *hmpu, MPU6050_InterruptFlag int_flag);
MPU6050_Status MPU6050_get_interrupt(mpu6050_t *hmpu, MPU6050_InterruptFlag* out_status);


MPU6050_Status mpu6050_set_accelerometer(mpu6050_t *hmpu, MPU6050_Accelerometer accelerometer);
MPU6050_Status mpu6050_set_gyroscope(mpu6050_t *hmpu, MPU6050_Gyroscope gyroscope);
MPU6050_Status mpu6050_set_dlpf(mpu6050_t *hmpu, MPU6050_DLPF dlpf);

MPU6050_Status _mpu6050_read_accelerometer(mpu6050_t *hmpu, int16_t data_out[]);
MPU6050_Status _mpu6050_read_temperature(mpu6050_t *hmpu, int16_t data_out[]);
MPU6050_Status _mpu6050_read_gyroscope(mpu6050_t *hmpu, int16_t data_out[]);

MPU6050_Status _mpu6050_read_all_raw(mpu6050_t *hmpu, int16_t imu_data[]);
MPU6050_Status mpu6050_read_all(mpu6050_t *hmpu, mpu6050_outputs* imu_out);


// Helpers function

float mpu6050_calculate_temp(float imu_temp_raw);
float mpu6050_get_accel_sens(MPU6050_Accelerometer accel);
float mpu6050_get_gyro_sens(MPU6050_Gyroscope gyro);

#endif /* __MPU6050_H__ */
