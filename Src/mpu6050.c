#include "mpu6050.h"
#include "mpu6050_registers.h"
#include <assert.h>

#define IS_MPU6050_ADDR(ADDR) 	(((ADDR) == MPU6050_Device_0) || \
								((ADDR) == MPU6050_Device_1))


MPU6050_Status _MPU6050_I2C_WaitForFlag(
		uint32_t (*get_flag_status)(I2C_TypeDef*), I2C_TypeDef *i2c,
		uint32_t timeout) {
	while (!get_flag_status(i2c)) {
		if (LL_SYSTICK_IsActiveCounterFlag() && !(timeout-- > 0))
			return MPU6050_Status_Timeout;
	}

	return MPU6050_Status_Ok;
}
MPU6050_Status _MPU6050_I2C_WaitForFlagClear(
		uint32_t (*get_flag_status)(I2C_TypeDef*), I2C_TypeDef *i2c,
		uint32_t timeout) {
	while (get_flag_status(i2c)) {
		if (LL_SYSTICK_IsActiveCounterFlag() && !(timeout-- > 0))
			return MPU6050_Status_Timeout;
	}

	return MPU6050_Status_Ok;
}
MPU6050_Status _MPU6050_I2C_WaitForFlag2(
		uint32_t (*get_flag_status)(I2C_TypeDef*),
		uint32_t (*get_flag_status2)(I2C_TypeDef*), I2C_TypeDef *i2c,
		uint32_t timeout) {
	while (!get_flag_status(i2c) && !get_flag_status2(i2c)) {
		if (LL_SYSTICK_IsActiveCounterFlag() && !(timeout-- > 0))
			return MPU6050_Status_Timeout;
	}

	return MPU6050_Status_Ok;
}


MPU6050_Status _MPU6050_Write(mpu6050_t *hmpu, uint8_t tx_data[],
		uint32_t tx_data_length, uint32_t timeout) {
	if (tx_data_length < 1)
		return MPU6050_Status_Error;

	MPU6050_Status status = MPU6050_Status_Ok;
	I2C_TypeDef *i2c = hmpu->i2c_handler;

	// todo: dunno,


	LL_I2C_Enable(i2c);

  	LL_I2C_DisableBitPOS(i2c);

	LL_I2C_AcknowledgeNextData(i2c, LL_I2C_ACK);
	LL_I2C_GenerateStartCondition(i2c);

	if ((status = _MPU6050_I2C_WaitForFlag(LL_I2C_IsActiveFlag_SB, i2c, timeout))
			!= MPU6050_Status_Ok)
		return status;

	LL_I2C_TransmitData8(i2c, hmpu->dev_address);
	if ((status = _MPU6050_I2C_WaitForFlag(LL_I2C_IsActiveFlag_ADDR, i2c,
			timeout)) != MPU6050_Status_Ok)
		return status;

	LL_I2C_ClearFlag_ADDR(i2c);

	while (tx_data_length--) {
		if ((status = _MPU6050_I2C_WaitForFlag(LL_I2C_IsActiveFlag_TXE, i2c,
				timeout)) != MPU6050_Status_Ok)
			return status;

		LL_I2C_TransmitData8(i2c, *tx_data++);

		if (LL_I2C_IsActiveFlag_BTF(i2c) && tx_data_length) {
			LL_I2C_TransmitData8(i2c, *tx_data++);
			tx_data_length--;
		}

		if ((status = _MPU6050_I2C_WaitForFlag(LL_I2C_IsActiveFlag_BTF, i2c,
				timeout)) != MPU6050_Status_Ok)
			return status;
	}


	LL_I2C_GenerateStopCondition(i2c);

	return MPU6050_Status_Ok;
}
MPU6050_Status _MPU6050_Read(mpu6050_t *hmpu, uint8_t *rx_data,
		uint32_t rx_data_length, uint32_t timeout) {
	if (rx_data_length < 1)
		return MPU6050_Status_Error;

	MPU6050_Status status = MPU6050_Status_Ok;
	I2C_TypeDef *i2c = hmpu->i2c_handler;

	LL_I2C_Enable(i2c);
	LL_I2C_DisableBitPOS(i2c);
	LL_I2C_AcknowledgeNextData(i2c, LL_I2C_ACK);
	LL_I2C_GenerateStartCondition(i2c);

	if ((status = _MPU6050_I2C_WaitForFlag(LL_I2C_IsActiveFlag_SB, i2c, timeout))
			!= MPU6050_Status_Ok)
		return status;

	LL_I2C_TransmitData8(i2c, hmpu->dev_address | 0x01);
	if ((status = _MPU6050_I2C_WaitForFlag(LL_I2C_IsActiveFlag_ADDR, i2c,
			timeout)) != MPU6050_Status_Ok)
		return status;

	// adaptacja hala
	if (rx_data_length == 0U) {
		/* Clear ADDR flag */
		LL_I2C_ClearFlag_ADDR(i2c);

		/* Generate Stop */
		LL_I2C_GenerateStopCondition(i2c);
	} else if (rx_data_length == 1U) {
		/* Disable Acknowledge */
		LL_I2C_AcknowledgeNextData(i2c, LL_I2C_NACK);

		/* Disable all active IRQs around ADDR clearing and STOP programming because the EV6_3
		 software sequence must complete before the current byte end of transfer */
		__disable_irq();

		/* Clear ADDR flag */
		LL_I2C_ClearFlag_ADDR(i2c);

		/* Generate Stop */
		LL_I2C_GenerateStopCondition(i2c);

		/* Re-enable IRQs */
		__enable_irq();
	} else if (rx_data_length == 2U) {
		/* Enable Pos */
		LL_I2C_EnableBitPOS(i2c);

		/* Disable all active IRQs around ADDR clearing and STOP programming because the EV6_3
		 software sequence must complete before the current byte end of transfer */
		__disable_irq();

		/* Clear ADDR flag */
		LL_I2C_ClearFlag_ADDR(i2c);

		/* Disable Acknowledge */
		LL_I2C_AcknowledgeNextData(i2c, LL_I2C_NACK);

		/* Re-enable IRQs */
		__enable_irq();
	} else {
		/* Enable Acknowledge */
		LL_I2C_AcknowledgeNextData(i2c, LL_I2C_ACK);

		/* Clear ADDR flag */
		LL_I2C_ClearFlag_ADDR(i2c);
	}

	while (rx_data_length > 0U)
	{
		if (rx_data_length <= 3U) {
			/* One byte */
			if (rx_data_length == 1U) {
				/* Wait until RXNE flag is set */
				if ((status = _MPU6050_I2C_WaitForFlag(LL_I2C_IsActiveFlag_RXNE, i2c, timeout))
						!= MPU6050_Status_Ok)
					return status;

				/* Read data from DR */
				(*rx_data++) = LL_I2C_ReceiveData8(i2c);
				rx_data_length--;
			}
			/* Two bytes */
			else if (rx_data_length == 2U) {
				/* Wait until BTF flag is set */
				if ((status = _MPU6050_I2C_WaitForFlag(LL_I2C_IsActiveFlag_BTF, i2c, timeout))
						!= MPU6050_Status_Ok)
					return status;


				/* Disable all active IRQs around ADDR clearing and STOP programming because the EV6_3
				 software sequence must complete before the current byte end of transfer */
				__disable_irq();

				/* Generate Stop */
				LL_I2C_GenerateStopCondition(i2c);

				/* Read data from DR */
				(*rx_data++) = LL_I2C_ReceiveData8(i2c);
				rx_data_length--;

				/* Re-enable IRQs */
				__enable_irq();

				/* Read data from DR */
				(*rx_data++) = LL_I2C_ReceiveData8(i2c);
				rx_data_length--;
			}
			/* 3 Last bytes */
			else {
				/* Wait until BTF flag is set */
				if ((status = _MPU6050_I2C_WaitForFlag(LL_I2C_IsActiveFlag_BTF, i2c, timeout))
						!= MPU6050_Status_Ok)
					return status;

				/* Disable Acknowledge */
				LL_I2C_AcknowledgeNextData(i2c, LL_I2C_NACK);

				/* Disable all active IRQs around ADDR clearing and STOP programming because the EV6_3
				 software sequence must complete before the current byte end of transfer */
				__disable_irq();

				/* Read data from DR */
				(*rx_data++) = LL_I2C_ReceiveData8(i2c);
				rx_data_length--;

				/* Wait until BTF flag is set */
				if ((status = _MPU6050_I2C_WaitForFlag(LL_I2C_IsActiveFlag_BTF, i2c, timeout))
						!= MPU6050_Status_Ok)
					return status;

				/* Generate Stop */
				LL_I2C_GenerateStopCondition(i2c);

				/* Read data from DR */
				(*rx_data++) = LL_I2C_ReceiveData8(i2c);
				rx_data_length--;

				/* Re-enable IRQs */
				__enable_irq();

				/* Read data from DR */
				(*rx_data++) = LL_I2C_ReceiveData8(i2c);
				rx_data_length--;
			}
		} else {
			/* Wait until RXNE flag is set */
			if ((status = _MPU6050_I2C_WaitForFlag(LL_I2C_IsActiveFlag_RXNE, i2c, timeout))
					!= MPU6050_Status_Ok)
				return status;


			/* Read data from DR */
			(*rx_data++) = LL_I2C_ReceiveData8(i2c);
			rx_data_length--;

			if (LL_I2C_IsActiveFlag_BTF(i2c)) {
				/* Read data from DR */
				(*rx_data++) = LL_I2C_ReceiveData8(i2c);
				rx_data_length--;
			}
		}
	}

	return MPU6050_Status_Ok;
}


MPU6050_Status MPU6050_Write(mpu6050_t *hmpu, uint8_t tx_data[],
		uint32_t tx_data_length, uint32_t timeout) {

	if(hmpu->status != MPU6050_Status_Ok)
		return hmpu->status;

	MPU6050_Status status;

	if ((status = _MPU6050_Write(hmpu, tx_data, tx_data_length, timeout))
			!= MPU6050_Status_Ok) {
		LL_I2C_GenerateStopCondition(hmpu->i2c_handler);
	}

	hmpu->status = status;
	return status;
}
MPU6050_Status MPU6050_Read(mpu6050_t *hmpu, uint8_t *rx_data,
		uint32_t rx_data_length, uint32_t timeout) {

	if(hmpu->status != MPU6050_Status_Ok)
		return hmpu->status;

	MPU6050_Status status;

	if((status = _MPU6050_Read(hmpu,rx_data, rx_data_length, timeout))
			!= MPU6050_Status_Ok) {
		LL_I2C_GenerateStopCondition(hmpu->i2c_handler);
	}

	hmpu->status = status;
	return status;

}



MPU6050_Status mpu6050_get_status(mpu6050_t *hmpu)
{
	return hmpu->status;
}
void mpu6050_reset_state(mpu6050_t *hmpu)
{
	hmpu->status = MPU6050_Status_Ok;
}

MPU6050_Status mpu6050_init(mpu6050_t *hmpu, I2C_TypeDef *hi2c,
		MPU6050_Configuraton configuration) {
	if (hmpu == NULL || hi2c == NULL) {
		return MPU6050_Status_Error;
	}

	hmpu->i2c_handler = hi2c;
	hmpu->dev_address = MPU6050_I2C_ADDR | configuration.device;
	hmpu->status = MPU6050_Status_Ok;
	//assert_param(IS_MPU6050_ADDR(hmpu->dev_address));

	// Initialize MPU6050 device

	MPU6050_Status status;
	uint8_t rx_tmp;
	uint8_t who_am_i = MPU6050_WHO_AM_I;

	if ((status = MPU6050_Write(hmpu, &who_am_i, 1, 10000))
			!= MPU6050_Status_Ok) {
		return status;
	}

	if ((status = MPU6050_Read(hmpu, &rx_tmp, 1, 10000)) != MPU6050_Status_Ok) {
		return status;
	}
	if (rx_tmp != MPU6050_I_AM) {
		hmpu->status = MPU6050_Status_DeviceInvalid;
		return MPU6050_Status_DeviceInvalid;
	}


	// configure the device
	// todo: error check
	//LL_mDelay(1);
	if ((status =mpu6050_set_sample_rate(hmpu, configuration.sample_rate))
			!= MPU6050_Status_Ok) {
		return status;
	}
	//LL_mDelay(1);
	if ((status =mpu6050_set_gyroscope(hmpu, configuration.gyroscope))
			!= MPU6050_Status_Ok) {
		return status;
	}
	//LL_mDelay(1);
	if ((status =mpu6050_set_accelerometer(hmpu, configuration.accelerometer))
			!= MPU6050_Status_Ok) {
		return status;
	}
	//LL_mDelay(1);
	if ((status =mpu6050_set_dlpf(hmpu, configuration.dlpf))
			!= MPU6050_Status_Ok) {
		return status;
	}

	hmpu->config = configuration;
	return MPU6050_Status_Ok;
}
MPU6050_Status mpu6050_start(mpu6050_t *hmpu) {
	// Wakeup MPU6050
	MPU6050_Status status;
	uint8_t tx_data[2];

	tx_data[0] = MPU6050_PWR_MGMT_1;
	tx_data[1] = 0x00;

	status = MPU6050_Write(hmpu, tx_data, 2, 1000);
	return status;
}

MPU6050_Status mpu6050_set_sample_rate(mpu6050_t *hmpu,
		MPU6050_SampleRate sampleRate) {
	MPU6050_Status status;
	uint8_t tx_data[2];

	tx_data[0] = MPU6050_SMPLRT_DIV;
	tx_data[1] = sampleRate;

	if ((status = MPU6050_Write(hmpu, tx_data, 2, 1000)) != MPU6050_Status_Ok) {
		return status;
	}
	hmpu->config.sample_rate = sampleRate;
	return status;
}
MPU6050_Status mpu6050_get_sample_rate(mpu6050_t *hmpu, MPU6050_SampleRate* out_sampleRate) {
	MPU6050_Status status;
	uint8_t int_register = MPU6050_SMPLRT_DIV;
	uint8_t rx_data;

	if ((status = MPU6050_Write(hmpu, &int_register, 1, 1000))
			!= MPU6050_Status_Ok) {
		return status;
	}

	if ((status = MPU6050_Read(hmpu, &rx_data, 1, 1000)) != MPU6050_Status_Ok) {
		return status;
	}

	out_sampleRate[0] = rx_data;
	return MPU6050_Status_Ok;
}

MPU6050_Status mpu6050_set_interrupt(mpu6050_t *hmpu, MPU6050_InterruptFlag int_flag) {
	MPU6050_Status status;
	uint8_t tx_data[2];

	tx_data[0] = MPU6050_INT_ENABLE;
	tx_data[1] = (uint8_t) int_flag;

	status = MPU6050_Write(hmpu, tx_data, 2, 1000);
	return status;
}
MPU6050_Status MPU6050_get_interrupt(mpu6050_t *hmpu, MPU6050_InterruptFlag* out_status) {
	MPU6050_Status status;
	uint8_t int_status = MPU6050_INT_STATUS;
	uint8_t rx_status = 0;

	if ((status = MPU6050_Write(hmpu, &int_status, 1, 1000))
			!= MPU6050_Status_Ok) {
		return status;
	}

	if ((status = MPU6050_Read(hmpu, &rx_status, 1, 1000))
			!= MPU6050_Status_Ok) {
		return status;
	}

	*out_status = rx_status;

	return status;
}


MPU6050_Status mpu6050_set_accelerometer(mpu6050_t *hmpu,
		MPU6050_Accelerometer accelerometer) {
	MPU6050_Status status;
	uint8_t accel_config = MPU6050_ACCEL_CONFIG;
	uint8_t accel_config_value;

	if ((status = MPU6050_Write(hmpu, &accel_config, 1, 1000))
			!= MPU6050_Status_Ok) {
		return status;
	}

	if ((status = MPU6050_Read(hmpu, &accel_config_value, 1, 1000))
			!= MPU6050_Status_Ok) {
		return status;
	}

	accel_config_value = (accel_config_value & 0b11100111)
			| (accelerometer << 3);

	uint8_t tx_data[2];
	tx_data[0] = MPU6050_ACCEL_CONFIG;
	tx_data[1] = accel_config_value;

	if ((status = MPU6050_Write(hmpu, tx_data, 2, 1000)) != MPU6050_Status_Ok) {
		return status;
	}

	hmpu->config.accelerometer = accelerometer;
	return MPU6050_Status_Ok;
}
MPU6050_Status mpu6050_set_gyroscope(mpu6050_t *hmpu,
		MPU6050_Gyroscope gyroscope) {
	MPU6050_Status status;
	uint8_t gyro_config = MPU6050_GYRO_CONFIG;
	uint8_t gyro_config_value;

	if ((status = MPU6050_Write(hmpu, &gyro_config, 1, 1000))
			!= MPU6050_Status_Ok) {
		return status;
	}

	if ((status = MPU6050_Read(hmpu, &gyro_config_value, 1, 1000))
			!= MPU6050_Status_Ok) {
		return status;
	}

	gyro_config_value = (gyro_config_value & 0b11100111) | (gyroscope << 3);

	uint8_t tx_data[2];
	tx_data[0] = MPU6050_GYRO_CONFIG;
	tx_data[1] = gyro_config_value;
	if ((status = MPU6050_Write(hmpu, tx_data, 2, 1000)) != MPU6050_Status_Ok) {
		return status;
	}

	hmpu->config.gyroscope = gyroscope;
	return MPU6050_Status_Ok;
}
MPU6050_Status mpu6050_set_dlpf(mpu6050_t *hmpu, MPU6050_DLPF dlpf)
{
	MPU6050_Status status;
	uint8_t config = MPU6050_CONFIG;
	uint8_t config_value;

	if ((status = MPU6050_Write(hmpu, &config, 1, 1000))
			!= MPU6050_Status_Ok) {
		return status;
	}

	if ((status = MPU6050_Read(hmpu, &config_value, 1, 1000))
			!= MPU6050_Status_Ok) {
		return status;
	}

	config_value = (config_value & 0xF8) | (dlpf & 0x07);
	uint8_t tx_data[2];
	tx_data[0] = MPU6050_CONFIG;
	tx_data[1] = config_value;
	if ((status = MPU6050_Write(hmpu, tx_data, 2, 1000)) != MPU6050_Status_Ok) {
		return status;
	}

	hmpu->config.dlpf = dlpf;
	return MPU6050_Status_Ok;
}

MPU6050_Status _mpu6050_read_accelerometer(mpu6050_t *hmpu, int16_t data_out[]) {
	MPU6050_Status status;
	uint8_t accel_out_register = MPU6050_ACCEL_XOUT_H;
	uint8_t rx_data[6];

	if ((status = MPU6050_Write(hmpu, &accel_out_register, 1, 1000))
			!= MPU6050_Status_Ok) {
		return status;
	}

	if ((status = MPU6050_Read(hmpu, rx_data, 6, 1000)) != MPU6050_Status_Ok) {
		return status;
	}

	data_out[0] = ((uint16_t) rx_data[0] << 8) | rx_data[1];
	data_out[1] = ((uint16_t) rx_data[2] << 8) | rx_data[3];
	data_out[2] = ((uint16_t) rx_data[4] << 8) | rx_data[5];
	return MPU6050_Status_Ok;
}
MPU6050_Status _mpu6050_read_temperature(mpu6050_t *hmpu, int16_t data_out[])
{
	MPU6050_Status status;
	uint8_t temp_out_register = MPU6050_TEMP_OUT_H;
	uint8_t rx_data[2];

	if ((status = MPU6050_Write(hmpu, &temp_out_register, 1, 1000))
			!= MPU6050_Status_Ok) {
		return status;
	}

	if ((status = MPU6050_Read(hmpu, rx_data, 2, 1000)) != MPU6050_Status_Ok) {
		return status;
	}

	data_out[0] = ((uint16_t) rx_data[0] << 8) | rx_data[1];
	return MPU6050_Status_Ok;
}
MPU6050_Status _mpu6050_read_gyroscope(mpu6050_t *hmpu, int16_t data_out[])
{
	MPU6050_Status status;
	uint8_t gyro_out_register = MPU6050_GYRO_XOUT_H;
	uint8_t rx_data[6];

	if ((status = MPU6050_Write(hmpu, &gyro_out_register, 1, 1000))
			!= MPU6050_Status_Ok) {
		return status;
	}

	if ((status = MPU6050_Read(hmpu, rx_data, 6, 1000)) != MPU6050_Status_Ok) {
		return status;
	}

	data_out[0] = ((uint16_t) rx_data[0] << 8) | rx_data[1];
	data_out[1] = ((uint16_t) rx_data[2] << 8) | rx_data[3];
	data_out[2] = ((uint16_t) rx_data[4] << 8) | rx_data[5];
	return MPU6050_Status_Ok;
}

MPU6050_Status _mpu6050_read_all_raw(mpu6050_t *hmpu, int16_t imu_data[]) {
	MPU6050_Status status;
	uint8_t accel_out_register = MPU6050_ACCEL_XOUT_H;
	uint8_t rx_data[14]; // ACCL - TMP - GYRO

	if ((status = MPU6050_Write(hmpu, &accel_out_register, 1, 1000))
			!= MPU6050_Status_Ok) {
		return status;
	}

	if ((status = MPU6050_Read(hmpu, rx_data, 14, 1000)) != MPU6050_Status_Ok) {
		return status;
	}

	imu_data[0] = ((uint16_t) rx_data[0] << 8) | rx_data[1];
	imu_data[1] = ((uint16_t) rx_data[2] << 8) | rx_data[3];
	imu_data[2] = ((uint16_t) rx_data[4] << 8) | rx_data[5];

	imu_data[3] = ((uint16_t) rx_data[6] << 8) | rx_data[7];

	imu_data[4] = ((uint16_t) rx_data[8] << 8) | rx_data[9];
	imu_data[5] = ((uint16_t) rx_data[10] << 8) | rx_data[11];
	imu_data[6] = ((uint16_t) rx_data[12] << 8) | rx_data[13];

	/*imu_data[0] = ((uint16_t)rx_data[0] << 8) | rx_data[1];
	 imu_data[1] = ((uint16_t)rx_data[2] << 8) | rx_data[3];
	 imu_data[2] = ((uint16_t)rx_data[4] << 8) | rx_data[5];

	 imu_data[3] = ((uint16_t)rx_data[6] << 8) | rx_data[7];

	 imu_data[4] = ((uint16_t)rx_data[8] << 8) | rx_data[9];
	 imu_data[5] = ((uint16_t)rx_data[10] << 8) | rx_data[11];
	 imu_data[6] = ((uint16_t)rx_data[12] << 8) | rx_data[13];*/

	return MPU6050_Status_Ok;
}
MPU6050_Status mpu6050_read_all(mpu6050_t *hmpu, mpu6050_outputs* imu_out)
{
	int16_t data[7]; // ACCL - TMP - GYRO

	MPU6050_Status status;

	if((status = _mpu6050_read_all_raw(hmpu, data)) != MPU6050_Status_Ok)
		return status;

	float accel_sens = mpu6050_get_accel_sens(hmpu->config.accelerometer);
	float gyro_sens = mpu6050_get_gyro_sens(hmpu->config.gyroscope);

	imu_out->accel.x = (float)data[0]/accel_sens;
	imu_out->accel.y = (float)data[1]/accel_sens;
	imu_out->accel.z = (float)data[2]/accel_sens;

	imu_out->temp = mpu6050_calculate_temp((float)data[3]);

	imu_out->gyro.x = (float)data[4]/gyro_sens;
	imu_out->gyro.y = (float)data[5]/gyro_sens;
	imu_out->gyro.z = (float)data[6]/gyro_sens;

	return MPU6050_Status_Ok;
}



// Helpers functions

const float _mpu6050_accel_sens_tab[]  	= { 16384.0f, 8192.0f, 4096.0f, 2048.0f };
const float _mpu6050_gyro_sens_tab[]	= { 131.0f, 65.5f, 32.75f, 16.375f };

float mpu6050_calculate_temp(float imu_temp_raw)
{
	return imu_temp_raw/340.0f + 36.53;
}
float mpu6050_get_accel_sens(MPU6050_Accelerometer accel)
{
	if(accel >=0 && accel < 4)
		return _mpu6050_accel_sens_tab[accel];
	else
		return 0.0f;
}
float mpu6050_get_gyro_sens(MPU6050_Gyroscope gyro)
{
	if( gyro >= 0 && gyro < 4)
		return _mpu6050_gyro_sens_tab[gyro];
	else
		return 0.0f;
}
