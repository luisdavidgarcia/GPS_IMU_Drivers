#include "IMU/imu.hpp"

/**
 * @brief   Constructor for the IMU class.
 *
 * Takes the HAL I2C Handle
 */
IMU::IMU(I2C_HandleTypeDef *hi2c) :
		m_hi2c_(hi2c) {
	begin();
}

/**
 * @brief   Initialize the IMU sensor configuration.
 *
 * Sets up the IMU sensor by configuring registers and settings
 * for accelerometer, gyroscope, and magnetometer communication.
 */
void IMU::begin() const {
	// Reset the IMU
	uint8_t reset = 0x80;
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(m_hi2c_,
			IMU_I2C_ADDRESS << BYTE, PWR_MGMT_1, BYTE, &reset, BYTE,
			DEFAULT_TIMEOUT);
	HAL_Delay(100);
	if (status == HAL_ERROR) {
		return;
	}

	// Wake up the IMU
	uint8_t wakeup = 0;
	status = HAL_I2C_Mem_Write(m_hi2c_, IMU_I2C_ADDRESS << BYTE, PWR_MGMT_1,
			BYTE, &wakeup, BYTE, DEFAULT_TIMEOUT);
	HAL_Delay(100);
	if (status == HAL_ERROR) {
		return;
	}

	// Select the stable clock
	uint8_t stableClock = 0x01;
	status = HAL_I2C_Mem_Write(m_hi2c_, IMU_I2C_ADDRESS << BYTE, PWR_MGMT_1,
			BYTE, &stableClock, BYTE, DEFAULT_TIMEOUT);
	HAL_Delay(100);
	if (status == HAL_ERROR) {
		return;
	}

	// Configure the Gyroscope
	uint8_t set_500_dps = 0x08;
	status = HAL_I2C_Mem_Write(m_hi2c_, IMU_I2C_ADDRESS << BYTE, GYRO_CONFIG,
			BYTE, &set_500_dps, BYTE, DEFAULT_TIMEOUT);
	if (status == HAL_ERROR) {
		return;
	}

	// Configure the Accelerometer
	uint8_t set_4g = 0x08;
	status = HAL_I2C_Mem_Write(m_hi2c_, IMU_I2C_ADDRESS << BYTE, ACCEL_CONFIG,
			BYTE, &set_4g, BYTE, DEFAULT_TIMEOUT);
	if (status == HAL_ERROR) {
		return;
	}

	// Enable bypass
	uint8_t bypass = 0x02;
	status = HAL_I2C_Mem_Write(m_hi2c_, IMU_I2C_ADDRESS << BYTE, INT_PIN_CFG,
			BYTE, &bypass, BYTE, DEFAULT_TIMEOUT);
	HAL_Delay(100);
	if (status == HAL_ERROR) {
		return;
	}

	// Configure the Magnetmeteor (Turn if off)
	uint8_t turnOff = 0x00;
	status = HAL_I2C_Mem_Write(m_hi2c_, AK8963_ADDRESS << BYTE, AK8963_CNTL,
			BYTE, &turnOff, BYTE, DEFAULT_TIMEOUT);
	HAL_Delay(100);
	if (status == HAL_ERROR) {
		return;
	}

	// Turn on the Magnetmeteor (Continuous 16-Bit Mode)
	uint8_t turnOn = 0x16;
	status = HAL_I2C_Mem_Write(m_hi2c_, AK8963_ADDRESS << BYTE, AK8963_CNTL,
			BYTE, &turnOn, BYTE, DEFAULT_TIMEOUT);
	if (status == HAL_ERROR) {
		return;
	}

}

/**
 * @brief   Read sensor data from IMU
 *          accelerometer[1] = <read accelerometer Y value>;
 *          accelerometer[2] = <read accelerometer Z value>;
 *          gyroscope[0] = <read gyroscope X value>;
 *          gyroscope[1] = <read gyroscope Y value>;
 *          gyroscope[2] = <read gyroscope Z value>;
 *          magnetometer[0] = <read magnetometer X value>;
 *          magnetometer[1] = <read magnetometer Y value>;
 *          magnetometer[2] = <read magnetometer Z value>;
 */
void IMU::Read() {
	readAccelerometer();
	readGyroscope();
	readMagnetometer();
}

void IMU::readAccelerometer() {
	/* Read accelerometer data (Big Endian Order) */
	uint8_t raw_accel_data[6] = { 0, 0, 0, 0, 0, 0 };
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(m_hi2c_, IMU_I2C_ADDRESS << BYTE, ACCEL_XOUT_H,
			BYTE, raw_accel_data, 6, DEFAULT_TIMEOUT);
	if (status == HAL_ERROR) {
		return;
	}

	// Converting Raw Accel Data to Readable data
	accelerometer_.x = static_cast<int16_t>((raw_accel_data[0] << BITS_PER_BYTE)
			| (raw_accel_data[1] & BYTE_MASK));
	accelerometer_.y = static_cast<int16_t>((raw_accel_data[2] << BITS_PER_BYTE)
			| (raw_accel_data[3] & BYTE_MASK));
	accelerometer_.z = static_cast<int16_t>((raw_accel_data[4] << BITS_PER_BYTE)
			| (raw_accel_data[5] & BYTE_MASK));
}

void IMU::readGyroscope() {
	/* Read gyroscope data (Big Endian Order) */
	uint8_t raw_gyro_data[6] = { 0, 0, 0, 0, 0, 0 };
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(m_hi2c_, IMU_I2C_ADDRESS << BYTE, GYRO_XOUT_H,
			BYTE, raw_gyro_data, 6, DEFAULT_TIMEOUT);
	if (status == HAL_ERROR) {
		return;
	}

	// Converting Raw Gyro Data to Readable data
	gyroscope_.x = static_cast<int16_t>((raw_gyro_data[0] << BITS_PER_BYTE)
			| (raw_gyro_data[1] & BYTE_MASK));
	gyroscope_.y = static_cast<int16_t>((raw_gyro_data[2] << BITS_PER_BYTE)
			| (raw_gyro_data[3] & BYTE_MASK));
	gyroscope_.z = static_cast<int16_t>((raw_gyro_data[4] << BITS_PER_BYTE)
			| (raw_gyro_data[5] & BYTE_MASK));
}

void IMU::readMagnetometer() {
	/* Read magnetometer data one-burst (Little Endian Order) */
	uint8_t st1 = 0;
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(m_hi2c_, AK8963_ADDRESS << BYTE, AK8963_ST1, BYTE,
			&st1, BYTE, DEFAULT_TIMEOUT);
	if ((status == HAL_ERROR) || !(st1 & AK8963_DATA_READY)) {
		return;
	}

	uint8_t raw_mag_data[7] = { 0, 0, 0, 0, 0, 0, 0 };
	status = HAL_I2C_Mem_Read(m_hi2c_, AK8963_ADDRESS << BYTE, AK8963_XOUT_L,
			BYTE, raw_mag_data, 7, DEFAULT_TIMEOUT);

	uint8_t st2 = raw_mag_data[6];
	if (!(st2 & AK8963_MAGNETIC_OVERFLOW)) {
		// Converting Raw Mag Data to Readable data
		magnetometer_.x =
				static_cast<int16_t>((raw_mag_data[1] << BITS_PER_BYTE)
						| (raw_mag_data[0] & BYTE_MASK));
		magnetometer_.y =
				static_cast<int16_t>((raw_mag_data[3] << BITS_PER_BYTE)
						| (raw_mag_data[2] & BYTE_MASK));
		magnetometer_.z =
				static_cast<int16_t>((raw_mag_data[5] << BITS_PER_BYTE)
						| (raw_mag_data[4] & BYTE_MASK));
	}
}
