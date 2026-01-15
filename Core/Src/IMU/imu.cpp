#include "IMU/imu.hpp"

/**
 * @brief   Constructor for the IMU class.
 *
 * Takes the HAL I2C Handle
 */
IMU::IMU(I2C_HandleTypeDef* hi2c) : m_hi2c_(hi2c)
{
	begin();
}

/**
 * @brief   Initialize the IMU sensor configuration.
 *
 * Sets up the IMU sensor by configuring registers and settings
 * for accelerometer, gyroscope, and magnetometer communication.
 */
void IMU::begin() const {
	HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(m_hi2c_, IMU_I2C_ADDRESS << 1, 3, 100);

	if (status == HAL_ERROR) {
		HAL_Delay(100);
		return;
	}

	uint8_t who_am_i = 0;
	status = HAL_I2C_Mem_Read(
			m_hi2c_,
			IMU_I2C_ADDRESS << 1,
			0x75,
			1,
			&who_am_i,
			1,
			1000);
	if (status == HAL_OK) {
		HAL_Delay(100);
	}
}

/**
 * @brief   Read sensor data from IMU over I2C.
 *          Replace this section with your actual I2C communication to read
 * 					sensor data. For example: accelerometer[0] = <read accelerometer X value>;
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
//	/* Read accelerometer data */
//	uint8_t accel_x_h = HAL_I2C_Mem_Read(i2c_fd, ACCEL_XOUT_H);
//	uint8_t accel_x_l = HAL_I2C_Mem_Read(i2c_fd, ACCEL_XOUT_L);
//	uint8_t accel_y_h = HAL_I2C_Mem_Read(i2c_fd, ACCEL_YOUT_H);
//	uint8_t accel_y_l = HAL_I2C_Mem_Read(i2c_fd, ACCEL_YOUT_L);
//	uint8_t accel_z_h = HAL_I2C_Mem_Read(i2c_fd, ACCEL_ZOUT_H);
//	uint8_t accel_z_l = HAL_I2C_Mem_Read(i2c_fd, ACCEL_ZOUT_L);
//
//	// Converting Raw Accel Data to Readable data
//	accelerometer_.x = static_cast<int16_t>(
//		(accel_x_h << BITS_PER_BYTE) | (accel_x_l & BYTE_MASK)
//	);
//	accelerometer_.y = static_cast<int16_t>(
//		(accel_y_h << BITS_PER_BYTE) | (accel_y_l & BYTE_MASK)
//	);
//	accelerometer_.z = static_cast<int16_t>(
//		(accel_z_h << BITS_PER_BYTE) | (accel_z_l & BYTE_MASK)
//	);
//
//	/* Read gyroscope data */
//	uint8_t gyro_x_h = HAL_I2C_Mem_Read(i2c_fd, GYRO_XOUT_H);
//	uint8_t gyro_x_l = HAL_I2C_Mem_Read(i2c_fd, GYRO_XOUT_L);
//	uint8_t gyro_y_h = HAL_I2C_Mem_Read(i2c_fd, GYRO_YOUT_H);
//	uint8_t gyro_y_l = HAL_I2C_Mem_Read(i2c_fd, GYRO_YOUT_L);
//	uint8_t gyro_z_h = HAL_I2C_Mem_Read(i2c_fd, GYRO_ZOUT_H);
//	uint8_t gyro_z_l = HAL_I2C_Mem_Read(i2c_fd, GYRO_ZOUT_L);
//
//	// Converting Raw Gyro Data to Readable data
//	gyroscope_.x = static_cast<int16_t>(
//		(gyro_x_h << BITS_PER_BYTE) | (gyro_x_l & BYTE_MASK)
//	);
//	gyroscope_.y = static_cast<int16_t>(
//		(gyro_y_h << BITS_PER_BYTE) | (gyro_y_l & BYTE_MASK)
//	);
//	gyroscope_.z = static_cast<int16_t>(
//		(gyro_z_h << BITS_PER_BYTE) | (gyro_z_l & BYTE_MASK)
//	);
//
//	/* Read magentometer data */
//	uint8_t mag_x_h = HAL_I2C_Mem_Read(i2c_fd, MAGNETO_XOUT_H);
//	uint8_t mag_x_l = HAL_I2C_Mem_Read(i2c_fd, MAGNETO_XOUT_L);
//	uint8_t mag_y_h = HAL_I2C_Mem_Read(i2c_fd, MAGNETO_YOUT_H);
//	uint8_t mag_y_l = HAL_I2C_Mem_Read(i2c_fd, MAGNETO_YOUT_L);
//	uint8_t mag_z_h = HAL_I2C_Mem_Read(i2c_fd, MAGNETO_ZOUT_H);
//	uint8_t mag_z_l = HAL_I2C_Mem_Read(i2c_fd, MAGNETO_ZOUT_L);
//
//	// Converting Raw Mag Data to Readable data
//	magnetometer_.x = static_cast<int16_t>(
//		(mag_x_l << BITS_PER_BYTE) | (mag_x_h & BYTE_MASK)
//	);
//	magnetometer_.y = static_cast<int16_t>(
//		(mag_y_l << BITS_PER_BYTE) | (mag_y_h & BYTE_MASK)
//	);
//	magnetometer_.z = static_cast<int16_t>(
//		(mag_z_l << BITS_PER_BYTE) | (mag_z_h & BYTE_MASK)
//	);
}
