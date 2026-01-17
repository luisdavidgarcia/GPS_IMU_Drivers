/*
 * imu.h - Header file for IMU (Inertial Measurement Unit) sensor integration
 *
 * This header file provides an interface for interacting with an IMU sensor over
 * the I2C communication protocol. It includes functions and constants for retrieving
 * accelerometer, gyroscope, and magnetometer data from the IMU sensor. The code is
 * designed for a specific IMU sensor and may require configuration adjustments for
 * different hardware or use cases.
 *
 * Class:
 * - Imu: A class that encapsulates IMU functionality, including I2C communication,
 *   sensor data retrieval, and telemetry functions.
 *
 * Usage:
 * - Include this header file in your C++ project to interact with an IMU sensor.
 * - Instantiate the Imu class to communicate with the IMU sensor and retrieve data.
 *
 * Note: This code is designed for a specific IMU sensor and may require adaptation for
 *       other IMU sensors or hardware configurations. Refer to the provided credit and
 *       documentation for further details on usage and customization.
 */

#ifndef IMU_HPP
#define IMU_HPP

extern "C" {
#include "stm32l4xx_hal.h"
}

#include <mpu9250_imu/mpu9250_register_map.hpp>

#include <cstdint>
#include <ctime>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>

struct AccelMPS2 {
	int16_t x; // m/s^2
	int16_t y; // m/s^2
	int16_t z; // m/s^2
};

struct GyroRPS {
	int16_t x; // rad/s
	int16_t y; // rad/s
	int16_t z; // rad/s
};

struct MagUT {
	int16_t x; // µT
	int16_t y; // µT
	int16_t z; // µT
};

struct ScaledAccelMPS2 {
	float x; // m/s^2
	float y; // m/s^2
	float z; // m/s^2
};

struct ScaledGyroRPS {
	float x; // rad/s
	float y; // rad/s
	float z; // rad/s
};

struct ScaledMagUT {
	float x; // µT
	float y; // µT
	float z; // µT
};

class IMU {
public:
	explicit IMU(I2C_HandleTypeDef *t_hi2c);
	IMU(I2C_HandleTypeDef *t_hi2c, uint8_t t_mpu9250_address);

	void Read();

	const AccelMPS2& GetRawAcceleration() const {
		return accelerometer_;
	}
	const GyroRPS& GetRawAngularVelocity() const {
		return gyroscope_;
	}
	const MagUT& GetRawMagneticField() const {
		return magnetometer_;
	}

	inline ScaledAccelMPS2 GetScaledAcceleration() const {
		return {
			static_cast<float>(accelerometer_.x) * ACCEL_MG_LSB_4G - ACCEL_X_OFFSET,
			static_cast<float>(accelerometer_.y) * ACCEL_MG_LSB_4G - ACCEL_Y_OFFSET,
			static_cast<float>(accelerometer_.z) * ACCEL_MG_LSB_4G - ACCEL_Z_OFFSET,
		};
	}

	inline ScaledGyroRPS GetScaledAngularVelocity() const {
		return {
			static_cast<float>(gyroscope_.x) * GYRO_SENSITIVITY_500DPS * DEG_TO_RAD - GYRO_X_BIAS,
			static_cast<float>(gyroscope_.y) * GYRO_SENSITIVITY_500DPS * DEG_TO_RAD - GYRO_Y_BIAS,
			static_cast<float>(gyroscope_.z) * GYRO_SENSITIVITY_500DPS * DEG_TO_RAD - GYRO_Z_BIAS,
		};
	}

	inline ScaledMagUT GetScaledMagneticField() const {
		return {
			static_cast<float>(magnetometer_.x) * MAG_UT_LSB,
			static_cast<float>(magnetometer_.y) * MAG_UT_LSB,
			static_cast<float>(magnetometer_.z) * MAG_UT_LSB,
		};
	}

private:
	/** I2C Specifics */
	static constexpr uint16_t MPU9250_ADDRESS = 0x68;
	static constexpr uint16_t AK8963_ADDRESS = 0x0C;
	static constexpr uint16_t DEFAULT_TIMEOUT = 1000;

	/** Gyroscope sensitivity at 500dps */
	static constexpr float GYRO_SENSITIVITY_500DPS = 1.0F / 65.5F;
	static constexpr size_t GYRO_DATA_SIZE = 6;
	static constexpr uint8_t GYRO_CONFIG_VALUE = 0x11 << 1;

	/** Macro for mg per LSB at +/- 4g sensitivity (1 LSB = 0.000488mg) */
	static constexpr float ACCEL_MG_LSB_4G = 1.0F / 8192.0F;

	/** Macro for micro tesla (uT) per LSB (1 LSB = 0.1uT) */
	static constexpr float MAG_UT_LSB = 0.15F;
	static constexpr float MAG_MAX_THRESHOLD = 5000.0F; // µT, adjust as needed

	/** Filter constants */
	static constexpr float ACCEL_X_OFFSET = -0.0567F;
	static constexpr float ACCEL_Y_OFFSET = -0.0141F;
	static constexpr float ACCEL_Z_OFFSET = -0.7006F;
	static constexpr float GYRO_X_BIAS = -0.00845F;
	static constexpr float GYRO_Y_BIAS = 0.00647F;
	static constexpr float GYRO_Z_BIAS = -0.00955F;

	I2C_HandleTypeDef *m_hi2c_;
	uint8_t m_mpu9250_address_{MPU9250_ADDRESS};
	AccelMPS2 accelerometer_ { };
	GyroRPS gyroscope_ { };
	MagUT magnetometer_ { };
	void begin() const;
	void readAccelerometer();
	void readGyroscope();
	void readMagnetometer();
};

#endif // IMU_HPP
