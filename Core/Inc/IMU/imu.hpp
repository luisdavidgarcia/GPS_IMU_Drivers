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

#include "mpu9250_register_map.hpp"

#include <sys/cdefs.h>
#include <cstdint>
#include <ctime>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>

/** IMU Constants */
constexpr uint16_t TIME_DELAY_MS = 1000;
constexpr size_t ACCEL_MAG_DATA_SIZE = 12;
constexpr float MATH_PI = 3.1416;
constexpr float DEG_TO_RAD = MATH_PI / 180.0F;
constexpr float RAD_TO_DEG = 180.0F / MATH_PI;
constexpr float SENSORS_GRAVITY_STD = 9.807F;
constexpr float GYRO_MAX_THRESHOLD_RPS = 2200.0F;
constexpr float ACCEL_MAX_THRESHOLD_MPS2 = 17.6F;
constexpr int BYTE_SHIFT_AMOUNT = 8;
constexpr int X_AXIS = 0;
constexpr int Y_AXIS = 1;
constexpr int Z_AXIS = 2;
constexpr int BITS_PER_BYTE = 8;
constexpr int BYTE_MASK = 0xFF;
constexpr float ACCEL_SCALE = 0.3F;
constexpr uint8_t BYTE = 1;

/** I2C Specifics */
constexpr uint16_t IMU_I2C_ADDRESS = 0x68;
constexpr uint16_t AK8963_ADDRESS = 0x0C;
constexpr uint16_t DEFAULT_TIMEOUT = 1000;

/** Gyroscope sensitivity at 500dps */
constexpr float GYRO_SENSITIVITY_500DPS = 1.0F / 65.5F;
constexpr size_t GYRO_DATA_SIZE = 6;
constexpr uint8_t GYRO_CONFIG_VALUE = 0x11 << 1;

/** Macro for mg per LSB at +/- 4g sensitivity (1 LSB = 0.000488mg) */
constexpr float ACCEL_MG_LSB_4G = 1.0F / 8192.0F;

/** Macro for micro tesla (uT) per LSB (1 LSB = 0.1uT) */
constexpr float MAG_UT_LSB = 0.15F;
constexpr float MAG_MAX_THRESHOLD = 5000.0F; // µT, adjust as needed

/** Filter constants */
constexpr float ALPHA = 0.5F; // (range: 0-1)
constexpr float ACCEL_X_OFFSET = -0.0567F;
constexpr float ACCEL_Y_OFFSET = -0.0141F;
constexpr float ACCEL_Z_OFFSET = -0.7006F;
constexpr float GYRO_X_BIAS = -0.00845F;
constexpr float GYRO_Y_BIAS = 0.00647F;
constexpr float GYRO_Z_BIAS = -0.00955F;

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
	explicit IMU(I2C_HandleTypeDef *hi2c);
	~IMU() = default;

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
	I2C_HandleTypeDef *m_hi2c_;
	AccelMPS2 accelerometer_ { };
	GyroRPS gyroscope_ { };
	MagUT magnetometer_ { };
	void begin() const;
	void readAccelerometer();
	void readGyroscope();
	void readMagnetometer();
};

#endif // IMU_HPP
