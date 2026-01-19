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

extern "C"
{
#include "stm32l4xx_hal.h"
}

#include <mpu9250_imu/mpu9250_register_map.hpp>

#include <cstdint>
#include <ctime>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>

struct Accel_mps2
{
    int16_t x; // m/s^2
    int16_t y; // m/s^2
    int16_t z; // m/s^2
};

struct Gyro_rps
{
    int16_t x; // rad/s
    int16_t y; // rad/s
    int16_t z; // rad/s
};

struct Mag_ut
{
    int16_t x; // µT
    int16_t y; // µT
    int16_t z; // µT
};

struct Scaled_accel_mps2
{
    float x; // m/s^2
    float y; // m/s^2
    float z; // m/s^2
};

struct Scaled_gyro_rps
{
    float x; // rad/s
    float y; // rad/s
    float z; // rad/s
};

struct Scaled_mag_ut
{
    float x; // µT
    float y; // µT
    float z; // µT
};

class IMU
{
public:
    explicit IMU(I2C_HandleTypeDef *t_hi2c);
    IMU(I2C_HandleTypeDef *t_hi2c, uint8_t t_mpu9250_address);

    void Read();

    const Accel_mps2& get_raw_acceleration() const
    {
        return m_accelerometer_;
    }
    const Gyro_rps& get_raw_angular_velocity() const
    {
        return m_gyroscope_;
    }
    const Mag_ut& get_raw_magnetic_field() const
    {
        return m_magnetometer_;
    }

    inline Scaled_accel_mps2 get_scaled_acceleration() const
    {
        return {
            static_cast<float>(m_accelerometer_.x) * accel_mg_lsb_4g - accel_x_offset,
            static_cast<float>(m_accelerometer_.y) * accel_mg_lsb_4g - accel_y_offset,
            static_cast<float>(m_accelerometer_.z) * accel_mg_lsb_4g - accel_z_offset,
        };
    }

    inline Scaled_gyro_rps get_scaled_angular_velocity() const
    {
        return {
            static_cast<float>(m_gyroscope_.x) * gyro_sensitivity_500dps * deg_to_rad - gyro_x_bias,
            static_cast<float>(m_gyroscope_.y) * gyro_sensitivity_500dps * deg_to_rad - gyro_y_bias,
            static_cast<float>(m_gyroscope_.z) * gyro_sensitivity_500dps * deg_to_rad - gyro_z_bias,
        };
    }

    inline Scaled_mag_ut get_scaled_magnetic_field() const
    {
        return {
            static_cast<float>(m_magnetometer_.x) * mag_ut_lsb,
            static_cast<float>(m_magnetometer_.y) * mag_ut_lsb,
            static_cast<float>(m_magnetometer_.z) * mag_ut_lsb,
        };
    }

private:
    /** I2C specifics */
    static constexpr uint16_t mpu9250_address = 0x68;
    static constexpr uint16_t ak8963_address = 0x0c;
    static constexpr uint16_t default_timeout = 1000;

    /** Gyroscope sensitivity at 500dps */
    static constexpr float gyro_sensitivity_500dps = 1.0f / 65.5f;
    static constexpr size_t gyro_data_size = 6;
    static constexpr uint8_t gyro_config_value = 0x11 << 1;

    /** Macro for mg per lsb at +/- 4g sensitivity (1 lsb = 0.000488mg) */
    static constexpr float accel_mg_lsb_4g = 1.0f / 8192.0f;

    /** Macro for micro tesla (ut) per lsb (1 lsb = 0.1ut) */
    static constexpr float mag_ut_lsb = 0.15f;
    static constexpr float mag_max_threshold = 5000.0f; // µt, adjust as needed

    /** Filter constants */
    static constexpr float accel_x_offset = -0.0567f;
    static constexpr float accel_y_offset = -0.0141f;
    static constexpr float accel_z_offset = -0.7006f;
    static constexpr float gyro_x_bias = -0.00845f;
    static constexpr float gyro_y_bias = 0.00647f;
    static constexpr float gyro_z_bias = -0.00955f;


    I2C_HandleTypeDef *m_hi2c_;
    uint8_t m_mpu9250_address_ { mpu9250_address };
    Accel_mps2 m_accelerometer_ {};
    Gyro_rps m_gyroscope_ {};
    Mag_ut m_magnetometer_ {};
    void begin() const;
    void read_accelerometer();
    void read_gyroscope();
    void read_magnetometer();
};

#endif // IMU_HPP
