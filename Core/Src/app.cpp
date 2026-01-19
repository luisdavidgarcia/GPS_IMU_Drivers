/*
 * app.cpp
 *
 *      Author: Luis
 */

#include <mpu9250_imu/mpu9250_imu.hpp>
#include <gps/gps.hpp>
#include "app.hpp"

// All C++ Includes must go here for C not to freak out

// Main Cpp event loop to run application
void CPP_app_main(I2C_HandleTypeDef *hi2c1, UART_HandleTypeDef *huart4)
{
    IMU imu
        { hi2c1 };

    GPS gps
        { 2026, huart4 };

    while (1) {
        imu.Read();
    }
}

// Define all C function calls from main.c below
extern "C"
{
    void C_app_main(I2C_HandleTypeDef *hi2c1, UART_HandleTypeDef *huart4)
    {
        CPP_app_main(hi2c1, huart4);
    }
}
