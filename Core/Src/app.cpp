/*
 * app.cpp
 *
 *      Author: Luis
 */

#include <gtu7/gtu7.hpp>
#include <mpu9250/mpu9250.hpp>
#include "app.hpp"

// All C++ Includes must go here for C not to freak out

// Main Cpp event loop to run application
void CPP_app_main(I2C_HandleTypeDef *hi2c1, UART_HandleTypeDef *huart4)
{
    MPU9250 imu
        { hi2c1 };

    GTU7 gps
        { huart4 };

    while (1) {
//        imu.read();
        gps.get_pvt();
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
