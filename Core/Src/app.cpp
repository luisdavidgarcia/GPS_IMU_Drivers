/*
 * app.cpp
 *
 *      Author: Luis
 */

#include <mpu9250_imu/mpu9250_imu.hpp>
#include "app.hpp"

// All C++ Includes must go here for C not to freak out

// Main Cpp event loop to run application
void app_main_CPP(I2C_HandleTypeDef* hi2c1) {
	IMU imu{hi2c1};

	while (1)
	{
		imu.Read();
	}
}

// Define all C function calls from main.c below
extern "C"
{
	void app_main_C(I2C_HandleTypeDef* hi2c1) {
		app_main_CPP(hi2c1);
	}
}
