/*
 * app.hpp
 *
 *      Author: Luis
 */

#ifndef APP_HPP
#define APP_HPP

#include "stm32l4xx_hal.h"

// Cpp function to call into main event loop
void app_main_CPP(I2C_HandleTypeDef* hi2c1);

#ifdef __cplusplus
extern "C"
{
#endif
	// C function to call into Cpp event loop from main
	void app_main_C(I2C_HandleTypeDef* hi2c1);
#ifdef __cplusplus
}
#endif


#endif /* APP_HPP */
