/*
 * app.cpp
 *
 *      Author: Luis
 */

#include <gtu7/gtu7.hpp>
#include <mpu9250/mpu9250.hpp>
#include "app.hpp"

#define UART_BUF_LEN 128
uint8_t uart_buf[UART_BUF_LEN];
volatile bool ubx_message_ready = false;

extern UART_HandleTypeDef huart4;

// All C++ Includes must go here for C not to freak out

// Main Cpp event loop to run application
void CPP_app_main(I2C_HandleTypeDef *hi2c1, UART_HandleTypeDef *huart4)
{
    MPU9250 imu { hi2c1 };
    GTU7 gps { huart4 };

    HAL_UART_Receive_DMA(huart4, uart_buf, UART_BUF_LEN);

    Polssh data{};
    while (1) {
//        imu.read();
        if (ubx_message_ready) {
            gps.get_posllh(data, uart_buf, UART_BUF_LEN);
            ubx_message_ready = false;
        }
    }
}

// Define all C function calls from main.c below
extern "C"
{
    void C_app_main(I2C_HandleTypeDef *hi2c1, UART_HandleTypeDef *huart4)
    {
        CPP_app_main(hi2c1, huart4);
    }

    void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
    {
        if (huart == &huart4) {
            ubx_message_ready = true;

            HAL_UART_Receive_DMA(huart, uart_buf, UART_BUF_LEN);
        }
    }
}

