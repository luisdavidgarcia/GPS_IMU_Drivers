/*
 * app.cpp
 *
 *      Author: Luis
 */

#include <gtu7/gtu7.hpp>
#include <mpu9250/mpu9250.hpp>
#include "app.hpp"

#define UART_BUF_LEN 256
uint8_t uart_buf[UART_BUF_LEN];

// All C++ Includes must go here for C not to freak out

// Main Cpp event loop to run application
void CPP_app_main(I2C_HandleTypeDef *hi2c1, UART_HandleTypeDef *huart4,
        DMA_HandleTypeDef *hdma_uart4_rx)
{
    MPU9250 imu { hi2c1 };
    GTU7 gps { huart4 };

    HAL_UART_Receive_DMA(huart4, uart_buf, UART_BUF_LEN);

    while (1) {
//        imu.read();
        gps.get_pvt();
    }
}

// Define all C function calls from main.c below
extern "C"
{
    void C_app_main(I2C_HandleTypeDef *hi2c1, UART_HandleTypeDef *huart4,
            DMA_HandleTypeDef *hdma_uart4_rx)
    {
        CPP_app_main(hi2c1, huart4, hdma_uart4_rx);
    }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart4)
{
    HAL_UART_Receive_DMA(huart4, uart_buf, UART_BUF_LEN);
}
