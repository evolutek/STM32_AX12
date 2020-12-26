/** @file dxl_hal.h @brief Interface to the hal version of dxl */

# pragma once

#include "stm32f3xx_hal.h"

void dxl_set_tx_callback(void (*tx_callback)());
void dxl_set_rx_callback(void (*rx_callback)());

HAL_StatusTypeDef dxl_hal_open(UART_HandleTypeDef *huart, const int baudrate);
void dxl_hal_close(void);
HAL_StatusTypeDef dxl_hal_abort();
HAL_StatusTypeDef dxl_hal_set_baudrate(int baudrate);
HAL_StatusTypeDef dxl_hal_tx(unsigned char *packet, const unsigned size);
HAL_StatusTypeDef dxl_hal_rx(unsigned char *packet, const unsigned size);
