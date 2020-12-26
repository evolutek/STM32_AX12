/**
 * Modification of the HAL of the Dynamixel SDK for STM32 UART.
 *
 * Arthus Leroy
 **/

# include "main.h"

UART_HandleTypeDef *dxl_huart = NULL;

static void (*tx_callback)(void) = NULL;
void dxl_set_tx_callback(void (*func)(void))
{
	tx_callback = func;
}

static void (*rx_callback)(void) = NULL;
void dxl_set_rx_callback(void (*func)(void))
{
	rx_callback = func;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (tx_callback != NULL)
		(*tx_callback)();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (rx_callback != NULL)
		(*rx_callback)();
}

HAL_StatusTypeDef dxl_hal_abort(void)
{
	ASSERT(dxl_huart != NULL);

	return HAL_UART_Abort(dxl_huart);
}

HAL_StatusTypeDef dxl_hal_set_baudrate(const int baudrate)
{
	ASSERT(dxl_huart != NULL);

	HAL_StatusTypeDef err;
	if ((err = dxl_hal_abort()) != HAL_OK)
		return err;

// FIXME
	__HAL_UART_DISABLE(dxl_huart);
//	LL_USART_SetBaudRate(dxl_huart->Instance, baudrate);
	__HAL_UART_ENABLE(dxl_huart);

	return HAL_OK;
}

HAL_StatusTypeDef dxl_hal_open(UART_HandleTypeDef *huart, const int baudrate)
{
	dxl_huart = huart;

	return dxl_hal_set_baudrate(baudrate);
}

void dxl_hal_close(void)
{
	dxl_huart = NULL;
}

HAL_StatusTypeDef dxl_hal_tx(unsigned char* packet, const unsigned size)
{
	ASSERT(dxl_huart != NULL);
	ASSERT(packet != NULL);
	ASSERT(size > 0);

	return HAL_UART_Transmit_DMA(dxl_huart, packet, size);
}

HAL_StatusTypeDef dxl_hal_rx(unsigned char *packet, const unsigned size)
{
	ASSERT(dxl_huart != NULL);
	ASSERT(packet != NULL);
	ASSERT(size > 0);

	return HAL_UART_Receive_DMA(dxl_huart, packet, size);
}
