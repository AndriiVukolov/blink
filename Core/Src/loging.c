#include "usart.h"
#include "string.h"


//UART_HandleTypeDef huart4;
//UART_HandleTypeDef huart1;

UART_HandleTypeDef UartInit(int type)
{
	UART_HandleTypeDef * uartHandle;

	if (type == 4)
	{
		uartHandle = &huart4;
		MX_UART4_Init();
	}
	else if (type == 1)
	{
		uartHandle = &huart1;
		MX_USART1_UART_Init();
	}
	return (*uartHandle);
}


void UartSendString(unsigned char * phrase, UART_HandleTypeDef * huart)
{
	int qty = 0;

	qty = strlen(phrase);
	if (HAL_UART_Transmit(huart, phrase, qty, 1000) != HAL_OK)
	  {
			Error_Handler();
	  }
}
