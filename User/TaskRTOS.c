#include "TaskRTOS.h"

void HandleUART(void *pvParameters)
{
	uint8_t i = 0;
	xSemaphore = xSemaphoreCreateBinary();
	while(1)
	{		
			USART_SendData(USART2,(uint16_t)rxData[0]);
			vTaskDelay(100);
			if( xSemaphoreTake( xSemaphore, pdMS_TO_TICKS(10) ) == pdTRUE ){
				USART_SendData(USART2,(uint16_t)rxData[0]);
			if (rxData[0] == 0x53)
			{
				
				GPIO_WriteBit(GPIOD,GPIO_Pin_12,Bit_SET);
			}
		}
		//vTaskDelay(200);
	}
}
