#ifndef __MAIN_H
#define __MAIN_H

#include <stdbool.h> // thu vien bool true false

#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_dma.h"
#include "misc.h"

//For FreeRTOS
#include "cmsis_os.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "string.h"
#include <stdio.h>
#include <stdlib.h>

#define mainCHECK_TASK_PRIORITY			( tskIDLE_PRIORITY + 2 )
#define mainFLASH_TASK_PRIORITY			( tskIDLE_PRIORITY + 1 )
#define mainINTEGER_TASK_PRIORITY   ( tskIDLE_PRIORITY )
void vTaskLed1(void *pvParameters);
void vTaskLed2(void *pvParameters);

#endif

typedef enum {FAILED = 0, PASSED = !FAILED} Status;

/* Definition for USARTx resources ******************************************/
#define USARTx                           USART2
#define USARTx_CLK                       RCC_APB1Periph_USART2
#define USARTx_CLK_INIT                  RCC_APB1PeriphClockCmd
#define USARTx_IRQn                      USART2_IRQn
#define USARTx_IRQHandler                USART2_IRQHandler

#define USARTx_TX_PIN                    GPIO_Pin_2           
#define USARTx_TX_GPIO_PORT              GPIOA                      
#define USARTx_TX_GPIO_CLK               RCC_AHB1Periph_GPIOA
#define USARTx_TX_SOURCE                 GPIO_PinSource2
#define USARTx_TX_AF                     GPIO_AF_USART2

#define USARTx_RX_PIN                    GPIO_Pin_3               
#define USARTx_RX_GPIO_PORT              GPIOA                   
#define USARTx_RX_GPIO_CLK               RCC_AHB1Periph_GPIOA
#define USARTx_RX_SOURCE                 GPIO_PinSource3
#define USARTx_RX_AF                     GPIO_AF_USART2

/* Definition for DMAx resources ********************************************/
#define USARTx_DR_ADDRESS                ((uint32_t)USART2 + 0x04) 

#define USARTx_DMA                       DMA1
#define USARTx_DMAx_CLK                  RCC_AHB1Periph_DMA1
	 
#define USARTx_TX_DMA_CHANNEL            DMA_Channel_4
#define USARTx_TX_DMA_STREAM             DMA1_Stream6
#define USARTx_TX_DMA_FLAG_FEIF          DMA_FLAG_FEIF6
#define USARTx_TX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF6
#define USARTx_TX_DMA_FLAG_TEIF          DMA_FLAG_TEIF6
#define USARTx_TX_DMA_FLAG_HTIF          DMA_FLAG_HTIF6
#define USARTx_TX_DMA_FLAG_TCIF          DMA_FLAG_TCIF6
						
#define USARTx_RX_DMA_CHANNEL            DMA_Channel_4
#define USARTx_RX_DMA_STREAM             DMA1_Stream5
#define USARTx_RX_DMA_FLAG_FEIF          DMA_FLAG_FEIF5
#define USARTx_RX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF5
#define USARTx_RX_DMA_FLAG_TEIF          DMA_FLAG_TEIF5
#define USARTx_RX_DMA_FLAG_HTIF          DMA_FLAG_HTIF5
#define USARTx_RX_DMA_FLAG_TCIF          DMA_FLAG_TCIF5

#define USARTx_DMA_TX_IRQn               DMA1_Stream5_IRQn
#define USARTx_DMA_RX_IRQn               DMA1_Stream6_IRQn
#define USARTx_DMA_TX_IRQHandler         DMA1_Stream6_IRQn   
#define USARTx_DMA_RX_IRQHandler         DMA1_Stream5_IRQn 

#define BUFFERSIZE                       1000
#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
