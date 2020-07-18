#ifndef __MAIN_H
#define __MAIN_H

//Library Standard C/C++
#include <stdbool.h> // thu vien bool true false
#include "string.h"
#include <stdio.h>
#include <stdlib.h>

//Library Standard STM32F4
#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_it.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_dma.h"
#include "misc.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_i2c.h"

//Include Library for FreeRTOS
#include "cmsis_os.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#define mainCHECK_TASK_PRIORITY			( tskIDLE_PRIORITY + 2 )
#define mainFLASH_TASK_PRIORITY			( tskIDLE_PRIORITY + 1 )
#define mainINTEGER_TASK_PRIORITY   ( tskIDLE_PRIORITY )


//Include library for SDCard

//Include for RTC
#include "rtc_usart.h"
#include "rtc_i2c.h"
#include "rtc_ds1307.h"

//Derlace varible 
double data_sensor[4];
typedef struct {
	double tds __attribute__((packed)) __attribute__((aligned(4)));
	double PH __attribute__((packed)) __attribute__((aligned(4)));
	double temperature __attribute__((packed)) __attribute__((aligned(4)));
	double sonic __attribute__((packed)) __attribute__((aligned(4)));
} frame_esp;

typedef enum {FAILED = 0, PASSED = !FAILED} Status;

#define BYTE_FIRST_START_PAGE 0x66	//byte start xac dinh page
#define BYTE_FIRST_SEND_DATA 0xF5 //byte start send data of page
#define END_DATA 0xFF
#define LENGTH_DATA_HMI 100

typedef enum {
	HOME 								= 0U,
	WARINING            = 1U,
	WIFI            		= 2U,
	PPM           			= 3U,
	PPM2            		= 4U,
	PH            			= 5U,
	TANK            		= 6U,
	TANK2            		= 7U,
	VALVE            		= 8U,
	VALVE2            	= 9U,
	PERISTALTIC_PUMP    = 10U,
	PUMP            		= 11U,
	PUMP2            		= 12U,
	PUMP3            		= 13U,
	PUMP4            		= 14U,
	PUMP5            		= 15U,
	GRAPH            		= 16U,
	SETTING            	= 17U,
	SETTING_DATE        = 18U,
	REFILL_DATE         = 19U,
	CAL_PH 							= 21U,
	CAL_PPM            	= 22U,
	CAL_TEMP            = 23U,
	CAL_SONIC           = 24U,
	CAL_FLOW            = 25U,
	PROFILE            	= 26U,
	FERITILIZER         = 27U, 
	UNKNOWNPAGE
}TypeofPage;
typedef enum {
	START_PAGE 	= 	0U,
	SEND_DATA,
	UNKNOWN
}TypeofData;
TypeofPage page = UNKNOWNPAGE;
TypeofData typedata = UNKNOWN;
uint8_t countRxData = 0 ;
uint8_t DataHMI[LENGTH_DATA_HMI];


Status Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);
/**
  * @brief  Configures the USART2 Peripheral.
  * @param  None
  * @retval None
  */
static void USART1_Config(void)
{
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  /* Peripheral Clock Enable -------------------------------------------------*/
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  
  /* Enable USART clock */
  RCC_APB2PeriphClockCmd (RCC_APB2Periph_USART1, ENABLE);
 
  /* USARTx GPIO configuration -----------------------------------------------*/ 
  /* Connect USART pins to AF7 */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART2);
  
  /* Configure USART Tx and Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
 
  /* USARTx configuration ----------------------------------------------------*/
  /* Enable the USART OverSampling by 8 */
  USART_OverSampling8Cmd(USART1, ENABLE); 
  
  /* USARTx configured as follows:
        - BaudRate = 5250000 baud
		   - Maximum BaudRate that can be achieved when using the Oversampling by 8
		     is: (USART APB Clock / 8) 
			 Example: 
			    - (USART3 APB1 Clock / 8) = (42 MHz / 8) = 5250000 baud
			    - (USART1 APB2 Clock / 8) = (84 MHz / 8) = 10500000 baud
		   - Maximum BaudRate that can be achieved when using the Oversampling by 16
		     is: (USART APB Clock / 16) 
			 Example: (USART3 APB1 Clock / 16) = (42 MHz / 16) = 2625000 baud
			 Example: (USART1 APB2 Clock / 16) = (84 MHz / 16) = 5250000 baud
        - Word Length = 8 Bits
        - one Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */ 
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  /* When using Parity the word length must be configured to 9 bits */
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);
         
  /* Enable USART1 */
  USART_Cmd(USART1, ENABLE);

}


/**
  * @brief  Configures the USART2 Peripheral.
  * @param  None
  * @retval None
  */
static void USART3_Config(void)
{
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
  /* Peripheral Clock Enable -------------------------------------------------*/
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  
  /* Enable USART clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
 
  /* USARTx GPIO configuration -----------------------------------------------*/ 
  /* Connect USART pins to AF7 */
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);
  
  /* Configure USART Tx and Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
 
  /* USARTx configuration ----------------------------------------------------*/
  /* Enable the USART OverSampling by 8 */
  USART_OverSampling8Cmd(USART3, ENABLE); 
  
  /* USARTx configured as follows:
        - BaudRate = 5250000 baud
		   - Maximum BaudRate that can be achieved when using the Oversampling by 8
		     is: (USART APB Clock / 8) 
			 Example: 
			    - (USART3 APB1 Clock / 8) = (42 MHz / 8) = 5250000 baud
			    - (USART1 APB2 Clock / 8) = (84 MHz / 8) = 10500000 baud
		   - Maximum BaudRate that can be achieved when using the Oversampling by 16
		     is: (USART APB Clock / 16) 
			 Example: (USART3 APB1 Clock / 16) = (42 MHz / 16) = 2625000 baud
			 Example: (USART1 APB2 Clock / 16) = (84 MHz / 16) = 5250000 baud
        - Word Length = 8 Bits
        - one Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */ 
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  /* When using Parity the word length must be configured to 9 bits */
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART3, &USART_InitStructure);
         
  /* Enable USART2 */
  USART_Cmd(USART3, ENABLE);
	
	 /* NVIC configuration */
  /* Configure the Priority Group to 4 bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  
  /* Enable the USART2 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	/*Enable Interrupt USART*/
	USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
}

/**
  * @brief  Configures the I2C2 Peripheral.
  * @param  None
  * @retval None
  */
static void I2C2_Config(void)
{
		GPIO_InitTypeDef GPIO_InitStruct;
    I2C_InitTypeDef I2C_InitStruct;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
     
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);
    
    I2C_InitStruct.I2C_ClockSpeed = 100000;
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStruct.I2C_OwnAddress1 = 0x00;
    I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C2, &I2C_InitStruct);
    
    I2C_Cmd(I2C2, ENABLE);
}
/**
  * @brief  Configures the SysTick Base time to 1 ms.
  * @param  None
  * @retval None
  */
void SysTickConfig(void)
{
  /* Set SysTick Timer for 1ms interrupts  */
  if (SysTick_Config(SystemCoreClock / 1000))
  {
    /* Capture error */
    while (1);
  }
  /* Configure the SysTick handler priority */
  NVIC_SetPriority(SysTick_IRQn, 0x0F); // Priority = 15
}

void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);   // Do Port D xung tu bus ahb1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;              //Output, tro keo
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;             //Clock GPIO 50Mhz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);  
	
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);                      //Cai dat GPIOC
} 

/**
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval PASSED: pBuffer1 identical to pBuffer2
  *         FAILED: pBuffer1 differs from pBuffer2
  */

Status Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if (*pBuffer1 != *pBuffer2)
    {
      return FAILED;
    }
    pBuffer1++;
    pBuffer2++;
  }
  
  return PASSED;
}

/**
  * @brief  Configurate SPI1, 
						**A4 -- CS/NSS
						**A5 -- SCK
						**A6 -- MISO
						**A7 -- MOSI
  * @param  None
  * @retval None
  */
void SPI1_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed=GPIO_High_Speed;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5 | GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed=GPIO_High_Speed;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA, GPIO_Pin_4);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);
	
	SPI_Cmd(SPI1, ENABLE);
}

/**
  * @brief  Send data by SPI1
  * @param  u8Data : Data want to send (1 byte)
  * @retval the most recent received data by the SPI1 peripheral. 
  */
uint8_t SPI_Exchange(uint8_t u8Data)
{
	SPI_I2S_SendData(SPI1, u8Data);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET) {
	}
	return SPI_I2S_ReceiveData(SPI1);
}




/**
	* Khai bao Semphore and Task
	*
	*/
SemaphoreHandle_t xSemaphoreUARTRX = NULL; // Semaphore UART from ISR

void handleUARTRX(void *pvParameters)
{
	xSemaphoreUARTRX = xSemaphoreCreateBinary();
	countRxData = 0;
	memset(DataHMI,'\0',LENGTH_DATA_HMI);

	//uint8_t countEndFrame = 0;
	while(1)
	{		
			
			if (xSemaphoreTake( xSemaphoreUARTRX, 0) == pdTRUE ){
				// Xu ly nhan byte dau tien
				if (countRxData == 0 && DataHMI[countRxData] != '\0') {
				switch (DataHMI[countRxData]) {
					case BYTE_FIRST_START_PAGE:
					{
						typedata = START_PAGE;
						countRxData = 1;
						continue;
					}
					case BYTE_FIRST_SEND_DATA:
					{
						typedata = SEND_DATA;
						countRxData = 1;
						continue;
					}
					default:
					{
						typedata = UNKNOWN;
						break;
					}
				}
			}
				if (typedata == START_PAGE) {
					if (DataHMI[countRxData] != END_DATA && countRxData <5 && countRxData != 1 )
					{
						countRxData = 0;
						typedata = UNKNOWN;
						page = UNKNOWNPAGE;
						USART_puts((char *)DataHMI); //test code
						memset(DataHMI,'\0',LENGTH_DATA_HMI);
						continue;
					}
					countRxData += 1;
					if (DataHMI[2] == END_DATA && DataHMI[3] == END_DATA && DataHMI[4] == END_DATA) {
						countRxData = 0;
						typedata = UNKNOWN;
						USART_puts((char *)DataHMI); //test code
					switch (DataHMI[1]) {
						case HOME:
						{
							page = HOME;
							break;
						}
						case WARINING:
						{	
							page = WARINING;
							break;
						}
						case WIFI:
						{
							page = WIFI;
							break;
						}
						case PPM:
						{
							page = PPM;
							break;
						}
						case PPM2:
						{
							page = PPM2;
							break;
						}
						case PH:
						{
							page = PH;
							break;
						}
						case TANK:
						{
							page = TANK;
							break;
						}
						case TANK2:
						{
							page = TANK2;
							break;
						}
						case VALVE:
						{
							page = VALVE;
							break;
						}
						case VALVE2:
						{
							page = VALVE2;
							break;
						}
						case PERISTALTIC_PUMP:
						{
							page = PERISTALTIC_PUMP;
							break;
						}
						case PUMP:
						{
							page = PUMP;
							break;
						}
						case PUMP2:
						{
							page = PUMP2;
							break;
						}
						case PUMP3:
						{
							page = PUMP3;
							break;
						}
						case PUMP4:
						{
							page = PUMP4;
							break;
						}
						case PUMP5:
						{
							page = PUMP5;
							break;
						}
						case GRAPH:
						{
							page = GRAPH;
							break;
						}
						case SETTING:
						{
							page = SETTING;
							break;
						}
						case SETTING_DATE:
						{
							page = SETTING_DATE;
							break;
						}
						case REFILL_DATE:
						{
							page = REFILL_DATE;
							break;
						}
						case CAL_PH:
						{
							page = CAL_PH;
							break;
						}
						case CAL_PPM:
						{
							page = CAL_PPM;
							break;
						}
						case CAL_TEMP:
						{
							page = CAL_TEMP;
							break;
						}
						case CAL_SONIC:
						{
							page = CAL_SONIC;
							break;
						}
						case CAL_FLOW:
						{
							page = CAL_FLOW;
							break;
						}
						case PROFILE:
						{
							page = PROFILE;
							break;
						}
						case FERITILIZER:
						{
							page = FERITILIZER;
							break;
						}
						default:
						{
							page = UNKNOWNPAGE;
							break;
						}
					}
					memset(DataHMI,'\0',LENGTH_DATA_HMI);
					}
				}
				if (typedata == SEND_DATA) {
					countRxData += 1;
					if(countRxData == 2)
					{
						
					}
					
				}
				
		}
	}
}

void pageHOME(void *pvParameters)
{
	rtc_ds1307_datetime_t rtc_datetime;
	while (1)
	{
		if (page == HOME)
		{
		
		}
	}
}
/**
  * @brief  Configures the USART Peripheral.
  * @param  None
  * @retval None
  */
void USART_Config()
{
	//Enable UART1 for Comunication with PC
	USART1_Config();
	//Enable UART2 with interrupt RXNE
	USART3_Config(); 
}
void I2C_Config()
{
	//Enable I2C2 for RTC
	I2C2_Config();
}


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
#endif
