#include "main.h"

/*Khai bao Semphore and Task*/
void vTaskLed(void *pvParameters);
void HandleUART(void *pvParameters);
static xSemaphoreHandle xButton1Semphore = NULL;   //Semaphore BUTTON1

uint8_t aTxBuffer[BUFFERSIZE] = "USART DMA Example: Communication between two USART using DMA";
uint8_t aRxBuffer [BUFFERSIZE];



static Status Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);
static void USART_Config(void);
static void SysTickConfig(void);
static void Configuration(void);

/*Khai bao phuc vu cho nhan du lieu Rx from HMI UART*/
/*
*	enum kiem tra du lieu nhan duoc thuoc Start, ID, Data hay Ket thuc hoac Error
*	Chuong trinh con tra ve trang thai.
*/
typedef enum
{
	START = 0U, 	// start frame
	PREPARE_BYTE, // byte empty
	ID_DEVICE,		// id cua thiet bi tren hmi
	DATA,					// trang thai cua thiet bi
	BYTES_END,		// ket thuc truyen
	ERROR_FRAME	// error frame
}DataTypeRx;
DataTypeRx prevStatus = ERROR_FRAME; //Trang thai truoc 
static DataTypeRx checkByteType(uint16_t Data);
#define LENGTH  8
uint8_t rxData[LENGTH] ;
uint8_t countRxData = 0 ;
uint8_t countEndRx = 0 ;
static Status HandleRxData(uint8_t* bufferHandle);
 
/* Configuration Semaphore and task for freeRTOS -----------------------------------------------------*/
SemaphoreHandle_t SenaphoreISRUART=NULL;

int main(void)
{
	  /* USART configuration -----------------------------------------------------*/
  USART_Config();

	Configuration();         //Config LED PIN
	/*Clear mang ky tu nhan ve*/
	memset(rxData,'\0',LENGTH);
  
  /* Enable DMA USART TX Stream */
  DMA_Cmd(USARTx_TX_DMA_STREAM,ENABLE);
   
  /* Enable USART DMA TX Requsts */
  USART_DMACmd(USARTx, USART_DMAReq_Tx, ENABLE);

  /* Waiting the end of Data transfer */
  while (USART_GetFlagStatus(USARTx,USART_FLAG_TC)==RESET);    
  while (DMA_GetFlagStatus(USARTx_TX_DMA_STREAM,USARTx_TX_DMA_FLAG_TCIF)==RESET);
	
  /* Clear DMA Transfer Complete Flags */
  DMA_ClearFlag(USARTx_TX_DMA_STREAM,USARTx_TX_DMA_FLAG_TCIF);
  /* Clear USART Transfer Complete Flags */
  USART_ClearFlag(USARTx,USART_FLAG_TC); 
	
	vSemaphoreCreateBinary(SenaphoreISRUART);

	if( SenaphoreISRUART != NULL ){

			xTaskCreate(vTaskLed, "ON OFF LED neu nhan duoc dung du lieu", 1000, NULL, 1, NULL);
			xTaskCreate(HandleUART, "xu ly qua trinh nhan va truyen", 1000, NULL, 1, NULL);
			vTaskStartScheduler();
	}

	for(;;){}
//  while (1)
//	{
//		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)==SET)
//		{
//			memset(aTxBuffer,'\0',sizeof(aTxBuffer));
//			memmove(aTxBuffer,rxData,sizeof(rxData));
//			DMA_Cmd(USARTx_TX_DMA_STREAM,ENABLE);
//			while (USART_GetFlagStatus(USARTx,USART_FLAG_TC)==RESET);    
//			while (DMA_GetFlagStatus(USARTx_TX_DMA_STREAM,USARTx_TX_DMA_FLAG_TCIF)==RESET);
//			DMA_Cmd(USARTx_TX_DMA_STREAM,DISABLE);
//			/* Clear DMA Transfer Complete Flags */
//			DMA_ClearFlag(USARTx_TX_DMA_STREAM,USARTx_TX_DMA_FLAG_TCIF);
//			/* Clear USART Transfer Complete Flags */
//			USART_ClearFlag(USARTx,USART_FLAG_TC); 			
//		}
}
//Haha demo thanh cong roi 

/**
  * @brief  Configures the USART Peripheral.
  * @param  None
  * @retval None
  */
static void USART_Config(void)
{
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
  /* Peripheral Clock Enable -------------------------------------------------*/
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(USARTx_TX_GPIO_CLK | USARTx_RX_GPIO_CLK, ENABLE);
  
  /* Enable USART clock */
  USARTx_CLK_INIT(USARTx_CLK, ENABLE);
  
  /* Enable the DMA clock */
  RCC_AHB1PeriphClockCmd(USARTx_DMAx_CLK, ENABLE);
  
  /* USARTx GPIO configuration -----------------------------------------------*/ 
  /* Connect USART pins to AF7 */
  GPIO_PinAFConfig(USARTx_TX_GPIO_PORT, USARTx_TX_SOURCE, USARTx_TX_AF);
  GPIO_PinAFConfig(USARTx_RX_GPIO_PORT, USARTx_RX_SOURCE, USARTx_RX_AF);
  
  /* Configure USART Tx and Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  
  GPIO_InitStructure.GPIO_Pin = USARTx_TX_PIN;
  GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = USARTx_RX_PIN;
  GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStructure);
 
  /* USARTx configuration ----------------------------------------------------*/
  /* Enable the USART OverSampling by 8 */
  USART_OverSampling8Cmd(USARTx, ENABLE); 
  
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
  USART_Init(USARTx, &USART_InitStructure);

  /* Configure DMA controller to manage USART TX and RX DMA request ----------*/ 
   
	DMA_InitTypeDef DMA_InitStructure;
  /* Configure DMA Initialization Structure */
	
  DMA_InitStructure.DMA_BufferSize = strlen((char*)aTxBuffer);
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(USARTx->DR)) ;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  /* Configure TX DMA */
  DMA_InitStructure.DMA_Channel = USARTx_TX_DMA_CHANNEL ;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
  DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)aTxBuffer ;
  DMA_Init(USARTx_TX_DMA_STREAM,&DMA_InitStructure);
//  /* Configure RX DMA */
//  DMA_InitStructure.DMA_Channel = USARTx_RX_DMA_CHANNEL ;
//  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;
//  DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)aRxBuffer ; 
//  DMA_Init(USARTx_RX_DMA_STREAM,&DMA_InitStructure);
         
  /* Enable USART */
  USART_Cmd(USARTx, ENABLE);
	
	 /* NVIC configuration */
  /* Configure the Priority Group to 2 bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  
  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	/*Enable Interrupt USART*/
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
}
static DataTypeRx checkByteType(uint16_t Data)
{
	if (Data == 0x65 && (prevStatus == ERROR_FRAME || prevStatus == BYTES_END))
	{
		prevStatus = START;
		return START;
	}
	if (Data == 0x00)
	{
		switch (prevStatus)
		{
			case START:
			{
				prevStatus = PREPARE_BYTE;
				return PREPARE_BYTE;
			}
			case PREPARE_BYTE:
			{
				prevStatus = ID_DEVICE;
				return ID_DEVICE;
			}
			case ID_DEVICE:
			{
				prevStatus = DATA;
				return DATA;
			}
			case DATA:
			{
				return ERROR_FRAME;
			}
			case BYTES_END:
			{
				return ERROR_FRAME;
			}
			default:
				return ERROR_FRAME;
		}	
		
	}
	if (prevStatus == PREPARE_BYTE && Data != 0x00)
	{
		return ID_DEVICE;
	}
	if (prevStatus == ID_DEVICE && Data != 0x00)
	{
		return DATA;
	}
	if (prevStatus == DATA && Data == 0xFF )
	{
		return BYTES_END;
	}
	if (prevStatus == BYTES_END && Data == 0xFF)
			return BYTES_END;
	return ERROR_FRAME;	
}

static Status HandleRxData(uint8_t* bufferHandle)
{
	if (checkByteType(*bufferHandle) == START );
	else
	{
		memset(bufferHandle,'\0',LENGTH);
		return FAILED;
	}
	if (checkByteType(*(bufferHandle+1)) == PREPARE_BYTE )
		
	memset(rxData,'\0',LENGTH);
}

void USART2_IRQHandler()
{
	if (USART_GetITStatus(USART2,USART_IT_RXNE)==SET )
	{
		rxData[countRxData] = USART_ReceiveData(USART2);
		countRxData++;
		}	
	if (countRxData == 7){
		countRxData = 0;
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR( SenaphoreISRUART, &xHigherPriorityTaskWoken );
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
	}
	USART_ClearFlag(USART2,USART_FLAG_RXNE);
}
/**
  * @brief  Configures the SysTick Base time to 10 ms.
  * @param  None
  * @retval None
  */
static void SysTickConfig(void)
{
  /* Set SysTick Timer for 10ms interrupts  */
  if (SysTick_Config(SystemCoreClock / 100))
  {
    /* Capture error */
    while (1);
  }
  /* Configure the SysTick handler priority */
  NVIC_SetPriority(SysTick_IRQn, 0x0);
}

/**
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval PASSED: pBuffer1 identical to pBuffer2
  *         FAILED: pBuffer1 differs from pBuffer2
  */

static Status Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
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
static void Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitConf;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);   // Do Port D xung tu bus ahb1
	GPIO_InitConf.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13; 
	GPIO_InitConf.GPIO_Mode = GPIO_Mode_OUT;              //Output, tro keo
	GPIO_InitConf.GPIO_Speed = GPIO_High_Speed;             //Clock GPIO 50Mhz
	GPIO_InitConf.GPIO_OType = GPIO_OType_PP;
	GPIO_InitConf.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitConf);  
	
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	GPIO_InitConf.GPIO_Pin = GPIO_Pin_0; 
	GPIO_InitConf.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitConf.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitConf.GPIO_OType = GPIO_OType_PP;
	GPIO_InitConf.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitConf);                      //Cai dat GPIOC
} 




void vTaskLed(void *pvParameters)
{
	char led_val = 0;
	unsigned int Frequency = 100;         //100ms
	while(1)
	{
		if(xButton1Semphore != NULL)
		{
			if(xSemaphoreTake(xButton1Semphore, (portTickType)1) == pdTRUE)
			{
				led_val = 1 - led_val;
				GPIO_WriteBit(GPIOD,GPIO_Pin_12,led_val ? Bit_SET : Bit_RESET);
			}
		}
		vTaskDelay(Frequency);
	}
}

void HandleUART(void *pvParameters)
{

	while(1)
	{
		if (xSemaphoreTake( SenaphoreISRUART, pdMS_TO_TICKS(100)) == pdPASS ) 
		{
			
		}
	}
}

