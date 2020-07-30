#ifndef __MAIN_H
#define __MAIN_H

//Library Standard C/C++
#include <stdbool.h> // thu vien bool true false
#include "string.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

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
#include "stm32f4xx_adc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
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


//Include library for onewire
#include "ds18b20_mflib.h"
//#include "DS18B20.h"

//Include for RTC
#include "rtc_usart.h"
#include "rtc_i2c.h"
#include "rtc_ds1307.h"

/*-----------------------------------Derlace varible-----------------------------------*/
//Tham so dinh nghia
#define FALSE 0
#define TRUE 1
/*
**	BIEN PHUC VU CHO DO DAC CAM BIEN
*/
/* Luu gia tri cua cam bien tds vao mang 
**        data_sensor[0] |  gia tri cam bien tds
**				data_sensor[1] |  gia tri cam bien pH
**				data_sensor[2] |  gia tri cam bien nhiet do ds18b20
**				data_sensor[3] |  gia tri cam bien sieu am
*/
float data_sensor[4]; 
//	Frame truyen du lieu qua ESP32
typedef struct {
	double tds __attribute__((packed)) __attribute__((aligned(4)));
	double PH __attribute__((packed)) __attribute__((aligned(4)));
	double temperature __attribute__((packed)) __attribute__((aligned(4)));
	double sonic __attribute__((packed)) __attribute__((aligned(4)));
} frame;
/* Luu gia tri cua cam bien tu DMA ADC1 vao mang 
**        uhADCxConvertedValue[0] |  gia tri adc cam bien pH ( ADC1_IN1)
**				uhADCxConvertedValue[1] |  gia tri adc cam bien tds ( ADC1_IN10)
**				uhADCxConvertedValue[2] |  gia tri cam bien dong dien ZCT103p 20A (ADC1_IN11)
*/
__IO uint16_t uhADCxConvertedValue[3] = {0,0,0};
/*
**	BIEN PHUC VU CHO GIAI THUAT MACHINE LEARNING
*/
typedef enum  {
  PH_DOWN = 1U,
  PH_UP,
	FRE_A,
	FRE_B,
	NOMODE
}modeOfPump;

modeOfPump modePumpDoser[4] = {NOMODE,NOMODE,NOMODE,FRE_B};
uint8_t speedPumpDoser[4] = {0,0,0,0};

#define maxSize 20 // size toi da cua mang
unsigned int phDownDosingInc[maxSize] = {0};
unsigned int phUpDosingInc[maxSize]={0};
unsigned int tdsDosingInc[maxSize]={0};

uint8_t phDownArrayBlock = 0;
uint8_t phUpArrayBlock = 0;
uint8_t tdsArrayBlock = 0;

bool savePhDownResult = false;
bool savePhUpResult = false;
bool saveTdsResult = false;

bool logPhDownResult = false;
bool logPhUpResult = false;
bool logTdsResult = false;

float phDownMls = 0, previousPhDownMls = 0;
float phUpMls = 0, previousPhUpMls = 0;
float tdsMls = 0, previousTdsMls = 0;

float previousPhDownSensor = 0, phDownSensorHistory = 0;
float previousPhUpSensor = 0, phUpSensorHistory = 0;
float previousTdsSensor = 0, tdsSensorHistory = 0;

float phDownMultipler = 1;
float phUpMultipler = 1;
float tdsMultipler = 1;
float tdsAMultipler = 1;
float tdsBMultipler = 1;

float dosingAmount = 1;

float targetMinPh = 6.0;
float targetMaxPh = 6.5;
float phOffset = 0.5;

unsigned int targetMinTds = 400;
unsigned int targetMaxTds = 600;
unsigned int tdsOffset = 20;

unsigned int targetMinTdsALL = 0;
unsigned int targetMaxTdsALL = 0;
unsigned int tdsOffsetALL = 0;

int targetMinTankLv = 0;
int targetMaxTankLv = 100;
int TankLvOffset = 2;


bool adjustPhDown = false;
uint8_t swapInterval = 2;

float ratiosDosing = 0;
bool dosingComplete = false;
uint8_t dosingInterval = 4;

uint16_t tankDepth = 0;

uint8_t sensorPHinDay[1000];
uint8_t sensorPPMinDay[1000];
uint8_t sensorTANKinDay[1000];
uint8_t sensorTEMPinDay[1000];

uint16_t NumValSaved = 0;

float errorMarginPH = 0.0;
float errorMarginPPM = 0.0;
float errorMarginTANK = 0.0;
float errorMarginTEMP = 0.0;

uint16_t WifiInterval = 0;
/*
**	BIEN PHUC VU CHO NGUOI DUNG
*/
uint16_t countEXTI = 0;


/*
**	BIEN PHUC VU CHO HE THONG
*/
uint8_t currentlyDosing = FALSE;
/*
*	KHAI BAO BIEN VA DINH NGHIA PHUC VU XU LY GIAO TIEP HMI
*/
typedef enum {FAILED = 0, PASSED = !FAILED} Status;

#define BYTE_FIRST_START_PAGE 0x66	//byte start xac dinh page
#define BYTE_FIRST_SEND_DATA 0xF5 //byte start send data of page
#define END_DATA 0xFF // byte ket thuc chuoi du lieu tu HMI
#define LENGTH_DATA_HMI 100 // Do dai chuoi du lieu toi da tu HMI

//	Xac dinh page hien tai cua HMI
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
//	Xac dinh du lieu nhan duoc tu HMI thuoc loai gi. Bat dau page hoac du lieu cua page
typedef enum {
	START_PAGE 	= 	0U,
	SEND_DATA,
	UNKNOWN
}TypeofData;
//	Luu page hien tai 
TypeofPage page = UNKNOWNPAGE;
//	Luu page truoc do
TypeofPage prevpage = UNKNOWNPAGE;
//	Luu kieu du lieu hien tai
TypeofData typedata = UNKNOWN;

/*		GIAI THICH CAC Y  NGHIA CUA BIEN PHUC VU XU LY DU LIEU HMI
**	FlagTC la bien de thong bao da nhan du chuoi du lieu va da co 3 byte ket thuc la FF FF FF. 
**	FlagTC duoc set len bang ngat UART ket hop voi Semaphore de control task hoat dong

**	countRxData de phuc vu xac dinh noi luu mang du lieu nhan duoc. 
**	NOTE: KHONG DUOC THAY DOI GIA TRI CUA countRxData. NO CHI DUOC THAY DOI TRONG NGAT USART

**	FlagHanldeData de thong bao du lieu nhan duoc da duoc xu ly chua
**	Bat cu khi nao lay du lieu IDMHI va DataHMI thi phai set FlagHandleData thanh FALSE

**	Data nhan duoc tu HMI se duoc luu vao DataHMIREV va duoc tach ra thanh 2 Data:
**		+IDHMI: Xac dinh cac toolbox da duoc tac dong tu HMI. Moi page se co cac ID khac nhau va gia tri khac nhau
**		+DataHMI[]: Data cua page gui ve. 
**	Phan tich code HMI de hieu ro hon
*/
uint8_t FlagTC = FALSE; // Co bao da nhan va xu ly du lieu
uint8_t countRxData = 0;
uint8_t FlagHandleData = TRUE;
uint8_t DataHMIREV[LENGTH_DATA_HMI];
uint8_t DataHMI[LENGTH_DATA_HMI];
uint8_t IDHMI = 0;

/**
  * @brief  So sanh 2 chuoi voi do dai x
  * @param  pBuffer1 : Chuoi 1
						pBuffer2 : Chuoi 2
						BufferLength : So phan tu so sanh
  * @retval None
  */
Status Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);
/**
  * @brief  Configures the USART2 Peripheral.
  * @param  None
  * @retval None
  */
static void USART1_Config(void) {
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  /* Peripheral Clock Enable -------------------------------------------------*/
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  
  /* Enable USART clock */
  RCC_APB2PeriphClockCmd (RCC_APB2Periph_USART1, ENABLE);
 
  /* USARTx GPIO configuration -----------------------------------------------*/ 
  /* Connect USART pins to AF7 */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
  
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

static void USART2_Config(void) {
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
  /* Peripheral Clock Enable -------------------------------------------------*/
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  
  /* Enable USART clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
 
  /* USARTx GPIO configuration -----------------------------------------------*/ 
  /* Connect USART pins to AF7 */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
  
  /* Configure USART Tx and Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
 
  /* USARTx configuration ----------------------------------------------------*/
  /* Enable the USART OverSampling by 8 */
  USART_OverSampling8Cmd(USART2, ENABLE); 
  
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
  USART_Init(USART2, &USART_InitStructure);
  
	/* NVIC configuration */
  
/* Enable the USART3 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	/*Enable Interrupt USART*/
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
	
  /* Enable USART2 */
  USART_Cmd(USART2, ENABLE);
	

	

}
/**
  * @brief  Configures the USART2 Peripheral.
  * @param  None
  * @retval None
  */
static void USART3_Config(void) {
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
  
  /* Enable the USART3 Interrupt */
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
static void I2C2_Config(void) {
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
void SysTickConfig(void) {
  /* Set SysTick Timer for 1us interrupts  */
  if (SysTick_Config(SystemCoreClock / 1000000))
  {
    /* Capture error */
    while (1);
  }
  /* Configure the SysTick handler priority */
  NVIC_SetPriority(SysTick_IRQn, 0x0F); // Priority = 15
}

static void GPIO_Configuration(void){
	/**
  * @param  
  * 				GPIO_Hight_Speed is Speed 100MHz
  * 
  */
	/*-----------------------------PORT D----------------------------*/	
	GPIO_InitTypeDef GPIOD_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);   
	
	GPIOD_InitStructure.GPIO_Pin = GPIO_Pin_12; 
	GPIOD_InitStructure.GPIO_Mode = GPIO_Mode_OUT;              
	GPIOD_InitStructure.GPIO_Speed = GPIO_High_Speed;             
	GPIOD_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIOD_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIOD_InitStructure);  
	
	/*Configure D12 D13 D14 D15 for PWM output TIM4*/
	GPIOD_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIOD_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIOD_InitStructure.GPIO_Speed = GPIO_High_Speed;
  GPIOD_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIOD_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOD, &GPIOD_InitStructure); 

  /* Connect TIM4 pins to AF */  
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4); 
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4); 
	
/*-----------------------------PORT A----------------------------*/	
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
//	GPIO_InitTypeDef GPIOA_InitStructure;
//	
//	GPIOA_InitStructure.GPIO_Pin = GPIO_Pin_0; 
//	GPIOA_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//	GPIOA_InitStructure.GPIO_Speed = GPIO_High_Speed;
//	GPIOA_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIOA_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_Init(GPIOA, &GPIOA_InitStructure);                      
	
	
	
	/*-----------------------------PORT B----------------------------*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	GPIO_InitTypeDef GPIOB_InitStructure;
	
	/* Configure PB2 pin for One wire Temperatute sensor*/
	GPIOB_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIOB_InitStructure.GPIO_Mode = GPIO_Mode_OUT;              
	GPIOB_InitStructure.GPIO_Speed = GPIO_High_Speed;            
	GPIOB_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIOB_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOB, &GPIOB_InitStructure);  
	  
	/* Configure PB0 pin for external interrupt sensor Flow */
  GPIOB_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIOB_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIOB_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_Init(GPIOB, &GPIOB_InitStructure);
	
	
} 


/**
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval PASSED: pBuffer1 identical to pBuffer2
  *         FAILED: pBuffer1 differs from pBuffer2
  */

Status Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength) {
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
static void SPI1_Config(void) {
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
uint8_t SPI_Exchange(uint8_t u8Data) {
	SPI_I2S_SendData(SPI1, u8Data);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET) {
	}
	return SPI_I2S_ReceiveData(SPI1);
}
static void TIM2_Config() {
	TIM_TimeBaseInitTypeDef TIM_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_InitStructure.TIM_Prescaler = 84 - 1;
	TIM_InitStructure.TIM_Period = 0xFFFFFFFF ; // Update event every overflow
	TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_InitStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM2, &TIM_InitStructure);

  TIM_Cmd(TIM2, ENABLE);
}
static void TIM5_Config() {
	TIM_TimeBaseInitTypeDef TIM_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_InitStructure.TIM_Prescaler = 42000 - 1;
	TIM_InitStructure.TIM_Period = 0xFFFFFFFF ; // Update event every overflow
	TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_InitStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM5, &TIM_InitStructure);

  TIM_Cmd(TIM5, ENABLE);
}
static void TIM4_Config() {
	TIM_TimeBaseInitTypeDef TIM_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	TIM_InitStructure.TIM_Period = 21000 - 1;
  TIM_InitStructure.TIM_Prescaler = 4 - 1;
  TIM_InitStructure.TIM_ClockDivision = 0;
  TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_InitStructure);
  TIM_Cmd(TIM4, ENABLE);
}

void mydelayus(unsigned int time) {

    TIM_SetCounter(TIM2,0);  //Load timer CNT = 0  
    while (TIM_GetCounter(TIM2) <= time);
}
void mydelayms(unsigned int time) {

    TIM_SetCounter(TIM5,0);  //Load timer CNT = 0  
    while (TIM_GetCounter(TIM5) <= (time*2));
}

uint8_t mystrlength(volatile uint8_t *buffer) {
	uint8_t countFF = 0;
	for (uint8_t i = 0;;i++)
	{
		if (*buffer == 0)
			countFF ++;
		if (countFF > 0 && *buffer != 0)
			countFF = 0;
		if (countFF == 4)
			return i-3;
		*buffer++;
	}
}
/**
  * @brief  ADC3 channel07 with DMA configuration
  * @note   This function Configure the ADC peripheral  
            1) Enable peripheral clocks
            2) DMA2_Stream0 channel2 configuration
            3) Configure ADC Channel7 pin as analog input
            4) Configure ADC3 Channel7 
  * @param  None
  * @retval None
  */
static void ADC1_Config(void) {
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  DMA_InitTypeDef       DMA_InitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;

  /* Enable ADCx, DMA and GPIO clocks ****************************************/ 
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  

  /* DMA2 Stream0 channel0 configuration **************************************/
  DMA_InitStructure.DMA_Channel = DMA_Channel_0;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) 0x4001204C;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&uhADCxConvertedValue[0];
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 3;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
 

  /* Configure ADC1 Channel1 pin as analog input ******************************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* Configure ADC1 Channel10 pin as analog input ******************************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Configure ADC1 Channel10 pin as analog input ******************************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
  /* ADC Common Init **********************************************************/
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_10Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  /* ADC1 Init ****************************************************************/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 3;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channel1 configuration **************************************/
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_3Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 2, ADC_SampleTime_3Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 3, ADC_SampleTime_3Cycles);
 /* Enable DMA request after last transfer (Single-ADC mode) */
  ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);


  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);
	/* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);
	
	DMA_Cmd(DMA2_Stream0, ENABLE);


}
static void EXTI_Line1_Config() {
	EXTI_InitTypeDef   EXTI_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;

  /* Enable SYSCFG clock (Manual page 289)*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
  /* Connect EXTI Line0 to PB1 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource1);

  /* Configure EXTI Line0 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line1;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI Line1 Interrupt to the 7 priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
/**
	* Khai bao Semphore and Task
	*
	*/
//***************************HMI***************************//
SemaphoreHandle_t xSemaphoreUARTRX = NULL; // Semaphore UART from ISR
SemaphoreHandle_t xSemaphoreDataHMI = NULL; // Semaphore notification has id and data
SemaphoreHandle_t xSemaphoreSonicSensor = NULL;
SemaphoreHandle_t xSemaphoredosingInterval = NULL;
SemaphoreHandle_t xSemaphoresensorsReady = NULL;
SemaphoreHandle_t xSemaphoreHandleData = NULL;
SemaphoreHandle_t xSemaphoreEXTI = NULL;
//Semaphore for handle page's HMI

xTaskHandle TaskpageHome;
xTaskHandle TaskpageWarning;
xTaskHandle TaskpageWifi;
xTaskHandle TaskpagePPM;
xTaskHandle TaskpagePPM2;
xTaskHandle TaskpagepH;
xTaskHandle TaskpageTank;
xTaskHandle TaskpageTank2;
xTaskHandle TaskpageValve;
xTaskHandle TaskpageValve2;
xTaskHandle TaskpagePeristaltic;
xTaskHandle TaskpagePump;
xTaskHandle TaskpagePump2;
xTaskHandle TaskpagePump3;
xTaskHandle TaskpagePump4;
xTaskHandle TaskpagePump5;
xTaskHandle TaskpageGraph;
xTaskHandle TaskpageSetting;
xTaskHandle TaskpageSettingDate;
xTaskHandle TaskpageRefilldate;
xTaskHandle TaskpageCalpH;
xTaskHandle TaskpageCalPPM;
xTaskHandle TaskpageCalTemp;
xTaskHandle TaskpageCalSonic;
xTaskHandle TaskpageFlow;
xTaskHandle TaskpageProfile;
xTaskHandle TaskpageFeritilizer;


void handleHMI(void *pvParameters);
void handleUARTRX(void *pvParameters);
//tasks page HMI
void pageHome(void *pvParameters);
void pageWarning(void *pvParameters);
void pageWifi(void *pvParameters);
void pagePPM(void *pvParameters);
void pagePPM2(void *pvParameters);
void pagepH(void *pvParameters);
void pageTank(void *pvParameters);
void pageTank2(void *pvParameters);
void pageValve(void *pvParameters);
void pageValve2(void *pvParameters);
void pagePeristaltic(void *pvParameters);
void pagePump(void *pvParameters);
void pagePump2(void *pvParameters);
void pagePump3(void *pvParameters);
void pagePump4(void *pvParameters);
void pagePump5(void *pvParameters);
void pageGraph(void *pvParameters);
void pageSetting(void *pvParameters);
void pageSettingDate(void *pvParameters);
void pageRefilldate(void *pvParameters);
void pageCalpH(void *pvParameters);
void pageCalPPM(void *pvParameters);
void pageCalTemp(void *pvParameters);
void pageCalSonic(void *pvParameters);
void pageFlow(void *pvParameters);
void pageProfile(void *pvParameters);
void pageFeritilizer(void *pvParameters);

// Function handle page for HMI
void USART3_puts(volatile char *s); 

//*********************************SENSOR********************//
void readSensor(void *pvParameters);
void temperature_task(void *pvParameters);
void sonic_task(void *pvParameters);
void display_rtc(rtc_ds1307_datetime_t *rtc_datetime);
void adc_task(void *pvParameters);

//*********************************CONTROL ENVIROMENTAL********************//
void ControlWater(void *pvParameters);
void loadMachineLearning(float* a_sensor, float* a_previousSensor, float* a_sensorAdjustment, float* a_sensorTarget, bool* a_logSensor, unsigned int* a_dosingIncrementArray,
                         uint8_t* a_resultArrayBlock, bool* a_saveResult, float* a_mls, float* a_newDoserMls, float* a_dosingMultipler, float* a_previousMls, bool* a_compressData);
int doubleToInt(float *a_value);
float averageResults(unsigned int *a_array, bool *a_convertToFloat);
float percentOutOfRange(const float a_setPoint, const float a_val);
void runDosePHUP(uint8_t speed, uint8_t a_mlis);
void runDosePHDOWN(uint8_t speed, uint8_t a_mlis);
void runDoseGROUPA(uint8_t speed, uint8_t a_mlis);
void runDoseGROUPB(uint8_t speed, uint8_t a_mlis);
float speedtomils(uint8_t speed);

void WaterPlants(void *pvParameters);
/**
  * @brief  Configures the USART Peripheral.
  * @param  None
  * @retval None
  */
void USART_Config() {
	
	//Enable UART2 with interrupt RXNE for HMI
	USART3_Config(); 
	//Enable UART1 for Comunication with PC
	USART1_Config();
	//Enable UART2 for Ultrasonic
	USART2_Config(); 
}
void I2C_Config() {
	//Enable I2C2 for RTC
	I2C2_Config();
}
void ADC_Config() {
	ADC1_Config();
	ADC_SoftwareStartConv(ADC1);
}

void TIM_Config()
{
	TIM2_Config(); // timer 32bit counter
	TIM5_Config(); // timer 32bit counter
	TIM4_Config(); // timer 16bit for pwm at 10khz.
}

void EXTI_Config()
{
	EXTI_Line1_Config();
	/* Generate software interrupt: simulate a falling edge applied on EXTI0 line */
  EXTI_GenerateSWInterrupt(EXTI_Line1);
	
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
