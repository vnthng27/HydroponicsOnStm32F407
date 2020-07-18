#include "main.h"
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
//extern uint8_t countRxData;
 

	
	
int main(void)
{
	  /* USART configuration -----------------------------------------------------*/
  USART_Config();
	GPIO_Configuration();         //Config LED PIN
	//ds1307_init_rtc(1);
	//I2C_Config();
	
	xTaskCreate(handleUARTRX, "xu ly data rev from hmi", 1000, NULL, 1, NULL);
	//xTaskCreate(pageHOME, "handle pageHOME", 1000, NULL, 2, NULL);
	vTaskStartScheduler();
	
	for(;;){}
}


void USART3_IRQHandler()
{
	static BaseType_t xHigherPriorityTaskWoken ;
	xHigherPriorityTaskWoken = pdFALSE ;
	if (USART_GetITStatus(USART3,USART_IT_RXNE)==SET )
	{
		DataHMI[countRxData] = USART_ReceiveData(USART3);
		xSemaphoreGiveFromISR( xSemaphoreUARTRX, &xHigherPriorityTaskWoken );
		USART_ClearFlag(USART3,USART_FLAG_RXNE);
	}	
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );	
}





