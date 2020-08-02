#include "main.h"

int main(void) {
	  /* USART configuration -----------------------------------------------------*/
  USART_Config();
	GPIO_Configuration();         //Config LED PIN
	I2C_Config();
	ADC_Config();
	TIM_Config();
	EXTI_Config();
	//Create Semaphore 
	xSemaphoreUARTRX = xSemaphoreCreateBinary();
	xSemaphoreChangepage = xSemaphoreCreateBinary();
	xSemaphoreSonicSensor = xSemaphoreCreateBinary();
	xSemaphoredosingInterval = xSemaphoreCreateBinary();
	xSemaphoresensorsReady = xSemaphoreCreateBinary();
	xSemaphoreHandleData = xSemaphoreCreateBinary();
	xSemaphoreEXTIZCD = xSemaphoreCreateBinary();
	xSemaphoreHadData = xSemaphoreCreateBinary();
	GPIO_WriteBit(GPIOD,GPIO_Pin_2,Bit_SET);
	GPIO_WriteBit(GPIOD,GPIO_Pin_3,Bit_SET);
	
	xTaskCreate(handleUARTRX, "xu ly data rev from hmi",  configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(handleHMI, "xu ly HMI",  configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	//Tasks handle page HMI
	xTaskCreate(pageHome, "pageHome",  configMINIMAL_STACK_SIZE, NULL, 1, &TaskpageHome);
	xTaskCreate(pageWarning, "pageWarning",  configMINIMAL_STACK_SIZE, NULL, 1, &TaskpageWarning);
	xTaskCreate(pageWifi, "pageWifi",  configMINIMAL_STACK_SIZE, NULL, 1, &TaskpageWifi);
	xTaskCreate(pagePPM, "pagePPM",  configMINIMAL_STACK_SIZE, NULL, 1, &TaskpagePPM);
	xTaskCreate(pagePPM2, "pagePPM2",  configMINIMAL_STACK_SIZE, NULL, 1, &TaskpagePPM2);
	xTaskCreate(pagepH, "pagepH",  configMINIMAL_STACK_SIZE, NULL, 1, &TaskpagepH);
	xTaskCreate(pageTank, "pageTank",  configMINIMAL_STACK_SIZE, NULL, 1, &TaskpageTank);
	xTaskCreate(pageTank2, "pageTank2",  configMINIMAL_STACK_SIZE, NULL, 1, &TaskpageTank2);
	xTaskCreate(pageValve, "pageValve",  configMINIMAL_STACK_SIZE, NULL, 1, &TaskpageValve);
	xTaskCreate(pageValve2, "pageValve2",  configMINIMAL_STACK_SIZE, NULL, 1, &TaskpageValve2);
	xTaskCreate(pagePeristaltic, "pagePeristaltic",  configMINIMAL_STACK_SIZE, NULL, 1, &TaskpagePeristaltic);
	xTaskCreate(pagePump, "pagePump",  configMINIMAL_STACK_SIZE, NULL, 1, &TaskpagePump);
	xTaskCreate(pagePump2, "pagePump2",  configMINIMAL_STACK_SIZE, NULL, 1, &TaskpagePump2);
	xTaskCreate(pagePump3, "pagePump3",  configMINIMAL_STACK_SIZE, NULL, 1, &TaskpagePump3);
	xTaskCreate(pagePump4, "pagePump4",  configMINIMAL_STACK_SIZE, NULL, 1, &TaskpagePump4);
	xTaskCreate(pagePump5, "pagePump5",  configMINIMAL_STACK_SIZE, NULL, 1, &TaskpagePump5);
	xTaskCreate(pageGraph, "pageGraph",  configMINIMAL_STACK_SIZE, NULL, 1, &TaskpageGraph);
	xTaskCreate(pageSetting, "pageSetting",  configMINIMAL_STACK_SIZE, NULL, 1, &TaskpageSetting);
	xTaskCreate(pageSettingDate, "pageSettingDate",  configMINIMAL_STACK_SIZE, NULL, 1, &TaskpageSettingDate);
	xTaskCreate(pageRefilldate, "pageRefilldate",  configMINIMAL_STACK_SIZE, NULL, 1, &TaskpageRefilldate);
	xTaskCreate(pageCalpH, "pageCalpH",  configMINIMAL_STACK_SIZE, NULL, 1, &TaskpageCalpH);
	xTaskCreate(pageCalPPM, "pageCalPPM",  configMINIMAL_STACK_SIZE, NULL, 1, &TaskpageCalPPM);
	xTaskCreate(pageCalTemp, "pageCalTemp",  configMINIMAL_STACK_SIZE, NULL, 1, &TaskpageCalTemp);
	xTaskCreate(pageCalSonic, "pageCalSonic",  configMINIMAL_STACK_SIZE, NULL, 1, &TaskpageCalSonic);
	xTaskCreate(pageFlow, "pageFlow",  configMINIMAL_STACK_SIZE, NULL, 1, &TaskpageFlow);
	xTaskCreate(pageProfile, "pageProfile",  configMINIMAL_STACK_SIZE, NULL, 1, &TaskpageProfile);
	xTaskCreate(pageFeritilizer, "pageFeritilizer",  configMINIMAL_STACK_SIZE, NULL, 1, &TaskpageFeritilizer);
	//xTaskCreate(readSensor, "readSensor", 200, NULL, 1, NULL);
	xTaskCreate(temperature_task, "temperature_task", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(sonic_task, "sonic_task", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(adc_task, "adc_task", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	//xTaskCreate(ControlWater, "ControlEnviromental", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
//	xTaskCreate(WaterPlants, "WaterPlants", 200, NULL, 1, NULL);
	vTaskStartScheduler();

	
	for(;;){}
}


void USART3_IRQHandler() {
	static BaseType_t xHigherPriorityTaskWoken ;
	xHigherPriorityTaskWoken = pdFALSE ;
	if (USART_GetITStatus(USART3,USART_IT_RXNE)==SET && FlagTC == FALSE)
	{
		DataHMIREV[countRxData] = USART_ReceiveData(USART3);
		if (DataHMIREV[countRxData] == 0xFF && DataHMIREV[countRxData-1] == 0xFF && DataHMIREV[countRxData-2] == 0xFF) {
			FlagTC = TRUE;
			countRxData = 0;
			xSemaphoreGiveFromISR( xSemaphoreUARTRX, &xHigherPriorityTaskWoken );
		}	
		else {
			countRxData += 1;
		}
		USART_ClearFlag(USART3,USART_FLAG_RXNE);
	}	
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );	
}
void USART2_IRQHandler() {
	static BaseType_t xHigherPriorityTaskWoken ;
	xHigherPriorityTaskWoken = pdFALSE ;
	if (USART_GetITStatus(USART2,USART_IT_RXNE)==SET && FlagTC == FALSE)
	{
		xSemaphoreGiveFromISR( xSemaphoreSonicSensor, &xHigherPriorityTaskWoken );
		USART_ClearFlag(USART2,USART_FLAG_RXNE);
	}
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );	
}
void USART3_puts(volatile char *s) {
    while(*s){
		  // wait until data register is empty
		  while( !(USART3->SR & 0x00000040) ) ;
		      USART_SendData(USART3, *s);
		  *s++;
    }
}
/**
  * @brief  This function handles External line 0 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI1_IRQHandler(void) {
	static BaseType_t xHigherPriorityTaskWoken ;
	xHigherPriorityTaskWoken = pdFALSE ;
  if(EXTI_GetITStatus(EXTI_Line1) != RESET)
  {
		if (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1) == Bit_RESET) {
			//xSemaphoreGiveFromISR( xSemaphoreEXTIZCD, &xHigherPriorityTaskWoken );
			//GPIO_WriteBit(GPIOD,GPIO_Pin_4,Bit_SET);
			//GPIO_WriteBit(GPIOD,GPIO_Pin_5,Bit_SET);
		}
		/* Clear the EXTI line 1 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line1);
  }
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );	
}

void pageHome(void *pvParameters) {
	uint8_t prevHour = 0;
	uint8_t prevMinute = 0;
	char bufferSend[] = "";
	prevHour = rtc_datetime.hour;
	prevMinute = rtc_datetime.minutes;
	uint8_t prevID = 10;
	uint16_t preNumValSaved = 0;
	vTaskSuspend(NULL);
	while (1) {
		prevpage = HOME;
		if (reloadPage == true) {
			prevID = 10;
			reloadPage = false;
		}
		if (IDHMI == 0x00 && DataHMI[0] == 0) {
			if (prevID != IDHMI ) {
				USART3_puts("cle 19,0");
				endData;
				if (NumValSaved - 1 > 0 ) {
					preNumValSaved = NumValSaved;
					sprintf(bufferSend,"addt 19,0,%d",(NumValSaved-1 > 0) ? NumValSaved-1:0);
					USART3_puts(&bufferSend[0]);
					for (int j = 0; j<NumValSaved;j++) {
						USART3_puts(&sensorPHinDay[j]);
					}
				}
				
				prevID = IDHMI;
				
				USART3_puts("t0.txt=\"PPM\"");
				endData;
				
				USART3_puts("b0.bco=1024");
				endData;
				USART3_puts("b1.bco=50712");
				endData;
				USART3_puts("b3.bco=50712");
				endData;
				USART3_puts("b2.bco=50712");
				endData;
				
				sprintf(bufferSend,"t4.txt=\"%d\"",tdsOffset);
				USART3_puts(&bufferSend[0]);
				endData;
				
				sprintf(bufferSend,"t7.txt=\"%d\"",targetMinTds);
				USART3_puts(&bufferSend[0]);
				endData;
				
				sprintf(bufferSend,"t8.txt=\"%d\"",targetMaxTds);
				USART3_puts(&bufferSend[0]);
				endData;
				
			}
			if (NumValSaved - preNumValSaved >0) {
				preNumValSaved = NumValSaved;
				sprintf(bufferSend,"addt 19,0,%d",NumValSaved - preNumValSaved);
				for (int j = preNumValSaved;j<NumValSaved;j++) {
					USART3_puts(&sensorPHinDay[j]);
				}
			}
			sprintf(bufferSend,"t3.txt=\"%0.1f\"",data_sensor[0]);
			USART3_puts(&bufferSend[0]);
			endData;
			xSemaphoreGive(xSemaphoreHandleData);
			vTaskDelay(250/portTICK_PERIOD_MS);
		}
		else if (IDHMI == 0x02 && DataHMI[0] == 2) {
			if (prevID != IDHMI) {
				USART3_puts("cle 19,0");
				endData;
				if (NumValSaved - 1 > 0 ) {
					preNumValSaved = NumValSaved;
					sprintf(bufferSend,"addt 19,0,%d",(NumValSaved-1 > 0) ? NumValSaved-1:0);
					USART3_puts(&bufferSend[0]);
					for (int j = 0; j<NumValSaved;j++) {
						USART3_puts(&sensorTEMPinDay[j]);
					}
				}
				
				prevID = IDHMI;
				USART3_puts("t0.txt=\"TEMPERATURE\"");
				endData;
				USART3_puts("b0.bco=50712");
				endData;
				USART3_puts("b1.bco=1024");
				endData;
				USART3_puts("b3.bco=50712");
				endData;
				USART3_puts("b2.bco=50712");
				endData;
				
				USART3_puts("t4.txt=\"\"");
				endData;
				USART3_puts("t7.txt=\"\"");
				endData;
				USART3_puts("t8.txt=\"\"");
				endData;
				
			}

			if (NumValSaved - preNumValSaved >0) {
				sprintf(bufferSend,"addt 19,0,%d",NumValSaved - preNumValSaved);
				for (int j = preNumValSaved;j<NumValSaved;j++) {
					USART3_puts(&sensorTEMPinDay[j]);
				}
			}
			sprintf(bufferSend,"t3.txt=\"%0.1f\"",data_sensor[3]);
			USART3_puts(&bufferSend[0]);
			endData;
			xSemaphoreGive(xSemaphoreHandleData);
			vTaskDelay(250/portTICK_PERIOD_MS);
		}
		else if (IDHMI == 0x03 && DataHMI[0] == 3) {
			if (prevID != IDHMI) {
				USART3_puts("cle 19,0");
				endData;
				if (NumValSaved - 1 > 0 ) {
					preNumValSaved = NumValSaved;
					sprintf(bufferSend,"addt 19,0,%d",(NumValSaved-1 > 0) ? NumValSaved-1:0);
					USART3_puts(&bufferSend[0]);
					for (int j = 0; j<NumValSaved;j++) {
						USART3_puts(&sensorTANKinDay[j]);
					}
				}
				prevID = IDHMI;
				USART3_puts("t0.txt=\"TANK\"");
				endData;
				USART3_puts("b0.bco=50712");
				endData;
				USART3_puts("b1.bco=50712");
				endData;
				USART3_puts("b3.bco=1024");
				endData;
				USART3_puts("b2.bco=50712");
				endData;
				
				sprintf(bufferSend,"t4.txt=\"%d\"",TankLvOffset);
				USART3_puts(&bufferSend[0]);
				endData;
				sprintf(bufferSend,"t7.txt=\"%d\"",targetMinTankLv);
				USART3_puts(&bufferSend[0]);
				endData;
				sprintf(bufferSend,"t8.txt=\"%d\"",targetMaxTankLv);
				USART3_puts(&bufferSend[0]);
				endData;
				
			}
			
			if (NumValSaved - preNumValSaved >0) {
				sprintf(bufferSend,"addt 19,0,%d",NumValSaved - preNumValSaved);
				for (int j = preNumValSaved;j<NumValSaved;j++) {
					USART3_puts(&sensorTANKinDay[j]);
				}
			}
//			sprintf(bufferSend,"t3.txt=\"%0.1f\"",data_sensor[2]);
//			USART3_puts(&bufferSend[0]);
//			endData;
			xSemaphoreGive(xSemaphoreHandleData);
			vTaskDelay(250/portTICK_PERIOD_MS);
		}
		else if (IDHMI == 0x04 && DataHMI[0] == 4) {
			if (prevID != IDHMI) {
				USART3_puts("cle 19,0");
				endData;
				if (NumValSaved - 1 > 0 ) {
					preNumValSaved = NumValSaved;
					sprintf(bufferSend,"addt 19,0,%d",(NumValSaved-1 > 0) ? NumValSaved-1:0);
					USART3_puts(&bufferSend[0]);
					for (int j = 0; j<NumValSaved;j++) {
						USART3_puts(&sensorPHinDay[j]);
					}
				}
				prevID = IDHMI;
				USART3_puts("t0.txt=\"PH\"");
				endData;
				USART3_puts("b0.bco=50712");
				endData;
				USART3_puts("b1.bco=50712");
				endData;
				USART3_puts("b3.bco=50712");
				endData;
				USART3_puts("b2.bco=1024");
				endData;
				
				sprintf(bufferSend,"t4.txt=\"%0.1f\"",phOffset);
				USART3_puts(&bufferSend[0]);
				endData;
				sprintf(bufferSend,"t7.txt=\"%0.1f\"",targetMinPh);
				USART3_puts(&bufferSend[0]);
				endData;
				sprintf(bufferSend,"t8.txt=\"%0.1f\"",targetMaxPh);
				USART3_puts(&bufferSend[0]);
				endData;
			}

			if (NumValSaved - preNumValSaved >0) {
				sprintf(bufferSend,"addt 19,0,%d",NumValSaved - preNumValSaved);
				for (int j = preNumValSaved;j<NumValSaved;j++) {
					USART3_puts(&sensorPHinDay[j]);
				}
			}
			sprintf(bufferSend,"t3.txt=\"%0.1f\"",data_sensor[1]);
			USART3_puts(&bufferSend[0]);
			endData;
			xSemaphoreGive(xSemaphoreHandleData);
			vTaskDelay(250/portTICK_PERIOD_MS);
		}
		else
			xSemaphoreGive(xSemaphoreHandleData);
		if (rtc_datetime.hour != prevHour) {
			sprintf(bufferSend,"hour.val=%d",rtc_datetime.hour);
			USART3_puts(&bufferSend[0]);
			endData;
			prevHour = rtc_datetime.hour;
			vTaskDelay(250/portTICK_PERIOD_MS);
		}
		if (rtc_datetime.minutes != prevMinute) {
			sprintf(bufferSend,"minute.txt=\"%d\"",rtc_datetime.minutes);
			USART3_puts(&bufferSend[0]);
			endData;
			prevMinute = rtc_datetime.minutes;
			vTaskDelay(250/portTICK_PERIOD_MS);
		}
	}
}
void pageWarning(void *pvParameters) {
	uint8_t prevID = 10;
	char bufferSend[] = "";
	char threeByteEnd[3] = {0xFF,0xFF,0xFF};
	vTaskSuspend(NULL);
	while (1) {
		prevpage = WARINING;
		if (reloadPage == true) {
			prevID = 10;
			reloadPage = false;
		}
		if (IDHMI == 0 && DataHMI[0] == 0) {
			if (prevID != IDHMI) {
				USART3_puts("t2.txt=\"PPM WARNING\"");
				endData;
				USART3_puts("b0.bco=1024");
				endData;
				USART3_puts("b1.bco=50712");
				endData;
				USART3_puts("b2.bco=50712");
				endData;
				USART3_puts("b3.bco=50712");
				endData;
			}
			prevID = IDHMI;
			
		}
		else if (IDHMI == 1 && DataHMI[0] == 1) {
			if (prevID != IDHMI) {
				USART3_puts("t2.txt=\"TEMPERATURE WARNING\"");
				endData;
				USART3_puts("b0.bco=50712");
				endData;
				USART3_puts("b1.bco=1024");
				endData;
				USART3_puts("b2.bco=50712");
				endData;
				USART3_puts("b3.bco=50712");
				endData;
			}
			prevID = IDHMI;
			
		}
		else if (IDHMI == 2 && DataHMI[0] == 2) {
			if (prevID != IDHMI) {
				USART3_puts("t2.txt=\"pH WARNING\"");
				endData;
				USART3_puts("b0.bco=50712");
				endData;
				USART3_puts("b1.bco=50712");
				endData;
				USART3_puts("b2.bco=1024");
				endData;
				USART3_puts("b3.bco=50712");
				endData;
			}
			prevID = IDHMI;
		}
		else if (IDHMI == 3 && DataHMI[0] == 3) {
			if (prevID != IDHMI) {
				USART3_puts("t2.txt=\"TANK WARNING\"");
				endData;
				USART3_puts("b0.bco=50712");
				endData;
				USART3_puts("b1.bco=50712");
				endData;
				USART3_puts("b2.bco=50712");
				endData;
				USART3_puts("b3.bco=1024");
				endData;
			}
			prevID = IDHMI;
		}
		if (prevID == 0) {
			sprintf(bufferSend,"t3.txt=\"%0.1f\"",data_sensor[0]);
			USART3_puts(&bufferSend[0]);
			endData; 	
			sprintf(bufferSend,"t1.txt=\"%f\"",errorMarginPPM);
			USART3_puts(&bufferSend[0]);
			endData;
			xSemaphoreGive(xSemaphoreHandleData);
		}
		if (prevID == 1) {
			sprintf(bufferSend,"t3.txt=\"%0.1f\"",data_sensor[3]);
			USART3_puts(&bufferSend[0]);
			endData;
			sprintf(bufferSend,"t1.txt=\"%0.1f\"",errorMarginTEMP);
			USART3_puts(&bufferSend[0]);
			endData;
			xSemaphoreGive(xSemaphoreHandleData);
		}
		if (prevID == 2) {
			sprintf(bufferSend,"t3.txt=\"%0.1f\"",data_sensor[1]);
			USART3_puts(&bufferSend[0]);
			endData;
			sprintf(bufferSend,"x0.val=%d",(uint8_t)(errorMarginPH*10));
			USART3_puts(&bufferSend[0]);
			endData;
			xSemaphoreGive(xSemaphoreHandleData);
		}
		if (prevID == 3) {
			sprintf(bufferSend,"t3.txt=\"%0.1f\"",data_sensor[2]);
			USART3_puts(&bufferSend[0]);
			endData;
			sprintf(bufferSend,"t1.txt=\"%d\"",(uint16_t)errorMarginTANK);
			USART3_puts(&bufferSend[0]);
			endData;
			xSemaphoreGive(xSemaphoreHandleData);	
		}
		if (IDHMI == 5 ) {
			errorMarginPPM = (float)((DataHMI[1]<<8) | DataHMI[0]);
		}
		if (IDHMI == 6 ) {
			errorMarginTEMP = (float)((DataHMI[1]<<8) | DataHMI[0]);
		}
		if (IDHMI == 7 ) {
			errorMarginPH = (float)(DataHMI[0]/10.0);
		}
		if (IDHMI == 8 ) {
			errorMarginTANK = (float)((DataHMI[1]<<8) | DataHMI[0]);
		}
		xSemaphoreGive(xSemaphoreHandleData);
		vTaskDelay(150/portTICK_PERIOD_MS);
	}
}
void pageWifi(void *pvParameters) {
	vTaskSuspend(NULL);
	while (1) {
		if (IDHMI == 0) {
			WifiInterval = (uint16_t)((DataHMI[1]<<8) | DataHMI[0]);
		}
		xSemaphoreGive(xSemaphoreHandleData);
		reloadPage = false;
		vTaskDelay(150/portTICK_PERIOD_MS);
	}
}
void pagePPM(void *pvParameters) {
	char bufferSend[] = "";
	vTaskSuspend(NULL);
	while (1) {
//		if (reloadPage == true) {
//			sprintf(bufferSend,"min.val=%d",targetMinTdsALL);
//			USART3_puts(&bufferSend[0]);
//			endData;
//			sprintf(bufferSend,"max.val=%d",targetMaxTdsALL);
//			USART3_puts(&bufferSend[0]);
//			endData;
//			sprintf(bufferSend,"tol.val=%d",tdsOffsetALL);
//			USART3_puts(&bufferSend[0]);
//			endData;
//			sprintf(bufferSend,"t4.txt=\"%d\"",targetMinTdsALL);
//			USART3_puts(&bufferSend[0]);
//			endData;
//			sprintf(bufferSend,"t5.txt=\"%d\"",targetMaxTdsALL);
//			USART3_puts(&bufferSend[0]);
//			endData;
//			sprintf(bufferSend,"t6.txt=\"%d\"",tdsOffsetALL);
//			USART3_puts(&bufferSend[0]);
//			endData;
//			reloadPage = false;
//		}
		if (xSemaphoreTake( xSemaphoreHadData,portMAX_DELAY) == pdTRUE) {
			if (IDHMI == 1) {
				targetMinTdsALL = (unsigned int)((DataHMI[1]<<8) | DataHMI[0]);
		
				targetMaxTdsALL = (unsigned int)((DataHMI[3]<<8) | DataHMI[2]);
				
				tdsOffsetALL = (unsigned int)((DataHMI[5]<<8) | DataHMI[4]);
			}
			xSemaphoreGive(xSemaphoreHandleData);
		}	
	}
}
void pagePPM2(void *pvParameters) {
	vTaskSuspend(NULL);
	while (1) {
		vTaskDelete(NULL);
	}
}
void pagepH(void *pvParameters) {
	vTaskSuspend(NULL);
	while (1) {
		vTaskDelete(NULL);
	}
}
void pageTank(void *pvParameters) {
	vTaskSuspend(NULL);
	while (1) {
		vTaskDelete(NULL);
	}
}
void pageTank2(void *pvParameters) {
	vTaskSuspend(NULL);
	while (1) {
		vTaskDelete(NULL);
	}
}
void pageValve(void *pvParameters) {
	vTaskSuspend(NULL);
	while (1) {
		vTaskDelete(NULL);
	}
}
void pageValve2(void *pvParameters) {
	vTaskSuspend(NULL);
	while (1) {
		vTaskDelete(NULL);
	}
}
void pagePeristaltic(void *pvParameters) {
	vTaskSuspend(NULL);
	while (1) {
		vTaskDelete(NULL);
	}
}
void pagePump(void *pvParameters) {
	vTaskSuspend(NULL);
	while (1) {
		vTaskDelete(NULL);
	}
}
void pagePump2(void *pvParameters) {
	vTaskSuspend(NULL);
	while (1) {
		vTaskDelete(NULL);
	}
}
void pagePump3(void *pvParameters) {
	vTaskSuspend(NULL);
	while (1) {
		vTaskDelete(NULL);
	}
}
void pagePump4(void *pvParameters) {
	vTaskSuspend(NULL);
	while (1) {
		vTaskDelete(NULL);
	}
}
void pagePump5(void *pvParameters) {
	vTaskSuspend(NULL);
	while (1) {
		vTaskDelete(NULL);
	}
}
void pageGraph(void *pvParameters) {
	vTaskSuspend(NULL);
	while (1) {
		vTaskDelete(NULL);
	}
}
void pageSetting(void *pvParameters) {
	vTaskSuspend(NULL);
	while (1) {
		vTaskDelete(NULL);
	}
}
void pageSettingDate(void *pvParameters) {
	vTaskSuspend(NULL);
	while (1) {
		vTaskDelete(NULL);
	}
}
void pageRefilldate(void *pvParameters) {
	vTaskSuspend(NULL);
	while (1) {
		vTaskDelete(NULL);
	}
}
void pageCalpH(void *pvParameters) {
	vTaskSuspend(NULL);
	while (1) {
		vTaskDelete(NULL);
	}
}
void pageCalPPM(void *pvParameters) {
	vTaskSuspend(NULL);
	while (1) {
		vTaskDelete(NULL);
	}
}
void pageCalTemp(void *pvParameters) {
	vTaskSuspend(NULL);
	while (1) {
		vTaskDelete(NULL);
	}
}
void pageCalSonic(void *pvParameters) {
	vTaskSuspend(NULL);
	while (1) {
		vTaskDelete(NULL);
	}
}
void pageFlow(void *pvParameters) {
	vTaskSuspend(NULL);
	while (1) {
		vTaskDelete(NULL);
	}
}
void pageProfile(void *pvParameters) {
	vTaskSuspend(NULL);
	while (1) {
		vTaskDelete(NULL);
	}
}
void pageFeritilizer(void *pvParameters) {
	vTaskSuspend(NULL);
	while (1) {
	}
}
void handleHMI(void *pvParameters) {
	while (1)
	{ 
			if (xSemaphoreTake(xSemaphoreChangepage,portMAX_DELAY) == pdTRUE)
			{
				xSemaphoreGive(xSemaphoreHandleData);
				switch (prevpage) {
					case HOME:
					{
						vTaskSuspend(TaskpageHome);
						break;
					}
					case WARINING:
					{
						vTaskSuspend(TaskpageWarning);
						break;
					}
					case WIFI:
					{
						vTaskSuspend(TaskpageWifi);
						break;
					}
					case PPM:
					{
						vTaskSuspend(TaskpagePPM);
						break;
					}
					case PPM2:
					{
						vTaskSuspend(TaskpagePPM2);
						break;
					}
					case PH:
					{
						vTaskSuspend(TaskpagepH);
						break;
					}
					case TANK:
					{
						vTaskSuspend(TaskpageTank);
						break;
					}
					case TANK2:
					{
						vTaskSuspend(TaskpageTank2);
						break;
					}
					case VALVE:
					{
						vTaskSuspend(TaskpageValve);
						break;
					}
					case VALVE2:
					{
						vTaskSuspend(TaskpageValve2);
						break;
					}
					case PERISTALTIC_PUMP:
					{
						vTaskSuspend(TaskpagePeristaltic);
						break;
					}
					case PUMP:
					{
						vTaskSuspend(TaskpagePump);
						break;
					}
					case PUMP2:
					{
						vTaskSuspend(TaskpagePump2);
						break;
					}
					case PUMP3:
					{
						vTaskSuspend(TaskpagePump3);
						break;
					}
					case PUMP4:
					{
						vTaskSuspend(TaskpagePump4);
						break;
					}
					case PUMP5:
					{
						vTaskSuspend(TaskpagePump5);
						break;
					}
					case GRAPH:
					{
						vTaskSuspend(TaskpageGraph);
						break;
					}
					case SETTING:
					{
						vTaskSuspend(TaskpageSetting);
						break;
					}
					case SETTING_DATE:
					{
						vTaskSuspend(TaskpageSettingDate);
						break;
					}
					case REFILL_DATE:
					{
						vTaskSuspend(TaskpageRefilldate);
						break;
					}
					case CAL_PH:
					{
						vTaskSuspend(TaskpageCalpH);
						break;
					}
					case CAL_PPM:
					{
						vTaskSuspend(TaskpageCalPPM);
						break;
					}
					case CAL_TEMP:
					{
						vTaskSuspend(TaskpageCalTemp);
						break;
					}
					case CAL_SONIC:
					{
						vTaskSuspend(TaskpageCalSonic);
						break;
					}
					case CAL_FLOW:
					{
						vTaskSuspend(TaskpageFlow);
						break;
					}
					case PROFILE:
					{
						vTaskSuspend(TaskpageProfile);
						break;
					}
					case FERITILIZER:
					{
						vTaskSuspend(TaskpageFeritilizer);
						break;
					}
					default:{}
				}
				switch (page) {
					case HOME:
					{
						reloadPage = true;
						vTaskResume(TaskpageHome);
						break;
					}
					case WARINING:
					{
						reloadPage = true;
						vTaskResume(TaskpageWarning);
						break;
					}
					case WIFI:
					{
						reloadPage = true;
						vTaskResume(TaskpageWifi);
						break;
					}
					case PPM:
					{
						reloadPage = true;
						vTaskResume(TaskpagePPM);
						break;
					}
					case PPM2:
					{
						reloadPage = true;
						vTaskResume(TaskpagePPM2);
						break;
					}
					case PH:
					{
						reloadPage = true;
						vTaskResume(TaskpagepH);
						break;
					}
					case TANK:
					{
						reloadPage = true;
						vTaskResume(TaskpageTank);
						break;
					}
					case TANK2:
					{
						reloadPage = true;
						vTaskResume(TaskpageTank2);
						break;
					}
					case VALVE:
					{
						reloadPage = true;
						vTaskResume(TaskpageValve);
						break;
					}
					case VALVE2:
					{
						reloadPage = true;
						vTaskResume(TaskpageValve2);
						break;
					}
					case PERISTALTIC_PUMP:
					{
						reloadPage = true;
						vTaskResume(TaskpagePeristaltic);
						break;
					}
					case PUMP:
					{
						reloadPage = true;
						vTaskResume(TaskpagePump);
						break;
					}
					case PUMP2:
					{
						reloadPage = true;
						vTaskResume(TaskpagePump2);
						break;
					}
					case PUMP3:
					{
						reloadPage = true;
						vTaskResume(TaskpagePump3);
						break;
					}
					case PUMP4:
					{
						reloadPage = true;
						vTaskResume(TaskpagePump4);
						break;
					}
					case PUMP5:
					{
						reloadPage = true;
						vTaskResume(TaskpagePump5);
						break;
					}
					case GRAPH:
					{
						reloadPage = true;
						vTaskResume(TaskpageGraph);
						break;
					}
					case SETTING:
					{
						reloadPage = true;
						vTaskResume(TaskpageSetting);
						break;
					}
					case SETTING_DATE:
					{
						reloadPage = true;
						vTaskResume(TaskpageSettingDate);
						break;
					}
					case REFILL_DATE:
					{
						reloadPage = true;
						vTaskResume(TaskpageRefilldate);
						break;
					}
					case CAL_PH:
					{
						reloadPage = true;
						vTaskResume(TaskpageCalpH);
						break;
					}
					case CAL_PPM:
					{
						reloadPage = true;
						vTaskResume(TaskpageCalPPM);
						break;
					}
					case CAL_TEMP:
					{
						reloadPage = true;
						vTaskResume(TaskpageCalTemp);
						break;
					}
					case CAL_SONIC:
					{
						reloadPage = true;
						vTaskResume(TaskpageCalSonic);
						break;
					}
					case CAL_FLOW:
					{
						reloadPage = true;
						vTaskResume(TaskpageFlow);
						break;
					}
					case PROFILE:
					{
						reloadPage = true;
						vTaskResume(TaskpageProfile);
						break;
					}
					case FERITILIZER:
					{
						reloadPage = true;
						vTaskResume(TaskpageFeritilizer);
						break;
					}
					default:{}
				}
			}
	}
}

void handleUARTRX(void *pvParameters) {
	memset(DataHMIREV,'\0',LENGTH_DATA_HMI);
	while(1)
	{	
			if (xSemaphoreTake( xSemaphoreUARTRX,portMAX_DELAY) == pdTRUE ){
				if (FlagTC == TRUE){
					// Xu ly nhan byte dau tien
						switch (DataHMIREV[0]) {
							case BYTE_FIRST_START_PAGE:
							{
								if (mystrlength(DataHMIREV) == 5) {
									typedata = START_PAGE;
								}
								else {
									typedata = UNKNOWN;
									memset(DataHMIREV,'\0',LENGTH_DATA_HMI);
									FlagTC = FALSE;
									break;
								}
								break;
							}
							case BYTE_FIRST_SEND_DATA:
							{
								if (mystrlength(DataHMIREV) >= 5) {
									typedata = SEND_DATA;
								}
								else {
									typedata = UNKNOWN;
									memset(DataHMIREV,'\0',LENGTH_DATA_HMI);
									FlagTC = FALSE;
									break;
								}
								break;
							}
							default:
							{
									typedata = UNKNOWN;
									memset(DataHMIREV,'\0',LENGTH_DATA_HMI);
									FlagTC = FALSE;
									break;
							}
					}
						if (typedata == START_PAGE) {
							switch (DataHMIREV[1]) {
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
								}
							}
							FlagTC = FALSE;
							xSemaphoreGive(xSemaphoreChangepage); // Give Semaphore to task handleHMI
							memset(DataHMIREV,'\0',LENGTH_DATA_HMI);
							memset(DataHMI,'\0',LENGTH_DATA_HMI);
							IDHMI = 0;
							
						}
						if (typedata == SEND_DATA) {
							if (xSemaphoreTake( xSemaphoreHandleData,10000) == pdTRUE) {
								if (DataHMIREV[1] == page ) {
									IDHMI = DataHMIREV[2];
									memset(DataHMI,'\0',LENGTH_DATA_HMI);
									for (int i = 0;;i++)
									{
										if (DataHMIREV[i+3] == 0xFF && DataHMIREV[i+4] == 0xFF && DataHMIREV[i+5] == 0xFF)
											break;
										else 
											DataHMI[i] = DataHMIREV[i+3];
									}
									xSemaphoreGive(xSemaphoreHadData);
									FlagTC = FALSE;
									memset(DataHMIREV,'\0',LENGTH_DATA_HMI);
								}
								else {
									FlagTC = FALSE;
									memset(DataHMIREV,'\0',LENGTH_DATA_HMI);
								}	
							}
						}	
				}
		}
	}
}
//Debug RTC by UART3
void display_rtc(rtc_ds1307_datetime_t *rtc_datetime) {
    char buffer[200];

    sprintf( (char *) &buffer[0], "rtc_datetime\r\n\t->seconds = %d\r\n" \
            "\t->minutes = %d\r\n\t->hour = %d\r\n\t->day = %d\r\n" \
            "\t->date = %d\r\n\t->month = %d\r\n\t->year = %d\r\n\r\n", 
            rtc_datetime->seconds, rtc_datetime->minutes, rtc_datetime->hour, 
            rtc_datetime->day, rtc_datetime->date, rtc_datetime->month, 
            rtc_datetime->year);


    USART3_puts((char *) &buffer[0]);

    return;
}
void handleRTC(void *pvParameters) {
	rtc_ds1307_datetime_t rtc_datetime;
	while (1) {
		ds1307_get_rtc_datetime(&rtc_datetime);
		display_rtc(&rtc_datetime);
		vTaskDelay(1000);
	}
}
void readSensor(void *pvParameters) {
	data_sensor[3]=0.0;
	
	uint8_t data_sonic[4];
	uint8_t data_sonic_trigger = 0x55;
	uint16_t distance;
	
	float volt_cur=0.0;
	float val_con = 0.0; 
	float volt_cur_ph =0.0;
	
	uint16_t countSaveinDay = 0;
	
	unsigned long int sumSensorPHinDay = 0;
	unsigned long int sumSensorPPMinDay = 0;
	unsigned long int sumSensorTANKinDay = 0;
	unsigned long int sumSensorTEMPinDay = 0;
	GPIO_WriteBit(GPIOD,GPIO_Pin_2,Bit_SET);
	GPIO_WriteBit(GPIOD,GPIO_Pin_3,Bit_SET);
	while (1){
		ds18b20_init_seq();
		ds18b20_send_rom_cmd(SKIP_ROM_CMD_BYTE);
		ds18b20_send_function_cmd(CONVERT_T_CMD);

		mydelayus(100);

		ds18b20_init_seq();
		ds18b20_send_rom_cmd(SKIP_ROM_CMD_BYTE);
		ds18b20_send_function_cmd(READ_SCRATCHPAD_CMD);
		data_sensor[3] = ds18b20_read_temp();
		
		//vTaskDelay(100/portTICK_PERIOD_MS);
		USART_SendData(USART2,(uint16_t )data_sonic_trigger);
		for (int i = 0;i<4;i++) {
			if (xSemaphoreTake( xSemaphoreSonicSensor,portMAX_DELAY) == pdTRUE )
				data_sonic[i] = (uint8_t)USART_ReceiveData(USART2);
		}
		distance = (distance &0)| data_sonic[1];
		distance = (distance <<8)| data_sonic[2];
		data_sensor[2]=distance;
		
		//vTaskDelay(100/portTICK_PERIOD_MS);
		volt_cur = ((float)uhADCxConvertedValue[1]*3)/4095;
		val_con = volt_cur/(1+(0.02*(data_sensor[3]-25)));
		data_sensor[0] = ((133.42*val_con*val_con*val_con)-(255.86*val_con*val_con)+(857.39*val_con))*0.5;
		
		volt_cur_ph =(((float)uhADCxConvertedValue[0]*3)/4095);
		data_sensor[1]=volt_cur_ph*3.5 + 1.2;
		
		sumSensorTEMPinDay += (unsigned long int)(data_sensor[3]*255/100);
		sumSensorTANKinDay += (unsigned long int)(data_sensor[2]*255/tankDepth);
		sumSensorPPMinDay	+= (unsigned long int)(data_sensor[0]*255/10000);
		sumSensorPHinDay += (unsigned long int)(data_sensor[1]*255/14);
		countSaveinDay++;
		
		if (countSaveinDay >= 288) {
			sensorPHinDay[NumValSaved] = (uint8_t)(sumSensorPHinDay/countSaveinDay);
			sensorPPMinDay[NumValSaved] = (uint8_t)(sumSensorPPMinDay/countSaveinDay);
			sensorTANKinDay[NumValSaved] = (uint8_t)(sumSensorTANKinDay/countSaveinDay);
			sensorTEMPinDay[NumValSaved] = (uint8_t)(sumSensorTEMPinDay/countSaveinDay);
			NumValSaved++;
			for (int j = 0;j<1000;j++) {
				sensorPHinDay[j] = 0;
				sensorPPMinDay[j] = 0;
				sensorTANKinDay[j] = 0;
				sensorTEMPinDay[j] = 0;
			}
			countSaveinDay = 0;
		}
		xSemaphoreGive(xSemaphoresensorsReady);
		vTaskDelay(500/portTICK_PERIOD_MS);

		
	}
}
void temperature_task(void *pvParameters) {
	data_sensor[3] = 0.0	;
	while(1) {
			ds18b20_init_seq();
			ds18b20_send_rom_cmd(SKIP_ROM_CMD_BYTE);
			ds18b20_send_function_cmd(CONVERT_T_CMD);

			mydelayus(100);

			ds18b20_init_seq();
			ds18b20_send_rom_cmd(SKIP_ROM_CMD_BYTE);
			ds18b20_send_function_cmd(READ_SCRATCHPAD_CMD);
			data_sensor[3] = ds18b20_read_temp();
			vTaskDelay(500/portTICK_PERIOD_MS);
		}
}
void sonic_task(void *pvParameters) {
	uint8_t data_sonic[4];
	uint8_t data_sonic_trigger = 0x55;
	uint16_t distance;
	while (1) {
		USART_SendData(USART2,(uint16_t )data_sonic_trigger);
		for (int i = 0;i<4;i++) {
			if (xSemaphoreTake( xSemaphoreSonicSensor,portMAX_DELAY) == pdTRUE )
				data_sonic[i] = (uint8_t)USART_ReceiveData(USART2);
		}
		distance = (distance &0)| data_sonic[1];
		distance = (distance <<8)| data_sonic[2];
		data_sensor[2]=distance;
		vTaskDelay(500/portTICK_PERIOD_MS);
	}
}
void adc_task(void *pvParameters) {
	float volt_cur=0.0;
	float val_con = 0.0; 
	float volt_cur_ph =0.0;
	while (1) {
		volt_cur = ((float)uhADCxConvertedValue[1]*3)/4095;
		val_con = volt_cur/(1+(0.02*(data_sensor[3]-25)));
		data_sensor[0] = ((133.42*val_con*val_con*val_con)-(255.86*val_con*val_con)+(857.39*val_con))*0.5;
		
		volt_cur_ph =(((float)uhADCxConvertedValue[0]*3)/4095);
		data_sensor[1]=volt_cur_ph*3.5 + 1.2;
		vTaskDelay(500/portTICK_PERIOD_MS);
		}
}
float speedtomils(uint8_t speed){
	return ((54.0/2549.0)*(float)speed) - 0.2615;
}
int doubleToInt(float *a_value) {
  int val = *a_value * 1000;
  return val;
}
float averageResults(unsigned int *a_array, bool *a_convertToFloat) {
  float averagedResult = 0;
  uint8_t count = 0;
  for (uint8_t i = 0; i < maxSize; i++) {
    if (*(a_array+i) != 0) {
      averagedResult += *a_convertToFloat ? *(a_array+i) / 1000.0 : *(a_array+i);
      count++;
    }
  }
  if (count != 0)
    averagedResult /= count;
  return averagedResult;
}
// Return the percent out of range
float percentOutOfRange(const float a_setPoint, const float a_val) {
  float outOfRange = fabs(a_setPoint - a_val);
  float percent = a_setPoint / 100.0;
  return outOfRange / percent;
}
void loadMachineLearning(float *a_sensor, float *a_previousSensor,float *a_sensorAdjustment, 
												float *a_sensorTarget, bool *a_logSensor, unsigned int *a_dosingIncrementArray,
                        uint8_t *a_resultArrayBlock, bool *a_saveResult, float *a_mls, float *a_newDoserMls, 
												float *a_dosingMultipler, float *a_previousMls, bool *a_compressData) {
  if (!*a_saveResult) {
    *a_previousSensor = *a_sensor;
    *a_previousMls = *a_mls;
    *a_logSensor = true;
  }
  else if (*a_sensorAdjustment != 0) {
    // Work out the changes since the last dosing
    float dosingResult = fabs(*a_previousSensor - *a_sensorAdjustment); // a_sensor
    float incrementPerMl = dosingResult / *a_previousMls;
    float currentDosingMargin = fabs(*a_sensor - *a_sensorTarget);

    // Save increment per ml to the given modes array.
    if (*a_resultArrayBlock < maxSize) {
      *(a_dosingIncrementArray + *a_resultArrayBlock) = *a_compressData ? doubleToInt(&incrementPerMl) : incrementPerMl;
      *a_resultArrayBlock = *a_resultArrayBlock + 1;
    }
    else {
      for (uint8_t i = 0; i < maxSize - 1; i++) {
        *(a_dosingIncrementArray+i) = *(a_dosingIncrementArray+i+1);
      }
      *a_dosingIncrementArray = *a_compressData ? doubleToInt(&incrementPerMl) : incrementPerMl;
    }
    // Get the new dosing ml's based on the previous calculations saved history
    *a_newDoserMls = currentDosingMargin / averageResults(&(*a_dosingIncrementArray), &(*a_compressData));
    *a_dosingMultipler = *a_newDoserMls / *a_mls;
    if (*a_dosingMultipler == 0)
      *a_dosingMultipler = 1;
    //
    *a_previousSensor = *a_sensor;
    *a_previousMls = *a_newDoserMls;
    *a_logSensor = true;
  }
  *a_saveResult = true;
}
void runDosePHUP(uint8_t speed, uint8_t a_mlis) {
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	for (int i = 0; i<4; i++) {
	if (modePumpDoser[i] == PH_UP) {
		if( i == 0) {
			TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			TIM_OCInitStructure.TIM_Pulse = (2100/speed)*100;
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

			TIM_OC1Init(TIM4, &TIM_OCInitStructure);

			TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
			
		}
		else if (i == 1) {
			TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			TIM_OCInitStructure.TIM_Pulse = (2100/speed)*100;
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

			TIM_OC2Init(TIM4, &TIM_OCInitStructure);

			TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
		}
		else if (i == 2) {
			TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			TIM_OCInitStructure.TIM_Pulse = (2100/speed)*100;
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

			TIM_OC3Init(TIM4, &TIM_OCInitStructure);

			TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
		}
		else if ( i== 3) {
			TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			TIM_OCInitStructure.TIM_Pulse = (2100/speed)*100;
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

			TIM_OC4Init(TIM4, &TIM_OCInitStructure);

			TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
		}
		else break;
		mydelayms((unsigned int)a_mlis*1000);
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
		TIM_OC1Init(TIM4, &TIM_OCInitStructure);
		TIM_OC2Init(TIM4, &TIM_OCInitStructure);
		TIM_OC3Init(TIM4, &TIM_OCInitStructure);
		TIM_OC4Init(TIM4, &TIM_OCInitStructure);
		GPIO_WriteBit(GPIOD,GPIO_Pin_12,Bit_RESET);
		GPIO_WriteBit(GPIOD,GPIO_Pin_13,Bit_RESET);
		GPIO_WriteBit(GPIOD,GPIO_Pin_14,Bit_RESET);
		GPIO_WriteBit(GPIOD,GPIO_Pin_15,Bit_RESET);
		}		
	}
}
void runDosePHDOWN(uint8_t speed, uint8_t a_mlis) {
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	for (int i = 0; i<4; i++) {
	if (modePumpDoser[i] == PH_DOWN) {
		if( i == 0) {
			TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			TIM_OCInitStructure.TIM_Pulse = (2100/speed)*100;
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

			TIM_OC1Init(TIM3, &TIM_OCInitStructure);

			TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
			
		}
		else if (i == 1) {
			TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			TIM_OCInitStructure.TIM_Pulse = (2100/speed)*100;
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

			TIM_OC2Init(TIM3, &TIM_OCInitStructure);

			TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
		}
		else if (i == 2) {
			TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			TIM_OCInitStructure.TIM_Pulse = (2100/speed)*100;
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

			TIM_OC3Init(TIM3, &TIM_OCInitStructure);

			TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
		}
		else if ( i== 3) {
			TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			TIM_OCInitStructure.TIM_Pulse = (2100/speed)*100;
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

			TIM_OC4Init(TIM3, &TIM_OCInitStructure);

			TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
		}
		else break;
		mydelayms((unsigned int)a_mlis*1000);
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
		TIM_OC1Init(TIM3, &TIM_OCInitStructure);
		TIM_OC2Init(TIM3, &TIM_OCInitStructure);
		TIM_OC3Init(TIM3, &TIM_OCInitStructure);
		TIM_OC4Init(TIM3, &TIM_OCInitStructure);
		GPIO_WriteBit(GPIOD,GPIO_Pin_12,Bit_RESET);
		GPIO_WriteBit(GPIOD,GPIO_Pin_13,Bit_RESET);
		GPIO_WriteBit(GPIOD,GPIO_Pin_14,Bit_RESET);
		GPIO_WriteBit(GPIOD,GPIO_Pin_15,Bit_RESET);
		}		
	}
}
void runDoseGROUPA(uint8_t speed, uint8_t a_mlis){
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	for (int i = 0; i<4; i++) {
	if (modePumpDoser[i] == FRE_A) {
		if( i == 0) {
			TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			TIM_OCInitStructure.TIM_Pulse = (2100/speed)*100;
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

			TIM_OC1Init(TIM3, &TIM_OCInitStructure);

			TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
			
		}
		else if (i == 1) {
			TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			TIM_OCInitStructure.TIM_Pulse = (2100/speed)*100;
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

			TIM_OC2Init(TIM3, &TIM_OCInitStructure);

			TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
		}
		else if (i == 2) {
			TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			TIM_OCInitStructure.TIM_Pulse = (2100/speed)*100;
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

			TIM_OC3Init(TIM3, &TIM_OCInitStructure);

			TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
		}
		else if ( i== 3) {
			TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			TIM_OCInitStructure.TIM_Pulse = (2100/speed)*100;
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

			TIM_OC4Init(TIM3, &TIM_OCInitStructure);

			TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
		}
		else break;
		mydelayms((unsigned int)a_mlis*1000);
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
		TIM_OC1Init(TIM3, &TIM_OCInitStructure);
		TIM_OC2Init(TIM3, &TIM_OCInitStructure);
		TIM_OC3Init(TIM3, &TIM_OCInitStructure);
		TIM_OC4Init(TIM3, &TIM_OCInitStructure);
		GPIO_WriteBit(GPIOD,GPIO_Pin_12,Bit_RESET);
		GPIO_WriteBit(GPIOD,GPIO_Pin_13,Bit_RESET);
		GPIO_WriteBit(GPIOD,GPIO_Pin_14,Bit_RESET);
		GPIO_WriteBit(GPIOD,GPIO_Pin_15,Bit_RESET);
		}		
	}
}
void runDoseGROUPB(uint8_t speed, uint8_t a_mlis) {
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	for (int i = 0; i<4; i++) {
	if (modePumpDoser[i] == FRE_B) {
		if( i == 0) {
			TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			TIM_OCInitStructure.TIM_Pulse = (21000/speed)*100;
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

			TIM_OC1Init(TIM4, &TIM_OCInitStructure);

			TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
			
		}
		else if (i == 1) {
			TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			TIM_OCInitStructure.TIM_Pulse = (21000/speed)*100;
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

			TIM_OC2Init(TIM4, &TIM_OCInitStructure);

			TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
		}
		else if (i == 2) {
			TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			TIM_OCInitStructure.TIM_Pulse = (21000/speed)*100;
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

			TIM_OC3Init(TIM4, &TIM_OCInitStructure);

			TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
		}
		else if ( i== 3) {
			TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			TIM_OCInitStructure.TIM_Pulse = (21000/speed)*100;
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

			TIM_OC4Init(TIM4, &TIM_OCInitStructure);

			TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
		}
		else break;
		mydelayms((unsigned int)a_mlis*1000);
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
		TIM_OC1Init(TIM4, &TIM_OCInitStructure);
		TIM_OC2Init(TIM4, &TIM_OCInitStructure);
		TIM_OC3Init(TIM4, &TIM_OCInitStructure);
		TIM_OC4Init(TIM4, &TIM_OCInitStructure);
		GPIO_WriteBit(GPIOD,GPIO_Pin_12,Bit_RESET);
		GPIO_WriteBit(GPIOD,GPIO_Pin_13,Bit_RESET);
		GPIO_WriteBit(GPIOD,GPIO_Pin_14,Bit_RESET);
		GPIO_WriteBit(GPIOD,GPIO_Pin_15,Bit_RESET);
		}		
	}
}
void ControlWater(void *pvParameters)	{

	while (1) {
		
		float phPercent = 0, tdsPercent = 0;
		float targetPh = 0, targetTds = 0;
		float sensorPercent = 0, mlsMultipler = 1;
		uint8_t dosingMode = 0;
		uint8_t phDoseCount = 0, tdsDoseCount = 0;
		uint8_t i = 0;
		if (xSemaphoreTake(xSemaphoredosingInterval,portMAX_DELAY) == pdTRUE ) {
			dosingComplete = false;
			while (!dosingComplete && xSemaphoreTake(xSemaphoresensorsReady,portMAX_DELAY) == pdTRUE) {
				logPhDownResult = false;
				logPhUpResult = false;
				logTdsResult = false;
				// Check if the PPM is out of range
				if (data_sensor[0] == 0) {
					targetTds = data_sensor[0];
				}
				else if (data_sensor[0] < targetMinTds - tdsOffset) {
					targetTds = targetMinTds;
					tdsPercent = percentOutOfRange(targetTds, data_sensor[0]);
				}
				// Check if the PH is out of range
				if (data_sensor[1] == 0) {
					targetPh = data_sensor[1];
				}
				else if (data_sensor[1] < targetMinPh - phOffset) {
					targetPh = targetMinPh;
					adjustPhDown = false;
					phPercent = percentOutOfRange(targetPh, data_sensor[1]);
				}
				else if (data_sensor[1] > targetMaxPh + phOffset) {
					targetPh = targetMaxPh;
					adjustPhDown = true;
					phPercent = percentOutOfRange(targetPh, data_sensor[1]);
				}
				if (phPercent > 0) {
					if (phPercent != 0 && tdsPercent > 0 && phDoseCount >= swapInterval) {
						phPercent = 0;
						phDoseCount = 0;
					}
					else if (tdsPercent != 0 && phPercent > 0 && tdsDoseCount >= swapInterval) {
						tdsPercent = 0;
						tdsDoseCount = 0;
					}
				}
				
				sensorPercent = phPercent;
				if (tdsPercent > phPercent) {
					sensorPercent = tdsPercent;
					dosingMode = 1;
				}
				
				if (sensorPercent > 0) {
					// ====== PH ======
          if (dosingMode == 0) {
            //Setting up PH dosing
            if (phDoseCount < 255)
              phDoseCount++;
            tdsDoseCount = 0;
            if (adjustPhDown) {
							for (i =0; i<4;i++) {
								if (modePumpDoser[i] == PH_DOWN) {
									dosingAmount = speedtomils(speedPumpDoser[i]);
									break;
								}
							}
              loadMachineLearning(&data_sensor[1], &previousPhDownSensor, &phDownSensorHistory, &targetMinPh
                                  , &logPhDownResult, &phDownDosingInc[0], &phDownArrayBlock, &savePhDownResult
                                  , &dosingAmount, &phDownMls, &phDownMultipler, &previousPhDownMls, (bool*)true);
              mlsMultipler = phDownMultipler;
							//Run doser ph down 
							runDosePHDOWN(speedPumpDoser[i], mlsMultipler);
            }
            else {
							for (i =0; i<4;i++) {
								if (modePumpDoser[i] == PH_UP) {
									dosingAmount = speedtomils(speedPumpDoser[i]);
									break;
								}
							}
              loadMachineLearning(&data_sensor[1], &previousPhUpSensor, &phUpSensorHistory, &targetMaxPh
                                  , &logPhUpResult, &phUpDosingInc[0], &phUpArrayBlock, &savePhUpResult
                                  , &dosingAmount, &phUpMls, &phUpMultipler, &previousPhUpMls, (bool*)true);
              mlsMultipler = phUpMultipler;
							runDosePHUP(speedPumpDoser[i], mlsMultipler);
            }
          }
          // ====== PPM ======
          if (dosingMode == 1) {
            //Setting up PPM dosing
            if (tdsDoseCount < 255)
              tdsDoseCount++;
            phDoseCount = 0;
						for (i =0; i<4;i++) {
								if (modePumpDoser[i] == FRE_A) 
									dosingAmount += speedtomils(speedPumpDoser[i]);
								if (modePumpDoser[i] == FRE_B) 
									dosingAmount += speedtomils(speedPumpDoser[i]);
							}
            loadMachineLearning(&data_sensor[0], &previousTdsSensor, &tdsSensorHistory, (float*)targetMaxTds
                                , &logTdsResult, &tdsDosingInc[0], &tdsArrayBlock, &saveTdsResult
                                , &dosingAmount, &tdsMls, &tdsMultipler, &previousTdsMls, (bool*)true);
						tdsAMultipler = tdsMultipler * ratiosDosing;
						tdsBMultipler = tdsMultipler - tdsAMultipler;
						for (i =0; i<4;i++) {
								if (modePumpDoser[i] == FRE_A) 
									runDoseGROUPA(speedPumpDoser[i],tdsAMultipler);
								if (modePumpDoser[i] == FRE_B) 
									runDoseGROUPB(speedPumpDoser[i],tdsBMultipler);
							}	
						}
					}
					else 
          dosingComplete = true;
			} 
		}
	}
}
void WaterPlants(void *pvParameters) {
	GPIO_WriteBit(GPIOD,GPIO_Pin_0,Bit_SET);
	GPIO_WriteBit(GPIOD,GPIO_Pin_1,Bit_SET);
	vTaskSuspend(NULL);
	while (1) {
		if (xSemaphoreTake(xSemaphoreEXTIZCD,portMAX_DELAY) == pdTRUE ) {
			vTaskDelay(1);
			GPIO_WriteBit(GPIOD,GPIO_Pin_4,Bit_SET);
			GPIO_WriteBit(GPIOD,GPIO_Pin_5,Bit_SET);
			delayus(100);
			GPIO_WriteBit(GPIOD,GPIO_Pin_4,Bit_RESET);
			GPIO_WriteBit(GPIOD,GPIO_Pin_5,Bit_RESET);
		}
	}
}


