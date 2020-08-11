#include "main.h"

int main(void) {
	  /* USART configuration -----------------------------------------------------*/
  USART_Config();
	GPIO_Configuration();         //Config LED PIN
	I2C_Config();
	ADC_Config();
	
	EXTI_Config();
	//Create Semaphore 
	xSemaphoreUARTRX = xSemaphoreCreateBinary();
	xSemaphoreChangepage = xSemaphoreCreateBinary();

	xSemaphoredosingInterval = xSemaphoreCreateBinary();
	xSemaphoresensorsReady = xSemaphoreCreateBinary();
	xSemaphoreHandleData = xSemaphoreCreateBinary();
	xSemaphoreEXTIZCD = xSemaphoreCreateBinary();
	xSemaphoreHadData = xSemaphoreCreateBinary();
	SaveDataEvery3Minute = xSemaphoreCreateBinary();
	
	xTaskCreate(handleUARTRX, "xu ly data rev from hmi",  configMINIMAL_STACK_SIZE, NULL, 2, NULL);
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
	xTaskCreate(pageCal, "pageCal",  40, NULL, 1, &TaskpageCal);
	xTaskCreate(pageCalpH, "pageCalpH",  configMINIMAL_STACK_SIZE, NULL, 1, &TaskpageCalpH);
	xTaskCreate(pageCalPPM, "pageCalPPM",  configMINIMAL_STACK_SIZE, NULL, 1, &TaskpageCalPPM);
	xTaskCreate(pageCalTemp, "pageCalTemp",  configMINIMAL_STACK_SIZE, NULL, 1, &TaskpageCalTemp);
	xTaskCreate(pageCalSonic, "pageCalSonic",  configMINIMAL_STACK_SIZE, NULL, 1, &TaskpageCalSonic);
	xTaskCreate(pageFlow, "pageFlow",  configMINIMAL_STACK_SIZE, NULL, 1, &TaskpageFlow);
	xTaskCreate(pageProfile, "pageProfile",  configMINIMAL_STACK_SIZE, NULL, 1, &TaskpageProfile);
	xTaskCreate(pageFeritilizer, "pageFeritilizer",  configMINIMAL_STACK_SIZE, NULL, 1, &TaskpageFeritilizer);
	
	//Task control systerm
	xTaskCreate(readSensor, "readSensor", 200, NULL, 2, NULL);
	xTaskCreate(handleRTC, "handleRTC", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate(runPeristaltic, "runPeristaltic",  configMINIMAL_STACK_SIZE, NULL, 2, &runPer);
//	xTaskCreate(temperature_task, "temperature_task", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
//	xTaskCreate(sonic_task, "sonic_task", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
//	xTaskCreate(adc_task, "adc_task", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	//xTaskCreate(ControlWater, "ControlEnviromental", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(WaterPlants, "WaterPlants", 400, NULL, 2, NULL);
	xTaskCreate(DrainingWater, "DrainingWater", 200, NULL, 1, NULL);
	xTaskCreate(I2C_ESP, "I2C_ESP", 1024, NULL, 2, NULL);
	vTaskStartScheduler();
	
	for(;;){}
}

void TIM1_BRK_TIM9_IRQHandler() {
	uint16_t capture = 0;
	//static BaseType_t xHigherPriorityTaskWoken ;
	//xHigherPriorityTaskWoken = pdFALSE ;
	if (TIM_GetITStatus(TIM9, TIM_IT_CC1) != RESET) {
    //TIM_ClearITPendingBit(TIM9, TIM_IT_CC1);

    /* LED4 toggling with frequency = 500 Hz (Debug hardware) */
		
    GPIO_ToggleBits(GPIOD,GPIO_Pin_6);
    capture = TIM_GetCapture1(TIM9);
//		if (swapPWM == true) {
//			capture = 1000;
//			swapPWM = false;
//		}
//		else {
//			swapPWM = true;
//			capture += CCR4_Val;
//		}
    //TIM_SetCompare1(TIM9, capture + CCR4_Val);
  }
	//portYIELD_FROM_ISR( xHigherPriorityTaskWoken );	
}
void TIM4_IRQHandler() {
	static BaseType_t xHigherPriorityTaskWoken ;
	xHigherPriorityTaskWoken = pdFALSE ;
	if (TimeOut != 0x0)
  {
    TimeOut--;
  }
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );	
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
  if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1) == SET && StartWaterPlant == true)
  {
		xSemaphoreGiveFromISR( xSemaphoreEXTIZCD, &xHigherPriorityTaskWoken );
		/* Clear the EXTI line 1 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line1);
  }
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void EndData(void) {
	char threeByteEnd[3] = {END_DATA,END_DATA,END_DATA};
	USART3_puts(&threeByteEnd[0]);
} // EndData();
void pageHome(void *pvParameters) {
	uint8_t prevHour = 0;
	uint8_t prevMinute = 0;
	char bufferSend[] = "";
	prevHour = rtc_getdatetime.hour;
	prevMinute = rtc_getdatetime.minutes;
	uint8_t prevID = 10;
	uint16_t preNumValSaved = 0;
	vTaskSuspend(NULL);
	while (1) {
		prevpage = HOME;
		if (reloadPage == true) {
			prevID = 10;
			preNumValSaved = 0;
			reloadPage = false;
		}
		if (IDHMI == 0x00 && DataHMI[0] == 0) {
			if (prevID != IDHMI ) {
				USART3_puts("cle 19,0");
				EndData();
				if (NumValSaved > 0 ) {
					preNumValSaved = NumValSaved;
					sprintf(bufferSend,"addt 19,0,%d",NumValSaved);
					USART3_puts(&bufferSend[0]);
					EndData();
					for (int j = 0; j<NumValSaved;j++) {
						USART3_puts(&sensorPHinDay[j]);
					}
				}
				
				prevID = IDHMI;
				
				USART3_puts("t20.txt=\"PPM\"");
				EndData();
				
				USART3_puts("b0.bco=1024");
				EndData();
				USART3_puts("b1.bco=50712");
				EndData();
				USART3_puts("b3.bco=50712");
				EndData();
				USART3_puts("b2.bco=50712");
				EndData();
				
				sprintf(bufferSend,"t24.txt=\"%d\"",tdsOffsetALL);
				USART3_puts(&bufferSend[0]);
				EndData();
				
				sprintf(bufferSend,"t27.txt=\"%d\"",targetMinTdsALL);
				USART3_puts(&bufferSend[0]);
				EndData();
				
				sprintf(bufferSend,"t28.txt=\"%d\"",targetMaxTdsALL);
				USART3_puts(&bufferSend[0]);
				EndData();
				
			}
			if (NumValSaved - preNumValSaved >0) {
				sprintf(bufferSend,"addt 19,0,%d",NumValSaved - preNumValSaved);
				USART3_puts(&bufferSend[0]);
				EndData();
				for (int j = preNumValSaved;j<NumValSaved;j++) {
					USART3_puts(&sensorPHinDay[j]);
				}
				preNumValSaved = NumValSaved;
			}
			sprintf(bufferSend,"t23.txt=\"%0.1f\"",data_sensor[0]);
			USART3_puts(&bufferSend[0]);
			EndData();
			xSemaphoreGive(xSemaphoreHandleData);
		}
		else if (IDHMI == 0x02 && DataHMI[0] == 2) {
			if (prevID != IDHMI) {
				USART3_puts("cle 19,0");
				EndData();
				if (NumValSaved > 0 ) {
					preNumValSaved = NumValSaved;
					sprintf(bufferSend,"addt 19,0,%d",NumValSaved);
					USART3_puts(&bufferSend[0]);
					EndData();
					for (int j = 0; j<NumValSaved;j++) {
						USART3_puts(&sensorTEMPinDay[j]);
					}
				}
				
				prevID = IDHMI;
				USART3_puts("t20.txt=\"TEMPERATURE\"");
				EndData();
				USART3_puts("b0.bco=50712");
				EndData();
				USART3_puts("b1.bco=1024");
				EndData();
				USART3_puts("b3.bco=50712");
				EndData();
				USART3_puts("b2.bco=50712");
				EndData();
				
				USART3_puts("t24.txt=\"\"");
				EndData();
				USART3_puts("t27.txt=\"\"");
				EndData();
				USART3_puts("t28.txt=\"\"");
				EndData();
				
			}

			if (NumValSaved - preNumValSaved >0) {
				sprintf(bufferSend,"addt 19,0,%d",NumValSaved - preNumValSaved);
				USART3_puts(&bufferSend[0]);
				EndData();
				for (int j = preNumValSaved;j<NumValSaved;j++) {
					USART3_puts(&sensorTEMPinDay[j]);
				}
				preNumValSaved = NumValSaved;
			}
			sprintf(bufferSend,"t23.txt=\"%0.1f\"",data_sensor[3]);
			USART3_puts(&bufferSend[0]);
			EndData();
			xSemaphoreGive(xSemaphoreHandleData);
		}
		else if (IDHMI == 0x03 && DataHMI[0] == 3) {
			if (prevID != IDHMI) {
				USART3_puts("cle 19,0");
				EndData();
				if (NumValSaved > 0 ) {
					preNumValSaved = NumValSaved;
					sprintf(bufferSend,"addt 19,0,%d",NumValSaved);
					USART3_puts(&bufferSend[0]);
					EndData();
					for (int j = 0; j<NumValSaved;j++) {
						USART3_puts(&sensorTANKinDay[j]);
					}
				}
				prevID = IDHMI;
				USART3_puts("t20.txt=\"TANK\"");
				EndData();
				USART3_puts("b0.bco=50712");
				EndData();
				USART3_puts("b1.bco=50712");
				EndData();
				USART3_puts("b3.bco=1024");
				EndData();
				USART3_puts("b2.bco=50712");
				EndData();
				
				sprintf(bufferSend,"t24.txt=\"%d\"",TankLvOffset);
				USART3_puts(&bufferSend[0]);
				EndData();
				sprintf(bufferSend,"t27.txt=\"%d\"",targetMinTankLv);
				USART3_puts(&bufferSend[0]);
				EndData();
				sprintf(bufferSend,"t28.txt=\"%d\"",targetMaxTankLv);
				USART3_puts(&bufferSend[0]);
				EndData();
				
			}
			
			if (NumValSaved - preNumValSaved >0) {
				sprintf(bufferSend,"addt 19,0,%d",NumValSaved - preNumValSaved);
				USART3_puts(&bufferSend[0]);
				EndData();
				for (int j = preNumValSaved;j<NumValSaved;j++) {
					USART3_puts(&sensorTANKinDay[j]);
				}
				preNumValSaved = NumValSaved;
			}
			sprintf(bufferSend,"t23.txt=\"%0.1f\"",data_sensor[2]);
			USART3_puts(&bufferSend[0]);
			EndData();
			xSemaphoreGive(xSemaphoreHandleData);
		}
		else if (IDHMI == 0x04 && DataHMI[0] == 4) {
			if (prevID != IDHMI) {
				USART3_puts("cle 19,0");
				EndData();
				if (NumValSaved > 0 ) {
					preNumValSaved = NumValSaved;
					sprintf(bufferSend,"addt 19,0,%d",NumValSaved);
					USART3_puts(&bufferSend[0]);
					EndData();
					for (int j = 0; j<NumValSaved;j++) {
						USART3_puts(&sensorPHinDay[j]);
					}
				}
				prevID = IDHMI;
				USART3_puts("t20.txt=\"PH\"");
				EndData();
				USART3_puts("b0.bco=50712");
				EndData();
				USART3_puts("b1.bco=50712");
				EndData();
				USART3_puts("b3.bco=50712");
				EndData();
				USART3_puts("b2.bco=1024");
				EndData();
				
				sprintf(bufferSend,"t24.txt=\"%0.1f\"",phOffset);
				USART3_puts(&bufferSend[0]);
				EndData();
				sprintf(bufferSend,"t27.txt=\"%0.1f\"",targetMinPh);
				USART3_puts(&bufferSend[0]);
				EndData();
				sprintf(bufferSend,"t28.txt=\"%0.1f\"",targetMaxPh);
				USART3_puts(&bufferSend[0]);
				EndData();
			}

			if (NumValSaved - preNumValSaved >0) {
				sprintf(bufferSend,"addt 19,0,%d",NumValSaved - preNumValSaved);
				USART3_puts(&bufferSend[0]);
				EndData();
				for (int j = preNumValSaved;j<NumValSaved;j++) {
					USART3_puts(&sensorPHinDay[j]);
				}
				preNumValSaved = NumValSaved;
			}
			sprintf(bufferSend,"t23.txt=\"%0.1f\"",data_sensor[1]);
			USART3_puts(&bufferSend[0]);
			EndData();
			xSemaphoreGive(xSemaphoreHandleData);
		}
//		else
//			xSemaphoreGive(xSemaphoreHandleData);
		if (page != HOME) 
		{
			vTaskSuspend(NULL);
		}
		sprintf(bufferSend,"hour.val=%d",rtc_getdatetime.hour);
		USART3_puts(&bufferSend[0]);
		EndData();
		sprintf(bufferSend,"minute.txt=\"%d\"",rtc_getdatetime.minutes);
		USART3_puts(&bufferSend[0]);
		EndData();
		vTaskDelay(300);
	}
}
void pageWarning(void *pvParameters) {
	uint8_t prevID = 10;
	char bufferSend[] = "";
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
				EndData();
				USART3_puts("b0.bco=1024");
				EndData();
				USART3_puts("b1.bco=50712");
				EndData();
				USART3_puts("b2.bco=50712");
				EndData();
				USART3_puts("b3.bco=50712");
				EndData();
			}
			prevID = IDHMI;	
		}
		else if (IDHMI == 1 && DataHMI[0] == 1) {
			if (prevID != IDHMI) {
				USART3_puts("t2.txt=\"TEMPERATURE WARNING\"");
				EndData();
				USART3_puts("b0.bco=50712");
				EndData();
				USART3_puts("b1.bco=1024");
				EndData();
				USART3_puts("b2.bco=50712");
				EndData();
				USART3_puts("b3.bco=50712");
				EndData();
			}
			prevID = IDHMI;
			
		}
		else if (IDHMI == 2 && DataHMI[0] == 2) {
			if (prevID != IDHMI) {
				USART3_puts("t2.txt=\"pH WARNING\"");
				EndData();
				USART3_puts("b0.bco=50712");
				EndData();
				USART3_puts("b1.bco=50712");
				EndData();
				USART3_puts("b2.bco=1024");
				EndData();
				USART3_puts("b3.bco=50712");
				EndData();
			}
			prevID = IDHMI;
		}
		else if (IDHMI == 3 && DataHMI[0] == 3) {
			if (prevID != IDHMI) {
				USART3_puts("t2.txt=\"TANK WARNING\"");
				EndData();
				USART3_puts("b0.bco=50712");
				EndData();
				USART3_puts("b1.bco=50712");
				EndData();
				USART3_puts("b2.bco=50712");
				EndData();
				USART3_puts("b3.bco=1024");
				EndData();
			}
			prevID = IDHMI;
		}
		if (prevID == 0) {
			sprintf(bufferSend,"t3.txt=\"%0.1f\"",data_sensor[0]);
			USART3_puts(&bufferSend[0]);
			EndData(); 	
			sprintf(bufferSend,"t1.txt=\"%f\"",errorMarginPPM);
			USART3_puts(&bufferSend[0]);
			EndData();
			xSemaphoreGive(xSemaphoreHandleData);
		}
		if (prevID == 1) {
			sprintf(bufferSend,"t3.txt=\"%0.1f\"",data_sensor[3]);
			USART3_puts(&bufferSend[0]);
			EndData();
			sprintf(bufferSend,"t1.txt=\"%0.1f\"",errorMarginTEMP);
			USART3_puts(&bufferSend[0]);
			EndData();
			xSemaphoreGive(xSemaphoreHandleData);
		}
		if (prevID == 2) {
			sprintf(bufferSend,"t3.txt=\"%0.1f\"",data_sensor[1]);
			USART3_puts(&bufferSend[0]);
			EndData();
			sprintf(bufferSend,"x0.val=%d",(uint8_t)(errorMarginPH*10));
			USART3_puts(&bufferSend[0]);
			EndData();
			xSemaphoreGive(xSemaphoreHandleData);
		}
		if (prevID == 3) {
			sprintf(bufferSend,"t3.txt=\"%0.1f\"",data_sensor[2]);
			USART3_puts(&bufferSend[0]);
			EndData();
			sprintf(bufferSend,"t1.txt=\"%d\"",(uint16_t)errorMarginTANK);
			USART3_puts(&bufferSend[0]);
			EndData();
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
	char bufferSend[] = "";
	vTaskSuspend(NULL);
	while (1) {
		prevpage = WIFI;
		if (reloadPage == true) 
		{
			sprintf(bufferSend,"n0.val=%d",WifiInterval);
			USART3_puts(&bufferSend[0]);
			EndData();
			reloadPage = false;
		}
		if (IDHMI == 0x01) {
			WifiInterval = (uint16_t)((DataHMI[1]<<8) | DataHMI[0]);
		}
		xSemaphoreGive(xSemaphoreHandleData);

		vTaskDelay(150/portTICK_PERIOD_MS);
	}
}
void pagePPM(void *pvParameters) {
	char bufferSend[] = "";
	vTaskSuspend(NULL);
	while (1) {
		prevpage = PPM;
		if (reloadPage == true) {
			sprintf(bufferSend,"n0.val=%d",targetMinTdsALL);
			USART3_puts(&bufferSend[0]);
			EndData();
			sprintf(bufferSend,"n1.val=%d",targetMaxTdsALL);
			USART3_puts(&bufferSend[0]);
			EndData();
			sprintf(bufferSend,"n2.val=%d",tdsOffsetALL);
			USART3_puts(&bufferSend[0]);
			EndData();
			reloadPage = false;
		}
		if (xSemaphoreTake( xSemaphoreHadData,1000) == pdTRUE) {
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
	char bufferSend[] = "";
	uint8_t NumDay = 0;
	const uint16_t Green = 1024;
	const uint16_t Red = 63488;
	const uint16_t Blue = 255;
	const uint16_t Yellow = 65504;
	const uint16_t Purple = 32784;
	uint8_t count = 0;
	vTaskSuspend(NULL);
	while (1) {
		prevpage = PPM2;
		if (reloadPage == true) {
			count = 0;
			while (DayPPMStageOne[count] != 0) {
				sprintf(bufferSend,"n%d.bco=%d",DayPPMStageOne[count],Green);
				USART3_puts(&bufferSend[0]);
				EndData();
				count++;
			}
			
			count = 0;
			while (DayPPMStageTwo[count] != 0) {
				sprintf(bufferSend,"n%d.bco=%d",DayPPMStageTwo[count],Red);
				USART3_puts(&bufferSend[0]);
				EndData();
				count++;
			}
			
			count = 0;
			while (DayPPMStageThree[count] != 0) {
				sprintf(bufferSend,"n%d.bco=%d",DayPPMStageThree[count],Yellow);
				USART3_puts(&bufferSend[0]);
				EndData();
				count++;
			}

			count = 0;
			while (DayPPMStageFour[count] != 0) {
				sprintf(bufferSend,"n%d.bco=%d",DayPPMStageFour[count],Blue);
				USART3_puts(&bufferSend[0]);
				EndData();
				count++;
			}
			
			count = 0;
			while (DayPPMStageFive[count] != 0) {
				sprintf(bufferSend,"n%d.bco=%d",DayPPMStageFive[count],Purple);
				USART3_puts(&bufferSend[0]);
				EndData();
				count++;
			}

			sprintf(bufferSend,"va0.val=%d",PPMStage[0]);
			USART3_puts(&bufferSend[0]);
			EndData();
			sprintf(bufferSend,"n32.val=%d",PPMStage[0]);
			USART3_puts(&bufferSend[0]);
			EndData();
			
			sprintf(bufferSend,"va1.val=%d",PPMStage[1]);
			USART3_puts(&bufferSend[0]);
			EndData();
			
			sprintf(bufferSend,"va2.val=%d",PPMStage[2]);
			USART3_puts(&bufferSend[0]);
			EndData();
			
			sprintf(bufferSend,"va3.val=%d",PPMStage[3]);
			USART3_puts(&bufferSend[0]);
			EndData();
			
			sprintf(bufferSend,"va4.val=%d",PPMStage[4]);
			USART3_puts(&bufferSend[0]);
			EndData();		
			
			reloadPage = false;
		}
		if (xSemaphoreTake( xSemaphoreHadData,1000) == pdTRUE) {
			if (IDHMI == 0x03) {
				count = 0;
				PPMStage[0] = (uint16_t)((DataHMI[1]<<8) | DataHMI[0]);
				if (PPMStage[0] < targetMinTdsALL - tdsOffsetALL ||PPMStage[0] > targetMaxTdsALL + tdsOffsetALL)
					PPMStage[0] = 0;
				while (DataHMI[count+2] != 0) {
					DayPPMStageOne[count] = DataHMI[count+2];
					count ++ ;
				}
				xSemaphoreGive(xSemaphoreHandleData);
			}
			if (IDHMI == 0x04) {
				count = 0;
				PPMStage[1] = (uint16_t)((DataHMI[1]<<8) | DataHMI[0]);
				if (PPMStage[1] < targetMinTdsALL - tdsOffsetALL ||PPMStage[1] > targetMaxTdsALL + tdsOffsetALL)
					PPMStage[1] = 0;
				while (DataHMI[count+2] != 0) {
					DayPPMStageTwo[count] = DataHMI[count+2];
					count ++ ;
				}
				xSemaphoreGive(xSemaphoreHandleData);
			}
			if (IDHMI == 0x05) {
				count = 0;
				PPMStage[2] = (uint16_t)((DataHMI[1]<<8) | DataHMI[0]);
				if (PPMStage[2] < targetMinTdsALL - tdsOffsetALL ||PPMStage[2] > targetMaxTdsALL + tdsOffsetALL)
					PPMStage[2] = 0;
				while (DataHMI[count+2] != 0) {
					DayPPMStageThree[count] = DataHMI[count+2];
					count ++ ;
				}
				xSemaphoreGive(xSemaphoreHandleData);
			}
			if (IDHMI == 0x06) {
				count = 0;
				PPMStage[3] = (uint16_t)((DataHMI[1]<<8) | DataHMI[0]);
				if (PPMStage[3] < targetMinTdsALL - tdsOffsetALL ||PPMStage[3] > targetMaxTdsALL + tdsOffsetALL)
					PPMStage[3] = 0;
				while (DataHMI[count+2] != 0) {
					DayPPMStageFour[count] = DataHMI[count+2];
					count ++ ;
				}
				xSemaphoreGive(xSemaphoreHandleData);
			}
			if (IDHMI == 0x07) {
				count = 0;
				PPMStage[4] = (uint16_t)((DataHMI[1]<<8) | DataHMI[0]);
				if (PPMStage[4] < targetMinTdsALL - tdsOffsetALL ||PPMStage[4] > targetMaxTdsALL + tdsOffsetALL)
					PPMStage[4] = 0;
				while (DataHMI[count+2] != 0) {
					DayPPMStageFive[count] = DataHMI[count+2];
					count ++ ;
				}
				xSemaphoreGive(xSemaphoreHandleData);
			}
		}
	}
}
void pagepH(void *pvParameters) {
	char bufferSend[] = "";
	vTaskSuspend(NULL);
	while (1) {
		prevpage = PH;
		if (reloadPage == true) {
			sprintf(bufferSend,"x0.val=%d",(uint16_t)(targetMinPh*10));
			USART3_puts(&bufferSend[0]);
			EndData();
			sprintf(bufferSend,"x1.val=%d",(uint16_t)(targetMaxPh*10));
			USART3_puts(&bufferSend[0]);
			EndData();
			sprintf(bufferSend,"x2.val=%d",(uint16_t)(phOffset*10));
			USART3_puts(&bufferSend[0]);
			EndData();
			reloadPage = false;
		}
		if (xSemaphoreTake( xSemaphoreHadData,1000) == pdTRUE) {
			if (IDHMI == 1) {
				targetMinPh = (float)(((DataHMI[1]<<8) | DataHMI[0])/10);
		
				targetMaxPh = (float)(((DataHMI[3]<<8) | DataHMI[2])/10);
				
				phOffset = (float)(((DataHMI[5]<<8) | DataHMI[4])/10);
			}
			xSemaphoreGive(xSemaphoreHandleData);
		}
	}
}
void pageTank(void *pvParameters) {
	char bufferSend[] = "";
	vTaskSuspend(NULL);
	while (1) {
		prevpage = TANK;
		if (reloadPage == true) {
			sprintf(bufferSend,"n0.val=%d",targetMinTankLv);
			USART3_puts(&bufferSend[0]);
			EndData();
			sprintf(bufferSend,"n1.val=%d",targetMaxTankLv);
			USART3_puts(&bufferSend[0]);
			EndData();
			sprintf(bufferSend,"n2.val=%d",TankLvOffset);
			USART3_puts(&bufferSend[0]);
			EndData();
			reloadPage = false;
		}
		if (xSemaphoreTake( xSemaphoreHadData,1000) == pdTRUE) {
			if (IDHMI == 1) {
				targetMinTankLv = (uint16_t)((DataHMI[1]<<8) | DataHMI[0]);
		
				targetMaxTankLv = (uint16_t)((DataHMI[3]<<8) | DataHMI[2]);
				
				TankLvOffset = (uint16_t)((DataHMI[5]<<8) | DataHMI[4]);
			}
			xSemaphoreGive(xSemaphoreHandleData);
		}
	}
}
void pageTank2(void *pvParameters) {
	vTaskSuspend(NULL);
	while (1) {
		vTaskDelete(NULL);
	}
}
void pageValve(void *pvParameters) {
	char bufferSend[] = "";
	vTaskSuspend(NULL);
	while (1) {
		prevpage = VALVE;
		if (reloadPage == true) {
			switch (ValveOneInfo[0]) {
				case 0x00: {
					USART3_puts("t7.txt=\"NONE\"");
					EndData();
					break;
				}
				case 0x01: {
					USART3_puts("t7.txt=\"OUT\"");
					EndData();
					break;
				}
				case 0x02: {
					USART3_puts("t7.txt=\"IN\"");
					EndData();
					break;
				}
				case 0x03: {
					USART3_puts("t7.txt=\"DRAIN\"");
					EndData();
					break;
				}
				case 0x04: {
					USART3_puts("t7.txt=\"SUP\"");
					EndData();
					break;
				}
			}
			switch (ValveOneInfo[1]) {
				case 0x00: {
					USART3_puts("t1.txt=\"NONE\"");
					EndData();
					break;
				}
				case 0x01: {
					USART3_puts("t1.txt=\"OUT\"");
					EndData();
					break;
				}
				case 0x02: {
					USART3_puts("t1.txt=\"IN\"");
					EndData();
					break;
				}
				case 0x03: {
					USART3_puts("t1.txt=\"DRAIN\"");
					EndData();
					break;
				}
				case 0x04: {
					USART3_puts("t1.txt=\"SUP\"");
					EndData();
					break;
				}
			}
			switch (ValveOneInfo[2]) {
				case 0x00: {
					USART3_puts("t2.txt=\"NO\"");
					EndData();
					break;
				}
				case 0x01: {
					USART3_puts("t2.txt=\"NC\"");
					EndData();
					break;
				}
			}
			switch (ValveTwoInfo[0]) {
				case 0x00: {
					USART3_puts("t8.txt=\"NONE\"");
					EndData();
					break;
				}
				case 0x01: {
					USART3_puts("t8.txt=\"OUT\"");
					EndData();
					break;
				}
				case 0x02: {
					USART3_puts("t8.txt=\"IN\"");
					EndData();
					break;
				}
				case 0x03: {
					USART3_puts("t8.txt=\"DRAIN\"");
					EndData();
					break;
				}
				case 0x04: {
					USART3_puts("t8.txt=\"SUP\"");
					EndData();
					break;
				}
			}
			switch (ValveTwoInfo[1]) {
				case 0x00: {
					USART3_puts("t3.txt=\"NONE\"");
					EndData();
					break;
				}
				case 0x01: {
					USART3_puts("t3.txt=\"OUT\"");
					EndData();
					break;
				}
				case 0x02: {
					USART3_puts("t3.txt=\"IN\"");
					EndData();
					break;
				}
				case 0x03: {
					USART3_puts("t3.txt=\"DRAIN\"");
					EndData();
					break;
				}
				case 0x04: {
					USART3_puts("t3.txt=\"SUP\"");
					EndData();
					break;
				}
			}
			switch (ValveTwoInfo[2]) {
				case 0x00: {
					USART3_puts("t4.txt=\"NO\"");
					EndData();
					break;
				}
				case 0x01: {
					USART3_puts("t4.txt=\"NC\"");
					EndData();
					break;
				}
			}
			switch (ValveThreeInfo[0]) {
				case 0x00: {
					USART3_puts("t9.txt=\"NONE\"");
					EndData();
					break;
				}
				case 0x01: {
					USART3_puts("t9.txt=\"OUT\"");
					EndData();
					break;
				}
				case 0x02: {
					USART3_puts("t9.txt=\"IN\"");
					EndData();
					break;
				}
				case 0x03: {
					USART3_puts("t9.txt=\"DRAIN\"");
					EndData();
					break;
				}
				case 0x04: {
					USART3_puts("t9.txt=\"SUP\"");
					EndData();
					break;
				}
			}
			switch (ValveThreeInfo[1]) {
				case 0x00: {
					USART3_puts("t5.txt=\"NONE\"");
					EndData();
					break;
				}
				case 0x01: {
					USART3_puts("t5.txt=\"OUT\"");
					EndData();
					break;
				}
				case 0x02: {
					USART3_puts("t5.txt=\"IN\"");
					EndData();
					break;
				}
				case 0x03: {
					USART3_puts("t5.txt=\"DRAIN\"");
					EndData();
					break;
				}
				case 0x04: {
					USART3_puts("t5.txt=\"SUP\"");
					EndData();
					break;
				}
			}
			switch (ValveThreeInfo[2]) {
				case 0x00: {
					USART3_puts("t6.txt=\"NO\"");
					EndData();
					break;
				}
				case 0x01: {
					USART3_puts("t6.txt=\"NC\"");
					EndData();
					break;
				}
			}
			reloadPage = false;
		}	
		if (xSemaphoreTake( xSemaphoreHadData,1000) == pdTRUE) {
			if (IDHMI == 0) {
				ValveOneInfo[0] = DataHMI[0];
				ValveOneInfo[1] = DataHMI[1];
				ValveOneInfo[2] = DataHMI[2];
			}
			if (IDHMI == 1) {
				ValveTwoInfo[0] = DataHMI[0];
				ValveTwoInfo[1] = DataHMI[1];
				ValveTwoInfo[2] = DataHMI[2];
			}
			if (IDHMI == 2) {
				ValveThreeInfo[0] = DataHMI[0];
				ValveThreeInfo[1] = DataHMI[1];
				ValveThreeInfo[2] = DataHMI[2];
			}
			xSemaphoreGive(xSemaphoreHandleData);
		}
	}
}
void pageValve2(void *pvParameters) {
	char bufferSend[] = "";
	vTaskSuspend(NULL);
	while (1) {
		prevpage = VALVE;
		if (reloadPage == true) {
			switch (ValveFourInfo[0]) {
				case 0x00: {
					USART3_puts("t7.txt=\"NONE\"");
					EndData();
					break;
				}
				case 0x01: {
					USART3_puts("t7.txt=\"OUT\"");
					EndData();
					break;
				}
				case 0x02: {
					USART3_puts("t7.txt=\"IN\"");
					EndData();
					break;
				}
				case 0x03: {
					USART3_puts("t7.txt=\"DRAIN\"");
					EndData();
					break;
				}
				case 0x04: {
					USART3_puts("t7.txt=\"SUP\"");
					EndData();
					break;
				}
			}
			switch (ValveFourInfo[1]) {
				case 0x00: {
					USART3_puts("t1.txt=\"NONE\"");
					EndData();
					break;
				}
				case 0x01: {
					USART3_puts("t1.txt=\"OUT\"");
					EndData();
					break;
				}
				case 0x02: {
					USART3_puts("t1.txt=\"IN\"");
					EndData();
					break;
				}
				case 0x03: {
					USART3_puts("t1.txt=\"DRAIN\"");
					EndData();
					break;
				}
				case 0x04: {
					USART3_puts("t1.txt=\"SUP\"");
					EndData();
					break;
				}
			}
			switch (ValveFourInfo[2]) {
				case 0x00: {
					USART3_puts("t2.txt=\"NO\"");
					EndData();
					break;
				}
				case 0x01: {
					USART3_puts("t2.txt=\"NC\"");
					EndData();
					break;
				}
			}
			switch (ValveFiveInfo[0]) {
				case 0x00: {
					USART3_puts("t8.txt=\"NONE\"");
					EndData();
					break;
				}
				case 0x01: {
					USART3_puts("t8.txt=\"OUT\"");
					EndData();
					break;
				}
				case 0x02: {
					USART3_puts("t8.txt=\"IN\"");
					EndData();
					break;
				}
				case 0x03: {
					USART3_puts("t8.txt=\"DRAIN\"");
					EndData();
					break;
				}
				case 0x04: {
					USART3_puts("t8.txt=\"SUP\"");
					EndData();
					break;
				}
			}
			switch (ValveFiveInfo[1]) {
				case 0x00: {
					USART3_puts("t3.txt=\"NONE\"");
					EndData();
					break;
				}
				case 0x01: {
					USART3_puts("t3.txt=\"OUT\"");
					EndData();
					break;
				}
				case 0x02: {
					USART3_puts("t3.txt=\"IN\"");
					EndData();
					break;
				}
				case 0x03: {
					USART3_puts("t3.txt=\"DRAIN\"");
					EndData();
					break;
				}
				case 0x04: {
					USART3_puts("t3.txt=\"SUP\"");
					EndData();
					break;
				}
			}
			switch (ValveFiveInfo[2]) {
				case 0x00: {
					USART3_puts("t4.txt=\"NO\"");
					EndData();
					break;
				}
				case 0x01: {
					USART3_puts("t4.txt=\"NC\"");
					EndData();
					break;
				}
			}
			switch (ValveSixInfo[0]) {
				case 0x00: {
					USART3_puts("t9.txt=\"NONE\"");
					EndData();
					break;
				}
				case 0x01: {
					USART3_puts("t9.txt=\"OUT\"");
					EndData();
					break;
				}
				case 0x02: {
					USART3_puts("t9.txt=\"IN\"");
					EndData();
					break;
				}
				case 0x03: {
					USART3_puts("t9.txt=\"DRAIN\"");
					EndData();
					break;
				}
				case 0x04: {
					USART3_puts("t9.txt=\"SUP\"");
					EndData();
					break;
				}
			}
			switch (ValveSixInfo[1]) {
				case 0x00: {
					USART3_puts("t5.txt=\"NONE\"");
					EndData();
					break;
				}
				case 0x01: {
					USART3_puts("t5.txt=\"OUT\"");
					EndData();
					break;
				}
				case 0x02: {
					USART3_puts("t5.txt=\"IN\"");
					EndData();
					break;
				}
				case 0x03: {
					USART3_puts("t5.txt=\"DRAIN\"");
					EndData();
					break;
				}
				case 0x04: {
					USART3_puts("t5.txt=\"SUP\"");
					EndData();
					break;
				}
			}
			switch (ValveSixInfo[2]) {
				case 0x00: {
					USART3_puts("t6.txt=\"NO\"");
					EndData();
					break;
				}
				case 0x01: {
					USART3_puts("t6.txt=\"NC\"");
					EndData();
					break;
				}
			}
			reloadPage = false;
		}
		if (xSemaphoreTake( xSemaphoreHadData,1000) == pdTRUE) {
			if (IDHMI == 0x01) {
				ValveFourInfo[0] = DataHMI[0];
				ValveFourInfo[1] = DataHMI[1];
				ValveFourInfo[2] = DataHMI[2];
			}
			if (IDHMI == 0x02) {
				ValveFiveInfo[0] = DataHMI[0];
				ValveFiveInfo[1] = DataHMI[1];
				ValveFiveInfo[2] = DataHMI[2];
			}
			if (IDHMI == 0x03) {
				ValveSixInfo[0] = DataHMI[0];
				ValveSixInfo[1] = DataHMI[1];
				ValveSixInfo[2] = DataHMI[2];
			}
			xSemaphoreGive(xSemaphoreHandleData);
		}
	}
}
void pagePeristaltic(void *pvParameters) {
	char bufferSend[] = "";
	vTaskSuspend(NULL);
	while (1) {
		prevpage = PERISTALTIC_PUMP;
		if (reloadPage == true) {
			sprintf(bufferSend,"n0.val=%d",PerPumpOneInfo[0]);
			USART3_puts(&bufferSend[0]);
			EndData();
			sprintf(bufferSend,"h0.val=%d",PerPumpOneInfo[0]);
			USART3_puts(&bufferSend[0]);
			EndData();
			switch (PerPumpOneInfo[1]) {
				case 0x00: {
					USART3_puts("t2.txt=\"A\"");
					EndData();
					break;
				}
				case 0x01: {
					USART3_puts("t2.txt=\"B\"");
					EndData();
					break;
				}
				case 0x02: {
					USART3_puts("t2.txt=\"PH UP\"");
					EndData();
					break;
				}
				case 0x03: {
					USART3_puts("t2.txt=\"PH DOWN\"");
					EndData();
					break;
				}
			}
			sprintf(bufferSend,"n1.val=%d",PerPumpTwoInfo[0]);
			USART3_puts(&bufferSend[0]);
			EndData();
			sprintf(bufferSend,"h1.val=%d",PerPumpTwoInfo[0]);
			USART3_puts(&bufferSend[0]);
			EndData();
			
			switch (PerPumpTwoInfo[1]) {
				case 0x00: {
					USART3_puts("t1.txt=\"A\"");
					EndData();
					break;
				}
				case 0x01: {
					USART3_puts("t1.txt=\"B\"");
					EndData();
					break;
				}
				case 0x02: {
					USART3_puts("t1.txt=\"PH UP\"");
					EndData();
					break;
				}
				case 0x03: {
					USART3_puts("t1.txt=\"PH DOWN\"");
					EndData();
					break;
				}
			}
			
			sprintf(bufferSend,"n2.val=%d",PerPumpThreeInfo[0]);
			USART3_puts(&bufferSend[0]);
			EndData();
			sprintf(bufferSend,"h2.val=%d",PerPumpThreeInfo[0]);
			USART3_puts(&bufferSend[0]);
			EndData();
			
			switch (PerPumpThreeInfo[1]) {
				case 0x00: {
					USART3_puts("t3.txt=\"A\"");
					EndData();
				}
				case 0x01: {
					USART3_puts("t3.txt=\"B\"");
					EndData();
					break;
				}
				case 0x02: {
					USART3_puts("t3.txt=\"PH UP\"");
					EndData();
					break;
				}
				case 0x03: {
					USART3_puts("t3.txt=\"PH DOWN\"");
					EndData();
					break;
				}
			}
			
			sprintf(bufferSend,"n3.val=%d",PerPumpFourInfo[0]);
			USART3_puts(&bufferSend[0]);
			EndData();
			sprintf(bufferSend,"h3.val=%d",PerPumpFourInfo[0]);
			USART3_puts(&bufferSend[0]);
			EndData();
			
			switch (PerPumpFourInfo[1]) {
				case 0x00: {
					USART3_puts("t4.txt=\"A\"");
					EndData();
					break;
				}
				case 0x01: {
					USART3_puts("t4.txt=\"B\"");
					EndData();
					break;
				}
				case 0x02: {
					USART3_puts("t4.txt=\"PH UP\"");
					EndData();
					break;
				}
				case 0x03: {
					USART3_puts("t4.txt=\"PH DOWN\"");
					EndData();
					break;
				}
			}
			reloadPage = false;
		}
		if (xSemaphoreTake(xSemaphoreHadData,1000) == pdTRUE) {
			if (IDHMI == 0) {
				PerPumpOneInfo[0] = DataHMI[0];
				PerPumpOneInfo[1] = DataHMI[1];
			}
			if (IDHMI == 1) {
				PerPumpTwoInfo[0] = DataHMI[0];
				PerPumpTwoInfo[1] = DataHMI[1];
			}
			if (IDHMI == 2) {
				PerPumpThreeInfo[0] = DataHMI[0];
				PerPumpThreeInfo[1] = DataHMI[1];
			}
			if (IDHMI == 3) {
				PerPumpFourInfo[0] = DataHMI[0];
				PerPumpFourInfo[1] = DataHMI[1];
			}
			xSemaphoreGive(xSemaphoreHandleData);
		}
	}
}
void pagePump(void *pvParameters) {
	char bufferSend[] = "";
	vTaskSuspend(NULL);
	while (1) {
		prevpage = PUMP;
		if (reloadPage == true) {
			sprintf(bufferSend,"n0.val=%d",PumpAcOneInfo[0]);
			USART3_puts(&bufferSend[0]);
			EndData();
			sprintf(bufferSend,"h1.val=%d",PumpAcOneInfo[0]);
			USART3_puts(&bufferSend[0]);
			EndData();
			switch (PumpAcOneInfo[1]) {
				case 0x01: {
					USART3_puts("t1.txt=\"OUT\"");
					EndData();
					break;
				}
				case 0x02: {
					USART3_puts("t2.txt=\"IN\"");
					EndData();
					break;
				}
			}
			
			sprintf(bufferSend,"n1.val=%d",PumpAcTwoInfo[0]);
			USART3_puts(&bufferSend[0]);
			EndData();
			sprintf(bufferSend,"h0.val=%d",PumpAcTwoInfo[0]);
			USART3_puts(&bufferSend[0]);
			EndData();
			switch (PumpAcTwoInfo[1]) {
				case 0x01: {
					USART3_puts("t1.txt=\"OUT\"");
					EndData();
					break;
				}
				case 0x02: {
					USART3_puts("t2.txt=\"IN\"");
					EndData();
					break;
				}
			}
			reloadPage = false;
		}
		if (xSemaphoreTake( xSemaphoreHadData,1000) == pdTRUE) {
			if (IDHMI == 0) {
				PumpAcOneInfo[0] = DataHMI[0];
				PumpAcOneInfo[1] = DataHMI[1];
			}
			if (IDHMI == 1) {
				PumpAcTwoInfo[0] = DataHMI[0];
				PumpAcTwoInfo[1] = DataHMI[1];
			}
			xSemaphoreGive(xSemaphoreHandleData);
		}
	}
}
void pagePump2(void *pvParameters) {
	char bufferSend[] = "";
	vTaskSuspend(NULL);
	while (1) {
		prevpage = PUMP2;
		if (reloadPage == true) {
			for (uint8_t i = 0;i<56;i++)
			{
				sprintf(bufferSend,"n%d.val=%d",i,TimeWaterthePlants[i]);
				USART3_puts(&bufferSend[0]);
				EndData();
			}
			reloadPage = false;
		}
		if (xSemaphoreTake( xSemaphoreHadData,1000) == pdTRUE) {
			if (IDHMI == 2) {
				memcpy(TimeWaterthePlants,DataHMI,56);
			}
			xSemaphoreGive(xSemaphoreHandleData);
		}
			
	}
}
void pagePump3(void *pvParameters) {
	char bufferSend[] = "";
	vTaskSuspend(NULL);
	while (1) {
		prevpage = PUMP3;
		if (reloadPage == true) {
			for (uint8_t i = 56;i<110;i++)
			{
				sprintf(bufferSend,"n%d.val=%d",i-56,TimeWaterthePlants[i]);
				USART3_puts(&bufferSend[0]);
				EndData();
			}
			reloadPage = false;
		}
		if (xSemaphoreTake( xSemaphoreHadData,1000) == pdTRUE) {
			if (IDHMI == 3) {
				for (uint8_t i = 56;i <111;i++)
				{
					TimeWaterthePlants[i] = DataHMI[i-56];
				}
			}
			xSemaphoreGive(xSemaphoreHandleData);
		}
			
	}
}
void pagePump4(void *pvParameters) {
	char bufferSend[] = "";
	vTaskSuspend(NULL);
	while (1) {
		prevpage = PUMP4;
		if (reloadPage == true) {
			if (DayWaterthePlants[0] == 0x01) {
				USART3_puts("b1.bco=1024");
				EndData();
			}
			if (DayWaterthePlants[1] == 0x02) {
				USART3_puts("b2.bco=1024");
				EndData();
			}
			if (DayWaterthePlants[2] == 0x03) {
				USART3_puts("b3.bco=1024");
				EndData();
			}
			if (DayWaterthePlants[3] == 0x04) {
				USART3_puts("b4.bco=1024");
				EndData();
			}
			if (DayWaterthePlants[4] == 0x05) {
				USART3_puts("b5.bco=1024");
				EndData();
			}
			if (DayWaterthePlants[5] == 0x06) {
				USART3_puts("b6.bco=1024");
				EndData();
			}
			if (DayWaterthePlants[6] == 0x07) {
				USART3_puts("b7.bco=1024");
				EndData();
			}
			reloadPage = false;
		}
		if (xSemaphoreTake( xSemaphoreHadData,1000) == pdTRUE) {
			if (IDHMI == 4) {
				memcpy(DayWaterthePlants,DataHMI,7);
			}
			xSemaphoreGive(xSemaphoreHandleData);
		}
	}
}
void pagePump5(void *pvParameters) {
	vTaskSuspend(NULL);
	while (1) {
		vTaskDelete(NULL);
	}
}
void pageGraph(void *pvParameters) {
	char bufferSend[] = "";
	uint8_t prevID = 10;
	uint16_t preNumGraph = 0;
	uint16_t NumGraph = 0;
	vTaskSuspend(NULL);
	while (1) {
		prevpage = GRAPH;
		if (reloadPage == true) {
			reloadPage = false;
			prevID = 10;
			preNumGraph = 0;
			NumGraph = 0;
			
			USART3_puts("b1.bco=1024");
			EndData();
			USART3_puts("b2.bco=50712");
			EndData();
			USART3_puts("b4.bco=50712");
			EndData();
			USART3_puts("b3.bco=50712");
			EndData();
			USART3_puts("cle 3,0");
			EndData();
			NumGraph = mystrlength(&GraphPH[0]);
			if (NumGraph > 0 ) {
				sprintf(bufferSend,"addt 19,0,%d",NumGraph);
				USART3_puts(&bufferSend[0]);
				EndData();
				for (int j = 0; j<NumGraph+1;j++) {
					USART3_puts(&GraphPPM[j]);
				}
			}
		}
		if (xSemaphoreTake( xSemaphoreHadData,1000) == pdTRUE) {
			if (IDHMI == 0x00 && DataHMI[0] == 0) {
				if (prevID != IDHMI ) {
					USART3_puts("b1.bco=1024");
					EndData();
					USART3_puts("b2.bco=50712");
					EndData();
					USART3_puts("b4.bco=50712");
					EndData();
					USART3_puts("b3.bco=50712");
					EndData();
					USART3_puts("cle 3,0");
					EndData();
					NumGraph = mystrlength(&GraphPH[0]);
					if (NumGraph > 0 ) {
						sprintf(bufferSend,"addt 19,0,%d",NumGraph);
						USART3_puts(&bufferSend[0]);
						EndData();
						for (int j = 0; j<NumGraph+1;j++) {
							USART3_puts(&GraphPPM[j]);
						}
					}
					prevID = IDHMI;
				}
			}
			else if (IDHMI == 0x01 && DataHMI[0] == 1) {
				if (prevID != IDHMI ) {
					USART3_puts("b2.bco=1024");
					EndData();
					USART3_puts("b1.bco=50712");
					EndData();
					USART3_puts("b4.bco=50712");
					EndData();
					USART3_puts("b3.bco=50712");
					EndData();
					USART3_puts("cle 3,0");
					EndData();
					NumGraph = mystrlength(&GraphTemp[0]);
					if (NumGraph > 0 ) {
						sprintf(bufferSend,"addt 19,0,%d",NumGraph);
						USART3_puts(&bufferSend[0]);
						EndData();
						for (int j = 0; j<NumGraph+1;j++) {
							USART3_puts(&GraphTemp[j]);
						}
					}
					prevID = IDHMI;
				}
			}	
			else if (IDHMI == 0x03 && DataHMI[0] == 3) {
				if (prevID != IDHMI ) {
					USART3_puts("b4.bco=1024");
					EndData();
					USART3_puts("b1.bco=50712");
					EndData();
					USART3_puts("b2.bco=50712");
					EndData();
					USART3_puts("b3.bco=50712");
					EndData();
					USART3_puts("cle 3,0");
					EndData();
					NumGraph = mystrlength(&GraphTank[0]);
					if (NumGraph > 0 ) {
						sprintf(bufferSend,"addt 19,0,%d",NumGraph);
						USART3_puts(&bufferSend[0]);
						EndData();
						for (int j = 0; j<NumGraph+1;j++) {
							USART3_puts(&GraphTank[j]);
						}
					}
					prevID = IDHMI;
				}
			}	
			else if (IDHMI == 0x04 && DataHMI[0] == 4) {
				if (prevID != IDHMI ) {
					USART3_puts("b3.bco=1024");
					EndData();
					USART3_puts("b1.bco=50712");
					EndData();
					USART3_puts("b2.bco=50712");
					EndData();
					USART3_puts("b4.bco=50712");
					EndData();
					USART3_puts("cle 3,0");
					EndData();
					NumGraph = mystrlength(&GraphPH[0]);
					if (NumGraph > 0 ) {
						sprintf(bufferSend,"addt 19,0,%d",NumGraph);
						USART3_puts(&bufferSend[0]);
						EndData();
						for (int j = 0; j<NumGraph+1;j++) {
							USART3_puts(&GraphPH[j]);
						}
					}
					prevID = IDHMI;
				}
			}	
			xSemaphoreGive(xSemaphoreHandleData);
		}
	}
}
void pageSetting(void *pvParameters) {
	char bufferSend[] = "";
	vTaskSuspend(NULL);
	while (1) {
		prevpage = SETTING;
		if (reloadPage == true) {
			reloadPage = false;
			if (RefillWhenLow == false) {
				USART3_puts("b1.bco=50712");
				EndData();
			}
			else {
				USART3_puts("b1.bco=1024");
				EndData();
			}
			sprintf(bufferSend,"va0.val=%d",DisplayTimeOut[0]);
			USART3_puts(&bufferSend[0]);
			EndData();
			sprintf(bufferSend,"n1.val=%d",DisplayTimeOut[1]);
			USART3_puts(&bufferSend[0]);
			EndData();
			sprintf(bufferSend,"n2.val=%d",DisplayTimeOut[2]);
			USART3_puts(&bufferSend[0]);
			EndData();
		}
		
		if (xSemaphoreTake( xSemaphoreHadData,1000) == pdTRUE) {
			if (IDHMI == 0 && DataHMI[0] == 0x01) {
				RefillWhenLow = !RefillWhenLow;
				if (RefillWhenLow) {
					USART3_puts("b1.bco=1024");
					EndData();
				}
				else {
					USART3_puts("b1.bco=50712");
					EndData();
				}
			}
			if (IDHMI == 4) {
				DisplayTimeOut[0] = (uint16_t) (uint16_t)((DataHMI[1]<<8) | DataHMI[0]);
				DisplayTimeOut[1] = (uint16_t) DataHMI[2];
				DisplayTimeOut[2] = (uint16_t) DataHMI[3];
			}
			if (IDHMI == 3 && DataHMI[0] == 0x01) {
				ResetALL = true;
			}
			xSemaphoreGive(xSemaphoreHandleData);
		}
	}
}
void pageSettingDate(void *pvParameters) {
	vTaskSuspend(NULL);
	while (1) {
		prevpage = SETTING_DATE;
		if (xSemaphoreTake( xSemaphoreHadData,1000) == pdTRUE) {
			if (IDHMI == 1) {
				rtc_setdatetime.day = DataHMI[0];
				rtc_setdatetime.date = DataHMI[1];
				rtc_setdatetime.month = DataHMI[2];
				rtc_setdatetime.year = DataHMI[3];
				rtc_setdatetime.hour = DataHMI[4];
				rtc_setdatetime.minutes = DataHMI[5];
				rtc_setdatetime.seconds = DataHMI[6];
				ds1307_init_rtc(1);
				ds1307_set_rtc_datetime(&rtc_setdatetime);
			}
			xSemaphoreGive(xSemaphoreHandleData);
		}
	}
}
void pageRefilldate(void *pvParameters) {
	vTaskSuspend(NULL);
	while (1) {
		prevpage = REFILL_DATE;
		if (xSemaphoreTake( xSemaphoreHadData,1000) == pdTRUE) {
			if (IDHMI == 2) {
				memcpy(RefillDates,DataHMI,31);
			}
			xSemaphoreGive(xSemaphoreHandleData);
		}
	}
}
void pageCal(void *pvParameters) {
	vTaskSuspend(NULL);
	while (1) {
		prevpage = CAL;
		vTaskDelay(4294967295);
	}
}
void pageCalpH(void *pvParameters) {
	char bufferSend[] = "";
	vTaskSuspend(NULL);
	while (1) {
		prevpage = CAL_PH;
		if (reloadPage == true) {
			sprintf(bufferSend,"x0.val=%d",(uint16_t)(sensorPHCalibration));
			USART3_puts(&bufferSend[0]);
			EndData();
			reloadPage = false;
		}
		sprintf(bufferSend,"t3.txt=\"%0.1f\"",data_sensor[1]);
		USART3_puts(&bufferSend[0]);
		EndData(); 	
		if (xSemaphoreTake( xSemaphoreHadData,400) == pdTRUE) {
			if (IDHMI == 0) {
				sensorPHCalibration = ((float)((DataHMI[1]<<8) | DataHMI[0]))/10;
			}
			xSemaphoreGive(xSemaphoreHandleData);
		}
		
	}
}
void pageCalPPM(void *pvParameters) {
	char bufferSend[] = "";
	vTaskSuspend(NULL);
	while (1) {
		prevpage = CAL_PPM;
		if (reloadPage == true) {
			sprintf(bufferSend,"n0.val=%d",(uint16_t)(sensorPPMCalibration));
			USART3_puts(&bufferSend[0]);
			EndData();
			reloadPage = false;
		}
		sprintf(bufferSend,"t3.txt=\"%0.1f\"",data_sensor[0]);
		USART3_puts(&bufferSend[0]);
		EndData(); 	
		if (xSemaphoreTake( xSemaphoreHadData,400) == pdTRUE) {
			if (IDHMI == 1) {
				sensorPPMCalibration = (uint16_t)((DataHMI[1]<<8) | DataHMI[0]);
			}
			xSemaphoreGive(xSemaphoreHandleData);
		}
		
	}
}
void pageCalTemp(void *pvParameters) {
	char bufferSend[] = "";
	vTaskSuspend(NULL);
	while (1) {
		prevpage = CAL_TEMP;
		if (reloadPage == true) {
			sprintf(bufferSend,"n0.val=%d",(uint16_t)(sensorTempCalibration));
			USART3_puts(&bufferSend[0]);
			EndData();
			reloadPage = false;
		}
		sprintf(bufferSend,"t3.txt=\"%0.1f\"",data_sensor[3]);
		USART3_puts(&bufferSend[0]);
		EndData(); 	
		if (xSemaphoreTake( xSemaphoreHadData,400) == pdTRUE) {
			if (IDHMI == 2) {
				sensorTempCalibration = (float)((DataHMI[1]<<8) | DataHMI[0]);
			}
			xSemaphoreGive(xSemaphoreHandleData);
		}
		
	}
}
void pageCalSonic(void *pvParameters) {
	char bufferSend[] = "";
	vTaskSuspend(NULL);
	while (1) {
		prevpage = CAL_SONIC;
		if (reloadPage == true) {
			sprintf(bufferSend,"n0.val=%d",(uint16_t)(sensorSonicCalibration));
			USART3_puts(&bufferSend[0]);
			EndData();
			reloadPage = false;
		}
		sprintf(bufferSend,"t3.txt=\"%0.1f\"",data_sensor[2]);
		USART3_puts(&bufferSend[0]);
		EndData(); 	
		if (xSemaphoreTake( xSemaphoreHadData,400) == pdTRUE) {
			if (IDHMI == 3) {
				sensorSonicCalibration = (uint16_t)((DataHMI[1]<<8) | DataHMI[0]);
			}
			xSemaphoreGive(xSemaphoreHandleData);
		}
		
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
	char bufferSend[] = "";
	vTaskSuspend(NULL);
	while (1) {
		prevpage = FERITILIZER;
		if (reloadPage == true) {
			sprintf(bufferSend,"n0.val=%d",Ratio[0]);
			USART3_puts(&bufferSend[0]);
			EndData();
			sprintf(bufferSend,"n1.val=%d",Ratio[1]);
			USART3_puts(&bufferSend[0]);
			EndData();
			reloadPage = false;
		}
		if (xSemaphoreTake( xSemaphoreHadData,1000) == pdTRUE) {
			if (IDHMI == 1) {
				Ratio[0] = (uint16_t) DataHMI[1]<<8 | DataHMI[0];
				Ratio[1] = (uint16_t) DataHMI[3]<<8 | DataHMI[2];
			}
			xSemaphoreGive(xSemaphoreHandleData);
		}
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
						//vTaskSuspend(TaskpageHome);
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
					case CAL:
					{
						vTaskSuspend(TaskpageCal);
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
					case CAL:
					{
						reloadPage = true;
						vTaskResume(TaskpageCal);
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
	TIM_Config();
	memset(DataHMIREV,'\0',LENGTH_DATA_HMI);
	while(1)
	{	
			if (xSemaphoreTake( xSemaphoreUARTRX,portMAX_DELAY) == pdTRUE ){
				if (FlagTC == TRUE){
					// Xu ly nhan byte dau tien
						switch (DataHMIREV[0]) {
							case BYTE_FIRST_START_PAGE: {
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
							case BYTE_FIRST_SEND_DATA: {
								if (mystrlength(DataHMIREV) >= 5 || page == PUMP2 || page == PUMP3) {
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
							default: {
									//typedata = UNKNOWN;
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
									if (page == PPM || page == PPM2 || page == PH || page == TANK || page == VALVE || page == VALVE2 || page == PERISTALTIC_PUMP ||
											page == PUMP || page == PUMP2 || page == PUMP3 || page == PUMP4 || page == SETTING || page == SETTING_DATE || 
											page == REFILL_DATE || page == CAL_PH || page == CAL_PPM || page == CAL_TEMP || page == CAL_SONIC || page == FERITILIZER ||
											page == GRAPH)
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
	uint8_t prevMinuteSaved = 0;
	
	
	
	uint16_t Temp = 0;
	uint32_t SumGraphPH = 0;
	uint32_t SumGraphPPM = 0;
	uint32_t SumGraphTEMP = 0;
	uint32_t SumGraphTank = 0;
	uint8_t prevHourDoser = 0;
	
	uint16_t countDataGraph = 0;
	ds1307_get_rtc_datetime(&rtc_getdatetime);
	prevHourDoser = rtc_getdatetime.hour;
	prevMinuteSaved = rtc_getdatetime.minutes;
	
	uint8_t lengArray = 0;
	while (1) {
		if (page != SETTING_DATE)
			ds1307_get_rtc_datetime(&rtc_getdatetime);
		
		if ((uint8_t)rtc_getdatetime.minutes - prevMinuteSaved >= 2) {
			xSemaphoreGive(SaveDataEvery3Minute);
			prevMinuteSaved = rtc_getdatetime.minutes;
		}
		if ((uint8_t)rtc_getdatetime.hour - prevHourDoser >= 4){
			xSemaphoreGive(xSemaphoredosingInterval);
			prevHourDoser = rtc_getdatetime.hour;
		}
		if (rtc_getdatetime.minutes >= 2 && rtc_getdatetime.hour == 0) {
			Temp = NumValSaved/10;
			for (uint16_t i = 0;i<Temp;i++) {
				for (uint16_t j = 0;j<30;j++) {
					SumGraphPH += sensorPHinDay[30*i+j];
					SumGraphPPM += sensorPPMinDay[30*i+j];
					SumGraphTEMP += sensorTEMPinDay[30*i+j];
					SumGraphTank += sensorTANKinDay[30*i+j];
				}
				GraphPH[countDataGraph] = SumGraphPH/30;
				SumGraphPH = 0;
				
				GraphPPM[countDataGraph] = SumGraphPPM/30;
				SumGraphPPM = 0;
				
				GraphTemp[countDataGraph] = SumGraphTEMP/30;
				SumGraphTEMP = 0;
				
				GraphTank[countDataGraph] = SumGraphTank/30;
				SumGraphTank = 0;
				
				countDataGraph++;
			}
			memset(sensorPHinDay,'\0',sizeof(sensorPHinDay));
			memset(sensorPPMinDay,'\0',sizeof(sensorPHinDay));
			memset(sensorTANKinDay,'\0',sizeof(sensorPHinDay));
			memset(sensorTEMPinDay,'\0',sizeof(sensorPHinDay));
			
			lengArray = mystrlength(&DayPPMStageOne[0]);
			for (uint8_t i =0;i<lengArray;i++) {
				if (DayPPMStageOne[i] == rtc_getdatetime.day) {
						targetTds = PPMStage[0];
				}
			}
			lengArray = mystrlength(&DayPPMStageTwo[0]);
			for (uint8_t i =0;i<lengArray;i++) {
				if (DayPPMStageTwo[i] == rtc_getdatetime.day) {
						targetTds = PPMStage[1];
				}
			}
			lengArray = mystrlength(&DayPPMStageThree[0]);
			for (uint8_t i =0;i<lengArray;i++) {
				if (DayPPMStageThree[i] == rtc_getdatetime.day) {
						targetTds = PPMStage[2];
				}
			}
			lengArray = mystrlength(&DayPPMStageFour[0]);
			for (uint8_t i =0;i<lengArray;i++) {
				if (DayPPMStageFour[i] == rtc_getdatetime.day) {
						targetTds = PPMStage[3];
				}
			}
			lengArray = mystrlength(&DayPPMStageFour[0]);
			for (uint8_t i =0;i<lengArray;i++) {
				if (DayPPMStageFour[i] == rtc_getdatetime.day) {
						targetTds = PPMStage[4];
				}
			}
		}
		if (rtc_getdatetime.hour > StartPump_hour && rtc_getdatetime.hour <= StopPump_hour && StartWaterPlant == false) {
			StartWaterPlant = true;
		}
		else 
				StartWaterPlant = false;
		if (rtc_getdatetime.hour == StartPump_hour && rtc_getdatetime.minutes >= StartPump_minute && StartWaterPlant == false) {
			StartWaterPlant = true;
		}
		else 
				StartWaterPlant = false;
		if (rtc_getdatetime.hour == StopPump_hour && rtc_getdatetime.minutes >= StopPump_minute && StartWaterPlant == true){
			StartWaterPlant = false;
		}
		
		vTaskDelay(400);
	}
}
void readSensor(void *pvParameters) {
	data_sensor[3]=0.0;
	
	uint8_t data_sonic_trigger = 0x55;
	uint16_t distance;
	
	float volt_cur=0.0;
	float val_con = 0.0; 
	float volt_cur_ph =0.0;
	
	uint16_t countSavedinDay = 0;
	
	unsigned long int sumSensorPHinDay = 0;
	unsigned long int sumSensorPPMinDay = 0;
	unsigned long int sumSensorTANKinDay = 0;
	unsigned long int sumSensorTEMPinDay = 0;
	
	frame datasend;
	
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
		
		USART_SendData(USART2,(uint16_t )data_sonic_trigger);
		/* Waiting the end of Data transfer */
		while (USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);    
		while (DMA_GetFlagStatus(DMA1_Stream5,DMA_FLAG_TCIF5)==RESET);
					 
		/* Clear DMA Transfer Complete Flags */
		DMA_ClearFlag(DMA1_Stream5,DMA_FLAG_TCIF5);
		/* Clear USART Transfer Complete Flags */
		USART_ClearFlag(USART2,USART_FLAG_TC);
		
		distance = (distance &0)| data_sonic[1];
		distance = (distance <<8)| data_sonic[2];
		data_sensor[2]=distance;
		
		volt_cur = ((float)uhADCxConvertedValue[1]*3)/4095;
		val_con = volt_cur/(1+(0.02*(data_sensor[3]-25)));
		data_sensor[0] = ((133.42*val_con*val_con*val_con)-(255.86*val_con*val_con)+(857.39*val_con))*0.5;
		
		volt_cur_ph =(((float)uhADCxConvertedValue[0]*3)/4095);
		data_sensor[1]=volt_cur_ph*3.5 + 1.2;
		
		sumSensorTEMPinDay += (unsigned long int)(data_sensor[3]*255/100);
		sumSensorTANKinDay += (unsigned long int)(data_sensor[2]*255/tankDepth);
		sumSensorPPMinDay	+= (unsigned long int)(data_sensor[0]*255/10000);
		sumSensorPHinDay += (unsigned long int)(data_sensor[1]*255/14);
		countSavedinDay++;
		
		if (xSemaphoreTake( SaveDataEvery3Minute,0) == pdTRUE) {
			sensorPHinDay[NumValSaved] = (uint8_t)(sumSensorPHinDay/countSavedinDay);
			sensorPPMinDay[NumValSaved] = (uint8_t)(sumSensorPPMinDay/countSavedinDay);
			sensorTANKinDay[NumValSaved] = (uint8_t)(sumSensorTANKinDay/countSavedinDay);
			sensorTEMPinDay[NumValSaved] = (uint8_t)(sumSensorTEMPinDay/countSavedinDay);
			NumValSaved++;
			countSavedinDay = 0;
		}
		
		datasend.PH = (double)data_sensor[1];
		datasend.tds = (double)data_sensor[0];
		datasend.sonic = (double)data_sensor[2];
		datasend.temperature= (double)data_sensor[3];
		
		memcpy(buffer,&datasend,sizeof(frame));
		
		xSemaphoreGive(xSemaphoresensorsReady);
		vTaskDelay(500/portTICK_PERIOD_MS);
		
	}
}
void I2C_ESP(void *pvParameters) {
	while (1) {
		I2C_DMACmd(I2C1, ENABLE);
		while (!I2C_CheckEvent(I2C1, I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED));
		/* Enable DMA RX Channel */
		DMA_Cmd(DMA1_Stream6, ENABLE);
		/* Wait until I2Cx_DMA_STREAM_TX enabled or time out */
		TimeOut = USER_TIMEOUT;
		while ((DMA_GetCmdStatus(DMA1_Stream6)!= ENABLE)&&(TimeOut != 0x00));
		/* Transfer complete or time out */
		TimeOut = USER_TIMEOUT;
		while ((DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6)==RESET)&&(TimeOut != 0x00));
		
		TimeOut = USER_TIMEOUT;
		while ((!I2C_CheckEvent(I2C1, I2C_EVENT_SLAVE_ACK_FAILURE))&&(TimeOut != 0x00));
		/* Disable DMA TX Channel */
		DMA_Cmd(DMA1_Stream6, DISABLE);
		/* Wait until I2Cx_DMA_STREAM_TX disabled or time out */
		TimeOut = USER_TIMEOUT;
		while ((DMA_GetCmdStatus(DMA1_Stream6)!= DISABLE)&&(TimeOut != 0x00));
		
		/* Disable I2C DMA request */  
		I2C_DMACmd(I2C1,DISABLE);
		
			/* Clear any pending flag on Tx Stream  */
		DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF6 | DMA_FLAG_FEIF6 | DMA_FLAG_DMEIF6 | 
																				 DMA_FLAG_TEIF6 | DMA_FLAG_HTIF6);
		vTaskDelay(1000);
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
void runPeristaltic(void *pvParameters) {
	dosingComplete = false;
	unsigned int ppmerror = 0;
	float pherror = 0;
	while (1) {
		if (xSemaphoreTake( xSemaphoredosingInterval,portMAX_DELAY) == pdTRUE ){
			if (xSemaphoreTake( xSemaphoresensorsReady,portMAX_DELAY) == pdTRUE ){
				while (StartWaterPlant == true || StartDraining == true );
				dosingComplete = false;
				while (dosingComplete == false) {
					ppmerror = targetTds - (unsigned int)data_sensor[0];
					if (ppmerror > tdsOffsetALL) {
						runPump = FRE_A;
						continue;
					}
					else 
						runPump = NOMODE;
					
					if (data_sensor[1] > targetMaxPh + phOffset || data_sensor[1] < targetMinPh - phOffset ) {
						if (data_sensor[1] > targetMaxPh + phOffset)
							runPump = PH_DOWN;
						if (data_sensor[1] < targetMinPh - phOffset)
							runPump = PH_UP;
					}
					else
						dosingComplete = true;
					vTaskDelay(10);
				}
			}
		}
	}
}
void DrainingWater(void *pvParameters) {
	StartDraining = false;
	//vTaskSuspend(NULL);
	while (1) {
		
		vTaskDelete(NULL);
	}
}
void WaterPlants(void *pvParameters) {
	GPIO_WriteBit(GPIOD,GPIO_Pin_0,Bit_RESET);
	GPIO_WriteBit(GPIOD,GPIO_Pin_1,Bit_RESET);
	//EXTI_GenerateSWInterrupt(EXTI_Line1);
	uint8_t nearhournow = 0;
	uint8_t location = 0;
	uint8_t trunggian0 = 0;
	uint8_t trunggian1 = 0;
	uint8_t trunggian2 = 0;
	uint8_t trunggian3 = 0;
	uint8_t tempArray[4][112] = {0};
	uint8_t countTimestamp = 0;
	bool TimeWrong = false;
	const uint16_t maxDelay = 7400;
	uint16_t SpeedtoDelay = 0;
	while (1) {
		if (StartWaterPlant == true ) {
			while (dosingComplete == false || StartDraining == true);
			GPIO_WriteBit(GPIOD,GPIO_Pin_0,Bit_SET);
			GPIO_WriteBit(GPIOD,GPIO_Pin_1,Bit_SET);
			while (StartWaterPlant == true) {
				if (xSemaphoreTake(xSemaphoreEXTIZCD,10) == pdTRUE ) {
					if (PumpAcOneInfo[1] == 0x01) {
						SpeedtoDelay = (PumpAcOneInfo[0]*maxDelay)/100;
						delayus(maxDelay-SpeedtoDelay);
						GPIO_WriteBit(GPIOD,GPIO_Pin_4,Bit_SET);
						vTaskDelay(1);
						GPIO_WriteBit(GPIOD,GPIO_Pin_4,Bit_RESET);
					}
					else if (PumpAcTwoInfo[1] == 0x01) {
						SpeedtoDelay = (PumpAcTwoInfo[0]*maxDelay)/100;
						delayus(maxDelay-SpeedtoDelay);
						GPIO_WriteBit(GPIOD,GPIO_Pin_5,Bit_SET);
						vTaskDelay(1);
						GPIO_WriteBit(GPIOD,GPIO_Pin_5,Bit_RESET);
					}
				}
			}
		}
		else {
			GPIO_WriteBit(GPIOD,GPIO_Pin_0,Bit_RESET);
			GPIO_WriteBit(GPIOD,GPIO_Pin_1,Bit_RESET);
		}
		for (uint8_t i = 0; i<112;i=i+4) {
			if (TimeWaterthePlants[i] != 0 || TimeWaterthePlants[i+1] != 0 ||
					TimeWaterthePlants[i+2] != 0 || TimeWaterthePlants[i+3] != 0 ) {
				
				if ( TimeWaterthePlants[i+2] >= TimeWaterthePlants[i]  ) {
					if (TimeWaterthePlants[i+3] >= TimeWaterthePlants[i+1]) {
						Timestamp[countTimestamp] = i;
						countTimestamp++;
					}
					else {
						TimeWrong = true;
					}
				}
				else
					TimeWrong = true;
			}
			else 
				TimeWrong = true;
			
			if (TimeWrong == true ) {
				TimeWrong = false;
				TimeWaterthePlants[i] = 0;
				TimeWaterthePlants[i+1] = 0;
				TimeWaterthePlants[i+2] = 0;
				TimeWaterthePlants[i+3] = 0;
			}
		}
		
		for (uint8_t i = 0;i<countTimestamp;i++) {
			tempArray[0][i] = TimeWaterthePlants[Timestamp[i]];
			tempArray[1][i] = TimeWaterthePlants[Timestamp[i]+1];
			tempArray[2][i] = TimeWaterthePlants[Timestamp[i]+2];
			tempArray[3][i] = TimeWaterthePlants[Timestamp[i]+3];
		}
		for (uint8_t i = 0;i<countTimestamp-1;i++) {
			for (uint8_t j = i+1;j<countTimestamp;j++) {
				if (tempArray[0][i] > tempArray[0][j]) {
					
					trunggian0 = tempArray[0][i];
					trunggian1 = tempArray[1][i];
					trunggian2 = tempArray[2][i];
					trunggian3 = tempArray[3][i];
					
					tempArray[0][i] = tempArray[0][j];
					tempArray[1][i] = tempArray[1][j];
					tempArray[2][i] = tempArray[2][j];
					tempArray[3][i] = tempArray[3][j];
					
					tempArray[0][j] = trunggian0;
					tempArray[1][j] = trunggian1;
					tempArray[2][j] = trunggian2;
					tempArray[3][j] = trunggian3;
				}
			}
		}
		for (uint8_t i = 0;i<countTimestamp;i++) {
			if (tempArray[0][i] - rtc_getdatetime.hour >=0) {
				StartPump_hour = tempArray[0][i] ;
				StartPump_minute = tempArray[1][i];
				StopPump_hour = tempArray[2][i];
				StopPump_minute = tempArray[3][i];
				break;
			}
			else {
				StartPump_hour = 25;
				StartPump_minute = 61;
				StopPump_hour = 25;
				StopPump_minute = 61;
			}
		}
		countTimestamp = 0;
		vTaskDelay(1000);
	}
}


