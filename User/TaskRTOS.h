#ifndef __TASKRTOS_H
#define __TASKRTOS_H

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

void HandleUART(void *pvParameters);

SemaphoreHandle_t xSemaphore = NULL; // Semaphore UART from ISR

#endif