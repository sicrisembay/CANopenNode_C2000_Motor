/*
 * FreeRTOSConfig.h
 *
 *  Created on: 4 Aug 2024
 *      Author: Sicris
 */

#ifndef FREERTOSCONFIG_H_
#define FREERTOSCONFIG_H_

#include "stdint.h"
#include "stdbool.h"

//--------------------------------------------------------------------------------------------------
// Application specific definitions.
//
// These definitions should be adjusted for your particular hardware and
// application requirements.
//
// THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
// FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
//
// See http://www.freertos.org/a00110.html.
//--------------------------------------------------------------------------------------------------

#define configUSE_PREEMPTION                1
#define configUSE_IDLE_HOOK                 1
#define configUSE_TICK_HOOK                 1
#define configCPU_CLOCK_HZ                  ( ( unsigned long ) 150000000 )
#define configTICK_RATE_HZ                  ( ( TickType_t ) 1000 )
#define configMAX_PRIORITIES                ( 32 )
#define configMINIMAL_STACK_SIZE            ( ( unsigned short ) 512 )
#define configTOTAL_HEAP_SIZE               ( ( size_t ) ( 4 * 1024 ) )
#define configMAX_TASK_NAME_LEN             ( 16 )
#define configUSE_TRACE_FACILITY            0
#define configUSE_16_BIT_TICKS              0
#define configIDLE_SHOULD_YIELD             0
#define configCHECK_FOR_STACK_OVERFLOW      0
#define configSUPPORT_STATIC_ALLOCATION     1
#define configSUPPORT_DYNAMIC_ALLOCATION    0
#define configUSE_MUTEXES                   1
#define configUSE_RECURSIVE_MUTEXES         1

// Set the following definitions to 1 to include the API function, or zero
// to exclude the API function.

#define INCLUDE_vTaskPrioritySet            0
#define INCLUDE_uxTaskPriorityGet           0
#define INCLUDE_vTaskDelete                 0
#define INCLUDE_vTaskCleanUpResources       0
#define INCLUDE_vTaskSuspend                0
#define INCLUDE_vTaskDelayUntil             1
#define INCLUDE_vTaskDelay                  1
#define INCLUDE_xTaskGetCurrentTaskHandle   1

extern void FREERTOS_configAssert(const bool cond, char const * module, const uint16_t line);

#include "string.h"
#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#define configASSERT(x)     FREERTOS_configAssert(x, __FILENAME__, __LINE__)

#endif /* FREERTOSCONFIG_H_ */
