/*
 * FreeRTOS Kernel V10.2.1
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#if defined(__ADSPSC589_FAMILY__)
#include <defSC589.h>
#elif defined(__ADSPSC594_FAMILY__)
#include <defSC594.h>
#elif defined(__ADSPSC598_FAMILY__)
#include <defSC598.h>
#else
#error Unsupported processor!
#endif

#include "clocks.h"


// New stuff
#if 0
#define configUSE_TASK_NOTIFICATIONS            1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES   3
#define configUSE_TIME_SLICING                  1
#define configUSE_NEWLIB_REENTRANT              0
#define configENABLE_BACKWARD_COMPATIBILITY     0
#define configSTACK_DEPTH_TYPE                  uint16_t
#define configMESSAGE_BUFFER_LENGTH_TYPE        size_t
#define configSUPPORT_DYNAMIC_ALLOCATION            1
#define configSTACK_ALLOCATION_FROM_SEPARATE_HEAP   1
#define configUSE_DAEMON_TASK_STARTUP_HOOK      0
#define configKERNEL_INTERRUPT_PRIORITY         18
#define configMAX_API_CALL_INTERRUPT_PRIORITY   18
/* FreeRTOS MPU specific definitions. */
#define configINCLUDE_APPLICATION_DEFINED_PRIVILEGED_FUNCTIONS 0
#define configTOTAL_MPU_REGIONS                                8 /* Default value. */
#define configTEX_S_C_B_FLASH                                  0x07UL /* Default value. */
#define configTEX_S_C_B_SRAM                                   0x07UL /* Default value. */
#define configENFORCE_SYSTEM_CALLS_FROM_KERNEL_ONLY            1
/* ARMv8-M secure side port related definitions. */
#define secureconfigMAX_SECURE_CONTEXTS         5
#define INCLUDE_xResumeFromISR                  1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_xEventGroupSetBitFromISR        1
#define INCLUDE_xTaskAbortDelay                 0
#define INCLUDE_xTaskGetHandle                  0
#define INCLUDE_xTaskResumeFromISR              1
/* If configTASK_RETURN_ADDRESS is not defined then a task that attempts to
return from its implementing function will end up in a "task exit error"
function - which contains a call to configASSERT().  However this can give GCC
some problems when it tries to unwind the stack, as the exit error function has
nothing to return to.  To avoid this define configTASK_RETURN_ADDRESS to 0.  */
#define configTASK_RETURN_ADDRESS   NULL
#endif

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * See http://www.freertos.org/a00110.html.
 *----------------------------------------------------------*/

/*
 * The FreeRTOS Cortex-A port implements a full interrupt nesting model.
 *
 * Interrupts that are assigned a priority at or below
 * configMAX_API_CALL_INTERRUPT_PRIORITY (which counter-intuitively in the ARM
 * generic interrupt controller [GIC] means a priority that has a numerical
 * value above configMAX_API_CALL_INTERRUPT_PRIORITY) can call FreeRTOS safe API
 * functions and will nest.
 *
 * Interrupts that are assigned a priority above
 * configMAX_API_CALL_INTERRUPT_PRIORITY (which in the GIC means a numerical
 * value below configMAX_API_CALL_INTERRUPT_PRIORITY) cannot call any FreeRTOS
 * API functions, will nest, and will not be masked by FreeRTOS critical
 * sections (although it is necessary for interrupts to be globally disabled
 * extremely briefly as the interrupt mask is updated in the GIC).
 *
 * FreeRTOS functions that can be called from an interrupt are those that end in
 * "FromISR".  FreeRTOS maintains a separate interrupt safe API to enable
 * interrupt entry to be shorter, faster, simpler and smaller.
 *
 * The ADSP-SC5xx Generic Interrupt Controller 32 unique interrupt priorities.  For the
 * purpose of setting configMAX_API_CALL_INTERRUPT_PRIORITY 32 represents the
 * lowest priority.
 */
#define configMAX_API_CALL_INTERRUPT_PRIORITY   18

/* The application will define the array used as the RTOS heap to ensure it can
be located in the (faster) on-chip RAM.  When this parameter is set to 1 the
application must define an array using the name and size as follows below, but
is free to locate the array in any suitable RAM region (the faster the better as
the stacks used by the tasks are allocated from this array):
*/
#define configAPPLICATION_ALLOCATED_HEAP        0
#define configSUPPORT_STATIC_ALLOCATION         0
#define configCPU_CLOCK_HZ                      CCLK
#define configSCLK0_HZ                          SCLK0
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#define configUSE_TICKLESS_IDLE                 0
#define configTICK_RATE_HZ                      ( ( TickType_t ) 1000 )
#define configUSE_PREEMPTION                    1
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configMAX_PRIORITIES                    ( 7 )
#define configMINIMAL_STACK_SIZE                ( ( unsigned short ) (512 + 64) )
#define configTOTAL_HEAP_SIZE                   ( 512 * 1024 )
#define configMAX_TASK_NAME_LEN                 ( 32 )
#define configUSE_TRACE_FACILITY                0
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_MUTEXES                       1
#define configQUEUE_REGISTRY_SIZE               8
#define configCHECK_FOR_STACK_OVERFLOW          2
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_MALLOC_FAILED_HOOK            1
#define configUSE_APPLICATION_TASK_TAG          0
#define configUSE_COUNTING_SEMAPHORES           1
#define configUSE_QUEUE_SETS                    1
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS 16

/*
 * Application and minimal OSAL TLS defines (not used by FreeRTOS).
 * The minimal OSAL expects its TLS pointers to start at
 * zero.
 */
#define configNUM_OSAL_TLS_POINTERS \
    (configNUM_THREAD_LOCAL_STORAGE_POINTERS / 2)
#define configNUM_APP_TLS_POINTERS \
    (configNUM_THREAD_LOCAL_STORAGE_POINTERS - configNUM_OSAL_TLS_POINTERS)
#define configSTART_APP_TLS_POINTERS \
    (configNUM_OSAL_TLS_POINTERS)

/* Pre-defined application TLS pointers */
#define configSTDIO_APP_TLS_POINTER (configSTART_APP_TLS_POINTERS + 0)

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES                   0
#define configMAX_CO_ROUTINE_PRIORITIES         ( 2 )

/* Software timer definitions. */
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               ( configMAX_PRIORITIES - 1 )
#define configTIMER_QUEUE_LENGTH                5
#define configTIMER_TASK_STACK_DEPTH            ( configMINIMAL_STACK_SIZE * 2 )

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskCleanUpResources           1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTimerPendFunctionCall          1
#define INCLUDE_eTaskGetState                   1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
#define INCLUDE_xTaskGetIdleTaskHandle          1

/* Task switch hook for idle time calculations */
void taskSwitchHook(void *taskHandle);
#define traceTASK_SWITCHED_IN()  taskSwitchHook(pxCurrentTCB)

/* Enable FPU context support in all tasks.  This also config option also
 * enables FPU context support in the ISRs if configUSE_TASK_FPU_SUPPORT > 0.
 * To force disabling ISR FPU context support, define "configNO_ISR_FPU_SUPPORT"  */
#define configUSE_TASK_FPU_SUPPORT              2

/* This demo makes use of one or more example stats formatting functions.  These
format the raw data provided by the uxTaskGetSystemState() function in to human
readable ASCII form.  See the notes in the implementation of vTaskList() within
FreeRTOS/Source/tasks.c for limitations. */
#define configUSE_STATS_FORMATTING_FUNCTIONS    1

/* Run time stats related definitions. */
#define configGENERATE_RUN_TIME_STATS 0
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()
#define portGET_RUN_TIME_COUNTER_VALUE()  ( ( uint32_t ) ( alt_globaltmr_get64() >> ( uint64_t ) 16 ) )


/* The size of the global output buffer that is available for use when there
are multiple command interpreters running at once (for example, one on a UART
and one on TCP/IP).  This is done to prevent an output buffer being defined by
each implementation - which would waste RAM.  In this case, there is only one
command interpreter running. */
#define configCOMMAND_INT_MAX_OUTPUT_SIZE 2096

/* Normal assert() semantics without relying on the provision of an assert.h
header file. */
void vAssertCalled( const char * pcFile, unsigned long ulLine );
#define configASSERT( x ) if( ( x ) == 0 ) vAssertCalled( __FILE__, __LINE__ );

/* If configTASK_RETURN_ADDRESS is not defined then a task that attempts to
return from its implementing function will end up in a "task exit error"
function - which contains a call to configASSERT().  However this can give GCC
some problems when it tries to unwind the stack, as the exit error function has
nothing to return to.  To avoid this define configTASK_RETURN_ADDRESS to 0.  */
#define configTASK_RETURN_ADDRESS   NULL


/****** Hardware specific settings. *******************************************/
void vApplicationIRQHandler( uint32_t ulICCIAR );

/*
 * The application must provide a function that configures a peripheral to
 * create the FreeRTOS tick interrupt, then define configSETUP_TICK_INTERRUPT()
 * in FreeRTOSConfig.h to call the function.  This file contains a function
 * that is suitable for use on the Zynq MPU.  FreeRTOS_Tick_Handler() must
 * be installed as the peripheral's interrupt handler.
 */
void vConfigureTickInterrupt( void );
#define configSETUP_TICK_INTERRUPT() vConfigureTickInterrupt();

void vClearTickInterrupt( void );

/* The following constants describe the hardware, and are correct for the ADSP-SC58x */
#define configINTERRUPT_CONTROLLER_BASE_ADDRESS         ( REG_GICDST0_EN )
#define configINTERRUPT_CONTROLLER_CPU_INTERFACE_OFFSET ( REG_GICCPU1_CTL - REG_GICDST0_EN )
#define configUNIQUE_INTERRUPT_PRIORITIES               32

#endif /* FREERTOS_CONFIG_H */

