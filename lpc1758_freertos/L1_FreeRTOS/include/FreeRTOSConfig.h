/*
    FreeRTOS V7.0.1 - Copyright (C) 2011 Real Time Engineers Ltd.
	

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS tutorial books are available in pdf and paperback.        *
     *    Complete, revised, and edited pdf reference manuals are also       *
     *    available.                                                         *
     *                                                                       *
     *    Purchasing FreeRTOS documentation will not only help you, by       *
     *    ensuring you get running as quickly as possible and with an        *
     *    in-depth knowledge of how to use FreeRTOS, it will also help       *
     *    the FreeRTOS project to continue with its mission of providing     *
     *    professional grade, cross platform, de facto standard solutions    *
     *    for microcontrollers - completely free of charge!                  *
     *                                                                       *
     *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
     *                                                                       *
     *    Thank you for using FreeRTOS, and thank you for your support!      *
     *                                                                       *
    ***************************************************************************


    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    >>>NOTE<<< The modification to the GPL is included to allow you to
    distribute a combined work that includes FreeRTOS without being obliged to
    provide the source code for proprietary components outside of the FreeRTOS
    kernel.  FreeRTOS is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.

    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.

    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
*/

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H



/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *----------------------------------------------------------*/
#include "sys_config.h"
#define configUSE_PREEMPTION		    1
#define configUSE_IDLE_HOOK			    1
#define configUSE_TICK_HOOK 		    1
#define configUSE_MALLOC_FAILED_HOOK    1
#define configCPU_CLOCK_HZ			    (DESIRED_CPU_CLOCK)
#define configTICK_RATE_HZ			    ( 1000 )
#define configMAX_PRIORITIES			( 4 )

#if   (configMAX_PRIORITIES == 2)
    #define PRIORITY_IDLE       0
    #define PRIORITY_LOW        0
    #define PRIORITY_HIGH       1
#elif (configMAX_PRIORITIES == 3)
    #define PRIORITY_IDLE		0
    #define PRIORITY_LOW		0
    #define PRIORITY_MEDIUM		1
    #define PRIORITY_HIGH		2
#elif (configMAX_PRIORITIES == 4)
    #define PRIORITY_IDLE       0
    #define PRIORITY_LOW        0
    #define PRIORITY_MEDIUM     1
    #define PRIORITY_HIGH       2
    #define PRIORITY_CRITICAL 	3
#else
    #error "You should really not need more than 4 priorities.  Consider using 4 or less, or override this error message"
#endif

/**
 * @{ FreeRTOS Memory configuration
 * 1 - Get from the pool defined by configTOTAL_HEAP_SIZE.  No free()
 * 2 - Get from the pool defined by configTOTAL_HEAP_SIZE with free()
 * 3 - Just redirect FreeRTOS memory to malloc() and free()
 * 4 - Same as 2, but coalescencent blocks can be combined.
 *
 * configTOTAL_HEAP_SIZE only matters when scheme 1, 2 or 4 is used above.
 */
#define configMEM_MANG_TYPE             3
#define configTOTAL_HEAP_SIZE           ( ( size_t ) ( 24 * 1024 ) )
/** @} */

/* Stack size and utility functions */
#define configMINIMAL_STACK_SIZE		( ( unsigned short ) 128 )	 /* Do not change this */
#define STACK_BYTES(x)					((x)/4)	/* freeRTOS allocates 4-times the size given due to 32-bit ARM */
#define MS_PER_TICK()					( 1000 / configTICK_RATE_HZ)
#define OS_MS(x)						( x / MS_PER_TICK() )
#define vTaskDelayMs(x)                 vTaskDelay(OS_MS((x)))
#define xTaskGetMsCount()               (xTaskGetTickCount() * MS_PER_TICK())

/* General config */
#define configMAX_TASK_NAME_LEN		    ( 8 )
#define configUSE_16_BIT_TICKS          0
#define configIDLE_SHOULD_YIELD         1
#define configCHECK_FOR_STACK_OVERFLOW  2
#define configUSE_ALTERNATIVE_API       0
#define configQUEUE_REGISTRY_SIZE       0

/* FreeRTOS Co-routine */
#define configUSE_CO_ROUTINES 		    0
#define configMAX_CO_ROUTINE_PRIORITIES ( 1 )

/* FreeRTOS Timer */
#define configUSE_TIMERS				0
#define configTIMER_TASK_PRIORITY		PRIORITY_HIGH
#define configTIMER_QUEUE_LENGTH		10
#define configTIMER_TASK_STACK_DEPTH    STACK_BYTES(2048)

/* Run time and task stats gathering related definitions. */
#define configGENERATE_RUN_TIME_STATS           1
#define configUSE_TRACE_FACILITY                1
#define configUSE_STATS_FORMATTING_FUNCTIONS    0

/* Features config */
#define configUSE_MUTEXES                   1
#define configUSE_RECURSIVE_MUTEXES         0
#define configUSE_COUNTING_SEMAPHORES       1
#define configUSE_QUEUE_SETS                1
#define INCLUDE_vTaskPrioritySet			0
#define INCLUDE_uxTaskPriorityGet			0
#define INCLUDE_vTaskDelete					0
#define INCLUDE_vTaskCleanUpResources		0
#define INCLUDE_vTaskSuspend				1
#define INCLUDE_vTaskDelayUntil				1
#define INCLUDE_vTaskDelay					1
#define INCLUDE_uxTaskGetStackHighWaterMark	1
#define INCLUDE_xTaskGetSchedulerState      1
#define INCLUDE_xTaskGetIdleTaskHandle      1






/* ARM Cortex M3 has hardware instruction to count leading zeroes */
#define configUSE_PORT_OPTIMISED_TASK_SELECTION    1

/* Use the system definition, if there is one */
#ifdef __NVIC_PRIO_BITS
	#define configPRIO_BITS       __NVIC_PRIO_BITS
#else
	#define configPRIO_BITS       5        /* 32 priority levels */
#endif

#include "isr_priorities.h"
/* The lowest priority. */
#define configKERNEL_INTERRUPT_PRIORITY 	    ( IP_KERNEL <<   (8 - configPRIO_BITS) )
/* Priority 5, or 160 as only the top three bits are implemented. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	( IP_SYSCALL <<  (8 - configPRIO_BITS) )


/*-----------------------------------------------------------
 * Macros required to setup the timer for the run time stats.
 *-----------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif
    void vConfigureTimerForRunTimeStats( void );
    unsigned int uxGetTimerForRunTimeStats();
    void resetRunTimeCounter();
#ifdef __cplusplus
}
#endif

#define portGET_RUN_TIME_COUNTER_VALUE() 			uxGetTimerForRunTimeStats()
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()	vConfigureTimerForRunTimeStats()
#define portRESET_TIMER_FOR_RUN_TIME_STATS()        resetRunTimeCounter()


#endif /* FREERTOS_CONFIG_H */
