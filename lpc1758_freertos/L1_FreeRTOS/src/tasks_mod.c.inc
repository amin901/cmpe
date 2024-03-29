/*
 *     SocialLedge.com - Copyright (C) 2013
 *
 *     This file is part of free software framework for embedded processors.
 *     You can use it and/or distribute it as long as this copyright header
 *     remains unmodified.  The code is free for personal use and requires
 *     permission to use in a commercial product.
 *
 *      THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *      OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *      MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *      I SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 *      CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *     You can reach the author of this software at :
 *          p r e e t . w i k i @ g m a i l . c o m
 */

#if (1 == configGENERATE_RUN_TIME_STATS)
unsigned portBASE_TYPE uxTaskGetCpuUsage(xTaskHandle xTask)
{
    tskTCB *pxTCB = prvGetTCBFromHandle( xTask );
    unsigned long ulTotalRunTime;
#ifdef portALT_GET_RUN_TIME_COUNTER_VALUE
    portALT_GET_RUN_TIME_COUNTER_VALUE( ulTotalRunTime );
#else
    ulTotalRunTime = portGET_RUN_TIME_COUNTER_VALUE();
#endif

    return (pxTCB->ulRunTimeCounter * 100) / ulTotalRunTime;
}

static void prvResetRunTimeStatsInList(xList *pxList)
{
    volatile tskTCB *pxNextTCB, *pxFirstTCB;

    listGET_OWNER_OF_NEXT_ENTRY( pxFirstTCB, pxList );
    do
    {
        listGET_OWNER_OF_NEXT_ENTRY( pxNextTCB, pxList );
        pxNextTCB->ulRunTimeCounter = 0;
    } while( pxNextTCB != pxFirstTCB );
}

void vTaskResetRunTimeStats(void)
{
    unsigned portBASE_TYPE uxQueue = configMAX_PRIORITIES;
    vTaskSuspendAll();
    {
        do
        {
            uxQueue--;
            if (listLIST_IS_EMPTY(&(pxReadyTasksLists[uxQueue])) == pdFALSE)
            {
                prvResetRunTimeStatsInList(
                        (xList *) &(pxReadyTasksLists[uxQueue]));
            }
        } while (uxQueue > ( unsigned portBASE_TYPE ) tskIDLE_PRIORITY);

        if (listLIST_IS_EMPTY(pxDelayedTaskList) == pdFALSE)
        {
            prvResetRunTimeStatsInList((xList *) pxDelayedTaskList);
        }

        if (listLIST_IS_EMPTY(pxOverflowDelayedTaskList) == pdFALSE)
        {
            prvResetRunTimeStatsInList((xList *) pxOverflowDelayedTaskList);
        }

#if ( INCLUDE_vTaskDelete == 1 )
        {
            if( listLIST_IS_EMPTY( &xTasksWaitingTermination ) == pdFALSE )
            {
                prvResetRunTimeStatsInList(( xList * ) &xTasksWaitingTermination );
            }
        }
#endif

#if ( INCLUDE_vTaskSuspend == 1 )
        {
            if( listLIST_IS_EMPTY( &xSuspendedTaskList ) == pdFALSE )
            {
                prvResetRunTimeStatsInList(( xList * ) &xSuspendedTaskList );
            }
        }
#endif
        ulTaskSwitchedInTime = 0;
        portRESET_TIMER_FOR_RUN_TIME_STATS();
    }
    xTaskResumeAll();
}
#endif