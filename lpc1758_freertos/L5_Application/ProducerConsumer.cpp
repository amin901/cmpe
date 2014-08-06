/*
 * ProducerConsumer.cpp
 *
 *  Created on: Mar 15, 2014
 *      Author: Arpit
 */


#include "ProducerConsumer.hpp"
#include "uart0_min.h"
#include "queue.h"
#include "io.hpp"
#include "stdio.h"

xQueueHandle myQueue = 0;

typedef enum
{
    QueueID,
} sharedHandleID;

typedef enum
        {
            noMovement=0,
            left = 1,
            right = 2,
            up  = 3,
            down = 4,

        }orientation;

        orientation orientation_t = noMovement;

orient_compute::orient_compute(uint8_t priority) : scheduler_task("compute", 512, priority)
{
    myQueue = xQueueCreate(1, sizeof(orientation));
   //addSharedObject(QueueID, myQueue);

}

bool orient_compute::orient_compute_init()
{
    //myQueue = xQueueCreate(1, sizeof(int));
    //addSharedObject("queue id", myQueue);
    return 1;
}

bool orient_compute::run(void *p)
{
      orientation_t = noMovement;

      printf("Task one BEFORE\n");
      xQueueSend(myQueue, &orientation_t, 1000);
      printf("Task one AFTER\n");


    vTaskDelay(1000);

    return 1;
}

bool orient_compute::taskEntry(void)
{
    //setRunDuration(1000);
    return 1;
}

orient_process::orient_process(uint8_t priority) : scheduler_task("process", 512, priority)
{

}

bool orient_process::run(void *p)
{

    if( xQueueReceive(myQueue,&orientation_t,10000))
    {
     printf("Task 2\n");

    }

    //vTaskDelay(500);
    return 1;
}

bool orient_process::taskEntry(void)
{
    //setRunDuration(1000);
    return 1;
}



