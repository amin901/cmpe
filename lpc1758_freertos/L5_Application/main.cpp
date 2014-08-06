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

/**
 * @file
 * @brief This is the application entry point.
 * 			FreeRTOS and stdio printf is pre-configured to use uart2_min.h before main() enters.
 * 			@see L0_LowLevel/lpc_sys.h if you wish to override printf/scanf functions.
 *
 * @note  printf of %f may be turned off to save memory, this can be configured at sys_config.h
 */
#include "tasks.hpp"
#include "utilities.h"
#include "examples/examples.hpp"
#include "io.hpp"
#include "stdio.h"
#include "uart0_min.h"
#include "ProducerConsumer.hpp"
#include "semphr.h"
#include "eint.h"
#include "soft_timer.hpp"
#include "uart2.hpp"
#include "gps_task.hpp"

#include "ExtraCredit.hpp"

#include "Bluetooth.hpp"


//void task_one(void *p);
//void task_two(void *p);
//
//Uart2 &u2 = Uart2::getInstance();
//
//void task();
//SoftTimer timer(1000);
//void task_take_semaphore(void *p);
//
//xSemaphoreHandle switchSignal = 0;
//
//const unsigned char temp = 0;

/**
 * The main() creates tasks or "threads".  See the documentation of scheduler_task class at cpp_task.hpp
 * for details.  There is a very simple example towards the beginning of this class's declaration.
 *
 * @warning SPI #1 bus usage notes (interfaced to SD & Flash):
 *      - You can read/write files from multiple tasks because it automatically goes through SPI semaphore.
 *      - If you are going to use the SPI Bus in a FreeRTOS task, you need to use the API at L4_IO/fat/spi_sem.h
 *
 * @warning SPI #0 usage notes (Nordic wireless)
 *      - This bus is more tricky to use because if FreeRTOS is not running, the RIT interrupt may use the bus.
 *      - If FreeRTOS is running, then wireless task may use it.
 *        In either case, you should avoid using this bus or interfacing to external components because
 *        there is no semaphore configured for this bus and it should be used exclusively by nordic wireless.
 */
int main(void)
{

//    vSemaphoreCreateBinary(switchSignal);
//    LPC_GPIO2->FIODIR &= ~(1<<0); // input configuration for port 2.0
//    LPC_PINCON->PINMODE4 |= (3<<0);
//    LPC_GPIOINT->IO2IntEnR &= ~(1<<0); // reset
//    LPC_GPIOINT->IO2IntEnR |= (1<<0); // set port 2.0 for rising edge interrupt
//
//    eint3_enable_port2(temp,eint_rising_edge,task); // enabling port2 eint.h





    /**
     * A few basic tasks for this bare-bone system :
     *      1.  Terminal task provides gateway to interact with the board through UART terminal.
     *      2.  Remote task allows you to use remote control to interact with the board.
     *      3.  Wireless task responsible to receive, retry, and handle mesh network.
     *
     * Disable remote task if you are not using it.  Also, it needs ENABLE_TELEMETRY
     * such that it can save remote control codes to non-volatile memory.  IR remote
     * control codes can be learned by typing "learn" command.
     */
    scheduler_add_task(new terminalTask(PRIORITY_HIGH));
    scheduler_add_task(new remoteTask  (PRIORITY_LOW));

    /* Consumes very little CPU, but need highest priority to handle mesh network ACKs */
    scheduler_add_task(new wirelessTask(PRIORITY_CRITICAL));


#if 0
    scheduler_add_task(new Bluetooth(PRIORITY_LOW));
#endif

#if 0
    scheduler_add_task(new gps_task(PRIORITY_LOW));
#endif


#if 0

    xTaskCreate(task_take_semaphore,(signed char *)"take",512,NULL,1,NULL);

#endif

#if 0
    scheduler_add_task(new taskOne(PRIORITY_LOW));
    scheduler_add_task(new taskTwo(PRIORITY_LOW));
#endif


    #if 0
    scheduler_add_task(new orient_compute(PRIORITY_LOW));
    scheduler_add_task(new orient_process(PRIORITY_LOW));
    #endif

    /* Your tasks should probably used PRIORITY_MEDIUM or PRIORITY_LOW because you
     * want the terminal task to always be responsive so you can poke around in
     * case something goes wrong.
     */
    #if 0
    xTaskCreate(task_one,(signed char *)"task one",512,NULL,1,NULL);
    xTaskCreate(task_two,(signed char *)"task two",512,NULL,1,NULL);
    #endif

    #if 0
    scheduler_add_task(new i2c_hw3(PRIORITY_LOW));
    #endif

    #if 0
    scheduler_add_task(new spi1_hw2(PRIORITY_LOW));
    #endif

    #if 0
    scheduler_add_task(new uart2_hw1(PRIORITY_LOW));
    #endif

    #if 0
    scheduler_add_task(new Led_Switch(PRIORITY_LOW));
    #endif
    /**
     * This is a the board demonstration task that can be used to test the board.
     * This also shows you how to send a wireless packets to other boards.
     */
    #if 0
        scheduler_add_task(new example_io_demo());
    #endif

    /**
     * Change "#if 0" to "#if 1" to enable examples.
     * Try these examples one at a time.
     */
    #if 0
        scheduler_add_task(new example_task());
        scheduler_add_task(new example_alarm());
        scheduler_add_task(new example_logger_qset());
        scheduler_add_task(new example_nv_vars());
    #endif

    /**
	 * Try the rx / tx tasks together to see how they queue data to each other.
	 */
    #if 0
        scheduler_add_task(new queue_tx());
        scheduler_add_task(new queue_rx());
    #endif

    /**
     * If you have RN-XV on your board, you can connect to Wifi using this task.
     * This task will also let you perform an HTTP web request.  See wifiTask
     * documentation for details.
     */
    #if 0
        Uart3::getInstance().init(WIFI_BAUD_RATE, WIFI_RXQ_SIZE, WIFI_TXQ_SIZE);
        scheduler_add_task(new wifiTask(Uart3::getInstance(), PRIORITY_LOW));
    #endif

    scheduler_start(false); /** This shouldn't return */
    return -1;
}

//void task()
//{
//
//    if(timer.expired())
//    {
//
//    xSemaphoreGiveFromISR(switchSignal,NULL);
//    //timer.restart();
//   }
//
//}




//void task_take_semaphore(void *p)
//{
//    while (1)
//    {
//        xSemaphoreTake(switchSignal,portMAX_DELAY);
//        puts("Semaphore given...");
//        timer.reset();
//    }
//
//}
//
//
//
//
//
//void task_one(void *p)
//{
//
//    while(1)
//    {
//        //uart0_puts("aaaaaaaaaaaaaa");
//        vTaskDelay(1000);
//    }
//}
//
//void task_two(void *p)
//{
//    while(1)
//    {
//        uart0_puts("bbbbbbbbbbbbbb");
//        vTaskDelay(1000);
//    }
//}
