/*
 * uart2_hw1.cpp
 *
 *  Created on: Feb 15, 2014
 *      Author: Arpit
 */


#include "FreeRTOS.h"
#include "sys_config.h"
#include "tasks.hpp"
#include "stdio.h"

uart2_hw1::uart2_hw1(uint8_t priority) :
        scheduler_task("remote", 512, priority)
{

}

bool uart2_hw1::uart2_hw1_init()
{

    uint8_t divider = sys_get_cpu_clock() /(16*9600) + 0.5;
    LPC_SC->PCONP |= (1<<24); // power on to uart 2
    LPC_SC->PCLKSEL1 &= ~(3<<16); // clock for the uart 2
    LPC_SC->PCLKSEL1 |= (1<<16);

    LPC_PINCON->PINSEL4 &= ~(0xF<<16); // Pin select for Uart2 RX/TX
    LPC_PINCON->PINSEL4 |= (0xA<<16);

    LPC_UART2->LCR = (1<<7); // enable DLAB
    LPC_UART2->DLM = (divider >> 8);


    LPC_UART2->DLL = (divider >> 0);
    LPC_UART2->LCR = 3; // 8-bit character length

    return 1;
}

char uart2_hw1::uart2_outchar(char out)
{

    LPC_UART2->THR = out;
    while(!(LPC_UART2->LSR & (1<<6)));
    return 1;
}

char uart2_hw1::uart2_inchar()
{
    char in;
   in = LPC_UART2->RBR;
   printf("%c",in);
   return 1;

}

bool uart2_hw1::run(void *p)
{
       // char out;
       // printf("Enter the Character. ");
       // scanf("%c",&out);


       // uart2_outchar('H'); // Sending character H
        uart2_inchar(); // reading the register RBR
        return 1;

}

bool uart2_hw1::taskEntry(void)
{
    uart2_hw1_init();
    setRunDuration(1000);
    return 1;
}
