/*
 * Led_Switch.cpp
 *
 *  Created on: Sep 7, 2014
 *      Author: Arpit
 */
#include "tasks.hpp"
#include "FreeRTOS.h"
#include "sys_config.h"
#include "stdio.h"

Led_Switch::Led_Switch(uint8_t priority):scheduler_task("led_switch",512,priority)
{
    //switch input port
    LPC_GPIO2->FIODIR &= ~(1<<6);
    LPC_PINCON->PINMODE4 |= (3<<12);
    //led output port
    LPC_GPIO2->FIODIR |= (1<<7);

}

bool Led_Switch::run(void *p)
{
    if(LPC_GPIO2->FIOPIN & (1<<6))
    {
       // LPC_GPIO2->FIOSET = (1<<7);
        if(LPC_GPIO2->FIOPIN & (1<<7))
        {
            LPC_GPIO2->FIOCLR = (1<<7);
        }

        else
        {
            LPC_GPIO2->FIOSET = (1<<7);
        }
    }
    else
    {
       // LPC_GPIO2->FIOCLR = (1<<7);
    }
    return true;
}

bool Led_Switch::taskEntry()
{
    setRunDuration(100);
    return true;
}


