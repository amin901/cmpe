/*
 * i2c_hw3.cpp
 *
 *  Created on: Mar 2, 2014
 *      Author: Arpit
 */

#include "tasks.hpp"
#include "io.hpp"
#include "stdio.h"


i2c_hw3::i2c_hw3(uint8_t priority) :
                scheduler_task("i2c",512,priority)
{

}

bool i2c_hw3::run(void *p)
{
    const uint8_t switches = SW.getSwitchValues();
        enum
        {
            sw1 = (1<<0),
            sw2 = (1<<1),
            sw3 = (1<<2),
            sw4 = (1<<3),
        };

        switch(switches)
            {
                case sw1 :
                  // printf("Switch 1 press\n");
                    LD.clear();
                    LD.setNumber(LS.getPercentValue());

                    break;

                case sw2 :
                  // printf("Switch 2 Press\n");

                    LD.clear();
                    LD.setNumber(SW.getSwitchValues());
                    break;

                case sw3 :
                   // printf("Switch 3 press\n");
                    LE.setAll(0xA);
                    break;

                case sw4 :
                    /* Send broadcast message, and increment led number if we get a packet back */
                   // printf("Switch 4 press\n");
                    LE.setAll(0xF);
                    break;

                default :
                    break;
            }
    return true;
}

bool i2c_hw3::taskEntry(void)
{
    setRunDuration(100);
    return true;
}
