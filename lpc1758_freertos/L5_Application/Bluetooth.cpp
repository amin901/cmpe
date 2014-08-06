/*
 * Bluetooth.cpp
 *
 *  Created on: May 25, 2014
 *      Author: Arpit
 */

#include <Bluetooth.hpp>
#include "uart3.hpp"
#include "stdio.h"
#include <stdlib.h>
#include <string.h>
#include "utilities.h"
#include "uart0_min.h"




Bluetooth::Bluetooth(uint8_t priority) : scheduler_task("bluetooth", 1024*2, priority),ble(0)
{
    // TODO Auto-generated constructor stub

}

Bluetooth::~Bluetooth()
{
    // TODO Auto-generated destructor stub
}

bool Bluetooth::init(void)
{
    {
        Uart3 &u3 = Uart3::getInstance();
        u3.init(9600);
        ble = &u3;
    }
    return true;
}

bool Bluetooth::run(void *p)
{
    char buff[64] ={"AT+BAUD"};
    char data[] = {0};
    uart0_puts("Running..");

    ble->putline(buff);
    vTaskDelay(100);
    ble->gets(data,sizeof(data));
    uart0_puts(data);

    return true;
}

bool Bluetooth::taskEntry(void)
{
    setRunDuration(100);
    return true;
}
