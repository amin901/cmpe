/*
 * Bluetooth.hpp
 *
 *  Created on: May 25, 2014
 *      Author: Arpit
 */

#ifndef BLUETOOTH_HPP_
#define BLUETOOTH_HPP_

#include "gps_task.hpp"
#include "uart3.hpp"

class Bluetooth : public scheduler_task
{
    public:
        Bluetooth(uint8_t priority);
        virtual ~Bluetooth();
        bool init(void);
        bool run(void *p);
        bool taskEntry(void);
        Uart3 *ble;
};

#endif /* BLUETOOTH_HPP_ */
