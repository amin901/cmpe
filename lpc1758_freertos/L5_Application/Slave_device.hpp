/*
 * Slave_device.hpp
 *
 *  Created on: Mar 3, 2014
 *      Author: Arpit
 */

#ifndef SLAVE_DEVICE_HPP_
#define SLAVE_DEVICE_HPP_

#include "singleton_template.hpp"
#include "src/I2C_Device_Base.hpp"


const char SlaveAddress = 0xAC;

class I2C_Slave : private I2C_Device_Base
{
    public:
        bool init();
        I2C_Slave(char addr) : I2C_Device_Base(addr){}



};

class Slave_device : public I2C_Slave, public SingletonTemplate<Slave_device>
{
    private:
        Slave_device() : I2C_Slave(SlaveAddress){}

        friend class SingletonTemplate<Slave_device>;
};




#endif /* SLAVE_DEVICE_HPP_ */
