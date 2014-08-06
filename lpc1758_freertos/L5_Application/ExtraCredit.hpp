/*
 * ExtraCredit.hpp
 *
 *  Created on: May 12, 2014
 *      Author: Arpit
 */

#ifndef EXTRACREDIT_HPP_
#define EXTRACREDIT_HPP_

#include "cpp_task.hpp"
#include "soft_timer.hpp"
#include "command_handler.hpp"
#include "wireless.h"
#include "char_dev.hpp"
#include "stdio.h"

class taskOne : public scheduler_task
{
    public:
        taskOne(uint8_t priority);
        bool taskOne_init(void);
        bool run(void *p);
        bool taskEntry(void);
};

class taskTwo : public scheduler_task
{
    public:
        taskTwo(uint8_t priorty);
        bool taskTwo_init(void);
        bool run(void *p);
        FILE *file;
};




#endif /* EXTRACREDIT_HPP_ */
