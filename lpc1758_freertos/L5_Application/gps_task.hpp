/*
 * gps_task.hpp
 *
 *  Created on: Apr 10, 2014
 *      Author: Arpit
 */

#ifndef GPS_TASK_HPP_
#define GPS_TASK_HPP_

#include "cpp_task.hpp"
#include "uart2.hpp"


class gps_task : public scheduler_task
{
    public:
        gps_task(uint8_t priority);
        bool run(void *p);
        bool taskEntry(void *p);
        Uart2 *gps;
        bool init(void);
        void calculate(double,char*);

};



#endif /* GPS_TASK_HPP_ */
