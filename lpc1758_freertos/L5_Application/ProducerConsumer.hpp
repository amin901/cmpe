/*
 * ProducerConsumer.hpp
 *
 *  Created on: Mar 15, 2014
 *      Author: Arpit
 */

#ifndef PRODUCERCONSUMER_HPP_
#define PRODUCERCONSUMER_HPP_

#include "cpp_task.hpp"



class orient_compute : public scheduler_task

{
    public:
        bool run(void *p);
        orient_compute(uint8_t priority);
        bool orient_compute_init(void);
        bool taskEntry(void);
};

class orient_process : public scheduler_task
{
    public:
        bool run(void *p);
        orient_process(uint8_t priority);
        bool taskEntry(void);
        bool orient_process_init(void);
};



#endif /* PRODUCERCONSUMER_HPP_ */
