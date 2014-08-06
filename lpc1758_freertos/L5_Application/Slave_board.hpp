/*
 * Slave_board.hpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Arpit
 */

#ifndef SLAVE_BOARD_HPP_
#define SLAVE_BOARD_HPP_

#include "singleton_template.hpp"

class Slave_board : public SingletonTemplate<Slave_board>
{
    public:
        bool init();

    private:
        Slave_board() {}
        friend class SingletonTemplate<Slave_board>;

};




#endif /* SLAVE_BOARD_HPP_ */
