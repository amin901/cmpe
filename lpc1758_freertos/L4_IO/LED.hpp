/*
 *     SocialLedge.com - Copyright (C) 2013
 *
 *     This file is part of free software framework for embedded processors.
 *     You can use it and/or distribute it as long as this copyright header
 *     remains unmodified.  The code is free for personal use and requires
 *     permission to use in a commercial product.
 *
 *      THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *      OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *      MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *      I SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 *      CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *     You can reach the author of this software at :
 *          p r e e t . w i k i @ g m a i l . c o m
 */

/**
 * @file
 */
 
#ifndef LED_HPP__
#define LED_HPP__


/**
 * LED class used to control the Board's 8 output LEDs
 *
 * @ingroup BoardIO
 */
class LED : public SingletonTemplate<LED>
{
    public:
        bool init(); ///< Initializes this device, @returns true if successful

        void on(int ledNum);    ///< Turns  ON LED. @param ledNum The LED # from 1-4
        void off(int ledNum);   ///< Turns OFF LED. @param ledNum The LED # from 1-4
        void setAll(char value);///< Sets 8-bit value of 8 LEDs; 1 bit per LED
        uint8_t getValues(void) const; ///< Get the LED bit values currently set

    private:
        unsigned char mLedValue; ///< Current bits set on the LEDs

        LED() : mLedValue (0) {}    ///< Private constructor of this Singleton class
        friend class SingletonTemplate<LED>;  ///< Friend class used for Singleton Template
};

#endif /* LED_HPP__ */
