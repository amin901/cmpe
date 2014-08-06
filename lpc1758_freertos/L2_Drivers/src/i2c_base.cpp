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

#include <string.h>         // memcpy

#include "i2c_base.hpp"
#include "lpc_sys.h"
#include "isr_priorities.h"



/**
 * Instead of using a dedicated variable for read vs. write, we just use the LSB of
 * the user address to indicate read or write mode.
 */
#define I2C_SET_READ_MODE(addr)     (addr |= 1)     ///< Set the LSB to indicate read-mode
#define I2C_SET_WRITE_MODE(addr)    (addr &= 0xFE)  ///< Reset the LSB to indicate write-mode
#define I2C_READ_MODE(addr)         (addr & 1)      ///< Read address is ODD
#define I2C_WRITE_ADDR(addr)        (addr & 0xFE)   ///< Write address is EVEN
#define I2C_READ_ADDR(addr)         (addr | 1)      ///< Read address is ODD



void I2C_Base::handleInterrupt()
{
    /* If transfer finished (not busy), then give the signal */
    if (busy != i2cStateMachine()) {
        long higherPriorityTaskWaiting = 0;
        xSemaphoreGiveFromISR(mTransferCompleteSignal, &higherPriorityTaskWaiting);
        portEND_SWITCHING_ISR(higherPriorityTaskWaiting);
    }
}

char I2C_Base::readReg(char deviceAddress, char registerAddress)
{
    char byte = 0;
    readRegisters(deviceAddress, registerAddress, &byte, 1);
    return byte;
}

bool I2C_Base::readRegisters(char deviceAddress, char firstReg, char* pData, unsigned int bytesToRead)
{
    I2C_SET_READ_MODE(deviceAddress);
    return transfer(deviceAddress, firstReg, pData, bytesToRead);
}

bool I2C_Base::writeReg(char deviceAddress, char registerAddress, char value)
{
    return writeRegisters(deviceAddress, registerAddress, &value, 1);
}

bool I2C_Base::writeRegisters(char deviceAddress, char firstReg, char* pData, unsigned int bytesToWrite)
{
    I2C_SET_WRITE_MODE(deviceAddress);
    return transfer(deviceAddress, firstReg, pData, bytesToWrite);
}

bool I2C_Base::transfer(char deviceAddress, char firstReg, char* pData, unsigned int transferSize)
{
    bool status = false;
    if(mDisableOperation || transferSize > sizeof(mI2CIOFrame.rwBuffer)) {
        return status;
    }

    // If scheduler not running, perform polling transaction
    if(taskSCHEDULER_RUNNING != xTaskGetSchedulerState())
    {
        i2cKickOffTransfer(deviceAddress, firstReg, pData, transferSize);

        // Wait for transfer to finish and copy the data if it was read mode
        const uint64_t timeout = sys_get_uptime_ms() + I2C_TIMEOUT_MS;
        while (!xSemaphoreTakeFromISR(mTransferCompleteSignal, NULL)) {
            if (sys_get_uptime_ms() > timeout) {
                break;
            }
        }

        status = (0 == mI2CIOFrame.error);
        if (status && I2C_READ_MODE(deviceAddress)) {
            memcpy(pData, &mI2CIOFrame.rwBuffer[0], transferSize);
        }
    }
    else if (xSemaphoreTake(mI2CMutex, OS_MS(I2C_TIMEOUT_MS)))
    {
        // Clear potential stale signal and start the transfer
        xSemaphoreTake(mTransferCompleteSignal, 0);
        i2cKickOffTransfer(deviceAddress, firstReg, pData, transferSize);

        // Wait for transfer to finish and copy the data if it was read mode
        if (xSemaphoreTake(mTransferCompleteSignal, OS_MS(I2C_TIMEOUT_MS))) {
            status = (0 == mI2CIOFrame.error);
            if (status && I2C_READ_MODE(deviceAddress)) {
                memcpy(pData, &mI2CIOFrame.rwBuffer[0], transferSize);
            }
        }

        xSemaphoreGive(mI2CMutex);
    }

    return status;
}

bool I2C_Base::isDevicePresent(char deviceAddress)
{
    char dummyReg = 0;
    char notUsed = 0;

    // The I2C State machine will not continue after 1st state when length is set to 0
    unsigned int lenZeroToTestDeviceReady = 0;

    return readRegisters(deviceAddress, dummyReg, &notUsed, lenZeroToTestDeviceReady);
}

I2C_Base::I2C_Base(LPC_I2C_TypeDef* pI2CBaseAddr) :
        mpI2CRegs(pI2CBaseAddr),
        mDisableOperation(false)
{
    mI2CMutex = xSemaphoreCreateMutex();
    vSemaphoreCreateBinary(mTransferCompleteSignal);

    /// Binary semaphore needs to be taken after creating it
    xSemaphoreTake(mTransferCompleteSignal, 0);

    if((unsigned int)mpI2CRegs == LPC_I2C0_BASE)
    {
        mIRQ = I2C0_IRQn;
    }
    else if((unsigned int)mpI2CRegs == LPC_I2C1_BASE)
    {
        mIRQ = I2C1_IRQn;
    }
    else if((unsigned int)mpI2CRegs == LPC_I2C2_BASE)
    {
        mIRQ = I2C2_IRQn;
    }
    else {
        mIRQ = (IRQn_Type)99; // Using invalid IRQ on purpose
    }
}

//bool I2C_Base::init(unsigned int pclk, unsigned int busRateInKhz)
//{
//    // Power on I2C
//    switch(mIRQ) {
//        case I2C0_IRQn: lpc_pconp(pconp_i2c0, true);  break;
//        case I2C1_IRQn: lpc_pconp(pconp_i2c1, true);  break;
//        case I2C2_IRQn: lpc_pconp(pconp_i2c2, true);  break;
//        default: return false;
//    }
//
//    mpI2CRegs->I2CONCLR = 0x6C;           // Clear ALL I2C Flags
//
//    // Set the I2C Dividers to attain the frequency
//    const unsigned int i2cFrequency = busRateInKhz > 1000 ? 100*1000 : busRateInKhz * 1000;
//    mpI2CRegs->I2SCLH = (pclk/(i2cFrequency + 1)) / 2;
//    mpI2CRegs->I2SCLL = (pclk / i2cFrequency) / 2;
//
//    // Set I2C slave address and enable I2C
//    mpI2CRegs->I2ADR0 = 0;
//    mpI2CRegs->I2ADR1 = 0;
//    mpI2CRegs->I2ADR2 = 0;
//    mpI2CRegs->I2ADR3 = 0;
//
//    // After PINSEL, just need 1 line to enable I2C
//    mpI2CRegs->I2CONSET = 0x40;
//
//    NVIC_EnableIRQ(mIRQ);
//
//    return true;
//}



/// Private ///

void I2C_Base::i2cKickOffTransfer(char devAddr, char regStart, char* pBytes, int len)
{
    if(len <= I2C_FRAME_MAX_DATABYTES)
    {
        mI2CIOFrame.error     = 0;
        mI2CIOFrame.slaveAddr = devAddr;
        mI2CIOFrame.firstReg  = regStart;
        mI2CIOFrame.trxSize   = len;
        mI2CIOFrame.bytePtr   = 0;

        // Copy data to write into I2C Buffer.
        if(!I2C_READ_MODE(devAddr)) {
            memcpy(mI2CIOFrame.rwBuffer, pBytes, len);
        }

        // Send START, I2C State Machine will finish the rest.
        mpI2CRegs->I2CONSET = 0x20;
    }
}

/*
 * I2CONSET
 * 0x04 AA
 * 0x08 SI
 * 0x10 STOP
 * 0x20 START
 * 0x40 ENABLE
 *
 * I2CONCLR
 * 0x04 AA
 * 0x08 SI
 * 0x20 START
 * 0x40 ENABLE
 */
I2C_Base::mI2CStateMachineStatusType I2C_Base::i2cStateMachine()
{
    enum I2CStatus{ busError=0, start=0x08, repeatStart=0x10, arbitrationLost=0x38,
            // Master Transmitter States:
            slaveAddressAcked=0x18, slaveAddressNacked=0x20, dataAckedBySlave=0x28, dataNackedBySlave=0x30,
            // Master Receiver States:
            readAckedBySlave=0x40, readModeNackedBySlave=0x48, dataAvailableAckSent=0x50, dataAvailableNackSent=0x58,
            //slave receiver
            slaveaddressack = 0x60,
            slaveaddressnack = 0x80,
            stoporrepeatstart = 0xA0,

           //slave transmitter
            slaveaddresstransmitterdata = 0xA8,
            slavetransmitterdata = 0xB8,
            slavetransmitterdatanack = 0xC0,


    };

    mI2CStateMachineStatusType state = busy;

    /*
     ***********************************************************************************************************
     * Write-mode state transition :
     * start --> slaveAddressAcked --> dataAckedBySlave --> ... (dataAckedBySlave) --> (stop)
     *
     * Read-mode state transition :
     * start --> slaveAddressAcked --> dataAcked --> repeatStart --> readAckedBySlave
     *  For 2+ bytes:  dataAvailableAckSent --> ... (dataAvailableAckSent) --> dataAvailableNackSent --> (stop)
     *  For 1  byte :  dataAvailableNackSent --> (stop)
     ***********************************************************************************************************
     */

    #define clearSIFlag()       mpI2CRegs->I2CONCLR = (1<<3)
    #define setSTARTFlag()      mpI2CRegs->I2CONSET = (1<<5)
    #define clearSTARTFlag()    mpI2CRegs->I2CONCLR = (1<<5)

    // busInUse is only set to 0 for write operation since read operation should set to 0 itself
    #define setStop()           clearSTARTFlag();                           \
                                mpI2CRegs->I2CONSET = (1<<4);               \
                                clearSIFlag();                              \
                                while((mpI2CRegs->I2CONSET&(1<<4)));        \
                                if(I2C_READ_MODE(mI2CIOFrame.slaveAddr))    \
                                    state = readComplete;                   \
                                else                                        \
                                    state = writeComplete;

    switch (mpI2CRegs->I2STAT)
    {
        case start:
            mpI2CRegs->I2DAT = I2C_WRITE_ADDR(mI2CIOFrame.slaveAddr);
            clearSIFlag();
            break;
        case repeatStart:
            mpI2CRegs->I2DAT = I2C_READ_ADDR(mI2CIOFrame.slaveAddr);
            clearSIFlag();
            break;

        case slaveAddressAcked:
            clearSTARTFlag();
            if(0 == mI2CIOFrame.trxSize) {
                setStop();
            }
            else {
                mpI2CRegs->I2DAT = mI2CIOFrame.firstReg;
                clearSIFlag();
            }
            break;
        case slaveAddressNacked:
            mI2CIOFrame.error = mpI2CRegs->I2STAT;
            setStop();
            break;

        case dataAckedBySlave:
            if (I2C_READ_MODE(mI2CIOFrame.slaveAddr)) {
                setSTARTFlag(); // Send Repeat-start for read-mode
                clearSIFlag();
            }
            else {
                if(mI2CIOFrame.bytePtr >= mI2CIOFrame.trxSize) {
                    setStop();
                }
                else {
                    mpI2CRegs->I2DAT = mI2CIOFrame.rwBuffer[mI2CIOFrame.bytePtr++];
                    clearSIFlag();
                }
            }
            break;
        case dataNackedBySlave:
            mI2CIOFrame.error = mpI2CRegs->I2STAT;
            setStop();
            break;


        case readAckedBySlave:
            clearSTARTFlag();
            if(mI2CIOFrame.trxSize > 1)
                mpI2CRegs->I2CONSET = 0x04;  // Send ACK to receive a byte and transition to dataAvailableAckSent
            else
                mpI2CRegs->I2CONCLR = 0x04;  // NACK next byte to go to dataAvailableNackSent for 1-byte read.
            clearSIFlag();
            break;
        case readModeNackedBySlave:
            mI2CIOFrame.error = mpI2CRegs->I2STAT;
            setStop();
            break;
        case dataAvailableAckSent:
            mI2CIOFrame.rwBuffer[mI2CIOFrame.bytePtr++] = mpI2CRegs->I2DAT;
            if(mI2CIOFrame.bytePtr >= (mI2CIOFrame.trxSize-1)) {    // Only 1 more byte remaining
                mpI2CRegs->I2CONCLR = 0x04; // NACK next byte --> Next state: dataAvailableNackSent
            }
            else {
                mpI2CRegs->I2CONSET = 0x04; // ACK next byte --> Next state: dataAvailableAckSent(back to this state)
            }
            clearSIFlag();
            break;
        case dataAvailableNackSent: // Read last-byte from Slave
            mI2CIOFrame.rwBuffer[mI2CIOFrame.bytePtr++] = mpI2CRegs->I2DAT;
            setStop();
            break;

        case arbitrationLost:
            // We should not issue stop() in this condition, but we still need to end our
            // i2c transaction.
            state = I2C_READ_MODE(mI2CIOFrame.slaveAddr) ? readComplete : writeComplete;
            mI2CIOFrame.error = mpI2CRegs->I2STAT;
            break;

        case busError:
        default:
            mI2CIOFrame.error = mpI2CRegs->I2STAT;
            setStop();
            break;

        case slaveaddressack:

            break;

        case slaveaddressnack:
            break;

        case stoporrepeatstart:
            break;

        case slaveaddresstransmitterdata:
            break;

        case slavetransmitterdata:
            break;

        case slavetransmitterdatanack:
            break;
    }

    return state;
}



bool I2C_Base::init(unsigned int pclk, unsigned int busRateInKhz)
{
    // Power on I2C
        switch(mIRQ) {
            case I2C0_IRQn: lpc_pconp(pconp_i2c0, true);  break;
            case I2C1_IRQn: lpc_pconp(pconp_i2c1, true);  break;
            case I2C2_IRQn: lpc_pconp(pconp_i2c2, true);  break;
            default: return false;
        }

        mpI2CRegs->I2CONCLR = 0x6C;           // Clear ALL I2C Flags

        // Set the I2C Dividers to attain the frequency
        const unsigned int i2cFrequency = busRateInKhz > 1000 ? 100*1000 : busRateInKhz * 1000;
        mpI2CRegs->I2SCLH = (pclk/(i2cFrequency + 1)) / 2;
        mpI2CRegs->I2SCLL = (pclk / i2cFrequency) / 2;

        // Set I2C slave address and enable I2C
        mpI2CRegs->I2ADR0 = 0xAC;
        mpI2CRegs->I2ADR1 = 0;
        mpI2CRegs->I2ADR2 = 0;
        mpI2CRegs->I2ADR3 = 0;

        mpI2CRegs->I2MASK0 = 0xFF;

        // After PINSEL, just need 1 line to enable I2C
        mpI2CRegs->I2CONSET = 0x44;

        NVIC_EnableIRQ(mIRQ);

        return true;
}



