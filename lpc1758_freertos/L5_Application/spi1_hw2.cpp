/*
 * spi1_hw2.cpp
 *
 *  Created on: Feb 23, 2014
 *      Author: Arpit
 */


#include "FreeRTOS.h"
#include "sys_config.h"
#include "tasks.hpp"
#include "stdio.h"

spi1_hw2::spi1_hw2(uint8_t priority) :
            scheduler_task("remote", 512, priority)
{

}

bool spi1_hw2::run(void *p)
{
   //readFlash();
   readSignature();
   return true;
}

uint8_t spi1_hw2::spi1_byte(uint8_t out)
{
    LPC_SSP1->DR = out;
    while (LPC_SSP1->SR & (1<<4));
    //char in;
    //in = LPC_SSP1->DR;
    //printf("%c\n",in);
    return LPC_SSP1->DR;
}

bool spi1_hw2::spi1_hw2_init(void)
{
    LPC_SC->PCONP |= (1<<10); // power on SSP1
    LPC_SC->PCLKSEL0 &= ~(3<<20); //clear clock bits
    LPC_SC->PCLKSEL0 |= (1<<20); // set clock to clk


    LPC_PINCON->PINSEL3 &= ~( (3<<8) | (3<<14) | (3<<16) );
    LPC_PINCON->PINSEL3 |= ( (3<<8) | (3<<14) | (3<<16) );

    LPC_SSP1->CR0 = 7; // 8-bit mode
    LPC_SSP1->CR1 = (1<<1);
    LPC_SSP1->CPSR = 8;

    return 1;
}

bool spi1_hw2::taskEntry(void)
{


    LPC_PINCON->PINSEL1 &= ~(3<<28);
    LPC_GPIO0->FIODIR |=(1<<30);

    deSelectFlash();
    spi1_hw2_init();
    setRunDuration(5 * 60 *1000);
    return 1;
}

void spi1_hw2::readFlash()
{
   // const uint8_t Low_Freq_read = 0x03;
    //const uint32_t sectorLength = 512;
    //const uint32_t sanityCheck = 2;

    //Flash ready
    waitFlashReady();

    selectFlash();
    deSelectFlash();
}

void spi1_hw2::waitFlashReady()
{
    selectFlash();
    spi1_byte(0xD7);
    uint8_t status= 0;

    do
    {
        status = spi1_byte(0x00);
    } while (!(status & (1<<7)));

    deSelectFlash();
}

void spi1_hw2::selectFlash()
{
    LPC_GPIO0->FIOCLR = (1<<30);
}

void spi1_hw2::deSelectFlash()
{
    LPC_GPIO0->FIOSET = (1<<30);
}

void spi1_hw2::readSignature()
{
    waitFlashReady();
    selectFlash();

    spi1_byte(0x9F);
    unsigned char mfgId = spi1_byte(0x00);
    unsigned char devId1 = spi1_byte(0x00);
    unsigned char devId2 = spi1_byte(0x00);

    printf("Manufacture ID = 0x%x \n",mfgId);
    printf("Device ID 1 = 0x%x \n",devId1);
    printf("Device ID 2= 0x%x \n",devId2);

    deSelectFlash();
}
