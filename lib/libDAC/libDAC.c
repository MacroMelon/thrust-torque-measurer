//
// Created by rin on 15/05/2026.
//

#include "libDAC.h"

int initDAC() {

    //configure the pins (PA4 PA5 and PA6)
    //enable clock to port A
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //will already be inited, but issok to have this here
    //set to analog mode (See note below Table 57 (page 311) of DM00031020 stm32f4 reference manual)
    GPIOA->MODER |= GPIO_MODER_MODE4_0;
    GPIOA->MODER |= GPIO_MODER_MODE4_1;

    //enable bus clock
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;

    //no need to enable any triggers (DAC_DHRx registers auto copy to output on write)

    //enable output buffer for lower output impedance (is enabled by default)
    //because INA823 datasheet says it wants a low impedance source for Vref for some reason?

    //wow, basically no config required?

    //enable DAC channel 1
    DAC->CR |= DAC_CR_EN1;

    return 0;
}

void setIAVref(uint16_t val) {
    //val = (val << 4) >> 4;  //ensure first 4 bits are zero because only 12 bits allowed
    DAC->DHR12R1 = val;
}
