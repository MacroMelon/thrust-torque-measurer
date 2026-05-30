//
// Created by rin on 16/02/2026.
//

#include "libADC.h"

int initADC() {

    //configure the pins (PA4 PA5 and PA6)
    //enable clock to port A
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; //for voltage and current sensing
    //set to analog mode (See Table 28 and section 7.3.2 item 3 of stm32f4 reference manual)
    GPIOA->MODER |= GPIO_MODER_MODE5_0;
    GPIOA->MODER |= GPIO_MODER_MODE5_1;
    GPIOA->MODER |= GPIO_MODER_MODE6_0;
    GPIOA->MODER |= GPIO_MODER_MODE6_1;
    GPIOA->MODER |= GPIO_MODER_MODE7_0;
    GPIOA->MODER |= GPIO_MODER_MODE7_1;
    //for voltage and current sensing
    GPIOC->MODER |= GPIO_MODER_MODE4_0;
    GPIOC->MODER |= GPIO_MODER_MODE4_1;
    GPIOC->MODER |= GPIO_MODER_MODE5_0;
    GPIOC->MODER |= GPIO_MODER_MODE5_1;

    //So ADC writes each conversion, one after the other, into the output register. Read it each write and send to
    //the appropriate memory address

    //set ADCCLK prescaler to 4 to meet max 36 MHz requirement (sets to 21 Mhz at 84 MHz APB2 speed)
    ADC->CCR |= ADC_CCR_ADCPRE_0;

    //for more speed, try triple ADC interleave mode
    //(but the pins selected only go to ADC 1 and 2, so try double interleave mode)

    //enable bus clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    ADC1->CR2 |= ADC_CR2_ADON;  //turn on ADC

    ADC1->CR2 |= ADC_CR2_CONT;  //set it to continuous
    ADC1->CR1 |= ADC_CR1_SCAN;  //enable scan mode for multiple channels
    //12 bit resolution selected by default

    //set channel sequence
    //5 channels to convert (1 channel is 0, 2 is 1, 3 is 2)
    ADC1->SQR1 |= 4 << ADC_SQR1_L_Pos;
    //the actual channel order
    ADC1->SQR3 |= 5 << ADC_SQR3_SQ1_Pos;
    ADC1->SQR3 |= 6 << ADC_SQR3_SQ2_Pos;
    ADC1->SQR3 |= 7 << ADC_SQR3_SQ3_Pos;
    ADC1->SQR3 |= 14 << ADC_SQR3_SQ4_Pos;
    ADC1->SQR3 |= 15 << ADC_SQR3_SQ5_Pos;

    /*
    From STM32f4 datasheet (RM0090), section 11.5 page 272 / 273:
    The total conversion time is calculated as follows:
    Tconv = Sampling time + 12 cycles
    Example:
    With ADCCLK = 30 MHz and sampling time = 3 cycles:
    Tconv = 3 + 12 = 15 cycles = 0.5 µs with APB2 at 60 MHz
    In our case - 1/((1/21000000)*(12+144))
    where 21000000 is adc clock speed, 144 is num samples
     */
    //Todo - dynamically set sample time based on reading rate
    ADC1->SMPR2 |= sampleTimeSetting << ADC_SMPR2_SMP5_Pos;
    ADC1->SMPR2 |= sampleTimeSetting << ADC_SMPR2_SMP6_Pos;
    ADC1->SMPR2 |= sampleTimeSetting << ADC_SMPR2_SMP7_Pos;
    ADC1->SMPR1 |= sampleTimeSetting << ADC_SMPR1_SMP14_Pos;
    ADC1->SMPR1 |= sampleTimeSetting << ADC_SMPR1_SMP15_Pos;

    //for DMA, either, peripheral -> memory and then memory -> peripheral
    //or try direct peripheral to peripheral based on
    //https://community.st.com/t5/stm32-mcus-products/stm32f407-dma-transfer-peripheral-to-peripheral/td-p/414462

    //init DMA - ADC1 is channel 0 stream 0 of DMA2
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    DMA2_Stream0->CR = 0;		//reset just in case
    //channel zero is default
    //peripheral to memory is default
    //direct transfer is default (DMA_SxFCR_DMDIS of DMA2_Stream0->FCR should be 0)

    DMA2_Stream0->CR |= DMA_SxCR_PL_1;			//Priority level 10b, High
    DMA2_Stream0->CR |= DMA_SxCR_MSIZE_0;		//memory data size 01b half word (16bit)
    DMA2_Stream0->CR |= DMA_SxCR_PSIZE_0;		//peripheral data size 01b half word (16bit)
    DMA2_Stream0->CR |= DMA_SxCR_MINC;			//increment memory address after each transfer
    DMA2_Stream0->CR |= DMA_SxCR_CIRC;			//Circular mode, reset address back to 0 after transfers done
    //peripheral to memory direction selected by default
    DMA2_Stream0->NDTR = numSensors;			//number of data items to be transferred

    DMA2_Stream0->PAR = &(ADC1->DR);				//Peripheral address of transfer
    DMA2_Stream0->M0AR = &(sensorValues);		//memory address of transfer

    //enable DMA requests
    ADC1->CR2 |= ADC_CR2_DMA;
    ADC1->CR2 |= ADC_CR2_DDS;       //okay. NO ONE SAID TO ENABLE THIS! thanks to https://community.st.com/t5/stm32-mcus-products/adc-to-dac-using-dma-in-stm32f4/td-p/623403
    DMA2_Stream0->CR |= DMA_SxCR_EN;

    //start conversions
    ADC1->CR2 |= ADC_CR2_SWSTART;

    return 0;
}
