#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>
#include "libCLCK.h"
#include "libADC.h"
#include "libDAC.h"
#include "usb.h"    //had to disable linecoding set check to work

//#define numSensors 3 //+2

const char separator[] = "\n";

uint16_t loadCellOffsets[3] = {0, 175, 225};

int main() {

    //debug LED
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER |= GPIO_MODER_MODER1_0;
    GPIOC->MODER &= ~GPIO_MODER_MODER1_1;

    GPIOC->ODR |= (1<<1);

    set_system_clock_to_168Mhz();

    __enable_irq();

    USB_Init();

    initADC();  //yeah it would help if you actually called it before debugging
    initDAC();

    //try to offset instrumentation amplifier REF voltage to try and reduce common mode voltage related issues
    setIAVref(2700);  //max 2^12 = 4096

    GPIOC->ODR &= ~(1<<1);

    //uint8_t t = 0;

    //uint8_t outBuffer[numSensors * 2];

    //seems like adc sample time too low, S&H capacitor not fully discharging or something so channels affect each other
    //https://community.st.com/t5/stm32-mcus-products/one-adc-channel-affecting-the-other-adc-channels-stm32f407/td-p/433816
    //or not?

    while (1) {

        /*
        sensorValues[0] = 3*t + 4;
        sensorValues[1] = 10*t +3;
        sensorValues[2] = 2*t + 6;
        if (t > 250) {
            t = 0;
        }
        else {
            t++;
        }*/

        /*
        for (int i = 0; i < numSensors; i++) {
            const uint16_t safeSensorValue = sensorValues[i];
            //uint16_t safeSensorValue = ADC1->DR;
            outBuffer[i*2] = safeSensorValue >> 8;
            outBuffer[(i*2)+1] = safeSensorValue;
        }
        //outBuffer[numSensors - 2] = separator[0];
        //outBuffer[numSensors - 1] = separator[1];

        */

        char outString[16];  //does sprintf treat /n as 1 character?
        sprintf(outString, "%04u,%04u,%04u\n", sensorValues[0], sensorValues[1], sensorValues[2]);
        //also send back voltage, current, requested motor speed, requested tilt angle and requested tilt amount

        /*
        if (DMA2_Stream0->NDTR == 3) {
            GPIOC->ODR |= (1<<1);
        }
        else {
            GPIOC->ODR &= ~(1<<1);
        }*/
        //USB_StartTXTransfer(1, outBuffer, numSensors * 2);
        //USB_StartTXTransfer(1, outb, strlen(outb));
        USB_StartTXTransfer(1, outString, 15);
        WaitForTick();
    }

}

void USB_EP1RXCallBack(uint8_t * RX_buff, uint16_t length) {
    //USB_OUTEPSNAK(1);
    USB_StartTXTransfer(1, RX_buff, length);
    /*
    if (USB_StartTXTransfer(1, RX_buff, length) != USB_OK) {
        GPIOC->ODR |= (1<<1);
        ms_delay(500);
        GPIOC->ODR &= ~(1<<1);
    }
    else {
        GPIOC->ODR |= (1<<1);
    }*/
}

void USB_EP1TXTransferCompliteCallBack() {
    //USB_PrepareReceive(1);
    //GPIOC->ODR &= ~(1<<1);
}