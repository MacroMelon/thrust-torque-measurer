#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>
#include "libCLCK.h"
#include "libADC.h"
#include "libDAC.h"
#include "libI2C.h"
#include "usb.h"    //had to disable linecoding set check to work

//#define numSensors 3 //+2

const char separator[] = "\n";

uint16_t loadCellOffsets[3] = {0, 175, 225}; //should this be done during post-processing?

// [0] angle, [1] scaler, [2] throttle
uint16_t testMotorValues[3];

uint32_t buildPacket(uint16_t motorValues[3])
{
    //structure:
    /*
     *  [ header(2) | angle(9) | scaler(10) | throttle(11) ]
     */
    uint32_t final_packet = 0;
    //strip values to required number of bits:
    motorValues[0] &= (0x01FF);
    motorValues[1] &= (0x03FF);
    motorValues[2] &= (0x07FF);
    //add em all into one packet
    final_packet = (motorValues[0] << 21);
    final_packet |= (motorValues[1] << 11);
    final_packet |= (motorValues[2]);
    final_packet |= (2 << 30);	//header
    return final_packet;
}

int main() {

    //debug LED
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER |= GPIO_MODER_MODER1_0;
    GPIOC->MODER &= ~GPIO_MODER_MODER1_1;

    GPIOC->ODR |= (1<<1);

    set_system_clock_to_168Mhz();

    __enable_irq();

    I2C_init_100kHz(0x24);

    //wait for a bit for motor controller to initialise
    ms_delay(1000);

    //ensure motor stays off
    testMotorValues[0] = 0;
    testMotorValues[1] = 0;
    testMotorValues[2] = 0;
    Motor1_SendPacket(buildPacket(testMotorValues));

    USB_Init();

    initADC();  //yeah it would help if you actually called it before debugging
    initDAC();

    //try to offset instrumentation amplifier REF voltage to try and reduce common mode voltage related issues
    setIAVref(2700);  //max 2^12 = 4096

    //wait for a bit, just for... ummm reasons I guess?
    ms_delay(2000);

    GPIOC->ODR &= ~(1<<1);

    //uint8_t outBuffer[numSensors * 2];

    uint32_t motorControlRateControlCounter = 0;

    while (1) {

        // ---------------- Motor Control --------------------------

        //run this every 100 loop iterations so its around 15 Hz
        if (motorControlRateControlCounter >= 100) {
            motorControlRateControlCounter = 0;

            //send control packet
            testMotorValues[0] = 0;
            testMotorValues[1] = 100;
            testMotorValues[2] = 400;
            Motor1_SendPacket(buildPacket(testMotorValues));
        }
        motorControlRateControlCounter++;

        // ---------------- Sense Data -----------------------------

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

        char outString[40];  //does sprintf treat /n as 1 character?
        sprintf(outString, "%04u,%04u,%04u,%04u,%04u,%03u,%04u,%04u\n",
            sensorValues[0],
            sensorValues[1],
            sensorValues[2],
            sensorValues[3],
            sensorValues[4],
            testMotorValues[0],
            testMotorValues[1],
            testMotorValues[2]);
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
        USB_StartTXTransfer(1, outString, 39); //does sprintf squish \n into one character?
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