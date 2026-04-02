//
// Created by thari on 28/08/2021.
//

#ifndef TESTING_SWASHPLATELESS_CONTROLLER_1_LIBSPI_H
#define TESTING_SWASHPLATELESS_CONTROLLER_1_LIBSPI_H

#include "stm32f4xx.h"

volatile uint32_t scary_packet;

void SPIinit_slave();     //todo - DMA
void SPIwrite(uint16_t val);
uint16_t SPIread();

void SPI1_IRQHandler(void);

#endif //TESTING_SWASHPLATELESS_CONTROLLER_1_LIBSPI_H
