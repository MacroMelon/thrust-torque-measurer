//
// Created by thari on 18/08/2021.
//

#ifndef TESTING_SWASHPLATELESS_CONTROLLER_1_LIBCLCK_H
#define TESTING_SWASHPLATELESS_CONTROLLER_1_LIBCLCK_H

#include "stm32f4xx.h"

uint32_t volatile msTicks;                       // Counter for millisecond Interval

int set_system_clock_to_168Mhz();
void SysTick_Handler (void);
void WaitForTick (void);
void ms_delay(uint32_t ms);

#endif //TESTING_SWASHPLATELESS_CONTROLLER_1_LIBCLCK_H
