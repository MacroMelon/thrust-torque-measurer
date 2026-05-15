//
// Created by rin on 15/05/2026.
//

#ifndef THRUST_TORQUE_MEASURER_LIBDAC_H
#define THRUST_TORQUE_MEASURER_LIBDAC_H

#include <stm32f405xx.h>

int initDAC();
void setIAVref(uint16_t val);

#endif //THRUST_TORQUE_MEASURER_LIBDAC_H