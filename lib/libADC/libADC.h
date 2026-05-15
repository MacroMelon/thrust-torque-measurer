//
// Created by rin on 16/02/2026.
//

#ifndef THRUST_TORQUE_MEASURER_LIBADC_H
#define THRUST_TORQUE_MEASURER_LIBADC_H

#include <stm32f405xx.h>

#define numSensors 5

volatile uint16_t sensorValues[numSensors];

int initADC();

#endif //THRUST_TORQUE_MEASURER_LIBADC_H