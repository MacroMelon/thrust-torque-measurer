//
// Created by thari on 25/08/2021.
//

#ifndef TESTING_SWASHPLATELESS_CONTROLLER_1_LIBI2C_H
#define TESTING_SWASHPLATELESS_CONTROLLER_1_LIBI2C_H

#include "stm32f4xx.h"
#include <memory.h>		//memcpy
#include <libCLCK.h> 	//delay

#define motorCtrl1_addr 0x28
#define motorCtrl2_addr 0x29

void I2C_init_100kHz(uint8_t address);
void I2C_init_400kHz(uint8_t address);
void I2C_master_Transmit_byte(uint8_t slave_address, uint8_t data);
void I21C_EV_IRQHandler(void);

void Motor1_SendPacket(uint32_t packet);
void Motor2_SendPacket(uint32_t packet);

#endif //TESTING_SWASHPLATELESS_CONTROLLER_1_LIBI2C_H
