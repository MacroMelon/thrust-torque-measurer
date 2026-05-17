//
// Created by thari on 25/08/2021.
//

#include "libI2C.h"

//packet handling:
//TODO- this packet handling system is quite inefficeint and slow. maybe a better one?
/*
	 * So, heres the plan:
	 * One data packet will be 4 bytes (32 bits) long
	 * first 9 bits will be angle (0-512)
	 * next 10 bits will be scaler value (0-1024)
	 * next 13 bits will be throttle value (0-8192)
	 * TODO- Therefore, bitmasks:
	 */

void I2C_init_100kHz(uint8_t address)
{
	//I2C_CR1 is fine with defaults (0x0000) but we have to enable PE
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;		//enable clock to I2C1
	//enable GPIO port clock for i2c pins
	//(SCL/PB6, SDA/PB7)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	/*TODO-
	 * Now you need to set pins 6 and 7 of Port B to Alternate Function with Pull-up enabled and Open-Drain.
	 * We’ll also set them to the highest speed possible.
	 */
	//PB6
	GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL6);
	GPIOB->AFR[0] |= (4<< GPIO_AFRL_AFSEL6_Pos);	//enable SCL to PB6
	GPIOB->MODER &= ~(GPIO_MODER_MODE6);
	GPIOB->MODER |= GPIO_MODER_MODE6_1;				//10b - Alternate function
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT6);
	GPIOB->OTYPER |= GPIO_OTYPER_OT6;				//open drain output
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR6);
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6_1;		//high speed (50Mhz) TODO- 100Mz??
	//GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6_0;
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD6);
	//GPIOB->PUPDR |= GPIO_PUPDR_PUPD6_0;				//already pulled up

	//PB7
	GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL7);
	GPIOB->AFR[0] |= (4<< GPIO_AFRL_AFSEL7_Pos);	//enable SDA to PB7
	GPIOB->MODER &= ~(GPIO_MODER_MODE7);
	GPIOB->MODER |= GPIO_MODER_MODE7_1;				//10b - Alternate function
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT7);
	GPIOB->OTYPER |= GPIO_OTYPER_OT7;				//open drain output
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR7);
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7_1;		//high speed (50Mhz) TODO- 100Mz??
	//GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7_0;
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD7);
	//GPIOB->PUPDR |= GPIO_PUPDR_PUPD7_0;				//already pulled up


	//First disable the thingy, PE bit of I2C_CR1
	I2C1->CR1 &= ~(I2C_CR1_PE);
	//then reset it
	//I2C1 -> CR1 |= I2C_CR1_SWRST;	//set the bit
	//ms_delay(3);
	//I2C1 -> CR1 &= ~I2C_CR1_SWRST;	//aaaand clear the bit

	//NO need interrupts
	//in I2C_CR2, set ITBUFEN to generate interrupts for data
	//maybe also set ITEVTEN? yeah, needs both to generate interrupts
	//I2C1->CR2 |= I2C_CR2_ITBUFEN;
	//I2C1->CR2 |= I2C_CR2_ITEVTEN;

	//Next set peripheral clock frequency to 42mhz
	I2C1->CR2 &= ~(I2C_CR2_FREQ);		//clear first to make sure
	I2C1->CR2 |= 42;					//42Mhz
	//write address to OAR1 Register
	I2C1->OAR1 &= ~(I2C_OAR1_ADD1_7);	//clear first to make sure
	I2C1->OAR1 |= (address << 1);
	//also for some reason bit 14 of OAR1 should be set always?
	//I2C1->OAR1 &= (1 << 14);
	//I2C_CCR, set DUTY to 0, DUTY to 1 wont work here
	I2C1->CCR &= ~(I2C_CCR_DUTY);

	//I2C_CCR, set to standard mode
	I2C1->CCR &= ~(I2C_CCR_FS);
	//I2C_CCR set proper CRR bits

	//SO: we want 100kHz, which has a period of 10000ns
	//our TPCLK is (1/(42*10^-3))
	//therefore, (CCR * (1/(42*10^-3))) + (CCR * (1/(42*10^-3))) >= 10000
	//therefore CCR * (1/(42*10^-3)) * (1 + 1) = 10000
	//therefore CCR * (1/(42*10^-3)) * 2 = 10000
	//therefore CCR = 10000/((1/(42*10^-3)) * 2)
	//therefore CCR = 210 for 100kHz
	//perfection
	I2C1->CCR &= ~(I2C_CCR_CCR);
	I2C1->CCR |= 210;
	//Now the TRISE register - no idea
	//maximum standard mode rise time is 1000ns according to spec
	//so: 1000 / (1/(42*10^-3)) + 1 = 43
	// where (1/(42*10^-3)) is Period of PCLK (peripheral clock)
	//therefore : TRISE val should be 43 (dec)
	//to give us a max rise time of about 1000ns
	I2C1->TRISE &= ~(I2C_TRISE_TRISE);
	I2C1->TRISE |= 43;			//43 in dec

	//NVIC_SetPriority (I2C1_EV_IRQn, 1);            // no need interupts
	//NVIC_EnableIRQ (I2C1_EV_IRQn);

	//Finnaly enable the thingy, PE bit of I2C_CR1
	I2C1->CR1 |= I2C_CR1_PE;

	//in I2C_CR1, enable ACK (acknowlage bit)
	I2C1->CR1 |= I2C_CR1_ACK;	//Wow, this thing should be AFTER the darn PE lol, who knew??
}

//use address 0x28 and 0x29
void I2C_init_400kHz(uint8_t address)
{
	//TODO- finish / fix
	//SO: we want 400kHz, which has a period of 2500ns
	//our TPCLK is (1/(42*10^-3))
	//therefore, (CCR * (1/(42*10^-3))) + (2 * CCR * (1/(42*10^-3))) >= 2500
	//therefore CCR * (1/(42*10^-3)) * (1 + 2) = 2500
	//therefore CCR * (1/(42*10^-3)) * 3 = 2500
	//therefore CCR = 2500/((1/(42*10^-3)) * 3)
	//therefore CCR = 35 (0x23) for 400kHz
	//perfection

	//Now the TRISE register - no idea
	//maximum fast mode rise time is 300ns according to spec
	//so: 310 / (1/(42*10^-3)) + 1 = 14 ish
	//where (1/(42*10^-3)) is Period of PCLK (peripheral clock)
	//therefore : TRISE val should be 14 (dec)
	//to give us a max rise time of about 310ns
}

/*
 –Transmitter mode: Byte transmission starts automatically when a byte is written in the DR
 register. A continuous transmit stream can be maintained if the next data to be transmitted
 is put in DR once the transmission is started (TxE=1)

 –Receiver mode: Received byte is copied into DR (RxNE=1). A continuous transmit stream
 can be maintained if DR is read before the next data byte is received (RxNE=1).

 Note: In slave mode, the address is not copied into DR
 */

void I2C_master_Transmit_byte(uint8_t slave_address, uint8_t data)
{
	while(I2C1->SR2 & I2C_SR2_BUSY){}		//wait for start condition to generate
	I2C1->CR1 |= I2C_CR1_START;				//generate start condition
	while(!(I2C1->SR1 & I2C_SR1_SB)){}		//wait for start condition to generate
	I2C1->DR = (slave_address << 1);		//LSB is 0 for I2C write, also write address
	while(!(I2C1->SR1 & I2C_SR1_ADDR)){}	//wait for address match
	//uint8_t reg;
	//reg = I2C1->SR2;		//to clear ADDR
	I2C1->SR2;				//to clear ADDR
	I2C1->DR = data;
	while((!(I2C1->SR1 & I2C_SR1_TXE)) && (!(I2C1->SR1 & I2C_SR1_BTF))){}		//wait to generate stop condition
	I2C1->CR1 |= I2C_CR1_STOP;			//generate stop
	//GPIOC->ODR |= (1<<1);		//toggle ye olde debug LED
}

void Motor1_SendPacket(uint32_t packet)
{
	//then reset it
	//I2C1 -> CR1 |= I2C_CR1_SWRST;	//set the bit
	//ms_delay(1);
	//I2C1 -> CR1 &= ~I2C_CR1_SWRST;	//aaaand clear the bit

	while(I2C1->SR2 & I2C_SR2_BUSY){}		//wait for bus free
	I2C1->CR1 |= I2C_CR1_START;				//generate start condition
	while(!(I2C1->SR1 & I2C_SR1_SB)){}		//wait for start condition to generate
	I2C1->DR = (motorCtrl1_addr << 1);		//LSB is 0 for I2C write, also write address
	while(!(I2C1->SR1 & I2C_SR1_ADDR)){}	//wait for address match
	//uint8_t reg;
	//reg = I2C1->SR2;		//to clear ADDR
	I2C1->SR2;				//to clear ADDR

	//TODO- use while loop?
	I2C1->DR = packet >> 0;
	while(!(I2C1->SR1 & I2C_SR1_TXE)){}
	I2C1->DR = packet >> 8;
	while(!(I2C1->SR1 & I2C_SR1_TXE)){}
	I2C1->DR = packet >> 16;
	while(!(I2C1->SR1 & I2C_SR1_TXE)){}
	I2C1->DR = packet >> 24;
	while(!(I2C1->SR1 & I2C_SR1_TXE)){}

	while(!(I2C1->SR1 & I2C_SR1_BTF)){}		//wait to generate stop condition
	I2C1->CR1 |= I2C_CR1_STOP;			//generate stop
}

//TODO- configure GPIO for SS for Mot2
void Motor2_SendPacket(uint32_t packet)
{
	//then reset it
	//I2C1 -> CR1 |= I2C_CR1_SWRST;	//set the bit
	//ms_delay(1);
	//I2C1 -> CR1 &= ~I2C_CR1_SWRST;	//aaaand clear the bit

	while(I2C1->SR2 & I2C_SR2_BUSY){}		//wait for bus free
	I2C1->CR1 |= I2C_CR1_START;				//generate start condition
	while(!(I2C1->SR1 & I2C_SR1_SB)){}		//wait for start condition to generate
	I2C1->DR = (motorCtrl2_addr << 1);		//LSB is 0 for I2C write, also write address
	while(!(I2C1->SR1 & I2C_SR1_ADDR)){}	//wait for address match
	//uint8_t reg;
	//reg = I2C1->SR2;		//to clear ADDR
	I2C1->SR2;				//to clear ADDR

	//TODO- use while loop?
	I2C1->DR = packet >> 0;
	while(!(I2C1->SR1 & I2C_SR1_TXE)){}
	I2C1->DR = packet >> 8;
	while(!(I2C1->SR1 & I2C_SR1_TXE)){}
	I2C1->DR = packet >> 16;
	while(!(I2C1->SR1 & I2C_SR1_TXE)){}
	I2C1->DR = packet >> 24;
	while(!(I2C1->SR1 & I2C_SR1_TXE)){}

	while(!(I2C1->SR1 & I2C_SR1_BTF)){}		//wait to generate stop condition
	I2C1->CR1 |= I2C_CR1_STOP;			//generate stop
}

void I2C1_EV_IRQHandler(void){		//interrupt handler

}