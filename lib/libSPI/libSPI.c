//
// Created by thari on 28/08/2021.
//

#include "libSPI.h"

void SPIinit_slave()
{
	//the flash module CS is normal, use push pull (low to select, high to deselect)
	//The AS5047P SPI uses mode=1 (CPOL=0, CPHA=1) and MSB first wit parity upto 10Mhz
	/*
	 *
	Procedure
	1. 	Set the DFF bit to define 8- or 16-bit data frame format
	2. 	Select the CPOL and CPHA bits to define one of the four relationships between the
		data transfer and the serial clock (see Figure 273). For correct data transfer, the CPOL
		and CPHA bits must be configured in the same way in the slave device and the master
		device. This step is not required when the TI mode is selected through the FRF bit in
		the SPI_CR2 register.
	3. 	The frame format (MSB-first or LSB-first depending on the value of the LSBFIRST bit in
		the SPI_CR1 register) must be the same as the master device. This step is not required
		when TI mode is selected.
	4. 	In Hardware mode (refer to Slave select (NSS) pin management on page 798), the
		NSS pin must be connected to a low level signal during the complete byte transmit
		sequence. In NSS software mode, set the SSM bit and clear the SSI bit in the SPI_CR1
		register. This step is not required when TI mode is selected.
	5. 	Set the FRF bit in the SPI_CR2 register to select the TI mode protocol for serial
		communications.
	6. 	Clear the MSTR bit and set the SPE bit (both in the SPI_CR1 register) to assign the
		pins to alternate functions.
	In this configuration the MOSI pin is a data input and the MISO pin is a data output.
	*/

	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;		//enable clocks to SPI peripheral
	SPI2->CR1 = 0;						//Disable and zero all

	//enable clocks to GPIO port and configure pins
	//To select the SPI alternate function AF5 must be selected.
	//For SPI2 these are pins PB13 [CLK], PB14 [MISO] and PB15 [MOSI] PB9 for NSS
	//code to enable AF - SPI2
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;		//enable clock to GPIOB
	//PB13
	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL13);
	GPIOB->AFR[1] |= (5<< GPIO_AFRH_AFSEL13_Pos);	//enable SPI CLK to PB13
	GPIOB->MODER &= ~(GPIO_MODER_MODE13);
	GPIOB->MODER |= GPIO_MODER_MODE13_1;			//MODER5[1:0] = 10b, alternate function
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR13);
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR13_1;	// 10, 50Mhz speed, plenty
	//PB13 must be pulled down because CPOL = 0
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD13);
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD13_1;
	//PB14
	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL14);
	GPIOB->AFR[1] |= (5<< GPIO_AFRH_AFSEL14_Pos);	//enable MISO to PB14
	GPIOB->MODER &= ~(GPIO_MODER_MODE14);
	GPIOB->MODER |= GPIO_MODER_MODE14_1;			//MODER5[1:0] = 10b, alternate function
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR14);
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR14_1;	// 10, 50Mhz speed, plenty
	//PB15
	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL15);
	GPIOB->AFR[1] |= (5<< GPIO_AFRH_AFSEL15_Pos);	//enable MOSI to PB15
	GPIOB->MODER &= ~(GPIO_MODER_MODE15);
	GPIOB->MODER |= GPIO_MODER_MODE15_1;			//MODER5[1:0] = 10b, alternate function
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR15);
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR15_1;	// 10b, 50Mhz speed, plenty
	//PB9
	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL9);
	GPIOB->AFR[1] |= (5<< GPIO_AFRH_AFSEL9_Pos);	//enable NSS to PB9
	GPIOB->MODER &= ~(GPIO_MODER_MODE9);
	GPIOB->MODER |= GPIO_MODER_MODE9_1;				//MODER5[1:0] = 10 bin, Alternate function
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR9);
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9_1;		// 10, 25Mhz speed, plenty for SS
	//PB9 must be pulled up becuase a low signal enables it
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD9);
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD9_0;

	//TODO- make this faster by making this all a single bit assignment
	SPI2->CR1 &= ~(SPI_CR1_SPE);		//disable SPI first
	SPI2->CR1 |= SPI_CR1_CPHA;			//CPHA to 1 (capture on falling edge)
	SPI2->CR1 &= ~(SPI_CR1_CPOL);		//CPOL to 0 (clock 0 when idle)
	SPI2->CR1 &= ~(SPI_CR1_MSTR);		//MSTR bit to 0, slave mode selected

	//baud rate controls dont matter, slave mode (010 for 42/8 = about 5Mhz in master)
	//TODO- maybe go to 42/4 = 10.5Mhz, can the econder handle it?
	/*
	 * 	000: fPCLK/2     001: fPCLK/4     010: fPCLK/8      011: fPCLK/16
	 *	100: fPCLK/32   101: fPCLK/64   110: fPCLK/128   111: fPCLK/256
	 */

	SPI2->CR1 &= ~(SPI_CR1_LSBFIRST);	//MSB first data transmission (LSBFIRST to 0)
	SPI2->CR1 &= ~(SPI_CR1_SSM);		//SSOE = 0 and SSM = 0 to act as normal slave device
	SPI2->CR1 &= ~(SPI_CR1_RXONLY);		//enable full duplex (Rx and Tx)
	SPI2->CR1 |= SPI_CR1_DFF;			//16 bit data length

	//no CRC stuff, leave reset

	SPI2->CR1 &= ~(SPI_CR1_BIDIOE);		//we want full duplex
	SPI2->CR1 &= ~(SPI_CR1_BIDIMODE);	//we want full duplex

	SPI2->CR2 = 0;
	SPI2->CR2 |= SPI_CR2_RXNEIE;		//receiver buffer not empty interrupt enable
	SPI2->CR2 &= ~(SPI_CR2_SSOE);		//SSOE = 0 and SSM = 0 to act as normal slave device

	NVIC_SetPriority (SPI2_IRQn, 1);            // enable interupts
	NVIC_EnableIRQ (SPI2_IRQn);

	//config done! :D
	SPI2->CR1 |= SPI_CR1_SPE;			//enable the SPI peripheral
}

void SPI2_IRQHandler(void){
	if (SPI2->SR & SPI_SR_RXNE){
		//got data
		uint16_t data = SPI2->DR;
		if ((data >> 14) == 2){			//header of first (MSB) packet
			scary_packet = data << 16;
		}
		else{
			scary_packet |= data;		//then append to scary_packet
			GPIOC->ODR ^= (1<<1);		//toggle ye olde debug LED
		}
	}
}

void SPIwrite(uint16_t val)
{

}

uint16_t SPIread()
{
	// 2 16 bit packets*:

}
