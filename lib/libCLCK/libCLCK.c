//
// Created by thari on 18/08/2021.
//

#include "libCLCK.h"

/*
 	FLASH wait states:
	5 wait states at 168 Mhz @ 3.3v

 Increasing the CPU frequency
	1. Program the new number of wait states to the LATENCY bits in the FLASH_ACR
	register
	2. Check that the new number of wait states is taken into account to access the Flash
	memory by reading the FLASH_ACR register
	3. Modify the CPU clock source by writing the SW bits in the RCC_CFGR register
	4. If needed, modify the CPU clock prescaler by writing the HPRE bits in RCC_CFGR
	5. Check that the new CPU clock source or/and the new CPU clock prescaler value is/are
	taken into account by reading the clock source status (SWS bits) or/and the AHB
	prescaler value (HPRE bits), respectively, in the RCC_CFGR register.

 Decreasing the CPU frequency
	1. Modify the CPU clock source by writing the SW bits in the RCC_CFGR register
	2. If needed, modify the CPU clock prescaler by writing the HPRE bits in RCC_CFGR
	3. Check that the new CPU clock source or/and the new CPU clock prescaler value is/are
	taken into account by reading the clock source status (SWS bits) or/and the AHB
	prescaler value (HPRE bits), respectively, in the RCC_CFGR register
	4. Program the new number of wait states to the LATENCY bits in FLASH_ACR
	5. Check that the new number of wait states is used to access the Flash memory by
	reading the FLASH_ACR register

 Note: A change in CPU clock configuration or wait state (WS) configuration may not be effective
 straight away. To make sure that the current CPU clock frequency is the one you have
 configured, you can check the AHB prescaler factor and clock source status values. To
 make sure that the number of WS you have programmed is effective, you can read the
 FLASH_ACR register.

 The maximum frequency of the AHB domain is
 168 MHz. The maximum allowed frequency of the high-speed APB2 domain is 84 MHz. The
 maximum allowed frequency of the low-speed APB1 domain is 42 MHz

*/

/*
 * SO, steps are as follows-
 * disable stuff
 * enable power interface clock
 * set voltage scale to 1 for max frequency
 * enable clock security
 * turn on HSE
 * -while waiting till HSE settles:
 *   configure pll
 * WHen HSE is stable:
 *   Select HSE as RTC scource
 *   enable pll
 * -while waiting till pll stable:
 *   set RTC clock = HSE / 8 --no need for now
 *   set AHB, APB1 and ABP2 prescalers, 1, 4, 2 respectively for 168Mhz
 *   Configure Flash:
 *     prefetch enable (ACR:bit 8)
 *     instruction cache enable (ACR:bit 9)
 *     data cache enable (ACR:bit 10)
 *     set latency to 2 wait states (ARC:bits 2:0)
 * WHen PLL is stable:
 *   switch system clock scource to pll
 *   wait till the switch is confirmed
 *   Maybe turn of HSI now?
 */
int set_system_clock_to_168Mhz()		//TODO- also maybe have a low speed option
{
	//reset sysclk to defaults (HSI) while we do our things
	//RCC->CFGR = 0x0000;
	// no Necessary wait states for Flash for low speeds
	//FLASH->ACR &= ~(FLASH_ACR_LATENCY);
	//FLASH->ACR |= FLASH_ACR_LATENCY_0WS;
	//disable pll
	RCC->CR &= ~(RCC_CR_PLLON);

	//enable power interface clock
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	//set voltage scale to 1 for max frequency
	PWR->CR |= PWR_CR_VOS;

	//enable HSE clock security
	RCC->CR |= RCC_CR_CSSON;
	//enable HSE
	RCC->CR |= RCC_CR_HSEON;
	//-while waiting till HSE settles:

	//configure pll
	//So external crystal is 12Mhz - heavens that was difficult to find lol
	//VCO must give 336Mhz so that PLLP can be 2 and PLL outputs 168Mhz
	//PLLM to 6 so VCO gets 2Mhz as required
	//therefore, PLLN must be 336/2 = 168 (VCO gets 2mhz, should bump that to 336Mhz)
	//SO: ((12Mhz / 6) * 168) / 2 = 168Mhz
	//also, for 48Mhz, divide 336 by 7, therfore PLLQ is 7
	//PLLM = 6, PLLN = 168, PLLP = 2, PLLQ = 7
	uint32_t PLLM = 6, PLLN = 168, PLLP = 0, PLLQ = 7;
	RCC->PLLCFGR = (PLLM << RCC_PLLCFGR_PLLM_Pos) | (PLLN << RCC_PLLCFGR_PLLN_Pos)
				   | (PLLQ << RCC_PLLCFGR_PLLQ_Pos);
	RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLP);		//PLLP to 2

	// Wait untill HSE settles down
	//while (!(RCC->CR & RCC_CR_HSERDY));
	while ((RCC->CR & RCC_CR_HSERDY)==0);
	//switch PLL scource to HSE
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;
	//enable pll
	RCC->CR |= RCC_CR_PLLON;
	//while waiting for PLL to stabilise

	//set RTC clock = HSE / 8
	//RCC->CFGR &= ~(RCC_CFGR_RTCPRE);	//clear bits
	//RCC->CFGR |= RCC_CFGR_RTCPRE_3;

	//set AHB, APB1 and ABP2 prescalers, 1, 4, 2 respectively for 168Mhz
	RCC->CFGR &= (RCC_CFGR_HPRE);
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;		//set AHB prediv to 1

	RCC->CFGR &= (RCC_CFGR_PPRE1);
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;		//set APB1 prediv to 4

	RCC->CFGR &= (RCC_CFGR_PPRE2);
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;		//set APB2 prediv to 2

	//configure flash
	FLASH->ACR |= FLASH_ACR_PRFTEN;		//enable prefetch
	FLASH->ACR |= FLASH_ACR_ICEN;		//enable instruction Cache
	FLASH->ACR |= FLASH_ACR_DCEN;		//enable data Cache
	// 5 Necessary wait states for Flash @ 168Mhz
	FLASH->ACR &= ~(FLASH_ACR_LATENCY);
	FLASH->ACR |= FLASH_ACR_LATENCY_5WS;

	//while (!(FLASH->ACR | FLASH_ACR_LATENCY_5WS));		//wait till they are accepted

	//wait for PLL to stabilise
	//while (!(RCC->CR & RCC_CR_PLLRDY));
	while ((RCC->CR & RCC_CR_PLLRDY) == 0);
	// Finally, choose PLL as the system clock
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	//wait for success
	//while (!(RCC->CFGR & RCC_CFGR_SWS_PLL));
	while ((RCC->CFGR & RCC_CFGR_SWS_PLL) == 0);
	//TODO- Turn off HSI clock

	//The RCC feeds the external clock of the Cortex System Timer (SysTick) with the AHB clock
	//(HCLK) divided by 8. The SysTick can work either with this clock or with the Cortex clock
	//(HCLK), configurable in the SysTick control and status register.
	SystemCoreClock = 168000000;
	//SystemCoreClockUpdate();

	//setup systick
	if (SysTick_Config (112000)) { // SysTick timer to get freq of 1.5Khz
		return 1;
	}
	NVIC_SetPriority (SysTick_IRQn, 0);            // Set Timer priority
	NVIC_EnableIRQ (SysTick_IRQn);

	return 0;
}

void SysTick_Handler (void) {                    // SysTick Interrupt Handler
	msTicks++;                                     // Increment Counter
}

void WaitForTick (void)  {
	uint32_t curTicks;
	curTicks = msTicks;                            // Save Current SysTick Value
	while (msTicks == curTicks)  {                 // Wait for next SysTick Interrupt
		__WFI();                                    // Power-Down until next Event/Interrupt
	}
}

//TODO- prevent rollover bug
void ms_delay(uint32_t ms)
{
	uint32_t curTicks;
	curTicks = msTicks;
	while (msTicks < (curTicks + ms)){
		__WFI();
	}
}