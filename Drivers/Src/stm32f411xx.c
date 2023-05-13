
#include "stm32f411xx.h"

uint32_t SysClkFreq = 0;
uint32_t AHB1ClkFreq = 0;

uint16_t AHB_Prescalar[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB_Prescalar[4] = {2, 4, 8, 16};

uint32_t getSystemClockFreq(uint8_t recalculate) {
	if (SysClkFreq > 0 && recalculate == 0) {
		return SysClkFreq;
	}

	RCC_RegDef_t *pRCC = RCC;
	uint8_t clkSrc = (pRCC->CFGR >> RCC_CFGR_SWS) & 0x3;

	if (clkSrc == SYSCLK_HSI) {
		SysClkFreq = HSI_CLK_FREQ;
	} else if (clkSrc == SYSCLK_HSE) {
		SysClkFreq = 8000000UL;
	} else if (clkSrc == SYSCLK_PLL) {
		uint32_t pllsrcFreq;
		if (pRCC->PLLCFGR & (1 << RCC_PLLCFGR_PLLSRC)) {
			pllsrcFreq = 8000000UL;
		} else {
			pllsrcFreq = HSI_CLK_FREQ;
		}

		SysClkFreq = (pllsrcFreq * ((pRCC->PLLCFGR >> RCC_PLLCFGR_PLLN) & 0x1FF)) / ((pRCC->PLLCFGR & 0x3F) * ((pRCC->PLLCFGR >> RCC_PLLCFGR_PLLP) & 0x3));
	}

	return SysClkFreq;
}

uint32_t getAHB1ClockFreq(uint8_t recalculate) {
	if (AHB1ClkFreq > 0 && recalculate == 0) {
		return AHB1ClkFreq;
	}

	RCC_RegDef_t *pRCC = RCC;
	uint16_t prescalar = ((pRCC->CFGR >> RCC_CFGR_HPRE) & 0xF);
	// system clock not divided if pre scalar value is less than 8
	if (prescalar < 8) {
		prescalar = 1;
	} else {
		prescalar  = AHB_Prescalar[prescalar - 8];
	}

	AHB1ClkFreq = getSystemClockFreq(recalculate) / prescalar;

	return AHB1ClkFreq;
}

uint32_t getAPBClockFreq(uint8_t periperal, uint8_t recalculate) {
	RCC_RegDef_t *pRCC = RCC;
	uint8_t prescalar;

	if (periperal == APB_LOW_SPEED) {
		prescalar = ((pRCC->CFGR >> RCC_CFGR_PPRE1) & 0x7);
	} else {
		prescalar = ((pRCC->CFGR >> RCC_CFGR_PPRE2) & 0x7);
	}

	// AHB clock not divided if pre scalar value is less than 8
	if (prescalar < 4) {
		prescalar = 1;
	} else {
		prescalar  = APB_Prescalar[prescalar - 4];
	}

	return getAHB1ClockFreq(recalculate) / prescalar;
}
