
#include "stm32f411xx.h"

uint16_t AHBPrescaler[] = {2, 4, 8, 16, 32, 64, 128, 256, 512};
uint8_t APBPrescalar[] = {2, 4, 8, 16};

uint32_t GetSysClk(void) {
	RCC_RegDef_t *pRCC = RCC;
	uint32_t sysClk = 0;
	uint8_t clockSource = (pRCC->CFGR >> 2) & 0x3;
	if (clockSource == 0) {
		sysClk = HSI_CLK_FREQ;
	} else if (clockSource == 2) {
		sysClk = 8000000;
	} else {
		sysClk = (HSI_CLK_FREQ * ((pRCC->PLLCFGR >> 6) & 0x1FF)) / (((pRCC->PLLCFGR >> 16) & 0x3) * (pRCC->PLLCFGR & 0x3F));
	}

	return sysClk;
}


uint32_t GetAPB1Clk(void) {
	RCC_RegDef_t *pRCC = RCC;
	uint32_t sysClk = GetSysClk(), clkVal = 0;

	// AHB prescaler calculation
	uint8_t prescalerTemp = ((pRCC->CFGR >> 4) & 0xF) - 8;
	clkVal = sysClk / AHBPrescaler[prescalerTemp];

	// APB prescaler calculation
	prescalerTemp = ((pRCC->CFGR >> 10) & 0x7) - 4;
	clkVal /= prescalerTemp;

	return clkVal;
}



uint32_t GetAPB2Clk(void) {
	RCC_RegDef_t *pRCC = RCC;
	uint32_t sysClk = GetSysClk(), clkVal = 0;

	// AHB pre scaler calculation
	uint8_t prescalerTemp = ((pRCC->CFGR >> 4) & 0xF) - 8;
	clkVal = sysClk / AHBPrescaler[prescalerTemp];

	// APB pre scaler calculation
	prescalerTemp = ((pRCC->CFGR >> 13) & 0x7) - 4;
	clkVal /= prescalerTemp;

	return clkVal;
}


