/*
 * stm32f411xx_spi_driver.c
 *
 */

#include "stm32f411xx_spi_driver.h"

void SPIx_PeriphClkControl(SPI_RegDef_t *pSPIx, uint8_t enOrDis) {
	if (enOrDis == ENABLE) {
		if (pSPIx == SPI1) {
			SPI1_PERIPH_EN();
		} else if (pSPIx == SPI2) {
			SPI2_PERIPH_EN();
		} else if (pSPIx == SPI3) {
			SPI3_PERIPH_EN();
		} else if (pSPIx == SPI4) {
			SPI4_PERIPH_EN();
		} else if (pSPIx == SPI5) {
			SPI5_PERIPH_EN();
		}
	} else {
		if (pSPIx == SPI1) {
			SPI1_PERIPH_DIS();
		} else if (pSPIx == SPI2) {
			SPI2_PERIPH_DIS();
		} else if (pSPIx == SPI3) {
			SPI3_PERIPH_DIS();
		} else if (pSPIx == SPI4) {
			SPI4_PERIPH_DIS();
		} else if (pSPIx == SPI5) {
			SPI5_PERIPH_DIS();
		}
	}
}

void SPIx_Init(SPI_Config_t *pSPIConfig) {
	uint32_t tempReg = 0;
	tempReg |= (pSPIConfig->frameSize & 1) << SPI_CR1_DFF;
	tempReg |= (pSPIConfig->clkPhase & 1) << SPI_CR1_CPHA;
	tempReg |= (pSPIConfig->clkPolarity & 1) << SPI_CR1_CPOL;
	tempReg |= (pSPIConfig->baudRate & 7) << SPI_CR1_BR;
	tempReg |= (pSPIConfig->ssm & 1) << SPI_CR1_SSM;
	tempReg |= (pSPIConfig->txnBitPos &1) << SPI_CR1_LSBFIRST;

	pSPIConfig->pSPIx->CR1 |= tempReg;
}

/**
 * Disable SPIx
 */
void SPIx_DeInit(SPI_RegDef_t *pSPIx) {
	if (pSPIx == SPI1) {
		SPI1_REG_RESET();
	} else if (pSPIx == SPI2) {
		SPI2_REG_RESET();
	} else if (pSPIx == SPI3) {
		SPI3_REG_RESET();
	} else if (pSPIx == SPI4) {
		SPI4_REG_RESET();
	} else if (pSPIx == SPI5) {
		SPI5_REG_RESET();
	}
}
/**
 * setting device mode
 * mode = MASTER|SLAVE
 */
void SPIx_SetDeviceMode(SPI_RegDef_t *pSPIx, uint8_t mode) {
	pSPIx->CR1 |= (mode & 1) << SPI_CR1_MSTR;
}



