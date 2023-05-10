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

/**
 * Enable or disable SPI peripheral
 */
void SPIx_PeriphControl(SPI_RegDef_t *pSPIx, uint8_t enOrDis) {
	if (enOrDis == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/**
 * SPI initialization
 */
void SPIx_Init(SPI_Config_t *pSPIConfig) {
	SPIx_PeriphClkControl(pSPIConfig->pSPIx, ENABLE);

	uint32_t tempReg = 0;
	// device mode selection (master
	tempReg |= (pSPIConfig->mode & 1) << SPI_CR1_MSTR;
	// Data frame size selection
	tempReg |= (pSPIConfig->frameSize & 1) << SPI_CR1_DFF;
	// Clock Phase selection
	tempReg |= (pSPIConfig->clkPhase & 1) << SPI_CR1_CPHA;
	// Clock polarity selection
	tempReg |= (pSPIConfig->clkPolarity & 1) << SPI_CR1_CPOL;
	// Baud rate selection
	tempReg |= (pSPIConfig->baudRate & 7) << SPI_CR1_BR;
	// software slave select management selection
	tempReg |= (pSPIConfig->ssm & 1) << SPI_CR1_SSM;
	// LSBFIRST or MSBFIRST bit selection
	tempReg |= (pSPIConfig->txnBitPos & 1) << SPI_CR1_LSBFIRST;

	if (pSPIConfig->txnMode == SPI_FULL_DUPLEX) {
		tempReg &= ~((1 << SPI_CR1_BIDIMODE) |  (1 << SPI_CR1_RXONLY));
	} else if (pSPIConfig->txnMode == SPI_HALF_DUPLEX) {
		tempReg |= 1 << SPI_CR1_BIDIMODE;
	} else if (pSPIConfig->txnMode == SPI_SIMPLEX_RX_ONLY) {
		tempReg &= ~(1 << SPI_CR1_BIDIMODE);
		tempReg |= 1 << SPI_CR1_RXONLY;
	}

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

void  SPIx_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t enOrDis)
{
	if (enOrDis == ENABLE) {
		pSPIx->CR2 |=  (1 << SPI_CR2_SSOE);
	} else {
		pSPIx->CR2 &=  ~(1 << SPI_CR2_SSOE);
	}
}

uint8_t SPIx_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t flagPos) {
	if (pSPIx->SR & (1 << flagPos)) {
		return SET;
	}

	return RESET;
}

void SPIx_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuf, uint32_t len) {
	while (len) {
		while(SPIx_GetFlagStatus(pSPIx, SPI_SR_TXE) == RESET);

		if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
			pSPIx->DR = *((uint16_t *)pTxBuf);
			len--;
			len--;
			(uint16_t*)pTxBuf++;
		} else {
			pSPIx->DR = *pTxBuf++;
			len--;
		}
	}
}

void SPIx_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuf, uint32_t len) {

}





