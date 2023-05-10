/*
 * stm32f411xx_spi_driver.h
 *
 *  Created on: May 9, 2023
 *      Author: arul
 */

#ifndef INC_STM32F411XX_SPI_DRIVER_H_
#define INC_STM32F411XX_SPI_DRIVER_H_

#include <stm32f411xx.h>

#define SPI1_BASEADDR (APB2PERIPH_BASEADDR + 0x3000)
#define SPI2_BASEADDR (APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR (APB1PERIPH_BASEADDR + 0x3C00)
#define SPI4_BASEADDR (APB2PERIPH_BASEADDR + 0x3400)
#define SPI5_BASEADDR (APB2PERIPH_BASEADDR + 0x5000)

typedef struct {
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;
} SPI_RegDef_t;

#define SPI1 ((SPI_RegDef_t *) SPI1_BASEADDR)
#define SPI2 ((SPI_RegDef_t *) SPI2_BASEADDR)
#define SPI3 ((SPI_RegDef_t *) SPI3_BASEADDR)
#define SPI4 ((SPI_RegDef_t *) SPI4_BASEADDR)
#define SPI5 ((SPI_RegDef_t *) SPI5_BASEADDR)

#define SPI_CR1_CPHA  	 0
#define SPI_CR1_CPOL  	 1
#define SPI_CR1_MSTR  	 2
#define SPI_CR1_BR   	 3
#define SPI_CR1_SPE      6
#define SPI_CR1_LSBFIRST 7
#define SPI_CR1_SSM  	 9
#define SPI_CR1_DFF 	 11
#define SPI_CR1_RXONLY	 10
#define SPI_CR1_BIDIOE   14
#define SPI_CR1_BIDIMODE 15

#define SPI_CR2_SSOE 2

#define SPI_SR_RXNE  0
#define SPI_SR_TXE   1
#define SPI_SR_BSY   7

#define SPI_FRAME_SIZE_8  0
#define SPI_FRAME_SIZE_16 1

#define SPI_SSM_DIS 0
#define SPI_SSM_EN  1

#define SPI_MSB_FIRST 0
#define SPI_LSB_FIRST 1

#define SPI_IDLE_CPOL_LOW  0
#define SPI_IDLE_CPOL_HIGH 1

#define SPI_CPHA_FIRST  0
#define SPI_CPHA_SECOND 1

#define SPI_BAUD_RATE_2   0
#define SPI_BAUD_RATE_4   1
#define SPI_BAUD_RATE_8   2
#define SPI_BAUD_RATE_16  3
#define SPI_BAUD_RATE_32  4
#define SPI_BAUD_RATE_64  5
#define SPI_BAUD_RATE_128 6
#define SPI_BAUD_RATE_256 7

#define SPI_MODE_SLAVE  0
#define SPI_MODE_MASTER 1

#define SPI_FULL_DUPLEX 	0
#define SPI_HALF_DUPLEX 	1
#define SPI_SIMPLEX_RX_ONLY 2

typedef struct {
	uint8_t mode;
	uint8_t frameSize;
	uint8_t ssm;
	uint8_t txnBitPos;
	uint8_t baudRate;
	uint8_t clkPolarity;
	uint8_t clkPhase;
	uint8_t txnMode;
	SPI_RegDef_t *pSPIx;
} SPI_Config_t;


#define SPI1_PERIPH_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_PERIPH_EN() (RCC->APB1ENR |= (1 << 14))
#define SPI3_PERIPH_EN() (RCC->APB1ENR |= (1 << 15))
#define SPI4_PERIPH_EN() (RCC->APB2ENR |= (1 << 13))
#define SPI5_PERIPH_EN() (RCC->APB2ENR |= (1 << 20))

#define SPI1_PERIPH_DIS() (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PERIPH_DIS() (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PERIPH_DIS() (RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PERIPH_DIS() (RCC->APB2ENR &= ~(1 << 13))
#define SPI5_PERIPH_DIS() (RCC->APB2ENR &= ~(1 << 20))

#define SPI1_REG_RESET() do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET() do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET() do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); }while(0)
#define SPI4_REG_RESET() do{ (RCC->APB2RSTR |= (1 << 13)); (RCC->APB2RSTR &= ~(1 << 13)); }while(0)
#define SPI5_REG_RESET() do{ (RCC->APB2RSTR |= (1 << 20)); (RCC->APB2RSTR &= ~(1 << 20)); }while(0)


void SPIx_PeriphClkControl(SPI_RegDef_t *pSPIx, uint8_t enOrDis);
void SPIx_PeriphControl(SPI_RegDef_t *pSPIx, uint8_t enOrDis);
void SPIx_Init(SPI_Config_t *pSPIConfig);
void SPIx_DeInit(SPI_RegDef_t *pSPIx);
void  SPIx_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t enOrDis);
uint8_t SPIx_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t flagPos);
void SPIx_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuf, uint32_t len);

#endif /* INC_STM32F411XX_SPI_DRIVER_H_ */
