/*
 * stm32411xx.h
 *
 *  Created on: May 8, 2023
 *      Author: arul
 */

#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

#include <stdint.h>

#define LOW 0
#define HIGH 1

#define DISABLE LOW
#define ENABLE  HIGH

#define UNSET LOW
#define SET   HIGH

#define PERIPH_BASEADDR 0x40000000

#define APB1PERIPH_BASEADDR PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR (PERIPH_BASEADDR + 0x10000)
#define AHB1PERIPH_BASEADDR (PERIPH_BASEADDR + 0x20000)
#define AHB2PERIPH_BASEADDR (PERIPH_BASEADDR + 0x10000000)

#define RCC_BASEADDR (AHB1PERIPH_BASEADDR + 0x3800)
#define HSI_CLK_FREQ 16000000UL

typedef struct {
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t RES1[2];
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t RES2[2];
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t RES3[2];
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	volatile uint32_t RES4[2];
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t RES5[2];
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	volatile uint32_t RES6[2];
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	volatile uint32_t RES7[2];
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
	volatile uint32_t RES8;
	volatile uint32_t DCKCFGR;
} RCC_RegDef_t;

#define RCC ((RCC_RegDef_t *) RCC_BASEADDR)


uint32_t GetSystemClockValue(void);

#include "stm32f411xx_gpio_driver.h"
#include "stm32f411xx_spi_driver.h"

#endif /* INC_STM32F411XX_H_ */
