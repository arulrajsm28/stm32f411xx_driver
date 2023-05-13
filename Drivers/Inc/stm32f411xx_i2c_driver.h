/*
 * stm32f411xx_i2c_driver.h
 *
 *  Created on: 13-May-2023
 *      Author: arul
 */

#ifndef INC_STM32F411XX_I2C_DRIVER_H_
#define INC_STM32F411XX_I2C_DRIVER_H_

#include "stm32f411xx.h"

#define I2C1_BASEADDR (APB1PERIPH_BASEADDR + 0x5400U)
#define I2C2_BASEADDR (APB1PERIPH_BASEADDR + 0x5800U)
#define I2C3_BASEADDR (APB1PERIPH_BASEADDR + 0x5C00U)

#define I2C_APB_PERIPH APB_LOW_SPEED

typedef struct {
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t OAR1;
	volatile uint32_t OAR2;
	volatile uint32_t DR;
	volatile uint32_t SR1;
	volatile uint32_t SR2;
	volatile uint32_t CCR;
	volatile uint32_t TRISE;
	volatile uint32_t FLTR;
} I2C_RegDef_t;

#define I2C1 ((I2C_RegDef_t *) I2C1_BASEADDR)
#define I2C2 ((I2C_RegDef_t *) I2C2_BASEADDR)
#define I2C3 ((I2C_RegDef_t *) I2C3_BASEADDR)

typedef struct {
	uint16_t Address;
	uint8_t  Mode;
	I2C_RegDef_t *pI2Cx;
} I2C_Config_t;

#define I2C_CR1_PE	  0
#define I2C_CR1_START 8
#define I2C_CR1_STOP  9
#define I2C_CR1_ACK   10

#define I2C_CR2_FREQ 0

#define I2C_OAR1_ADDR_7 1

#define I2C_MODE_SLAVE			0
#define I2C_MODE_MASTER_SM      1
#define I2C_MODE_MASTER_FM_2    2
#define I2C_MODE_MASTER_FM_16_9 3

void I2Cx_Init(I2C_Config_t *pI2CxConfig);

#endif /* INC_STM32F411XX_I2C_DRIVER_H_ */
