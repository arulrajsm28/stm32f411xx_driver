/*
 * stm32f411xx_i2c_driver.c
 */

#include "stm32f411xx_i2c_driver.h"

void I2Cx_Init(I2C_Config_t *pI2CxConfig) {
	uint32_t tempReg = 0, apb1Clk = 0;
	tempReg |= (pI2CxConfig->Address & 0x7F) << I2C_OAR1_ADDR_7;
	pI2CxConfig->pI2Cx->OAR1 |= tempReg;

	if (pI2CxConfig->Mode > I2C_MODE_SLAVE) {
		return;
	}

	tempReg = 0;
	apb1Clk = getAPBClockFreq(I2C_APB_PERIPH, 0);
	tempReg = apb1Clk / 1000000UL;
	pI2CxConfig->pI2Cx->CR2 |= (tempReg & 0x3F);

	tempReg = 0;
	uint8_t tempConstant = 0;
	uint32_t tempBusSpeed = 0;
	if (pI2CxConfig->Mode == I2C_MODE_MASTER_FM_2) {
		tempConstant = 3;
		tempBusSpeed = 200000UL;
	} else if (pI2CxConfig->Mode == I2C_MODE_MASTER_FM_16_9) {
		tempConstant = 25;
		tempBusSpeed = 400000UL;
	} else {
		tempConstant = 2;
		tempBusSpeed = 100000UL;
	}
	tempReg |= ((apb1Clk / (tempConstant * tempBusSpeed)) & 0xFFF);
	pI2CxConfig->pI2Cx->CCR |= tempReg;

	tempReg = 0;
	if (pI2CxConfig->Mode > I2C_MODE_MASTER_SM) {
		pI2CxConfig->pI2Cx->CCR |= tempReg;
		tempReg = ((apb1Clk * 300) / 1000000000UL);
	} else {
		tempReg = ((apb1Clk * 1000) / 1000000000UL);
	}
	pI2CxConfig->pI2Cx->TRISE |= (tempReg & 0x3F);

}

