/*
 * stm32f411xx_i2c_driver.c
 */

#include "stm32f411xx_i2c_driver.h"

void I2Cx_PeriphClkControl(I2C_RegDef_t *pI2Cx, uint8_t enOrDis) {
	if (enOrDis == ENABLE) {
		if (pI2Cx == I2C1) {
			I2C1_PERIPH_EN();
		} else if (pI2Cx == I2C2) {
			I2C2_PERIPH_EN();
		} else if (pI2Cx == I2C3) {
			I2C3_PERIPH_EN();
		}
	} else {
		if (pI2Cx == I2C1) {
			I2C1_PERIPH_DIS();
		} else if (pI2Cx == I2C2) {
			I2C2_PERIPH_DIS();
		} else if (pI2Cx == I2C3) {
			I2C3_PERIPH_DIS();
		}
	}
}

void I2Cx_PeriphControl(I2C_RegDef_t *pI2Cx, uint8_t enOrDis) {
	if (enOrDis == ENABLE) {
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	} else {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

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
		tempReg |= (1 << I2C_CCR_FS);
		tempConstant = 3;
		tempBusSpeed = 200000UL;
	} else if (pI2CxConfig->Mode == I2C_MODE_MASTER_FM_16_9) {
		tempReg |= (1 << I2C_CCR_FS);
		tempReg |= (1 << I2C_CCR_DUTY);
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

void I2Cx_DeInit(I2C_RegDef_t *pI2Cx) {
	if (pI2Cx == I2C1) {
		I2C1_PERIPH_RESET();
	} else if (pI2Cx == I2C2) {
		I2C2_PERIPH_RESET();
	} else if (pI2Cx == I2C3) {
		I2C3_PERIPH_RESET();
	}
}

void I2Cx_GenerateStart(I2C_RegDef_t *pI2Cx) {
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

uint8_t I2Cx_GetFlagStatus(uint32_t *pI2CxReg, uint8_t flag) {
	if (*pI2CxReg & (1 << flag)) {
		return SET;
	}

	return RESET;
}

