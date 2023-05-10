/*
 * stm32411xx_gpio_driver.c
 *
 *      Author: arul
 */


#include <stm32f411xx_gpio_driver.h>

void GPIOx_PeriphClkControl(GPIO_RegDef_t *pGPIOx, uint8_t enOrDis) {
	if (enOrDis == ENABLE) {
		if (pGPIOx == GPIOA) {
			GPIOA_PERIPH_EN();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PERIPH_EN();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PERIPH_EN();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PERIPH_EN();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PERIPH_EN();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PERIPH_EN();
		}
	} else if (enOrDis == DISABLE) {
		if (pGPIOx == GPIOA) {
			GPIOA_PERIPH_DIS();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PERIPH_DIS();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PERIPH_DIS();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PERIPH_DIS();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PERIPH_DIS();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PERIPH_DIS();
		}
	}

}

void GPIOx_init(GPIO_Config_t *pGpioConfig) {
	GPIOx_PeriphClkControl(pGpioConfig->pGPIOx, ENABLE);

	uint32_t temp = 0;
	temp |= (pGpioConfig->InputMode & 0x3) << (2 * pGpioConfig->PinNumber);
	pGpioConfig->pGPIOx->MODER &= ~(0x3 << (2 * pGpioConfig->PinNumber));
	pGpioConfig->pGPIOx->MODER |= temp;

	if (pGpioConfig->InputMode == GPIO_MODE_OUTPUT || pGpioConfig->InputMode == GPIO_MODE_ALTFN) {
		temp = 0;
		temp |= (pGpioConfig->OutputType & 0x1) << (pGpioConfig->PinNumber);
		pGpioConfig->pGPIOx->OTYPER &= ~(0x1 << pGpioConfig->PinNumber);
		pGpioConfig->pGPIOx->OTYPER |= temp;

		temp = 0;
		temp |= (pGpioConfig->Speed & 0x3) << (2 * pGpioConfig->PinNumber);
		pGpioConfig->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGpioConfig->PinNumber));
		pGpioConfig->pGPIOx->OSPEEDR |= temp;
	}

	temp = 0;
	temp |= (pGpioConfig->PUPD & 0x3) << (2 * pGpioConfig->PinNumber);
	pGpioConfig->pGPIOx->PUPDR &= ~(0x3 << (2 * pGpioConfig->PinNumber));
	pGpioConfig->pGPIOx->PUPDR |= temp;

	if (pGpioConfig->InputMode == GPIO_MODE_ALTFN) {
		temp = 0;
		uint8_t altRegSel = pGpioConfig->PinNumber / 8;
		uint8_t tempLoc = pGpioConfig->PinNumber % 8;
		temp |= (pGpioConfig->AltFun & 0xF) << (4 * tempLoc);
		pGpioConfig->pGPIOx->AFR[altRegSel] &= ~(0xF << (4 * tempLoc));
		pGpioConfig->pGPIOx->AFR[altRegSel] |= temp;
	}
}

void GPIOx_WriteOutput(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value) {
	if (value == SET) {
		pGPIOx->ODR |= (1 << pinNumber);
	} else if (value == RESET) {
		pGPIOx->ODR &= ~(1 << pinNumber);
	}
}

void GPIOx_ToggleOutput(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {
	pGPIOx->ODR ^= (1 << pinNumber);
}

uint8_t GPIOx_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {
	if (pGPIOx->IDR & (1 << pinNumber)) {
		return SET;
	}

	return RESET;
}

uint16_t GPIOx_ReadInputPort(GPIO_RegDef_t *pGPIOx) {
	return (uint16_t)pGPIOx->IDR;
}


