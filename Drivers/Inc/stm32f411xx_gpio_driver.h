/*
 * stm32411xx_gpio_driver.h
 *
 *  Created on: May 8, 2023
 *      Author: arul
 */

#ifndef INC_STM32F411XX_GPIO_DRIVER_H_
#define INC_STM32F411XX_GPIO_DRIVER_H_

#include "stm32411xx.h"

#define GPIOA_BASEADDR (AHB1PERIPH_BASEADDR + 0x0)
#define GPIOB_BASEADDR (AHB1PERIPH_BASEADDR + 0x400)
#define GPIOC_BASEADDR (AHB1PERIPH_BASEADDR + 0x800)
#define GPIOD_BASEADDR (AHB1PERIPH_BASEADDR + 0xC00)
#define GPIOE_BASEADDR (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOH_BASEADDR (AHB1PERIPH_BASEADDR + 0x1C00)

typedef struct {
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t AFR[2];
} GPIO_RegDef_t;

#define GPIOA ((GPIO_RegDef_t *) GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t *) GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t *) GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t *) GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t *) GPIOE_BASEADDR)
#define GPIOH ((GPIO_RegDef_t *) GPIOH_BASEADDR)

typedef struct {
	GPIO_RegDef_t *pGPIOx;
	uint8_t PinNumber;
	uint8_t InputMode;
	uint8_t OutputType;
	uint8_t Speed;
	uint8_t PUPD;
	uint8_t AltFun;
} GPIO_Config_t;

#define GPIO_PIN_0  0
#define GPIO_PIN_1  1
#define GPIO_PIN_2  2
#define GPIO_PIN_3  3
#define GPIO_PIN_4  4
#define GPIO_PIN_5  5
#define GPIO_PIN_6  6
#define GPIO_PIN_7  7
#define GPIO_PIN_8  8
#define GPIO_PIN_9  9
#define GPIO_PIN_10 10
#define GPIO_PIN_11 11
#define GPIO_PIN_12 12
#define GPIO_PIN_13 13
#define GPIO_PIN_14 14
#define GPIO_PIN_15 15

#define GPIO_MODE_INPUT  0
#define GPIO_MODE_OUTPUT 1
#define GPIO_MODE_ALTFN  2
#define GPIO_MODE_ANALOG 3

#define GPIO_OP_PUPD 0
#define GPIO_OP_OD   1

#define GPIO_SPEED_LOW 	0
#define GPIO_SPEED_MED  1
#define GPIO_SPEED_FAST 2
#define GPIO_SPEED_HIGH 3

#define GPIO_NO_PUPD 	0
#define GPIO_PULL_UP    1
#define GPIO_PULL_DOWN  2

#define GPIOA_PERIPH_EN() (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PERIPH_EN() (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PERIPH_EN() (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PERIPH_EN() (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PERIPH_EN() (RCC->AHB1ENR |= (1 << 4))
#define GPIOH_PERIPH_EN() (RCC->AHB1ENR |= (1 << 5))

#define GPIOA_PERIPH_DIS() (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PERIPH_DIS() (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PERIPH_DIS() (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PERIPH_DIS() (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PERIPH_DIS() (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOH_PERIPH_DIS() (RCC->AHB1ENR &= ~(1 << 5))

/*
 *  Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); }while(0)

void GPIOx_PeriphClkControl(GPIO_RegDef_t *pGPIOx, uint8_t enOrDis);
void GPIOx_init(GPIO_Config_t *pGpioConfig);

void GPIOx_WriteOutput(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value);
void GPIOx_ToggleOutput(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);

uint8_t GPIOx_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
uint16_t GPIOx_ReadInputPort(GPIO_RegDef_t *pGPIOx);


#endif /* INC_STM32F411XX_GPIO_DRIVER_H_ */
