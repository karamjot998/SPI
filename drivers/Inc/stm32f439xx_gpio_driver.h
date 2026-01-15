/*
 * stm32f439xx_gpio_driver.h
 *
 *  Created on: Jan 10, 2026
 *      Author: binni
 */

#ifndef INC_STM32F439XX_GPIO_DRIVER_H_
#define INC_STM32F439XX_GPIO_DRIVER_H_

#include "stm32f439xx.h"

/*
 * Configuration structure for a GPIO Pin
 */
typedef struct{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/*
 * Hnadle structure for the GPIO Pin
 */
typedef struct {

	// pointer to hold the base address of the GPIO Peripheral
	GPIO_RegDef_t *pGPIOx;

	// structure to hold the configuration of pin
	GPIO_PinConfig_t GPIO_PinConfig;



}GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBERS
 */
#define GPIO_PIN_NO_0			0
#define GPIO_PIN_NO_1			1
#define GPIO_PIN_NO_2			2
#define GPIO_PIN_NO_3			3
#define GPIO_PIN_NO_4			4
#define GPIO_PIN_NO_5			5
#define GPIO_PIN_NO_6			6
#define GPIO_PIN_NO_7			7
#define GPIO_PIN_NO_8			8
#define GPIO_PIN_NO_9			9
#define GPIO_PIN_NO_10			10
#define GPIO_PIN_NO_11			11
#define GPIO_PIN_NO_12			12
#define GPIO_PIN_NO_13			13
#define GPIO_PIN_NO_14			14
#define GPIO_PIN_NO_15			15

/*
 *  GPIO pin Possible modes
 */
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALT 		2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6

/*
 *  GPIO PIN POSSIBLE OUTPUT TYPE
 */
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OP		1

/*
 * GPIO POSSIBLE OUTPUT SPEEDS
 */
#define GPIO_OP_SPEED_LOW			0
#define GPIO_OP_SPEED_MEDIUM		1
#define GPIO_OP_SPEED_FAST			2
#define GPIO_OP_SPEED_HIGH			3

/*
 * GPIO POSSIBLE PULL UP/ PULL DOWN CONFIGURATIONS
 */
#define GPIO_NO_PUPD		0
#define GPIO_PU				1
#define GPIO_PD				2






/*********************************************
 *         APIs Support by this driver
 *********************************************/

/*
 *  Peripheral Clock Setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t ENOrDi);

/*
 *  Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DedInit(GPIO_RegDef_t *pGPIOx);

/*
 * data read write
 */
uint8_t GPIO_ReadPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum);
uint16_t GPIO_ReadPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WritePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum, uint8_t value);
void GPIO_WritePort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum);

/*
 * GPIO interrupt handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNum, uint8_t ENOrDI);
void GPIO_IRQPriorityConfig(uint8_t IRQNum, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNum);

#endif /* INC_STM32F439XX_GPIO_DRIVER_H_ */
