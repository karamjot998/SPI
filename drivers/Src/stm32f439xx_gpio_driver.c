/*
 * stm32f439xx_gpio_driver.c
 *
 *  Created on: Jan 10, 2026
 *      Author: binni
 */

#include "stm32f439xx_gpio_driver.h"



/*
 *  Peripheral Clock Setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t ENOrDi){

	if(ENOrDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}else if(pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}else if(pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}else if(pGPIOx == GPIOI){
			GPIOI_PCLK_EN();
		}else if(pGPIOx == GPIOJ){
			GPIOJ_PCLK_EN();
		}else if(pGPIOx == GPIOK){
			GPIOK_PCLK_EN();
		}
	}else{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}else if(pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}else if(pGPIOx == GPIOG){
			GPIOG_PCLK_DI();
		}else if(pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}else if(pGPIOx == GPIOI){
			GPIOI_PCLK_DI();
		}else if(pGPIOx == GPIOJ){
			GPIOJ_PCLK_DI();
		}else if(pGPIOx == GPIOK){
			GPIOK_PCLK_DI();
		}
	}

}

/*
 *  Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	uint32_t tmp = 0;
	// Configuring mode of the gpio pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){

		tmp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));
		pGPIOHandle->pGPIOx->MODER |= tmp;

	}else{

		// configuring pin as input for input
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){

			// configure the FISR
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear the corresponding RTSR bit
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){

			// configure the RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			// clear the corresponding FTSR bit
			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			// cofigure both FTSR and RTSR
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		// enable syscfg peripheral
		SYSCFG_PCLK_EN();
		// configure GPIO port selection in SYSCFG_EXTICR
		uint8_t tmp1, tmp2;

		tmp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		tmp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;

		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);          // getting the code for port corresponding to address

		SYSCFG->EXTICR[tmp1] &= ~(0b1111 << (tmp2 * 4));
		SYSCFG->EXTICR[tmp1] |= ( portcode << (tmp2 * 4));
		// clearing EXTI line before enabling interrupt
		EXTI->PR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		// enable the exti interrupt delivery using IMR (Interrupt mask register)
		EXTI->IMR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	tmp = 0;

	// configuring speed of the pin
	tmp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));
	pGPIOHandle->pGPIOx->OSPEEDR |= tmp;

	tmp = 0;

	// configuring the pull up pull down settings
	tmp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));
	pGPIOHandle->pGPIOx->PUPDR |= tmp;

	tmp = 0;

	// configuring output type of the pin
	tmp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= tmp;

	tmp = 0;

	// configuring the alt function
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALT){
		uint32_t tmp1, tmp2;

		tmp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		tmp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

		pGPIOHandle->pGPIOx->AFR[tmp1] &= ~( 0xf << ( 4 * tmp2 ));
		pGPIOHandle->pGPIOx->AFR[tmp1] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * tmp2);
	}else{

	}
}


void GPIO_DedInit(GPIO_RegDef_t *pGPIOx){

	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}else if(pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}else if(pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}else if(pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	}else if(pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	}else if(pGPIOx == GPIOG){
		GPIOG_REG_RESET();
	}else if(pGPIOx == GPIOH){
		GPIOH_REG_RESET();
	}else if(pGPIOx == GPIOI){
		GPIOI_REG_RESET();
	}else if(pGPIOx == GPIOJ){
		GPIOJ_REG_RESET();
	}else if(pGPIOx == GPIOK){
		GPIOK_REG_RESET();
	}
}

/*
 * data read write
 */
uint8_t GPIO_ReadPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum){

	uint8_t value;

	value = (uint8_t)(( pGPIOx->IDR >> PinNum ) & 0x00000001 );

	return value;
}


uint16_t GPIO_ReadPort(GPIO_RegDef_t *pGPIOx){

	uint16_t value;

	value = (uint16_t)pGPIOx->IDR;

	return value;

}


void GPIO_WritePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum, uint8_t value){

	if(value == GPIO_PIN_SET){

		pGPIOx->ODR |= ( 1 << PinNum);

	}else{

		pGPIOx->ODR &= ~( 1 << PinNum);

	}
}


void GPIO_WritePort(GPIO_RegDef_t *pGPIOx, uint16_t value){

	pGPIOx->ODR = value;

}



void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum){

	pGPIOx->ODR ^= ( 1 << PinNum );

}

/*
 * GPIO interrupt handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNum, uint8_t ENOrDI){
	if( ENOrDI == ENABLE ){

		if(IRQNum <= 31){

			*NVIC_ISER0 |= ( 1 << IRQNum);

		}else if(IRQNum <= 63){

			*NVIC_ISER1 |= ( 1 << (IRQNum % 32));

		}else if(IRQNum <= 95){

			*NVIC_ISER2 |= ( 1 << (IRQNum % 32));

		}
	}else{

		if(IRQNum <= 31){

			*NVIC_ICER0 |= ( 1 << IRQNum);

		}else if(IRQNum <= 63){

			*NVIC_ICER1 |= ( 1 << (IRQNum % 32));

		}else if(IRQNum <= 95){

			*NVIC_ICER2 |= ( 1 << (IRQNum % 32));

		}
	}
}


void GPIO_IRQPriorityConfig(uint8_t IRQNum, uint32_t IRQPriority){
	// finding the ipr number
	uint8_t iprx = IRQNum / 4;
	uint8_t iprx_section = IRQNum % 4;

	uint8_t shift = (8 * iprx_section) + (8 - _NVIC_PRIO_BITS);

	NVIC_PR_BASE_ADDR[iprx] &= ~(0xFF << (8 * iprx_section));   // clear old priority

	NVIC_PR_BASE_ADDR[iprx] |= (IRQPriority << shift);          // set new priority
}

void GPIO_IRQHandling(uint8_t PinNum){
	// clear the pending bit
	if(EXTI->PR & ( 1 << PinNum)){

		// clear
		EXTI->PR |= (1 << PinNum);
	}


}
