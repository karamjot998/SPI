/*
 * stm32f439xx_spi_driver.c
 *
 *  Created on: Jan 15, 2026
 *      Author: binni
 */

#include <stdint.h>
#include "stm32f439xx.h"
#include "stm32f439xx_spi_driver.h"


void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t ENOrDi){

	if(ENOrDi == ENABLE){
		if(pSPIx == SPI1){
			SPI1_PCLK_EN();
		}else if(pSPIx == SPI2){
			SPI2_PCLK_EN();
		}else if(pSPIx == SPI3){
			SPI3_PCLK_EN();
		}else if(pSPIx == SPI4){
			SPI4_PCLK_EN();
		}else if(pSPIx == SPI5){
			SPI5_PCLK_EN();
		}else if(pSPIx == SPI6){
			SPI6_PCLK_EN();
		}
	}else{
		if(pSPIx == SPI1){
			SPI1_PCLK_DI();
		}else if(pSPIx == SPI2){
			SPI2_PCLK_DI();
		}else if(pSPIx == SPI3){
			SPI3_PCLK_DI();
		}else if(pSPIx == SPI4){
			SPI4_PCLK_DI();
		}else if(pSPIx == SPI5){
			SPI5_PCLK_DI();
		}else if(pSPIx == SPI6){
			SPI6_PCLK_DI();
		}
	}
}

/*
 * Init and De - Init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle){
	uint32_t tmpReg = 0;

	// enabling	SPI peripheral clock control
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// configuring device as master or slave
	tmpReg |= ( pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR );


	// configure the bus configuration
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		// BIDI MODE SHOULD BE CLEARED
		tmpReg &= ~( 1 << SPI_CR1_BIDIMODE );
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		// BIDI MODE SHOULD BE ST
		tmpReg |= ( 1 << SPI_CR1_BIDIMODE );
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		// BIDI MODE SHOULD BE CLEARED
		tmpReg &= ~( 1 << SPI_CR1_BIDIMODE );
		// RXONLY MODE SHOULD BE SET
		tmpReg |= ( 1 << SPI_CR1_RXONLY );
	}

	// configure the spi serial clock speed ( baud rate)
	tmpReg |= ( pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR );

	// configure the DFF
	tmpReg |= ( pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF );

	// configure CPOL
	tmpReg |= ( pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL );

	// configure CPHA
	tmpReg |= ( pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA );

	// configure hardware/software slave management
	tmpReg |= ( pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM );

	pSPIHandle->pSPIx->CR1 |= tmpReg;
}

void SPI_DedInit(SPI_RegDef_t *pSPIx){
	if(pSPIx == SPI1){
		SPI1_REG_RESET();
		}else if(pSPIx == SPI2){
			SPI2_REG_RESET();
		}else if(pSPIx == SPI3){
			SPI3_REG_RESET();
		}else if(pSPIx == SPI4){
			SPI4_REG_RESET();
		}else if(pSPIx == SPI5){
			SPI5_REG_RESET();
		}else if(pSPIx == SPI6){
			SPI6_REG_RESET();
		}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t FlagName){
	if (pSPIx->SR & ( 1 << FlagName)){
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*
 * 	data send and Receive.     (this is a blocking call)
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len){

	while (len > 0){
		// wait until TXE is set
		while( 	SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		// check the DFF bit in cr1
		if( pSPIx->CR1 & ( 1 << SPI_CR1_DFF ) ){
			// 16 bit DFF
			pSPIx->DR = *(( uint16_t *)pTxBuffer);
			len--;
			len--;
			(uint16_t*)pTxBuffer++;
		}else{
			pSPIx->DR = *pTxBuffer;
			len--;
			pTxBuffer++;
		}
	}
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx,  uint8_t *pRxBuffer, uint32_t len){


}


/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNum, uint8_t ENOrDI){


}

void SPI_IRQPriorityConfig(uint8_t IRQNum, uint32_t IRQPriority){


}

void SPI_IRQHandling(SPI_Handle_t *pHandle){


}

/*
 * other Peripheral Control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t ENOrDi){
	if(ENOrDi == ENABLE){
		pSPIx->CR1 |= ( 1 << SPI_CR1_SPE);
	}else{
		pSPIx->CR1 &= ~( 1 << SPI_CR1_SPE);
	}
}


void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t ENOrDi){
	if(ENOrDi == ENABLE){
		pSPIx->CR1 |= ( 1 << SPI_CR1_SSI);
	}else{
		pSPIx->CR1 &= ~( 1 << SPI_CR1_SSI);
	}
}
