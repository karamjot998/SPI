/*
 * stm32f439xx_spi_driver.c
 *
 *  Created on: Jan 15, 2026
 *      Author: binni
 */

#include <stdint.h>
#include "stm32f439xx.h"
#include "stm32f439xx_spi_driver.h"


static void spi_txe_interrupt_handle(SPI_Handle_t *pHandle);
static void spi_rxe_interrupt_handle(SPI_Handle_t *pHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pHandle);


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

	pSPIHandle->pSPIx->CR1 = tmpReg;
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

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName){
	if (pSPIx->SR & (FlagName)){
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
			pTxBuffer+=2;
		}else{
			pSPIx->DR = *pTxBuffer;
			len--;
			pTxBuffer++;
		}
	}
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx,  uint8_t *pRxBuffer, uint32_t len){

	while (len > 0){
		// wait until TXE is set
		while( 	SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		// check the DFF bit in cr1
		if( pSPIx->CR1 & ( 1 << SPI_CR1_DFF ) ){
			// 16 bit DFF
			*(( uint16_t *)pRxBuffer) = pSPIx->DR;
			len--;
			len--;
			pRxBuffer+=2;
		}else{
			*pRxBuffer = pSPIx->DR;
			len--;
			pRxBuffer++;
		}
	}
}


/*
 * 	data send and Receive	INTERRUPT
 */
uint8_t SPI_SendData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len){
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX){
		// 1. save the tx buffer addres and len information in some global variable
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = len;
		// 2. mark the SPI state as bsy in transmition so that no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;
		// 3. enable the txeie control bit to get information whenever TXE flag is et in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_TXEIE );
	}
	// data transmission will e handled by the ISR code
	return state;
}


uint8_t SPI_ReceiveData_IT(SPI_Handle_t *pSPIHandle,  uint8_t *pRxBuffer, uint32_t len){
	uint8_t state = pSPIHandle->RxState;
	if(state != SPI_BUSY_IN_RX){
		// 1. save the tx buffer addres and len information in some global variable
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = len;
		// 2. mark the SPI state as bsy in transmition so that no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;
		// 3. enable the txeie control bit to get information whenever TXE flag is et in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_RXNEIE );
	}
	// data transmission will e handled by the ISR code
	return state;
}



/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNum, uint8_t ENOrDI){
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

void SPI_IRQPriorityConfig(uint8_t IRQNum, uint32_t IRQPriority){
	// finding the ipr number
	uint8_t iprx = IRQNum / 4;
	uint8_t iprx_section = IRQNum % 4;

	uint8_t shift = (8 * iprx_section) + (8 - _NVIC_PRIO_BITS);

	NVIC_PR_BASE_ADDR[iprx] &= ~(0xFF << (8 * iprx_section));   // clear old priority

	NVIC_PR_BASE_ADDR[iprx] |= (IRQPriority << shift);          // set new priority

}

void SPI_IRQHandling(SPI_Handle_t *pHandle){
	// check for Txe
	uint8_t tmp1, tmp2;
	tmp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_TXE);
	tmp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE);

	if ( tmp1 && tmp2){
		// handle txe
		spi_txe_interrupt_handle(pHandle);
	}

	tmp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE);
	tmp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE);

	if ( tmp1 && tmp2){
		// handle txe
		spi_rxe_interrupt_handle(pHandle);
	}

	// check for OVR Flag
	tmp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_OVR );
	tmp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE );

	if ( tmp1 && tmp2){
		// handle txe
		spi_ovr_err_interrupt_handle(pHandle);
	}
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

/*
 * SPI Slave Select (SSI) Control
 * Used when software slave management (SSM = 1) is enabled
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t ENOrDi){
	if(ENOrDi == ENABLE){
		pSPIx->CR1 |= ( 1 << SPI_CR1_SSI);
	}else{
		pSPIx->CR1 &= ~( 1 << SPI_CR1_SSI);
	}
}

/*
 * SPI Slave Select Output Enable (SSOE) Control
 * Used when hardware slave management (SSM = 0) is enabled
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t ENOrDi){
	if(ENOrDi == ENABLE){
		pSPIx->CR2 |= ( 1 << SPI_CR2_SSOE);
	}else{
		pSPIx->CR2 &= ~( 1 << SPI_CR2_SSOE);
	}
}

// helper function implementation
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle){
	// check the DFF bit in cr1
	if( pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF ) ){
		// 16 bit DFF
		pSPIHandle->pSPIx->DR = *(( uint16_t *)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer+=2;
	}else{
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if (! pSPIHandle->TxLen){
		// close spi communication is TxLen is 0
		// inform application that Tx is over
		// prevent interupts from txe flag
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}


static void spi_rxe_interrupt_handle(SPI_Handle_t *pSPIHandle){

	if( pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF ) ){
		// 16 bit DFF
		*(( uint16_t *)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer+=2;
	}else{
		*pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if (! pSPIHandle->TxLen){
		// close spi communication is TxLen is 0
		// inform application that Tx is over
		// prevent interupts from txe flag
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}

}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle){

	// clear the ovr flag
	if( pSPIHandle->TxState != SPI_BUSY_IN_TX ){
		SPI_ClearOVRFlag(pSPIHandle->pSPIx);
	}
	// Inform the Application

	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);

}


void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx){
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE );
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;

}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE );
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;

}
