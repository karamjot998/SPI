/*
 * stm32f439_spi_driver.h
 *
 *  Created on: Jan 14, 2026
 *      Author: binni
 */

#ifndef INC_STM32F439XX_SPI_DRIVER_H_
#define INC_STM32F439XX_SPI_DRIVER_H_

#include "stm32f439xx.h"

// MASTER OR SLAVE MODE
#define SPI_DEVICE_MODE_MASTER			1
#define SPI_DEVICE_MODE_SLAVE			0

// DATA BUS CONFIGURATION
#define SPI_BUS_CONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2
#define SPI_BUS_CONFIG_SIMPLEX_TXONLY	3
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	4

// SPI DATA TRANSMISSION SPEED
#define SPI_SCLK_SPEED_DIV2				1
#define SPI_SCLK_SPEED_DIV4				2
#define SPI_SCLK_SPEED_DIV8				3
#define SPI_SCLK_SPEED_DIV16			4
#define SPI_SCLK_SPEED_DIV32			5
#define SPI_SCLK_SPEED_DIV64			6
#define SPI_SCLK_SPEED_DIV128			7
#define SPI_SCLK_SPEED_DIV256			8

// DATA FRAME
#define SPI_DFF_8BIT					0
#define SPI_DFF_16BIT					1

// CLOCK POLARITY. IDLE STATE LOW OR HIGH
#define SPI_CPOL_HIGH					1
#define SPI_CPOL_LOW					0

// CLOCK PHASE. CAPTURE DATA AT FIRST CLOCK TRANSITION OR AT SECOND
#define SPI_CPHA_FIRST					0
#define SPI_CPHA_SECOND					1

// SOFTWARE SLAVE MANAGEMENT
#define SPI_SSM_DI						0
#define SPI_SSM_EN						1

// SPI related status flag definition
#define SPI_TXE_FLAG					( 1 << SPI_SR_TXE )
#define SPI_RXNE_FLAG					( 1 << SPI_SR_RXNE )
#define SPI_CHSIDE_FLAG					( 1 << SPI_SR_CHSIDE )
#define SPI_UDR_FLAG					( 1 << SPI_SR_UDR )
#define SPI_CRCERR_FLAG					( 1 << SPI_SR_CRCERR )
#define SPI_MODF_FLAG					( 1 << SPI_SR_MODF )
#define SPI_OVR_FLAG					( 1 << SPI_SR_OVR )
#define SPI_BSY_FLAG					( 1 << SPI_SR_BSY )
#define SPI_FRE_FLAG					( 1 << SPI_SR_FRE )

/*
 * Possible SPI Application State
 */
#define SPI_READY						0
#define SPI_BUSY_IN_RX					1
#define SPI_BUSY_IN_TX					2

/*
 * possible SPI Application Events
 */
#define SPI_EVENT_TX_CMPLT				1
#define SPI_EVENT_RX_CMPLT				2
#define SPI_EVENT_OVR_ERR				3
#define SPI_EVENT_CRC_ERR				4

/*
 * configuration structure for SPI
 */

typedef struct {
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;


/*
 * Handle structure for SPI
 */
typedef struct {
	SPI_RegDef_t	*pSPIx;			// holds the base address of spi register
	SPI_Config_t	SPIConfig;
	uint8_t			*pTxBuffer;		// to store the app Tx Buffer address
	uint8_t			*pRxBuffer; 		// to store the Rx buffer address
	uint32_t		TxLen;			// to store the Tx length
	uint32_t		RxLen;			// to store the Rx len
	uint8_t			TxState;		// to store the TxState
	uint8_t			RxState;		// to store the RxState
}SPI_Handle_t;


/***************************************************************************************
 * 							APIs SUPPORTED BY THE DRIVER
 *
 ****************************************************************************************/

/*
 * Peripheral Clock Setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t ENOrDi);


/*
 * Init and De - Init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DedInit(SPI_RegDef_t *pSPIx);

/*
 * 	data send and Receive	BLOCK
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,  uint8_t *pRxBuffer, uint32_t len);

/*
 * 	data send and Receive	INTERRUPT
 */
uint8_t SPI_SendData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len);
uint8_t SPI_ReceiveData_IT(SPI_Handle_t *pSPIHandle,  uint8_t *pRxBuffer, uint32_t len);


/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNum, uint8_t ENOrDI);
void SPI_IRQPriorityConfig(uint8_t IRQNum, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 * other Peripheral Control APIs
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t ENOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t ENOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t ENOrDi);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);
#endif /* INC_STM32F439XX_SPI_DRIVER_H_ */
