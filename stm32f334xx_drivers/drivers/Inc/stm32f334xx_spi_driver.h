/*
 * stm32f334xx_spi_driver.h
 *
 *  Created on: Apr 2, 2024
 *      Author: ardau
 */

#ifndef INC_STM32F334XX_SPI_DRIVER_H_
#define INC_STM32F334XX_SPI_DRIVER_H_

#include "stm32f334xx.h"

/*
 * This is a Configuration structure for SPI communication
*/
typedef struct
{
    uint8_t SPI_DeviceMode;                                                 /*!< possible values from @SPI_DeviceMode >*/
    uint8_t SPI_BusConfig;                                                  /*!< possible values from @SPI_BusConfig >*/
    uint8_t SPI_SclkSpeed;                                                  /*!< possible values from @SPI_SclkSpeed >*/
    uint8_t SPI_CPOL;                                                       /*!< possible values from @SPI_CPOL >*/
    uint8_t SPI_CPHA;                                                       /*!< possible values from @SPI_CPHA >*/
    uint8_t SPI_SSM;                                                        /*!< possible values from @SPI_SSM >*/

}SPI_Config_t;

/*
 * This is a Handle structure for SPI communication
*/
typedef struct
{
    // Pointer to hold the base address of the SPI peripheral
    SPI_RegDef_t      *pSPIx;                                              /*!< This holds the base address of the SPI1 peripheral >*/
    SPI_Config_t      SPIConfig;                                           /*!< This holds SPI configuration settings >*/
    uint8_t           *pTxBuffer;                                          /*!< To store the app. Tx buffer address >*/
    uint8_t           *pRxBuffer;                                          /*!< To store the app. Rx buffer address >*/
    uint32_t          TxLen;                                               /*!< To store Tx len >*/
    uint32_t          RxLen;                                               /*!< To store Rx len >*/
    uint8_t           TxStatus;                                            /*!< To store Tx state >*/
    uint32_t          RxStatus;                                            /*!< To store Rx state >*/

}SPI_Handle_t;

/*************************************** Some Macros For SPI_Config_t****************************************/

/*
 * SPI application states
*/
#define SPI_READY                                  0
#define SPI_BUSY_IN_RX                             1    
#define SPI_BUSY_IN_TX                             2

/*
 * Possible SPI application events
*/
#define SPI_EVENT_TX_CMPLT                         1
#define SPI_EVENT_RX_CMPLT                         2
#define SPI_EVENT_OVR_ERR                          3
#define SPI_EVENT_CRC_ERR                          4

/*
 * @SPI_DeviceMode
*/
#define SPI_DEVICE_MODE_MASTER                     1
#define SPI_DEVICE_MODE_SLAVE                      0

/*
 * @SPI_BusConfig
*/
#define SPI_BUS_CONFIG_FD                          1
#define SPI_BUS_CONFIG_HD                          2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY              3

/*
 * @SPI_SclkSpeed
*/
#define SPI_SCLK_SPEED_DIV2                        0
#define SPI_SCLK_SPEED_DIV4                        1
#define SPI_SCLK_SPEED_DIV8                        2
#define SPI_SCLK_SPEED_DIV16                       3   
#define SPI_SCLK_SPEED_DIV32                       4  
#define SPI_SCLK_SPEED_DIV64                       5 
#define SPI_SCLK_SPEED_DIV128                      6  
#define SPI_SCLK_SPEED_DIV256                      7  

/*
 * @SPI_CPOL
*/
#define SPI_CPOL_LOW                               0
#define SPI_CPOL_HIGH                              1

/*
 * @SPI_CPHA
*/
#define SPI_CPHA_LOW                               0
#define SPI_CPHA_HIGH                              1

/*
 * @SPI_SSM
*/
#define SPI_SSM_DI                                 0
#define SPI_SSM_EN                                 1

/*
 * SPI related status flag definitions
*/
#define SPI_RXNE_FLAG                              ( 1 << SPI_SR_RXNE  )
#define SPI_TXE_FLAG                               ( 1 << SPI_SR_TXE   )
#define SPI_CRCERR_FLAG                            ( 1 << SPI_SR_RXNE  )
#define SPI_MODF_FLAG                              ( 1 << SPI_SR_MODF  )
#define SPI_OVR_FLAG                               ( 1 << SPI_SR_OVR   )
#define SPI_BSY_FLAG                               ( 1 << SPI_SR_BSY   )
#define SPI_FRE_FLAG                               ( 1 << SPI_SR_FRE   )
#define SPI_FRLVL_FLAG                             ( 1 << SPI_SR_FRLVL )
#define SPI_FTLVL_FLAG                             ( 1 << SPI_SR_FTLVL )

/**********************************************************************************************************
 *                                   APIs supported by this driver 
 *               For more information about the APIs check the function definitions
***********************************************************************************************************/

/*
 * Peripheral Clock Setup
*/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Init and De-init
*/
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_Handle_t *pSPIHandle);

/*
 * Data Send and Receive
*/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/*
 * Data Send and Receive with Interrupt
*/
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ and ISR
*/
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/*
 * Other Peripheral Control APIs
*/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * Application callback 
*/
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);

#endif /* INC_STM32F334XX_SPI_DRIVER_H_ */
