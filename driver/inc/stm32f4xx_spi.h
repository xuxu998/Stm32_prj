#ifndef _SPI_HEADER_
#define _SPI_HEADER_


#include <stdint.h>
#define __vo volatile
#define SPI_READY		  			0
#define SPI_BUSY_IN_RX    			1
#define SPI_BUSY_IN_TX    			2
#define SPI_EVENT_RX_CMPLT			1
#define SPI_EVENT_TX_CMPLT			2
#define SPI_EVENT_OVR_ERR			3

typedef struct
{
     /* data */
     __vo uint32_t CR1;                                     /*  */
     __vo uint32_t CR2;                                     /*  */
     __vo uint32_t SR;                                      /*  */
     __vo uint32_t DR;                                      /*  */
     __vo uint32_t CRCPR;
     __vo uint32_t RXCRCR;
     __vo uint32_t TXCRCR;
     __vo uint32_t I2SCFGR;
     __vo uint32_t I2SPR;

}SPI_RegDef_t;

typedef struct
{
     /* data */
     uint8_t SPI_DeviceMode;								/*SPI device mode*/
     uint8_t SPI_BusConfig;									/*SPI bus config*/
     uint8_t SPI_ClkSpeed;									/*SPI Clock Speed*/
     uint8_t SPI_DFF;										/*SPI data frame*/
     uint8_t SPI_CPOL;										/*SPI CPOL and CPHA*/
     uint8_t SPI_CPHA;
     uint8_t SPI_SSM;										/*SPI software management*/
}SPI_Config_t;

typedef struct
{
     /* data */
     SPI_RegDef_t *SPIx;
     SPI_Config_t SPIConfig;
     uint8_t *Txbuffer;
     uint8_t *Rxbuffer;
     uint32_t TxLength;
     uint32_t RxLength;
     uint8_t TxState;
     uint8_t RxState;


}SPI_Handle_t;


void SPI_Init(SPI_Handle_t *pSPIx);
/*  */
void SPI_Deinit(SPI_RegDef_t *pSPIx);
/*  */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi);

/* Send and receive data*/

/*  */
uint8_t SPI_GetStatusFlag(SPI_RegDef_t *pSPIx);
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *Txbuffer, uint32_t Len);
/*  */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *Rxbuffer, uint32_t Len);
/*  */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIx_handle, uint8_t *Txbuffer, uint32_t Len);
/*  */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIx_handle, uint8_t *Rxbuffer, uint32_t Len);
/*  */
void SPI_InteruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
/*  */
//void SPI_PriorityConfig(SPI_Handle_t *pHandle);
void SPI_PriorityConfig(uint8_t IRQNumber, uint8_t Priority);

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi);
void SPI_IrqHandling(SPI_Handle_t *SPI_Handle_t);

void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi);

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi);
void SPI_ApplicationCallback(SPI_Handle_t *,uint8_t);
void SPI_CloseTransmission(SPI_Handle_t *pSPIx_handle);
void SPI_CloseReception(SPI_Handle_t *pSPIx_handle);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_ApplicationCallback(SPI_Handle_t*,uint8_t);



#endif
