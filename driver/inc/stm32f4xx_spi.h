#ifndef _SPI_HEADER_
#define _SPI_HEADER_


#include <stdint.h>
#define __vo volatile
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

}SPI_Handle_t;


void SPI_Init(SPI_Handle_t *pSPIx);
/*  */
void SPI_Deinit(SPI_RegDef_t *pSPIx);
/*  */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi);

/* Send and receive data*/

/*  */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *Txbuffer, uint32_t Len);
/*  */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *Rxbuffer, uint32_t Len);
/*  */
void SPI_InteruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
/*  */
void SPI_IrqHandling(uint8_t PinNum);
/*  */
void SPI_PriorityConfig(SPI_Handle_t *pHandle);

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi);

void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi);





#endif
