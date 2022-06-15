#ifndef _I2C_PERIPHERAL_
#define _I2C_PERIPHERAL_

#include "stm32f4xx_driver.h"

typedef struct 
{
    /* data */
    __vo uint32_t CR1;
    __vo uint32_t CR2;
    __vo uint32_t OAR1;
    __vo uint32_t OAR2;
    __vo uint32_t DR;
    __vo uint32_t SR1;
    __vo uint32_t SR2;
    __vo uint32_t CCR;
    __vo uint32_t TRISE;
    __vo uint32_t FLTR;

}I2C_RegDef_t;

typedef struct
{
    uint8_t 

}I2C_Config_t;

typedef struct
{
    I2C_RegDef_t * I2Cx;
    I2C_Config_t I2C_Config;
}I2C_Handle_t;

void I2C_Init(I2C_Handle_t * I2C_Handle);
/* */
void I2C_Deinit(I2C_RegDef_t * I2Cx);
/* */
void I2C_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi);

/* Send and receive data*/

/*  */
void I2C_SendData(I2C_RegDef_t *pSPIx, uint8_t *Txbuffer, uint32_t Len);
/*  */
void I2C_ReceiveData(I2C_RegDef_t *pSPIx, uint8_t *Rxbuffer, uint32_t Len);
/*  */
void I2C_InteruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
/*  */
void I2C_IrqHandling(uint8_t PinNum);
/*  */
void I2C_PriorityConfig(I2C_Handle_t *pHandle);

void I2C_PeripheralControl(I2C_RegDef_t *pSPIx,uint8_t EnorDi);

void I2C_SSIConfig(I2C_RegDef_t *pSPIx,uint8_t EnorDi);

#endif