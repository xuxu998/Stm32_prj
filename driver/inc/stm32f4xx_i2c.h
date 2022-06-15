#ifndef _I2C_PERIPHERAL_
#define _I2C_PERIPHERAL_

#include "stm32f4xx_driver.h"

#define I2C_SCL_SPEED_SM 100000
#define I2C_SCL_SPEED_FM4K 400000
#define I2C_SCL_SPEED_FM2K 200000

#define I2C_ACK_ENABLE      1
#define I2C_ACK_DISABLE     0   

#define I2C_FM_DUTY_2       0
#define I2C_FM_DUTY_16_9    1

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
    uint8_t I2C_SCLSpeed;
    uint8_t I2C_DeviceAddress;
    uint8_t I2C_ACKControl;
    uint8_t I2C_FMDutyCycle;

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
void I2C_PeriClockControl(I2C_RegDef_t *I2Cx,uint8_t EnorDi);

/* Send and receive data*/

/*  */
void I2C_InteruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
/*  */
void I2C_PriorityConfig(I2C_Handle_t *pHandle);

void I2C_PeripheralControl(I2C_RegDef_t *I2Cx,uint8_t EnorDi);

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *I2Cx, uint32_t FlagName);

void I2C_ApplicationEventCallback(I2C_Handle_t * I2C_Handle, uint8_t AppEv);

#endif