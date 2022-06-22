#ifndef _I2C_PERIPHERAL_
#define _I2C_PERIPHERAL_

#include "stm32f4xx_driver.h"
#define I2C_READY	0
#define I2C_BUSY_IN_RX	1
#define I2C_BUSY_IN_TX	2
#define I2C1_IRQ_NUMBER_EV 	31
#define I2C2_IRQ_NUMBER_EV	33
#define I2C3_IRQ_NUMBER_EV	72
#define I2C1_IRQ_NUMBER_ER 	32
#define I2C2_IRQ_NUMBER_ER 	34
#define I2C3_IRQ_NUMBER_ER 	73
#define FLAG_SET    1
#define FLAG_RESET  0
#define I2C_SCL_SPEED_SM 100000
#define I2C_SCL_SPEED_FM4K 400000
#define I2C_SCL_SPEED_FM2K 200000

#define I2C_ACK_ENABLE      1
#define I2C_ACK_DISABLE     0   

#define I2C_FM_DUTY_2       0
#define I2C_FM_DUTY_16_9    1

#define I2C_F_S_BIT_POSITION    15
#define I2C_DUTY_BIT_POSITION   14
#define I2C_ACK_BIT_POSITION	10
#define I2C_START_BIT_POSITION  8
#define I2C_TXE_BIT_POSITION    7
#define I2C_RXNE_BIT_POSITION   6
#define I2C_ADDR_BIT_POSITION   1
#define I2C_STOP_BIT_CONDITION  9
#define I2C_STOPF_BIT_POSITION  4
#define I2C_SB_BIT_POSITION 	0
#define I2C_BTF_BIT_POSITION    2
#define ITBUFEN_BIT_POSITION    10
#define ITEVTEN_BIT_POSITION    9
#define ITERREN_BIT_POSITION	8
#define I2C_FLAG_SB (1 << I2C_SB_BIT_POSITION)
#define I2C_FLAG_TXE (1 << I2C_TXE_BIT_POSITION)
#define I2C_FLAG_RXNE (1 << I2C_RXNE_BIT_POSITION)
#define I2C_FLAG_BTF (1 << I2C_BTF_BIT_POSITION)
#define I2C_FLAG_ADDR ( 1<< I2C_ADDR_BIT_POSITION)

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
    uint32_t I2C_SCLSpeed;
    uint8_t I2C_DeviceAddress;
    uint8_t I2C_ACKControl;
    uint8_t I2C_FMDutyCycle;

}I2C_Config_t;

typedef struct
{
    I2C_RegDef_t *I2Cx;
    I2C_Config_t I2C_Config;
    uint8_t *TxBuffer;
    uint32_t TxLength;
    uint8_t *RxBuffer;
    uint32_t RxLength;
    uint8_t TxRxStatus;
    uint8_t SlaveAddress;
    uint8_t Sr; //to store repeated start value
    uint32_t RxSize;
}I2C_Handle_t;
/* */
void I2C_EV_IRQHandling(I2C_Handle_t *I2C_Handle);
/* */
void I2C_ER_IRQHandling(I2C_Handle_t *I2C_Handle);
/* */
void I2C_Init(I2C_Handle_t *I2C_Handle);
/* */
void I2C_Deinit(I2C_RegDef_t *I2Cx);
/* */
void I2C_PeriClockControl(I2C_RegDef_t *I2Cx,uint8_t EnorDi);

/* Send and receive data*/
void I2C_MasterSendData(I2C_Handle_t *pHandle,uint8_t *buffer,uint8_t Len,uint8_t SlaveAddress);
void I2C_MasterReceiveData(I2C_Handle_t *pHandle,uint8_t *buffer,uint8_t Len,uint8_t SlaveAddress);

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pHandle,uint8_t *buffer,uint8_t Len,uint8_t SlaveAddress,uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pHandle,uint8_t *buffer,uint8_t Len,uint8_t SlaveAddress,uint8_t Sr);

/*  */
void I2C_InteruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
/*  */
void I2C_PriorityConfig(uint8_t IRQNumber, uint8_t Priority);

void I2C_PeripheralControl(I2C_RegDef_t *I2Cx,uint8_t EnorDi);


void I2C_ApplicationEventCallback(I2C_Handle_t * I2C_Handle, uint8_t AppEv);

#endif
