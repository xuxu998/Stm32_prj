#ifndef _STM32_GPIO_
#define _STM32_GPIO_

#include "stm32f4xx_driver.h"

typedef struct
{
    uint8_t GPIO_PinNumber;                 /* possible values from @GPIO_PIN_NUMBER */
    uint8_t GPIO_PinMode;                   /* possible values from  @GPIO_PIN_MODE */
    uint8_t GPIO_PinSpeed;                  /* possible values from @GPIO_SPEED */
    uint8_t GPIO_PinPuPdControl;            /* possible values from @GPIO_PUPD */
    uint8_t GPIO_PinOpType;                 /* possible values from @GPIO_OUTPUT_TYPE */
    uint8_t GPIO_AltFunMode;
}GPIO_PinConfig_t;


typedef struct
{
    GPIO_RegDef_t *pGPIOx;
    GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;

/*  */
void GPIO_Init(GPIO_Handle_t *pGPIOx);
/*  */
void GPIO_Deinit(GPIO_RegDef_t *pGPIOx);
/*  */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi);
/*  */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum );
/*  */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
/*  */
void GPIO_WriteOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum, uint8_t value);
/*  */
void GPIO_WriteOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
/*  */
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum);
/*  */
void GPIO_InteruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
/*  */
void GPIO_IrqHandling(uint8_t PinNum);

void GPIO_PriorityConfig(uint8_t, uint8_t);

/* @GPIO_PIN_NUMBER */

#define GPIO_PIN_NO_0 0
#define GPIO_PIN_NO_1 1
#define GPIO_PIN_NO_2 2
#define GPIO_PIN_NO_3 3
#define GPIO_PIN_NO_4 4
#define GPIO_PIN_NO_5 5
#define GPIO_PIN_NO_6 6
#define GPIO_PIN_NO_7 7
#define GPIO_PIN_NO_8 8
#define GPIO_PIN_NO_9 9
#define GPIO_PIN_NO_10 10
#define GPIO_PIN_NO_11 11
#define GPIO_PIN_NO_12 12
#define GPIO_PIN_NO_13 13
#define GPIO_PIN_NO_14 14
#define GPIO_PIN_NO_15 15

/* @GPIO_PIN_MODE */

#define GPIO_MODE_IN 0
#define GPIO_MODE_OUT 1
#define GPIO_MODE_ALT 2
#define GPIO_MODE_ANALOG 3
#define GPIO_MODE_IT_FT 4
#define GPIO_MODE_IT_RT 5
#define GPIO_MODE_IT_RFT 6

/* @GPIO_OUTPUT_TYPE */

#define GPIO_OP_TYPE_PP 0
#define GPIO_OP_TYPE_OD 1

/* @GPIO_PUPD */

#define GPIO_NO_PUPD 0
#define GPIO_PIN_PU 1
#define GPIO_PIN_PD 2

/* @GPIO_SPEED */

#define GPIO_SPEED_LOW 0
#define GPIO_SPEED_MEDIUM 1
#define GPIO_SPEED_HIGH 2
#define GPIO_SPEED_VHIGH 3

#endif
