#include "stm32f4xx_uart.h"
static inline void TXEIE_Control(UART_RegDef_t *USARTx,uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        USARTx->CR1 |= 1 << UART_TXE_BIT_POS;
    }
    else
    {
        USARTx->CR1 &= ~(1 << UART_TXE_BIT_POS);
    }
}
static inline uint8_t Is_TXEIE_Enable(UART_RegDef_t *USARTx)
{
    return (USARTx->CR1 >> UART_TXE_BIT_POS ) & 0x1;
}
static inline uint8_t Is_TXE_Set(UART_RegDef_t *USARTx)
{
    return (USARTx->SR >> UART_TXE_BIT_POS ) & 0x1;
}
uint32_t GetPLLClock()
{
	return 0;
}
void Control_USART(UART_RegDef_t *USARTx,uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)
    {
        USARTx->CR1 |= 1 << UART_UE_BIT_POS;
    }
    else
    {
        USARTx->CR1 &= ~(1 << UART_UE_BIT_POS);
    }
}
static inline void Transmission_Control(UART_RegDef_t *USARTx,uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)
    {
        USARTx->CR1 |= 1 << UART_TE_BIT_POS;
    }
    else
    {
        USARTx->CR1 &= ~(1 << UART_TE_BIT_POS);
    }
}
static inline uint8_t Transmision_Not_Complete(UART_RegDef_t *USARTx)
{
    return ((USARTx->SR >> UART_TC_BIT_POS) & 0x1);
}
static inline uint8_t Busy_Tx(UART_RegDef_t *USARTx)
{
    return ((USARTx->SR >> UART_TXE_BIT_POS) & 0x1);
}
static uint32_t GetClock(void)
{
	uint16_t AHB_ClockPre[]={2,4,8,16,32,64,128,256,512};
	uint16_t APB1_ClockPre[]={2,4,8,16};
	uint32_t Clock,SystemClk;
	uint16_t AHBPre;
	uint16_t APB1Pre;
	uint8_t ClockSource;
	ClockSource = (RCC->CFGR >> 2) & (0x3);
	if(ClockSource == 0)
	{
		SystemClk = 16000000;
	}
	else if(ClockSource == 1)
	{
		SystemClk = 8000000;
	}
	else if(ClockSource == 2)
	{
		SystemClk = GetPLLClock();
	}
	int8_t temp =((RCC->CFGR >> 4)&(0xF)) - 8;
	if(temp < 8)
	{
		AHBPre = 1;
	}
	else
	{
		AHBPre = AHB_ClockPre[temp];
	}
	temp = ((RCC->CFGR >> 10)&(0x7)) - 4;
	if(temp < 4)
	{
		APB1Pre = 1;
	}
	else
	{
		APB1Pre = APB1_ClockPre[temp];
	}
	Clock = (SystemClk / AHBPre)/APB1Pre;
	return Clock;
}
static inline void BaudRateSetting(UART_RegDef_t *USARTx,uint32_t BaudRate)
{
    /* USARTDIV = DIV_Mantissa + (DIV_Fraction / 8 × (2 – OVER8)) */
    /* Tx/Rx baud = fCK / (8 x (2-OVER8) × USARTDIV) */
    uint32_t Clock = GetClock();
    double USARTDIV = 0;
    uint16_t DIV_Mantissa = 0;
    uint8_t DIV_Fraction = 0;
    /* using OVER8 = 0 */
    //USARTx->CR1 |= 1 << UART_OVER8_BIT_POS;
    USARTDIV = ((double)Clock / BaudRate ) / 16;
    DIV_Mantissa = ( Clock / BaudRate ) / 16;
    DIV_Fraction = (USARTDIV - (double)DIV_Mantissa) * 16;
    if(DIV_Fraction >=16)
    {
        DIV_Fraction = 0;
        DIV_Mantissa++;
    }
    USARTx->BRR |= DIV_Mantissa << UART_MANTISSA_POS;
    USARTx->BRR |= DIV_Fraction << UART_FRACTION_POS;
}
void USART_Init(USART_Handle_t *USART_Handle)
{
    USART_PeriClockControl(USART_Handle->USARTx,ENABLE);
    if(USART_Handle->UART_Config.USARTEnable == USART_ENABLE)
    {
        USART_Handle->USARTx->CR1 |= (1<<UART_USART_ENABLE_BIT_POS);
    }
    if(USART_Handle->UART_Config.WordLength == UART_WORD_LENGTH_9_BITS)
    {
        USART_Handle->USARTx->CR1 |= (1<<UART_M_BIT_POS);
    }
    if(USART_Handle->UART_Config.Parity == UART_PARITY_ENABLE)
    {
        USART_Handle->USARTx->CR1 |= (1<<UART_PCE_BIT_POS);
    }
    if((USART_Handle->UART_Config.Parity == UART_PARITY_ENABLE) &&(USART_Handle->UART_Config.ParitySelection == UART_ODD_PARITY))
    {
        USART_Handle->USARTx->CR1 |= (1<<UART_PARITY_SELECTION_BIT_POS);
    }
    switch (USART_Handle->UART_Config.NumberOfStopBit)
    {
        case STOP_BIT_1:
            USART_Handle->USARTx->CR2 |= (STOP_BIT_1<<UART_STOP_BITS_BIT_POS);
        break;
        case STOP_BIT_0_5:
            USART_Handle->USARTx->CR2 |= (STOP_BIT_0_5<<UART_STOP_BITS_BIT_POS);
        break;
        case STOP_BIT_2:
            USART_Handle->USARTx->CR2 |= (STOP_BIT_2<<UART_STOP_BITS_BIT_POS);
        break;
        case STOP_BIT_1_5:
            USART_Handle->USARTx->CR2 |= (STOP_BIT_1_5<<UART_STOP_BITS_BIT_POS);
        break;
    }
    BaudRateSetting(USART_Handle->USARTx,USART_Handle->UART_Config.BaudRate);

}
void USART_PeriClockControl(UART_RegDef_t *USARTx,uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        if(USARTx == UART4)
        {
            UART4_PCLK_EN();
        }

    }
    else
    {
        if(USARTx == UART4)
        {
            UART4_PCLK_DI();
        }
    }
}
void USART_Handling(USART_Handle_t *USART_Handle_t)
{
    if(Is_TXEIE_Enable(USART_Handle_t->USARTx) && Is_TXE_Set(USART_Handle_t->USARTx))
    {
        Transmission_Control(USART_Handle_t->USARTx,ENABLE);
        USART_Handle_t->USARTx->DR = *USART_Handle_t->TxBuffer;
        USART_Handle_t->TxBuffer++;
        USART_Handle_t->TxLength--;
        if(USART_Handle_t->TxLength == 0)
        {
        	USART_Handle_t->TxState = UART_READY;
        	TXEIE_Control(USART_Handle_t->USARTx,DISABLE);
        }
    }
}
uint8_t USART_TransmitDataIT(USART_Handle_t *USART_Handle_t,uint8_t *buffer,uint8_t Length)
{
    uint8_t state = USART_Handle_t->TxState;
    if(USART_Handle_t->TxState != UART_BUSY_IN_TX)
    {
        /* set new data  */
        USART_Handle_t->TxBuffer = buffer;
        USART_Handle_t->TxLength = Length;
        /* set state */
        USART_Handle_t->TxState = UART_BUSY_IN_TX;
        /* enable interupt for TXE */
        TXEIE_Control(USART_Handle_t->USARTx,ENABLE);
    }
    return state;
}
void USART_TransmitData(USART_Handle_t *USART_Handle_t,uint8_t *buffer,uint8_t Length)
{
    /* UART enable by writing UE bit in CR1 to 1 */
    /* set TE bit in CR1 register to sen an idle frame as first transmission */
    Transmission_Control(USART_Handle_t->USARTx,ENABLE);
    /* write data to DR register */
    while(Length != 0)
    {
        while(!Busy_Tx(USART_Handle_t->USARTx));
        USART_Handle_t->USARTx->DR = *buffer;
        buffer++;
        Length--;
    }
    /* waiting for transmission complete */
    while(!Transmision_Not_Complete(USART_Handle_t->USARTx));
    Transmission_Control(USART_Handle_t->USARTx,DISABLE);

}
void USART_ReceiveData(USART_Handle_t *USART_Handle_t,uint8_t *buffer,uint8_t Length)
{

}

void USART_InteruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
    uint8_t RegLocation = IRQNumber / 32;
    uint8_t FieldLocation = IRQNumber % 32;
    uint32_t tempReg;
    if(EnorDi == ENABLE)
    {
        tempReg = NVIC->ISER[RegLocation] & ~(1 <<FieldLocation);
        NVIC->ISER[RegLocation] = tempReg | (ENABLE << FieldLocation);
    }
    else
    {
        tempReg = NVIC->ICER[RegLocation] & ~(1 <<FieldLocation);
        NVIC->ICER[RegLocation] = tempReg | (DISABLE << FieldLocation);
    }
    return;
}
/*  */
void USART_IrqHandling(uint8_t PinNum)
{

}

void USART_PriorityConfig(uint8_t IRQNumber, uint8_t Priority)
{
    uint8_t RegLocation = IRQNumber / 4;
    uint8_t FieldLocation = IRQNumber % 4 ;
    uint8_t shift_amount = FieldLocation * 8 + 8 - IMPLEMENTED_BITS;
    uint32_t tempReg = NVIC->IPR[RegLocation] & ~(0xF << shift_amount);
    NVIC->IPR[RegLocation] = tempReg | (Priority << shift_amount);
    return;
}
