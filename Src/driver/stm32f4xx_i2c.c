#include "stm32f4xx_i2c.h"
#include "stm32f4xx_driver.h"
#include <stdint.h>
#define GUIDE
static inline uint8_t I2CIsMatched(I2C_RegDef_t *I2Cx)
{
	return (I2Cx->SR1>>I2C_ADDR_BIT_POSITION);
}
static inline uint8_t I2CIsEmpty(I2C_RegDef_t *I2Cx)
{
	return (I2Cx->SR1 >> I2C_TXE_BIT_POSITION);
}
static inline uint8_t I2C_Is_Full(I2C_RegDef_t *I2Cx)
{
	return (I2Cx->SR1 >> I2C_RXNE_BIT_POSITION);
}
static inline void I2CStop(I2C_RegDef_t *I2Cx)
{
	I2Cx->CR1 |= 1 << I2C_STOP_BIT_CONDITION;
	return;
}
static inline void I2CEnable(I2C_RegDef_t *I2Cx)
{
	I2Cx->CR1 |= 1 << 0;
}
static inline void I2CDisable(I2C_RegDef_t *I2Cx)
{
	I2Cx->CR1 &= ~(1 << 0);
}
static inline void I2C_GenerateStartCondition(I2C_RegDef_t *I2Cx)
{
	I2Cx->CR1 |= 1 << I2C_START_BIT_POSITION;
	return;
}
static inline uint8_t I2C_GetFlagStatus(I2C_RegDef_t *I2Cx,uint8_t FlagName)
{
	if(I2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	else
	{
		return FLAG_RESET;
	}
}
static inline void ACK_Control(I2C_RegDef_t *I2Cx,uint8_t state)
{
	if(state == ENABLE)
	{
		I2Cx->CR1 |= (1 << I2C_ACK_BIT_POSITION);
	}
	else
	{
		I2Cx->CR1 &= ~(1 << I2C_ACK_BIT_POSITION);
	}
}
static inline void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *I2Cx,uint8_t SlaveAddress)
{
	SlaveAddress = SlaveAddress << 1;
	SlaveAddress &= ~(1 << 0);
	I2Cx->DR = SlaveAddress;
}
static inline void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *I2Cx,uint8_t SlaveAddress)
{
	SlaveAddress = SlaveAddress << 1;
	SlaveAddress |= 1 << 0;
	I2Cx->DR = SlaveAddress;
}
static inline void I2C_ADDRClearFlag(I2C_Handle_t *I2C_Handle)
{
	uint32_t dummy;
	if(I2C_Handle->I2Cx->SR2 >> I2C_MSL_BIT_POSITION & 0x1)
	{
		if(I2C_Handle->TxRxStatus == I2C_BUSY_IN_RX)
		{
			if(I2C_Handle->RxSize == 1)
			{
				ACK_Control(I2C_Handle->I2Cx,DISABLE);
				/* clear addr flag */
				dummy = I2C_Handle->I2Cx->SR1;
				dummy = I2C_Handle->I2Cx->SR2;
				(void)dummy;
			}
		}
		else
		{
			/* clear addr flag */
			dummy = I2C_Handle->I2Cx->SR1;
			dummy = I2C_Handle->I2Cx->SR2;
			(void)dummy;
		}
	}
	else
	{
		/* clear addr flag */
		dummy = I2C_Handle->I2Cx->SR1;
		dummy = I2C_Handle->I2Cx->SR2;
		(void)dummy;
	}
	

}
static inline void I2C_BufferInteruptEnable(I2C_RegDef_t *I2Cx)
{
	I2Cx->CR2 |= (1 << ITBUFEN_BIT_POSITION);
	return;
}
static inline void I2C_EventInteruptEnable(I2C_RegDef_t *I2Cx)
{
	I2Cx->CR2 |= (1 << ITEVTEN_BIT_POSITION);
	return;
}
static inline void I2C_ErrorInteruptEnable(I2C_RegDef_t *I2Cx)
{
	I2Cx->CR2 |= (1 << ITERREN_BIT_POSITION);
	return;
}
void I2C_MasterSendData(I2C_Handle_t *pHandle,uint8_t *buffer,uint8_t Len,uint8_t SlaveAddress)
{
	#if defined MY_CODE
	uint32_t tempReg = 0;
	I2CEnable(pHandle->I2Cx);
	/* set start condition */
	pHandle->I2Cx->CR1 |= 1 << I2C_START_BIT_POSITION;
	/* waiting for SB is set */
	while(!(pHandle->I2Cx->SR1 & 0x01))
	/* clear SB bit by reading SR1 and writing to DR*/
	/* reading SR1 */
	tempReg = pHandle->I2Cx->SR1;
	/* writing to DR */
	pHandle->I2Cx->DR = SlaveAddress << 1;
	/* waiting for address matched */
	//while(!I2CIsMatched(pHandle->I2Cx))
	/* clear ADDR */
	tempReg = pHandle->I2Cx->SR1;
	tempReg = pHandle->I2Cx->SR2;
	//while(!I2CIsEmpty(pHandle->I2Cx));
	while(Len != EMPTY)
	{
		while(I2CIsEmpty(pHandle->I2Cx));
		pHandle->I2Cx->DR = *buffer;
		buffer++;
		Len--;
	}
	//while(!I2CIsEmpty(pHandle->I2Cx));
	I2CStop(pHandle->I2Cx);
	I2CDisable(pHandle->I2Cx);
	#endif
	#if defined GUIDE
	//1 .generate start condition
	I2C_GenerateStartCondition(pHandle->I2Cx);
	//2 .waiting until start condition generation is done
	while(!I2C_GetFlagStatus(pHandle->I2Cx,I2C_FLAG_SB));
	//3 .send address
	I2C_ExecuteAddressPhaseWrite(pHandle->I2Cx,SlaveAddress);
	/*4. waiting for ADDR flag is set */
	while( !  I2C_GetFlagStatus(pHandle->I2Cx,I2C_FLAG_ADDR));
	//5 clear ADDR flag.
	I2C_ADDRClearFlag(pHandle);
	while(Len != EMPTY)
	{
		while(I2C_GetFlagStatus(pHandle->I2Cx,I2C_FLAG_TXE));
		pHandle->I2Cx->DR = *buffer;
		buffer++;
		Len--;
	}
	while(I2C_GetFlagStatus(pHandle->I2Cx,I2C_FLAG_TXE));
	while(I2C_GetFlagStatus(pHandle->I2Cx,I2C_FLAG_BTF));
	I2CStop(pHandle->I2Cx);
	#endif

}
void I2C_MasterReceiveData(I2C_Handle_t *pHandle,uint8_t *buffer,uint8_t Len,uint8_t SlaveAddress)
{
	//1 .generate start condition
	I2C_GenerateStartCondition(pHandle->I2Cx);
	//2 .waiting until start condition generation is done
	while(!I2C_GetFlagStatus(pHandle->I2Cx,I2C_FLAG_SB));
	//3 .send address
	I2C_ExecuteAddressPhaseRead(pHandle->I2Cx,SlaveAddress);
	/*4. waiting for ADDR flag is set */
	while(!I2C_GetFlagStatus(pHandle->I2Cx,I2C_FLAG_ADDR));
	//5 clear ADDR flag.
	I2C_ADDRClearFlag(pHandle);
	if(Len > 1)
	{
		while(Len != EMPTY)
		{
			if(Len == 2)
			{
				ACK_Control(pHandle->I2Cx,DISABLE);
				I2CStop(pHandle->I2Cx);
			}
			while(I2C_GetFlagStatus(pHandle->I2Cx,I2C_FLAG_RXNE));
			*buffer= pHandle->I2Cx->DR;
			buffer++;
			Len--;
		}
	}
	else if(Len == 1)
	{
		ACK_Control(pHandle->I2Cx,DISABLE);
		I2CStop(pHandle->I2Cx);
		while(I2C_GetFlagStatus(pHandle->I2Cx,I2C_FLAG_RXNE));
		*buffer= pHandle->I2Cx->DR;
	}
	if(pHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		ACK_Control(pHandle->I2Cx,ENABLE);
	}
	return;
}
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pHandle,uint8_t *buffer,uint8_t Len,uint8_t SlaveAddress,uint8_t Sr)
{
	uint8_t State = pHandle->TxRxStatus;
	if((State != I2C_BUSY_IN_RX)&&(State != I2C_BUSY_IN_TX))
	{
		pHandle->TxBuffer = buffer;
		pHandle->TxLength = Len;
		pHandle->TxRxStatus = I2C_BUSY_IN_TX;
		pHandle->SlaveAddress = SlaveAddress;
		pHandle->Sr = Sr;
		/* generate start condition */
		I2C_GenerateStartCondition(pHandle->I2Cx);
		/* enable interupt buffer control bit */
		I2C_BufferInteruptEnable(pHandle->I2Cx);
		/* enablt interupt event control bit */
		I2C_EventInteruptEnable(pHandle->I2Cx);
		/* enable error interupt control bit */
		I2C_ErrorInteruptEnable(pHandle->I2Cx);
	}
	return State;
}
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pHandle,uint8_t *buffer,uint8_t Len,uint8_t SlaveAddress,uint8_t Sr)
{
	uint8_t State = pHandle->TxRxStatus;
	if((State != I2C_BUSY_IN_RX)&&(State != I2C_BUSY_IN_TX))
	{
		pHandle->RxBuffer = buffer;
		pHandle->RxLength = Len;
		pHandle->TxRxStatus = I2C_BUSY_IN_RX;
		pHandle->SlaveAddress = SlaveAddress;
		pHandle->Sr = Sr;
		/* generate start condition */
		I2C_GenerateStartCondition(pHandle->I2Cx);
		/* enable interupt buffer control bit */
		I2C_BufferInteruptEnable(pHandle->I2Cx);
		/* enablt interupt event control bit */
		I2C_EventInteruptEnable(pHandle->I2Cx);
		/* enable error interupt control bit */
		I2C_ErrorInteruptEnable(pHandle->I2Cx);
	}
	return State;
}
static uint32_t GetPLLClock(void)
{
	/* not implemented yet */
	return 0;
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

void I2C_Init(I2C_Handle_t *I2C_Handle)
{
	/* enable I2C clock */
	uint32_t tempReg = 0;
	I2C_PeriClockControl(I2C_Handle->I2Cx,ENABLE);
	if(I2C_Handle->I2C_Config.I2C_ACKControl == ENABLE)
	{
		I2C_Handle->I2Cx->CR1 |= (1 << I2C_ACK_BIT_POSITION);
	}
	else
	{
		I2C_Handle->I2Cx->CR1 &= ~(1 << I2C_ACK_BIT_POSITION);
	}
	/* config FREQ  */
	tempReg = GetClock()/1000000U;
	I2C_Handle->I2Cx->CR2 = tempReg & 0x3F;
	/* config address */
	tempReg = I2C_Handle->I2C_Config.I2C_DeviceAddress << 1;
	tempReg |= (1<<14);
	I2C_Handle->I2Cx->OAR1 = tempReg ;
	/* CCR calculation */
	uint16_t ccr_value = 0;
	tempReg = 0;
	if(I2C_Handle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		/* in standard mode */
		ccr_value = GetClock() / (2 * I2C_Handle->I2C_Config.I2C_SCLSpeed);
		tempReg |= (uint32_t)(ccr_value & (0xFFF));
	}
	else
	{
		/* in fast mode */
		tempReg |= 1 << I2C_F_S_BIT_POSITION;
		if(I2C_Handle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = GetClock() / (3 * I2C_Handle->I2C_Config.I2C_SCLSpeed);
		}
		else
		{
			tempReg |= 1 << I2C_DUTY_BIT_POSITION;
			ccr_value = GetClock() / (25 * I2C_Handle->I2C_Config.I2C_SCLSpeed);
		}
		tempReg |= (uint32_t)(ccr_value & (0xFFF));
	}
	I2C_Handle->I2Cx->CCR = tempReg;
	/*t rise calculation*/
	if(I2C_Handle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		/* in standard mode */
		tempReg = (GetClock() / 1000000) + 1;
	}
	else
	{
		/* in fast mode */
		tempReg = ((GetClock() * 300) / 1000000) + 1;
	}
	I2C_Handle->I2Cx->TRISE = tempReg & (0x3F);
}
void I2C_PeriClockControl(I2C_RegDef_t *I2Cx,uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
	{
	    if(I2Cx == I2C1)
	    {
        	I2C1_PCLK_EN();
        }
        if(I2Cx == I2C2)
	    {
        	I2C2_PCLK_EN();
	    }
	    if(I2Cx == I2C3)
	    {
        	I2C3_PCLK_EN();
        }
    }
    if(EnorDi == DISABLE)
	{
	    if(I2Cx == I2C1)
	    {
	        I2C1_PCLK_DI();
	    }
	    if(I2Cx == I2C2)
	    {
	        I2C2_PCLK_DI();
	    }
	    if(I2Cx == I2C3)
	    {
	        I2C3_PCLK_DI();
	    }
    }
    return;
}
void I2C_PeripheralControl(I2C_RegDef_t *I2Cx,uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
	{
		I2Cx->CR1 |= 1 << PE_BIT_POSITION;
		return;
	}
	else
	{
		I2Cx->CR1 &= ~(1 << PE_BIT_POSITION);
		return;
	}
}
void I2C_InteruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
    /* @NVIC_CONFIGURATION relation */
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
void I2C_PriorityConfig(uint8_t IRQNumber, uint8_t Priority)
{
    uint8_t RegLocation = IRQNumber / 4;
    uint8_t FieldLocation = IRQNumber % 4 ;
    uint8_t shift_amount = FieldLocation * 8 + 8 - IMPLEMENTED_BITS;
    uint32_t tempReg = NVIC->IPR[RegLocation] & ~(0xF << shift_amount);
    NVIC->IPR[RegLocation] = tempReg | (Priority << shift_amount);
    return;
}
static inline void SB_Ev_Handler(void)
{

}
static inline void ADDR_Ev_Handler(void)
{

}
static inline void BTF_Ev_Handler(void)
{

}
static inline void STOPF_Ev_Handler(void)
{

}
static inline void TXE_Ev_Handler(void)
{

}
static inline void RXNE_Ev_Handler(void)
{

}
void I2C_EV_IRQHandling(I2C_Handle_t *I2C_Handle)
{
	uint32_t temp1 = (I2C_Handle->I2Cx->CR2 >> ITEVTEN_BIT_POSITION) & 0x1;
	uint32_t temp2 = (I2C_Handle->I2Cx->CR2 >> ITBUFEN_BIT_POSITION) & 0x1;
	uint32_t temp3 = (I2C_Handle->I2Cx->SR1 >> I2C_SB_BIT_POSITION) & 0x1;
	if(temp3 & temp1)
	{
		/* SB flag is set */
		SB_Ev_Handler();
		if(I2C_Handle->TxRxStatus == I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAddressPhaseRead(I2C_Handle->I2Cx,I2C_Handle->SlaveAddress);
		}
		else
		{
			I2C_ExecuteAddressPhaseWrite(I2C_Handle->I2Cx,I2C_Handle->SlaveAddress);
		}
	}
	temp3 = (I2C_Handle->I2Cx->SR1 >> I2C_ADDR_BIT_POSITION) & 0x1;
	if(temp3 & temp1)
	{
		/* ADDR flag is set */
		ADDR_Ev_Handler();
		I2C_ADDRClearFlag(I2C_Handle);
	}
	temp3 = (I2C_Handle->I2Cx->SR1 >> I2C_BTF_BIT_POSITION) & 0x1;
	if(temp3 & temp1)
	{
		/* BTF flag is set */
		BTF_Ev_Handler();
		if(I2C_Handle->TxRxStatus == I2C_BUSY_IN_TX)
		{
			if(I2C_Handle->TxLength == 0)
			{
				if(I2C_GetFlagStatus(I2C_Handle->I2Cx,I2C_FLAG_TXE))
				{
					if(I2C_Handle->Sr == DISABLE)
					I2CStop(I2C_Handle->I2Cx);
					//reset all member(not done yet!)
					// LECTURE_221_CONTINUE
					//notify application about transmission complete(not done yet!)
				}
			}
		}
		else
		{

		}
	}
	temp3 = (I2C_Handle->I2Cx->SR1 >> I2C_STOPF_BIT_POSITION) & 0x1;
	if(temp3 & temp1)
	{
		/* STOPF flag is set */
		STOPF_Ev_Handler();

	}
	temp3 = (I2C_Handle->I2Cx->SR1 >> I2C_TXE_BIT_POSITION) & 0x1;
	if(temp3 & temp1 & temp2)
	{	
		/* TXE flag is set */
		TXE_Ev_Handler();
		if(I2C_Handle->I2Cx->SR1 >> I2C_MSL_BIT_POSITION & 0x1)
		{
			if(I2C_Handle->TxRxStatus == I2C_BUSY_IN_TX)
			{
				/* load the data into data register */
				I2C_Handle->I2Cx->DR = *(I2C_Handle->TxBuffer);
				/* increase the pointer pointing to the next byte */
				I2C_Handle->TxBuffer++;
				/* decrease the number of bytes need to be transmiting */
				I2C_Handle->TxLength--;
			}
		}
	}
	temp3 = (I2C_Handle->I2Cx->SR1 >> I2C_RXNE_BIT_POSITION) & 0x1;
	if(temp3 & temp1 & temp2)
	{
		/* RXNE flag is set */
		RXNE_Ev_Handler();
		if(I2C_Handle->TxRxStatus == I2C_BUSY_IN_RX)
		{
			if(I2C_Handle->RxSize == 1)
			{
				*(I2C_Handle->RxBuffer) = I2C_Handle->I2Cx->DR;
				I2C_Handle->RxLength--;
			}
			if(I2C_Handle->RxSize > 1)
			{
				if(I2C_Handle->RxLength == 2)
				{
					/* clear ack bit */
					ACK_Control(I2C_Handle->I2Cx,DISABLE);
				}
				*(I2C_Handle->RxBuffer) = I2C_Handle->I2Cx->DR;
				I2C_Handle->RxLength--;
				I2C_Handle->RxBuffer++;
			}
			if(I2C_Handle->RxLength == 0)
			{
				if(I2C_Handle->Sr == DISABLE)
				{
					I2CStop(I2C_Handle->I2Cx);
				}
			}
		}
	}





}
void I2C_ER_IRQHandling(I2C_Handle_t *I2C_Handle)
{

}
void I2C_ApplicationEventCallback(I2C_Handle_t * I2C_Handle, uint8_t AppEv)
{

}
