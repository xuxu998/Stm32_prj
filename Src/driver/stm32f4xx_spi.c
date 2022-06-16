#include "stm32f4xx_spi.h"
#include "stm32f4xx_driver.h"


/******************************************************************
 * @fn              -
 *
 * @brief
 *
 * @param[in]       -
 * @param[in]       -
 * @param[in]       -
 *
 * @return          -
 *
 * @Note            -
 *
 */
void SPI_Init(SPI_Handle_t *pSPIx)
{
	uint32_t tempReg = 0;
	/*enable clock for SPIx*/
	SPI_PeriClockControl(pSPIx->SPIx,ENABLE);
	/*Config device mode for microcontroller*/
	tempReg |= pSPIx->SPIConfig.SPI_DeviceMode << MSRT_BIT_POSITION;
	/*Config bus configuration*/
	if(pSPIx->SPIConfig.SPI_BusConfig == SPI_BUS_FULL_DUPLEX)
	{
		tempReg &= ~(1 << BIDI_MODE_BIT_POSITION);
	}
	else if(pSPIx->SPIConfig.SPI_BusConfig == SPI_BUS_HAFL_DUPLEX)
	{
		tempReg |= 1 << BIDI_MODE_BIT_POSITION;
	}
	else if (pSPIx->SPIConfig.SPI_BusConfig == SPI_BUS_SIMPLEX_RX_ONLY)
	{
		tempReg |= 1 << BIDI_MODE_BIT_POSITION;
		tempReg |= 1 << RX_ONLY_BIT_POSITION;
	}
	/*Config clock speed for SPI*/
	tempReg |= pSPIx->SPIConfig.SPI_ClkSpeed << BR_BIT_POSITION;
	/*Config data frame for SPI*/
	tempReg |= pSPIx->SPIConfig.SPI_DFF << DFF_BIT_POSITION;
	/*Config CPOL and CPHA for SPI*/
	tempReg |= pSPIx->SPIConfig.SPI_CPHA << CPHA_BIT_POSITION;
	tempReg |= pSPIx->SPIConfig.SPI_CPOL << CPOL_BIT_POSITION;
	/*Config software management for slave slected*/
	tempReg |= pSPIx->SPIConfig.SPI_SSM << SSM_BIT_POSITION;

	pSPIx->SPIx->CR1 = tempReg;

}
/******************************************************************
 * @fn              -
 *
 * @brief
 *
 * @param[in]       -
 * @param[in]       -
 * @param[in]       -
 *
 * @return          -
 *
 * @Note            -
 *
 */
void SPI_Deinit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_PCLK_DI();
	}
	if(pSPIx == SPI2)
	{
		SPI2_PCLK_DI();
	}
	if(pSPIx == SPI3)
	{
		SPI3_PCLK_DI();
	}
	if(pSPIx == SPI4)
	{
		SPI4_PCLK_DI();
	}
	if(pSPIx == SPI5)
	{
		SPI5_PCLK_DI();
	}
	if(pSPIx == SPI6)
	{
		SPI6_PCLK_DI();
	}
}
/******************************************************************
 * @fn              -
 *
 * @brief
 *
 * @param[in]       -
 * @param[in]       -
 * @param[in]       -
 *
 * @return          -
 *
 * @Note            -
 *
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	    {
	        if(pSPIx == SPI1)
	        {
	        	SPI1_PCLK_EN();
	        }
	        if(pSPIx == SPI2)
	        {
	        	SPI2_PCLK_EN();
	        }
	        if(pSPIx == SPI3)
	        {
	        	SPI3_PCLK_EN();
	        }
	        if(pSPIx == SPI4)
	        {
	        	SPI4_PCLK_EN();
	        }
	        if(pSPIx == SPI5)
	        {
	        	SPI5_PCLK_EN();
	        }
	        if(pSPIx == SPI6)
	        {
	        	SPI6_PCLK_EN();
	        }
	    }
	    else
	    {
	        if(pSPIx == SPI1)
	        {
	        	SPI1_PCLK_DI();
	        }
	        if(pSPIx == SPI2)
	        {
	        	SPI2_PCLK_DI();
	        }
	        if(pSPIx == SPI3)
	        {
	        	SPI3_PCLK_DI();
	        }
	        if(pSPIx == SPI4)
	        {
	        	SPI4_PCLK_DI();
	        }
	        if(pSPIx == SPI5)
	        {
	        	SPI5_PCLK_DI();
	        }
	        if(pSPIx == SPI6)
	        {
	        	SPI6_PCLK_DI();
	        }
	    }
}

/******************************************************************
 * @fn              -
 *
 * @brief
 *
 * @param[in]       -
 * @param[in]       -
 * @param[in]       -
 *
 * @return          -
 *
 * @Note            -
 *
 */
static uint8_t Is_Empty(SPI_RegDef_t *pSPIx)
{
	return (((pSPIx->SR)>>TXE_BIT_POSITION) & 0x00000001);
}
static uint8_t Is_16BitFrame(SPI_RegDef_t *pSPIx)
{
	return (((pSPIx->CR1)>>DFF_BIT_POSITION)  & 0x00000001);
}
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *Txbuffer, uint32_t Len)
{
	while(Len != EMPTY)
	{
		while(!Is_Empty(pSPIx))
		{

		}
		if(Is_16BitFrame(pSPIx))
		{
			/*16 bit data frame*/
			pSPIx->DR = *((uint16_t*)Txbuffer);
			(uint16_t*)Txbuffer++;
			Len -= 2;
		}
		else
		{
			/* 8 bit data frame*/
			pSPIx->DR = *((uint8_t*)Txbuffer);
			(uint8_t*)Txbuffer++;
			Len -= 1;
		}
	}
	return;
}
/******************************************************************
 * @fn              -
 *
 * @brief
 *
 * @param[in]       -
 * @param[in]       -
 * @param[in]       -
 *
 * @return          -
 *
 * @Note            -
 *
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *Rxbuffer, uint32_t Len)
{

}
/******************************************************************
 * @fn              -
 *
 * @brief
 *
 * @param[in]       -
 * @param[in]       -
 * @param[in]       -
 *
 * @return          -
 *
 * @Note            -
 *
 */
void SPI_InteruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

}
/******************************************************************
 * @fn              -
 *
 * @brief
 *
 * @param[in]       -
 * @param[in]       -
 * @param[in]       -
 *
 * @return          -
 *
 * @Note            -
 *
 */

/******************************************************************
 * @fn              -
 *
 * @brief
 *
 * @param[in]       -
 * @param[in]       -
 * @param[in]       -
 *
 * @return          -
 *
 * @Note            -
 *
 */
void SPI_IrqHandling(uint8_t PinNum)
{

}
/******************************************************************
 * @fn              -
 *
 * @brief
 *
 * @param[in]       -
 * @param[in]       -
 * @param[in]       -
 *
 * @return          -
 *
 * @Note            -
 *
 */
void SPI_PriorityConfig(SPI_Handle_t *pHandle)
{

}
/******************************************************************
 * @fn              -
 *
 * @brief
 *
 * @param[in]       -
 * @param[in]       -
 * @param[in]       -
 *
 * @return          -
 *
 * @Note            -
 *
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPE_BIT_POSITION);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPE_BIT_POSITION);
	}
}
/******************************************************************
 * @fn              -
 *
 * @brief
 *
 * @param[in]       -
 * @param[in]       -
 * @param[in]       -
 *
 * @return          -
 *
 * @Note            -
 *
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= 1 << SSI_BIT_POSITION;
		return;
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SSI_BIT_POSITION);
		return;
	}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SSOE_BIT_POSIITON);
		return;
	}
	else
	{
		pSPIx->CR2 &= ~(1 << SSOE_BIT_POSIITON);
		return;
	}
}
/******************************************************************
 * @fn              -
 *
 * @brief
 *
 * @param[in]       -
 * @param[in]       -
 * @param[in]       -
 *
 * @return          -
 *
 * @Note            -
 *
 */
uint8_t SPI_GetStatusFlag(SPI_RegDef_t *pSPIx)
{
	return (uint8_t)((pSPIx->SR) >> BSY_BIT_POSITION);
}
/******************************************************************
 * @fn              -
 *
 * @brief
 *
 * @param[in]       -
 * @param[in]       -
 * @param[in]       -
 *
 * @return          -
 *
 * @Note            -
 *
 */
void SPI_SendDataIT(SPI_Handle_t *pSPIx_handle, uint8_t *Txbuffer, uint32_t Len)
{

}
/******************************************************************
 * @fn              -
 *
 * @brief
 *
 * @param[in]       -
 * @param[in]       -
 * @param[in]       -
 *
 * @return          -
 *
 * @Note            -
 *
 */
void SPI_ReceiveDataIT(SPI_Handle_t *pSPIx_handle, uint8_t *Rxbuffer, uint32_t Len)
{

}
void SPI_InteruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
#if 1
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
#endif
#if 0
    if(EnorDi == ENABLE)
    	{
    		if(IRQNumber <= 31)
    		{
    			//program ISER0 register
    			*NVIC_ISER0 |= ( 1 << IRQNumber );

    		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
    		{
    			//program ISER1 register
    			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
    		}
    		else if(IRQNumber >= 64 && IRQNumber < 96 )
    		{
    			//program ISER2 register //64 to 95
    			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
    		}
    	}
    else
    	{
    		if(IRQNumber <= 31)
    		{
    			//program ICER0 register
    			*NVIC_ICER0 |= ( 1 << IRQNumber );
    		}else if(IRQNumber > 31 && IRQNumber < 64 )
    		{
    			//program ICER1 register
    			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
    		}
    		else if(IRQNumber >= 64 && IRQNumber < 96 )
    		{
    			//program ICER2 register
    			*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
    		}
        }
#endif
}
void SPI_PriorityConfig(uint8_t IRQNumber, uint8_t Priority)
{
#if 1
    uint8_t RegLocation = IRQNumber / 4;
    uint8_t FieldLocation = IRQNumber % 4 ;
    uint8_t shift_amount = FieldLocation * 8 + 8 - IMPLEMENTED_BITS;
    uint32_t tempReg = NVIC->IPR[RegLocation] & ~(0xF << shift_amount);
    NVIC->IPR[RegLocation] = tempReg | (Priority << shift_amount);
    return;
#endif
#if 0
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx  ) |=  ( Priority << shift_amount );
#endif


}
void SPI_IrqHandling(uint8_t PinNum)
{

}
