#include "stm32f4xx_spi.h"
#include "stm32f4xx_driver.h"

static void spi_ovr_handle(SPI_Handle_t *pSPIx);
static void spi_reception_handle(SPI_Handle_t *pSPIx);
static void spi_transmission_handle(SPI_Handle_t *pSPIx);
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
static uint8_t Is_Empty_Tx(SPI_RegDef_t *pSPIx)
{
	return (((pSPIx->SR)>>TXE_BIT_POSITION) & 0x00000001);
}
static uint8_t Is_Not_Empty_Rx(SPI_RegDef_t *pSPIx)
{
	return (((pSPIx->SR)>>RXNE_BIT_POSITION) & 0x00000001);
}
static uint8_t Is_16BitFrame(SPI_RegDef_t *pSPIx)
{
	return (((pSPIx->CR1)>>DFF_BIT_POSITION)  & 0x00000001);
}
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *Txbuffer, uint32_t Len)
{
	while(Len != EMPTY)
	{
		while(!Is_Empty_Tx(pSPIx))
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
	while(Len != EMPTY)
	{
		while(Is_Not_Empty_Rx(pSPIx))
		{

		}
		if(Is_16BitFrame(pSPIx))
		{
			/*16 bit data frame*/
			*(uint16_t*)Rxbuffer = pSPIx->DR;
			(uint16_t*)Rxbuffer++;
			Len -= 2;
		}
		else
		{
			/* 8 bit data frame*/
			*(uint8_t*)Rxbuffer = pSPIx->DR;
			(uint8_t*)Rxbuffer++;
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
static void spi_transmission_handle(SPI_Handle_t *pSPIx)
{
	if(Is_16BitFrame(pSPIx->SPIx))
		{
			/*16 bit data frame*/
			pSPIx->SPIx->DR = *((uint16_t*)pSPIx->Txbuffer);
			(uint16_t*)pSPIx->Txbuffer++;
			pSPIx->TxLength -= 2;
		}
		else
		{
			/* 8 bit data frame*/
			pSPIx->SPIx->DR = *((uint8_t*)pSPIx->Txbuffer);
			(uint8_t*)pSPIx->Txbuffer++;
			pSPIx->TxLength -= 1;
		}
	if(pSPIx->TxLength == 0)
	{
		SPI_CloseTransmission(pSPIx);
		//SPI_ApplicationCallback(pSPIx,SPI_EVENT_TX_CMPLT);
	}
	return;
}
static void spi_reception_handle(SPI_Handle_t *pSPIx)
{
	if(Is_16BitFrame(pSPIx->SPIx))
	{
		/*16 bit data frame*/
		*(uint16_t*)(pSPIx->Rxbuffer) = pSPIx->SPIx->DR;
		(uint16_t*)(pSPIx->Rxbuffer)++;
		pSPIx->RxLength -= 2;
	}
	else
	{
		/* 8 bit data frame*/
		*(uint8_t*)(pSPIx->Rxbuffer) = pSPIx->SPIx->DR;
		(uint8_t*)(pSPIx->Rxbuffer)++;
		pSPIx->RxLength -= 1;
	}
	if(pSPIx->TxLength == 0)
	{
		SPI_CloseReception(pSPIx);
		//SPI_ApplicationCallback(pSPIx,SPI_EVENT_RX_CMPLT);
	}
	return;

}
static void spi_ovr_handle(SPI_Handle_t *pSPIx)
{
	//clear ovr flag
	uint8_t temp;
	if(pSPIx->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIx->SPIx->DR;
		temp = pSPIx->SPIx->SR;
	}
	(void)temp;
	//inform application
	//SPI_ApplicationCallback(pSPIx,SPI_EVENT_OVR_ERR);

}
void SPI_CloseTransmission(SPI_Handle_t *pSPIx)
{
	pSPIx->SPIx->CR2 &= ~(1 << TXEIE_BIT_POSITION);
	pSPIx->Txbuffer = NULL;
	pSPIx->TxLength = 0;
	pSPIx->TxState = SPI_READY;
	SPI_PeripheralControl(SPI2,DISABLE);
}
void SPI_CloseReception(SPI_Handle_t *pSPIx)
{
	pSPIx->SPIx->CR2 &= ~(1 << RXNEIE_BIT_POSITION);
	pSPIx->Rxbuffer = NULL;
	pSPIx->RxLength = 0;
	pSPIx->RxState = SPI_READY;
	SPI_PeripheralControl(SPI2,DISABLE);
}
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	pSPIx->SR &= ~(1 << SPI_OVR_BIT_POSITION);
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
void SPI_IrqHandling(SPI_Handle_t *SPI_Handle_t)
{
	uint8_t temp1, temp2;
	temp1 = SPI_Handle_t->SPIx->CR2 & (1 << TXEIE_BIT_POSITION);
	temp2 = SPI_Handle_t->SPIx->SR & (1 << TXE_BIT_POSITION);
	if(temp1 && temp2)
	{
		spi_transmission_handle(SPI_Handle_t);
	}
	temp1 = SPI_Handle_t->SPIx->CR2 & (1 << RXNEIE_BIT_POSITION);
	temp2 = SPI_Handle_t->SPIx->SR & (1 << RXNE_BIT_POSITION);
	if(temp1 && temp2)
	{
		spi_reception_handle(SPI_Handle_t);
	}
	temp1 = SPI_Handle_t->SPIx->CR2 & (1 << ERRIE_BIT_POSITION);
	temp2 = SPI_Handle_t->SPIx->SR & (1 << SPI_OVR_BIT_POSITION);
	if(temp1 && temp2)
	{
		spi_ovr_handle(SPI_Handle_t);
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
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIx_handle, uint8_t *Txbuffer, uint32_t Len)
{
	uint8_t state = pSPIx_handle->TxState;
	if(state != SPI_BUSY_IN_TX)
	{
		//1. save the tx buffer address and length information in some variables
		pSPIx_handle->Txbuffer = Txbuffer;
		pSPIx_handle->TxLength = Len;
		//2. mark SPI state as busy so no other transmission could take the same SPI until transmission is done
		pSPIx_handle->TxState = SPI_BUSY_IN_TX;
		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIx_handle->SPIx->CR2 |= 1 << TXEIE_BIT_POSITION;
		// Data transmission is handled by the ISR code
	}
	return state;

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
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIx_handle, uint8_t *Rxbuffer, uint32_t Len)
{
	uint8_t state = pSPIx_handle->RxState;
	if(state != SPI_BUSY_IN_RX)
	{
		//1. save the rx buffer address and length information in some variables
		pSPIx_handle->Rxbuffer = Rxbuffer;
		pSPIx_handle->RxLength = Len;
		//2. mark SPI state as busy so no other reception could take the same SPI until reception is done
		pSPIx_handle->RxState = SPI_BUSY_IN_RX;
		//3. Enable the RXNEIE control bit to get interrupt whenever RXNEIE flag is set in SR
		pSPIx_handle->SPIx->CR2 |= 1 << RXNEIE_BIT_POSITION;
		// Data reception is handled by the ISR code
	}
	return state;
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
