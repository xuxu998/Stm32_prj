#include "stm32f4xx_gpio.h"

/******************************************************************
 * @fn              - GPIO_Init
 *
 * @brief           - Inititialize GPIO port based on given parameters
 *
 * @param[in]       - GPIO_Handle_t struct type of GPIO
 * @param[in]       -
 * @param[in]       -
 *
 * @return          - None
 *
 * @Note            - None
 *
*/
void GPIO_Init(GPIO_Handle_t *pGPIO)
{
    uint32_t tempReg;
    /* config the mode of the pin */
    /*enable clock for GPIOx*/
    GPIO_PeriClockControl(pGPIO->pGPIOx,ENABLE);
    if(pGPIO->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
    {
        tempReg = pGPIO->pGPIOx->MODER & ~(3 << pGPIO->GPIO_PinConfig.GPIO_PinNumber * 2);
        pGPIO->pGPIOx->MODER = tempReg | (pGPIO->GPIO_PinConfig.GPIO_PinMode << pGPIO->GPIO_PinConfig.GPIO_PinNumber * 2);
    }
     /*  this code only does configuration from microcontroller side including SYSCFG and EXTI configuration
      * another side which has configurations needed is NVIC controller which is implemented below @NVIC_CONFIGURATION*/
    else
    {
    	/*Init clock for EXTI and SYSCFG peripherals*/
    	//EXTI_CLK_ENB();
        /* Configuration of EXTI peripheral */
        if(pGPIO->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
        {
            /* config the FTRS register */
            EXTI->FTSR |= (1 << pGPIO->GPIO_PinConfig.GPIO_PinNumber);
            EXTI->RTSR &= ~(1 << pGPIO->GPIO_PinConfig.GPIO_PinNumber);

        }
        else if(pGPIO->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
        {
            /* config the RTRS register */
            EXTI->RTSR |= (1 << pGPIO->GPIO_PinConfig.GPIO_PinNumber);
            EXTI->FTSR &= ~(1 << pGPIO->GPIO_PinConfig.GPIO_PinNumber);
        }
        else if (pGPIO->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
        {
            /* config both RTRS and FRRS */
            EXTI->FTSR |= (1 << pGPIO->GPIO_PinConfig.GPIO_PinNumber);
            EXTI->RTSR |= (1 << pGPIO->GPIO_PinConfig.GPIO_PinNumber);
        }

        /* config interupt mask register from EXTI side
         * this register allows interupt to be deliveried to NVIC controller */
        EXTI->IMR |= (1 << pGPIO->GPIO_PinConfig.GPIO_PinNumber);

        /* End of configuration of EXTI peripheral */


        /* Configuration of SYSFG peripherals */

        /* config GPIO port selection in SYSCFG_EXTICR */
        uint8_t temp1 = pGPIO->GPIO_PinConfig.GPIO_PinNumber / 4 ;
        		uint8_t temp2 = pGPIO->GPIO_PinConfig.GPIO_PinNumber % 4;
        		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIO->pGPIOx);
        		SYSCFG_PCLK_EN();
        		SYSCFG->EXTICR[temp1] = portcode << ( temp2 * 4);

        /* End of configuration of SYSFG peripherals */
    }
    /* config speed */
    tempReg = 0;
    tempReg = pGPIO->pGPIOx->OSPEEDR & ~(0x3 << pGPIO->GPIO_PinConfig.GPIO_PinNumber * 2); /* 2 bits need to be deleted */
    pGPIO->pGPIOx->OSPEEDR = tempReg | (pGPIO->GPIO_PinConfig.GPIO_PinSpeed << pGPIO->GPIO_PinConfig.GPIO_PinNumber * 2);
    /* config pupd  */
    tempReg = 0;
    tempReg = pGPIO->pGPIOx->PUPDR & ~(0x3 << pGPIO->GPIO_PinConfig.GPIO_PinNumber * 2); /* 2 bits need to be deleted */
    pGPIO->pGPIOx->PUPDR = tempReg | (pGPIO->GPIO_PinConfig.GPIO_PinPuPdControl << pGPIO->GPIO_PinConfig.GPIO_PinNumber * 2);

    /*OUTPUT type*/
    if(pGPIO->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_OUT)
    {
		tempReg = 0;
		tempReg = pGPIO->pGPIOx->OTYPER & ~(0x1 << pGPIO->GPIO_PinConfig.GPIO_PinNumber); /* 1 bit need to be deleted */
		pGPIO->pGPIOx->OTYPER = tempReg | (pGPIO->GPIO_PinConfig.GPIO_PinOpType << pGPIO->GPIO_PinConfig.GPIO_PinNumber);
    }

    /* Alternate function */
    if(pGPIO->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALT)
    {
        tempReg = 0;
        if(pGPIO->GPIO_PinConfig.GPIO_PinNumber < 8)
        {
            tempReg = pGPIO->pGPIOx->AFR[0] & ~(0xF << pGPIO->GPIO_PinConfig.GPIO_PinNumber * 4);
            pGPIO->pGPIOx->AFR[0] = tempReg | (pGPIO->GPIO_PinConfig.GPIO_AltFunMode << pGPIO->GPIO_PinConfig.GPIO_PinNumber * 4);
        }
        else
        {
            uint8_t PinLocation = pGPIO->GPIO_PinConfig.GPIO_PinNumber % 8;
            tempReg = pGPIO->pGPIOx->AFR[1] & ~(0xF << PinLocation * 4); /* 4 bits need to be deleted */
            pGPIO->pGPIOx->AFR[1] = tempReg | (pGPIO->GPIO_PinConfig.GPIO_AltFunMode << PinLocation * 4);
        }
    }
    return;
}

/******************************************************************
 * @fn              - GPIO_Deinit
 *
 * @brief           - Deinitialize a GPIO port
 *
 * @param[in]       - Base address of a GPIO
 * @param[in]       -
 * @param[in]       -
 *
 * @return          - None
 *
 * @Note            - None
 *
 */
void GPIO_Deinit(GPIO_RegDef_t *pGPIOx)
{
    if(pGPIOx == GPIOA)
    {
        GPIOA_REG_RESET();
    }
    if(pGPIOx == GPIOB)
    {
        GPIOB_REG_RESET();
    }
    if(pGPIOx == GPIOC)
    {
        GPIOC_REG_RESET();
    }
    if(pGPIOx == GPIOD)
    {
        GPIOD_REG_RESET();
    }
    if(pGPIOx == GPIOE)
    {
        GPIOE_REG_RESET();
    }
    if(pGPIOx == GPIOF)
    {
        GPIOF_REG_RESET();
    }
    if(pGPIOx == GPIOG)
    {
        GPIOG_REG_RESET();
    }
    if(pGPIOx == GPIOH)
    {
        GPIOH_REG_RESET();
    }
    if(pGPIOx == GPIOI)
    {
        GPIOI_REG_RESET();
    }
    if(pGPIOx == GPIOJ)
    {
        GPIOJ_REG_RESET();
    }
}

/******************************************************************
 * @fn              - GPIO_PeriClockControl
 *
 * @brief
 *
 * @param[in]       - base address of GPIO port
 * @param[in]       - Enable or Disable macros
 * @param[in]       -
 *
 * @return          -
 *
 * @Note            -
 *
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        if(pGPIOx == GPIOA)
        {
            GPIOA_PCLK_EN();
        }
        if(pGPIOx == GPIOB)
        {
            GPIOB_PCLK_EN();
        }
        if(pGPIOx == GPIOC)
        {
            GPIOC_PCLK_EN();
        }
        if(pGPIOx == GPIOD)
        {
            GPIOD_PCLK_EN();
        }
        if(pGPIOx == GPIOE)
        {
            GPIOE_PCLK_EN();
        }
        if(pGPIOx == GPIOF)
        {
            GPIOF_PCLK_EN();
        }
        if(pGPIOx == GPIOG)
        {
            GPIOG_PCLK_EN();
        }
        if(pGPIOx == GPIOH)
        {
            GPIOH_PCLK_EN();
        }
        if(pGPIOx == GPIOI)
        {
            GPIOI_PCLK_EN();
        }
        if(pGPIOx == GPIOJ)
        {
            GPIOJ_PCLK_EN();
        }

    }
    else
    {
        if(pGPIOx == GPIOA)
        {
            GPIOA_PCLK_DI();
        }
        if(pGPIOx == GPIOB)
        {
            GPIOB_PCLK_DI();
        }
        if(pGPIOx == GPIOC)
        {
            GPIOC_PCLK_DI();
        }
        if(pGPIOx == GPIOD)
        {
            GPIOD_PCLK_DI();
        }
        if(pGPIOx == GPIOE)
        {
            GPIOE_PCLK_DI();
        }
        if(pGPIOx == GPIOF)
        {
            GPIOF_PCLK_DI();
        }
        if(pGPIOx == GPIOG)
        {
            GPIOG_PCLK_DI();
        }
        if(pGPIOx == GPIOH)
        {
            GPIOH_PCLK_DI();
        }
        if(pGPIOx == GPIOI)
        {
            GPIOI_PCLK_DI();
        }
        if(pGPIOx == GPIOJ)
        {
            GPIOJ_PCLK_DI();
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
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum )
{
    uint8_t bit_value;
    bit_value = (uint8_t)((uint32_t)(pGPIOx->IDR >> PinNum) & (uint32_t)(0x00000001));
    return bit_value;
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
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
    return (uint16_t)(pGPIOx->IDR);
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
void GPIO_WriteOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum, uint8_t value)
{
    #if 1
    uint32_t tempReg;
    tempReg = pGPIOx->ODR & ~(uint32_t)(1 << PinNum);
    pGPIOx->ODR = tempReg | (value << PinNum);
    return;
    #endif
    #if 0
    /*another implementation for this function:*/
    if(value == GPIO_PIN_SET)
    {
        pGPIOx->ODR |= 1 << PinNum;
    }
    else
    {
        pGPIOx->ODR &= ~(1 << PinNum);
    }
    return
    #endif
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
void GPIO_WriteOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
    pGPIOx->ODR = value;
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
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum)
{
    /* another implementation for this function */
    pGPIOx->ODR ^= (1 << PinNum);
    return;
}
void GPIO_PriorityConfig(uint8_t IRQNumber, uint8_t Priority)
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
void GPIO_InteruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
void GPIO_IrqHandling(uint8_t PinNum)
{
	if(EXTI->PR & (1 << PinNum))
	{
		EXTI->PR |= (1 << PinNum);
	}
}
