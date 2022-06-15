#include "stm32f4xx_i2c.h"
#include "stm32f4xx_driver.h"
#include <stdint.h>
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