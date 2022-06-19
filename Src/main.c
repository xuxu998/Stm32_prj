#include "stm32f4xx_driver.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_spi.h"
#include <string.h>
#undef GPIO_LECTURE
#if defined GPIO_LECTURE
GPIO_PinConfig_t UserLed =
{
		.GPIO_PinNumber = GPIO_PIN_NO_14,
		.GPIO_PinMode = GPIO_MODE_OUT,
		.GPIO_PinSpeed = GPIO_SPEED_LOW,
		.GPIO_PinPuPdControl = GPIO_NO_PUPD,
		.GPIO_PinOpType = GPIO_OP_TYPE_PP,
		.GPIO_AltFunMode = 0
};

GPIO_PinConfig_t UserButton =
{
		.GPIO_PinNumber = GPIO_PIN_NO_5,
		.GPIO_PinMode = GPIO_MODE_IT_FT,
		.GPIO_PinSpeed = GPIO_SPEED_LOW,
		.GPIO_PinPuPdControl = GPIO_PIN_PU,
		.GPIO_AltFunMode = 0
};
void Delay(void)
{
	for(int us = 0 ; us <= 500000 ; us++)
	{

	}
}
void EXTI9_5_IRQHandler(void)
{
	Delay();
	GPIO_IrqHandling(5);
	GPIO_TogglePin(GPIOD,GPIO_PIN_NO_14);
}
int main(void)
{
	GPIO_Handle_t GpioLed;
	memset(&GpioLed,0,sizeof(GpioLed));
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig = UserLed;
	GPIO_PeriClockControl(GpioLed.pGPIOx,ENABLE);
	GPIO_Init(&GpioLed);

	GPIO_Handle_t GpioButton;
	memset(&GpioButton,0,sizeof(GpioButton));
	GpioButton.pGPIOx = GPIOD;
	GpioButton.GPIO_PinConfig = UserButton;
	GPIO_PeriClockControl(GpioButton.pGPIOx,ENABLE);
	GPIO_Init(&GpioButton);
	GPIO_WriteOutputPin(GPIOD,GPIO_PIN_NO_14,GPIO_PIN_RESET);
	GPIO_PriorityConfig(IRQ_NO_EXTI9_5,10);
	GPIO_InteruptConfig(IRQ_NO_EXTI9_5,ENABLE);
	while(1)
	{

	}


	return 0;
}
#endif
__attribute((weak))void __SPI_ApplicationCallback(SPI_Handle_t* pSPI_handle,uint8_t EventNumber)
{

}
void Delay(void)
{
	for(int us = 0 ; us <= 500000 ; us++)
	{

	}
}
GPIO_PinConfig_t UserButton =
{
		.GPIO_PinNumber = GPIO_PIN_NO_5,
		.GPIO_PinMode = GPIO_MODE_IT_FT,
		.GPIO_PinSpeed = GPIO_SPEED_LOW,
		.GPIO_PinPuPdControl = GPIO_PIN_PU,
		.GPIO_AltFunMode = 0
};
SPI_Handle_t SPI2handle;
char user_data[] = "hello world pham dat";
void SPI2_IRQHandler(void)
{
	SPI_IrqHandling(&SPI2handle);
}
void EXTI9_5_IRQHandler(void)
{
	Delay();
	GPIO_IrqHandling(5);
	SPI_PeripheralControl(SPI2,ENABLE);
	SPI_SendDataIT(&SPI2handle,(uint8_t*)user_data,(uint32_t)strlen(user_data));
	//while(SPI_GetStatusFlag(SPI2));
	//SPI_PeripheralControl(SPI2,DISABLE);
	//GPIO_IrqHandling(5);
	//GPIO_TogglePin(GPIOD,GPIO_PIN_NO_14);
}
/*
 * PB12 SPI2_NSS
 * PB13 SPI2_SCK
 * PB14 SPI2_MISO
 * PB15 SPI2_MOSI
 * */
void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_AltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	SPIPins.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	//CLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);
	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);
	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

}
void SPI2_Init(void)
{
	SPI2handle.SPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_FULL_DUPLEX;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MASTER;
	SPI2handle.SPIConfig.SPI_ClkSpeed = SPI_CLOCK_DIV2;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_HIGH;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_HIGH;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN;
	SPI_Init(&SPI2handle);

}
int main(void)
{
	GPIO_Handle_t GpioButton;
		memset(&GpioButton,0,sizeof(GpioButton));
		GpioButton.pGPIOx = GPIOD;
		GpioButton.GPIO_PinConfig = UserButton;
		GPIO_PeriClockControl(GpioButton.pGPIOx,ENABLE);
		GPIO_Init(&GpioButton);
		GPIO_PriorityConfig(IRQ_NO_EXTI9_5,10);
		GPIO_InteruptConfig(IRQ_NO_EXTI9_5,ENABLE);
	SPI2_GPIOInits();
	SPI2_Init();
	SPI_InteruptConfig(36,ENABLE);
	SPI_PriorityConfig(36,11);
	//enable SPI2 peripherals
	SPI_SSIConfig(SPI2,ENABLE);
	//SPI_SSOEConfig(SPI2,ENABLE);
	//SPI_PeripheralControl(SPI2,ENABLE);
	//SPI_SSOEConfig(SPI2,ENABLE);
	//SPI_SendData(SPI2,(uint8_t*)user_data,(uint32_t)strlen(user_data));
	//SPI_PeripheralControl(SPI2,DISABLE);
	while(1)
	{
		/*while(GPIO_ReadFromInputPin(GPIOD,GPIO_PIN_NO_5));
		Delay();
		SPI_PeripheralControl(SPI2,ENABLE);
		SPI_SendData(SPI2,(uint8_t*)user_data,(uint32_t)strlen(user_data));
		while(SPI_GetStatusFlag(SPI2));
		SPI_PeripheralControl(SPI2,DISABLE);*/

	}
	return 0;
}

