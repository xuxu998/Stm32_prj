#include "stm32f4xx_driver.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_uart.h"
#include <string.h>
//#define GPIO_LECTURE
//#define SPI_LECTURE
//#define I2C_LECTURE
#define UART_LECTURE
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
#if defined SPI_LECTURE
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
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
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
	//SPI_SSIConfig(SPI2,DISABLE);
	SPI_SSOEConfig(SPI2,ENABLE);
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
#endif
#if defined I2C_LECTURE
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
I2C_Handle_t I2C_handle;
uint8_t buffer[] = "pham ngoc dat";
void EXTI9_5_IRQHandler(void)
{
	Delay();
	GPIO_IrqHandling(5);
	I2C_MasterSendData(&I2C_handle,buffer,sizeof(buffer),30);
}
void I2C_Initialization(void)
{
	GPIO_Handle_t I2CPins;
	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_AltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	I2CPins.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_OD;
	//SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = 6;
	GPIO_Init(&I2CPins);
	//SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = 9;
	GPIO_Init(&I2CPins);

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


	memset(&I2C_handle,0, sizeof(I2C_handle));
	I2C_handle.I2C_Config.I2C_SCLSpeed =(uint32_t)I2C_SCL_SPEED_SM;
	I2C_handle.I2C_Config.I2C_DeviceAddress = 0x61;
	I2C_handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C_handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C_handle.I2Cx = I2C1;
	I2C_Init(&I2C_handle);
	I2C_Initialization();
	I2C_PeripheralControl(I2C1,ENABLE);
	//I2C_MasterSendData(&I2C_handle,buffer,sizeof(buffer),10);
	while(1)
	{
	}
}
#endif
#ifdef UART_LECTURE
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
USART_Handle_t USART_handle;
uint8_t buffer[] = "pham ngoc dat";
void UART4_IRQHandler(void)
{
	USART_Handling(&USART_handle);
}
void EXTI9_5_IRQHandler(void)
{
	Delay();
	GPIO_IrqHandling(5);
	//USART_TransmitData(&USART_handle,buffer,sizeof(buffer));
	USART_TransmitDataIT(&USART_handle,buffer,sizeof(buffer));
}
void UART_Initialization()
{
	GPIO_Handle_t UARTPin;
	UARTPin.pGPIOx = GPIOA;
	UARTPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT;
	UARTPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	UARTPin.GPIO_PinConfig.GPIO_AltFunMode = 8;
	UARTPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	UARTPin.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	UARTPin.GPIO_PinConfig.GPIO_PinNumber = 0;
	GPIO_Init(&UARTPin);
	UARTPin.GPIO_PinConfig.GPIO_PinNumber = 1;
	GPIO_Init(&UARTPin);
}
int main(void)
{
	GPIO_Handle_t GpioButton;
	UART_Initialization();
	memset(&GpioButton,0,sizeof(GpioButton));
	GpioButton.pGPIOx = GPIOD;
	GpioButton.GPIO_PinConfig = UserButton;
	GPIO_PeriClockControl(GpioButton.pGPIOx,ENABLE);
	GPIO_Init(&GpioButton);
	GPIO_PriorityConfig(IRQ_NO_EXTI9_5,10);
	GPIO_InteruptConfig(IRQ_NO_EXTI9_5,ENABLE);

	USART_handle.USARTx = UART4;
	USART_handle.UART_Config.BaudRate = 115200;
	USART_handle.UART_Config.NumberOfStopBit = STOP_BIT_2;
	USART_handle.UART_Config.Parity = UART_PARITY_DISABLE;
	USART_handle.UART_Config.WordLength = UART_WORD_LENGTH_8_BITS;
	USART_handle.TxState = UART_READY;
	USART_InteruptConfig(UART4_IRQ_NUMBER,ENABLE);
	USART_PriorityConfig(UART4_IRQ_NUMBER,UART4_DEFAULT_PRIORITY);
	USART_Init(&USART_handle);
	Control_USART(UART4,ENABLE);
	while(1)
	{

	}

}
#endif
