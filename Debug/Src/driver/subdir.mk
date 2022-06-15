################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/driver/stm32f4xx_gpio.c \
../Src/driver/stm32f4xx_i2c.c \
../Src/driver/stm32f4xx_spi.c 

OBJS += \
./Src/driver/stm32f4xx_gpio.o \
./Src/driver/stm32f4xx_i2c.o \
./Src/driver/stm32f4xx_spi.o 

C_DEPS += \
./Src/driver/stm32f4xx_gpio.d \
./Src/driver/stm32f4xx_i2c.d \
./Src/driver/stm32f4xx_spi.d 


# Each subdirectory must supply rules for building sources it contributes
Src/driver/%.o Src/driver/%.su: ../Src/driver/%.c Src/driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F407VGTx -c -I"D:/STM32_IDE/DatProject/driver/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src-2f-driver

clean-Src-2f-driver:
	-$(RM) ./Src/driver/stm32f4xx_gpio.d ./Src/driver/stm32f4xx_gpio.o ./Src/driver/stm32f4xx_gpio.su ./Src/driver/stm32f4xx_i2c.d ./Src/driver/stm32f4xx_i2c.o ./Src/driver/stm32f4xx_i2c.su ./Src/driver/stm32f4xx_spi.d ./Src/driver/stm32f4xx_spi.o ./Src/driver/stm32f4xx_spi.su

.PHONY: clean-Src-2f-driver

