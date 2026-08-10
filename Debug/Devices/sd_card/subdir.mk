################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Devices/sd_card/sd_card.c \
../Devices/sd_card/sd_logger.c 

OBJS += \
./Devices/sd_card/sd_card.o \
./Devices/sd_card/sd_logger.o 

C_DEPS += \
./Devices/sd_card/sd_card.d \
./Devices/sd_card/sd_logger.d 


# Each subdirectory must supply rules for building sources it contributes
Devices/sd_card/%.o Devices/sd_card/%.su Devices/sd_card/%.cyclo: ../Devices/sd_card/%.c Devices/sd_card/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F446xx -c -I../Core/Inc -I../Drivers/dac -I../Drivers/adc -I../Devices/sd_card -I../Devices/oled -I../Drivers/uart -I../Drivers/timer -I../Drivers/spi -I../Protocol -I../Drivers -I../Drivers/gpio -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/crc_hw -I../Drivers/i2c -I../Devices/bar30 -I../Drivers/iwdg -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Devices-2f-sd_card

clean-Devices-2f-sd_card:
	-$(RM) ./Devices/sd_card/sd_card.cyclo ./Devices/sd_card/sd_card.d ./Devices/sd_card/sd_card.o ./Devices/sd_card/sd_card.su ./Devices/sd_card/sd_logger.cyclo ./Devices/sd_card/sd_logger.d ./Devices/sd_card/sd_logger.o ./Devices/sd_card/sd_logger.su

.PHONY: clean-Devices-2f-sd_card

