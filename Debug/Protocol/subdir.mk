################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Protocol/packet.c \
../Protocol/ring_buffer.c \
../Protocol/struct.c 

OBJS += \
./Protocol/packet.o \
./Protocol/ring_buffer.o \
./Protocol/struct.o 

C_DEPS += \
./Protocol/packet.d \
./Protocol/ring_buffer.d \
./Protocol/struct.d 


# Each subdirectory must supply rules for building sources it contributes
Protocol/%.o Protocol/%.su Protocol/%.cyclo: ../Protocol/%.c Protocol/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F446xx -c -I../Core/Inc -I"C:/STM32_Files/Nucleo_AUV_Bare_Metal+RTOS/Protocol/sd_card" -I"C:/STM32_Files/Nucleo_AUV_Bare_Metal+RTOS/Middlewares/FreeRTOS/Source/include" -I"C:/STM32_Files/Nucleo_AUV_Bare_Metal+RTOS/Middlewares/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I../Drivers/dac -I../Drivers/adc -I../Devices/sd_card -I../Devices/oled -I../Drivers/uart -I../Drivers/timer -I../Drivers/spi -I../Protocol -I../Drivers -I../Drivers/gpio -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/crc_hw -I../Drivers/i2c -I../Devices/bar30 -I../Drivers/iwdg -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Protocol

clean-Protocol:
	-$(RM) ./Protocol/packet.cyclo ./Protocol/packet.d ./Protocol/packet.o ./Protocol/packet.su ./Protocol/ring_buffer.cyclo ./Protocol/ring_buffer.d ./Protocol/ring_buffer.o ./Protocol/ring_buffer.su ./Protocol/struct.cyclo ./Protocol/struct.d ./Protocol/struct.o ./Protocol/struct.su

.PHONY: clean-Protocol

