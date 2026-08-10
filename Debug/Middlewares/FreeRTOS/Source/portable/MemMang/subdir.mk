################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/FreeRTOS/Source/portable/MemMang/heap_4.c 

OBJS += \
./Middlewares/FreeRTOS/Source/portable/MemMang/heap_4.o 

C_DEPS += \
./Middlewares/FreeRTOS/Source/portable/MemMang/heap_4.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/FreeRTOS/Source/portable/MemMang/%.o Middlewares/FreeRTOS/Source/portable/MemMang/%.su Middlewares/FreeRTOS/Source/portable/MemMang/%.cyclo: ../Middlewares/FreeRTOS/Source/portable/MemMang/%.c Middlewares/FreeRTOS/Source/portable/MemMang/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F446xx -c -I../Core/Inc -I"C:/STM32_Files/Nucleo_AUV_Bare_Metal+RTOS/Protocol/sd_card" -I"C:/STM32_Files/Nucleo_AUV_Bare_Metal+RTOS/Middlewares/FreeRTOS/Source/include" -I"C:/STM32_Files/Nucleo_AUV_Bare_Metal+RTOS/Middlewares/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I../Drivers/dac -I../Drivers/adc -I../Devices/sd_card -I../Devices/oled -I../Drivers/uart -I../Drivers/timer -I../Drivers/spi -I../Protocol -I../Drivers -I../Drivers/gpio -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/crc_hw -I../Drivers/i2c -I../Devices/bar30 -I../Drivers/iwdg -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-FreeRTOS-2f-Source-2f-portable-2f-MemMang

clean-Middlewares-2f-FreeRTOS-2f-Source-2f-portable-2f-MemMang:
	-$(RM) ./Middlewares/FreeRTOS/Source/portable/MemMang/heap_4.cyclo ./Middlewares/FreeRTOS/Source/portable/MemMang/heap_4.d ./Middlewares/FreeRTOS/Source/portable/MemMang/heap_4.o ./Middlewares/FreeRTOS/Source/portable/MemMang/heap_4.su

.PHONY: clean-Middlewares-2f-FreeRTOS-2f-Source-2f-portable-2f-MemMang

