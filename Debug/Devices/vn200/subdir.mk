################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Devices/vn200/vn200.c 

OBJS += \
./Devices/vn200/vn200.o 

C_DEPS += \
./Devices/vn200/vn200.d 


# Each subdirectory must supply rules for building sources it contributes
Devices/vn200/%.o Devices/vn200/%.su Devices/vn200/%.cyclo: ../Devices/vn200/%.c Devices/vn200/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F446xx -c -I../Core/Inc -I"C:/STM32_Files/Nucleo_AUV_Bare_Metal+RTOS/Protocol/sd_card" -I"C:/STM32_Files/Nucleo_AUV_Bare_Metal+RTOS/Middlewares/FreeRTOS/Source/include" -I"C:/STM32_Files/Nucleo_AUV_Bare_Metal+RTOS/Middlewares/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I../Drivers/dac -I../Drivers/adc -I../Devices/sd_card -I../Devices/oled -I../Drivers/uart -I../Drivers/timer -I../Drivers/spi -I../Protocol -I../Drivers -I../Drivers/gpio -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/crc_hw -I../Drivers/i2c -I../Devices/bar30 -I../Drivers/iwdg -I"C:/STM32_Files/Nucleo_AUV_Bare_Metal+RTOS/Devices/vn200" -I"C:/STM32_Files/Nucleo_AUV_Bare_Metal+RTOS/Devices/dvl" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Devices-2f-vn200

clean-Devices-2f-vn200:
	-$(RM) ./Devices/vn200/vn200.cyclo ./Devices/vn200/vn200.d ./Devices/vn200/vn200.o ./Devices/vn200/vn200.su

.PHONY: clean-Devices-2f-vn200

