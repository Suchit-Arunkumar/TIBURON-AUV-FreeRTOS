################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/timer/timer_basic.c \
../Drivers/timer/timer_pwm.c \
../Drivers/timer/timer_timebase.c 

OBJS += \
./Drivers/timer/timer_basic.o \
./Drivers/timer/timer_pwm.o \
./Drivers/timer/timer_timebase.o 

C_DEPS += \
./Drivers/timer/timer_basic.d \
./Drivers/timer/timer_pwm.d \
./Drivers/timer/timer_timebase.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/timer/%.o Drivers/timer/%.su Drivers/timer/%.cyclo: ../Drivers/timer/%.c Drivers/timer/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F446xx -c -I../Core/Inc -I"C:/STM32_Files/Nucleo_AUV_Bare_Metal+RTOS/Protocol/sd_card" -I"C:/STM32_Files/Nucleo_AUV_Bare_Metal+RTOS/Middlewares/FreeRTOS/Source/include" -I"C:/STM32_Files/Nucleo_AUV_Bare_Metal+RTOS/Middlewares/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I../Drivers/dac -I../Drivers/adc -I../Devices/sd_card -I../Devices/oled -I../Drivers/uart -I../Drivers/timer -I../Drivers/spi -I../Protocol -I../Drivers -I../Drivers/gpio -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/crc_hw -I../Drivers/i2c -I../Devices/bar30 -I../Drivers/iwdg -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-timer

clean-Drivers-2f-timer:
	-$(RM) ./Drivers/timer/timer_basic.cyclo ./Drivers/timer/timer_basic.d ./Drivers/timer/timer_basic.o ./Drivers/timer/timer_basic.su ./Drivers/timer/timer_pwm.cyclo ./Drivers/timer/timer_pwm.d ./Drivers/timer/timer_pwm.o ./Drivers/timer/timer_pwm.su ./Drivers/timer/timer_timebase.cyclo ./Drivers/timer/timer_timebase.d ./Drivers/timer/timer_timebase.o ./Drivers/timer/timer_timebase.su

.PHONY: clean-Drivers-2f-timer

