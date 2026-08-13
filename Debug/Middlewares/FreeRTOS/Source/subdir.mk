################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/FreeRTOS/Source/event_groups.c \
../Middlewares/FreeRTOS/Source/list.c \
../Middlewares/FreeRTOS/Source/queue.c \
../Middlewares/FreeRTOS/Source/tasks.c \
../Middlewares/FreeRTOS/Source/timers.c 

OBJS += \
./Middlewares/FreeRTOS/Source/event_groups.o \
./Middlewares/FreeRTOS/Source/list.o \
./Middlewares/FreeRTOS/Source/queue.o \
./Middlewares/FreeRTOS/Source/tasks.o \
./Middlewares/FreeRTOS/Source/timers.o 

C_DEPS += \
./Middlewares/FreeRTOS/Source/event_groups.d \
./Middlewares/FreeRTOS/Source/list.d \
./Middlewares/FreeRTOS/Source/queue.d \
./Middlewares/FreeRTOS/Source/tasks.d \
./Middlewares/FreeRTOS/Source/timers.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/FreeRTOS/Source/%.o Middlewares/FreeRTOS/Source/%.su Middlewares/FreeRTOS/Source/%.cyclo: ../Middlewares/FreeRTOS/Source/%.c Middlewares/FreeRTOS/Source/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F446xx -c -I../Core/Inc -I"C:/STM32_Files/Nucleo_AUV_Bare_Metal+RTOS/Protocol/sd_card" -I"C:/STM32_Files/Nucleo_AUV_Bare_Metal+RTOS/Middlewares/FreeRTOS/Source/include" -I"C:/STM32_Files/Nucleo_AUV_Bare_Metal+RTOS/Middlewares/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I../Drivers/dac -I../Drivers/adc -I../Devices/sd_card -I../Devices/oled -I../Drivers/uart -I../Drivers/timer -I../Drivers/spi -I../Protocol -I../Drivers -I../Drivers/gpio -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/crc_hw -I../Drivers/i2c -I../Devices/bar30 -I../Drivers/iwdg -I"C:/STM32_Files/Nucleo_AUV_Bare_Metal+RTOS/Devices/vn200" -I"C:/STM32_Files/Nucleo_AUV_Bare_Metal+RTOS/Devices/dvl" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-FreeRTOS-2f-Source

clean-Middlewares-2f-FreeRTOS-2f-Source:
	-$(RM) ./Middlewares/FreeRTOS/Source/event_groups.cyclo ./Middlewares/FreeRTOS/Source/event_groups.d ./Middlewares/FreeRTOS/Source/event_groups.o ./Middlewares/FreeRTOS/Source/event_groups.su ./Middlewares/FreeRTOS/Source/list.cyclo ./Middlewares/FreeRTOS/Source/list.d ./Middlewares/FreeRTOS/Source/list.o ./Middlewares/FreeRTOS/Source/list.su ./Middlewares/FreeRTOS/Source/queue.cyclo ./Middlewares/FreeRTOS/Source/queue.d ./Middlewares/FreeRTOS/Source/queue.o ./Middlewares/FreeRTOS/Source/queue.su ./Middlewares/FreeRTOS/Source/tasks.cyclo ./Middlewares/FreeRTOS/Source/tasks.d ./Middlewares/FreeRTOS/Source/tasks.o ./Middlewares/FreeRTOS/Source/tasks.su ./Middlewares/FreeRTOS/Source/timers.cyclo ./Middlewares/FreeRTOS/Source/timers.d ./Middlewares/FreeRTOS/Source/timers.o ./Middlewares/FreeRTOS/Source/timers.su

.PHONY: clean-Middlewares-2f-FreeRTOS-2f-Source

