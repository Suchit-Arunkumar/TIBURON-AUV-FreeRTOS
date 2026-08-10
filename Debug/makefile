################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Protocol/sd_card/subdir.mk
-include Protocol/subdir.mk
-include Middlewares/FreeRTOS/Source/portable/MemMang/subdir.mk
-include Middlewares/FreeRTOS/Source/portable/GCC/ARM_CM4F/subdir.mk
-include Middlewares/FreeRTOS/Source/subdir.mk
-include Drivers/uart/subdir.mk
-include Drivers/timer/subdir.mk
-include Drivers/spi/subdir.mk
-include Drivers/iwdg/subdir.mk
-include Drivers/i2c/subdir.mk
-include Drivers/gpio/subdir.mk
-include Drivers/dac/subdir.mk
-include Drivers/crc_hw/subdir.mk
-include Drivers/adc/subdir.mk
-include Devices/oled/subdir.mk
-include Devices/bar30/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := Nucleo_AUV_Bare_Metal+RTOS
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Nucleo_AUV_Bare_Metal+RTOS.elf \

MAP_FILES += \
Nucleo_AUV_Bare_Metal+RTOS.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Nucleo_AUV_Bare_Metal+RTOS.list \


# All Target
all: main-build

# Main-build Target
main-build: Nucleo_AUV_Bare_Metal+RTOS.elf secondary-outputs

# Tool invocations
Nucleo_AUV_Bare_Metal+RTOS.elf Nucleo_AUV_Bare_Metal+RTOS.map: $(OBJS) $(USER_OBJS) C:\STM32_Files\Nucleo_AUV_Bare_Metal+RTOS\STM32F446RETX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "Nucleo_AUV_Bare_Metal+RTOS.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\STM32_Files\Nucleo_AUV_Bare_Metal+RTOS\STM32F446RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Nucleo_AUV_Bare_Metal+RTOS.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Nucleo_AUV_Bare_Metal+RTOS.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Nucleo_AUV_Bare_Metal+RTOS.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) Nucleo_AUV_Bare_Metal+RTOS.elf Nucleo_AUV_Bare_Metal+RTOS.list Nucleo_AUV_Bare_Metal+RTOS.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
