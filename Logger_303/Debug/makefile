################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include sdcard/subdir.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include SD_HAL_MPU6050-master/subdir.mk
-include Drivers/STM32F3xx_HAL_Driver/Src/subdir.mk
-include BMP280/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Logger_303.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Logger_303.list \

OBJCOPY_HEX += \
Logger_303.hex \

OBJCOPY_BIN += \
Logger_303.bin \


# All Target
all: Logger_303.elf secondary-outputs

# Tool invocations
Logger_303.elf: $(OBJS) $(USER_OBJS) D:\Data\STM32CubeIDE\workspace_102\Logger_303\STM32F303K6TX_FLASH.ld
	arm-none-eabi-gcc -o "Logger_303.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"D:\Data\STM32CubeIDE\workspace_102\Logger_303\STM32F303K6TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Logger_303.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Logger_303.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Logger_303.list"
	@echo 'Finished building: $@'
	@echo ' '

Logger_303.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "Logger_303.hex"
	@echo 'Finished building: $@'
	@echo ' '

Logger_303.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Logger_303.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
