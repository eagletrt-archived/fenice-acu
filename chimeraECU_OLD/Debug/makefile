################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include startup/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
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
EngineControlUnitChimeraNoControl_2.0.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
EngineControlUnitChimeraNoControl_2.0.list \

OBJCOPY_HEX += \
EngineControlUnitChimeraNoControl_2.0.hex \


# All Target
all: EngineControlUnitChimeraNoControl_2.0.elf secondary-outputs

# Tool invocations
EngineControlUnitChimeraNoControl_2.0.elf: $(OBJS) $(USER_OBJS) ../STM32F446RETx_FLASH.ld
	arm-none-eabi-gcc -o "EngineControlUnitChimeraNoControl_2.0.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"../STM32F446RETx_FLASH.ld" --specs=nosys.specs -Wl,-Map="EngineControlUnitChimeraNoControl_2.0.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

EngineControlUnitChimeraNoControl_2.0.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "EngineControlUnitChimeraNoControl_2.0.list"
	@echo 'Finished building: $@'
	@echo ' '

EngineControlUnitChimeraNoControl_2.0.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "EngineControlUnitChimeraNoControl_2.0.hex"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
