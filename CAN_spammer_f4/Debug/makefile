################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
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
CAN_Spammer_f4.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
CAN_Spammer_f4.list \


# All Target
all: CAN_Spammer_f4.elf secondary-outputs

# Tool invocations
CAN_Spammer_f4.elf: $(OBJS) $(USER_OBJS) /home/luca/Documents/git/fenice-sensors/CAN_spammer_f4/STM32F446RETX_FLASH.ld
	arm-none-eabi-gcc -o "CAN_Spammer_f4.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/luca/Documents/git/fenice-sensors/CAN_spammer_f4/STM32F446RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="CAN_Spammer_f4.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

CAN_Spammer_f4.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "CAN_Spammer_f4.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
