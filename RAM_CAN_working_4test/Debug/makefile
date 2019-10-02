################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include startup/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32F7xx_HAL_Driver/Src/subdir.mk
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
RAM_CAN_working_4test.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
RAM_CAN_working_4test.list \

OBJCOPY_HEX += \
RAM_CAN_working_4test.hex \


# All Target
all: RAM_CAN_working_4test.elf secondary-outputs

# Tool invocations
RAM_CAN_working_4test.elf: $(OBJS) $(USER_OBJS) ../STM32F765VGTx_FLASH.ld
	arm-none-eabi-gcc -o "RAM_CAN_working_4test.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m7 -T"../STM32F765VGTx_FLASH.ld" --specs=nosys.specs -Wl,-Map="RAM_CAN_working_4test.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

RAM_CAN_working_4test.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "RAM_CAN_working_4test.list"
	@echo 'Finished building: $@'
	@echo ' '

RAM_CAN_working_4test.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "RAM_CAN_working_4test.hex"
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
