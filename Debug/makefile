################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: test_1.axf

# Tool invocations
test_1.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"/Users/lszuba/dev/LPCXpresso/workspace/lpc_board_nxp_lpcxpresso_824/Debug" -L"/Users/lszuba/dev/LPCXpresso/workspace/lpc_chip_82x/Debug" -Xlinker -Map="test_1.map" -Xlinker --gc-sections -mcpu=cortex-m0 -mthumb -T "test_1_Debug.ld" -o "test_1.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(EXECUTABLES) test_1.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "test_1.axf"; # arm-none-eabi-objcopy -v -O binary "test_1.axf" "test_1.bin" ; # checksum -p LPC824 -d "test_1.bin";
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
