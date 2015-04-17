################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/avg_buffer.c \
../src/cr_startup_lpc82x.c \
../src/crp.c \
../src/fir.c \
../src/init.c \
../src/mtb.c \
../src/pid.c \
../src/power_avg.c \
../src/sysinit.c \
../src/test_1.c 

S_SRCS += \
../src/aeabi_romdiv_patch.s 

OBJS += \
./src/aeabi_romdiv_patch.o \
./src/avg_buffer.o \
./src/cr_startup_lpc82x.o \
./src/crp.o \
./src/fir.o \
./src/init.o \
./src/mtb.o \
./src/pid.o \
./src/power_avg.o \
./src/sysinit.o \
./src/test_1.o 

C_DEPS += \
./src/avg_buffer.d \
./src/cr_startup_lpc82x.d \
./src/crp.d \
./src/fir.d \
./src/init.d \
./src/mtb.d \
./src/pid.d \
./src/power_avg.d \
./src/sysinit.d \
./src/test_1.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.s
	@echo 'Building file: $<'
	@echo 'Invoking: MCU Assembler'
	arm-none-eabi-gcc -c -x assembler-with-cpp -DDEBUG -D__CODE_RED -DCORE_M0PLUS -D__USE_ROMDIVIDE -D__USE_LPCOPEN -D__LPC82X__ -D__REDLIB__ -I"/Users/lszuba/dev/LPCXpresso/workspace/lpc_board_nxp_lpcxpresso_824/inc" -I"/Users/lszuba/dev/LPCXpresso/workspace/lpc_chip_82x/inc" -g3 -mcpu=cortex-m0 -mthumb -specs=redlib.specs -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -DDEBUG -D__CODE_RED -DCORE_M0PLUS -D__MTB_BUFFER_SIZE=256 -D__USE_ROMDIVIDE -D__USE_LPCOPEN -D__LPC82X__ -D__REDLIB__ -I"/Users/lszuba/dev/LPCXpresso/workspace/lpc_board_nxp_lpcxpresso_824/inc" -I"/Users/lszuba/dev/LPCXpresso/workspace/lpc_chip_82x/inc" -O0 -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m0 -mthumb -D__REDLIB__ -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


