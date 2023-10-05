################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../debug/debug.c 

OBJS += \
./debug/debug.o 

C_DEPS += \
./debug/debug.d 


# Each subdirectory must supply rules for building sources it contributes
debug/%.o: ../debug/%.c
	@	@	riscv-none-embed-gcc -march=rv32imc -mabi=ilp32 -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -I"../incs" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

