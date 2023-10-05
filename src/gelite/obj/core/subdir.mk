################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../core/core_riscv.c 

OBJS += \
./core/core_riscv.o 

C_DEPS += \
./core/core_riscv.d 


# Each subdirectory must supply rules for building sources it contributes
core/%.o: ../core/%.c
	@	@	riscv-none-embed-gcc -march=rv32imc -mabi=ilp32 -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -I"../incs" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

