################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../startup/startup_ch32f10x.s 

OBJS += \
./startup/startup_ch32f10x.o 

S_DEPS += \
./startup/startup_ch32f10x.d 


# Each subdirectory must supply rules for building sources it contributes
startup/%.o: ../startup/%.s
	@	@	riscv-none-embed-gcc -march=rv32imc -mabi=ilp32 -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -x assembler -I"../startup" -I"../LiteOS/arch" -I"../LiteOS/arch" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

