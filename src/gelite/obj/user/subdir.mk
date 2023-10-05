################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../user/Main.c \
../user/ch32f10x_it.c \
../user/system_ch32f10x.c 

OBJS += \
./user/Main.o \
./user/ch32f10x_it.o \
./user/system_ch32f10x.o 

C_DEPS += \
./user/Main.d \
./user/ch32f10x_it.d \
./user/system_ch32f10x.d 


# Each subdirectory must supply rules for building sources it contributes
user/%.o: ../user/%.c
	@	@	riscv-none-embed-gcc -march=rv32imc -mabi=ilp32 -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -I"../incs" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

