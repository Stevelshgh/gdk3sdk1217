################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../LiteOS/componets/los_backtrace.c \
../LiteOS/componets/los_cpup.c \
../LiteOS/componets/los_exc_info.c \
../LiteOS/componets/los_exchook.c \
../LiteOS/componets/los_pm.c 

OBJS += \
./LiteOS/componets/los_backtrace.o \
./LiteOS/componets/los_cpup.o \
./LiteOS/componets/los_exc_info.o \
./LiteOS/componets/los_exchook.o \
./LiteOS/componets/los_pm.o 

C_DEPS += \
./LiteOS/componets/los_backtrace.d \
./LiteOS/componets/los_cpup.d \
./LiteOS/componets/los_exc_info.d \
./LiteOS/componets/los_exchook.d \
./LiteOS/componets/los_pm.d 


# Each subdirectory must supply rules for building sources it contributes
LiteOS/componets/%.o: ../LiteOS/componets/%.c
	@	@	riscv-none-embed-gcc -march=rv32imc -mabi=ilp32 -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -I"../incs" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

