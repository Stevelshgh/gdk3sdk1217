################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../LiteOS/arch/los_context.c \
../LiteOS/arch/los_interrupt.c \
../LiteOS/arch/los_timer.c 

S_UPPER_SRCS += \
../LiteOS/arch/los_dispatch.S \
../LiteOS/arch/los_exc.S 

OBJS += \
./LiteOS/arch/los_context.o \
./LiteOS/arch/los_dispatch.o \
./LiteOS/arch/los_exc.o \
./LiteOS/arch/los_interrupt.o \
./LiteOS/arch/los_timer.o 

S_UPPER_DEPS += \
./LiteOS/arch/los_dispatch.d \
./LiteOS/arch/los_exc.d 

C_DEPS += \
./LiteOS/arch/los_context.d \
./LiteOS/arch/los_interrupt.d \
./LiteOS/arch/los_timer.d 


# Each subdirectory must supply rules for building sources it contributes
LiteOS/arch/%.o: ../LiteOS/arch/%.c
	@	@	riscv-none-embed-gcc -march=rv32imc -mabi=ilp32 -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -I"../incs" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@
LiteOS/arch/%.o: ../LiteOS/arch/%.S
	@	@	riscv-none-embed-gcc -march=rv32imc -mabi=ilp32 -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -x assembler -I"../startup" -I"../LiteOS/arch" -I"../LiteOS/arch" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

