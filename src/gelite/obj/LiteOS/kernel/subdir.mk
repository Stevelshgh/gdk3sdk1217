################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../LiteOS/kernel/los_event.c \
../LiteOS/kernel/los_init.c \
../LiteOS/kernel/los_membox.c \
../LiteOS/kernel/los_memory.c \
../LiteOS/kernel/los_mux.c \
../LiteOS/kernel/los_queue.c \
../LiteOS/kernel/los_sched.c \
../LiteOS/kernel/los_sem.c \
../LiteOS/kernel/los_sortlink.c \
../LiteOS/kernel/los_swtmr.c \
../LiteOS/kernel/los_task.c \
../LiteOS/kernel/los_tick.c 

OBJS += \
./LiteOS/kernel/los_event.o \
./LiteOS/kernel/los_init.o \
./LiteOS/kernel/los_membox.o \
./LiteOS/kernel/los_memory.o \
./LiteOS/kernel/los_mux.o \
./LiteOS/kernel/los_queue.o \
./LiteOS/kernel/los_sched.o \
./LiteOS/kernel/los_sem.o \
./LiteOS/kernel/los_sortlink.o \
./LiteOS/kernel/los_swtmr.o \
./LiteOS/kernel/los_task.o \
./LiteOS/kernel/los_tick.o 

C_DEPS += \
./LiteOS/kernel/los_event.d \
./LiteOS/kernel/los_init.d \
./LiteOS/kernel/los_membox.d \
./LiteOS/kernel/los_memory.d \
./LiteOS/kernel/los_mux.d \
./LiteOS/kernel/los_queue.d \
./LiteOS/kernel/los_sched.d \
./LiteOS/kernel/los_sem.d \
./LiteOS/kernel/los_sortlink.d \
./LiteOS/kernel/los_swtmr.d \
./LiteOS/kernel/los_task.d \
./LiteOS/kernel/los_tick.d 


# Each subdirectory must supply rules for building sources it contributes
LiteOS/kernel/%.o: ../LiteOS/kernel/%.c
	@	@	riscv-none-embed-gcc -march=rv32imc -mabi=ilp32 -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -I"../incs" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

