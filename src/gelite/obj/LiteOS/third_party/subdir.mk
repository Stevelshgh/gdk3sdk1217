################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../LiteOS/third_party/memcpy_s.c \
../LiteOS/third_party/memmove_s.c \
../LiteOS/third_party/memset_s.c \
../LiteOS/third_party/strcpy_s.c \
../LiteOS/third_party/strncpy_s.c 

OBJS += \
./LiteOS/third_party/memcpy_s.o \
./LiteOS/third_party/memmove_s.o \
./LiteOS/third_party/memset_s.o \
./LiteOS/third_party/strcpy_s.o \
./LiteOS/third_party/strncpy_s.o 

C_DEPS += \
./LiteOS/third_party/memcpy_s.d \
./LiteOS/third_party/memmove_s.d \
./LiteOS/third_party/memset_s.d \
./LiteOS/third_party/strcpy_s.d \
./LiteOS/third_party/strncpy_s.d 


# Each subdirectory must supply rules for building sources it contributes
LiteOS/third_party/%.o: ../LiteOS/third_party/%.c
	@	@	riscv-none-embed-gcc -march=rv32imc -mabi=ilp32 -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -I"../incs" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

