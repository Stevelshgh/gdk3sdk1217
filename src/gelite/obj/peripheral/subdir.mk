################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../peripheral/ch32f10x_adc.c \
../peripheral/ch32f10x_bkp.c \
../peripheral/ch32f10x_can.c \
../peripheral/ch32f10x_crc.c \
../peripheral/ch32f10x_dac.c \
../peripheral/ch32f10x_dbgmcu.c \
../peripheral/ch32f10x_dma.c \
../peripheral/ch32f10x_exti.c \
../peripheral/ch32f10x_flash.c \
../peripheral/ch32f10x_gpio.c \
../peripheral/ch32f10x_i2c.c \
../peripheral/ch32f10x_iwdg.c \
../peripheral/ch32f10x_misc.c \
../peripheral/ch32f10x_pwr.c \
../peripheral/ch32f10x_rcc.c \
../peripheral/ch32f10x_rtc.c \
../peripheral/ch32f10x_spi.c \
../peripheral/ch32f10x_tim.c \
../peripheral/ch32f10x_usart.c \
../peripheral/ch32f10x_wwdg.c 

OBJS += \
./peripheral/ch32f10x_adc.o \
./peripheral/ch32f10x_bkp.o \
./peripheral/ch32f10x_can.o \
./peripheral/ch32f10x_crc.o \
./peripheral/ch32f10x_dac.o \
./peripheral/ch32f10x_dbgmcu.o \
./peripheral/ch32f10x_dma.o \
./peripheral/ch32f10x_exti.o \
./peripheral/ch32f10x_flash.o \
./peripheral/ch32f10x_gpio.o \
./peripheral/ch32f10x_i2c.o \
./peripheral/ch32f10x_iwdg.o \
./peripheral/ch32f10x_misc.o \
./peripheral/ch32f10x_pwr.o \
./peripheral/ch32f10x_rcc.o \
./peripheral/ch32f10x_rtc.o \
./peripheral/ch32f10x_spi.o \
./peripheral/ch32f10x_tim.o \
./peripheral/ch32f10x_usart.o \
./peripheral/ch32f10x_wwdg.o 

C_DEPS += \
./peripheral/ch32f10x_adc.d \
./peripheral/ch32f10x_bkp.d \
./peripheral/ch32f10x_can.d \
./peripheral/ch32f10x_crc.d \
./peripheral/ch32f10x_dac.d \
./peripheral/ch32f10x_dbgmcu.d \
./peripheral/ch32f10x_dma.d \
./peripheral/ch32f10x_exti.d \
./peripheral/ch32f10x_flash.d \
./peripheral/ch32f10x_gpio.d \
./peripheral/ch32f10x_i2c.d \
./peripheral/ch32f10x_iwdg.d \
./peripheral/ch32f10x_misc.d \
./peripheral/ch32f10x_pwr.d \
./peripheral/ch32f10x_rcc.d \
./peripheral/ch32f10x_rtc.d \
./peripheral/ch32f10x_spi.d \
./peripheral/ch32f10x_tim.d \
./peripheral/ch32f10x_usart.d \
./peripheral/ch32f10x_wwdg.d 


# Each subdirectory must supply rules for building sources it contributes
peripheral/%.o: ../peripheral/%.c
	@	@	riscv-none-embed-gcc -march=rv32imc -mabi=ilp32 -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -I"../incs" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

