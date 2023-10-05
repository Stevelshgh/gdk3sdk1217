include ../makefile.conf
NAME=gekm
STARTUP_DEFS=

LDSCRIPTS=-L. -L$(BASE)/ldscripts -T gdk3.ld
LFLAGS=$(USE_NANO) $(USE_SEMIHOST) $(LDSCRIPTS) $(GC) $(MAP)
INC=-I$(BASE)/inc -I"usblib/inc" -I"usblib/config"
SRC=$(NAME).c $(BASE)/debug/debug.c $(BASE)/driver/system_ch32f10x.c $(BASE)/driver/ch32f10x_misc.c \
	$(BASE)/driver/ch32f10x_rcc.c $(BASE)/driver/ch32f10x_gpio.c $(BASE)/driver/ch32f10x_usart.c \
	nan_usbd_desc.c km_sender.c usbhd.c ch32f10x_it.c $(BASE)/driver/ch32f10x_tim.c \
	$(BASE)/driver/ch32f10x_exti.c km_receiver.c usblib/config/hw_config.c usblib/config/bei_usb_desc.c \
	usblib/src/usb_init.c usblib/config/usb_istr.c usblib/config/usb_pwr.c usblib/config/usb_prop.c \
	usblib/config/usb_endp.c usblib/src/usb_core.c usblib/src/usb_mem.c usblib/src/usb_regs.c \
	usblib/src/usb_sil.c usblib/src/usb_int.c

all: $(NAME).axf $(NAME).elf $(NAME).hex $(NAME).siz $(NAME).lst

$(NAME).axf: $(SRC) $(STARTUP)
	$(CC) $(INC) $^ $(CFLAGS) $(LFLAGS) -o $@

$(NAME).elf: $(SRC) $(STARTUP)
	$(CC) -g3 $(INC) $^ $(CFLAGS) $(LFLAGS) -o $@

$(NAME).hex: $(NAME).elf
	@	arm-none-eabi-objcopy -O ihex $^  $@
	@	@
$(NAME).lst: $(NAME).elf
	@	arm-none-eabi-objdump --source --all-headers --demangle --disassemble $^ > $@
	@	@
$(NAME).siz: $(NAME).elf
	@	arm-none-eabi-size --format=berkeley $^
	@	@

clean: 
	rm -f $(NAME)*.axf $(NAME)*.elf $(NAME)*.hex $(NAME)*.siz *.map
