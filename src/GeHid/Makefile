include ../makefile.conf
NAME=gehid
STARTUP_DEFS=

LDSCRIPTS=-L. -L$(BASE)/ldscripts -T gdk3.ld
LFLAGS=$(USE_NANO) $(USE_NOHOST) $(LDSCRIPTS) $(GC) $(MAP)
INC=-I$(BASE)/inc -I"USBLIB/USB-Driver/inc" -I"USBLIB/CONFIG"
SRC=$(NAME).c $(BASE)/debug/debug.c  $(BASE)/driver/ch32f10x_misc.c \
	$(BASE)/driver/ch32f10x_rcc.c $(BASE)/driver/ch32f10x_gpio.c \
	$(BASE)/driver/ch32f10x_dma.c $(BASE)/driver/ch32f10x_tim.c \
	$(BASE)/driver/ch32f10x_usb.c $(BASE)/driver/ch32f10x_usart.c \
	$(BASE)/src/gehid/ch32f10x_it.c $(BASE)/src/gehid/system_ch32f10x.c\
	$(BASE)/src/gehid/USB_Device/ch32f10x_usbfs_device.c $(BASE)/src/gehid/USB_Device/usbhd_desc.c \
	$(BASE)/src/gehid/UART/UART.c \
	\
	USBLIB/CONFIG/usb_prop.c USBLIB/USB-Driver/src/usb_core.c USBLIB/CONFIG/usb_istr.c USBLIB/CONFIG/usb_desc.c \
	USBLIB/USB-Driver/src/*.c USBLIB/CONFIG/usb_endp.c USBLIB/CONFIG/hw_config.c \
	USBLIB/CONFIG/usb_pwr.c usbd_compostie_km.c USBLIB/USB-Driver/src/usb_init.c

all: $(NAME).axf $(NAME).elf $(NAME).hex $(NAME).siz $(NAME).lst

$(NAME).axf: $(SRC) $(STARTUP)
	$(CC) $(INC) $^ $(CFLAGS) $(LFLAGS) -o $@

$(NAME).elf: $(SRC) $(STARTUP)
	$(CC) -g3 -I$(INC) $^ $(CFLAGS) $(LFLAGS) -o $@

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
