include ../makefile.conf
NAME=gedisk
STARTUP_DEFS=

LDSCRIPTS=-L. -L$(BASE)/ldscripts -T gdk3.ld
LFLAGS=$(USE_NANO) $(USE_NOHOST) $(LDSCRIPTS) $(GC) $(MAP)
INC=$(BASE)/inc
SRC=$(NAME).c $(BASE)/debug/debug.c  $(BASE)/driver/ch32f10x_misc.c \
	$(BASE)/driver/ch32f10x_rcc.c $(BASE)/driver/ch32f10x_gpio.c  $(BASE)/driver/ch32f10x_flash.c \
	$(BASE)/driver/ch32f10x_usb.c $(BASE)/driver/ch32f10x_usart.c \
	$(BASE)/src/gedisk/ch32f10x_it.c $(BASE)/src/gedisk/ch32f10x_usbfs_device.c $(BASE)/src/gedisk/Internal_Flash.c \
	$(BASE)/src/gedisk/system_ch32f10x.c $(BASE)/src/gedisk/SW_UDISK.c $(BASE)/src/gedisk/SPI_FLASH.c \
	$(BASE)/src/gedisk/usb_desc.c

all: $(NAME).axf $(NAME).elf $(NAME).hex $(NAME).siz $(NAME).lst

$(NAME).axf: $(SRC) $(STARTUP)
	$(CC) -I$(INC) $^ $(CFLAGS) $(LFLAGS) -o $@

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
