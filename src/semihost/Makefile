include ../makefile.conf
NAME=gem3
STARTUP_DEFS=

LDSCRIPTS=-L. -L$(BASE)/ldscripts -T gdk3.ld
LFLAGS=$(USE_NANO) $(USE_SEMIHOST) $(LDSCRIPTS) $(GC) $(MAP)

all: $(NAME).axf $(NAME).elf $(NAME).hex $(NAME).siz

$(NAME).axf: semihost.c $(STARTUP)
	$(CC) $^ $(CFLAGS) $(LFLAGS) -o $@

$(NAME).elf: semihost.c $(STARTUP)
	$(CC) $^ $(CFLAGS) $(LFLAGS) -o $@

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
