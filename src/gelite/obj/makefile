################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include user/subdir.mk
-include startup/subdir.mk
-include peripheral/subdir.mk
-include debug/subdir.mk
-include core/subdir.mk
-include LiteOS/utils/subdir.mk
-include LiteOS/third_party/subdir.mk
-include LiteOS/kernel/subdir.mk
-include LiteOS/componets/subdir.mk
-include LiteOS/arch/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
gelite.hex \

SECONDARY_LIST += \
gelite.lst \

SECONDARY_SIZE += \
gelite.siz \


# All Target
all: gelite.elf secondary-outputs

# Tool invocations
gelite.elf: $(OBJS) $(USER_OBJS)
	@	@	riscv-none-embed-gcc -march=rv32imc -mabi=ilp32 -msmall-data-limit=8 -mno-save-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -T .ld -nostartfiles -Xlinker --gc-sections -L"../" -Xlinker --print-memory-usage -Wl,-Map,"gelite.map" --specs=nano.specs --specs=nosys.specs -o "gelite.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@	@
gelite.hex: gelite.elf
	@	riscv-none-embed-objcopy -O ihex "gelite.elf"  "gelite.hex"
	@	@
gelite.lst: gelite.elf
	@	riscv-none-embed-objdump --source --all-headers --demangle --line-numbers --wide "gelite.elf" > "gelite.lst"
	@	@
gelite.siz: gelite.elf
	@	riscv-none-embed-size --format=berkeley "gelite.elf"
	@	@
# Other Targets
clean:
	-$(RM) $(ASM_UPPER_DEPS)$(OBJS)$(SECONDARY_FLASH)$(SECONDARY_LIST)$(SECONDARY_SIZE)$(ASM_DEPS)$(S_DEPS)$(S_UPPER_DEPS)$(C_DEPS) gelite.elf
	-@
secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents

-include ../makefile.targets
