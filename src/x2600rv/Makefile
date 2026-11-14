# Ingenic X2600 RISC-V core build rules

dirs-y += src/x2600rv src/generic

# No LTO: the compile time requests are read from the plain objects
CFLAGS := $(filter-out -flto=auto -fwhole-program -fno-use-linker-plugin,$(CFLAGS))
# rv32imc; zig cc spells it -mcpu=generic_rv32+m+c
X2600RV_ARCH ?= -march=rv32imc -mabi=ilp32
CFLAGS += $(X2600RV_ARCH) -mno-relax -fno-pic \
          -fno-common -isystem src/x2600rv/include
CFLAGS_klipper.elf := $(CFLAGS) -nostdlib -static -T src/x2600rv/x2600rv.ld \
          -Wl,--gc-sections

src-y += x2600rv/main.c x2600rv/timer.c x2600rv/ring.c x2600rv/util.c
src-y += generic/crc16_ccitt.c generic/timer_irq.c

OBJS_klipper.elf += $(OUT)src/x2600rv/start.o $(OUT)src/x2600rv/setjmp.o

$(OUT)src/x2600rv/%.o: src/x2600rv/%.S $(OUT)autoconf.h
	@echo "  Assembling $@"
	$(Q)$(CC) $(CFLAGS) -c $< -o $@

target-y += $(OUT)x2600rv.bin

$(OUT)x2600rv.bin: $(OUT)klipper.elf
	@echo "  Creating $@"
	$(Q)$(OBJCOPY) -O binary $< $@
