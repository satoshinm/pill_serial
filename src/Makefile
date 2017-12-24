PROBE_HOST ?= stlink
ENABLE_DEBUG ?=

ifneq ($(V), 1)
MAKEFLAGS += --no-print-dir
Q := @
endif

OPT_FLAGS ?= -O2

CFLAGS += -Wall -Wextra -Werror -Wno-char-subscripts\
	$(OPT_FLAGS) -std=gnu99 -g3 -MD \
	-I.
LDFLAGS += $(OPT_FLAGS)

ifeq ($(ENABLE_DEBUG), 1)
CFLAGS += -DENABLE_DEBUG
endif

SRC =			\
	main.c		\
	platform.c	\
	cdcacm.c	\
	usbuart.c	\

CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy

OPT_FLAGS = -Os
CFLAGS += -mcpu=cortex-m3 -mthumb \
	-DSTM32F1 -DDISCOVERY_STLINK -I../libopencm3/include \
	-I .
LDFLAGS_BOOT := $(LDFLAGS) --specs=nano.specs \
	-lopencm3_stm32f1 -Wl,--defsym,_stack=0x20005000 \
	-Wl,-T,stlink.ld -nostartfiles -lc \
	-Wl,-Map=mapfile -mthumb -mcpu=cortex-m3 -Wl,-gc-sections \
	-L../libopencm3/lib
LDFLAGS = $(LDFLAGS_BOOT)

ifeq ($(ENABLE_DEBUG), 1)
LDFLAGS += --specs=rdimon.specs
else
LDFLAGS += --specs=nosys.specs
endif

all:	blackmagic.bin

host_clean:
	-$(Q)$(RM) blackmagic.bin

OBJ = $(SRC:.c=.o)

blackmagic.elf: version.h $(OBJ)
	@echo "  LD      $@"
	$(Q)$(CC) -o $@ $(OBJ) $(LDFLAGS)

%.o:	%.c
	@echo "  CC      $<"
	$(Q)$(CC) $(CFLAGS) -c $< -o $@

%.bin:	%.elf
	@echo "  OBJCOPY $@"
	$(Q)$(OBJCOPY) -O binary $^ $@

%.hex:	%
	@echo "  OBJCOPY $@"
	$(Q)$(OBJCOPY) -O ihex $^ $@

.PHONY:	clean host_clean FORCE

clean:	host_clean
	$(Q)echo "  CLEAN"
	-$(Q)$(RM) -f *.o *.d *~ blackmagic.elf $(HOSTFILES)
	-$(Q)$(RM) -f mapfile version.h

version.h: FORCE
	$(Q)echo "  GIT     version.h"
	$(Q)echo "#define FIRMWARE_VERSION \"`git describe --always --dirty`\"" > $@

-include *.d
