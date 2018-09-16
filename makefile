BINARY		= stm32-helloworld
DEVICE          = stm32f303xc
OPENCM3_DIR     = libopencm3
OBJS            += stm32-helloworld.o debounce-stm32-cm3.o

OOCD		?= openocd
OOCD_INTERFACE	?= stlink-v2
OOCD_TARGET	?= stm32f3x

CFLAGS          += -Os -ggdb3
CPPFLAGS	+= -MD
LDFLAGS         += -static -nostartfiles
LDLIBS          += -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group

include $(OPENCM3_DIR)/mk/genlink-config.mk
include $(OPENCM3_DIR)/mk/gcc-config.mk

.PHONY: clean all

all: $(BINARY).elf $(BINARY).hex

clean:
	$(Q)$(RM) -rf $(BINARY).elf $(BINARY).hex generated.*.ld *.d *.o

flash: all
	@printf "  FLASH   $(BINARY).hex\n"
	(echo "halt; program $(realpath $(BINARY).hex) verify reset" | nc -4 localhost 4444 2>/dev/null) || \
		$(OOCD) -f interface/$(OOCD_INTERFACE).cfg \
		-f target/$(OOCD_TARGET).cfg \
		-c "program $(BINARY).hex verify reset exit" \
		$(NULL)

include $(OPENCM3_DIR)/mk/genlink-rules.mk
include $(OPENCM3_DIR)/mk/gcc-rules.mk
