
export FW_VER := 1.1

PROJ = FF_OSD
VER := v$(FW_VER)

SUBDIRS += src

.PHONY: all clean dist flash start serial

ifneq ($(RULES_MK),y)

export ROOT := $(CURDIR)

all:
	$(MAKE) -C src -f $(ROOT)/Rules.mk $(PROJ).elf $(PROJ).bin $(PROJ).hex
clean:
	rm -rf $(PROJ)-$(VER)*
	$(MAKE) -f $(ROOT)/Rules.mk $@

dist: all
	rm -rf $(PROJ)-$(VER)*
	mkdir -p $(PROJ)-$(VER)
	cp -a src/$(PROJ).elf $(PROJ)-$(VER)/$(PROJ)-$(VER).elf
	cp -a src/$(PROJ).bin $(PROJ)-$(VER)/$(PROJ)-$(VER).bin
	cp -a src/$(PROJ).hex $(PROJ)-$(VER)/$(PROJ)-$(VER).hex
	cp -a COPYING $(PROJ)-$(VER)/
	cp -a README.md $(PROJ)-$(VER)/
	zip -r $(PROJ)-$(VER).zip $(PROJ)-$(VER)
	rm -rf $(PROJ)-$(VER)

endif

BAUD=921600

flash: all
	sudo ~/stm32flash/stm32flash -b $(BAUD) \
	-vw src/$(PROJ).hex /dev/ttyUSB0

start:
	sudo ~/stm32flash/stm32flash -b $(BAUD) -g 0 /dev/ttyUSB0

serial:
	sudo miniterm.py /dev/ttyUSB0 115200
