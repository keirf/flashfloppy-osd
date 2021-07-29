
export FW_VER := 1.9

PROJ = FF_OSD
VER := v$(FW_VER)

SUBDIRS += src

.PHONY: all clean dist flash start serial

ifneq ($(RULES_MK),y)

export ROOT := $(CURDIR)

all:
	$(MAKE) -C src -f $(ROOT)/Rules.mk $(PROJ).elf $(PROJ).bin $(PROJ).hex
debug:
	debug=y $(MAKE) -C src -f $(ROOT)/Rules.mk $(PROJ).elf $(PROJ).bin $(PROJ).hex
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
	cp -a RELEASE_NOTES $(PROJ)-$(VER)/
	zip -r $(PROJ)-$(VER).zip $(PROJ)-$(VER)
	rm -rf $(PROJ)-$(VER)

endif

BAUD=921600
DEV=/dev/ttyUSB0

flash: all
	sudo stm32flash -b $(BAUD) \
	-vw src/$(PROJ).hex $(DEV)

start:
	sudo stm32flash -b $(BAUD) -g 0 $(DEV)

serial:
	sudo miniterm.py $(DEV) 115200
