# @file
# @brief Global rules for the build process
# @internal
#
# @copyright (C) 2017-2020 Melexis N.V.
#
# Melexis N.V. is supplying this code for use with Melexis N.V. processor based microcontrollers only.
#
# THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY,
# INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.  MELEXIS N.V. SHALL NOT IN ANY CIRCUMSTANCES,
# BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
#
# @endinternal
#
# @ingroup platform
#
# @details Global rules for the build process on the platform level.
#

# default rules
$(OBJDIR)/%.o:%.c | $(OBJDIR) $(OBJSUBDIRS_TOCREATE) CC-version
	@$(ECHO) --- Processing: $(call RELPATH,$<)
	$(HIDE_CMD)$(CC) -c -MMD $(INSTSET) $(CFLAGS) $(CPPFLAGS) $< -o $@

$(OBJDIR)/%.o:%.S | $(OBJDIR) $(OBJSUBDIRS_TOCREATE) CC-version
	@$(ECHO) --- Processing: $(call RELPATH,$<)
	$(HIDE_CMD)$(CC) -c -MMD $(INSTSET) $(ASFLAGS) $(CPPFLAGS) $< -o $@

$(OBJDIR)/%.ld: $(LDDIR)/%.ld | $(OBJDIR) CC-version
	$(HIDE_CMD)$(CC) -E -x c -P -C $(INSTSET) $(CFLAGS) $(CPPFLAGS) $< > $@

$(OBJDIR):
	-$(HIDE_CMD)$(MKDIR) $(OBJDIR)

$(OBJSUBDIRS_TOCREATE):
	-$(HIDE_CMD) $(MKDIR) $(OBJSUBDIRS_TOCREATE)

# flash bist crc calculation
flash_crc = $(shell $(PYMLXHEX) calculate --start=$(FLASH_BIST_START) --length=$(FLASH_BIST_LEN) rom_bist $(OBJDIR)/$(TARGET)_nocrc.hex)

.PHONY: checkgaps
checkgaps: $(OBJDIR)/$(TARGET)_nocrc.hex
	$(HIDE_CMD)$(PYMLXHEX) checkgap --start=$(FLASH_BIST_START) --length=$(FLASH_BIST_LEN) $(OBJDIR)/$(TARGET)_nocrc.hex

# create hex file to be able to calculate all interrupt vector crc's
$(OBJDIR)/$(TARGET)_nocrc.elf: $(OBJDIR)/$(CHIP_PATH_NAME).ld $(OBJS) | CC-version
	@$(ECHO) "Link application with dummy CRC"
	$(HIDE_CMD)$(CC) $(OBJS) -o $@ $(INSTSET) $(LDFLAGS) $(LDFLAGS_APP) $(addprefix -T,$(LINKER_SCRIPTS)) -Wl,--defsym,FL_SIGN=0x0000

# create final elf file to program to the chip
ifneq ($(BUILD_RAM_APP),1)
$(TARGET).elf: checkgaps
endif
$(TARGET).elf: $(OBJDIR)/$(CHIP_PATH_NAME).ld $(OBJDIR)/$(TARGET)_nocrc.hex $(OBJS) | PYMLXHEX-version
	@$(ECHO) "Linking ->" $(call RELPATH,$@)
	$(HIDE_CMD)$(CC) $(OBJS) -o $@ $(INSTSET) $(LDFLAGS) $(LDFLAGS_APP) $(addprefix -T,$(LINKER_SCRIPTS)) -Wl,--defsym,FL_SIGN=$(flash_crc)

# create final hex file to program to the chip
$(TARGET).hex: $(TARGET).elf
	@$(ECHO) --- Generating: $@
	$(HIDE_CMD)$(OCP) -O ihex --remove-section=".ep.data" $< $@

# create final hex file to program to the chip
$(TARGET)_eeprom.hex: $(TARGET).elf
	@$(ECHO) --- Generating: $@
	$(HIDE_CMD)$(OCP) -O ihex --only-section=".ep.data" $< $@

# target to create elf file with removed leading underscores from symbols needed for debugging in gdb
%.gdb.elf: %.syms %.elf
	@$(ECHO) --- Generating: $@
	$(HIDE_CMD)$(OCP) --redefine-syms=$^ $@

%.syms: %.elf
	$(eval NM_OUT:= $$(shell $(NM) -f posix --defined-only $$<))
	$(eval NM_SORT:= $(sort $(NM_OUT)))
	$(HIDE_CMD)DIRTY_SYMBOLS="$(filter _%,$(NM_SORT))"; \
	$(RM) $@;\
	for symbol in $${DIRTY_SYMBOLS} ; \
	do \
		echo $${symbol}" "$${symbol#_} >> $@; \
	done

# target to create a gdb debug file
%.gdb: %.elf
	$(HIDE_CMD)$(READELF) $< --debug-dump >> $@

# create hex / bin files from ELF output file
%.hex: %.elf
	@$(ECHO) --- Generating: $(call RELPATH,$@)
	$(HIDE_CMD)$(OCP) -O ihex $< $@

%.bin: %.elf
	@$(ECHO) --- Generating: $(call RELPATH,$@)
	$(HIDE_CMD)$(OCP) -O binary $< $@

# create extended listing file from ELF output file
%.lss: %.elf
	@$(ECHO) --- Generating: $(call RELPATH,$@)
	$(HIDE_CMD)$(CC) --version > $@
	$(HIDE_CMD)$(ODP) -h -S -z $< >> $@

# create a symbol table from ELF output file
%.sym: %.elf
	@$(ECHO) --- Generating: $(call RELPATH,$@)
	$(HIDE_CMD)$(NM) -n $< > $@

# create assembler files from C source files.
%.s: %.c
	@$(ECHO) --- Generating: $(call RELPATH,$@)
	$(HIDE_CMD)$(CC) -S $(INSTSET) $(CFLAGS) $(CPPFLAGS) $< -o $@

# create preprocessed source
%.i: %.c
	@$(ECHO) --- Generating: $(call RELPATH,$@)
	$(HIDE_CMD)$(CC) -E $(INSTSET) $(CFLAGS) $(CPPFLAGS) $< -o $@

# tool version checking
CPAREN =)
PYMLXHEX_INSTALLED_VERSION = $(word 1, $(sort $(PYMLXHEX_VERSION) $(word 2, $(shell $(PYMLXHEX) --version))))
CC_INSTALLED_VERSION = $(word 1, $(sort $(CC_VERSION) $(subst $(CPAREN),,$(word 4, $(shell $(CC) --version)))))
LIN_NODEGEN_INSTALLED_VERSION = $(word 1, $(sort $(LIN_NODEGEN_VERSION) $(word 3, $(shell $(LIN_NODEGEN) --version))))
MELIBU_NODEGEN_INSTALLED_VERSION = $(word 1, $(sort $(MELIBU_NODEGEN_VERSION) $(word 3, $(shell $(MELIBU_NODEGEN) --version))))

.PHONY: %-version
%-version:
	$(HIDE_CMD)if [ "$($*)" = "" ]; then \
		echo "Variable $* is not defined"; \
		exit 1; \
	fi
	$(HIDE_CMD)if [ "$($*_VERSION)" = "" ]; then \
		echo "Variable $*_VERSION is not defined"; \
		exit 1; \
	fi
	$(HIDE_CMD)if [ "$($*_INSTALLED_VERSION)" != "$($*_VERSION)" ]; then \
		echo "Bad version of $* $($*_INSTALLED_VERSION), please install $($*_VERSION) or newer"; \
		exit 1; \
	fi;
