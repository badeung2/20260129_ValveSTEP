# @file
# @brief Application Makefile
# @internal
#
# @copyright (C) 2018-2021 Melexis N.V.
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
# @ingroup application
#
# @details Makefile to build the application
#

#
# DEFAULT RULE
#
help:

# 
# target dependent configurations
#
include Makefile.configure.mk

ifndef ORDER_CODE
$(error 'Error: Variable ORDER_CODE is not defined')
endif

# get the PRODUCT code
PRODUCT ?= $(word 1,$(subst -, ,$(ORDER_CODE)))

include Makefile.srcs.mk

TARGET  = $(ORDER_CODE)_$(TARGET_EXT)

PROJ_DIR = ../..
CODE_DIR = $(PROJ_DIR)/code
PLTF_DIR = $(CODE_DIR)/camcu_platform
BU_LIBS_DIR = $(CODE_DIR)/libraries
HIL_DIR  = $(PROJ_DIR)/verification/hil_tests
LINT_DIR = $(PROJ_DIR)/verification/lint
UNCR_DIR = $(PROJ_DIR)/verification/uncrustify

# do not use any variable in the following define!!!!
PLTF_DEFS_FILE = ../../verification/eclipse/pltf_defs.h
PLTF_DEFS_DIR = $(dir $(PLTF_DEFS_FILE))


# platform common configurations
include $(PLTF_DIR)/config/Config.mk


#
# LIBRARIES
#
-include $(BU_LIBS_DIR)/BuLibraries.mk
INC_DIRS += $(BU_LIBS_INC_DIRS)


#
# HANDLE EXTRA SOURCES SUB-DIRECTORIES
#
SRCS_APP_DIRS = $(sort $(filter-out ./,$(dir $(SRCS_APP))))
OBJSUBDIRS += $(SRCS_APP_DIRS)
INC_DIRS += $(SRCS_APP_DIRS)


#
# CREATE TOTAL SOURCE/OBJ FILES LISTS
#
SRCS  = $(SRCS_APP)
SRCS += $(BU_LIBS_SRCS)
SRCS += $(APP_MODULES_SRCS)

OBJS  = $(patsubst %.S, $(OBJDIR)/%.o, $(filter %.S, $(sort $(SRCS))))
OBJS += $(patsubst %.c, $(OBJDIR)/%.o, $(filter %.c, $(sort $(SRCS))))

# include automatic dependencies
OBJDEPS = $(OBJS:%.o=%.d)
-include $(OBJDEPS)

#CFLAGS += -Werror # promote warnings to errors

#
# RULES
#

.PHONY: help
help: help-header help-info

.PHONY: help-header
help-header:
	@$(ECHO) "Usage: make [target] [OPTION=VALUE] ..."
	@$(ECHO)

.PHONY: help-info
help-info:
	@$(ECHO) $(HELP_LEADING)"Targets:"
	@$(ECHO) $(HELP_LEADING)"  all             Build the application output files."
	@$(ECHO) $(HELP_LEADING)"  clean           Remove the build artifacts."
	@$(ECHO) $(HELP_LEADING)"  clean_drv       Remove ldf file based automatic generated files."
	@$(ECHO) $(HELP_LEADING)"  release         Copy application output files to release folder."
	@$(ECHO) $(HELP_LEADING)"  uncrustify      Correct code style."
	@$(ECHO)
	@$(ECHO) $(HELP_LEADING)"Options:"
	@$(ECHO) $(HELP_LEADING)"  ORDER_CODE      Overwrite MCU order code selection of Makefile.configure.mk"
	@$(ECHO) $(HELP_LEADING)"  PROTECTION_KEY  Overwrite flash readout protection key defined in Makefile.configure.mk"

.PHONY: all
all: app
	$(HIDE_CMD)$(OCP) -R .flash_fill $(TARGET).elf $(OBJDIR)/$(TARGET)_no_fill.elf > /dev/null 2>&1
	@$(ECHO) --- Project : $(TARGET) memory consumption without filling
	$(HIDE_CMD)$(SIZE) -B $(OBJS)
	$(HIDE_CMD)$(SIZE) -B $(OBJDIR)/$(TARGET)_no_fill.elf
	@$(ECHO) --- Project : $(TARGET) successfully created

.PHONY: app
app: $(APP_GEN_SRCS) $(TARGET).elf $(TARGET).hex $(TARGET).map $(PLTF_DEFS_FILE)

.PHONY: release
release: all
	$(HIDE_CMD)$(CP) $(TARGET).elf $(TARGET).hex $(TARGET).map $(PROJ_DIR)/release
ifneq ($(LDF_FILE),)
	$(HIDE_CMD)$(CP) $(LDF_FILE) $(PROJ_DIR)/release
endif

.PHONY: clean
clean:
	-$(HIDE_CMD)$(RM) $(OBJDIR)
	-$(HIDE_CMD)$(RM) $(TARGET).cod $(TARGET).elf.MDS
	-$(HIDE_CMD)$(RM) $(TARGET).map $(TARGET).elf
	-$(HIDE_CMD)$(RM) $(TARGET).gdb.elf
	-$(HIDE_CMD)$(RM) $(TARGET).hex $(TARGET).lss
	-$(HIDE_CMD)$(RM) $(TARGET).sym
	-$(HIDE_CMD)$(RM) $(TARGET).syms
	-$(HIDE_CMD)$(RM) $(TARGET)*_lint.txt
	-$(HIDE_CMD)$(MKDIR) $(OBJDIR)/camculib
	-$(HIDE_CMD)$(MKDIR) $(OBJDIR)/commlib
	-$(HIDE_CMD)$(MKDIR) $(OBJDIR)/drivelib
	-$(HIDE_CMD)$(MKDIR) $(OBJDIR)/senselib
	@$(ECHO) --- Cleaning project $(TARGET) has been finished


# static code analysis
.PHONY: lint
lint:
# get correct gcc include directories
	@$(ECHO) $(subst /cygdrive/c,C:,$(foreach v,$(shell echo | mlx16-cpp -Wp,-v 2>&1),$(if $(findstring "/,$(v)), ,$(if $(findstring /,$(v)),-i\"$(v)\",)))) > $(LINT_DIR)/gcc-include-path.lnt
# get all include directories
	@$(ECHO) $(subst /cygdrive/c,C:,$(subst -I,,$(foreach v,$(CPPFLAGS),$(if $(findstring /,$(v):0:1),-i\"$(v)\",)))) > $(LINT_DIR)/project_include_path.lnt
	@$(ECHO) | $(CC) -E -dM $(INSTSET) $(CFLAGS) $(CPPFLAGS) - > $(LINT_DIR)/lint_cmac.h
	$(HIDE_CMD)$(LINT) -i$(LINT_DIR) common-options.lnt co-gcc.lnt project-options.lnt temp.lnt $(filter %.c, $(SRCS)) > $(TARGET)_lint.txt
	$(HIDE_CMD)$(LINT) -i$(LINT_DIR) common-options.lnt co-gcc.lnt au-misra2.lnt project-options.lnt temp.lnt $(filter %.c, $(SRCS)) > $(TARGET)_misra_lint.txt

# get all application c and h files to be uncrustified
SRCS_UNCRUSTIFY := $(filter-out $(APP_GEN_SRCS), $(wildcard *.c) $(wildcard *.h))
SRCS_UNCRUSTIFY += $(filter-out $(APP_GEN_SRCS), $(wildcard camculib/*.c) $(wildcard camculib/*.h))
SRCS_UNCRUSTIFY += $(filter-out $(APP_GEN_SRCS), $(wildcard commlib/*.c) $(wildcard commlib/*.h))
SRCS_UNCRUSTIFY += $(filter-out $(APP_GEN_SRCS), $(wildcard drivelib/*.c) $(wildcard drivelib/*.h))
SRCS_UNCRUSTIFY += $(filter-out $(APP_GEN_SRCS), $(wildcard senselib/*.c) $(wildcard senselib/*.h))

# code style formatting
.PHONY: uncrustify
uncrustify:
	$(HIDE_CMD)$(UNCRUSTIFY) -c $(UNCR_DIR)/uncrustify.cfg --replace --no-backup -l C $(SRCS_UNCRUSTIFY)

.PHONY: uncrustify-check
uncrustify-check:
	$(HIDE_CMD)$(UNCRUSTIFY) -c $(UNCR_DIR)/uncrustify.cfg --check -l C $(SRCS_UNCRUSTIFY)


# defines file for eclipse
.PHONY: $(PLTF_DEFS_FILE)
$(PLTF_DEFS_FILE): $(PLTF_DEFS_DIR)
	$(HIDE_CMD)$(CC) -E -dM $(INSTSET) $(CFLAGS) $(CPPFLAGS) - < /dev/null > $@

$(PLTF_DEFS_DIR):
	$(HIDE_CMD)$(MKDIR) $(PLTF_DEFS_DIR)


# platform common rules
include $(PLTF_DIR)/config/Rules.mk
