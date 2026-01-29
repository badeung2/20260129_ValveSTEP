# @file
# @brief Global configurations for the build process
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
# @details Global configurations for the build process on the platform level.
#
# Dependencies:
# - PROJ_DIR
#

ifndef PRODUCT
$(error 'Error: Variable PRODUCT is not defined')
endif

# Tool versioning
PYMLXHEX_VERSION = v2.0.0
CC_VERSION = v3.1.112
LIN_NODEGEN_VERSION = 2.1
MELIBU_NODEGEN_VERSION = 0.3.3

# Identify the root directory of the SW Platform (escape spaces in the path)
empty :=
space := $(empty) $(empty)
ROOTDIR := $(subst $(space),\$(space),$(dir $(filter %/Config.mk, $(MAKEFILE_LIST)))..)

#--- Helper make functions ----------------------------------------------------
# Relative path of arg1 vs arg2 (usage example below)
RELPATH_FUNC = $(subst $(2),.,$(1))
# Relative path of arg1 vs PROJ_DIR
RELPATH = $(call RELPATH_FUNC,$(abspath $(1)),$(abspath $(PROJ_DIR)))

#--- Include product specific variables ---------------------------------------
include $(ROOTDIR)/config/$(PRODUCT).mk

#--- Assemble the full chip name ----------------------------------------------
CHIP_FULL_NAME = $(PRODUCT)$(CHIP_REVISION)
CHIP_PATH_NAME = $(PRODUCT)$(PROJECT_PATH_EXT)


#--- RAM program support ------------------------------------------------------
BUILD_RAM_APP ?= 0
ifeq ($(BUILD_RAM_APP),1)
  PLTF_DEFS += RAM_APPLICATION
endif

#--- Loader configuration -----------------------------------------------------

ifeq ($(filter UDS_LOADER,$(ROM_BASED_LOADER)),UDS_LOADER)
ifeq ($(UDS_LOADER),1)
  LD_FILE_EXT ?= -uds-app
  LINKER_SCRIPTS += $(CHIP_FULL_NAME)-uds-services.ld
else
# uds loader is not enabled for this app
endif
else
# chip does not have UDS LOADER in ROM
endif

#--- Linker scripts -----------------------------------------------------------
ifeq ($(BUILD_RAM_APP),1)
  LD_FILE_EXT ?= -ram
else
  LD_FILE_EXT ?=
endif

LINKER_SCRIPTS += $(CHIP_PATH_NAME)$(LD_FILE_EXT).ld
LINKER_SCRIPTS += $(CHIP_PATH_NAME)-io-map.ld
LINKER_SCRIPTS += $(CHIP_PATH_NAME)-rom-defs.ld

#--- Common directories -------------------------------------------------------
LIBDIR   := $(ROOTDIR)/lib
LDDIR    := $(LIBDIR)/ldscripts
OBJDIR   := obj
APP_MOD_DIR := $(ROOTDIR)/src
OBJSUBDIRS ?=#default no subdirs need creation
OBJSUBDIRS_TOCREATE = $(patsubst %,$(OBJDIR)/%,$(OBJSUBDIRS))

# --- Set default debug info output level ---
ifndef DEBUG
DEBUG = 2
endif

ifeq ($(DEBUG), 3)
HIDE_CMD =
NO_OUTPUT =
else
HIDE_CMD = @
NO_OUTPUT = > /dev/null
endif

#--- Common compilation flags -------------------------------------------------
CPPFLAGS  = -I .

OPTIMIZATION ?= -Os

CFLAGS  = $(OPTIMIZATION)
CFLAGS += -g
CFLAGS += -std=gnu99

# Tweaks
CFLAGS += -mram-align-word
CFLAGS += -ffunction-sections
CFLAGS += -fdata-sections
# Use -mdpage-explicit to have global variables in nodp by default
CFLAGS += -mdpage-explicit

# Warnings
CFLAGS += -Wcast-align
CFLAGS += -Wall # all
CFLAGS += -Wextra # extra basic warnings!
#CFLAGS += -Werror # promote warnings to errors
CFLAGS += -Wstrict-prototypes # warn if function argument types are not defined
CFLAGS += -Wundef # warn of use of undefined macros
# CFLAGS += -Winline # warn when functions requested as inline are not inlined
CFLAGS += -Wno-packed-bitfield-compat # false warning on bitfields

ASFLAGS += $(OPTIMIZATION) -gdwarf-2

# Workaround for ITC bug (see MULAN-5 and MMC16-26 on Jira), aready fixed in HW
#CFLAGS  += -mwith-itc-bug-workaround
#ASFLAGS += -mwith-itc-bug-workaround

#--- Common linker flags ------------------------------------------------------
LDFLAGS := -N
LDFLAGS += -L $(OBJDIR)
LDFLAGS += -L $(LIBDIR)/$(CHIP_PATH_NAME)
LDFLAGS += -L $(LDDIR)
LDFLAGS += -Wl,--start-group
#LDFLAGS += -l$(PRODUCT)
LDFLAGS += -Wl,--whole-archive
LDFLAGS += $(addprefix -l,$(PLTF_LIBS))
LDFLAGS += -Wl,--no-whole-archive
LDFLAGS += -Wl,--end-group
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -Wl,--warn-common
LDFLAGS += -Wl,--discard-none

LDFLAGS_APP += -Wl,-Map,$(TARGET).map

INSTSET := -mlx16-fx

#--- Product specific configuration -------------------------------------------

# Flash configuration
ifeq ($(filter UDS_LOADER,$(ROM_BASED_LOADER)),UDS_LOADER)
ifeq ($(UDS_SERVICES_AT_START),1)
# uds services are stored in beginning of flash
ifeq ($(UDS_LOADER),1)
BIST_START_OFFSET = $(UDS_FLASH_SIZE)
else
BIST_START_OFFSET = 0x0000
endif
BIST_END_OFFSET = 0x0000
else
# uds services are stored in end of flash
BIST_START_OFFSET = 0x0000
ifeq ($(UDS_LOADER),1)
BIST_END_OFFSET = $(UDS_FLASH_SIZE)
else
BIST_END_OFFSET = 0x0000
endif
endif
else
BIST_START_OFFSET = 0x0000
BIST_END_OFFSET = 0x0000
endif

# Physical flash configurations
FLASH_LEN = $(shell echo $$(( $(FLASH_END) - $(FLASH_START) )))
FLASH_SECTOR_SIZE = $(shell echo $$(( $(FLASH_PAGE_SIZE) * $(FLASH_PAGES_IN_SECTOR) )))

# Flash BIST configurations
# ROM always start checking bist from start of flash
BIST_START_OFFSET_IN_SECTORS = $(shell echo $$(( $(BIST_START_OFFSET) / $(FLASH_SECTOR_SIZE) )))
BIST_PAGE_COUNT ?= $(shell echo $$(( (( $(FLASH_LEN) - $(BIST_START_OFFSET) - $(BIST_END_OFFSET) ) / $(FLASH_PAGE_SIZE) ) + $(BIST_START_OFFSET_IN_SECTORS) * 0x1000)))
FLASH_BIST_START = $(shell echo $$(( $(FLASH_START) + $(BIST_START_OFFSET) )))
FLASH_BIST_LEN = $(shell echo $$(( ($(BIST_PAGE_COUNT) & 0x0FFF) * $(FLASH_PAGE_SIZE) )))
FLASH_BIST_END = $(shell echo $$(( $(FLASH_BIST_START) + $(FLASH_BIST_LEN) )))

PLTF_DEFS += FLASH_BIST_START=$(FLASH_BIST_START)
PLTF_DEFS += FLASH_BIST_LEN=$(FLASH_BIST_LEN)
PLTF_DEFS += FLASH_BIST_END=$(FLASH_BIST_END)
PLTF_DEFS += BIST_PAGE_COUNT=$(BIST_PAGE_COUNT)

# FPLL is visible in all source files (CPU clock in kHz)
FPLL := $(FW_FREQ)000

# Extra defines from profile
PLTF_DEFS += PROJECT_ID=$(PROJECT_ID)

PLTF_DEFS += __PLTF_CAMCU__
PLTF_DEFS += __MLX$(PRODUCT)__
PLTF_DEFS += __MLX$(CHIP_FULL_NAME)__

CPPFLAGS += $(foreach folder,$(INC_DIRS),-I$(folder))
CPPFLAGS += $(foreach define,$(PLTF_DEFS),-D$(define))

#--- Product ROM specific configuration ---------------------------------------
# this will set all relevant variables to default if not defined before so this
# is included last. 
include $(ROOTDIR)/config/$(CHIP_PATH_NAME)-cpp-flags.mk
CPPFLAGS += $(PLTF_CPPFLAGS)
CFLAGS += $(PLTF_CFLAGS)

#--- Placeholder for defines from application Makefiles and command-line ------
#  - application Makefile: APP_OPTIONS += -Dxxx
#  - command-line: gmake CMD_OPTIONS="-Dxxx"
CPPFLAGS += $(APP_OPTIONS) $(CMD_OPTIONS)

#--- Toolchain and utilities --------------------------------------------------

# GCC programs
AS   = mlx16-as
AR   = mlx16-ar
CC   = mlx16-gcc
ODP  = mlx16-objdump
OCP  = mlx16-objcopy
NM   = mlx16-nm
SIZE = mlx16-size
READELF = mlx16-readelf

# Standard utilities (MUST be in the environment PATH)
ECHO  = echo
RM    = rm -fr
CP    = cp -f
MKDIR = mkdir -p
DIFF  = diff -Nq
UNCRUSTIFY = uncrustify

# Additional tools
ifeq ($(OS),Windows_NT)
	LIN_NODEGEN ?= $(ROOTDIR)/bin/lin_nodegen.exe
	MELIBU_NODEGEN ?= $(ROOTDIR)/bin/melibu_nodegen.exe
	PYMLXHEX  ?= $(ROOTDIR)/bin/pymlxhex.exe
	GENERATE_LD ?= $(ROOTDIR)/bin/generate_ld.exe
else
	LIN_NODEGEN ?= lin_nodegen
	MELIBU_NODEGEN ?= melibu_nodegen
	PYTHON3 ?= python3
	PYMLXHEX ?= pymlxhex
	GENERATE_LD ?= generate_ld
endif

#--- Application modules ------------------------------------------------------
include $(APP_MOD_DIR)/AppModules.mk

#--- Default search paths are included in the last term ---------------------- 
INC_DIRS += $(ROOTDIR)/include/$(CHIP_PATH_NAME)
