# @file
# @brief Application module sources list
# @internal
#
# @copyright (C) 2020 Melexis N.V.
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
# @details List of application sources to be build with the firmware application.
#
# @note The following variables are standard for all modules:
#       - VPATH
#       - APP_MODULES_SRCS
#       - APP_GEN_SRCS
#       - INC_DIRS
#       - ASFLAGS
#
# @note Module specific variables
#       - HAS_MLS_LOADER
#


#
# SEARCH PATHES
#
VPATH += $(APP_MOD_DIR)/mls_loader/config
VPATH += $(APP_MOD_DIR)/mls_loader/repository/src


#
# SOURCE FILES LIST
#
APP_MODULES_SRCS += mls_loader.c

APP_MODULES_SRCS += lin_ldr.c
APP_MODULES_SRCS += lin_ram_func_entry.c
APP_MODULES_SRCS += mlx_data_transfer.c
APP_MODULES_SRCS += protocol_extension.c
APP_MODULES_SRCS += read_write_eeprom.c
APP_MODULES_SRCS += read_write_flash.c
APP_MODULES_SRCS += read_write_ram.c


#
# HEADER SEARCH PATHES
#
INC_DIRS += $(APP_MOD_DIR)/mls_loader/config
INC_DIRS += $(APP_MOD_DIR)/mls_loader/repository/inc


#
# MODULE SPECIFIC DEFS
#

# Support for flash reading/programming messages
ifndef LDT_HAS_FLASH_FUNCTIONS
LDT_HAS_FLASH_FUNCTIONS = 0
endif
# Programmable flash area definitions
ifndef LDR_PROG_FLASH_START_INDEX
LDR_PROG_FLASH_START_INDEX = $(FLASH_START)
LDR_PROG_FLASH_END_INDEX = $(FLASH_END)
endif

PLTF_CPPFLAGS += -DLDR_PROG_FLASH_START_INDEX=$(LDR_PROG_FLASH_START_INDEX)
PLTF_CPPFLAGS += -DLDR_PROG_FLASH_END_INDEX=$(LDR_PROG_FLASH_END_INDEX)
PLTF_CPPFLAGS += -DLDT_HAS_FLASH_FUNCTIONS=$(LDT_HAS_FLASH_FUNCTIONS)

# Support for EEPROM reading/programming messages
ifndef LDT_HAS_EEPROM_FUNCTIONS
LDT_HAS_EEPROM_FUNCTIONS = 0
endif
# Programmable EEPROM area definitions
ifndef LDR_PROG_EEPROM_START_INDEX
LDR_PROG_EEPROM_START_INDEX = $(EEPROM_START)
LDR_PROG_EEPROM_END_INDEX = $(shell echo $$(( $(EEPROM_START) + $(EEPROM_SIZE) )))
endif

PLTF_CPPFLAGS += -DLDR_PROG_EEPROM_START_INDEX=$(LDR_PROG_EEPROM_START_INDEX)
PLTF_CPPFLAGS += -DLDR_PROG_EEPROM_END_INDEX=$(LDR_PROG_EEPROM_END_INDEX)
PLTF_CPPFLAGS += -DLDT_HAS_EEPROM_FUNCTIONS=$(LDT_HAS_EEPROM_FUNCTIONS)

PLTF_CPPFLAGS += -DHAS_MLS_LOADER
