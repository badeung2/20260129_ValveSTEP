# @file
# @brief Application module sources list
# @internal
#
# @copyright (C) 2018 Melexis N.V.
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
#       - APP_PRODUCT_ID
#       - APP_VERSION_MAJOR
#       - APP_VERSION_MINOR
#       - APP_VERSION_PATCH
#       - APP_VERSION_BUILD
#       - PROTECTION_KEY


#
# SEARCH PATHES
#
VPATH += $(APP_MOD_DIR)/app_descriptor


#
# SOURCE FILES LIST
#
APP_MODULES_SRCS += app_descriptor.c


#
# HEADER SEARCH PATHES
#
INC_DIRS += $(APP_MOD_DIR)/app_descriptor


#
# MODULE SPECIFIC DEFS
#
APP_PRODUCT_ID ?=
ifeq ($(strip $(APP_PRODUCT_ID)),)
# default product id = <chip name>+<path_ext>
APP_PRODUCT_ID = $(CHIP_PATH_NAME)
$(warning Warning: Variable APP_PRODUCT_ID is not defined, using default name $(APP_PRODUCT_ID))
endif

ifndef APP_VERSION_MAJOR
APP_VERSION_MAJOR ?= 0
$(warning Warning: Variable APP_VERSION_MAJOR is not defined, using default major version nr $(APP_VERSION_MAJOR))
endif
ifndef APP_VERSION_MINOR
APP_VERSION_MINOR ?= 0
$(warning Warning: Variable APP_VERSION_MINOR is not defined, using default minor version nr $(APP_VERSION_MINOR))
endif
ifndef APP_VERSION_PATCH
APP_VERSION_PATCH ?= 0
$(warning Warning: Variable APP_VERSION_PATCH is not defined, using default patch version nr $(APP_VERSION_PATCH))
endif
ifndef APP_VERSION_BUILD
APP_VERSION_BUILD ?= 0
$(warning Warning: Variable APP_VERSION_BUILD is not defined, using default build version nr $(APP_VERSION_BUILD))
endif

ifndef PROTECTION_KEY
PROTECTION_KEY ?= 0
$(warning Warning: Variable PROTECTION_KEY is not defined, flash read-out protection will be disabled)
endif

PLTF_CPPFLAGS += -DAPP_PRODUCT_ID="\"$(APP_PRODUCT_ID)\""
PLTF_CPPFLAGS += -DAPP_VERSION_MAJOR=$(APP_VERSION_MAJOR)
PLTF_CPPFLAGS += -DAPP_VERSION_MINOR=$(APP_VERSION_MINOR)
PLTF_CPPFLAGS += -DAPP_VERSION_PATCH=$(APP_VERSION_PATCH)
PLTF_CPPFLAGS += -DAPP_VERSION_BUILD=$(APP_VERSION_BUILD)
PLTF_CPPFLAGS += -DPROTECTION_KEY=$(PROTECTION_KEY)
