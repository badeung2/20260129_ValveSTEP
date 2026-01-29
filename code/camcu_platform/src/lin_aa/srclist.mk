# @file srclist.mk
# @brief Application module sources list
# @internal
#
# @copyright (C) 2018-2020 Melexis N.V.
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
#       - HAS_LIN_AUTO_ADDRESSING
#


#
# SEARCH PATHES
#
VPATH += $(APP_MOD_DIR)/lin_aa/


#
# SOURCE FILES LIST
#
APP_MODULES_SRCS += fw_lin_auto_addressing.c


#
# HEADER SEARCH PATHES
#
INC_DIRS += $(APP_MOD_DIR)/lin_aa/


#
# MODULE SPECIFIC DEFS
#
HAS_LIN_AA_CAPABILITY ?= 0

ifeq ($(HAS_LIN_AA_CAPABILITY),0)
  $(error 'Error: Chip $(ORDER_CODE) does not support LIN auto addressing.')
endif

# LIN AA needs VDDA to be switched to 5V
HAS_LIN_AA_VDDA_5V ?= 0
ifeq ($(HAS_LIN_AA_VDDA_5V),1)
PLTF_CPPFLAGS += -DHAS_LIN_AA_VDDA_5V
endif

# extended LIN AA voltage range (only MLX81340AB)
HAS_LIN_AA_EXT_RANGE ?= 0
ifeq ($(HAS_LIN_AA_EXT_RANGE),1)
PLTF_CPPFLAGS += -DHAS_LIN_AA_EXT_RANGE
endif

# LIN AA needs motor driver supply to be enabled
HAS_LIN_AA_DRV_SUP_ENABLE ?= 0
ifeq ($(HAS_LIN_AA_DRV_SUP_ENABLE),1)
PLTF_CPPFLAGS += -DHAS_LIN_AA_DRV_SUP_ENABLE
endif

# the variable is set when the APP_MODULE is included
PLTF_CPPFLAGS += -DHAS_LIN_AUTO_ADDRESSING

PLTF_CPPFLAGS += -DLIN_AA_PULLUP_BY_CS_VERSION=$(LIN_AA_PULLUP_BY_CS_VERSION)
