# @file srclist.mk
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
#       - HAS_MLS_DEVICE_ID
#       - ROM_BASED_LOADER

#
# SEARCH PATHES
#
VPATH += $(APP_MOD_DIR)/mls_device_id


#
# SOURCE FILES LIST
#
APP_MODULES_SRCS += fw_mls_device_id.c


#
# HEADER SEARCH PATHES
#
INC_DIRS += $(APP_MOD_DIR)/mls_device_id


#
# MODULE SPECIFIC DEFS
#
ifneq ($(filter PPM,$(ROM_BASED_LOADER)),)
HAS_PPM_EPM = 1
else
HAS_PPM_EPM = 0
endif
PLTF_CPPFLAGS += -DHAS_PPM_EPM=$(HAS_PPM_EPM)

ifneq ($(filter LIN,$(ROM_BASED_LOADER)),)
HAS_LIN_EPM = 1
else
HAS_LIN_EPM = 0
endif
PLTF_CPPFLAGS += -DHAS_LIN_EPM=$(HAS_LIN_EPM)

PLTF_CPPFLAGS += -DHAS_MLS_DEVICE_ID
