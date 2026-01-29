# @file
# @brief Application module sources list
# @internal
#
# @copyright (C) 2021 Melexis N.V.
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
# SEARCH PATHES
#
VPATH += $(APP_MOD_DIR)/patch_rom/$(PRODUCT)


#
# SOURCE FILES LIST
#
APP_MODULES_SRCS += $(notdir $(wildcard $(APP_MOD_DIR)/patch_rom/$(PRODUCT)/*.c))
APP_MODULES_SRCS += $(notdir $(wildcard $(APP_MOD_DIR)/patch_rom/$(PRODUCT)/*.S))


#
# ADDITIONAL SOURCE FILES LIST
#
-include $(APP_MOD_DIR)/patch_rom/$(PRODUCT)/srclist.mk


#
# HEADER SEARCH PATHES
#
INC_DIRS += $(APP_MOD_DIR)/patch_rom/$(PRODUCT)


#
# MODULE SPECIFIC DEFS
#
PLTF_CPPFLAGS += -DHAS_PATCH_ROM
