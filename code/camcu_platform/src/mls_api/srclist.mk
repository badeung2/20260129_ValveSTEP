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
#       - HAS_MLS_API
#       - FW_LIN_TL_EMPTY_ERROR_HANDLER
#       - FW_LIN_TL_LDT_TICK_FIX
#       - FW_LIN_TL_FUNCTIONAL_NAD_FIX
#       - LIN_CLK_DIV (relies on external definition)

#
# SEARCH PATHES
#
VPATH += $(APP_MOD_DIR)/mls_api


#
# SOURCE FILES LIST
#
APP_MODULES_SRCS += fw_mls_module_cst_table.S
APP_MODULES_SRCS += fw_mls_api.c


#
# HEADER SEARCH PATHES
#
INC_DIRS += $(APP_MOD_DIR)/mls_api


#
# ADDITIONAL COMPILER FLAGS
#
ASFLAGS += -Wa,-I$(LIBDIR)/$(CHIP_PATH_NAME)


#
# MODULE SPECIFIC DEFS
#

# overwrite LIN TL handler for MLX4 errors by the empty
FW_LIN_TL_EMPTY_ERROR_HANDLER ?= 0 

# fix LIN TL ldt_Tick() issue with intermediate handler
FW_LIN_TL_LDT_TICK_FIX ?= 0

# fix LIN TL Functional NAD frame issue with intermediate handler
FW_LIN_TL_FUNCTIONAL_NAD_FIX ?= 0

PLTF_CPPFLAGS += -DHAS_MLS_API
PLTF_CPPFLAGS += -DFW_LIN_TL_EMPTY_ERROR_HANDLER=$(FW_LIN_TL_EMPTY_ERROR_HANDLER)
PLTF_CPPFLAGS += -DFW_LIN_TL_LDT_TICK_FIX=$(FW_LIN_TL_LDT_TICK_FIX)
PLTF_CPPFLAGS += -DFW_LIN_TL_FUNCTIONAL_NAD_FIX=$(FW_LIN_TL_FUNCTIONAL_NAD_FIX)
