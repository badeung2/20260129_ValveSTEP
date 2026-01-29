# @file srclist.mk
# @brief Application module sources list
# @internal
#
# @copyright (C) 2018-2019 Melexis N.V.
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
#       - LDF_NODEGEN_FLAGS
#       - LIN_MASTER_API
#       - LIN_SLAVE_API
#
# @note Module variables dependencies
#       - LDF_FILE
#       - LIN_NODE
#
# @note Module specific rules
#       - drv
#       - clean_drv
#       - doxy_drv
#       - clean_doxy_drv
#       - $(LDF_GEN_SRCS)


#
# MODULE SPECIFIC DEFS
#
# ldf nodegen checks and flags
ifndef LDF_FILE
$(error 'Error: Variable LDF_FILE is not defined correctly')
endif
ifeq ($(LDF_FILE),)
$(error 'Error: Variable LDF_FILE is not defined correctly')
endif

ifndef LIN_NODE
$(warning 'Warning: Variable LIN_NODE is not defined, generating code for all nodes in ldf file')
LDF_NODEGEN_FLAGS += -a
else
LDF_NODEGEN_FLAGS += -n$(LIN_NODE)
endif

LDF_NODEGEN_FLAGS += -Cf -Cv
# enable automatic overwriting existing files
LDF_NODEGEN_FLAGS += -f
# disable warnings for overwriting existing files
LDF_NODEGEN_FLAGS += -pf
# warning logging level, hides action summary from logs.
LDF_NODEGEN_FLAGS += -w

LIN_MASTER_API ?= 0
LIN_SLAVE_API ?= 1
PLTF_CPPFLAGS += -DLIN_MASTER_API=$(LIN_MASTER_API)
PLTF_CPPFLAGS += -DLIN_SLAVE_API=$(LIN_SLAVE_API)
PLTF_CPPFLAGS += -DHAS_STD_LIN_API


#
# SEARCH PATHES
#
VPATH += $(APP_MOD_DIR)/std_lin_api/config
VPATH += $(APP_MOD_DIR)/std_lin_api/repository/src/common
VPATH += $(APP_MOD_DIR)/std_lin_api/repository/src/master
VPATH += $(APP_MOD_DIR)/std_lin_api/repository/src/slave


#
# SOURCE FILES LIST
#
APP_MODULES_SRCS += lin_api_config.c lin_core.c lin_trans.c lin_signals.c lin_signals.h
ifeq ($(LIN_MASTER_API), 1)
APP_MODULES_SRCS += lin_core_ma.c lin_cfg_ma.c lin_trans_ma.c
endif
ifeq ($(LIN_SLAVE_API), 1)
APP_MODULES_SRCS += lin_core_sl.c lin_cfg_sl.c lin_trans_sl.c lin2b_romtbl.S
endif
LDF_GEN_SRCS = lin_signals.c lin_signals.h lin2b_romtbl.S
APP_GEN_SRCS += $(LDF_GEN_SRCS)


#
# HEADER SEARCH PATHES
#
INC_DIRS += $(APP_MOD_DIR)/std_lin_api/config
INC_DIRS += $(APP_MOD_DIR)/std_lin_api/repository/inc/common
INC_DIRS += $(APP_MOD_DIR)/std_lin_api/repository/inc/master
INC_DIRS += $(APP_MOD_DIR)/std_lin_api/repository/inc/slave


#
# MODULE SPECIFIC RULES
#

# Generate / clean LDF-file specific LIN API part by LDF-Nodegen tool
.PHONY: drv clean_drv
drv: $(LDF_GEN_SRCS)

clean_drv:
	-$(HIDE_CMD)$(RM) $(LDF_GEN_SRCS)
	@$(ECHO) --- Cleaning LDF-file specific LIN API part has been finished

$(LDF_GEN_SRCS): | LIN_NODEGEN-version
	@$(ECHO) --- Running LDF-nodegen from $(call RELPATH,$(CURDIR))
	$(HIDE_CMD)$(LIN_NODEGEN) $(LDF_NODEGEN_FLAGS) $(LDF_FILE)
	@$(ECHO) --- LDF-file specific LIN API part has been successfully generated

# Std. LIN API doxygen generated documentation
.PHONY: doxy_drv clean_doxy_drv
doxy_drv:
	$(HIDE_CMD)$(MAKE) -C $(APP_MOD_DIR)/std_lin_api/repository doxy DOXY_APPBUILDDIR=$(CURDIR)/doxy_std_lin_api GEN_DIR=$(CURDIR) LIN_MASTER_API=0 LIN_SLAVE_API=1

clean_doxy_drv:
	$(HIDE_CMD)$(MAKE) -C $(APP_MOD_DIR)/std_lin_api/repository clean_doxy DOXY_APPBUILDDIR=$(CURDIR)/doxy_std_lin_api
