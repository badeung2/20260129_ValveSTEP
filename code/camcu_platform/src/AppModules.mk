# @file
# @brief Application modules definitions
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
# @details Extension Makefile to the application Makefile.
#          It contains list of modules available for the application.
#

#
# Get list of all the derived libraries
#
-include $(patsubst %,$(PLTF_DIR)/src/%/derivedlibs.mk,$(APP_MODULES))

APP_MODULES_TOTAL = $(sort $(APP_MODULES) $(APP_MODULES_DERIVED))

#
# Getting definitions from each application module
#
LIBS_DIRS = $(addprefix $(PLTF_DIR)/src/,$(APP_MODULES_TOTAL))
include $(addsuffix /srclist.mk,$(LIBS_DIRS))
