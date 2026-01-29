# @file
# @brief Project documentation makefile
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
# @details Makefile to run doxygen documentation generator
#

# Project root folder
PROJ_DIR = ./../..

# project specific settings
AUTHORS = "MMP"
PROJECT_NAME = Doxygen code documentation
PROJECT_NUMBER = MLX81330
PROJECT_BRIEF = MLX81330 BiPolar Demo Project Firmware Description

# project specific directories
DOXYGEN_OUTPUT_DIR = $(subst /cygdrive/c,C:,$(abspath $(PROJ_DIR)/release/doxygen))
DOXYGEN_INPUT_DIR = $(subst /cygdrive/c,C:,$(abspath $(PROJ_DIR)/code/src))

# list of arguments to pass to doxygen submodule makefile
DOXY_OPTIONS  = PROJECT_NAME="$(PROJECT_NAME)"
DOXY_OPTIONS += PROJECT_NUMBER="$(PROJECT_NUMBER)"
DOXY_OPTIONS += PROJECT_BRIEF="$(PROJECT_BRIEF)"
DOXY_OPTIONS += DOXYGEN_OUTPUT_DIR="$(DOXYGEN_OUTPUT_DIR)"
DOXY_OPTIONS += DOXYGEN_INPUT_DIR="$(DOXYGEN_INPUT_DIR)"
DOXY_OPTIONS += AUTHORS=$(AUTHORS)

# make targets for the doxygen submodule
DOXY_GOALS   := doxy_html doxy_pdf doxy_tex doxy_lua clean_doxy

.PHONY: $(DOXY_GOALS)
$(DOXY_GOALS):
	'$(MAKE)' --directory=./doxygen $@ $(DOXY_OPTIONS)
