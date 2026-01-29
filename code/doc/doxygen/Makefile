# @file
# @brief Generic doxygen makefile
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
# @details Makefile to trigger doxygen documentation generator
#

# Default values
DOXYGEN_IMAGES_DIR ?= $(DOXYGEN_INPUT_DIR)/images

DOXY_OPTIONS  = PROJECT_NAME="$(PROJECT_NAME)"
DOXY_OPTIONS += PROJECT_NUMBER="$(PROJECT_NUMBER)"
DOXY_OPTIONS += PROJECT_BRIEF="$(PROJECT_BRIEF)"
DOXY_OPTIONS += DOXYGEN_OUTPUT_DIR="$(subst /cygdrive/c,C:,$(DOXYGEN_OUTPUT_DIR))"
DOXY_OPTIONS += DOXYGEN_INPUT_DIR="$(subst /cygdrive/c,C:,$(DOXYGEN_INPUT_DIR)) $(DOXYGEN_MD_MAINPAGE)"
DOXY_OPTIONS += DOXYGEN_PREDEFINED="$(DOXYGEN_PREDEFINED)"
DOXY_OPTIONS += DOXYGEN_IMAGES_DIR="$(subst /cygdrive/c,C:,$(DOXYGEN_IMAGES_DIR))"
DOXY_OPTIONS += DOXYGEN_MD_MAINPAGE="$(DOXYGEN_MD_MAINPAGE)"

# Tools
ECHO  = echo
RM    = rm -fr
CP    = cp -f
MKDIR = mkdir -p

.PHONY: doxy_html
doxy_html: | $(DOXYGEN_OUTPUT_DIR)
	$(DOXY_OPTIONS) GENERATE_HTML="YES" GENERATE_LATEX="NO" doxygen ./doxyfile

.PHONY: doxy_tex
doxy_tex: build-spec-tex-update | $(DOXYGEN_OUTPUT_DIR)
	$(DOXY_OPTIONS) GENERATE_HTML="NO" GENERATE_LATEX="YES" doxygen ./doxyfile

.PHONY: doxy_pdf
doxy_pdf: doxy_tex
	'$(MAKE)' --directory=$(DOXYGEN_OUTPUT_DIR)/latex pdf

.PHONY: doxy_lua
doxy_lua: doxy_tex
	cd $(DOXYGEN_OUTPUT_DIR)/latex; lualatex --shell-escape refman.tex
	cd $(DOXYGEN_OUTPUT_DIR)/latex; lualatex --shell-escape refman.tex
	cd $(DOXYGEN_OUTPUT_DIR)/latex; lualatex --shell-escape refman.tex

.PHONY: clean clean_doxy
clean clean_doxy:
	-$(RM) $(DOXYGEN_OUTPUT_DIR)

$(DOXYGEN_OUTPUT_DIR):
	-$(MKDIR) $(DOXYGEN_OUTPUT_DIR)

.PHONY: build-spec-tex-update
build-spec-tex-update:
	sed -i 's|%AUTHORS%|$(AUTHORS)|g' latex/hdr-ext-build-spec.tex
	sed -i 's|"||g' latex/hdr-ext-build-spec.tex
