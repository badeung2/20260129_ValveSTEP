# @file
# @brief Project makefile
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
# @details
# Makefile which will re-route targets to the correct folder and Makefile
#

# project folders
ROOT_DIR     = .
DOC_DIR      = $(ROOT_DIR)/code/doc
SRC_DIR      = $(ROOT_DIR)/code/src
HIL_DIR      = $(ROOT_DIR)/verification/hil_tests
UTEST_DIR    = $(ROOT_DIR)/verification/unit_test

DOC_GOALS   := doxy_html doxy_pdf doxy_tex doxy_lua clean_doxy
SRC_GOALS   := all clean libs clean_libs drv clean_drv release release_hil uncrustify uncrustify-check lint 
HIL_GOALS   := hil_tests ptc_fws
UTEST_GOALS := utest

.PHONY: $(DOC_GOALS)
$(DOC_GOALS):
	@'$(MAKE)' --no-print-directory --directory=$(DOC_DIR) $@

.PHONY: $(SRC_GOALS)
$(SRC_GOALS):
	@'$(MAKE)' --no-print-directory --directory=$(SRC_DIR) $@

.PHONY: $(HIL_GOALS)
$(HIL_GOALS):
	@'$(MAKE)' --no-print-directory --directory=$(HIL_DIR) $@

.PHONY: $(UTEST_GOALS)
$(UTEST_GOALS):
	@'$(MAKE)' --no-print-directory --directory=$(UTEST_DIR) $@
