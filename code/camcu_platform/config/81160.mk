# @file
# @brief Translate REVISION to PROJECT_ID
# @internal
#
# @copyright (C) 2020 Melexis N.V.
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
# @ingroup config
#
# @details This file assigns the dependence between chip's revision and the project id's
#

ifeq ($(ORDER_CODE), 81160-xLW-AMx-001)
  PROJECT_ID = 0x1601
  PROJECT_PATH_EXT = A01
  ROM_BASED_LOADER = PPM
  CHIP_PACKAGE = QFN24
endif

ifndef PROJECT_ID
  $(error 'Error: Unknown order code selected ($(ORDER_CODE)).')
endif


CHIP_REVISION = $(PROJECT_PATH_EXT)

# expose the chip package type to the sources
PLTF_DEFS += CHIP_PACKAGE=$(CHIP_PACKAGE)
