# @file
# @brief Translate REVISION to PROJECT_ID
# @internal
#
# @copyright (C) 2019 Melexis N.V.
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

ifeq ($(ORDER_CODE), 81340-xLW-AMx-001)
  PROJECT_ID = 0x0A01
  PROJECT_PATH_EXT = A01
  ROM_BASED_LOADER = PPM
  CHIP_PACKAGE = QFN32
  # lin auto addressing support
  HAS_LIN_AA_CAPABILITY = 1
endif

ifeq ($(ORDER_CODE), 81340-xLW-AMx-002)
  PROJECT_ID = 0x0A03
  PROJECT_PATH_EXT = A01
  ROM_BASED_LOADER = PPM
  CHIP_PACKAGE = QFN32
  # lin auto addressing support
  HAS_LIN_AA_CAPABILITY = 1
endif

ifeq ($(ORDER_CODE), 81340-xLW-AMx-101)
  PROJECT_ID = 0x0A02
  PROJECT_PATH_EXT = A01
  ROM_BASED_LOADER = PPM
  CHIP_PACKAGE = QFN24
endif

ifeq ($(ORDER_CODE), 81340-xLW-AMx-102)
  PROJECT_ID = 0x0A04
  PROJECT_PATH_EXT = A01
  ROM_BASED_LOADER = PPM
  CHIP_PACKAGE = QFN24
endif

ifeq ($(ORDER_CODE), 81340-xLW-BMx-003)
  PROJECT_ID = 0x0A05
  PROJECT_PATH_EXT = B01
  ROM_BASED_LOADER = PPM
  ROM_BASED_LOADER += UDS_LOADER
  CHIP_PACKAGE = QFN32
  # lin auto addressing support
  HAS_LIN_AA_CAPABILITY = 1
endif

ifeq ($(ORDER_CODE), 81340-xLW-BMx-103)
  PROJECT_ID = 0x0A06
  PROJECT_PATH_EXT = B01
  ROM_BASED_LOADER = PPM
  ROM_BASED_LOADER += UDS_LOADER
  CHIP_PACKAGE = QFN24
endif


ifndef PROJECT_ID
  $(error 'Error: Unknown order code selected ($(ORDER_CODE)).')
endif


CHIP_REVISION = $(PROJECT_PATH_EXT)

# expose the chip package type to the sources
PLTF_DEFS += CHIP_PACKAGE=$(CHIP_PACKAGE)

# use the current source instead of the pullup in preselection stage from EEPROM calibration version X
LIN_AA_PULLUP_BY_CS_VERSION = 1

# uds services are stored in flash start
UDS_SERVICES_AT_START = 1

# default uds is not enabled
UDS_LOADER ?= 0
