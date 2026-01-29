# @file
# @brief Translate REVISION to PROJECT_ID
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
# @ingroup config
#
# @details This file assigns the dependence between chip's revision and the project id's
#

ifeq ($(ORDER_CODE), 81332-xLW-AMx-201)
  PROJECT_ID = 0x0701
  PROJECT_PATH_EXT = A01
  ROM_BASED_LOADER = PPM
  CHIP_PACKAGE = QFN24
  # lin auto addressing support
  HAS_LIN_AA_CAPABILITY = 1
  HAS_LIN_AA_VDDA_5V = 1
endif

ifeq ($(ORDER_CODE), 81332-xDC-AMx-001)
  PROJECT_ID = 0x0705
  PROJECT_PATH_EXT = A01
  ROM_BASED_LOADER = PPM
  CHIP_PACKAGE = SO8
  # lin auto addressing support
  HAS_LIN_AA_CAPABILITY = 1
  HAS_LIN_AA_VDDA_5V = 1
endif

ifeq ($(ORDER_CODE), 81332-xLW-BMx-201)
  PROJECT_ID = 0x070A
  PROJECT_PATH_EXT = B01
  ROM_BASED_LOADER = PPM
  CHIP_PACKAGE = QFN24
  # lin auto addressing support
  HAS_LIN_AA_CAPABILITY = 1
  HAS_LIN_AA_VDDA_5V = 1
  HAS_LIN_AA_DRV_SUP_ENABLE = 1
endif

ifeq ($(ORDER_CODE), 81332-xLW-BMx-202)
  PROJECT_ID = 0x070F
  PROJECT_PATH_EXT = B02
  ROM_BASED_LOADER = PPM
  CHIP_PACKAGE = QFN24
  # lin auto addressing support
  HAS_LIN_AA_CAPABILITY = 1
endif

ifeq ($(ORDER_CODE), 81332-xDC-BMx-001)
  PROJECT_ID = 0x070B
  PROJECT_PATH_EXT = B01
  ROM_BASED_LOADER = PPM
  CHIP_PACKAGE = SO8
  # lin auto addressing support
  HAS_LIN_AA_CAPABILITY = 1
  HAS_LIN_AA_VDDA_5V = 1
  HAS_LIN_AA_DRV_SUP_ENABLE = 1
endif

ifeq ($(ORDER_CODE), 81332-xDC-BMx-101)
  PROJECT_ID = 0x070C
  PROJECT_PATH_EXT = B01
  ROM_BASED_LOADER = PPM
  CHIP_PACKAGE = SO8
  # lin auto addressing support
  HAS_LIN_AA_CAPABILITY = 1
  HAS_LIN_AA_VDDA_5V = 1
  HAS_LIN_AA_DRV_SUP_ENABLE = 1
endif

ifeq ($(ORDER_CODE), 81332-xDC-BMx-201)
  PROJECT_ID = 0x070D
  PROJECT_PATH_EXT = B01
  ROM_BASED_LOADER = PPM
  CHIP_PACKAGE = SO8
endif

ifeq ($(ORDER_CODE), 81332-xDC-BMx-301)
  PROJECT_ID = 0x070E
  PROJECT_PATH_EXT = B01
  ROM_BASED_LOADER = PPM
  CHIP_PACKAGE = SO8
endif

ifeq ($(ORDER_CODE), 81332-xDC-BMx-002)
  PROJECT_ID = 0x0710
  PROJECT_PATH_EXT = B02
  ROM_BASED_LOADER = PPM
  CHIP_PACKAGE = SO8
  # lin auto addressing support
  HAS_LIN_AA_CAPABILITY = 1
endif

ifeq ($(ORDER_CODE), 81332-xDC-BMx-102)
  PROJECT_ID = 0x0711
  PROJECT_PATH_EXT = B02
  ROM_BASED_LOADER = PPM
  CHIP_PACKAGE = SO8
  # lin auto addressing support
  HAS_LIN_AA_CAPABILITY = 1
endif

ifeq ($(ORDER_CODE), 81332-xDC-BMx-202)
  PROJECT_ID = 0x0712
  PROJECT_PATH_EXT = B02
  ROM_BASED_LOADER = PPM
  CHIP_PACKAGE = SO8
endif

ifeq ($(ORDER_CODE), 81332-xDC-BMx-302)
  PROJECT_ID = 0x0713
  PROJECT_PATH_EXT = B02
  ROM_BASED_LOADER = PPM
  CHIP_PACKAGE = SO8
endif

ifndef PROJECT_ID
  $(error 'Error: Unknown order code selected ($(ORDER_CODE)).')
endif


CHIP_REVISION = $(PROJECT_PATH_EXT)

# expose the chip package type to the sources
PLTF_DEFS += CHIP_PACKAGE=$(CHIP_PACKAGE)

# use the current source instead of the pullup in preselection stage from EEPROM calibration version X
LIN_AA_PULLUP_BY_CS_VERSION = 3
