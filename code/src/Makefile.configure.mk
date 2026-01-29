# @file
# @brief Build Configuration File
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
# @details Build configuration file for the application.
#


#
# Target name
#
TARGET_EXT := I03_eVent


#
# MCU Order Code & Target name
#
# QFN24
ORDER_CODE ?= 81332-xLW-BMx-202



#
# Application CPU clock frequency (in MHz)
#
#FW_FREQ = 12
#FW_FREQ = 14
#FW_FREQ = 16
#FW_FREQ = 24
FW_FREQ = 28
#FW_FREQ = 32


#
# Divide MLX4 clock by (1+n)
#
#LIN_CLK_DIV = 1


#
# Flash bist page count definition
# @note Number of pages from start of flash to be checked by flash bist (< 0x0100)
#
BIST_PAGE_COUNT = 0x0100
#No Flash BIST
#BIST_PAGE_COUNT = 0x0000


#
# Special application options to be added to CPPFLAGS
#
APP_OPTIONS += -DREMAP_FP0_2_FLASH
