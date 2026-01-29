/**
 * @file
 * @brief ROM, RAM, EEPROM memory startup checks library
 * @internal
 *
 * @copyright (C) 2015-2017 Melexis N.V.
 * git flash d0014c23
 *
 * Melexis N.V. is supplying this code for use with Melexis N.V. processor based microcontrollers only.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY,
 * INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.  MELEXIS N.V. SHALL NOT IN ANY CIRCUMSTANCES,
 * BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 * @endinternal
 *
 * @ingroup startup
 *
 * @brief ROM, RAM, EEPROM memory startup checks library
 * @details
 * @{
 */

#ifndef START_CHECKS_H
#define START_CHECKS_H

#include <syslib.h>
#include <plib.h>
#include "fatal.h"
#include "sys_tools.h"
#include "mem_checks.h"

void HAL_BIST_romTest(void);
void HAL_BIST_RamDestructiveTest(void);
#ifdef HAS_NO_SKIP_STARTUP_CHECKS
void HAL_BIST_RamCommonTest(void);
#endif /* HAS_NO_SKIP_STARTUP_CHECKS */
void CheckEEPROM(void);
#ifndef UNITTEST
#define HAL_BIST_RamTest(void) HAL_BIST_RamDestructiveTest(void) __attribute__ ((deprecated( \
                                                                                     "use HAL_BIST_RamDestructiveTest instead")))
#else
void Clear_EE_SingleBitErrorFlags(void);
bool Get_EE_SingleBitErrorFlags(void);
#endif


#endif
/// @}

