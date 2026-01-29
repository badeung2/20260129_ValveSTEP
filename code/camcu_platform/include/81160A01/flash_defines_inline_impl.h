/**
 * @file
 * @brief FLASH write support and initialization functions library
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
 * @ingroup flash
 *
 * @brief FLASH write support and initialization functions library
 * @details
 * FLASH driver. Used to manage the dedicated FLASH type.
 * @{
 */

#ifndef FLASH_DEFINES_INLINE_IMPL_H
#define FLASH_DEFINES_INLINE_IMPL_H

#include "compiler_abstraction.h"
#include "io.h"
#include "eeprom_map.h"
#include "flash_cfg.h"
#include "sys_tools.h"

/**
 * Debug output for the Flash.
 * Use general project settings.
 */
#define FL_DEBUG_OUTPUT(info)   DEBUG_OUTPUT(info)

/** FLASH_getStatus returns the status of the FLASH HW */
STATIC INLINE uint16_t FLASH_getStatus(void)
{
    return IO_GET(EEPROM_FLASH, FL_STATUS);
}

/** FLASH_WAIT_STATUS waits for the FLASH's status specified
 * @param[in] status the status to expect
 * @note the function performs the infinite loop. Therefore AWD acknowledge should be handled accordingly
 */
STATIC INLINE void FLASH_WAIT_STATUS(uint16_t status)
{
#ifndef UNITTEST
    do {
        ;
    } while (IO_GET(EEPROM_FLASH, FL_STATUS) != status);
#else
    IO_SET(EEPROM_FLASH, FL_STATUS, status); /* Mimic the positive HW response for the unit-tests*/
#endif /* UNITTEST */
}

/** FLASH_WAIT_NOT_STATUS waits for the FLASH's status will be changed to any than specified
 * @param[in] status the status to expect it changed
 * @note the function performs the infinite loop. Therefore AWD acknowledge should be handled accordingly
 */
STATIC INLINE void FLASH_WAIT_NOT_STATUS(uint16_t status)
{
    do {
        ;
    } while (IO_GET(EEPROM_FLASH, FL_STATUS) == status);
}

STATIC INLINE void FLASH_SetTimings(void)
{
    FL_DEBUG_OUTPUT("FL: Trimming");

    IO_HOST(EEPROM_FLASH, T_10NS) = EE_FL_10NS_50NS_VALUE;
    IO_HOST(EEPROM_FLASH, T_1US) = EE_FL_1US_VALUE;
    IO_HOST(EEPROM_FLASH, FL_ER_TIME) = EE_FL_ER_WR_TIMING_VALUE;
#ifdef DBG_FPGA_
    dumpMem16((uint16_t)&EE_FL_10NS_50NS_VALUE, 1);
    dumpMem16((uint16_t)&EE_FL_1US_VALUE, 1);
    dumpMem16((uint16_t)&EE_FL_ER_WR_TIMING_VALUE, 1);
#endif
    IO_HOST(EEPROM_FLASH,
            FL_WAIT_STATES) =
        (EE_FL_CFG_FL &
         ~(IO_MASK(EEPROM_FLASH,
                   FL_WAIT_STATES))) |
        ((uint16_t)IO_GET(EEPROM_FLASH, FL_WAIT_STATES) << IO_OFFSET(EEPROM_FLASH, FL_WAIT_STATES));
}

/** FLASH_ClearDEDflag clears the DATA_CORRUPTED flag */
STATIC INLINE void FLASH_ClearDEDflag(void)
{
    IO_SET(EEPROM_FLASH, FL_DATA_CORRUPTED, 1u);
}

/** FLASH_ClearSECflag clears the FLASH single bit error flag */
STATIC INLINE void FLASH_ClearSECflag(void)
{
    IO_SET(EEPROM_FLASH, FL_SBE, 1u);
}

#endif /* FLASH_DEFINES_INLINE_IMPL_H */
/// @}

