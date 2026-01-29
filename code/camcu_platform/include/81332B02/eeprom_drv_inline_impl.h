/**
 * @file
 * @brief EEPROM support library functions
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
 * @ingroup eeprom
 * @ingroup memory
 *
 * @brief EEPROM support library functions
 * @details
 * EEPROM driver. Used to manage the dedicated EEPROM type.
 * @{
 */

#ifndef EEPROM_DRV_INLINE_IMPL_H
#define EEPROM_DRV_INLINE_IMPL_H

#include <stdbool.h>
#include "io.h"

STATIC INLINE void EEPROM_SetConfigured(void)
{
    /* NOTE: EE_CONFIGURED should not be set to 1  before all EEPROM CS setting are not loaded into IO ports */
    IO_SET(EEPROM_FLASH, EE_CONFIGURED, 1u);
    while ( IO_GET(EEPROM_FLASH, EE_BUSY) != 0u ) {/* Wait until the EEPROM will be "configurable" TODO: What if not? */
    }
}

STATIC INLINE void EEPROM_SetTimings(void)
{
    DBG_CHAR_OUTPUT("EEtms");
    IO_HOST(EEPROM_FLASH, EE_PRE_WR_TIME) = EE_EEP_WR_TIMING_VALUE;
    IO_HOST(EEPROM_FLASH, EE_ER_TIME) = EE_EEP_ER_TIMING_VALUE;
    IO_HOST(EEPROM_FLASH, EE_PROGRAM_CYCLE) = EE_EEP_PROG_CYCLE_VALUE;
    IO_HOST(EEPROM_FLASH,
            EE_WAIT_STATES) =
        (EE_EEP_READ_TIMING_VALUE &
         ~(IO_MASK(EEPROM_FLASH,
                   EE_WAIT_STATES))) |
        ((uint16_t)IO_GET(EEPROM_FLASH, EE_WAIT_STATES) << IO_OFFSET(EEPROM_FLASH, EE_WAIT_STATES));
}

STATIC INLINE void EEPROM_SetRxRegisters(void)
{
    register uint16_t tmp; /* EEPROM KEY should be written always before each write operation */
    DBG_CHAR_OUTPUT("EErg");
    /* Configuring the EEPROM registers */ /* Remove the R1_EE << EEPROM */
    tmp = EE_FL_R1_EE;
    IO_SET(EEPROM_FLASH, EE_WE_KEY, EEPROM_WE_KEY_VALUE); /* TODO: Optimise the request if possible, to avoid the read-modify-write operation */
    EEPROM_CFG_REGS[1] = tmp;
    tmp = EE_FL_R2_EE;
    IO_SET(EEPROM_FLASH, EE_WE_KEY, EEPROM_WE_KEY_VALUE);
    EEPROM_CFG_REGS[2] = tmp;
}

#ifndef UNITTEST
STATIC INLINE void EEPROM_ClearErrorFlags(void)
{
    IO_SET(EEPROM_FLASH,
           EE_SBE_1, 1u,
           EE_DATA_CORRUPTED_1, 1u,
           EE_SBE_2, 1u,
           EE_DATA_CORRUPTED_2, 1u
           );     /* Test */
}
#endif /* UNITTEST */

STATIC INLINE bool EEPROM_GetErrorFlags(void)
{
    return (IO_GET(EEPROM_FLASH, EE_DATA_CORRUPTED_1) != 0u) || (IO_GET(EEPROM_FLASH, EE_DATA_CORRUPTED_2) != 0u);
}

STATIC INLINE bool EEPROM_getEEBUSY(void)
{
    return (IO_GET(EEPROM_FLASH, EE_BUSY) != 0u);
}

STATIC INLINE bool EEPROM_isUserArea(uint16_t address, uint8_t size16)
{
    return ( (address >= EE_USER_AREA_START) &&
             ((address + ((uint16_t)size16 * sizeof(uint16_t))) <= EE_USER_AREA_END) );
}

STATIC INLINE bool EEPROM_isPatchArea(uint16_t address, uint8_t size16)
{
    return ( (address >= EE_PATCH_AREA_START) &&
             ((address + ((uint16_t)size16 * sizeof(uint16_t))) <= EE_PATCH_AREA_END) );
}

#endif /* EEPROM_DRV_INLINE_IMPL_H */
/// @}
