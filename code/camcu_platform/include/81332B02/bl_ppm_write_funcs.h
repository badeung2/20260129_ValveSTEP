/**
 * @file
 * @brief PPM bootloader FLASH-write functions
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
 * @ingroup ppm_phy
 *
 * @brief PPM bootloader FLASH-write functions
 * @details
 * The Pulse-position-modulation (PPM) is the Physical Layer library providing the simple API to maintain,
 * configure and provide the data transfer over the PPM-bus (will be based on the LIN PIN).
 */

#ifndef BL_PPM_WRITE_FUNCS_H
#define BL_PPM_WRITE_FUNCS_H

#include <stdint.h>
#include <syslib.h>
#include "static_assert.h"
#include "ppm_lib.h"
#include "bl_ppm.h"
#include "lib_miscio.h"
#include "flash_drv.h"

#ifdef HAS_SW_PPM_BL_USE_ROM_ONLY

/** Start the FLASH page write operation
 *
 * @param[in] index Not used
 */
void ppm_ProcessFlashData(uint16_t index);

/** Start the FLASH CS page write operation
 *
 * @param[in] index Not used
 */
void ppm_ProcessFlashCsData(uint16_t index);

/** Execute the EEPROM page write operation
 *
 * @param[in] index Not used
 */
void ppm_ProcessEepromData(uint16_t index);

/** Triggers the entire FLASH erase operation
 */
void ppm_ProcessFlashErase(void);

/** Triggers the FLASH CS erase operation
 */
void ppm_ProcessFlashEraseCs(void);

/** Triggers the FLASH state-machine step
 */
void ppm_ProcessFlashWrite(void);

#endif /* HAS_SW_PPM_BL_USE_ROM_ONLY */

#if !defined(HAS_SW_PPM_HAS_RAM_CODE_FRAME) || defined(HAS_SW_PPM_BL_USE_ROM_ONLY)
/** Process the Session's acknowledge event, when FLASH-write operation is expected to be finished
 *
 * @retval PPM_RERES_OK Succeeded to process the acknowledgement
 * @retval PPM_RERES_UNKNOWN otherwise
 */
ppm_RamEntryResult_t ppm_ProcessSessionAck(void);
#endif

#endif /* BL_PPM_WRITE_FUNCS_H */
