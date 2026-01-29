/**
 * @file
 * @brief PPM bootloader functions
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
 * @addtogroup ppm_phy PPM Physical Layer
 *
 * @brief PPM bootloader functions that require external linking
 * @details
 * The Pulse-position-modulation (PPM) is the Physical Layer library providing the simple API to maintain,
 * configure and provide the data transfer over the PPM-bus (will be based on the LIN PIN).
 * @{
 */

#ifndef BL_PPM_ENTRY_H
#define BL_PPM_ENTRY_H

#if !defined(HAS_SW_PPM_BL_USE_ROM_ONLY) && !defined(HAS_SW_PPM_HAS_RAM_CODE_FRAME)
#error Forbidden configuration. The one from (HAS_SW_PPM_BL_USE_ROM_ONLY, HAS_SW_PPM_HAS_RAM_CODE_FRAME) should be used!
#endif

#include "bl_ppm.h"

#ifdef HAS_SW_PPM_HAS_RAM_CODE_FRAME
ppm_RamEntryResult_t ppm_ram_entry_func(const ppm_RamEntryInput_t* input_data);
#endif

#endif
/// @}
