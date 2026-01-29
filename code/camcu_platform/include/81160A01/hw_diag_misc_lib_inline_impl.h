/**
 * @file
 * @brief Common library for the hw_diag module
 * @internal
 *
 * @copyright (C) 2019 Melexis N.V.
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
 * @ingroup hw_diag
 * @ingroup library
 *
 * @brief Common library for the hw_diag module
 * @details
 * @{
 */

#ifndef HW_DIAG_MISC_LIB_INLINE_IMPL_H
#define HW_DIAG_MISC_LIB_INLINE_IMPL_H

#include <stdint.h>
#include "compiler_abstraction.h"
#include "sys_tools.h"
#include "bl_tables.h"
#include "bl_bist.h"

/** Fills-in an array of data with the current data under control
 * @param[in, out] buf   specifies an array to write into
 */
STATIC INLINE void GetBasicDataDiagFootprint(uint16_t buf[])
{
    uint16_t buf_offset = 0;
    buf[buf_offset] = (uint16_t)bistHeader;
    ++buf_offset;
    buf[buf_offset] = (uint16_t)bl_table.key;
    ++buf_offset;
    buf[buf_offset] = (uint16_t)(bl_table.key >> 16);
}


/** Returns the checksum for the basic data values' set
 * @return checksum
 */
STATIC INLINE uint16_t GetBasicDataDiagFootprintCs(void)
{
    uint16_t buf[HW_DIAG_BASIC_DATA_SIZE];
    GetBasicDataDiagFootprint(buf);
    return HW_DIAG_GetDataCs(buf, HW_DIAG_BASIC_DATA_SIZE);
}

#endif /* HW_DIAG_MISC_LIB_INLINE_IMPL_H */
/// @}

