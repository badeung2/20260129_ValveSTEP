/**
 * @file
 * @brief Common library for the hw_diag module Complex timer
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
 * @ingroup ctimer
 *
 * @brief Common library for the hw_diag module Complex timer
 * @details
 */

#ifndef CTIMER_LIB_DIAG_INLINE_IMPL_H
#define CTIMER_LIB_DIAG_INLINE_IMPL_H

#include <stdint.h>
#include "io.h"
#include "static_assert.h"
#include "compiler_abstraction.h"
#include "sys_tools.h"
#include "hw_diag.h"

#if defined(HAS_HW_DIAG_CTIMER0_ENABLE)
/** Fills-in an array of a CTIMER0 HW registers data with the current state
 *
 * all CTIMER0 HW configuaration is saved
 *
 * @param[in, out] buf   specifies an array to write into
 */
STATIC INLINE void GetCtimer0DataHwDiagFootprint(uint16_t buf[])
{
    uint16_t buf_offset = 0;
    buf[buf_offset] = IO_HOST(CTIMER0, TREGB);
    ++buf_offset;
    buf[buf_offset] = IO_HOST(CTIMER0, TREGA);
    ++buf_offset;
    buf[buf_offset] = IO_HOST(CTIMER0, START);
}

/** Returns the checksum for the trimming values' set
 * @return checksum
 */
STATIC INLINE uint16_t GetCtimer0DataHwDiagFootprintCs(void)
{
    uint16_t buf[HW_DIAG_CTIMER0_DATA_SIZE];
    GetCtimer0DataHwDiagFootprint(buf);
    return HW_DIAG_GetDataCs(buf, HW_DIAG_CTIMER0_DATA_SIZE);
}

/* Additional assert assuring the fields are properly grouped. If ok, IO_MASK() will not fail */
ASSERT( IO_MASK(CTIMER0, START, STOP, EDGA, EDGB, PWMI, POL, OVRA, OVRB, ENCMP, MODE, DIV) != 0u );

#endif /* HAS_HW_DIAG_CTIMER0_ENABLE */

#if defined(IO_CTIMER1__START_GET) && defined(HAS_HW_DIAG_CTIMER1_ENABLE)
/** Fills-in an array of an CTIMER1 HW registers data with the current state
 *
 * all CTIMER1 HW configuaration is saved
 *
 * @param[in, out] buf   specifies an array to write into
 */
STATIC INLINE void GetCtimer1DataHwDiagFootprint(uint16_t buf[])
{
    uint16_t buf_offset = 0;
    buf[buf_offset] = IO_HOST(CTIMER1, TREGB);
    ++buf_offset;
    buf[buf_offset] = IO_HOST(CTIMER1, TREGA);
    ++buf_offset;
    buf[buf_offset] = IO_HOST(CTIMER1, START);
}

/** Returns the checksum for the trimming values' set
 * @return checksum
 */
STATIC INLINE uint16_t GetCtimer1DataHwDiagFootprintCs(void)
{
    uint16_t buf[HW_DIAG_CTIMER1_DATA_SIZE];
    GetCtimer1DataHwDiagFootprint(buf);
    return HW_DIAG_GetDataCs(buf, HW_DIAG_CTIMER1_DATA_SIZE);
}

/* Additional assert assuring the fields are properly grouped. If ok, IO_MASK() will not fail */
ASSERT( IO_MASK(CTIMER1, START, STOP, EDGA, EDGB, PWMI, POL, OVRA, OVRB, ENCMP, MODE, DIV) != 0u );

#endif /* CTIMER1 */

#endif /* CTIMER_LIB_DIAG_INLINE_IMPL_H */

