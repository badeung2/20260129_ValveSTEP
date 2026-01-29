/**
 * @file
 * @brief Common library for the hw_diag module ITC HW
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
 * @brief Common library for the hw_diag module ITC HW
 * @details
 * @{
 */

#ifndef HW_DIAG_ITC_INLINE_IMPL_H
#define HW_DIAG_ITC_INLINE_IMPL_H

#include <stdint.h>
#include "io.h"
#include "compiler_abstraction.h"
#include "sys_tools.h"
#include "hw_diag.h"

/** Fills-in an array of interrupts data with the current state
 *
 * all ITC masks and priorities are saved
 *
 * @param[in, out] buf   specifies an array to write into
 */
STATIC INLINE void GetItcDataHwDiagFootprint(uint16_t buf[])
{
    uint16_t buf_offset = 0;
    buf[buf_offset] = IO_HOST(MLX16, ITC_MASK0);
    ++buf_offset;
    buf[buf_offset] = IO_HOST(MLX16, ITC_PRIO0);
#ifdef IO_MLX16__ITC_MASK1_GET
    ++buf_offset;
    buf[buf_offset] = IO_HOST(MLX16, ITC_MASK1);
    ++buf_offset;
    buf[buf_offset] = IO_HOST(MLX16, ITC_PRIO1);
#endif /* MASK1 and PRIO1 */
#ifdef IO_MLX16__ITC_MASK2_GET
    ++buf_offset;
    buf[buf_offset] = IO_HOST(MLX16, ITC_MASK2);
    ++buf_offset;
    buf[buf_offset] = IO_HOST(MLX16, ITC_PRIO2);
#endif /* MASK2 and PRIO2 */
#ifdef IO_MLX16__ITC_MASK3_GET
    ++buf_offset;
    buf[buf_offset] = IO_HOST(MLX16, ITC_MASK3);
    ++buf_offset;
    buf[buf_offset] = IO_HOST(MLX16, ITC_PRIO3);
#endif /* MASK3 and PRIO3 */
#ifdef IO_MLX16__ITC_MASK4_GET
    ++buf_offset;
    buf[buf_offset] = IO_HOST(MLX16, ITC_MASK4);
    ++buf_offset;
    buf[buf_offset] = IO_HOST(MLX16, ITC_PRIO4);
#endif /* MASK4 and PRIO4 */
#ifdef IO_MLX16__ITC_MASK5_GET
    ++buf_offset;
    buf[buf_offset] = IO_HOST(MLX16, ITC_MASK5);
    ++buf_offset;
    buf[buf_offset] = IO_HOST(MLX16, ITC_PRIO5);
#endif /* MASK5 and PRIO5 */
#ifdef IO_MLX16__ITC_MASK6_GET
    ++buf_offset;
    buf[buf_offset] = IO_HOST(MLX16, ITC_MASK6);
    ++buf_offset;
    buf[buf_offset] = IO_HOST(MLX16, ITC_PRIO6);
#endif /* MASK6 and PRIO6 */
#ifdef IO_MLX16__ITC_MASK7_GET
    ++buf_offset;
    buf[buf_offset] = IO_HOST(MLX16, ITC_MASK7);
    ++buf_offset;
    buf[buf_offset] = IO_HOST(MLX16, ITC_PRIO7);
#endif /* MASK7 and PRIO7 */
#ifdef IO_MLX16__ITC_MASK8_GET
    ++buf_offset;
    buf[buf_offset] = IO_HOST(MLX16, ITC_MASK8);
    ++buf_offset;
    buf[buf_offset] = IO_HOST(MLX16, ITC_PRIO8);
#endif /* MASK8 and PRIO8 */
}


/** Returns the checksum for the trimming values' set
 * @return checksum
 */
STATIC INLINE uint16_t GetItcDataHwDiagFootprintCs(void)
{
    uint16_t buf[HW_DIAG_ITC_DATA_SIZE];
    GetItcDataHwDiagFootprint(buf);
    return HW_DIAG_GetDataCs(buf, HW_DIAG_ITC_DATA_SIZE);
}

#endif /* HW_DIAG_ITC_INLINE_IMPL_H */
/// @}

