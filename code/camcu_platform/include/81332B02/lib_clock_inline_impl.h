/**
 * @file
 * @brief Clock library inline implementations
 * @internal
 *
 * @copyright (C) 2017-2020 Melexis N.V.
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
 * @ingroup clock
 *
 * @details Clock library inline implementations
 * @{
 */

#ifndef LIB_CLOCK_INLINE_IMPL_H
#define LIB_CLOCK_INLINE_IMPL_H

#include <stdint.h>
#include "io.h"
#include "mathlib.h"

STATIC INLINE void SetRcoAndAcSel(uint16_t rco, uint8_t AcSel)
{
    IO_SET(TRIM_RCO32M, TR_RCO32M_IN, rco);
    IO_SET(PORT_CLOCK_CTRL, AC_SEL, AcSel); /* Set the half-freq flag as it set */
}

#if !defined (HAS_NO_SYSTEM_CLOCK_ERROR_CALIBRATION)
STATIC INLINE uint16_t clock_CorrectPeriodWithClockError(uint16_t period, int16_t clock_error)
{
    int16_t temp = 2048 + clock_error;
    return (uint16_t)(mulU32_U16byU16(period, (uint16_t)temp) / 2048u);
}
#endif /* HAS_NO_SYSTEM_CLOCK_ERROR_CALIBRATION */

/// @}
#endif /* LIB_CLOCK_INLINE_IMPL_H */

