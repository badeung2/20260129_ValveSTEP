
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
 * @ingroup startup
 *
 * @brief Common library for the hw_diag module
 * @details
 * @{
 */

#ifndef HW_DIAG_TRIM_INLINE_IMPL_H
#define HW_DIAG_TRIM_INLINE_IMPL_H

#include <stdint.h>
#include "compiler_abstraction.h"
#include "io.h"
#include "hw_diag.h"

/** Fills-in an array of trimming data with the current state
 * @param[in, out] buf   specifies an array to write into
 */
STATIC INLINE void GetTrimDataHwDiagFootprint(uint16_t buf[])
{
    uint16_t buf_offset = 0;
#ifdef IO_TRIM_BG_BIAS__TR_BGA_GET
    buf[buf_offset] = IO_HOST(TRIM_BG_BIAS, TR_BGA);
#else
    buf[buf_offset] = IO_HOST(TRIM_BG_BIAS, PRE_TR_BGA);    /* Upload all 16bits of data into an IO TODO: make it MLX81116-compliant */
#endif /* IO_TRIM_BG_BIAS__TR_BGA_GET */

#ifdef IO_TRIM_VDD__TR_VDDA
    ++buf_offset;
    buf[buf_offset] = IO_HOST(TRIM_VDD, TR_VDDA);
#else
    ++buf_offset;
    buf[buf_offset] = IO_HOST(TRIM_VDD, PRE_TR_VDDA);
#endif /* IO_TRIM_VDD__TR_VDDA */

#ifdef IO_TRIM_RCO1M__TR_RCO1M_GET
    ++buf_offset;
    buf[buf_offset] = IO_HOST(TRIM_RCO1M, TR_RCO1M);
#elif defined(IO_TRIM_RCO1M_LIN__PRE_TR_RCO1M_GET)
    ++buf_offset;
    buf[buf_offset] = IO_HOST(TRIM_RCO1M_LIN, PRE_TR_RCO1M);
#else
    ++buf_offset;
    buf[buf_offset] = IO_HOST(TRIM_RCO1M, PRE_TR_RCO1M);
#endif /* IO_TRIM_RCO1M__TR_RCO1M_GET */

#ifndef HAS_HW_EC_ADC
    ++buf_offset;
    buf[buf_offset] = IO_HOST(ADC_SAR, TR_ADCREF1);
    ++buf_offset;
    buf[buf_offset] = IO_HOST(ADC_SAR, TR_ADCREF3);
#endif /* HAS_HW_EC_ADC */

#if !defined(DBG_USE_CHAR_OUTPUT) && defined(IO_TRIM_VDD_EXT__TR_VDDIO_GET)
    ++buf_offset;
    buf[buf_offset] = IO_HOST(TRIM_VDD_EXT, TR_VDDIO);
#endif /* Removed for FPGA as it doesn't do this setting, left for old IC legacy, as new IC does not have TRIM_VDD_EXT, TR_VDDIO trimmings */
}


/** Returns the checksum for the trimming values' set
 * @return checksum
 */
STATIC INLINE uint16_t GetTrimDataHwDiagFootprintCs(void)
{
    uint16_t buf[HW_DIAG_TRIM_DATA_SIZE];
    GetTrimDataHwDiagFootprint(buf);
    return HW_DIAG_GetDataCs(buf, HW_DIAG_TRIM_DATA_SIZE);
}

#endif /* HW_DIAG_TRIM_INLINE_IMPL_H */
/// @}
