/**
 * @file
 * @brief Fast math
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
 * @ingroup fast_math
 *
 * @details Inlined functions definition
 */

#ifndef FM_INLINE_IMPL_H_
#define FM_INLINE_IMPL_H_

#ifndef FM_H_
#error "The file can't be included directly but via fm.h"
/* Code documentation */
#include "fm.h"
#endif /* FM_H_ */

#if FM_FULL_SIN_COS_LUT == 1
  #include "fm_sin_cos_2_5k_lut_inline_impl.h"
#else /* FM_FULL_SIN_COS_LUT */
  #include "fm_sin_cos_0_5k_lut_inline_impl.h"
#endif /* FM_FULL_SIN_COS_LUT */
#include "fm_atan2.h"
#include "fm_hypotenuse.h"

/* ---------------------------------------------
 * Global Variable Declarations
 * --------------------------------------------- */

/** Arctangent table for \f$theta=atan(y/x)\f$ where \f$y/x=[0,1)\f$.
 * The table on this range consists of 64 elements.
 * For table generation the input value \f$(y/x)\f$ is provided with \f$+1/64/2\f$ offset.
 * The 'theta' values are scaled up to 0x10000 which represents \f$2\pi\f$.
 */
extern const int16_t fm_AtanTable_256[FM_ATAN_TABLE_SIZE];

#if FM_FULL_SIN_COS_LUT != 1
/** Table with one quadrant of sine.
 * Resolution of the table is \f$\pi\f$/1024\f$.
 * Table is generated for one quadrant of sine.
 * For table generation the input value is provided with \f$+2\pi/1024/2\f$ offset.
 * The table values are represented as 16 bit signed fraction, i.e. Q15 format
 * (\f$\pm\f$ 0x7FFF represents \f$\pm\f$ 1).
 */
extern const int16_t fm_SinTableQ1_0_5k[FM_SIN_TABLE_0_5_SIZE];

#else /* FM_FULL_SIN_COS_LUT */

/** Size of flash interrupt vectors */
/** Table with full period sine and cosine.
 * Resolution of the table is \f$2\pi/1024\f$.
 * Table is generated for five quadrants.
 * Sine full period table starts from index 1.
 * Cosine full period table starts from index 256+1.
 * For table generation the input value is provided with \f$+2\pi/1024/2\f$ offset.
 * The table values are represented as 16 bit signed fraction, i.e. Q15 format
 * (\f$\pm\f$ 0x7FFF represents \f$\pm\f$ 1).
 */
extern const int16_t fm_SinCosTable_2_5k[FM_SIN_TABLE_2_5_SIZE];

#endif /* FM_FULL_SIN_COS_LUT */




/* ---------------------------------------------
 * Inlined Function Definitions
 * --------------------------------------------- */

#if FM_FULL_SIN_COS_LUT != 1

STATIC INLINE int16_t fm_SinU16DirectLutInlined(uint16_t theta)
{
    return fm_SinHelperDirectLutInlined_0_5k_lut(theta);
}

STATIC INLINE int16_t fm_SinI16DirectLutInlined(int16_t theta)
{
    return fm_SinHelperDirectLutInlined_0_5k_lut((uint16_t)theta);
}

STATIC INLINE int16_t fm_CosU16DirectLutInlined(uint16_t theta)
{
    return fm_SinHelperDirectLutInlined_0_5k_lut(theta + 0x4000U);
}

STATIC INLINE int16_t fm_CosI16DirectLutInlined(int16_t theta)
{
    return fm_SinHelperDirectLutInlined_0_5k_lut((uint16_t)theta + 0x4000U);
}

STATIC INLINE int16_t fm_SinU16InterpolationInlined(uint16_t theta)
{
    return fm_SinHelperInterpolationInlined_0_5k_lut(theta);
}

STATIC INLINE int16_t fm_SinI16InterpolationInlined(int16_t theta)
{
    return fm_SinHelperInterpolationInlined_0_5k_lut((uint16_t)theta);
}

STATIC INLINE int16_t fm_CosU16InterpolationInlined(uint16_t theta)
{
    return fm_SinHelperInterpolationInlined_0_5k_lut(theta + 0x4000U);
}

STATIC INLINE int16_t fm_CosI16InterpolationInlined(int16_t theta)
{
    return fm_SinHelperInterpolationInlined_0_5k_lut((uint16_t)theta + 0x4000U);
}

#else /* FM_FULL_SIN_COS_LUT */

STATIC INLINE int16_t fm_SinU16DirectLutInlined(uint16_t theta)
{
    return fm_SinHelperDirectLutInlined_2_5k_lut(theta);
}

STATIC INLINE int16_t fm_SinI16DirectLutInlined(int16_t theta)
{
    return fm_SinHelperDirectLutInlined_2_5k_lut((uint16_t)theta);
}

STATIC INLINE int16_t fm_CosU16DirectLutInlined(uint16_t theta)
{
    return fm_CosHelperDirectLutInlined_2_5k_lut(theta);
}

STATIC INLINE int16_t fm_CosI16DirectLutInlined(int16_t theta)
{
    return fm_CosHelperDirectLutInlined_2_5k_lut((uint16_t)theta);
}

STATIC INLINE int16_t fm_SinU16InterpolationInlined(uint16_t theta)
{
    return fm_SinHelperInterpolationInlined_2_5k_lut(theta);
}

STATIC INLINE int16_t fm_SinI16InterpolationInlined(int16_t theta)
{
    return fm_SinHelperInterpolationInlined_2_5k_lut((uint16_t)theta);
}

STATIC INLINE int16_t fm_CosU16InterpolationInlined(uint16_t theta)
{
    return fm_CosHelperInterpolationInlined_2_5k_lut(theta);
}

STATIC INLINE int16_t fm_CosI16InterpolationInlined(int16_t theta)
{
    return fm_CosHelperInterpolationInlined_2_5k_lut((uint16_t)theta);
}

#endif /* FM_FULL_SIN_COS_LUT */

STATIC INLINE int16_t fm_Atan2U16DirectLutInlined(uint16_t y, uint16_t x)
{
    return fm_Atan2HelperDirectLutInlined(y, x);
}

STATIC INLINE int16_t fm_Atan2U16InterpolationInlined(uint16_t y, uint16_t x)
{
    return fm_Atan2HelperInterpolationInlined(y, x);
}

#endif /* FM_INLINE_IMPL_H_ */

