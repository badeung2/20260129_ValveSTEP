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
 * @details
 *
 * @ingroup fast_math
 *
 * @details Functions definition which enable both inlined and non-inlined versions
 *
 */

#ifndef FM_SIN_COS_2_5K_LUT_H_
#define FM_SIN_COS_2_5K_LUT_H_

#include "fm.h"

#if FM_FULL_SIN_COS_LUT == 1


/* ---------------------------------------------
 * Function Definitions
 * --------------------------------------------- */

/** Inlined helper function for taking sine value from LUT directly.
 * The LUT - ::fm_SinCosTable_2_5k) is generated for full period.
 * @param[in]  theta  angle in Q16 representation
 * @return     LUT value in Q15 representation
 */
STATIC INLINE int16_t fm_SinHelperDirectLutInlined_2_5k_lut(uint16_t theta)
{
    int16_t result;
    uint16_t cl_y; /* clobbering of the Y register (is modified during calculations) */

    __asm__ __volatile__ (
        "lsr YA, #6\n\t"
        "lsl Y\n\t"
        "add Y, #(_fm_SinCosTable_2_5k+2)\n\t"
        "lod A, [Y]"
        : "=a" (result), "=y" (cl_y)
        : "y" (theta)
        );
    return result;
}

/** Inlined helper function for taking cosine value from LUT directly.
 * The LUT - ::fm_SinCosTable_2_5k) is generated for full period.
 * @param[in]  theta  angle in Q16 representation
 * @return     LUT value in Q15 representation
 */
STATIC INLINE int16_t fm_CosHelperDirectLutInlined_2_5k_lut(uint16_t theta)
{
    int16_t result;
    uint16_t cl_y; /* clobbering of the Y register (is modified during calculations) */

    __asm__ __volatile__ (
        "lsr YA, #6\n\t"
        "lsl Y\n\t"
        "add Y, #(_fm_SinCosTable_2_5k+2+512)\n\t"
        "lod A, [Y]"
        : "=a" (result), "=y" (cl_y)
        : "y" (theta)
        );
    return result;
}

/** Inlined helper function for taking sine value from LUT and doing interpolation.
 * The LUT - ::fm_SinCosTable_2_5k) is generated for full period.
 * @param[in]  theta  angle in Q16 representation
 * @return     LUT value in Q15 representation
 */
STATIC INLINE int16_t fm_SinHelperInterpolationInlined_2_5k_lut(uint16_t theta)
{
    int16_t result;
    __asm__ __volatile__ (
        "add A, #32\n\t"
        "lod Y, #0\n\t"
        "rl YA, #10\n\t"
        "lsl Y\n\t"
        "add Y, #(_fm_SinCosTable_2_5k)\n\t"
        "lod X, [Y+2]\n\t"
        "sub X, [Y]\n\t"
        "swap A\n\t"
        "asr A, #2\n\t"
        "muls D, A, X\n\t"
        "add D, #32\n\t"
        "asr D, #6\n\t"
        "mov A, Dl\n\t"
        "add A, [Y]"
        : "=a" (result)
        : "a" (theta)
        : "X", "Y", "D"
        );
    return result;
}

/** Inlined helper function for taking cosine value from LUT and doing interpolation.
 * The LUT - ::fm_SinCosTable_2_5k) is generated for full period.
 * @param[in]  theta  angle in Q16 representation
 * @return     LUT value in Q15 representation
 */
STATIC INLINE int16_t fm_CosHelperInterpolationInlined_2_5k_lut(uint16_t theta)
{
    int16_t result;

    __asm__ __volatile__ (
        "add A, #32\n\t"
        "lod Y, #0\n\t"
        "rl YA, #10\n\t"
        "lsl Y\n\t"
        "add Y, #(_fm_SinCosTable_2_5k+512)\n\t"
        "lod X, [Y+2]\n\t"
        "sub X, [Y]\n\t"
        "swap A\n\t"
        "asr A, #2\n\t"
        "muls D, A, X\n\t"
        "add D, #32\n\t"
        "asr D, #6\n\t"
        "mov A, Dl\n\t"
        "add A, [Y]"
        : "=a" (result)
        : "a" (theta)
        : "X", "Y", "D"
        );
    return result;
}

#endif /* FM_FULL_SIN_COS_LUT */

#endif /* FM_SIN_COS_2_5K_LUT_H_ */

