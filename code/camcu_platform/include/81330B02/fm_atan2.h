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

#ifndef FM_ATAN2_H_
#define FM_ATAN2_H_

#include "fm.h"

/** Atan2I16 optimized assembly implementation
 *
 * @param[in]  y  Y coordinate
 * @param[in]  x  X coordinate
   @return     The output angle, as 16 bit signed fraction of \f$2\pi\f$.
               32768(-1->-\f$\pi\f$) < out < 32768(1->\f$\pi\f$)
 */
extern int16_t p_atan2I16(uint16_t y, uint16_t x);

/* Rename the function with an alias to be aligned with the other function names */
/** Atan2I16 optimized assembly implementation
 *
 * @param[in]  y  Y coordinate
 * @param[in]  x  X coordinate
   @return     The output angle, as 16 bit signed fraction of \f$2\pi\f$.
               32768(-1->-\f$\pi\f$) < out < 32768(1->\f$\pi\f$)
 */
#define fm_Atan2I16 p_atan2I16


/* ---------------------------------------------
 * Function Definitions
 * --------------------------------------------- */

/** Non-inlined helper function for taking arctangent value from LUT (::fm_AtanTable_256) directly.
 * @param[in]  y  Y coordinate
 * @param[in]  x  X coordinate
 * @return     LUT value in Q16 representation
 */
STATIC INLINE int16_t fm_Atan2HelperDirectLutInlined(uint16_t y, uint16_t x)
{
    int16_t result;
    uint16_t modified_y;
    __asm__ __volatile__ (
        "cmp Y, A\n\t"
        "jul _Q2_%=\n\t"
        "je _PI_DIV_4_%=\n\t"
        /* assumed "jug _Q1_%=\n\t" */

        "_Q1_%=:\t"
        "swap YA\n\t"
        "lod X, A\n\t"
        "divu YA, X\n\t"
        "divu YA, X\n\t"
        "lod YL, AH\n\t"
        "lod YH, #0\n\t"
        "lsl Y\n\t"
        "add Y, #(_fm_AtanTable_256+2)\n\t"
        "lod A, [Y]\n\t"
        "jmp _END_%=\n\t"

        "_PI_DIV_4_%=:\t"
        "cmp A, #0\n\t"
        "jz _ZERO_%=\n\t"
        "lod A, #0x2000\n\t"
        "jmp _END_%=\n\t"

        "_ZERO_%=:\t"
        "lod A, #0\n\t"
        "jmp _END_%=\n\t"

        "_Q2_%=:\t"
        "lod X, A\n\t"
        "divu YA, X\n\t"
        "divu YA, X\n\t"
        "lod YL, AH\n\t"
        "lod YH, #0\n\t"
        "lsl Y\n\t"
        "add Y, #(_fm_AtanTable_256+2)\n\t"
        "lod X, [Y]\n\t"
        "lod A, #0x4000\n\t"
        "sub A, X\n\t"
        /* continue with _END_ */

        "_END_%=:\t"
        : "=a" (result), "=y" (modified_y)
        : "y" (x), "a" (y)
        : "X"
        );
    return result;
}

/** Non-inlined helper function for taking arctangent value from LUT (::fm_AtanTable_256) and doing interpolation.
 * @param[in]  y  Y coordinate
 * @param[in]  x  X coordinate
 * @return     value in Q16 representation resulted from interpolation of LUT
 */
STATIC INLINE int16_t fm_Atan2HelperInterpolationInlined(uint16_t y, uint16_t x)
{
    int16_t result;
    uint16_t modified_y;
    __asm__ __volatile__ (
        "cmp Y, A\n\t"
        "jul _Q2_%=\n\t"
        "je _PI_DIV_4_%=\n\t"
        /* assumed "jug _Q1_%=\n\t" */

        "_Q1_%=:\t"
        "swap YA\n\t"
        "lod X, A\n\t"
        "divu YA, X\n\t"
        "divu YA, X\n\t"
        "mov Y, #0\n\t"
        "add YA, #128\n\t"
        "lsl YA, #8\n\t"
        "lod AL, AH\n\t"
        "asl Y\n\t"
        "add Y, #(_fm_AtanTable_256)\n\t"
        "lod X, [Y+2]\n\t"
        "sub X, [Y]\n\t"
        "usex A\n\t"
        "muls A, A, X\n\t"
        "add A, #128\n\t"
        "lod AL, AH\n\t"
        "lod AH, #0\n\t"
        "add A, [Y]\n\t"
        "jmp _END_%=\n\t"

        "_PI_DIV_4_%=:\t"
        "cmp A, #0\n\t"
        "jz _ZERO_%=\n\t"
        "lod A, #0x2000\n\t"
        "jmp _END_%=\n\t"

        "_ZERO_%=:\t"
        "lod A, #0\n\t"
        "jmp _END_%=\n\t"

        "_Q2_%=:\t"
        "lod X, A\n\t"
        "divu YA, X\n\t"
        "divu YA, X\n\t"
        "mov Y, #0\n\t"
        "add YA, #128\n\t"
        "lsl YA, #8\n\t"
        "lod AL, AH\n\t"
        "asl Y\n\t"
        "add Y, #(_fm_AtanTable_256)\n\t"
        "lod X, [Y+2]\n\t"
        "sub X, [Y]\n\t"
        "usex A\n\t"
        "muls A, A, X\n\t"
        "add A, #128\n\t"
        "lod AL, AH\n\t"
        "lod AH, #0\n\t"
        "lod X, A\n\t"
        "add X, [Y]\n\t"
        "lod A, #0x4000\n\t"
        "sub A, X\n\t"
        /* continue with _END_ */

        "_END_%=:\t"
        : "=a" (result), "=y" (modified_y)
        : "y" (x), "a" (y)
        : "X"
        );
    return result;
}

STATIC INLINE int16_t fm_Atan2I16DirectLutInlined(int16_t y, int16_t x)
{
    return p_atan2I16(y,x);
}

STATIC INLINE int16_t fm_Atan2I16InterpolationInlined(int16_t y, int16_t x)
{
    int16_t retVal;
    if (y < 0) {
        if (x < 0) {
            uint16_t result_u16 =
                (uint16_t)fm_Atan2HelperInterpolationInlined((uint16_t)(-y), (uint16_t)(-x)) - 0x8000u; /* x - pi */
            retVal = (int16_t)result_u16; /* x - pi */
        } else {
            retVal = -fm_Atan2HelperInterpolationInlined((uint16_t)(-y), (uint16_t)x);
        }
    } else {
        if (x < 0) {
            uint16_t result_u16 = 0x8000u - (uint16_t)fm_Atan2HelperInterpolationInlined((uint16_t)y, (uint16_t)(-x)); /* pi - x */
            retVal = (int16_t)result_u16; /* pi - x */
        } else {
            retVal = fm_Atan2HelperInterpolationInlined((uint16_t)y, (uint16_t)x);
        }
    }
    return retVal;
}

#endif /* FM_ATAN2_H_ */

