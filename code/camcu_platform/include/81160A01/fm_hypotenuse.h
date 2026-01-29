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

#ifndef FM_HYPOTENUSE_H_
#define FM_HYPOTENUSE_H_

#include "fm.h"


/* ---------------------------------------------
 * Function Definitions
 * --------------------------------------------- */

STATIC INLINE uint16_t fm_HypotenusePythagoreanInlined(int16_t a, int16_t b)
{
    uint16_t u16Result;
    uint16_t u16Garbish;
    __asm__ __volatile__ (
        "mov A, X\n\t"                                                          /* [1] */
        "muls D, A, A\n\t"                                                      /* [4] D = X^2 */
        "asl A\n\t"                                                             /* [1] */
        "subc A, A\n\t"                                                         /* [1] A = X >> 15 */
        "add X, A\n\t"                                                          /* [1] */
        "xor X, A\n\t"                                                          /* [1] X = |X| */
        "mov A, Y\n\t"                                                          /* [1] */
        "macs D, A, A\n\t"                                                      /* [6] D = X^2 + Y^2*/
        "asl A\n\t"                                                             /* [1] */
        "subc A, A\n\t"                                                         /* [1] A = Y >> 15 */
        "add Y, A\n\t"                                                          /* [1] */
        "xor A, Y\n\t"                                                          /* [1] A = |Y| */
        "add A, X\n\t"                                                          /* [1] A = |Y| + |X| */
        "cmp A, #1\n\t"                                                         /* [1] */
        "jule .+28\n\t"                                                         /* [1] */
        "mulu YA, A, #55705\n\t"                                                /* [5] Y = (|Y| + |X|) * 0.85 */
        "mov X, Y\n\t"                                                          /* [1] Sqrt = X = (|Y| + |X|) * 0.85 */
        "mov YA, D\n\t"                                                         /* [2] YA = X^2 + Y^2 */
        "divu YA, X\n\t"                                                        /* [8] */
        "divu YA, X\n\t"                                                        /* [8] A = (X^2 + Y^2) / Sqrt */
        "add X, A\n\t"                                                          /* [1] */
        "rrc X\n\t"                                                             /* [1] Sqrt = (Sqrt + A)/2 */
        "mov YA, D\n\t"                                                         /* [2] YA = X^2 + Y^2 */
        "divu YA, X\n\t"                                                        /* [8] */
        "divu YA, X\n\t"                                                        /* [8] A = (X^2 + Y^2) / Sqrt */
        "add A, X\n\t"                                                          /* [1] */
        "rrc A"                                                                 /* [1] Sqrt = (Sqrt + A)/2 */
        : "=a" (u16Result), "=y" (u16Garbish), "=x" (u16Garbish)
        : "x" (a), "y" (b)
        : "D"
        );
    return( u16Result );
}

STATIC INLINE uint32_t fm_HypotenuseAmaxBminInlined(uint16_t a, uint16_t b)
{
    uint16_t u16Max, u16Min;
    if (b > a) {
        u16Max = b;
        u16Min = a;
    } else {
        u16Max = a;
        u16Min = b;
    }
    uint32_t u16Z0 = (uint32_t)u16Max + (uint32_t)mulU16hi_U16byU16(u16Min, (uint16_t)((5UL * 0xFFFFUL) / 32U));
    uint32_t u16Z1 = (uint32_t)mulU16hi_U16byU16(u16Max, (uint16_t)((27UL * 0xFFFFUL) / 32U)) +
                     (uint32_t)mulU16hi_U16byU16(u16Min, (uint16_t)((71UL * 0xFFFFUL) / 128U));
    if (u16Z1 > u16Z0) {
        return u16Z1;
    } else {
        return u16Z0;
    }
}

/** Unsigned integer division ( 32 = 32 / 16 )
 *
 * @param[in]  n     Unsigned 32-bit dividend
 * @param[in]  d     Unsigned 16-bit divisor
 *
 * @return     Unsigned 32-bit quotient
 */
STATIC INLINE uint32_t fm_DivU32_U32byU16(uint32_t n, uint16_t d);
STATIC INLINE uint32_t fm_DivU32_U32byU16(uint32_t n, uint16_t d)
{
    uint32_t u32Result;

    __asm__ __volatile__ (
        "push A\n\t"
        "lod  A, Y\n\t"
        "lod  Y, #0\n\t"
        "divu YA, X\n\t"
        "divu YA, X\n\t"
        "push A\n\t"
        "lod  A, [S-4]\n\t"
        "divu YA, X\n\t"
        "divu YA, X\n\t"
        "pop  Y\n\t"
        "dec  S, #2"
        : "=b" (u32Result)
        : "b" (n), "x" (d)
        );
    return( u32Result);
}

STATIC INLINE uint32_t fm_HypotenuseTrigonometryFullAccuracyInlined(uint16_t a, uint16_t b)
{
    uint16_t u16_a, u16_b;
    if (a < b) {
        u16_b = a;
        u16_a = b;
    } else {
        u16_a = a;
        u16_b = b;
    }
    int16_t theta = fm_Atan2U16InterpolationInlined(u16_a, u16_b);
    uint16_t sin_theta;
    sin_theta = (uint16_t)fm_SinI16InterpolationInlined(theta);
    uint32_t result = fm_DivU32_U32byU16((u16_a * 0x7FFFUL), sin_theta);
    return result;
}

STATIC INLINE uint32_t fm_HypotenuseTrigonometryReducedAccuracyInlined(uint16_t a, uint16_t b)
{
    uint16_t u16_a, u16_b;
    if (a < b) {
        u16_b = a;
        u16_a = b;
    } else {
        u16_a = a;
        u16_b = b;
    }
    int16_t theta = fm_Atan2U16DirectLutInlined(u16_a, u16_b);
    uint16_t sin_theta;
    sin_theta = (uint16_t)fm_SinI16DirectLutInlined(theta);
    uint32_t result = fm_DivU32_U32byU16((u16_a * 0x7FFFUL), sin_theta);
    return result;
}

#endif /* FM_HYPOTENUSE_H_ */

