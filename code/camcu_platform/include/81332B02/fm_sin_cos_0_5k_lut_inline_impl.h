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

#ifndef FM_SIN_COS_0_5K_LUT_H_
#define FM_SIN_COS_0_5K_LUT_H_

#include "fm.h"

#if FM_FULL_SIN_COS_LUT != 1


/* ---------------------------------------------
 * Function Definitions
 * --------------------------------------------- */

STATIC INLINE int16_t fm_SinHelperDirectLutInlined_0_5k_lut(uint16_t theta)
{
    int16_t result;
    __asm__ __volatile__ (
        "lod Y, A\n\t"
        "lsr YA, #6\n\t"
        "lod YH, #0\n\t"
        "lsl Y\n\t"
        "lod X, #(_fm_SinTableQ1_0_5k+2+510)\n\t"
        "and AH, #0x3\n\t"
        "jz _Q1_%=\n\t"
        "cmp AH, #0x2\n\t"
        "je _Q3_%=\n\t"
        "jug _Q4_%=\n\t"
        /* assumed "jul _Q2_%=\n\t" */

        "_Q2_%=:\t"
        "sub X, Y\n\t"
        "lod A, [X]\n\t"
        "jmp _END_%=\n\t"

        "_Q1_%=:\t"
        "add Y, #(_fm_SinTableQ1_0_5k+2)\n\t"
        "lod A, [Y]\n\t"
        "jmp _END_%=\n\t"

        "_Q3_%=:\t"
        "add Y, #(_fm_SinTableQ1_0_5k+2)\n\t"
        "lod A, [Y]\n\t"
        "neg A\n\t"
        "jmp _END_%=\n\t"

        "_Q4_%=:\t"
        "sub X, Y\n\t"
        "lod A, [X]\n\t"
        "neg A\n\t"
        "_END_%=:\t"
        : "=a" (result)
        : "a" (theta)
        : "Y", "X"
        );
    return result;
}

STATIC INLINE int16_t fm_SinHelperInterpolationInlined_0_5k_lut(uint16_t theta)
{
    int16_t result;
    __asm__ __volatile__ (
        "add A, #32\n\t"
        "lod Y, #0\n\t"
        "rl YA, #10\n\t"
        "mov AL, YH\n\t"
        "jz _Q1_%=\n\t"
        "lsl Y\n\t"
        "cmp AL, #0x2\n\t"
        "jug _Q4_%=\n\t"
        "je _Q3_%=\n\t"
        /* assumed "jul _Q2_%=\n\t" */

        "_Q2_%=:\t"
        "lod X, #(_fm_SinTableQ1_0_5k+512+512)\n\t"
        /* "sub Y, #256*1*2" */
        "sub X, Y\n\t"
        "lod Y, [X]\n\t"
        "sub Y, [X+2]\n\t"
        "mov AL, AH\n\t"
        "mov AH, #0\n\t"
        "asr A, #2\n\t"
        "muls D, A, Y\n\t"
        "add D, #32\n\t"
        "asr D, #6\n\t"
        "mov A, Dl\n\t"
        "add A, [X+2]\n\t"
        "jmp _END_%=\n\t"

        "_Q1_%=:\t"
        "lsl Y\n\t"
        "add Y, #(_fm_SinTableQ1_0_5k)\n\t"
        /* "sub Y, #256*0*2" */
        "lod X, [Y+2]\n\t"
        "sub X, [Y]\n\t"
        "mov AL, AH\n\t"
        "mov AH, #0\n\t"
        "asr A, #2\n\t"
        "muls D, A, X\n\t"
        "add D, #32\n\t"
        "asr D, #6\n\t"
        "mov A, Dl\n\t"
        "add A, [Y]\n\t"
        "jmp _END_%=\n\t"

        "_Q4_%=:\t"
        "lod X, #(_fm_SinTableQ1_0_5k+512+1536)\n\t"
        /* "sub Y, #256*3*2" */
        "sub X, Y\n\t"
        "lod Y, [X]\n\t"
        "sub Y, [X+2]\n\t"
        "mov AL, AH\n\t"
        "mov AH, #0\n\t"
        "asr A, #2\n\t"
        "muls D, A, Y\n\t"
        "add D, #32\n\t"
        "asr D, #6\n\t"
        "mov A, Dl\n\t"
        "add A, [X+2]\n\t"
        "neg A\n\t"
        "jmp _END_%=\n\t"

        "_Q3_%=:\t"
        "add Y, #(_fm_SinTableQ1_0_5k-1024)\n\t"
        /* "sub Y, #256*2*2" */
        "lod X, [Y+2]\n\t"
        "sub X, [Y]\n\t"
        "mov AL, AH\n\t"
        "mov AH, #0\n\t"
        "asr A, #2\n\t"
        "muls D, A, X\n\t"
        "add D, #32\n\t"
        "asr D, #6\n\t"
        "mov A, Dl\n\t"
        "add A, [Y]\n\t"
        "neg A\n\t"
        "_END_%=:\t"
        : "=a" (result)
        : "a" (theta)
        : "Y", "X", "D"
        );
    return result;
}

#endif /* FM_FULL_SIN_COS_LUT */

#endif /* FM_SIN_COS_0_5K_LUT_H_ */

