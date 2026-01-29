/** @file
 *  @brief Header file for 32 bit left shift implementations
 *  @internal
 *
 *  @copyright (C) Melexis N.V.
 * git flash d0014c23
 *
 *  Melexis N.V. is supplying this code for use with Melexis N.V. processor based microcontrollers only.
 *
 *  THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY,
 *  INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.  MELEXIS N.V. SHALL NOT IN ANY CIRCUMSTANCES,
 *  BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *  @endinternal
 *
 *  This header file provides 32 bit shift implementations for unsigned integers.
 *  The implementations here mean to compensate for inefficient compiler implementations, and
 *  are meant to be included in the compiler at some point.
 *
 *  The optimized functions in this header file can be used with any mlx16 architecture
 *  and optimization setting, but are specifically optimized for:
 *
 *  - O3 optimization setting
 *  - mlx16-8 and mlx16-e8 architectures (the others use coprocessor for shifting)
 *
 *  At this time, only optimizations are performed for shifts of 12, 13 and 14
 *  places. The other shift operations are handled well by the compiler, but do
 *  get their own function here to enable writing consistent code.
 *
 *  @ingroup mathlib
 *
 *  @details
 */

/** \addtogroup shift_32
 *  @{
 */

#ifndef SL32_H_
#define SL32_H_

#include <mlx16_cfg.h>
#include "mathlib.h"

#ifndef UNITTEST
/* returns a << n */
static __inline__ uint32_t _lsl32_0  (uint32_t a) __attribute__ ((always_inline));
static __inline__ uint32_t _lsl32_1  (uint32_t a) __attribute__ ((always_inline));
static __inline__ uint32_t _lsl32_2  (uint32_t a) __attribute__ ((always_inline));
static __inline__ uint32_t _lsl32_3  (uint32_t a) __attribute__ ((always_inline));
static __inline__ uint32_t _lsl32_4  (uint32_t a) __attribute__ ((always_inline));
static __inline__ uint32_t _lsl32_5  (uint32_t a) __attribute__ ((always_inline));
static __inline__ uint32_t _lsl32_6  (uint32_t a) __attribute__ ((always_inline));
static __inline__ uint32_t _lsl32_7  (uint32_t a) __attribute__ ((always_inline));
static __inline__ uint32_t _lsl32_8  (uint32_t a) __attribute__ ((always_inline));
static __inline__ uint32_t _lsl32_9  (uint32_t a) __attribute__ ((always_inline));
static __inline__ uint32_t _lsl32_10 (uint32_t a) __attribute__ ((always_inline));
static __inline__ uint32_t _lsl32_11 (uint32_t a) __attribute__ ((always_inline));
static __inline__ uint32_t _lsl32_12 (uint32_t a) __attribute__ ((always_inline));
static __inline__ uint32_t _lsl32_13 (uint32_t a) __attribute__ ((always_inline));
static __inline__ uint32_t _lsl32_14 (uint32_t a) __attribute__ ((always_inline));
static __inline__ uint32_t _lsl32_15 (uint32_t a) __attribute__ ((always_inline));
static __inline__ uint32_t _lsl32_16 (uint32_t a) __attribute__ ((always_inline));
#endif


/** Logical shift left by 0 places (32-bit)
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ uint32_t _lsl32_0  (uint32_t a)
{
    return a;
}

/** Logical shift left by 1 places (32-bit)
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ uint32_t _lsl32_1  (uint32_t a)
{
    return a << 1;
}

/** Logical shift left by 2 places (32-bit)
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ uint32_t _lsl32_2  (uint32_t a)
{
    return a << 2;
}

/** Logical shift left by 3 places (32-bit)
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ uint32_t _lsl32_3  (uint32_t a)
{
    return a << 3;
}

/** Logical shift left by 4 places (32-bit)
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ uint32_t _lsl32_4  (uint32_t a)
{
    return a << 4;
}

/** Logical shift left by 5 places (32-bit)
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ uint32_t _lsl32_5  (uint32_t a)
{
    return a << 5;
}

/** Logical shift left by 6 places (32-bit)
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ uint32_t _lsl32_6  (uint32_t a)
{
    return a << 6;
}

/** Logical shift left by 7 places (32-bit)
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ uint32_t _lsl32_7  (uint32_t a)
{
    return a << 7;
}

/** Logical shift left by 8 places (32-bit)
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ uint32_t _lsl32_8  (uint32_t a)
{
    return a << 8;
}

/** Logical shift left by 9 places (32-bit)
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ uint32_t _lsl32_9  (uint32_t a)
{
    return a << 9;
}

/** Logical shift left by 10 places (32-bit)
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ uint32_t _lsl32_10 (uint32_t a)
{
    return a << 10;
}

/** Logical shift left by 11 places (32-bit)
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ uint32_t _lsl32_11 (uint32_t a)
{
    return a << 11;
}

/** Logical shift left by 12 places (32-bit)
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ uint32_t _lsl32_12 (uint32_t a)
{
#if defined(__MLX16__) && !defined(HAS_MLX16_COPROCESSOR) && (!defined(__COVERITY__) && !defined(__POLYSPACE__)) && \
    !defined(MLX_FORCE_C_IMPLEMENTATION)
    __asm__ (
        ";; << 12\n\t"
        "and Y, #15\n\t"
        "rr Y, #2\t\t; rr 4 <=> rl 12\n\t"
        "rr Y, #2\n\t"
        "rr A, #2\t\t; rr 4 <=> rl 12\n\t"
        "rr A, #2\n\t"
        "xor Y, A\t\t; store A[11:0] in Y; Y[15:12] is temporary xor'ed with A[15:12]\n\t"
        "and A, #0xF000\n\t"
        "xor Y, A\t\t; xor Y[15:12] with A[15:12] again to restore original content"
        : "=b" (a)
        : "0" (a)
        );

    return a;
#else
    __asm__ ("; dummy asm code to attract Coverity or Polyspace");
    return a << 12;
#endif
}

/** Logical shift left by 13 places (32-bit)
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ uint32_t _lsl32_13 (uint32_t a)
{
#if defined(__MLX16__) && !defined(HAS_MLX16_COPROCESSOR) && (!defined(__COVERITY__) && !defined(__POLYSPACE__)) && \
    !defined(MLX_FORCE_C_IMPLEMENTATION)
    __asm__ (
        ";; << 13\n\t"
        "and Y, #7\n\t"
        "rr Y, #2\t\t; rr 3 <=> rl 13\n\t"
        "rr Y, #1\n\t"
        "rr A, #2\t\t; rr 3 <=> rl 13\n\t"
        "rr A, #1\n\t"
        "xor Y, A\t\t; store A[12:0] in Y; Y[15:13] is temporary xor'ed with A[15:13]\n\t"
        "and A, #0xE000\n\t"
        "xor Y, A\t\t; xor Y[15:13] with A[15:13] again to restore original content"
        : "=b" (a)
        : "0" (a)
        );

    return a;
#else
    __asm__ ("; dummy asm code to attract Coverity or Polyspace");
    return a << 13;
#endif
}

/** Logical shift left by 14 places (32-bit)
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ uint32_t _lsl32_14 (uint32_t a)
{
#if defined(__MLX16__) && !defined(HAS_MLX16_COPROCESSOR) && (!defined(__COVERITY__) && !defined(__POLYSPACE__)) && \
    !defined(MLX_FORCE_C_IMPLEMENTATION)
    __asm__ (
        ";; << 14\n\t"
        "and Y, #3\n\t"
        "rr Y, #2\t\t; rr 2 <=> rl 14\n\t"
        "rr A, #2\t\t; rr 2 <=> rl 14\n\t"
        "xor Y, A\t\t; store A[13:0] in Y; Y[15:14] is temporary xor'ed with A[15:14]\n\t"
        "and A, #0xC000\n\t"
        "xor Y, A\t\t; xor Y[15:14] with A[15:14] again to restore original content"
        : "=b" (a)
        : "0" (a)
        );

    return a;
#else
    __asm__ ("; dummy asm code to attract Coverity or Polyspace");
    return a << 14;
#endif
}

/** Logical shift left by 15 places (32-bit)
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ uint32_t _lsl32_15 (uint32_t a)
{
    return a << 15;
}

/** Logical shift left by 16 places (32-bit)
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ uint32_t _lsl32_16 (uint32_t a)
{
    return (a << 16);
}

#endif /* SL32_H_ */

/** @}*/