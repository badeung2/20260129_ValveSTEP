/** @file
 *  @brief Header file for 32 bit right shift implementations
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
 *  This header file provides 32 bit shift implementations for signed and unsigned integers.
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

#ifndef SR32_H_
#define SR32_H_

/* only use in case of -O3, not -Os TBC */

#include "mathlib.h"
#include <mlx16_cfg.h>


#ifndef UNITTEST
/* returns a >> n */
static __inline__ uint32_t _lsr32_0  (uint32_t a) __attribute__ ((always_inline));
static __inline__ int32_t  _asr32_0  (int32_t a)  __attribute__ ((always_inline));
static __inline__ uint32_t _lsr32_1  (uint32_t a) __attribute__ ((always_inline));
static __inline__ int32_t  _asr32_1  (int32_t a)  __attribute__ ((always_inline));
static __inline__ uint32_t _lsr32_2  (uint32_t a) __attribute__ ((always_inline));
static __inline__ int32_t  _asr32_2  (int32_t a)  __attribute__ ((always_inline));
static __inline__ uint32_t _lsr32_3  (uint32_t a) __attribute__ ((always_inline));
static __inline__ int32_t  _asr32_3  (int32_t a)  __attribute__ ((always_inline));
static __inline__ uint32_t _lsr32_4  (uint32_t a) __attribute__ ((always_inline));
static __inline__ int32_t  _asr32_4  (int32_t a)  __attribute__ ((always_inline));
static __inline__ uint32_t _lsr32_5  (uint32_t a) __attribute__ ((always_inline));
static __inline__ int32_t  _asr32_5  (int32_t a)  __attribute__ ((always_inline));
static __inline__ uint32_t _lsr32_6  (uint32_t a) __attribute__ ((always_inline));
static __inline__ int32_t  _asr32_6  (int32_t a)  __attribute__ ((always_inline));
static __inline__ uint32_t _lsr32_7  (uint32_t a) __attribute__ ((always_inline));
static __inline__ int32_t  _asr32_7  (int32_t a)  __attribute__ ((always_inline));
static __inline__ uint32_t _lsr32_8  (uint32_t a) __attribute__ ((always_inline));
static __inline__ int32_t  _asr32_8  (int32_t a)  __attribute__ ((always_inline));
static __inline__ uint32_t _lsr32_9  (uint32_t a) __attribute__ ((always_inline));
static __inline__ int32_t  _asr32_9  (int32_t a)  __attribute__ ((always_inline));
static __inline__ uint32_t _lsr32_10 (uint32_t a) __attribute__ ((always_inline));
static __inline__ int32_t  _asr32_10 (int32_t a)  __attribute__ ((always_inline));
static __inline__ uint32_t _lsr32_11 (uint32_t a) __attribute__ ((always_inline));
static __inline__ int32_t  _asr32_11 (int32_t a)  __attribute__ ((always_inline));
static __inline__ uint32_t _lsr32_12 (uint32_t a) __attribute__ ((always_inline));
static __inline__ int32_t  _asr32_12 (int32_t a)  __attribute__ ((always_inline));
static __inline__ uint32_t _lsr32_13 (uint32_t a) __attribute__ ((always_inline));
static __inline__ int32_t  _asr32_13 (int32_t a)  __attribute__ ((always_inline));
static __inline__ uint32_t _lsr32_14 (uint32_t a) __attribute__ ((always_inline));
static __inline__ int32_t  _asr32_14 (int32_t a)  __attribute__ ((always_inline));
static __inline__ uint32_t _lsr32_15 (uint32_t a) __attribute__ ((always_inline));
static __inline__ int32_t  _asr32_15 (int32_t a)  __attribute__ ((always_inline));
static __inline__ uint32_t _lsr32_16 (uint32_t a) __attribute__ ((always_inline));
static __inline__ int32_t  _asr32_16 (int32_t a)  __attribute__ ((always_inline));
#endif /* UNITTEST */


/** Logical shift right by 0 places (32-bit)
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ uint32_t _lsr32_0  (uint32_t a)
{
    return a;
}

/** Arithmetic shift right by 0 places (32-bit)
 *
 *  @param[in]  a   Signed 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ int32_t  _asr32_0  (int32_t a)
{
    return a;
}

/** Logical shift right by 1 places (32-bit)
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ uint32_t _lsr32_1  (uint32_t a)
{
    return a >> 1;
}

/** Arithmetic shift right by 1 places (32-bit)
 *
 *  @param[in]  a   Signed 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ int32_t  _asr32_1  (int32_t a)
{
    return a >> 1;
}

/** Logical shift right by 2 places (32-bit)
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ uint32_t _lsr32_2  (uint32_t a)
{
    return a >> 2;
}

/** Arithmetic shift right by 2 places (32-bit)
 *
 *  @param[in]  a   Signed 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ int32_t  _asr32_2  (int32_t a)
{
    return a >> 2;
}

/** Logical shift right by 3 places (32-bit)
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ uint32_t _lsr32_3  (uint32_t a)
{
    return a >> 3;
}

/** Arithmetic shift right by 3 places (32-bit)
 *
 *  @param[in]  a   Signed 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ int32_t  _asr32_3  (int32_t a)
{
    return a >> 3;
}

/** Logical shift right by 4 places (32-bit)
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ uint32_t _lsr32_4  (uint32_t a)
{
    return a >> 4;
}

/** Arithmetic shift right by 4 places (32-bit)
 *
 *  @param[in]  a   Signed 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ int32_t  _asr32_4  (int32_t a)
{
    return a >> 4;
}

/** Logical shift right by 5 places (32-bit)
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ uint32_t _lsr32_5  (uint32_t a)
{
    return a >> 5;
}

/** Arithmetic shift right by 5 places (32-bit)
 *
 *  @param[in]  a   Signed 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ int32_t  _asr32_5  (int32_t a)
{
    return a >> 5;
}

/** Logical shift right by 6 places (32-bit)
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ uint32_t _lsr32_6  (uint32_t a)
{
    return a >> 6;
}

/** Arithmetic shift right by 6 places (32-bit)
 *
 *  @param[in]  a   Signed 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ int32_t  _asr32_6  (int32_t a)
{
    return a >> 6;
}

/** Logical shift right by 7 places (32-bit)
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ uint32_t _lsr32_7  (uint32_t a)
{
    return a >> 7;
}

/** Arithmetic shift right by 7 places (32-bit)
 *
 *  @param[in]  a   Signed 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ int32_t  _asr32_7  (int32_t a)
{
    return a >> 7;
}

/** Logical shift right by 8 places (32-bit)
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ uint32_t _lsr32_8  (uint32_t a)
{
    return a >> 8;
}

/** Arithmetic shift right by 8 places (32-bit)
 *
 *  @param[in]  a   Signed 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ int32_t  _asr32_8  (int32_t a)
{
    return a >> 8;
}

/** Logical shift right by 9 places (32-bit)
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ uint32_t _lsr32_9  (uint32_t a)
{
    return a >> 9;
}

/** Arithmetic shift right by 9 places (32-bit)
 *
 *  @param[in]  a   Signed 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ int32_t  _asr32_9  (int32_t a)
{
    return a >> 9;
}

/** Logical shift right by 10 places (32-bit)
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ uint32_t _lsr32_10 (uint32_t a)
{
    return a >> 10;
}

/** Arithmetic shift right by 10 places (32-bit)
 *
 *  @param[in]  a   Signed 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ int32_t  _asr32_10 (int32_t a)
{
    return a >> 10;
}

/** Logical shift right by 11 places (32-bit)
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ uint32_t _lsr32_11 (uint32_t a)
{
    return a >> 11;
}

/** Arithmetic shift right by 11 places (32-bit)
 *
 *  @param[in]  a   Signed 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ int32_t  _asr32_11 (int32_t a)
{
    return a >> 11;
}

/** Logical shift right by 12 places (32-bit)
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ uint32_t _lsr32_12 (uint32_t a)
{
#if defined(__MLX16__) && !defined(HAS_MLX16_COPROCESSOR) && (!defined(__COVERITY__) && !defined(__POLYSPACE__)) && \
    !defined(MLX_FORCE_C_IMPLEMENTATION)
    __asm__ (
        ";; >> 12\n\t"
        "rl A, #2\t\t; rl 4 <==> rr 12\n\t"
        "rl A, #2\n\t"
        "and A, #15\n\t"
        "rl Y, #2\t\t; rl 4 <==> rr 12\n\t"
        "rl Y, #2\n\t"
        "xor A, Y\t\t; store Y[15:4] in A[15:4]; A[3:0] is temporary xor'ed with Y[3:0]\n\t"
        "and Y, #15\n\t"
        "xor A, Y\t\t; xor A[3:0] with Y[3:0] again to restore original content"
        : "=b" (a)
        : "0" (a)
        );

    return a;
#else
    __asm__ ("; dummy asm code to attract Coverity or Polyspace");
    return a >> 12;
#endif
}

/** Arithmetic shift right by 12 places (32-bit)
 *
 *  @param[in]  a   Signed 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ int32_t  _asr32_12 (int32_t a)
{
#if defined(__MLX16__) && !defined(HAS_MLX16_COPROCESSOR) && (!defined(__COVERITY__) && !defined(__POLYSPACE__)) && \
    !defined(MLX_FORCE_C_IMPLEMENTATION)
    /* >> 4 + >> 8 */
    __asm__ (
        ";; >> 12\n\t"
        "rl A, #2\t; >> 12\n\t" /* >> 12 */
        "rl A, #2\n\t"
        "and A, #15\n\t"
        "mov X, Y\t; dup, keep shifted 3 nibbles in X\n\t" /* dup, keep shifted 3 nibbles in X */
        "lsl X, #2\t; >> 12\n\t" /* >> 12 */
        "lsl X, #2\n\t"
        "or A, X\t\t; merge 3 nibbles of Y with A\n\t" /* merge 3 nibbles of Y with A */
        "mov YL, YH\t; >> 12\n\t" /* >> 12 */
        "ssex Y\n\t"
        "asr Y, #2\n\t"
        "asr Y, #2"
        : "=b" (a)
        : "0" (a)
        : "X" /* clobber */
        );

    return a;
#else
    __asm__ ("; dummy asm code to attract Coverity or Polyspace");
    return a >> 12;
#endif
}

/** Logical shift right by 13 places (32-bit)
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ uint32_t _lsr32_13 (uint32_t a)
{
#if defined(__MLX16__) && !defined(HAS_MLX16_COPROCESSOR) && (!defined(__COVERITY__) && !defined(__POLYSPACE__)) && \
    !defined(MLX_FORCE_C_IMPLEMENTATION)
    __asm__ (
        ";; >> 13\n\t"
        "rl A, #2\t\t; rl 3 <==> rr 13\n\t"
        "rl A, #1\n\t"
        "and A, #7\n\t"
        "rl Y, #2\t\t; rl 3 <==> rr 13\n\t"
        "rl Y, #1\n\t"
        "xor A, Y\t\t; store Y[15:3] in A[15:3]; A[2:0] is temporary xor'ed with Y[2:0]\n\t"
        "and Y, #7\n\t"
        "xor A, Y\t\t; xor A[2:0] with Y[2:0] again to restore original content"
        : "=b" (a)
        : "0" (a)
        );

    return a;
#else
    __asm__ ("; dummy asm code to attract Coverity or Polyspace");
    return a >> 13;
#endif
}

/** Arithmetic shift right by 13 places (32-bit)
 *
 *  @param[in]  a   Signed 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ int32_t  _asr32_13 (int32_t a)
{
#if defined(__MLX16__) && !defined(HAS_MLX16_COPROCESSOR) && (!defined(__COVERITY__) && !defined(__POLYSPACE__)) && \
    !defined(MLX_FORCE_C_IMPLEMENTATION)
    /* alt: >> 1 + >> 12 */
    a >>= 1;

    return _asr32_12 (a);
#else
    __asm__ ("; dummy asm code to attract Coverity or Polyspace");
    return a >> 13;
#endif
}

/** Logical shift right by 14 places (32-bit)
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ uint32_t _lsr32_14 (uint32_t a)
{
#if defined(__MLX16__) && !defined(HAS_MLX16_COPROCESSOR) && (!defined(__COVERITY__) && !defined(__POLYSPACE__)) && \
    !defined(MLX_FORCE_C_IMPLEMENTATION)
    __asm__ (
        ";; >> 14\n\t"
        "rl A, #2\t\t; rl 2 <==> rr 14\n\t"
        "and A, #3\n\t"
        "rl Y, #2\t\t; rl 2 <==> rr 14\n\t"
        "xor A, Y\t\t; store Y[15:2] in A[15:2]; A[1:0] is temporary xor'ed with Y[1:0]\n\t"
        "and Y, #3\n\t"
        "xor A, Y\t\t; xor A[1:0] with Y[1:0] again to restore original content"
        : "=b" (a)
        : "0" (a)
        );

    return a;
#else
    __asm__ ("; dummy asm code to attract Coverity or Polyspace");
    return a >> 14;
#endif
}

/** Arithmetic shift right by 14 places (32-bit)
 *
 *  @param[in]  a   Signed 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ int32_t  _asr32_14 (int32_t a)
{
#if defined(__MLX16__) && !defined(HAS_MLX16_COPROCESSOR) && (!defined(__COVERITY__) && !defined(__POLYSPACE__)) && \
    !defined(MLX_FORCE_C_IMPLEMENTATION)
    /* << 2 + >> 16 */
    __asm__ (
        ";; >> 14 = << 2 + >> 16\n\t"
        "lsl A\t\t; << 1\n\t" /* << 1 */
        "rlc Y\n\t"
        "sc X\t\t; keep carry, sign extend\n\t" /* keep carry, sign extend */
        "neg X\n\t"
        "lsl A\t\t; << 1\n\t" /* << 1 */
        "rlc Y\n\t"
        "mov A, Y\t; >> 16 ; mov does not modify Carry flag\n\t" /* >> 16 */  /* mov does not modify Carry flag */
        "sc Y\t\t; 2nd carry\n\t" /* 2nd carry */
        "lsl X\t\t; or with 1st carry\n\t" /* or with 1st carry */
        "or Y, X\n\t"
        : "=b" (a)
        : "0" (a)
        : "X" /* clobber */
        );

    return a;
#else
    __asm__ ("; dummy asm code to attract Coverity or Polyspace");
    return a >> 14;
#endif
}

/** Logical shift right by 15 places (32-bit)
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ uint32_t _lsr32_15 (uint32_t a)
{
    return a >> 15;
}

/** Arithmetic shift right by 15 places (32-bit)
 *
 *  @param[in]  a   Signed 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ int32_t  _asr32_15 (int32_t a)
{
    return a >> 15;
}

/** Logical shift right by 16 places (32-bit)
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ uint32_t _lsr32_16 (uint32_t a)
{
    return (a >> 16);
}

/** Arithmetic shift right by 16 places (32-bit)
 *
 *  @param[in]  a   Signed 32-bit input to be shifted
 *  @return     Shifted 32-bit input
 *  @ingroup    shift_left_right_result32
 */
static __inline__ int32_t  _asr32_16 (int32_t a)
{
    return (a >> 16);
}

#endif /* SR32_H_ */
/** @}*/