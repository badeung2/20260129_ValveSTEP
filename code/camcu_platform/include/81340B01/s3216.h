/** @file
 *  @brief Header file for 32 bit shift implementations (right and left) with 16 bit return type
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
 *  This header file provides 32 bit shift implementations for unsigned integers with 16-bit result.
 *
 *  2 categories of functions are available here:
 *
 *  - _lsr3216_X functions: Shift 32 bit input RIGHT by X and return the 16 LSBs of the result
 *  - _lsl3216_Y functions: Shift 32 bit input LEFT by Y and return the 16 MSBs of the result
 *
 *  The implementations here mean to compensate for inefficient compiler implementations, and
 *  are meant to be included in the compiler at some point.
 *  The optimized functions in this header file can be used with any mlx16 architecture
 *  and optimization setting, but are specifically optimized for:
 *
 *  - O3 optimization setting
 *  - mlx16-8 and mlx16-e8 architectures (the others use coprocessor for shifting)
 *
 *  @note    lsl3216_<n> = lsr3216_<16-n> and vise versa
 *  @ingroup mathlib
 *
 *  @details
 */


/** \defgroup shift_32 Bit shift operations on 32-bit variables
 *  \ingroup mathlib
 *
 *  The implementations here mean to compensate for inefficient compiler implementations, and
 *  are meant to be included in the compiler at some point.
 *  The optimized functions in this header file can be used with any mlx16 architecture
 *  and optimization setting, but are specifically optimized for:
 *
 *  - O3 optimization setting
 *  - mlx16-8 and mlx16-e8 architectures (the others use coprocessor for shifting)
 *
 *  There are 3 categories functions in this category:
 *
 *  - @ref shift_left_right_result32
 *  - Table-based implementations: crc_table
 *  - Table-based implementation on a full dataset: crc_table_dataset
 */


/** \defgroup shift_left_right_result32 Left and right shift implementations with 32-bit result
 *  \ingroup shift_32
 *
 *  Left and right shift implementations with 32-bit result
 *
 *  \b Function \b prototypes:
 *
 *  - \a uint32_t \a _lsr32_{shift_amount}(uint32_t a) : Logical shift right by \a shift_amount bits
 *  - \a uint32_t \a _asr32_{shift_amount}(int32_t a) : Arithmetic shift right (sign bit preserved) by \a shift_amount bits
 *  - \a uint32_t \a _lsl32_{shift_amount}(uint32_t a) : Logical shift left by \a shift_amount bits
 */

/** \defgroup shift_right_result16 Right shift implementations with 16-bit result
 *  \ingroup shift_32
 *
 *  Shift 32 bit input RIGHT by X and return the 16 LSBs of the result
 *
 *  \b Function \b prototype:
 *
 *  - \a uint16_t \a _lsr3216_{shift_amount}(uint32_t a) : Logical shift right by \a shift_amount bits, 16 LSBs returned
 */

/** \defgroup shift_left_result16 Left shift implementations with 16-bit result
 *  \ingroup shift_32
 *
 *  Shift 32 bit input LEFT by Y and return the 16 MSBs of the result
 *
 *  \b Function \b prototype:
 *
 *  - \a uint16_t \a _lsl3216_{shift_amount}(uint32_t a) : Logical shift left by \a shift_amount bits, 16 MSBs returned
 */

 /** \addtogroup shift_32
 *  @{
 */

#ifndef S3216_H_
#define S3216_H_

#include <mathlib.h>
#include <mlx16_cfg.h>

#ifndef UNITTEST
/* returns 16 LSBs of a >> n */
static __inline__ __attribute__ ((always_inline)) uint16_t _lsr3216_0 (uint32_t a);
static __inline__ __attribute__ ((always_inline)) uint16_t _lsr3216_1 (uint32_t a);
static __inline__ __attribute__ ((always_inline)) uint16_t _lsr3216_2 (uint32_t a);
static __inline__ __attribute__ ((always_inline)) uint16_t _lsr3216_3 (uint32_t a);
static __inline__ __attribute__ ((always_inline)) uint16_t _lsr3216_4 (uint32_t a);
static __inline__ __attribute__ ((always_inline)) uint16_t _lsr3216_5 (uint32_t a);
static __inline__ __attribute__ ((always_inline)) uint16_t _lsr3216_6 (uint32_t a);
static __inline__ __attribute__ ((always_inline)) uint16_t _lsr3216_7 (uint32_t a);
static __inline__ __attribute__ ((always_inline)) uint16_t _lsr3216_8 (uint32_t a);
static __inline__ __attribute__ ((always_inline)) uint16_t _lsr3216_9 (uint32_t a);
static __inline__ __attribute__ ((always_inline)) uint16_t _lsr3216_10 (uint32_t a);
static __inline__ __attribute__ ((always_inline)) uint16_t _lsr3216_11 (uint32_t a);
static __inline__ __attribute__ ((always_inline)) uint16_t _lsr3216_12 (uint32_t a);
static __inline__ __attribute__ ((always_inline)) uint16_t _lsr3216_13 (uint32_t a);
static __inline__ __attribute__ ((always_inline)) uint16_t _lsr3216_14 (uint32_t a);
static __inline__ __attribute__ ((always_inline)) uint16_t _lsr3216_15 (uint32_t a);
static __inline__ __attribute__ ((always_inline)) uint16_t _lsr3216_16 (uint32_t a);

#endif

/** Logical shift right by 0 places and return the 16 LSBs of the result
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     16 LSBs of the shifted input
 *  @ingroup    shift_right_result16
 */
static __inline__ uint16_t _lsr3216_0 (uint32_t a)
{
    return (uint16_t) a;
}

/** Logical shift right by 1 places and return the 16 LSBs of the result
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     16 LSBs of the shifted input
 *  @ingroup    shift_right_result16
 */
static __inline__ uint16_t _lsr3216_1 (uint32_t a)
{
    return (uint16_t) (a >> 1);
}

/** Logical shift right by 2 places and return the 16 LSBs of the result
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     16 LSBs of the shifted input
 *  @ingroup    shift_right_result16
 */
static __inline__ uint16_t _lsr3216_2 (uint32_t a)
{
    return (uint16_t) (a >> 2);
}

/** Logical shift right by 3 places and return the 16 LSBs of the result
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     16 LSBs of the shifted input
 *  @ingroup    shift_right_result16
 */
static __inline__ uint16_t _lsr3216_3 (uint32_t a)
{
    return (uint16_t) (a >> 3);
}

/** Logical shift right by 4 places and return the 16 LSBs of the result
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     16 LSBs of the shifted input
 *  @ingroup    shift_right_result16
 */
static __inline__ uint16_t _lsr3216_4 (uint32_t a)
{
#if defined(__MLX16__) && !defined(HAS_MLX16_COPROCESSOR)  \
    && !defined(__COVERITY__) && !defined(__POLYSPACE__)    \
    && !defined(MLX_FORCE_C_IMPLEMENTATION)

    uint16_t result;

    __asm__ (
        ";; >> 4\n\t"
        ";; mov lowest nibble of Y(l) to A(l)\n\t"
        "and Y, #15\n\t"
        "and Al, #-16\t; mask lowest nibble\n\t"
        "or A, Y\n\t"
        "rr A, #2\t; >> 4, and moving in lowest nibble of Y to highest nibble of A\n\t"
        "rr A, #2"
        : "=a" (result)
        : "b" (a)
        );

    return result;
#else
    __asm__ ("; dummy asm code to attract Coverity or Polyspace");
    return (uint16_t) (a >> 4);
#endif
}

/** Logical shift right by 8 places and return the 16 LSBs of the result
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     16 LSBs of the shifted input
 *  @ingroup    shift_right_result16
 */
static __inline__ uint16_t _lsr3216_8 (uint32_t a)
{
#if defined(__MLX16__) && !defined(HAS_MLX16_COPROCESSOR)  \
    && !defined(__COVERITY__) && !defined(__POLYSPACE__)    \
    && !defined(MLX_FORCE_C_IMPLEMENTATION)

    uint16_t result;

    __asm__ (
        ";; >> 8\n\t"
        "mov AL, AH\t; >> 8\n\t" /* >> 8 */
        "mov AH, YL\n\t"
        : "=a" (result)
        : "b" (a)
        );

    return result;
#else
    __asm__ ("; dummy asm code to attract Coverity or Polyspace");
    return (uint16_t) (a >> 8);
#endif
}

/** Logical shift right by 5 places and return the 16 LSBs of the result
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     16 LSBs of the shifted input
 *  @ingroup    shift_right_result16
 */
static __inline__ uint16_t _lsr3216_5 (uint32_t a)
{
#if defined(__MLX16__) && !defined(HAS_MLX16_COPROCESSOR)
    return _lsr3216_8(a << 3); /* (<<3 ) >> 8 */
#else
    return (uint16_t) (a >> 5);
#endif
}

/** Logical shift right by 6 places and return the 16 LSBs of the result
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     16 LSBs of the shifted input
 *  @ingroup    shift_right_result16
 */
static __inline__ uint16_t _lsr3216_6 (uint32_t a)
{
#if defined(__MLX16__) && !defined(HAS_MLX16_COPROCESSOR)
    return _lsr3216_8(a << 2); /* (<<2 ) >> 8 */
#else
    return (uint16_t) (a >> 6);
#endif
}

/** Logical shift right by 7 places and return the 16 LSBs of the result
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     16 LSBs of the shifted input
 *  @ingroup    shift_right_result16
 */
static __inline__ uint16_t _lsr3216_7 (uint32_t a)
{
#if defined(__MLX16__) && !defined(HAS_MLX16_COPROCESSOR)
    return _lsr3216_8(a << 1); /* (<< 1) >> 8 */
#else
    return (uint16_t) (a >> 7);
#endif
}

/** Logical shift right by 9 places and return the 16 LSBs of the result
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     16 LSBs of the shifted input
 *  @ingroup    shift_right_result16
 */
static __inline__ uint16_t _lsr3216_9 (uint32_t a)
{
#if defined(__MLX16__) && !defined(HAS_MLX16_COPROCESSOR)
    return _lsr3216_8(a >> 1); /* (>> 1) >> 8 */
#else
    return (uint16_t) (a >> 9);
#endif
}

/** Logical shift right by 10 places and return the 16 LSBs of the result
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     16 LSBs of the shifted input
 *  @ingroup    shift_right_result16
 */
static __inline__ uint16_t _lsr3216_10 (uint32_t a)
{
#if defined(__MLX16__) && !defined(HAS_MLX16_COPROCESSOR)
    return _lsr3216_8(a >> 2); /* (>> 2) >> 8 */
#else
    return (uint16_t) (a >> 10);
#endif
}

/** Logical shift right by 12 places and return the 16 LSBs of the result
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     16 LSBs of the shifted input
 *  @ingroup    shift_right_result16
 */
static __inline__ uint16_t _lsr3216_12 (uint32_t a)
{
#if defined(__MLX16__) && !defined(HAS_MLX16_COPROCESSOR)  \
    && !defined(__COVERITY__) && !defined(__POLYSPACE__)    \
    && !defined(MLX_FORCE_C_IMPLEMENTATION)

    uint16_t result;

    __asm__ (
        ";; >> 12\n\t"
        "rl A, #2\t; >> 12\n\t" /* >> 12 */
        "rl A, #2\n\t"
        "and A, #15\n\t"
        "lsl Y, #2\t; >> 12 = << 4 + >> 16\n\t" /* >> 12 = << 4 + >> 16*/
        "lsl Y, #2\n\t"
        "or A, Y\t\t; merge 3 nibbles of Y with A" /* merge 3 nibbles of Y with A */
        : "=a" (result)
        : "b" (a)
        );

    return result;
#else
    __asm__ ("; dummy asm code to attract Coverity or Polyspace");
    return (uint16_t) (a >> 12);
#endif
}

/** Logical shift right by 11 places and return the 16 LSBs of the result
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     16 LSBs of the shifted input
 *  @ingroup    shift_right_result16
 */
static __inline__ uint16_t _lsr3216_11 (uint32_t a)
{
#if defined(__MLX16__) && !defined(HAS_MLX16_COPROCESSOR)
    return _lsr3216_12(a << 1); /* (<< 1) >> 12 */
#else
    return (uint16_t) (a >> 11);
#endif
}

/** Logical shift right by 16 places and return the 16 LSBs of the result
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     16 LSBs of the shifted input
 *  @ingroup    shift_right_result16
 */
static __inline__ uint16_t _lsr3216_16 (uint32_t a)
{
#if defined(__MLX16__) && !defined(HAS_MLX16_COPROCESSOR)  \
    && !defined(__COVERITY__) && !defined(__POLYSPACE__)    \
    && !defined(MLX_FORCE_C_IMPLEMENTATION)

    uint16_t result;

    __asm__ (
        ";; >> 16\n\t"
        : "=y" (result) /* >> 16 */
        : "b" (a)
        );

    return result;
#else
    __asm__ ("; dummy asm code to attract Coverity or Polyspace");
    return (uint16_t) (a >> 16);
#endif
}

/** Logical shift right by 13 places and return the 16 LSBs of the result
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     16 LSBs of the shifted input
 *  @ingroup    shift_right_result16
 */
static __inline__ uint16_t _lsr3216_13 (uint32_t a)
{
#if defined(__MLX16__) && !defined(HAS_MLX16_COPROCESSOR)
    return _lsr3216_16(a << 3); /* (<< 3) >> 16 */
#else
    return (uint16_t) (a >> 13);
#endif
}

/** Logical shift right by 14 places and return the 16 LSBs of the result
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     16 LSBs of the shifted input
 *  @ingroup    shift_right_result16
 */
static __inline__ uint16_t _lsr3216_14 (uint32_t a)
{
#if defined(__MLX16__) && !defined(HAS_MLX16_COPROCESSOR)
    return _lsr3216_16(a << 2); /* (<< 2) >> 16 */
#else
    return (uint16_t) (a >> 14);
#endif
}

/** Logical shift right by 15 places and return the 16 LSBs of the result
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     16 LSBs of the shifted input
 *  @ingroup    shift_right_result16
 */
static __inline__ uint16_t _lsr3216_15 (uint32_t a)
{
#if defined(__MLX16__) && !defined(HAS_MLX16_COPROCESSOR)
    return _lsr3216_16(a << 1); /* (<< 1) >> 16 */
#else
    return (uint16_t) (a >> 15);
#endif
}

/** Logical shift left by 0 places and return the 16 MSBs of the result
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     16 MSBs of the shifted input
 *  @ingroup    shift_left_result16
 */
static __inline__ __attribute__ ((always_inline)) uint16_t _lsl3216_0 (uint32_t a)
{
    return _lsr3216_16(a);
}

/** Logical shift left by 1 places and return the 16 MSBs of the result
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     16 MSBs of the shifted input
 *  @ingroup    shift_left_result16
 */
static __inline__ __attribute__ ((always_inline)) uint16_t _lsl3216_1 (uint32_t a)
{
    return _lsr3216_15(a);
}

/** Logical shift left by 2 places and return the 16 MSBs of the result
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     16 MSBs of the shifted input
 *  @ingroup    shift_left_result16
 */
static __inline__ __attribute__ ((always_inline)) uint16_t _lsl3216_2 (uint32_t a)
{
    return _lsr3216_14(a);
}

/** Logical shift left by 3 places and return the 16 MSBs of the result
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     16 MSBs of the shifted input
 *  @ingroup    shift_left_result16
 */
static __inline__ __attribute__ ((always_inline)) uint16_t _lsl3216_3 (uint32_t a)
{
    return _lsr3216_13(a);
}

/** Logical shift left by 4 places and return the 16 MSBs of the result
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     16 MSBs of the shifted input
 *  @ingroup    shift_left_result16
 */
static __inline__ __attribute__ ((always_inline)) uint16_t _lsl3216_4 (uint32_t a)
{
    return _lsr3216_12(a);
}

/** Logical shift left by 5 places and return the 16 MSBs of the result
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     16 MSBs of the shifted input
 *  @ingroup    shift_left_result16
 */
static __inline__ __attribute__ ((always_inline)) uint16_t _lsl3216_5 (uint32_t a)
{
    return _lsr3216_11(a);
}

/** Logical shift left by 6 places and return the 16 MSBs of the result
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     16 MSBs of the shifted input
 *  @ingroup    shift_left_result16
 */
static __inline__ __attribute__ ((always_inline)) uint16_t _lsl3216_6 (uint32_t a)
{
    return _lsr3216_10(a);
}

/** Logical shift left by 7 places and return the 16 MSBs of the result
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     16 MSBs of the shifted input
 *  @ingroup    shift_left_result16
 */
static __inline__ __attribute__ ((always_inline)) uint16_t _lsl3216_7 (uint32_t a)
{
    return _lsr3216_9 (a);
}

/** Logical shift left by 8 places and return the 16 MSBs of the result
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     16 MSBs of the shifted input
 *  @ingroup    shift_left_result16
 */
static __inline__ __attribute__ ((always_inline)) uint16_t _lsl3216_8 (uint32_t a)
{
    return _lsr3216_8 (a);
}

/** Logical shift left by 9 places and return the 16 MSBs of the result
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     16 MSBs of the shifted input
 *  @ingroup    shift_left_result16
 */
static __inline__ __attribute__ ((always_inline)) uint16_t _lsl3216_9 (uint32_t a)
{
    return _lsr3216_7 (a);
}

/** Logical shift left by 10 places and return the 16 MSBs of the result
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     16 MSBs of the shifted input
 *  @ingroup    shift_left_result16
 */
static __inline__ __attribute__ ((always_inline)) uint16_t _lsl3216_10 (uint32_t a)
{
    return _lsr3216_6 (a);
}

/** Logical shift left by 11 places and return the 16 MSBs of the result
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     16 MSBs of the shifted input
 *  @ingroup    shift_left_result16
 */
static __inline__ __attribute__ ((always_inline)) uint16_t _lsl3216_11 (uint32_t a)
{
    return _lsr3216_5 (a);
}

/** Logical shift left by 12 places and return the 16 MSBs of the result
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     16 MSBs of the shifted input
 *  @ingroup    shift_left_result16
 */
static __inline__ __attribute__ ((always_inline)) uint16_t _lsl3216_12 (uint32_t a)
{
    return _lsr3216_4 (a);
}

/** Logical shift left by 13 places and return the 16 MSBs of the result
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     16 MSBs of the shifted input
 *  @ingroup    shift_left_result16
 */
static __inline__ __attribute__ ((always_inline)) uint16_t _lsl3216_13 (uint32_t a)
{
    return _lsr3216_3 (a);
}

/** Logical shift left by 14 places and return the 16 MSBs of the result
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     16 MSBs of the shifted input
 *  @ingroup    shift_left_result16
 */
static __inline__ __attribute__ ((always_inline)) uint16_t _lsl3216_14 (uint32_t a)
{
    return _lsr3216_2 (a);
}

/** Logical shift left by 15 places and return the 16 MSBs of the result
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     16 MSBs of the shifted input
 *  @ingroup    shift_left_result16
 */
static __inline__ __attribute__ ((always_inline)) uint16_t _lsl3216_15 (uint32_t a)
{
    return _lsr3216_1 (a);
}

/** Logical shift left by 16 places and return the 16 MSBs of the result
 *
 *  @param[in]  a   Unsigned 32-bit input to be shifted
 *  @return     16 MSBs of the shifted input
 *  @ingroup    shift_left_result16
 */
static __inline__ __attribute__ ((always_inline)) uint16_t _lsl3216_16 (uint32_t a)
{
    return _lsr3216_0 (a);
}

#endif /* S3216_H_ */
/** @}*/