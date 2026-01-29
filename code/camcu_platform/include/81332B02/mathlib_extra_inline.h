/** @file
 *  @brief Mathlib header file for extra inlined functions
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
 *  @ingroup mathlib
 *
 *  @details
 */

 /** \addtogroup mathlib
 *  @{
 */

/*
 *  Declaration of functions for extra inline
 */
#ifndef MATHLIB_EXTRA_INLINE_H
#define MATHLIB_EXTRA_INLINE_H

#include <stdint.h>
#include <stdlib.h> /* abs */

/* Helpers */
static inline __attribute__((always_inline)) uint16_t safe_neg16 (int16_t x);
static inline __attribute__((always_inline)) uint32_t safe_neg32 (int32_t x);

MATHLIB_EXTRA_INLINE int16_t mulI16hi_I16byU16(int16_t multiplicand, uint16_t multiplier)
{
    int16_t result;
    int16_t result2;    /* clobbering of the register */

    __asm__ __volatile__ (
        "cmp    A, #0\n\t"
        "jnn    l_mul_unsigned_%=\n\t"

        "neg    A\n\t"
        "mulu   YA, A, %w3\n\t"
        "neg    YA\n\t"
        "jmp    l_mul_end_%=\n\t"
        "l_mul_unsigned_%=:\n\t"
        "mulu   YA, A, %w3\n\t"
        "l_mul_end_%=:"
        : "=y" (result), "=a" (result2)
        : "%a" (multiplicand), "nr" (multiplier)
        );

    return result;
}

MATHLIB_EXTRA_INLINE int32_t mulI32_I16byU16(int16_t multiplicand, uint16_t multiplier)
{
    int32_t result;

    __asm__ __volatile__ (
        "cmp	A, #0\n\t"
        "jnn	l_mul_unsigned_%=\n\t"

        "neg	A\n\t"
        "mulu	YA, A, %w2\n\t"
        "neg	YA\n\t"
        "jmp    l_mul_end_%=\n\t"

        "l_mul_unsigned_%=:\n\t"
        "mulu	YA, A, %w2\n\t"

        "l_mul_end_%=:"
        : "=b" (result)
        : "%a" (multiplicand), "nr" (multiplier)
        );

    return (result);
}

MATHLIB_EXTRA_INLINE int32_t mulI32lo_I32byI16(int32_t multiplicand, int16_t multiplier)
{
    int32_t result;

    if (multiplier < 0)
    {
        result = -mulI32lo_I32byU16(multiplicand, safe_neg16(multiplier));
    }
    else
    {
        result = mulI32lo_I32byU16(multiplicand, (uint16_t)multiplier);
    }

    return result;
}

MATHLIB_EXTRA_INLINE int32_t mulI32lo_I32byU16(int32_t multiplicand, uint16_t multiplier)
{
    int32_t result;

    if (multiplicand < 0)
    {
        result = (int32_t)safe_neg32(mulU32lo_U32byU16(safe_neg32(multiplicand),
                                                       multiplier));
    }
    else
    {
        result = (int32_t)mulU32lo_U32byU16((uint32_t)multiplicand,
                                            multiplier);
    }

    return result;
}

MATHLIB_EXTRA_INLINE uint32_t mulU32lo_U32byU16(uint32_t multiplicand, uint16_t multiplier)
{
    uint32_t result;

    result = mulU32_U16byU16((uint16_t)(multiplicand >> 16), multiplier);
    result <<= 16;
    result += mulU32_U16byU16((uint16_t)multiplicand, multiplier);

    return result;
}

MATHLIB_EXTRA_INLINE uint32_t divU32_U32byU16(uint32_t dividend, uint16_t divisor)
{
    uint32_t result;

    __asm__ __volatile__ (
        "push A\n\t"
        "lod  A, Y\n\t"
        "lod  Y, #0\n\t"
        "divu YA, %w2\n\t"
        "divu YA, %w2\n\t"
        "push A\n\t"
        "lod  A, [S-4]\n\t"
        "divu YA, %w2\n\t"
        "divu YA, %w2\n\t"
        "pop  Y\n\t"
        "dec  S, #2"
        : "=b" (result)
        : "b" (dividend), "nx" (divisor)
        );

    return result;
}

MATHLIB_EXTRA_INLINE int32_t divI32_I32byI16(int32_t dividend, int16_t divisor)
{
    int32_t result;

    if (divisor < 0)
    {
        result = -divI32_I32byU16(dividend, safe_neg16(divisor));
    }
    else
    {
        result = divI32_I32byU16(dividend, (uint16_t)divisor);
    }

    return result;
}

MATHLIB_EXTRA_INLINE int32_t divI32_I32byU16(int32_t dividend, uint16_t divisor)
{
    int32_t result;

    if (dividend < 0)
    {
        result = (int32_t)safe_neg32(divU32_U32byU16(safe_neg32(dividend),
                                                     divisor));
    }
    else
    {
        result = (int32_t)divU32_U32byU16((uint32_t)dividend, divisor);
    }

    return result;
}

MATHLIB_EXTRA_INLINE int16_t divI16_I32byI16(int32_t dividend, int16_t divisor)
{
    int16_t result;
    int16_t result2;                                  /* Clobbering of the register */

    __asm__ __volatile__ (
        "divs YA, %w3\n\t"
        "divs YA, %w3\n\t"
        "dadjs YA,%w3"
        : "=a" (result), "=y" (result2)
        : "b" (dividend), "x" (divisor)
        );

    return result;
}

MATHLIB_EXTRA_INLINE int16_t divI16_I32byU16(int32_t dividend, uint16_t divisor)
{
    int16_t result;

    if (dividend < 0)
    {
        result = (int16_t)safe_neg16(divU16_U32byU16(safe_neg32(dividend),
                                                     divisor));
    }
    else
    {
        result = (int16_t)divU16_U32byU16((uint32_t)dividend, divisor);
    }

    return result;
}

MATHLIB_EXTRA_INLINE int8_t safeabs8(int8_t x)
{
    int8_t retval;
    if (x == (INT8_MIN)) {
        retval = INT8_MAX;
    } else {
        retval = (int8_t)abs(x);
    }
    return retval;
}

MATHLIB_EXTRA_INLINE int16_t safeabs16(int16_t x)
{
    int16_t retval;
    if (x == INT16_MIN) {
        retval = INT16_MAX;
    } else {
        retval = (int16_t)abs(x);
    }
    return retval;
}

/** Safe negation of int16_t value
 *
 *  Based on unsigned integer overflow (valid in C) instead of signed integer
 *  overflow which results in undefined behavior (UB) for `-INT16_MIN`.
 *
 *  @param[in]  x    Input value
 *
 *  @return Negated input value. For negative input values, result is returned
 *          as unsigned bitmap of two's complement representation (can be casted
 *          to int16_t, if needed)
 */
static inline __attribute__((always_inline)) uint16_t safe_neg16(int16_t x) {
    return ((uint16_t)0 - (uint16_t)x);
}

/** Safe negation of int32_t value
 *
 *  Based on unsigned integer overflow (valid in C) instead of signed integer
 *  overflow which results in undefined behavior (UB) for `-INT32_MIN`.
 *
 *  @param[in]  x    Input value
 *
 *  @return Negated input value. For negative input values, result is returned
 *          as unsigned bitmap of two's complement representation (can be casted
 *          to int32_t, if needed)
 */
static inline __attribute__((always_inline)) uint32_t safe_neg32(int32_t x) {
    return ((uint32_t)0 - (uint32_t)x);
}

#endif /* MATHLIB_EXTRA_INLINE_H */
/** @}*/
