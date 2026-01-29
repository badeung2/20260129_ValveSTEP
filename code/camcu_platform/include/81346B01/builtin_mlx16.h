/** @file
 *  @brief Header file for functions specifically optimized for the MLX16 processor
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

/** \addtogroup math_arithmetic
 *  @{
 */

#ifndef BUILTIN_MLX16_H
#define BUILTIN_MLX16_H

#include <stdint.h>
#include <mlx16_cfg.h>

#ifndef UNITTEST
static __attribute__((always_inline)) inline uint16_t  builtin_mlx16_fsb (uint16_t v);
static __attribute__((always_inline)) inline uint16_t  builtin_mlx16_sfb(uint16_t v);
static __attribute__((always_inline)) inline int16_t   builtin_mlx16_fsb32(uint32_t v);
static __attribute__((always_inline)) inline uint32_t  builtin_mlx16_sfb32(uint16_t v);

static __attribute__((always_inline)) inline int16_t   builtin_mlx16_iavg(int16_t val1, int16_t val2);
static __attribute__((always_inline)) inline int16_t   builtin_mlx16_hdiff(int16_t val1, int16_t val2);
static __attribute__((always_inline)) inline int32_t   builtin_mlx16_adds(int32_t val1, int16_t val2);
static __attribute__((always_inline)) inline int32_t   builtin_mlx16_subs(int32_t val1, int16_t val2);
static __attribute__((always_inline)) inline int32_t   builtin_mlx16_addu(int32_t val1, uint16_t val2);
static __attribute__((always_inline)) inline int32_t   builtin_mlx16_subu(int32_t val1, uint16_t val2);
static __attribute__((always_inline)) inline uint16_t  builtin_mlx16_uavg(uint16_t val1, uint16_t val2);
static __attribute__((always_inline)) inline int16_t   builtin_mlx16_q15(int32_t v);
#endif


/** Find the position (index) of the first set bit (first '1') starting from the MSB (most significant bit).
 *
 *  Example usage:
 *
 *  \code{.c}
 *
 *     // Finding the first set bit in 0 returns 0.
 *     builtin_mlx16_fsb(0) -> 0
 *
 *     // Finding the first set bit in 1 returns 0.
 *     builtin_mlx16_fsb(1) -> 0
 *
 *     // Finding the first set bit in 65535 returns 15.
 *     builtin_mlx16_fsb(0xFFFF) -> 15
 *
 *  \endcode
 *
 *  @param v The bit pattern for which the index of the first set bit is to be found.
 *
 *  @return  The index of the highest set bit; note that function returns 0 if all bits are cleared.
 *
 *  @internal The function makes use of the MLX16 `FSB` instruction.
 *
 *  Instruction details:
 *
 *  \code{.asm}
 *
 *     FSB    <dst>
 *            A | X | Y
 *
 *  \endcode
 *  @endinternal
 *  @ingroup math_bit_operation_group
 */
static __attribute__((always_inline)) inline uint16_t builtin_mlx16_fsb(uint16_t v)
{
#if defined(__MLX16__) && !defined(__COVERITY__) && !defined(__POLYSPACE__) && \
    !defined(MLX_FORCE_C_IMPLEMENTATION)

    __asm__ __volatile__ ("fsb %[res]" : [res] "=r" (v) : "0" (v));

    return v;
#else
    __asm__ ("; dummy asm code to attract Coverity or Polyspace");

    uint16_t result = 0u;

    if (v != 0u) {
        /* alt: first do a binary tree search */
        uint16_t one = 0x8000u;
        result = 15u;

        while ((v & one) == 0u) {
            one >>= 1;
            result--;
        }
    } else {
        /* builtin_mlx16_fsb(0) is 0 to match MLX16 FSB instruction */
    }

    return result;
#endif

}

/** Set the bit at the position (index) in the parameter, starting from the MSB (most significant bit).
 *
 *  Example usage:
 *
 *  \code{.c}
 *
 *     // Setting the bit at index 0 returns 1.
 *     builtin_mlx16_sfb(0) -> 0x0001
 *
 *     // Setting the bit at index 1 returns 2.
 *     builtin_mlx16_sfb(1) -> 0x0002
 *
 *     // Finding the bit at index 15 return 32768.
 *     builtin_mlx16_sfb(15) -> 0x8000
 *
 *  \endcode
 *
 *  @param v The bit number of the bit that needs to be set.
 *          Only the 4 least significant bits of v are used, i.e. the parameter is interpreted as v = v % 16.
 *
 *  @return  A 16 bit integer with one bit set to '1': the bit number indicated by the parameter.
 *
 *  @internal The function makes use of the MLX16 `FSB` instruction.
 *
 *  Instruction details:
 *
 *  \code{.asm}
 *
 *     SFB   <dst>
 *           A | X | Y
 *
 *  \endcode
 *  @endinternal
 *  @ingroup math_bit_operation_group
 */
static __attribute__((always_inline)) inline uint16_t builtin_mlx16_sfb(uint16_t v)
{
#if defined(__MLX16__) && !defined(__COVERITY__) && !defined(__POLYSPACE__) && \
    !defined(MLX_FORCE_C_IMPLEMENTATION)

    __asm("sfb %[res]" : [res] "=b" (v) : "0" (v));

    return v;
#else
    __asm__ ("; dummy asm code to attract Coverity or Polyspace");
    return (uint16_t)1 << (v & 0x000Fu);
#endif
}

/** Find the position (index) of the first set bit (first '1') starting from the MSB (most significant bit).
 *
 *  Example usage:
 *
 *  \code{.c}
 *
 *     // Finding the first set bit in 0 returns 0.
 *     builtin_mlx16_fsb32(0) -> 0
 *
 *     // Finding the first set bit in 1 returns 0.
 *     builtin_mlx16_fsb32(1) -> 0
 *
 *     // Finding the first set bit in 65535 returns 15.
 *     builtin_mlx16_fsb32(0xFFFFFFFF) -> 31
 *
 *  \endcode
 *
 *  @param v The 32-bit pattern for which the index of the first set bit is to be found.
 *
 *  @return  The index of the highest set bit; note that function returns 0 if all bits are cleared.
 *  @ingroup math_bit_operation_group
 */
static __attribute__((always_inline)) inline int16_t builtin_mlx16_fsb32(uint32_t v)
{
    uint16_t result;

    if ((v >> 16u) == 0u) {
        result = builtin_mlx16_fsb((uint16_t) v);
    } else {
        result = 16u + builtin_mlx16_fsb(((uint16_t) (v >> 16u)));
    }

    return (int16_t)result;
}

/** Set the bit at the position (index) in the parameter, starting from the MSB (most significant bit).
 *
 *  \code{.c}
 *
 *     // Setting the bit at index 0 returns 1.
 *     builtin_mlx16_sfb32(0) -> 0x00000001
 *
 *     // Setting the bit at index 1 returns 2.
 *     builtin_mlx16_sfb32(1) -> 0x00000002
 *
 *     // Finding the bit at index 31 return 32768.
 *     builtin_mlx16_sfb32(15) -> 0x80000000
 *
 *  \endcode
 *
 *  @param v The bit number of the bit that needs to be set, where 0 <= v <= 31.
 *
 *  @return  A 32-bit integer with one bit set to '1': the bit number indicated by the parameter.
 *  @ingroup math_bit_operation_group
 */
static __attribute__((always_inline)) inline uint32_t builtin_mlx16_sfb32(uint16_t v)
{
    uint32_t result;

    if (v < 16u) {
        result = (uint32_t) builtin_mlx16_sfb(v);
    } else {
        result = ((uint32_t) builtin_mlx16_sfb(v)) << 16u;
    }

    return result;
}

/** Calculating average of 2 unsigned 16-bit integers
 *
 *  In case the code is compiled for the MLX16, an algorithm is used for the
 *  calculation which prevents promotion to 32 bit, resulting in a faster
 *  implementation
 *
 *  @param[in]  val1    First unsigned 16 bit integer for average calculation
 *  @param[in]  val2    Second unsigned 16 bit integer for average calculation
 *  @return     The averaged value of the 2 input parameters
 *  @ingroup    math_average_group
 */
static __attribute__((always_inline)) inline uint16_t builtin_mlx16_uavg(uint16_t val1, uint16_t val2)
{
#if defined (__MLX16__) && (!defined(__COVERITY__) && !defined(__POLYSPACE__)) && !defined(MLX_FORCE_C_IMPLEMENTATION)
    uint16_t result;

    __asm__ (
        "add %0, %2\n\t"
        "rrc %0"
        : "=r" (result)
        : "0" (val1), "g" (val2) );

    return result;
#else
    uint32_t sum;
    __asm__ ("; dummy asm code to attract Coverity or Polyspace");

    sum = (uint32_t) val1 + val2;
    sum /= 2u;

    return (uint16_t) sum;
#endif /* __MLX16__ */
}

/** Calculating average of 2 signed 16-bit integers. Right shift is used instead
 *  of C-division
 *
 *  In case the code is compiled for the MLX16, an algorithm is used for the
 *  calculation which prevents promotion to 32 bit, resulting in a faster
 *  implementation
 *
 *  @param[in]  val1    First signed 16 bit integer for average calculation
 *  @param[in]  val2    Second signed 16 bit integer for average calculation
 *  @return     The averaged value of the 2 input parameters
 *  @ingroup    math_average_group
 */
static __attribute__((always_inline)) inline int16_t builtin_mlx16_iavg(int16_t val1, int16_t val2)
{
#if defined(__MLX16__) && (!defined(__COVERITY__) && !defined(__POLYSPACE__)) && !defined(MLX_FORCE_C_IMPLEMENTATION)
    int16_t result;

    __asm__ (
        "add %0, %2\n\t"
        "jnv NoOverflow_%=\n\t"
        "rrc %0\n\t"
        "jmp Done_%=\n"
        "NoOverflow_%=:\n\t"
        "asr %0, #1\n"
        "Done_%=:"
        : "=r" (result)
        : "0" (val1), "g" (val2) );

    return result;
#else
    int32_t sum;
    __asm__ ("; dummy asm code to attract Coverity or Polyspace");

    sum = (int32_t) val1 + val2;
    sum >>= 1u;

    return (int16_t) sum;
#endif /* __MLX16__*/
}

/** Calculating (val1 - val2)/2 for 16-bit integers. Right shift is used instead
 *  of C-division
 *
 *  In case the code is compiled for the MLX16, an algorithm is used for the
 *  calculation which prevents promotion to 32 bit, resulting in a faster
 *  implementation
 *
 *  @param[in]  val1    First signed 16 bit integer for calculation
 *  @param[in]  val2    Second signed 16 bit integer for calculation
 *  @return     (val1 - val2)/2
 *  @ingroup    math_average_group
 */
static __attribute__((always_inline)) inline int16_t builtin_mlx16_hdiff(int16_t val1, int16_t val2)
{
#if defined(__MLX16__) && (!defined(__COVERITY__) && !defined(__POLYSPACE__)) && !defined(MLX_FORCE_C_IMPLEMENTATION)
    int16_t result;

    __asm__ (
        "sub %0, %2\n\t"
        "jnv NoOverflow_%=\n\t"
        "cpl C\n\t"
        "rrc %0\n\t"
        "jmp Done_%=\n"
        "NoOverflow_%=:\n\t"
        "asr %0, #1\n"
        "Done_%=:"
        : "=r" (result)
        : "0" (val1), "g" (val2) );

    return result;
#else
    int32_t sum;
    __asm__ ("; dummy asm code to attract Coverity or Polyspace");

    sum = (int32_t) val1 - val2;
    sum >>= 1u;

    return (int16_t) sum;
#endif /* __MLX16__*/
}

/** Signed integers addition val1 + val2 using one 32 bits and one 16 bits register
 *
 *  In case the code is compiled for the MLX16, a builtin instruction is used for the
 *  calculation which prevents promotion to 32 bit for val2, resulting in a faster
 *  implementation
 *
 *  @warning    Overflow of the result is not catch and will produce usual wrapping
 *  @param[in]  val1    First signed 32 bit integer for calculation
 *  @param[in]  val2    Second signed 16 bit integer for calculation
 *  @return     val1 + val2
 *  @ingroup    math_arithmetic
 */
static __attribute__((always_inline)) inline int32_t builtin_mlx16_adds(int32_t val1, int16_t val2)
{
#if defined(__MLX16__) && defined (HAS_MLX16_COPROCESSOR) && (!defined(__COVERITY__) && !defined(__POLYSPACE__)) && !defined(MLX_FORCE_C_IMPLEMENTATION)
    int32_t result;

    __asm__ (
        "adds %0, %2"
        : "=q" (result) /* output */
        : "0" (val1), "r" (val2) /* input  */
        );
    return result;
#else
    int32_t sum;
    __asm__ ("; dummy asm code to attract Coverity or Polyspace");

    sum = val1 + val2;

    return sum;
#endif /* __MLX16__*/
}

/** Signed integers subtraction val1 - val2 using one 32 bits and one 16 bits register
 *
 *  In case the code is compiled for the MLX16, a builtin instruction is used for the
 *  calculation which prevents promotion to 32 bit for val2, resulting in a faster
 *  implementation
 *
 *  @warning    Overflow of the result is not catch and will produce usual wrapping
 *  @param[in]  val1    First signed 32 bit integer for calculation
 *  @param[in]  val2    Second signed 16 bit integer for calculation
 *  @return     val1 - val2
 *  @ingroup    math_arithmetic
 */
static __attribute__((always_inline)) inline int32_t builtin_mlx16_subs(int32_t val1, int16_t val2)
{
#if defined(__MLX16__) && defined (HAS_MLX16_COPROCESSOR) && (!defined(__COVERITY__) && !defined(__POLYSPACE__)) && !defined(MLX_FORCE_C_IMPLEMENTATION)
    int32_t result;

    __asm__ (
        "subs %0, %2"
        : "=q" (result) /* output */
        : "0" (val1), "r" (val2) /* input  */
        );
    return result;
#else
    int32_t sum;
    __asm__ ("; dummy asm code to attract Coverity or Polyspace");

    sum = val1 - val2;

    return sum;
#endif /* __MLX16__*/
}

/** Addition of val1 + val2 using of one 32 bits signed and one 16 bits unsigned
 *
 *  In case the code is compiled for the MLX16, a builtin instruction is used for the
 *  calculation which prevents promotion to 32 bit for val2, resulting in a faster
 *  implementation
 *
 *  @warning    Overflow of the result is not catch and will produce usual wrapping
 *  @param[in]  val1    First signed 32 bit integer for calculation
 *  @param[in]  val2    Second unsigned 16 bit integer for calculation
 *  @return     val1 + val2
 *  @ingroup    math_arithmetic
 */
static __attribute__((always_inline)) inline int32_t builtin_mlx16_addu(int32_t val1, uint16_t val2)
{
#if defined(__MLX16__) && defined (HAS_MLX16_COPROCESSOR) && (!defined(__COVERITY__) && !defined(__POLYSPACE__)) && !defined(MLX_FORCE_C_IMPLEMENTATION)
    int32_t result;

    __asm__ (
        "addu %0, %2"
        : "=q" (result) /* output */
        : "0" (val1), "r" (val2) /* input  */
        );
    return result;
#else
    int32_t sum;
    __asm__ ("; dummy asm code to attract Coverity or Polyspace");

    sum = val1 + (int32_t)val2;

    return sum;
#endif /* __MLX16__*/
}

/** Subtraction of val1 - val2 using of one 32 bits signed and one 16 bits unsigned
 *
 *  In case the code is compiled for the MLX16, a builtin instruction is used for the
 *  calculation which prevents promotion to 32 bit for val2, resulting in a faster
 *  implementation
 *
 *  @warning    Overflow of the result is not catch and will produce usual wrapping
 *  @param[in]  val1    First signed 32 bit integer for calculation
 *  @param[in]  val2    Second unsigned 16 bit integer for calculation
 *  @return     val1 - val2
 *  @ingroup    math_arithmetic
 */
static __attribute__((always_inline)) inline int32_t builtin_mlx16_subu(int32_t val1, uint16_t val2)
{
#if defined(__MLX16__) && defined (HAS_MLX16_COPROCESSOR) && (!defined(__COVERITY__) && !defined(__POLYSPACE__)) && !defined(MLX_FORCE_C_IMPLEMENTATION)
    int32_t result;

    __asm__ (
        "subu %0, %2"
        : "=q" (result) /* output */
        : "0" (val1), "r" (val2) /* input  */
        );
    return result;
#else
    int32_t sum;
    __asm__ ("; dummy asm code to attract Coverity or Polyspace");

    sum = val1 - (int32_t)val2;

    return sum;
#endif /* __MLX16__*/
}

/** Conversion of q30 to q15 number representation
 *
 *  Note: may result in stack reservation, but not all used :-(
 *
 *  @param[in]  v   q30 number to convert to q15
 *  @return     q15 result
 */
static __attribute__((always_inline)) inline int16_t builtin_mlx16_q15(int32_t v)
{
#if defined(__MLX16__) && !defined(HAS_MLX16_COPROCESSOR) && (!defined(__COVERITY__) && !defined(__POLYSPACE__)) && \
    !defined(MLX_FORCE_C_IMPLEMENTATION)

    /* v >> 15 */
    int16_t result;

    __asm__ (
        "asl A\n\t"
        "rlc Y"
        : "=y" (result)
        : "b" (v) /* YA register */
        /* : "Y", "A"  */
        );

    return result;
#else
    __asm__ ("; dummy asm code to attract Coverity or Polyspace");

    uint32_t u32 = (uint32_t)v;     /* cast to unsigned to make MISRA compliant unsigned shift .. */
    u32 = u32 >> 15u;               /* extract bits [30:15]: xiiii-iiii-iiii-iiii-ixxx-xxxx-xxxx-xxxx */

    return (int16_t)u32;
#endif
}

#endif /* BUILTIN_MLX16_H */
/** @}*/
