
/** @file
 *  @brief Header file for functions related to DSP calculations
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
 *  @ingroup dsp
 *
 *  @details
 */

#ifndef MATHLIB_DSP_H_
#define MATHLIB_DSP_H_

#include <stdint.h>

/** \defgroup dsp DSP functionalities
 *  \ingroup mathlib
 *
 *  DSP functions optimized for mlx16 instruction set
 *
 *  The functions in this category make use of the macu/s instructions
 *  available in MLX16 co-processor for more optimal DSP implementations.
 *  As mlx16-gcc doesn't make use of this instruction, the implementations are
 *  written in assembler.
 *
 *  The types of functions available:
 *
 *  - Vector sum calculations
 *  - Norm1 (sum of absolute values) calculations
 *  - Norm2 (squared Euclidian norm) calculations
 *  - NormMax (max of absolute values) calculations
 *  - Scalar (dot, inner) product calculations
 */


/** \defgroup vector_sum Vector sum implementations
 *  \ingroup dsp
 *
 *  These implementations calculate the sum of all values in a given dataset.
 */


/** \defgroup vector_max Vector max implementations
 *  \ingroup dsp
 *
 *  These implementations determine the max value in a given dataset.
 */

/** \defgroup vector_norm Vector norm calculations
 *  \ingroup dsp
 *
 *  L1, L2 and L-infinity norm calculations for values in a given dataset.
 */

/** \defgroup scalar_product Scalar product calculations
 *  \ingroup dsp
 *
 *  Scalar (dot, inner) product calculations
 */


/** \addtogroup dsp
 *  @{
 */

/** Unsigned 8-bit vector sum with 8-bit return value
 *
 *  This function calculates the sum of the elements of the specified vector of
 *  8-bit integer values and of length n > 0. The result is returned as a 8-bit
 *  integer.
 *
 *  @param[in]  *a  Pointer to vector of unsigned 8-bit elements
 *  @param[in]  n   Size of vector
 *
 *  @return     Unsigned 8-bit result
 *
 *  @note       All intermediate values are stored as 8 bit values.
 *  @warning    There are no internal checks for overflow inside the function.
 *  @warning    There is no internal check on valid length n inside the function.
 *  @ingroup    vector_sum
 */
extern uint8_t  vecsumU8_U8  (const uint8_t  *a, uint16_t n);
/** Signed 8-bit vector sum with 8-bit return value
 *
 *  This function calculates the sum of the elements of the specified vector of
 *  8-bit integer values and of length n > 0. The result is returned as a 8-bit
 *  integer.
 *
 *  @param[in]  *a  Pointer to vector of signed 8-bit elements
 *  @param[in]  n   Size of vector
 *
 *  @return     Signed 8-bit result
 *
 *  @note       All intermediate values are stored as 8 bit values.
 *  @warning    There are no internal checks for overflow inside the function.
 *  @warning    There is no internal check on valid length n inside the function.
 *  @ingroup    vector_sum
 */
extern int8_t  vecsumI8_I8  (const int8_t  *a, uint16_t n);
/** Unsigned 8-bit vector sum with 16-bit return value
 *
 *  This function calculates the sum of the elements of the specified vector of
 *  8-bit integer values and of length n > 0. The result is returned as a 16-bit
 *  integer.
 *
 *  @param[in]  *a  Pointer to vector of unsigned 8-bit elements
 *  @param[in]  n   Size of vector
 *
 *  @return     Unsigned 16-bit result
 *
 *  @note       All intermediate values are stored as 16 bit values.
 *  @warning    There are no internal checks for overflow inside the function.
 *  @warning    There is no internal check on valid length n inside the function.
 *  @ingroup    vector_sum
 */
extern uint16_t vecsumU16_U8 (const uint8_t  *a, uint16_t n);
/** Signed 8-bit vector sum with 16-bit return value
 *
 *  This function calculates the sum of the elements of the specified vector of
 *  8-bit integer values and of length n > 0. The result is returned as a 16-bit
 *  integer.
 *
 *  @param[in]  *a  Pointer to vector of signed 8-bit elements
 *  @param[in]  n   Size of vector
 *
 *  @return     Signed 16-bit result
 *
 *  @note       All intermediate values are stored as 16 bit values.
 *  @warning    There are no internal checks for overflow inside the function.
 *  @warning    There is no internal check on valid length n inside the function.
 *  @ingroup    vector_sum
 */
extern int16_t vecsumI16_I8 (const int8_t  *a, uint16_t n);
/** Unsigned 16-bit vector sum with 16-bit return value
 *
 *  This function calculates the sum of the elements of the specified vector of
 *  16-bit integer values and of length n > 0. The result is returned as a
 *  16-bit integer.
 *
 *  @param[in]  *a  Pointer to vector of unsigned 16-bit elements
 *  @param[in]  n   Size of vector
 *
 *  @return     Unsigned 16-bit result
 *
 *  @note       All intermediate values are stored as 16 bit values.
 *  @warning    There are no internal checks for overflow inside the function.
 *  @warning    There is no internal check on valid length n inside the function.
 *  @ingroup    vector_sum
 */
extern uint16_t vecsumU16_U16(const uint16_t *a, uint16_t n);
/** Signed 16-bit vector sum with 16-bit return value
 *
 *  This function calculates the sum of the elements of the specified vector of
 *  16-bit integer values and of length n > 0. The result is returned as a
 *  16-bit integer.
 *
 *  @param[in]  *a  Pointer to vector of signed 16-bit elements
 *  @param[in]  n   Size of vector
 *
 *  @return     Signed 16-bit result
 *
 *  @note       All intermediate values are stored as 16 bit values.
 *  @warning    There are no internal checks for overflow inside the function.
 *  @warning    There is no internal check on valid length n inside the function.
 *  @ingroup    vector_sum
 */
extern int16_t vecsumI16_I16(const int16_t *a, uint16_t n);
/** Unsigned 16-bit vector sum with 32-bit return value
 *
 *  This function calculates the sum of the elements of the specified vector of
 *  16-bit integer values and of length n > 0. The result is returned as a
 *  32-bit integer.
 *
 *  @param[in]  *vector     Pointer to vector of unsigned 16-bit elements
 *  @param[in]  n           Size of vector
 *                          - MLX16 without copro: 0 < n <= 16
 *                          - MLX16 without copro: No restriction
 *
 *  @return     Unsigned 32-bit result
 *
 *  @note       All intermediate values are stored as 32 bit values.
 *  @warning    There are no internal checks for overflow inside the function.
 *  @warning    There is no internal check on valid length n inside the function.
 *  @ingroup    vector_sum
 */
extern uint32_t vecsumU32_U16(const uint16_t *vector, uint16_t n);
/** Signed 16-bit vector sum with 32-bit return value
 *
 *  This function calculates the sum of the elements of the specified vector of
 *  16-bit integer values and of length n > 0. The result is returned as a
 *  32-bit integer.
 *
 *  @param[in]  *a  Pointer to vector of signed 16-bit elements
 *  @param[in]  n   Size of vector
 *                  - MLX16 without copro: 0 < n <= 16
 *                  - MLX16 without copro: No restriction
 *
 *  @return     Signed 32-bit result
 *
 *  @note       All intermediate values are stored as 32 bit values.
 *  @warning    There are no internal checks for overflow inside the function.
 *  @warning    There is no internal check on valid length n inside the function.
 *  @ingroup    vector_sum
 */
extern int32_t vecsumI32_I16(const int16_t *a, uint16_t n);
/** Unsigned 32-bit vector sum with 32-bit return value
 *
 *  This function calculates the sum of the elements of the specified vector of
 *  32-bit integer values and of length n > 0. The result is returned as a
 *  32-bit integer.
 *
 *  @param[in]  *vector     Pointer to vector of unsigned 32-bit elements
 *  @param[in]  n           Size of vector
 *                          - MLX16 without copro: 0 < n <= 16
 *                          - MLX16 without copro: No restriction
 *
 *  @return     Unsigned 32-bit result
 *
 *  @note       All intermediate values are stored as 32 bit values.
 *  @warning    There are no internal checks for overflow inside the function.
 *  @warning    There is no internal check on valid length n inside the function.
 *  @ingroup    vector_sum
 */
extern uint32_t vecsumU32_U32(const uint32_t *vector, uint16_t n);
/** Signed 32-bit vector sum with 32-bit return value
 *
 *  This function calculates the sum of the elements of the specified vector of
 *  32-bit integer values and of length n > 0. The result is returned as a
 *  32-bit integer.
 *
 *  @param[in]  *a  Pointer to vector of signed 32-bit elements
 *  @param[in]  n   Size of vector
 *                  - MLX16 without copro: 0 < n <= 16
 *                  - MLX16 without copro: No restriction
 *
 *  @return     Signed 32-bit result
 *
 *  @note       All intermediate values are stored as 32 bit values.
 *  @warning    There are no internal checks for overflow inside the function.
 *  @warning    There is no internal check on valid length n inside the function.
 *  @ingroup    vector_sum
 */
extern int32_t vecsumI32_I32(const int32_t *a, uint16_t n);
/** Unsigned 32-bit vector sum with 48-bit return value
 *
 *  This function calculates the sum of the elements of the specified vector of
 *  32-bit integer values and of length n > 0. The result is returned as a
 *  48-bit integer.
 *
 *  @param[in]      *vector     Pointer to vector of unsigned 32-bit elements
 *  @param[in]      n           Size of vector
 *                              - MLX16 without copro: 0 < n <= 16
 *                              - MLX16 without copro: No restriction
 *  @param[in,out]  *msw        Pointer to store 16MSBs of 48 bit result
 *
 *  @return     32-bit low-part (LSB) of the result which can be joined
 *              with 16-bit value in msw to produce the 48bit unsigned result
 *
 *  @note       This function returns part of the result through the return value
 *              (32 LSBs) and part through the pointer argument (16 MSBs)
 *  @warning    There are no internal checks for overflow inside the function.
 *  @warning    There is no internal check on valid length n inside the function.
 *  @ingroup    vector_sum
 */
extern uint32_t vecsumU48_U32(const uint32_t *vector, uint16_t n, uint16_t *msw);
/** Signed 32-bit vector sum with 48-bit return value
 *
 *  This function calculates the sum of the elements of the specified vector of
 *  32-bit integer values and of length n > 0. The result is returned as a
 *  48-bit integer.
 *
 *  @param[in]      *a      Pointer to vector of signed 32-bit elements
 *  @param[in]      n       Size of vector
 *                          - MLX16 without copro: 0 < n <= 16
 *                          - MLX16 without copro: No restriction
 *  @param[in,out]  *msw    Pointer to store 16MSBs of 48 bit result
 *
 *  @return     32-bit low-part (LSB) of the result which can be joined
 *              with 16-bit value in msw to produce the 48bit signed result
 *
 *  @note       This function returns part of the result through the return value
 *              (32 LSBs) and part through the pointer argument (16 MSBs)
 *  @warning    There are no internal checks for overflow inside the function.
 *  @warning    There is no internal check on valid length n inside the function.
 *  @ingroup    vector_sum
 */
extern int32_t vecsumI48_I32(const int32_t *a, uint16_t n, int16_t *msw);

/** Unsigned 16-bit integer max
 *
 *  These functions calculate the maximum value of the elements of the
 *  specified vector of 16-bit integer values and of length n > 0.
 *  The result is returned as a 16-bit integer.
 *
 *  @param[in]  *a  Pointer to vector of unsigned 16-bit elements
 *  @param[in]  n   Size of vector
 *
 *  @return     Unsigned 16-bit result
 *
 *  @warning    There is no internal check on valid length n inside the function.
 *  @ingroup    vector_max
 */
extern uint16_t vecmaxU16_U16(const uint16_t *a, uint16_t n);
/** Unsigned 32-bit integer max
 *
 *  These functions calculate the maximum value of the elements of the
 *  specified vector of 32-bit integer values and of length n > 0.
 *  The result is returned as a 32-bit integer.
 *
 *  @param[in]  *a  Pointer to vector of unsigned 32-bit elements
 *  @param[in]  n   Size of vector
 *                  - MLX16 without copro: 0 < n <= 16
 *                  - MLX16 without copro: No restriction
 *
 *  @return     Unsigned 32-bit result
 *
 *  @warning    There is no internal check on valid length n inside the function.
 *  @ingroup    vector_max
 */
extern uint32_t vecmaxU32_U32(const uint32_t *a, uint16_t n);
/** Squared Euclidian vector calculation of 2 integer values
 *
 *  The result of this function is equivalent to a^2 + b^2
 *
 *  @param[in]  a   Signed 16-bit first operand
 *  @param[in]  b   Signed 16-bit second operand
 *
 *  @return     Unsigned 32-bit result
 *
 *  @note       Intermediate values are stored as 32 bit values
 *  @warning    There are no internal checks for overflow.
 *  @ingroup    vector_norm
 */
extern uint32_t norm2U32_I16byI16(int16_t a, int16_t b);
/** Squared Euclidian vector calculation of 2 integer values
 *
 *  The result of this function is equivalent to a^2 + b^2
 *
 *  @param[in]  vec1    Unsigned 16-bit first operand
 *  @param[in]  vec2    Unsigned 16-bit second operand
 *
 *  @return     Unsigned 32-bit result
 *
 *  @note       Intermediate values are stored as 32 bit values
 *  @warning    There are no internal checks for overflow.
 *  @ingroup    vector_norm
 */
extern uint32_t norm2U32_U16byU16(uint16_t vec1, uint16_t vec2);
/** Squared Euclidian vector calculation of 2 integer values
 *
 *  The result of this function is equivalent to a^2 + b^2
 *
 *  @param[in]      a       Signed 16-bit first operand
 *  @param[in]      b       Signed 16-bit second operand
 *  @param[in,out]  *msw    Pointer to store 16MSBs of 48 bit result
 *
 *  @return     32-bit low-part (LSB) of the result which can be joined
 *              with 16-bit value in msw to produce the 48bit unsigned result
 *
 *  @note       This function returns part of the result through the return value
 *              (32 LSBs) and part through the pointer argument (16 MSBs)
 *  @warning    There are no internal checks for overflow.
 *  @ingroup    vector_norm
 */
extern uint32_t norm2U48_I16byI16(int16_t a, int16_t b, uint16_t *msw);
/** Squared Euclidian vector calculation of 2 integer values
 *
 *  The result of this function is equivalent to a^2 + b^2
 *
 *  @param[in]      vec1    Unsigned 16-bit first operand
 *  @param[in]      vec2    Unsigned 16-bit second operand
 *  @param[in,out]  *msw    pointer to store 16MSBs of 48 bit result
 *
 *  @return     32-bit low-part (LSB) of the result which can be joined
 *              with 16-bit value in msw to produce the 48bit unsigned result
 *
 *  @note       This function returns part of the result through the return value
 *              (32 LSBs) and part through the pointer argument (16 MSBs)
 *  @warning    There are no internal checks for overflow.
 *  @ingroup    vector_norm
 */
extern uint32_t norm2U48_U16byU16(uint16_t vec1, uint16_t vec2, uint16_t *msw);
/** Squared Euclidian vector calculation of a vector of integer values
 *
 *  The result of this function is equivalent to a[0]^2 + a[1]^2 + ... + a[n-1]^2
 *
 *  @param[in]  *a  Pointer to vector of signed 16-bit elements
 *  @param[in]  n   Size of vector (unsigned, >0)
 *
 *  @return     Unsigned 32-bit result
 *
 *  @note       Intermediate values are stored as 32 bit values
 *  @warning    There are no internal checks for overflow.
 *  @warning    There is no internal check on valid length n inside the function.
 *  @warning    There is no internal check on valid pointer msw, i.e. non-null, inside the function.
 *  @ingroup    vector_norm
 */
extern uint32_t norm2vectorU32_I16byI16(const int16_t *a, uint16_t n);
/** Squared Euclidian vector calculation of a vector of integer values
 *
 *  The result of this function is equivalent to a[0]^2 + a[1]^2 + ... + a[n-1]^2
 *
 *  @param[in]  *vector     Pointer to vector of unsigned 16-bit elements
 *  @param[in]  n           Size of vector (unsigned, >0)
 *
 *  @return     Unsigned 32-bit result
 *
 *  @note       Intermediate values are stored as 32 bit values
 *  @warning    There are no internal checks for overflow.
 *  @warning    There is no internal check on valid length n inside the function.
 *  @warning    There is no internal check on valid pointer msw, i.e. non-null, inside the function.
 *  @ingroup    vector_norm
 */
extern uint32_t norm2vectorU32_U16byU16(const uint16_t *vector, uint16_t n);
/** Squared Euclidian vector calculation of a vector of integer values
 *
 *  The result of this function is equivalent to a[0]^2 + a[1]^2 + ... + a[n-1]^2
 *
 *  @param[in]      *a      Pointer to vector of signed 16-bit elements
 *  @param[in]      n       Size of vector (unsigned, 0 < n <= 16)
 *  @param[in,out]  *msw    Pointer to store 16MSBs of 48 bit result
 *
 *  @return     32-bit low-part (LSB) of the result which can be joined
 *              with 16-bit value in msw to produce the 48bit unsigned result
 *
 *  @note       This function returns part of the result through the return value
 *              (32 LSBs) and part through the pointer argument (16 MSBs)
 *  @warning    There are no internal checks for overflow.
 *  @warning    There is no internal check on valid length n inside the function.
 *  @warning    There is no internal check on valid pointer msw, i.e. non-null,
 *              inside the function.
 *  @ingroup    vector_norm
 */
extern uint32_t norm2vectorU48_I16byI16(const int16_t *a, uint16_t n, uint16_t *msw);
/** Squared Euclidian vector calculation of a vector of integer values
 *
 *  The result of this function is equivalent to a[0]^2 + a[1]^2 + ... + a[n-1]^2
 *
 *  @param[in]      *vector     Pointer to vector of unsigned 16-bit elements
 *  @param[in]      n           Size of vector (unsigned, 0 < n <= 16)
 *  @param[in,out]  *msw        Pointer to store 16MSBs of 48 bit result
 *
 *  @return     32-bit low-part (LSB) of the result which can be joined
 *              with 16-bit value in msw to produce the 48bit unsigned result
 *
 *  @note       This function returns part of the result through the return value
 *              (32 LSBs) and part through the pointer argument (16 MSBs)
 *  @warning    There are no internal checks for overflow.
 *  @warning    There is no internal check on valid length n inside the function.
 *  @warning    There is no internal check on valid pointer msw, i.e. non-null,
 *              inside the function.
 *  @ingroup    vector_norm
 */
extern uint32_t norm2vectorU48_U16byU16(const uint16_t *vector, uint16_t n, uint16_t *msw);

/** Scalar (dot, inner) product of vectors with 16-bit integers
 *
 *  @param[in]  *a  Pointer to first vector of signed 16-bit elements
 *  @param[in]  *b  Pointer to second vector of signed 16-bit elements
 *  @param[in]  n   Size of vector (unsigned, 0 < n <= 16 (MLX16-x8 only))
 *
 *  @return     Signed 32-bit result
 *
 *  @note       Intermediate values are stored as 32 bit values
 *  @warning    There is no internal check on valid length n inside the function.
 *  @warning    There is no internal check on valid pointer msw, i.e. non-null,
 *              inside the function.
 *  @ingroup    scalar_product
 */
extern int32_t dotproductI32_I16byI16(const int16_t *a, const int16_t *b, uint16_t n);
/** Scalar (dot, inner) product of vectors with 16-bit integers
 *
 *  @param[in]      *a      Pointer to first vector of signed 16-bit elements
 *  @param[in]      *b      Pointer to second vector of signed 16-bit elements
 *  @param[in]      n       Size of vector (0 < n <= 16 (all CPU targets))
 *  @param[in,out]  *msw    Pointer to store 16MSBs of 48 bit result
 *
 *  @return     32-bit low-part (LSB) of the result which can be joined
 *              with 16-bit value in msw to produce the 48bit signed result
 *
 *  @note       This function returns part of the result through the return value
 *              (32 LSBs) and part through the pointer argument (16 MSBs)
 *  @warning    There is no internal check on valid length n inside the function.
 *  @warning    There is no internal check on valid pointer msw, i.e. non-null,
 *              inside the function.
 *  @ingroup    scalar_product
 */
extern int32_t dotproductI48_I16byI16(const int16_t *a, const int16_t *b, uint16_t n, int16_t *msw);
/** Scalar (dot, inner) product of vectors with 16-bit integers
 *
 *  @param[in]  *multiplicand   Pointer to first vector of unsigned 16-bit elements
 *  @param[in]  *multiplier     Pointer to second vector of unsigned 16-bit elements
 *  @param[in]  n               Size of vector (0 < n <= 16 (MLX16-x8 only))
 *
 *  @return     Unsigned 32-bit result
 *
 *  @note       Intermediate values are stored as 32 bit values
 *  @warning    There is no internal check on valid length n inside the function.
 *  @warning    There is no internal check on valid pointer msw, i.e. non-null,
 *              inside the function.
 *  @ingroup    scalar_product
 */
extern uint32_t dotproductU32_U16byU16(const uint16_t *multiplicand, const uint16_t *multiplier, uint16_t n);
/** Scalar (dot, inner) product of vectors with 16-bit integers
 *
 *  @param[in]      *multiplicand   Pointer to first vector of unsigned 16-bit elements
 *  @param[in]      *multiplier     Pointer to second vector of unsigned 16-bit elements
 *  @param[in]      n               Size of vector (0 < n <= 16 (MLX16-x8 only))
 *  @param[in,out]  *msw            Pointer to store 16MSBs of 48 bit result
 *
 *  @return     32-bit low-part (LSB) of the result which can be joined
 *              with 16-bit value in msw to produce the 48bit unsigned result
 *
 *  @note       This function returns part of the result through the return value
 *              (32 LSBs) and part through the pointer argument (16 MSBs)
 *  @warning    There is no internal check on valid length n inside the function.
 *  @warning    There is no internal check on valid pointer msw, i.e. non-null,
 *              inside the function.
 *  @ingroup    scalar_product
 */
extern uint32_t dotproductU48_U16byU16(const uint16_t *multiplicand,
                                       const uint16_t *multiplier,
                                       uint16_t n,
                                       uint16_t *msw);

/** Signed integer sum of absolute values (L1 norm)
 *
 *  @param[in]  *a  Pointer to vector of signed 16-bit elements
 *  @param[in]  n   Size of vector (unsigned, 0 < n <= 16 (MLX16 only))
 *
 *  @return     Unsigned 32-bit result
 *
 *  @note       Intermediate values are stored as 32 bit values
 *  @warning    There is no internal check on valid length n inside the function.
 *  @ingroup    vector_norm
 */
extern uint32_t norm1vectorU32_I16 (const int16_t *a, uint16_t n);
/** Signed integer sum of absolute values (no check on overflow) (L1 norm)
 *
 *  @param[in]  *a  Pointer to vector of signed 32-bit elements
 *  @param[in]  n   Size of vector (unsigned, 0 < n <= 16) (all CPU targets)
 *
 *  @return     Unsigned 32-bit result
 *
 *  @note       Intermediate values are stored as 32 bit values
 *  @warning    There is no internal check on valid length n inside the function.
 *  @ingroup    vector_norm
 */
extern uint32_t norm1vectorU32_I32(const int32_t *a, uint16_t n);
/** Signed integer sum of absolute values (L1 norm)
 *
 *  @param[in]      *a      pointer to vector of signed 32-bit elements
 *  @param[in]      n       size of vector (unsigned, 0 < n <= 16 (all CPU targets))
 *  @param[in,out]  *msw    pointer to store 16MSBs of 48 bit result
 *
 *  @return     32-bit low-part (LSB) of the result which can be joined
 *              with 16-bit value in msw to produce the 48bit unsigned result
 *
 *  @note       This function returns part of the result through the return value
 *              (32 LSBs) and part through the pointer argument (16 MSBs)
 *  @warning    There is no internal check on valid length n inside the function.
 *  @warning    There is no internal check on valid pointer msw, i.e. non-null,
 *              inside the function.
 *  @ingroup    vector_norm
 */
extern uint32_t norm1vectorU48_I32(const int32_t *a, uint16_t n, uint16_t *msw);

/** Signed integer max of absolute values (L-infinity norm)
 *
 *  @param[in]  *v  Pointer to vector of signed 16-bit elements
 *  @param[in]  n   Size of vector (0 < n <= 16)
 *
 *  @return     Unsigned 16-bit result
 *
 *  @warning    There is no internal check on valid length n inside the function.
 *  @ingroup    vector_norm
 */
extern uint16_t normmaxvectorU16_I16 (int16_t *v, uint16_t n);
/** Signed integer max of absolute values (L-infinity norm)
 *
 *  @param[in]  *v  Pointer to vector of signed 32-bit elements
 *  @param[in]  n   Size of vector (0 < n <= 16)
 *
 *  @return     Unsigned 32-bit result
 *
 *  @warning    There is no internal check on valid length n inside the function.
 *  @ingroup    vector_norm
 */
extern uint32_t normmaxvectorU32_I32 (int32_t *v, uint16_t n);
/** Calculate vector magnitude (approximation) (L2 norm)
 *
 *  result = sqrt(x^2 + y^2)
 *
 *  @note 		The margin of error is capped at 4% when the sum of the inputs is greater than 2^6, and maintained around 3% or less
 *         		for a sum of the inputs greater than 2^10.
 *
 *  @param[in] 	x 	First input for the vector
 *  @param[in] 	y 	Second input for the vector
 *
 *  @return 	Magnitude from the given input vector
 *  @ingroup    vector_norm
 */
extern uint16_t calculate_vector_magnitude (int16_t x, int16_t y);

/** @}*/

#endif /* MATHLIB_DSP_H_ */
