/** @file
 *  @brief General mathlib header file
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

/** \defgroup mathlib Math library */

/** \defgroup math_arithmetic Basic arithmetical and other mathematical functionalities
 *  \ingroup mathlib
 *
 *  This group implements basic arithmetical and other mathematical functions optimized for use with MLX16 instruction sets.
 *
 */

/** \defgroup math_division_group Division functions
 *  \ingroup math_arithmetic
 *
 *  Division functions optimized for MLX16 instruction set and given input parameter sizes
 */

/** \defgroup math_multiplication_group Multiplication functions
 *  \ingroup math_arithmetic
 *
 *  Multiplication functions optimized for MLX16 instruction set and given input parameter sizes
 */

/** \defgroup math_sqrt_group Square root functions
 *  \ingroup math_arithmetic
 *
 *  Square root functions optimized for MLX16 instruction set and given input parameter sizes
 */

/** \defgroup math_trigonometry_group Trigonometric functions
 *  \ingroup math_arithmetic
 *
 *  Trigonometric functions optimized for MLX16 instruction set and given input parameter sizes
 */

/** \defgroup math_lfsr_group Linear Feedback shift register functions
 *  \ingroup math_arithmetic
 *
 *  Linear Feedback shift register functions optimized for MLX16 instruction set and given input parameter sizes
 */

/** \defgroup math_bit_operation_group Bit operation functions
 *  \ingroup math_arithmetic
 *
 *  Bit operation functions optimized for MLX16 instruction set and given input parameter sizes.
 *
 *  These operations include
 *
 *  - Bit reversal
 *  - Bit interleaving
 *  - Setting and getting single bits
 */

/** \defgroup math_parity_group Parity calculation functions
 *  \ingroup math_arithmetic
 *
 *  Parity calculation functions optimized for MLX16 instruction set and given input parameter sizes
 */

/** \defgroup math_log_exp_group Logarithmic and exponential functions
 *  \ingroup math_arithmetic
 *
 *  Logarithmic and exponential functions (base 2) functions optimized for MLX16 instruction set and given input parameter sizes
 */

/** \defgroup math_average_group Averaging functions
 *  \ingroup math_arithmetic
 *
 *  Averaging functions optimized for MLX16 instruction set and given input parameter types
 */

/** \defgroup math_abs_group (Safe) absolute value functions
 *  \ingroup math_arithmetic
 *
 *  Safe absolute value functions functions optimized given input parameter sizes
 *
 *  These functions are "safe" in the sense that they eliminate undefined behaviour in case the maximum negative
 *  value for a given input type is passed. (gives the max positive value in that case)
 */


/** \addtogroup math_arithmetic
 *  @{
 */


#ifndef MATHLIB_H_
#define MATHLIB_H_

#include <mlx16_cfg.h>

/* SW component version */
#define MLX_MATHLIB_SW_MAJOR_VERSION    2
#define MLX_MATHLIB_SW_MINOR_VERSION    16
#define MLX_MATHLIB_SW_PATCH_VERSION    0

#ifdef MLX_FORCE_C_IMPLEMENTATION
#warning "MLX_FORCE_C_IMPLEMENTATION is defined in mathlib"
#endif

/* The user should not be able to define the following macros */
#if defined (__COVERITY__) || defined (__POLYSPACE__)
#warning "COVERITY or POLYSPACE should not be explicitly defined (ignore this warning for static analysis builds)"
#endif

/* Validate MLX16-GCC version */
#if ((__MLX16_GCC_MAJOR__ == 1) && (__MLX16_GCC_MINOR__ >= 8)) || (__MLX16_GCC_MAJOR__ > 1)
/* ok */
#else
#warning "Math library requires MLX16-GCC release 1.8 or later"
#endif


#if !defined (__ASSEMBLER__)

#include <stdint.h>
#include <builtin_mlx16.h>
#include <find_bits.h>

/* Check MathLib INLINE_ASM availability;
   Choosing INLINE_ASM or ASM functions declaration */
#if defined (HAS_MLX16_COPROCESSOR) && !defined (__MATHLIB_NON_INLINE__) && !defined (UNITTEST)
/* Use inline asm implementation from mathlib_inline.h */
#define MATHLIB_INLINE static __inline__ __attribute__ ((always_inline))
#else
/* Use call-able implementation */
#define MATHLIB_INLINE
#endif

/* Check is more inline requested */
#if defined (HAS_MLX16_COPROCESSOR) && defined (__MATHLIB_EXTRA_INLINE__) && !defined (UNITTEST)
/* Use inline asm implementation from mathlib_extra_inline.h */
#define MATHLIB_EXTRA_INLINE static __inline__ __attribute__ ((always_inline))
#else
/* Use call-able implementation */
#define MATHLIB_EXTRA_INLINE
#endif


/*
 *	Multiplication
 */

MATHLIB_INLINE int16_t  mulI16_I8byI8(int8_t multiplicand, int8_t multiplier);
MATHLIB_INLINE int16_t  mulI16_I8byU8(int8_t multiplicand, uint8_t multiplier);
MATHLIB_INLINE uint16_t mulU16_U8byU8(uint8_t multiplicand, uint8_t multiplier);

MATHLIB_INLINE int16_t  mulI16hi_I16byI8(int16_t multiplicand, int8_t multiplier);
MATHLIB_INLINE uint16_t mulU16hi_U16byU8(uint16_t multiplicand, uint8_t multiplier);
MATHLIB_INLINE int16_t  mulI16hi_I16byU8(int16_t multiplicand, uint8_t multiplier);

MATHLIB_INLINE int16_t  mulI16hi_I16byI16(int16_t multiplicand, int16_t multiplier);
MATHLIB_INLINE uint16_t mulU16hi_U16byU16(uint16_t multiplicand, uint16_t multiplier);

/** Signed/unsigned integer multiplication ( 16 = high (16 * 16) )
 *
 *  @param[in]   multiplicand           signed 16-bit multiplicand
 *  @param[in]   multiplier             unsigned 16-bit multiplier
 *
 *  @return      Signed 16-bit multiplication result (16 MSBs of the 32-bit result)
 *  @ingroup     math_multiplication_group
 */
MATHLIB_EXTRA_INLINE int16_t mulI16hi_I16byU16(int16_t multiplicand, uint16_t multiplier);

MATHLIB_INLINE int32_t  mulI24_I16byI8(int16_t multiplicand, int8_t multiplier);
MATHLIB_INLINE uint32_t mulU24_U16byU8(uint16_t multiplicand, uint8_t multiplier);
MATHLIB_INLINE int32_t  mulI24_I16byU8(int16_t multiplicand, uint8_t multiplier);

MATHLIB_INLINE uint32_t mulU32_U16byU16(uint16_t multiplicand, uint16_t multiplier);
MATHLIB_INLINE int32_t  mulI32_I16byI16(int16_t multiplicand, int16_t multiplier);

/** Signed/unsigned integer multiplication ( 32 = 16 * 16 )
 *
 *  @param[in]   multiplicand           signed 16-bit multiplicand
 *  @param[in]   multiplier             unsigned 16-bit multiplier
 *
 *  @return      signed 32-bit multiplication result
 *  @ingroup     math_multiplication_group
 */
MATHLIB_EXTRA_INLINE int32_t mulI32_I16byU16(int16_t multiplicand, uint16_t multiplier);

/** Signed integer multiplication (32 = low(32 * 16))
 *
 *  @param[in]  multiplicand  Signed 32-bit multiplicand
 *  @param[in]  multiplier    Signed 16-bit multiplier
 *
 *  @return     Signed 32-bit result (32 LSBs of the 48-bit result)
 *  @ingroup    math_multiplication_group
 */
MATHLIB_EXTRA_INLINE int32_t mulI32lo_I32byI16(int32_t multiplicand, int16_t multiplier);

/** Signed/Unsigned integer multiplication (32 = low(32 * 16))
 *
 *  @param[in]  multiplicand  Signed 32-bit multiplicand
 *  @param[in]  multiplier    Unsigned 16-bit multiplier
 *
 *  @return     Signed 32-bit result (32 LSBs of the 48-bit result)
 *  @ingroup    math_multiplication_group
 */
MATHLIB_EXTRA_INLINE int32_t mulI32lo_I32byU16(int32_t multiplicand, uint16_t multiplier);

/** Unsigned integer multiplication (32 = low(32 * 16))
 *
 *  @param[in]  multiplicand  Unsigned 32-bit multiplicand
 *  @param[in]  multiplier    Unsigned 16-bit multiplier
 *
 *  @return     Unsigned 32-bit result (32 LSBs of the 48-bit result)
 *  @ingroup    math_multiplication_group
 */
MATHLIB_EXTRA_INLINE uint32_t mulU32lo_U32byU16(uint32_t multiplicand, uint16_t multiplier);

/** Signed integer multiplication (32 = high(32 * 16))
 *
 *  @param[in]  multiplicand  Signed 32-bit multiplicand
 *  @param[in]  multiplier    Signed 16-bit multiplier
 *
 *  @return     Signed 32-bit result (32 MSBs of the 48-bit result)
 *  @ingroup    math_multiplication_group
 */
int32_t  mulI32hi_I32byI16(int32_t multiplicand, int16_t multiplier);

/** Signed/Unsigned integer multiplication (32 = high(32 * 16))
 *
 *  @param[in]  multiplicand  Signed 32-bit multiplicand
 *  @param[in]  multiplier    Unsigned 16-bit multiplier
 *
 *  @return     Signed 32-bit result (32 MSBs of the 48-bit result)
 *  @ingroup    math_multiplication_group
 */
int32_t  mulI32hi_I32byU16(int32_t multiplicand, uint16_t multiplier);

/** Unsigned integer multiplication (32 = high(32 * 16))
 *
 *  @param[in]  multiplicand  Unsigned 32-bit multiplicand
 *  @param[in]  multiplier    Unsigned 16-bit multiplier
 *
 *  @return     Unsigned 32-bit result (32 MSBs of the 48-bit result)
 *  @ingroup    math_multiplication_group
 */
uint32_t mulU32hi_U32byU16(uint32_t multiplicand, uint16_t multiplier);

MATHLIB_INLINE int16_t  mulQ15_Q15byQ15(int16_t multiplicand, int16_t multiplier);

/*
 *	Division
 */

/** Unsigned integer division ( 32 = 32 / 16 )
 *
 *  @param[in]  dividend           unsigned 32-bit dividend
 *  @param[in]  divisor            unsigned 16-bit divisor
 *
 *  @return     Unsigned 32-bit quotient
 *
 *  @warning    The result is undefined for a zero divisor. There is no internal
 *              check on the validity (i.e non-zero) of the divisor
 *  @ingroup    math_division_group
 */
MATHLIB_EXTRA_INLINE uint32_t divU32_U32byU16(uint32_t dividend, uint16_t divisor);

/** Signed integer division ( 32 = 32 / 16 )
 *
 *  @param[in]  dividend           signed 32-bit dividend
 *  @param[in]  divisor            signed 16-bit divisor
 *
 *  @return     Signed 32-bit quotient
 *
 *  @warning    The result is undefined for a zero divisor. There is no internal
 *              check on the validity (i.e non-zero) of the divisor
 *  @ingroup    math_division_group
 */
MATHLIB_EXTRA_INLINE int32_t divI32_I32byI16(int32_t dividend, int16_t divisor);

/** Signed / unsigned integer division ( 32 = 32 / 16 )
 *
 *  @param[in]  dividend           signed 32-bit dividend
 *  @param[in]  divisor            unsigned 16-bit divisor
 *
 *  @return     Signed 32-bit quotient
 *
 *  @warning    The result is undefined for a zero divisor. There is no internal
 *              check on the validity (i.e non-zero) of the divisor
 *  @ingroup    math_division_group
 */
MATHLIB_EXTRA_INLINE int32_t divI32_I32byU16(int32_t dividend, uint16_t divisor);

MATHLIB_INLINE uint16_t divU16_U32byU16(uint32_t dividend, uint16_t divisor);

/** Signed integer division ( 16 = 32 / 16 )
 *
 *  @param[in]  dividend           signed 32-bit dividend
 *  @param[in]  divisor            signed 16-bit divisor
 *
 *  @return     Signed 16-bit quotient (16 LSBs of the division result)
 *
 *  @warning    The result of the division is not guaranteed to fit the return
 *              type. This function does not perform any overflow detection
 *              or clipping of any kind.
 *  @warning    The result is undefined for a zero divisor. There is no internal
 *              check on the validity (i.e non-zero) of the divisor
 *  @ingroup    math_division_group
 */
MATHLIB_EXTRA_INLINE int16_t  divI16_I32byI16(int32_t dividend, int16_t divisor);

/** Signed/Unsigned integer division ( 16 = 32 / 16 )
 *
 *  @param[in]  dividend    Signed 32-bit dividend
 *  @param[in]  divisor    Unsigned 16-bit divisor
 *
 *  @return     Signed 16-bit quotient (16 LSBs of the division result)
 *
 *  @warning    The result of the division is not guaranteed to fit the return
 *              type. This function does not perform any overflow detection
 *              or clipping of any kind.
 *  @warning    The result is undefined for a zero divisor. There is no internal
 *              check on the validity (i.e non-zero) of the divisor
 *  @ingroup    math_division_group
 */
MATHLIB_EXTRA_INLINE int16_t divI16_I32byU16(int32_t dividend, uint16_t divisor);

/** Unsigned integer division: ( 16 = 16 * 2^16 / 16 )
 *
 *  @param[in]  n     Unsigned 16-bit dividend
 *  @param[in]  d     Unsigned 16-bit divisor
 *
 *  @return     Unsigned 16-bit quotient
 *
 *  @warning    The dividend 'n' should be less than divisor 'd'.
 *              If not, incorrect calculation occurs due to overflow. There is
 *              no internal check for overflow inside the function.
 *  @warning    The result is undefined for a zero divisor. There is no internal
 *              check on the validity (i.e non-zero) of the divisor
 *  @ingroup    math_division_group
 */
uint16_t divU16hi_U16byU16(uint16_t n, uint16_t d);

/** Signed integer division: ( 16 = 16 * 2^16 / 16 )
 *
 *  @param[in]  n     Signed 16-bit dividend
 *  @param[in]  d     Signed 16-bit divisor
 *
 *  @return     Signed 16-bit quotient
 *
 *  @warning    The dividend 'n' should be less than divisor 'd'.
 *              If not, incorrect calculation occurs due to overflow. There is
 *              no internal check for overflow inside the function.
 *  @warning    The result is undefined for a zero divisor. There is no internal
 *              check on the validity (i.e non-zero) of the divisor
 *  @ingroup    math_division_group
 */
int16_t  divI16hi_I16byI16(int16_t n, int16_t d);

/** Signed/Unsigned integer division: ( 16 = 16 * 2^16 / 16 )
 *
 *  @param[in]  n     Signed 16-bit dividend
 *  @param[in]  d     Unsigned 16-bit divisor
 *
 *  @return     Signed 16-bit quotient
 *
 *  @warning    The dividend 'n' should be less than divisor 'd'.
 *              If not, incorrect calculation occurs due to overflow. There is
 *              no internal check for overflow inside the function.
 *  @warning    The result is undefined for a zero divisor. There is no internal
 *              check on the validity (i.e non-zero) of the divisor
 *  @ingroup    math_division_group
 */
int16_t  divI16hi_I16byU16(int16_t n, uint16_t d);

/** Unsigned division rounded to nearest integer using "round half away from zero" (or "round half towards infinity")
 *
 *  @param[in]  dividend           unsigned 16-bit dividend
 *  @param[in]  divisor            unsigned 16-bit divisor
 *
 *  @return     unsigned 16-bit quotient = round(a/b)
 *
 *  @warning    The result is undefined for a zero divisor. There is no internal
 *              check on the validity (i.e non-zero) of the divisor
 *  @ingroup    math_division_group
 */
uint16_t div_rh_U16_U16byU16(uint16_t dividend, uint16_t divisor);

/** Signed division rounded to nearest integer using "round half away from zero" (or "round half towards infinity")
 *
 *  @param[in]  dividend           signed 16-bit dividend
 *  @param[in]  divisor            unsigned 16-bit divisor
 *
 *  @return     signed 16-bit quotient = round(a/b)
 *
 *  @warning    The result is undefined for a zero divisor. There is no internal
 *              check on the validity (i.e non-zero) of the divisor
 *  @ingroup    math_division_group
 */
int16_t div_rh_I16_I16byU16(int16_t dividend, uint16_t divisor);

/** Signed division rounded to nearest integer using "round half away from zero" (or "round half towards infinity")
 *
 *  @param[in]  dividend           signed 16-bit dividend
 *  @param[in]  divisor            signed 16-bit divisor
 *
 *  @return     signed 32-bit quotient = round(a/b)
 *
 *  @warning    The result is undefined for a zero divisor. There is no internal
 *              check on the validity (i.e non-zero) of the divisor
 *  @note       The result is represented with 32 bits for the special case of INT16_MIN / -1
 *  @ingroup    math_division_group
 */
int32_t div_rh_I32_I16byI16(int16_t dividend, int16_t divisor);

/** Unsigned integer division ( 8 = 8 / 8 )
 *
 *  @param[in]  n     Unsigned 8-bit dividend
 *  @param[in]  d     Unsigned 8-bit divisor
 *
 *  @return     Unsigned 8-bit quotient
 *
 *  @warning    The result is undefined for a zero divisor. There is no internal
 *              check on the validity (i.e non-zero) of the divisor
 *  @ingroup    math_division_group
 */
uint8_t  divU8_U8byU8(uint8_t n, uint8_t d);

/** Signed integer division ( 8 = 8 / 8 )
 *
 *  @param[in]  n     Signed 8-bit dividend
 *  @param[in]  d     Signed 8-bit divisor
 *
 *  @return     Signed 8-bit quotient
 *
 *  @warning    The result is undefined for a zero divisor. There is no internal
 *              check on the validity (i.e non-zero) of the divisor
 *  @ingroup    math_division_group
 */
int8_t   divI8_I8byI8(int8_t n, int8_t d);

/** Signed/Unsigned integer division ( 8 = 8 / 8 )
 *
 *  @param[in]  n     Signed 8-bit dividend
 *  @param[in]  d     Unsigned 8-bit divisor
 *
 *  @return     Signed 8-bit quotient
 *
 *  @warning    The result is undefined for a zero divisor. There is no internal
 *              check on the validity (i.e non-zero) of the divisor
 *  @ingroup    math_division_group
 */
int8_t   divI8_I8byU8(int8_t n, uint8_t d);

/** Unsigned integer division: ( 8 = 8 * 2^8 / 8 )
 *
 *  @param[in]  n     Unsigned 8-bit dividend
 *  @param[in]  d     Unsigned 8-bit divisor
 *
 *  @return     Unsigned 8-bit quotient
 *
 *  @warning    The dividend 'n' should be less than divisor 'd'.
 *              If not, incorrect calculation occurs due to overflow. There is
 *              no internal check for overflow inside the function.
 *  @warning    The result is undefined for a zero divisor. There is no internal
 *              check on the validity (i.e non-zero) of the divisor
 *  @ingroup    math_division_group
 */
uint8_t  divU8hi_U8byU8(uint8_t n, uint8_t d);

/*
 *	Power
 */

/** Function for calculating the square root of an input number
 *
 *  @param[in]  a   The 16-bit number for which to calculate the square root
 *  @return     The calculated square root
 *              (the greatest integer less than or equal to the exact square root)
 *
 *  @ingroup   math_sqrt_group
 *  @internal
 *      - On MLX16 (-O3), min 8*3+9=33, max 8*14+9=121 instructions
 *      - Can optimize on MLX16-x8 by using fsb instruction to find first scaling
 */
uint16_t isqrt16 (uint16_t a);

/** Function for calculating the square root of an input number
 *
 *  @param[in]  a   The 32-bit number for which to calculate the square root
 *  @return     The calculated square root
 *              (the greatest integer less than or equal to the exact square root)
 *  @ingroup    math_sqrt_group
 */
uint16_t isqrt32 (uint32_t a);

/*
 *	log/exp
 */

/** Function for calculating log2 of an input number
 *
 *  Calculates the integer part of the logarithm with base 2 of the
 *  input argument x>0.
 *  I.e. Returns the greatest integer n for which 2^n <= x.
 *
 *  @param[in]  v         Input parameter (16-bit) to calculate log2 from.
 *
 *  @retval     [0:15]    The log2 result of the input parameter.
 *  @retval     0xFFFF    Return value for input argument with value 0.
 *  @ingroup    math_log_exp_group
 */
uint16_t ilog2_U16 (uint16_t v);

/** Function for calculating log2 of an input number
 *
 *  Calculates the integer part of the logarithm with base 2 of the
 *  input argument x>0.
 *  I.e. Returns the greatest integer n for which 2^n <= x.
 *
 *  @param[in]  v         Input parameter (32-bit) to calculate log2 from.
 *  @retval     [0:31]    The log2 result of the input parameter.
 *  @retval     0xFFFF    Return value for input argument with value 0.
 *  @ingroup    math_log_exp_group
 */
uint16_t ilog2_U32 (uint32_t v);

/** Function for calculating the integer exponent with base 2 of an input number
 *
 *  @param[in]  v   The exponent used for the calculation (0 <= v <= 15)
 *  @return     The 16-bit result of the exponent calculation
 *  @ingroup    math_log_exp_group
 */
uint16_t iexp2_U16 (uint16_t v);

/** Function for calculating the integer exponent with base 2 of an input number
 *
 *  @param[in]  v   The exponent used for the calculation (0 <= v <= 31)
 *  @return     The 32-bit result of the exponent calculation
 *  @ingroup    math_log_exp_group
 */
uint32_t iexp2_U32 (uint16_t v);

/*
 *	Trigonometric
 */

/** Unsigned 16-bit sine function
 *
 *  This function calculates (an approximation of) sin (x) where x is an angle
 *  value represented as a 16 bit unsigned fraction of 2&pi;.
 *  The result is represented as a 16 bit signed fraction (Q15 format).
 *
 *  A couple of example values illustrating the function behaviour:
 *
 *  input(hex) | Equivalent angle (rad) | output
 *  ---------- | ---------------------- | -----
 *  0x0000     | 0                      | 0
 *  0x2000     | &pi;/4                 | 0x5A82 (23170)
 *  0x4000     | &pi;/2                 | 0x7FFF (32767)
 *  0x8000     | &pi;                   | 0
 *  0xC000     | 3&pi;/2                | 0x8001 (-32767)
 *
 *  @param[in]  in  The input angle, as 16 bit unsigned fraction of 2&pi;.
 *                  0 < in < 65536=2&pi;
 *
 *  @return     16-bit signed fraction (Q15 format)
 *  @ingroup    math_trigonometry_group
 */
int16_t sinU16 (uint16_t in);

/** Signed 16-bit sine function
 *
 *  This function calculates (an approximation of) sin (x) where x is an angle
 *  value represented as a 16 bit signed fraction of 2&pi;.
 *  The result is represented as a 16 bit signed fraction (Q15 format).
 *
 *  A couple of example values illustrating the function behaviour:
 *
 *  Input(hex) | Equivalent angle (rad) | Output
 *  ---------- | ---------------------- | -----
 *  0x0000     | 0                      | 0
 *  0x2000     | &pi;/4                 | 0x5A82 (23170)
 *  0x4000     | &pi;/2                 | 0x7FFF (32767)
 *  0x8000     | &pi;                   | 0
 *  0xC000     | 3&pi;/2                | 0x8001 (-32767)
 *
 *  @param[in]  x   The input angle, as 16 bit signed fraction of 2&pi;.
 *                  32768(-1->-&pi;) < in < 32768(1->&pi;)
 *
 *  @return     16-bit signed fraction (Q15 format)
 *  @ingroup    math_trigonometry_group
 */
int16_t sinI16 (int16_t x);

/** Unsigned 16-bit cosine function
 *
 *  This function calculates (an approximation of) cos(x) where x is an angle
 *  value represented as a 16 bit unsigned fraction of 2&pi;.
 *  The result is represented as a 16 bit signed fraction (Q15 format).
 *
 *  A couple of example values illustrating the function behaviour:
 *
 *  Input(hex) | Equivalent angle (rad) | Output
 *  ---------- | ---------------------- | -----
 *  0x0000     | 0                      | 0x7FFF (32767)
 *  0x2000     | &pi;/4                 | 0x5A82 (23170)
 *  0x4000     | &pi;/2                 | 0
 *  0x8000     | &pi;                   | 0x8001 (-32767)
 *  0xC000     | 3&pi;/2                | 0
 *
 *  @param[in]  x   The input angle, as 16 bit unsigned fraction of 2&pi;.
 *                  0 < in < 65536=2&pi;
 *
 *  @return     16-bit signed fraction (Q15 format)
 *  @ingroup    math_trigonometry_group
 */
int16_t cosU16 (uint16_t x);

/** Signed 16-bit cosine function
 *
 *  This function calculates (an approximation of) cos(x) where x is an angle
 *  value represented as a 16 bit signed fraction of 2&pi;.
 *  The result is represented as a 16 bit signed fraction (Q15 format).
 *
 *  A couple of example values illustrating the function behaviour:
 *
 *  Input(hex) | Equivalent angle (rad) | Output
 *  ---------- | ---------------------- | -----
 *  0x0000     | 0                      | 0x7FFF (32767)
 *  0x2000     | &pi;/4                 | 0x5A82 (23170)
 *  0x4000     | &pi;/2                 | 0
 *  0x8000     | &pi;                   | 0x8001 (-32767)
 *  0xC000     | 3&pi;/2                | 0
 *
 *  @param[in]  x   The input angle, as 16 bit signed fraction of 2&pi;.
 *                  32768(-1->-&pi;) < in < 32768(1->&pi;)
 *
 *  @return     16-bit signed fraction (Q15 format)
 *  @ingroup    math_trigonometry_group
 */
int16_t cosI16 (int16_t x);

/** Unsigned tangent function
 *
 *  This function calculates (an approximation of) tan(x) where x is an angle
 *  value represented as a 16 bit unsigned fraction of 2&pi;.
 *  The result is represented as a 32 bit signed value with 16 fractional bits
 *  (Q15.16 format)
 *
 *  A couple of example values illustrating the function behaviour:
 *
 *  Input(hex) | Equivalent angle (rad) | Output
 *  ---------- | ---------------------- | -----
 *  0x0000     | 0                      | 0
 *  0x2000     | &pi;/4                 | 0x00010000 (65536)
 *  0x4000     | &pi;/2                 | 0x80000001 (-2147483647)
 *  0x6000     | 3&pi;/4                | 0xFFFF0000 (-65536)
 *  0x8000     | &pi;                   | 0
 *  0xA000     | 5&pi;/4                | 0x00010000 (65536)
 *  0xC000     | 3&pi;/2                | 0x80000001 (-2147483647)
 *  0xE000     | 7&pi;/4                | 0xFFFF0000 (-65536)
 *
 *  @param[in]  x   The input angle, as 16 bit unsigned fraction of 2&pi;.
 *                  0 < in < 65536(1->2&pi;)
 *
 *  @return     32-bit signed fraction (Q15.16 format)
 *  @ingroup    math_trigonometry_group
 */
int32_t tanU16 (uint16_t x);

/** Signed tangent function
 *
 *  This function calculates (an approximation of) tan(x) where x is an angle
 *  value represented as a 16 bit signed fraction of 2&pi;.
 *  The result is represented as a 32 bit signed value with 16 fractional bits
 *  (Q15.16 format)
 *
 *  A couple of example values illustrating the function behaviour:
 *
 *  Input(hex) | Equivalent angle (rad) | Output
 *  ---------- | ---------------------- | -----
 *  0x0000     | 0                      | 0
 *  0x2000     | &pi;/4                 | 0x00010000 (65536)
 *  0x4000     | &pi;/2                 | 0x80000001 (-2147483647)
 *  0x6000     | 3&pi;/4                | 0xFFFF0000 (-65536)
 *  0x8000     | &pi;                   | 0
 *  0xA000     | 5&pi;/4                | 0x00010000 (65536)
 *  0xC000     | 3&pi;/2                | 0x80000001 (-2147483647)
 *  0xE000     | 7&pi;/4                | 0xFFFF0000 (-65536)
 *
 *  @param[in]  x   The input angle, as 16 bit signed fraction of 2&pi;
 *                  32768(-1->-&pi;) < in < 32768(1->&pi;)
 *
 *  @return     32-bit signed fraction (Q15.16 format)
 *  @ingroup    math_trigonometry_group
 */
int32_t tanI16 (int16_t x);

/** Unsigned atan2 function
 *
 *  This function calculates (an approximation of) arctan (y / x) where y and x
 *  are two 16 bit integer numbers.
 *  The result is an angle value between 0 and &pi;/2, represented as a 16 bit
 *  signed fraction of 2&pi; i.e. Q15 format.
 *
 *  The following table illustrates the input-output characteristics:
 *
 *  y (unsigned)  | x (unsigned)  | atan2 (y/x) | Equivalent angle (rad)
 *  ------------- | ------------- | ----------- | ----------------------
 *  0             | 0             | 0x0000      | 0
 *  1             | 1             | 0x2000      | &pi;/4
 *  0xFFFF        | 0xFFFF        | 0x2000      | &pi;/4
 *  0x8000        | 0x4000        | 0x2D1B      | 1.106
 *  1             | 0             | 0x4000      | &pi;/2
 *  0xFFFF        | 0             | 0x4000      | &pi;/2
 *
 *  @note       As only ranges between 0 and &pi;/2 are supported and the output
 *              encoding is defined as a fraction of 2&pi;, only a quarter of the
 *              16-bit output range is used.
 *  @note       atan2U16(0, 0) is definded as 0.
 *
 *  @param[in]  y     Y coordinate
 *  @param[in]  x     X coordinate
 *
 *  @return     The output angle, as 16 bit signed fraction of 2&pi;.
 *             32768(-1->-&pi;) < out < 32768(1->&pi;)
 *  @ingroup    math_trigonometry_group
 */
int16_t atan2U16 (uint16_t y, uint16_t x);

/** Signed atan2 function
 *
 *  This function calculates (an approximation of) arctan (y / x) where y and x
 *  are two 16 bit integer numbers.
 *  The result is an angle value between 0 and 2&pi;, represented as a 16 bit
 *  signed fraction of 2&pi; i.e. Q15 format.
 *
 *  y (signed)  | x (signed)  | atan2 (y/x) | Equivalent angle (rad)
 *  ----------- | ----------- | ----------- | ----------------------
 *  0           | 0           | 0x0000      | 0
 *  0           | 1           | 0x0000      | 0
 *  1           | 1           | 0x2000      | &pi;/4
 *  1           | 0           | 0x4000      | &pi;/2
 *  1           | -1 (0xFFFF) | 0x6000      | 3&pi;/4
 *  0           | -1 (0xFFFF) | 0x8000      | &pi;
 *  -1 (0xFFFF) | -1 (0xFFFF) | 0xA000      | 5&pi;/4
 *  -1 (0xFFFF) | 0           | 0xC000      | 3&pi;/2
 *  -1 (0xFFFF) | 1           | 0xE000      | 7&pi;/4
 *
 *  @note       atan2I16(0, 0) is definded as 0.
 *
 *  @param[in]  y     Y coordinate
 *  @param[in]  x     X coordinate
 *
 *  @return     The output angle, as 16 bit signed fraction of 2&pi;.
 *             32768(-1->-&pi;) < out < 32768(1->&pi;)
 *  @ingroup    math_trigonometry_group
 */
int16_t atan2I16 (int16_t y, int16_t x);

/*
 *	Other
 */

uint32_t rand32(uint32_t seed);

/** initialization of the LFSR
 *
 *  @param[in] seed   The seed for the LFSR implementation.
 *                    Should be different from zero
 *  @ingroup   math_lfsr_group
 */
void   init_lfsr16(uint16_t seed);

/** initialization of the LFSR
 *
 *  @param[in] seed   The seed for the LFSR implementation.
 *                    Should be different from zero
 *  @ingroup   math_lfsr_group
 */
void   init_lfsr32(uint32_t seed);

/** 16 bit Linear Feedback shift register implementation with polynome 0xB400
 *
 *  One iteration of 16 bit Linear Feedback shift register, e.g. for use with
 *  pseudo random number generation.
 *  See http:/en.wikipedia.org/wiki/Linear_feedback_shift_register (Galois LFSR)
 *
 *  http:/en.wikipedia.org/wiki/Scrambler
 *
 *  16bit polynomial for LFSR with period 2^16-1: 0xB400
 *
 *  Other polynomials:
 *  http:/en.wikipedia.org/wiki/Linear_feedback_shift_register
 *  http:/www.xilinx.com/support/documentation/application_notes/xapp052.pdf
 *
 *  @return     LFSR iteration result
 *
 *  @note   The lfsr implementation uses the *normal* polynomial representation.
 *          As an <B>example</B>: a polynomial with hex code 0xB400 corresponds to:
 *          - taps: 16, 14, 13, 11
 *          - \f$ x^{16} + x^{14} + x^{13} + x^{11} + 1 \f$
 *  @ingroup   math_lfsr_group
 */
uint16_t lfsr16(void);

/** 16 bit Linear Feedback shift register implementation with polynome 0xB400
 *
 *  One iteration of 16 bit Linear Feedback shift register, e.g. for use with
 *  pseudo random number generation.
 *  See http:/en.wikipedia.org/wiki/Linear_feedback_shift_register (Galois LFSR)
 *
 *  http:/en.wikipedia.org/wiki/Scrambler
 *
 *  16bit polynomial for LFSR with period 2^16-1: 0xB400
 *
 *  Other polynomials:
 *  http:/en.wikipedia.org/wiki/Linear_feedback_shift_register
 *  http:/www.xilinx.com/support/documentation/application_notes/xapp052.pdf
 *
 *  @return     LFSR iteration result
 *
 *  @note   The lfsr implementation uses the *normal* polynomial representation.
 *          As an <B>example</B>: a polynomial with hex code 0xB400 corresponds to:
 *          - taps: 16, 14, 13, 11
 *          - \f$ x^{16} + x^{14} + x^{13} + x^{11} + 1 \f$
 *  @ingroup   math_lfsr_group
 */
uint16_t lfsr16_B400 (void);

/** 32 bit Linear Feedback shift register implementation with polynome 0x80200003
 *
 *  One iteration of 32 bit Linear Feedback shift register, e.g. for use with
 *  pseudo random number generation.
 *  See http:/en.wikipedia.org/wiki/Linear_feedback_shift_register (Galois LFSR)
 *
 *  http:/en.wikipedia.org/wiki/Scrambler
 *
 *  32bit polynomial for LFSR with period 2^32-1: 0x80200003
 *
 *  Other polynomials:
 *  http:/en.wikipedia.org/wiki/Linear_feedback_shift_register
 *  http:/www.xilinx.com/support/documentation/application_notes/xapp052.pdf
 *
 *  @return     LFSR iteration result
 *  @ingroup   math_lfsr_group
 */
uint32_t lfsr32(void);

/** 32 bit Linear Feedback shift register implementation with polynome 0x0000000C
 *
 *  One iteration of 32 bit Linear Feedback shift register, e.g. for use with
 *  pseudo random number generation.
 *  See http:/en.wikipedia.org/wiki/Linear_feedback_shift_register (Galois LFSR)
 *
 *  http:/en.wikipedia.org/wiki/Scrambler
 *
 *  32bit polynomial for LFSR with period 2^32-1: 0x0000000C
 *
 *  Other polynomials:
 *  http:/en.wikipedia.org/wiki/Linear_feedback_shift_register
 *  http:/www.xilinx.com/support/documentation/application_notes/xapp052.pdf
 *
 *  @return     LFSR iteration result
 *  @ingroup   math_lfsr_group
 */
uint32_t lfsr32_0000000C (void);

/** 32 bit Linear Feedback shift register implementation with polynome 0x80200003
 *
 *  One iteration of 32 bit Linear Feedback shift register, e.g. for use with
 *  pseudo random number generation.
 *  See http:/en.wikipedia.org/wiki/Linear_feedback_shift_register (Galois LFSR)
 *
 *  http:/en.wikipedia.org/wiki/Scrambler
 *
 *  32bit polynomial for LFSR with period 2^32-1: 0x80200003
 *
 *  Other polynomials:
 *  http:/en.wikipedia.org/wiki/Linear_feedback_shift_register
 *  http:/www.xilinx.com/support/documentation/application_notes/xapp052.pdf
 *
 *  @return     LFSR iteration result
 *  @ingroup   math_lfsr_group
 */
uint32_t lfsr32_80200003 (void);

/** Parity-check function for the 4 LSBs of an input value
 *
 *  See also graphics.stanford.edu/~seander/bithacks.html
 *
 *  @param[in]  in  Input value
 *  @retval     0   Even number of set bits in input parameter
 *  @retval     1   Odd number of set bits in input parameter
 *  @ingroup   math_parity_group
 */
uint16_t parity4 (uint8_t in);

/** Parity-check function for an 8-bit input value
 *
 *  See also graphics.stanford.edu/~seander/bithacks.html
 *
 *  @param[in]  in  Input value
 *  @retval     0   Even number of set bits in input parameter
 *  @retval     1   Odd number of set bits in input parameter
 *
 *  @internal
 *  - Total code size can be reduced when all in same file and using assembler
 *    labels at the various entry points
 *  - 12 Instruction
 *  @endinternal
 *  @ingroup   math_parity_group
 */
uint16_t parity8 (uint8_t in);

/** Parity-check function for a 16-bit input value
 *
 *  See also graphics.stanford.edu/~seander/bithacks.html
 *
 *  @param[in]  in  Input value
 *  @retval     0   Even number of set bits in input parameter
 *  @retval     1   Odd number of set bits in input parameter
 *  @ingroup    math_parity_group
 */
uint16_t parity16(uint16_t in);

/** Parity-check function for a 32-bit input value
 *
 *  See also graphics.stanford.edu/~seander/bithacks.html
 *
 *  @param[in]  v   Input value
 *  @retval     0   Even number of set bits in input parameter
 *  @retval     1   Odd number of set bits in input parameter
 *  @ingroup    math_parity_group
 */
uint16_t parity32(uint32_t v);

/** Function for bit-reversing the 4 LSBs of the input value
 *
 *  @param[in]  x   Input value
 *  @return     Reversed 4-bit result
 *
 *  @warning    The 4 MSBs of the result are always 0, regardless of state of the
 *              4 MSBs of the input value.
 *  @ingroup    math_bit_operation_group
 */
uint8_t   bitrev4 (uint8_t x);

/** Function for bit-reversing a 8-bit input value
 *
 *  @param[in]  x   Input value
 *  @return     Reversed 8-bit result
 *  @ingroup    math_bit_operation_group
 */
uint8_t   bitrev8 (uint8_t x);

/** Function for bit-reversing a 16-bit input value
 *
 *  @param[in]  in  Input value
 *  @return     Reversed 16-bit result
 *  @ingroup    math_bit_operation_group
 */
uint16_t  bitrev16(uint16_t in);

/** Function for interleaving 4 bits of x and y
 *
 *  Bits of x are in even positions and bits from y in the odd positions.
 *  See also http:/graphics.stanford.edu/~seander/bithacks.html#InterleaveTableObvious
 *
 *  @param[in]  x   First number to interleave (ends up in the even positions)
 *  @param[in]  y   First number to interleave (ends up in the odd positions)
 *  @return     Interleaved 8-bit result
 *  @ingroup    math_bit_operation_group
 */
uint8_t   interleave4 (uint8_t x, uint8_t y);

/** Function for interleaving 8 bits of x and y
 *
 *  Bits of x are in even positions and bits from y in the odd positions.
 *  See also http:/graphics.stanford.edu/~seander/bithacks.html#InterleaveTableObvious
 *
 *  @param[in]  x   First number to interleave (ends up in the even positions)
 *  @param[in]  y   First number to interleave (ends up in the odd positions)
 *  @return     Interleaved 16-bit result
 *  @ingroup    math_bit_operation_group
 */
uint16_t  interleave8 (uint8_t x, uint8_t y);

/** Function for interleaving 16 bits of x and y
 *
 *  Bits of x are in even positions and bits from y in the odd positions.
 *  See also http:/graphics.stanford.edu/~seander/bithacks.html#InterleaveTableObvious
 *
 *  @param[in]  x   First number to interleave (ends up in the even positions)
 *  @param[in]  y   First number to interleave (ends up in the odd positions)
 *  @return     Interleaved 32-bit result
 *  @ingroup    math_bit_operation_group
 */
uint32_t  interleave16(uint16_t x, uint16_t y);

/** Safe 8 bit absolute function
 *
 *  This "safe" abs function alternative will calculate the absolute value
 *  of a given signed input value, but prevent undefined behavior when the input
 *  value is equal to INT8_MIN
 *
 *  @param[in]  x    The signed value to calculate the absolute for
 *
 *  @return     The absolute value of the input value, clipped to INT8_MAX
 *  @ingroup    math_abs_group
 */
MATHLIB_EXTRA_INLINE int8_t safeabs8(int8_t x);

/** Safe 16 bit absolute function
 *
 *  This "safe" abs function alternative will calculate the absolute value
 *  of a given signed input value, but prevent undefined behavior when the input
 *  value is equal to INT16_MIN
 *
 *  @param[in]  x    The signed value to calculate the absolute for
 *
 *  @return     The absolute value of the input value, clipped to INT16_MAX
 *  @ingroup    math_abs_group
 */
MATHLIB_EXTRA_INLINE int16_t safeabs16(int16_t x);

#ifdef MATHLIB_HAS_DEPRECATED_API
#include "mathlib_deprecated.h"
#endif

/* Check MathLib INLINE_ASM availability */
#if defined (HAS_MLX16_COPROCESSOR) && !defined (__MATHLIB_NON_INLINE__)
#ifndef UNITTEST
#include "mathlib_inline.h"
#endif
#endif

#if defined (HAS_MLX16_COPROCESSOR) && defined (__MATHLIB_EXTRA_INLINE__)
#ifndef UNITTEST
#include "mathlib_extra_inline.h"
#endif
#endif

#endif /* ! __ASSEMBLER__ */

#endif /* MATHLIB_H_ */
/** @}*/
