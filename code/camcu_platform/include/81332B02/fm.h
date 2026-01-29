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
 * @ingroup fast_math
 * @defgroup fast_math Fast math (token "fm")
 *
 * @details API description
 */

#ifndef FM_H_
#define FM_H_

#include <stdint.h>
#include "mathlib.h"
#include "compiler_abstraction.h"

/* ---------------------------------------------
 * Public Defines
 * --------------------------------------------- */

/** @name Fast math version */
/** Major */
#define FAST_MATH_VERSION_MAJOR 1
/** Minor */
#define FAST_MATH_VERSION_MINOR 2
/** Patch */
#define FAST_MATH_VERSION_PATCH 1
/** Release status (0: released; 1: under development) */
#define FAST_MATH_VERSION_WIP 0
/**@}*/

#ifndef HAS_MLX16_COPROCESSOR
#error "Fast Math library requires MLX16 co-processor be available"
#endif

/* ---------------------------------------------
 * Public Defines
 * --------------------------------------------- */

/** @def FM_EXEC_MEMORY
 * Allows to modify the target memory which is used for Fast Math functions execution.
 * Related LUTs placement is not affected by this definition.
 */
#ifdef FM_EXEC_FROM_RAM
#define FM_EXEC_MEMORY __attribute__((section(".ramfunc")))
#else
#define FM_EXEC_MEMORY
#endif

#ifndef FM_FULL_SIN_COS_LUT
/** The selection affects LUT size for sin/cos.
 * Value '1' selects LUT generated for full period which takes 2.5KBytes.
 * Value '0' selects LUT for quarter period is used which takes 0.5KBytes.
 * The selection also affects type of functions which are going to be used
 * and their speed.
 */
#define FM_FULL_SIN_COS_LUT 1
#endif

/** Size of the ATAN Look-up table of 256 points (256 + 2) */
#define FM_ATAN_TABLE_SIZE 258u
/** Size of the sine/cosine Look-up table of ~0.5 KB (256 + 2 points) */
#define FM_SIN_TABLE_0_5_SIZE 258u
/** Size of the sine/cosine Look-up table of ~2.5 KB (1280 + 2 points) */
#define FM_SIN_TABLE_2_5_SIZE 1282u

/* ---------------------------------------------
 * Global Function Declarations
 * --------------------------------------------- */

/** @name Sine and Cosine functions based on direct LUT */


/** Common comment block */
#define FM_SIN_U16_DIRECT_LUT_COMMENT \
/** Unsigned 16-bit sine function.
   This function calculates (a high-speed approximation of) \f$sin(x)\f$ where \a x is an angle
   value represented as a 16 bit unsigned fraction of \f$2\pi\f$.
   The result is represented as a 16 bit signed fraction (Q15 format).
   Function takes output value directly from the LUT w/o performing interpolation.
   Absolute error of the result is \f$\pm\f$ 101.
   The LUT size which is used by the function is configured by ::FM_FULL_SIN_COS_LUT.
   @param[in]  theta  The input angle, as 16 bit unsigned fraction of \f$2\pi\f$.
                      0 < in < 65536=\f$2\pi\f$
   @return     16-bit signed fraction (Q15 format) */
/* \fn int16_t fm_SinU16DirectLutInlined(uint16_t theta)
 * Inlined implementation (optimal memory usage, worse execution speed). */FM_SIN_U16_DIRECT_LUT_COMMENT
STATIC INLINE int16_t fm_SinU16DirectLutInlined(uint16_t theta);

/* \fn int16_t fm_SinU16DirectLutNonInlined(uint16_t theta)
 * Non-inlined implementation (optimal memory usage, worse execution speed). */FM_SIN_U16_DIRECT_LUT_COMMENT
int16_t fm_SinU16DirectLutNonInlined(uint16_t theta);


/** Common comment block */
#define FM_SIN_I16_DIRECT_LUT_COMMENT \
/** Signed 16-bit sine function.
   This function calculates (a high-speed approximation of) \f$sin(x)\f$ where \a x is an angle
   value represented as a 16 bit signed fraction of \f$2\pi\f$.
   The result is represented as a 16 bit signed fraction (Q15 format).
   Function takes output value directly from the LUT w/o performing interpolation.
   Absolute error of the result is \f$\pm\f$ 101.
   The LUT size which is used by the function is configured by ::FM_FULL_SIN_COS_LUT.
   @param[in]  theta  The input angle, as 16 bit signed fraction of \f$2\pi\f$.
                      32768(-1->-\f$\pi\f$) < in < 32768(1->\f$\pi\f$)
   @return     16-bit signed fraction (Q15 format) */
/* \fn int16_t fm_SinI16DirectLutInlined(int16_t theta)
 * Inlined implementation (optimal memory usage, worse execution speed). */FM_SIN_I16_DIRECT_LUT_COMMENT
STATIC INLINE int16_t fm_SinI16DirectLutInlined(int16_t theta);

/* \fn int16_t fm_SinI16DirectLutNonInlined(int16_t theta)
 * Non-inlined implementation (optimal memory usage, worse execution speed). */FM_SIN_I16_DIRECT_LUT_COMMENT
int16_t fm_SinI16DirectLutNonInlined(int16_t theta);


/** Common comment block */
#define FM_COS_U16_DIRECT_LUT_COMMENT \
/** Unsigned 16-bit cosine function.
   This function calculates (a high-speed approximation of) \f$cos(x)\f$ where \a x is an angle
   value represented as a 16 bit unsigned fraction of \f$2\pi\f$.
   The result is represented as a 16 bit signed fraction (Q15 format).
   Function takes output value directly from the LUT w/o performing interpolation.
   Absolute error of the result is \f$\pm\f$ 101.
   The LUT size which is used by the function is configured by ::FM_FULL_SIN_COS_LUT.
   @param[in]  theta  The input angle, as 16 bit unsigned fraction of \f$2\pi\f$.
                      0 < in < 65536=\f$2\pi\f$
   @return     16-bit signed fraction (Q15 format) */
/* \fn int16_t fm_CosU16DirectLutInlined(uint16_t theta)
 * Inlined implementation (optimal memory usage, worse execution speed). */FM_COS_U16_DIRECT_LUT_COMMENT
STATIC INLINE int16_t fm_CosU16DirectLutInlined(uint16_t theta);

/* \fn int16_t fm_CosU16DirectLutNonInlined(uint16_t theta)
 * Non-inlined implementation (optimal memory usage, worse execution speed). */FM_COS_U16_DIRECT_LUT_COMMENT
int16_t fm_CosU16DirectLutNonInlined(uint16_t theta);


/** Common comment block */
#define FM_COS_I16_DIRECT_LUT_COMMENT \
/** Signed 16-bit cosine function.
   This function calculates (a high-speed approximation of) \f$cos(x)\f$ where \a x is an angle
   value represented as a 16 bit signed fraction of \f$2\pi\f$.
   The result is represented as a 16 bit signed fraction (Q15 format).
   Function takes output value directly from the LUT w/o performing interpolation.
   Absolute error of the result is \f$\pm\f$ 101.
   The LUT size which is used by the function is configured by ::FM_FULL_SIN_COS_LUT.
   @param[in]  theta  The input angle, as 16 bit signed fraction of \f$2\pi\f$.
                      32768(-1->-\f$\pi\f$) < in < 32768(1->\f$\pi\f$)
   @return     16-bit signed fraction (Q15 format) */
/* \fn int16_t fm_CosI16DirectLutInlined(int16_t theta)
 * Inlined implementation (optimal memory usage, worse execution speed). */FM_COS_I16_DIRECT_LUT_COMMENT
STATIC INLINE int16_t fm_CosI16DirectLutInlined(int16_t theta);

/* \fn int16_t fm_CosI16DirectLutNonInlined(int16_t theta)
 * Non-inlined implementation (optimal memory usage, worse execution speed). */FM_COS_I16_DIRECT_LUT_COMMENT
int16_t fm_CosI16DirectLutNonInlined(int16_t theta);
/**@}*/


/** @name Sine and Cosine functions based on interpolation of LUT */


/** Common comment block */
#define FM_SIN_U16_INTERPOLATION_COMMENT \
/** Unsigned 16-bit sine function.
   This function calculates (a high-speed approximation of) \f$sin(x)\f$ where \a x is an angle
   value represented as a 16 bit unsigned fraction of \f$2\pi\f$.
   The result is represented as a 16 bit signed fraction (Q15 format).
   Absolute error of the result is \f$\pm\f$ 1.
   The LUT size which is used by the function is configured by ::FM_FULL_SIN_COS_LUT.
   @param[in]  theta  The input angle, as 16 bit unsigned fraction of \f$2\pi\f$.
                      0 < in < 65536=\f$2\pi\f$
   @return     16-bit signed fraction (Q15 format) */
/* \fn int16_t fm_SinU16InterpolationInlined(uint16_t theta)
 * Inlined implementation (optimal memory usage, worse execution speed). */FM_SIN_U16_INTERPOLATION_COMMENT
STATIC INLINE int16_t fm_SinU16InterpolationInlined(uint16_t theta);

/* \fn int16_t fm_SinU16InterpolationNonInlined(uint16_t theta)
 * Non-inlined implementation (optimal memory usage, worse execution speed). */FM_SIN_U16_INTERPOLATION_COMMENT
int16_t fm_SinU16InterpolationNonInlined(uint16_t theta);


/** Common comment block */
#define FM_SIN_I16_INTERPOLATION_COMMENT \
/** Signed 16-bit sine function.
   This function calculates (a high-speed approximation of) \f$sin(x)\f$ where \a x is an angle
   value represented as a 16 bit signed fraction of \f$2\pi\f$.
   The result is represented as a 16 bit signed fraction (Q15 format).
   Absolute error of the result is \f$\pm\f$ 1.
   The LUT size which is used by the function is configured by ::FM_FULL_SIN_COS_LUT.
   @param[in]  theta  The input angle, as 16 bit signed fraction of \f$2\pi\f$.
                      32768(-1->-\f$\pi\f$) < in < 32768(1->\f$\pi\f$)
   @return     16-bit signed fraction (Q15 format) */
/* \fn int16_t fm_SinI16InterpolationInlined(int16_t theta)
 * Inlined implementation (optimal memory usage, worse execution speed). */FM_SIN_I16_INTERPOLATION_COMMENT
STATIC INLINE int16_t fm_SinI16InterpolationInlined(int16_t theta);

/* \fn int16_t fm_SinI16InterpolationNonInlined(int16_t theta)
 * Non-inlined implementation (optimal memory usage, worse execution speed). */FM_SIN_I16_INTERPOLATION_COMMENT
int16_t fm_SinI16InterpolationNonInlined(int16_t theta);


/** Common comment block */
#define FM_COS_U16_INTERPOLATION_COMMENT \
/** Unsigned 16-bit cosine function.
   This function calculates (a high-speed approximation of) \f$cos(x)\f$ where \a x is an angle
   value represented as a 16 bit unsigned fraction of \f$2\pi\f$.
   The result is represented as a 16 bit signed fraction (Q15 format).
   Absolute error of the result is \f$\pm\f$ 1.
   The LUT size which is used by the function is configured by ::FM_FULL_SIN_COS_LUT.
   @param[in]  theta  The input angle, as 16 bit unsigned fraction of \f$2\pi\f$.
                      0 < in < 65536=\f$2\pi\f$
   @return     16-bit signed fraction (Q15 format) */
/* \fn int16_t fm_CosU16InterpolationInlined(uint16_t theta)
 * Inlined implementation (optimal memory usage, worse execution speed). */FM_COS_U16_INTERPOLATION_COMMENT
STATIC INLINE int16_t fm_CosU16InterpolationInlined(uint16_t theta);

/* \fn int16_t fm_CosU16InterpolationNonInlined(uint16_t theta)
 * Non-inlined implementation (optimal memory usage, worse execution speed). */FM_COS_U16_INTERPOLATION_COMMENT
int16_t fm_CosU16InterpolationNonInlined(uint16_t theta);


/** Common comment block */
#define FM_COS_I16_INTERPOLATION_COMMENT \
/** Signed 16-bit cosine function.
   This function calculates (a high-speed approximation of) \f$cos(x)\f$ where \a x is an angle
   value represented as a 16 bit signed fraction of \f$2\pi\f$.
   The result is represented as a 16 bit signed fraction (Q15 format).
   Absolute error of the result is \f$\pm\f$ 1.
   The LUT size which is used by the function is configured by ::FM_FULL_SIN_COS_LUT.
   @param[in]  theta  The input angle, as 16 bit signed fraction of \f$2\pi\f$.
                      32768(-1->-\f$\pi\f$) < in < 32768(1->\f$\pi\f$)
   @return     16-bit signed fraction (Q15 format) */
/* \fn int16_t fm_CosI16InterpolationInlined(int16_t theta)
 * Inlined implementation (optimal memory usage, worse execution speed). */FM_COS_I16_INTERPOLATION_COMMENT
STATIC INLINE int16_t fm_CosI16InterpolationInlined(int16_t theta);

/* \fn int16_t fm_CosI16InterpolationNonInlined(int16_t theta);
 * Non-inlined implementation (optimal memory usage, worse execution speed). */FM_COS_I16_INTERPOLATION_COMMENT
int16_t fm_CosI16InterpolationNonInlined(int16_t theta);
/**@}*/


/** @name Arctangent functions based on direct LUT */


/** Common comment block */
#define FM_ATAN2_U16_DIRECT_LUT_COMMENT \
/** Unsigned atan2 function based on direct LUT value (no interpolation).
   This function calculates (a high-speed approximation of) \f$arctan(y/x)\f$ where \a y and \a x
   are two 16 bit integer numbers.
   The result is an angle value between 0 and \f$\pi\f$/2, represented as a 16 bit
   signed fraction of \f$2\pi\f$ i.e. Q15 format.
   Function takes output value directly from the LUT w/o performing interpolation.
   Absolute error of the result is \f$\pm\f$ 21.
   @note       atan2U16(0, 0) is defined as 0.
   @param[in]  y  Y coordinate
   @param[in]  x  X coordinate
   @return     The output angle, as 16 bit signed fraction of \f$2\pi\f$.
               32768(-1->-\f$\pi\f$) < out < 32768(1->\f$\pi\f$) */
/* \fn int16_t fm_Atan2U16DirectLutInlined(uint16_t y, uint16_t x)
 * Inlined implementation (optimal memory usage, worse execution speed). */FM_ATAN2_U16_DIRECT_LUT_COMMENT
STATIC INLINE int16_t fm_Atan2U16DirectLutInlined(uint16_t y, uint16_t x);

/* \fn int16_t fm_Atan2U16DirectLutNonInlined(uint16_t y, uint16_t x)
 * Non-inlined implementation (optimal memory usage, worse execution speed). */FM_ATAN2_U16_DIRECT_LUT_COMMENT
int16_t fm_Atan2U16DirectLutNonInlined(uint16_t y, uint16_t x);


/** Common comment block */
#define FM_ATAN2_I16_DIRECT_LUT_COMMENT \
/** Signed atan2 function based on direct LUT value (no interpolation).
   This function calculates (a high-speed approximation of) \f$arctan(y/x)\f$ where \a y and \a x
   are two 16 bit integer numbers.
   The result is an angle value between 0 and \f$2\pi\f$, represented as a 16 bit
   signed fraction of \f$2\pi\f$ i.e. Q15 format.
   Function takes output value directly from the LUT w/o performing interpolation.
   Absolute error of the result is \f$\pm\f$ 21.
   @note       atan2I16(0, 0) is defined as 0.
   @param[in]  y  Y coordinate
   @param[in]  x  X coordinate
   @return     The output angle, as 16 bit signed fraction of \f$2\pi\f$.
               32768(-1->-\f$\pi\f$) < out < 32768(1->\f$\pi\f$) */
/* \fn int16_t fm_Atan2I16DirectLutInlined(int16_t y, int16_t x)
 * Inlined implementation (optimal memory usage, worse execution speed). */FM_ATAN2_I16_DIRECT_LUT_COMMENT
STATIC INLINE int16_t fm_Atan2I16DirectLutInlined(int16_t y, int16_t x);

/* \fn int16_t fm_Atan2I16DirectLutNonInlined(int16_t y, int16_t x)
 * Non-inlined implementation (optimal memory usage, worse execution speed). */FM_ATAN2_I16_DIRECT_LUT_COMMENT
int16_t fm_Atan2I16DirectLutNonInlined(int16_t y, int16_t x);
/**@}*/


/** @name Arctangent functions based on interpolation of LUT */


/** Common comment block */
#define FM_ATAN2_U16_INTERPOLATION_COMMENT \
/** Unsigned atan2 function based on interpolation of LUT.
   This function calculates (a high-speed approximation of) \f$arctan(y/x)\f$ where \a y and \a x
   are two 16 bit integer numbers.
   The result is an angle value between 0 and \f$\pi\f$/2, represented as a 16 bit
   signed fraction of \f$2\pi\f$ i.e. Q15 format.
   Absolute error of the result is \f$\pm\f$ 1.
   @note       atan2U16(0, 0) is defined as 0.
   @param[in]  y  Y coordinate
   @param[in]  x  X coordinate
   @return     The output angle, as 16 bit signed fraction of \f$2\pi\f$.
               32768(-1->-\f$\pi\f$) < out < 32768(1->\f$\pi\f$) */
/* \fn int16_t fm_Atan2U16InterpolationInlined(uint16_t y, uint16_t x)
 * Inlined implementation (optimal memory usage, worse execution speed). */FM_ATAN2_U16_INTERPOLATION_COMMENT
STATIC INLINE int16_t fm_Atan2U16InterpolationInlined(uint16_t y, uint16_t x);

/* \fn int16_t fm_Atan2U16InterpolationNonInlined(uint16_t y, uint16_t x)
 * Non-inlined implementation (optimal memory usage, worse execution speed). */FM_ATAN2_U16_INTERPOLATION_COMMENT
int16_t fm_Atan2U16InterpolationNonInlined(uint16_t y, uint16_t x);


/** Common comment block */
#define FM_ATAN2_I16_INTERPOLATION_COMMENT \
/** Signed atan2 function based on interpolation of LUT.
   This function calculates (a high-speed approximation of) \f$arctan(y/x)\f$ where \a y and \a x
   are two 16 bit integer numbers.
   The result is an angle value between 0 and \f$2\pi\f$, represented as a 16 bit
   signed fraction of \f$2\pi\f$ i.e. Q15 format.
   Absolute error of the result is \f$\pm\f$ 1.
   @note       atan2I16(0, 0) is defined as 0.
   @param[in]  y  Y coordinate
   @param[in]  x  X coordinate
   @return     The output angle, as 16 bit signed fraction of \f$2\pi\f$.
               32768(-1->-\f$\pi\f$) < out < 32768(1->\f$\pi\f$) */
/* \fn int16_t fm_Atan2I16InterpolationInlined(int16_t y, int16_t x)
 * Inlined implementation (optimal memory usage, worse execution speed). */FM_ATAN2_I16_INTERPOLATION_COMMENT
STATIC INLINE int16_t fm_Atan2I16InterpolationInlined(int16_t y, int16_t x);

/* \fn int16_t fm_Atan2I16InterpolationNonInlined(int16_t y, int16_t x)
 * Non-inlined implementation (optimal memory usage, worse execution speed). */FM_ATAN2_I16_INTERPOLATION_COMMENT
int16_t fm_Atan2I16InterpolationNonInlined(int16_t y, int16_t x);
/**@}*/


/** @name Square root of the sum of two squares */


/** Common comment block */
#define FM_HYPOTENUSE_PYTHAGOREAN_COMMENT \
/** Hypotenuse function by calculating the square-root of two square-integer values.
   This function calculates \f$\sqrt{a^2+b^2}\f$ where \a a and \a b are two 16 bit integer numbers.
   The result is hypotenuse value, represented as a 16 bit number.
   @param[in]  a  value a
   @param[in]  b  value b
   @return     high-speed approximation of \f$\sqrt{a^2+b^2}\f$ */
/* \fn uint16_t fm_HypotenusePythagoreanInlined(int16_t a, int16_t b)
 * Inlined implementation (optimal memory usage, worse execution speed). */FM_HYPOTENUSE_PYTHAGOREAN_COMMENT
STATIC INLINE uint16_t fm_HypotenusePythagoreanInlined(int16_t a, int16_t b);

/* \fn uint16_t fm_HypotenusePythagoreanNonInlined(int16_t a, int16_t b)
 * Non-inlined implementation (optimal memory usage, worse execution speed). */FM_HYPOTENUSE_PYTHAGOREAN_COMMENT
uint16_t fm_HypotenusePythagoreanNonInlined(int16_t a, int16_t b);


#define FM_HYPOTENUSE_AMAX_BMIN_COMMENT \
/** Hypotenuse function by alpha max plus beta min algorithm.
   The alpha max plus beta min algorithm is a high-speed approximation of the square root of the sum of two squares.
   This function calculates \f$\sqrt{a^2+b^2}\f$ where \a a and \a b are two 16 bit integer numbers.
   The result is hypotenuse value, represented as a 32 bit number.
   Absolute error of the result is \f$\pm\f$ 1036. Maximum relative uncertainty is \f$\pm\f$ 1.22%. Mean is 0.67%.
   Alpha max plus beta min algorithm constants:
    - alpha_0 = 1
    - beta_0  = 5/32
    - alpha_1 = 27/32
    - beta_1  = 71/128
   @param[in]  a  value a
   @param[in]  b  value b
   @return     high-speed approximation of \f$\sqrt{a^2+b^2}\f$ */
/* \fn uint32_t fm_HypotenuseAmaxBminInlined(uint16_t a, uint16_t b)
 * Inlined implementation (optimal memory usage, worse execution speed). */FM_HYPOTENUSE_AMAX_BMIN_COMMENT
STATIC INLINE uint32_t fm_HypotenuseAmaxBminInlined(uint16_t a, uint16_t b);

/* \fn uint32_t fm_HypotenuseAmaxBminNonInlined(uint16_t a, uint16_t b)
 * Non-inlined implementation (optimal memory usage, worse execution speed). */FM_HYPOTENUSE_AMAX_BMIN_COMMENT
uint32_t fm_HypotenuseAmaxBminNonInlined(uint16_t a, uint16_t b);


#define FM_HYPOTENUSE_TRIGONOMETRY_FULL_ACCURACY_COMMENT \
/** Hypotenuse function based on trigonometry functions with full accuracy.
   This function calculates \f$\frac{a}{sin(arctan(\frac{a}{b}))}\f$ where \a a and \a b are two 16 bit integer numbers.
   The result is hypotenuse value, represented as a 32 bit number.
   Absolute error of the result is \f$\pm\f$ 19. Maximum relative uncertainty is \f$\pm\f$ 0.02%. Mean is 0.004%.
   @param[in]  a  value a
   @param[in]  b  value b
   @return     high-speed approximation of \f$\sqrt{a^2+b^2}\f$ */
/* \fn uint32_t fm_HypotenuseTrigonometryFullAccuracyInlined(uint16_t a, uint16_t b)
 * Inlined implementation (optimal memory usage, worse execution speed). */FM_HYPOTENUSE_TRIGONOMETRY_FULL_ACCURACY_COMMENT
STATIC INLINE uint32_t fm_HypotenuseTrigonometryFullAccuracyInlined(uint16_t a, uint16_t b);

/* \fn uint32_t fm_HypotenuseTrigonometryFullAccuracyNonInlined(uint16_t a, uint16_t b)
 * Non-inlined implementation (optimal memory usage, worse execution speed). */FM_HYPOTENUSE_TRIGONOMETRY_FULL_ACCURACY_COMMENT
uint32_t fm_HypotenuseTrigonometryFullAccuracyNonInlined(uint16_t a, uint16_t b);


#define FM_HYPOTENUSE_TRIGONOMETRY_REDUCED_ACCURACY_COMMENT \
/** Hypotenuse function based on trigonometry functions with reduced accuracy.
   This function calculates \f$\frac{a}{sin(arctan(\frac{a}{b}))}\f$ where \a a and \a b are two 16 bit integer numbers.
   The result is hypotenuse value, represented as a 32 bit number.
   Absolute error of the result is \f$\pm\f$ 592. Maximum relative uncertainty is \f$\pm\f$ 0.64%. Mean is 0.18%.
   @param[in]  a  value a
   @param[in]  b  value b
   @return     high-speed approximation of \f$\sqrt{a^2+b^2}\f$ */
/* \fn uint32_t fm_HypotenuseTrigonometryReducedAccuracyInlined(uint16_t a, uint16_t b)
 * Inlined implementation (optimal memory usage, worse execution speed). */FM_HYPOTENUSE_TRIGONOMETRY_REDUCED_ACCURACY_COMMENT
STATIC INLINE uint32_t fm_HypotenuseTrigonometryReducedAccuracyInlined(uint16_t a, uint16_t b);

/* \fn uint32_t fm_HypotenuseTrigonometryReducedAccuracyNonInlined(uint16_t a, uint16_t b)
 * Non-inlined implementation (optimal memory usage, worse execution speed). */FM_HYPOTENUSE_TRIGONOMETRY_REDUCED_ACCURACY_COMMENT
uint32_t fm_HypotenuseTrigonometryReducedAccuracyNonInlined(uint16_t a, uint16_t b);
/**@}*/

#ifndef UNITTEST
#include "fm_inline_impl.h"
#endif /* UNITTEST */

#endif /* FM_H_ */

