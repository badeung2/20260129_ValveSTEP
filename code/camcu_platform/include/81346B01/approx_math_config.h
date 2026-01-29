/**
 * @file
 * @brief Approximate math configurations
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
 * @ingroup approx_math
 */

#ifndef APPROX_MATH_CONFIG_H_
#define APPROX_MATH_CONFIG_H_

#include "compiler_abstraction.h"
#include "mathlib.h"


/* ---------------------------------------------
 * Public Defines
 * --------------------------------------------- */

/** @def AM_EXEC_MEMORY
 * Allows to modify the target memory which is used for approximate math functions execution.
 * LUT table placement is not affected by this definition.
 */
#define AM_EXEC_MEMORY

/** @def AM_FULL_SIN_COS_LUT
 * The selection affects LUT size for sin/cos.
 * Value '1' selects LUT generated for full period which takes 2.5KBytes.
 * Value '0' selects LUT for quarter period is used which takes 0.5KBytes.
 * The selection also affects type of functions which are going to be used
 * and their speed.
 */
#define AM_FULL_SIN_COS_LUT 1

#endif /* APPROX_MATH_CONFIG_H_ */
