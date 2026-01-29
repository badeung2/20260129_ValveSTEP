/**
 * @file
 * @brief Public API for the motor table module
 * @internal
 *
 * @copyright (C) 2020 Melexis N.V.
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
 * @ingroup lib_motor_table
 * @addtogroup lib_motor_table Motor Table
 *
 * @brief Public API for the motor table module
 * @details
 * @{
 */
#ifndef MOTOR_TABLE_H
#define MOTOR_TABLE_H

#include <stdint.h>
#include "compiler_abstraction.h"

/** Number of steps for one electrical rotation : a complete sine */
#if defined(HAS_192PTS_MOTOR_SVM_TABLE)
#define MOTOR_SPACE_VECTOR_STEPS_PER_ROTATION   (192u)
#elif defined(HAS_384PTS_MOTOR_SVM_TABLE)
#define MOTOR_SPACE_VECTOR_STEPS_PER_ROTATION   (384u)
#endif

/** Number of electrical rotations stored in the table */
#define MOTOR_SPACE_VECTOR_ROTATIONS            (2u)

/** Number of elements in the MotorSpaceVectorModulation
 *
 * Since we store several complete sines, we have MOTOR_SPACE_VECTOR_ROTATIONS x MOTOR_SPACE_VECTOR_STEPS_PER_ROTATION
 * elements in the table. Having more sines reduces the number of operations to be performed to calculate the right
 * index in application.
 */
#define MOTOR_SPACE_VECTOR_MODULATION_SIZE      (MOTOR_SPACE_VECTOR_STEPS_PER_ROTATION * MOTOR_SPACE_VECTOR_ROTATIONS)

typedef struct {
    uint16_t a;
    uint16_t b;
    uint16_t c;
    uint16_t d;
} vector_t;


#ifdef HAS_MOTOR_SVM_WRAPPER_FUNC
/** Getter for the Space Vector Modulation table
 *
 * @note No modulo-operation is applied on the given index. The given index needs to be
 * below #MOTOR_SPACE_VECTOR_MODULATION_SIZE
 * @param idx Index in the space vector modulation table
 * @return Value for space vector modulation at given index.
 *
 * @deprecated This function still exists in some ROMs but it is replaced by Motor_getDirectSpaceVectorModulation
 * that is more flexible and efficient
 */
uint16_t Motor_getSpaceVectorModulation(uint16_t idx);
#endif

/** @brief Getter for the Space Vector Modulation table
 * @note it is possible to use this function to get 1 to 4 samples from the table
 * Because this function is inlined no time/space is lost by not using all returned values
 *
 * Some usage examples:
 *
 *   - Get 4 samples:
 * @code
 *     vector_t v = Motor_getSpaceVectorModulation(index, 0, 48, 96, 144);
 *
 *     test1 = v.a;
 *     test2 = v.b;
 *     test3 = v.c;
 *     test3 = v.d;
 * @endcode
 *
 *   - Get 3 samples:
 * @code
 *     vector_t v = Motor_getSpaceVectorModulation(index, 0, 48, 96, 144);
 *
 *     test1 = v.a;
 *     test2 = v.b;
 *     test3 = v.c;
 * @endcode
 *
 *   - Get only 1 sample:
 * @code
 *     vector_t v = Motor_getSpaceVectorModulation(index, 0, 0, 0, 0);
 *     test1 = v.a;
 * @endcode
 *
 * @note No modulo-operation is applied on the given index. The given index needs to be
 * below #MOTOR_SPACE_VECTOR_MODULATION_SIZE
 *
 * @param[in] index Index in the space vector modulation table
 * @param[in] o1, o2, o3, o4 index offsets of the desired samples
 *
 * @return The space vector modulations samples at given index and offsets.
 */
STATIC INLINE vector_t Motor_getDirectSpaceVectorModulation(uint16_t index,
                                                            uint16_t o1,
                                                            uint16_t o2,
                                                            uint16_t o3,
                                                            uint16_t o4);

#ifndef UNITTEST
#include "lib_motor_table_inline_impl.h"
#endif /* UNITTEST */

#endif
/// @}

