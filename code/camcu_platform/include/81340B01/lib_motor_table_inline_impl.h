/**
 * @file
 * @brief Motor table support library functions
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
 *
 * @brief
 * @details
 * @{
 */

#ifndef LIB_MOTOR_TABLE_INLINE_IMPL_H
#define LIB_MOTOR_TABLE_INLINE_IMPL_H

STATIC INLINE vector_t Motor_getDirectSpaceVectorModulation(uint16_t index,
                                                            uint16_t o1,
                                                            uint16_t o2,
                                                            uint16_t o3,
                                                            uint16_t o4)
{
    /* Declare SVM (Space Vector Modulation) table */
    extern CONST uint16_t MotorSpaceVectorModulation[];

    /* Set a pointer to the constant SVM table */
    uint16_t CONST* p = &MotorSpaceVectorModulation[index];

    /* Fill the vector with values from the SVM table */
    vector_t vector = {
        .a = p[o1],
        .b = p[o2],
        .c = p[o3],
        .d = p[o4]
    };

    return vector;
}

#endif /* LIB_MOTOR_TABLE_INLINE_IMPL_H */
/// @}
