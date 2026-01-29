/**
 * @file
 * @brief Public API for the Double Buffer module
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
 * @ingroup DoubleBuffer
 *
 * @details
 */

#ifndef DOUBLE_BUFFER_H
#define DOUBLE_BUFFER_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <compiler_abstraction.h>

/** Object definition to hold the information needed for a DoubleBuffer */
typedef struct DoubleBuffer_Object_s {
    /** Constant pointer to the hot buffer block to be used */
    volatile void* const Hot;
    /** Constant pointer to the cold buffer block to be used */
    void* const Cold;
    /** Constant size of one element to be stored in the DoubleBuffer */
    const uint16_t BufferSize;
} DoubleBuffer_Object_t;

/** Set the value of the cold buffer of a DoubleBuffer
 *
 * @param hDoubleBuffer The handle (pointer to DoubleBuffer structure) to the DoubleBuffer object
 * @param data pointer to the value that the cold buffer is going to be changed to.
 * @return True if the DoubleBuffer could set the cold buffer successfully, false otherwise
 */
bool DoubleBuffer_SetCold(const DoubleBuffer_Object_t* const hDoubleBuffer, const void* const data);

/** Get the value of the cold buffer of a DoubleBuffer
 *
 * @param hDoubleBuffer The handle (pointer to DoubleBuffer structure) to the DoubleBuffer object
 * @param data pointer to the value that the cold value is going to be changed to.
 * @return True if the DoubleBuffer could get the cold buffer successfully, false otherwise
 */
bool DoubleBuffer_GetCold(const DoubleBuffer_Object_t* const hDoubleBuffer, void* const data);

/** Swap the value of the hot buffer and the cold buffer of a DoubleBuffer
 *
 * @param hDoubleBuffer The handle (pointer to DoubleBuffer structure) to the DoubleBuffer object
 * @return True if the DoubleBuffer could swap the hot buffer and the cold buffer successfully, false otherwise
 */
bool DoubleBuffer_SwapHotCold(const DoubleBuffer_Object_t* const hDoubleBuffer);

/** Get the size (in bytes) of an buffer that fits in the DoubleBuffer
 *
 * @param hDoubleBuffer The handle (pointer to DoubleBuffer structure) to the DoubleBuffer object
 * @return The size (in bytes) of one buffer in the DoubleBuffer. A value of zero here
 * indicates a failure in the handle.
 */
uint16_t DoubleBuffer_GetBufferSize(const DoubleBuffer_Object_t* const hDoubleBuffer);

#endif

