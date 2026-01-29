/**
 * @file
 * @brief Public API for the FIFO module
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
 * @ingroup FIFO
 *
 * @details
 */

#ifndef FIFO_H
#define FIFO_H

#include <stdint.h>
#include <stdbool.h>
#include "compiler_abstraction.h"

/** The maximum number of elements of a FIFO */
#define FIFO_MAX_NB_OF_ELEMENTS ((uint16_t)(0xFFFDu))

/** The minimum number of elements of a FIFO */
#define FIFO_MIN_NB_OF_ELEMENTS ((uint16_t)(2u))

/**  The minimum size of each element in the FIFO_CREATE_OBJECT */
#define FIFO_MIN_ELEMENT_SIZE ((uint16_t)(1u))

/**  The default index for RIND and WIND to be set by the resource */
#define FIFO_DEFAULT_INDEX (UINT16_MAX)

/** Structure to hold the RIND and WIND of a FIFO */
typedef struct FIFO_Indices_s {
    /** The read index: location of the first valid element */
    uint16_t Read;
    /** The write index: location of the first element to be written */
    uint16_t Write;
} FIFO_Indices_t;

/** Object definition to hold the information needed for a FIFO */
typedef struct FIFO_Object_s {
    /** Constant pointer to the memory block to be used */
    void* const Memory;
    /** Constant pointer to the RIND and WIND structure */
    FIFO_Indices_t* const RWIndices;
    /** Constant number of elements that the FIFO can hold */
    const uint16_t NbOfElements;
    /** Constant size of one element to be stored in the FIFO */
    const uint16_t ElementSize;
} FIFO_Object_t;

/** Convert a given FIFO name to a C-identifier representing the FIFO object
 *  \param Name The given FIFO name
 */
#define FIFO_OBJECT(Name) FIFO_ ## Name ## _

/** Create a FIFO object with given parameters. RAM memory that the FIFO
 *  uses to store the enqueued samples is also reserved.
 *  \param Name The given FIFO name
 *  \param NbOfElements The number of elements that must be able to be stored in the FIFO
 *  \param SizeOfElement The size of one element to be stored in the FIFO
 */
#define FIFO_CREATE_OBJECT(Name, NbOfElements, SizeOfElement) \
    STATIC uint8_t FIFO_ ## Name ## _memory[(SizeOfElement) * ((NbOfElements) + 1)]; \
    STATIC FIFO_Indices_t FIFO_ ## Name ## _indices = {FIFO_DEFAULT_INDEX, FIFO_DEFAULT_INDEX}; \
    STATIC CONST FIFO_Object_t FIFO_OBJECT(Name) = { \
        FIFO_ ## Name ## _memory, \
        &FIFO_ ## Name ## _indices, \
        NbOfElements, \
        SizeOfElement \
    }

bool FIFO_Open(const FIFO_Object_t* const hFifo);
bool FIFO_IsOpen(const FIFO_Object_t* const hFifo);
bool FIFO_IsEmpty(const FIFO_Object_t* const hFifo);
bool FIFO_IsFull(const FIFO_Object_t* const hFifo);
bool FIFO_Enqueue(const FIFO_Object_t* const hFifo, const void* const element);
bool FIFO_Dequeue(const FIFO_Object_t* const hFifo, void* const element);
bool FIFO_Close(const FIFO_Object_t* const hFifo);
uint16_t FIFO_GetElementSize(const FIFO_Object_t* const hFifo);

#endif

