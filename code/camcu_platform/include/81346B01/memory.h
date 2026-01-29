/**
 * @file
 * @brief Memory API
 * @internal
 *
 * @copyright (C) 2018-2020 Melexis N.V.
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
 * @ingroup memory
 * @addtogroup memory Memory
 *
 * @brief Memory API
 * @details
 *
 * @{
 */

#ifndef MEMORY_H
#define MEMORY_H

#include <stddef.h>
#include <stdint.h>
#include "compiler_abstraction.h"

#ifdef HAS_MEM_GT_64KB
#define MEMORY_MAX_ADDRESS          (0xFFFFFu)
#else
#define MEMORY_MAX_ADDRESS          (0xFFFFu)
#endif

/** Copies word-wise data from any 16-bit memory location to a 16-bit memory location
 *
 * The function allows to copy data from 1 location to another word wise which is needed
 * for memories which do not allow byte write operations as EEPROM.
 *
 * @param[in]  destination  pointer to the location to start copying too (needs to be word aligned!).
 * @param[in]  source  pointer to the address to start copying from (needs to be word aligned!).
 * @param[in]  size  number of bytes to be copied (needs to be word aligned!).
 * @warning No additional checks are performed.
 */
STATIC INLINE void memcpy_word(void* destination, const void* source, size_t size);

/** Compares word-wise data from any 16-bit memory location with a 16-bit memory location
 *
 * The function allows to compare data from 1 location with another word wise which is needed
 * for memories which do not allow byte write operations as EEPROM.
 *
 * @param[in]  ptr1  pointer to first memory (needs to be word aligned!).
 * @param[in]  ptr2  pointer to second memory (needs to be word aligned!).
 * @param[in]  size  number of bytes to be compared (needs to be word aligned!).
 * @warning No additional checks are performed.
 */
STATIC INLINE int16_t memcmp_word(const void* ptr1, const void* ptr2, size_t size);

/** Copies data from any 20-bit memory location to a 16-bit memory location
 *
 * The function allows to copy data placed in addresses above 64kb address space.
 *
 * @param[in]  destination  16-bit pointer to the location to start copying too.
 * @param[in]  source  20-bit pointer to the address to start copying from.
 * @param[in]  num  number of bytes to be copied.
 * @warning No additional checks are performed, `source` + `size` shall not overflow a page.
 */
void memcpy_fromExtended(void* destination, uint32_t source, size_t num);

/** Copies data from any 16-bit memory location to a 20-bit memory location
 *
 * The function allows to copy data to addresses above 64kb address space.
 *
 * @param[in]  destination  20-bit pointer to the location to start copying too.
 * @param[in]  source  16-bit pointer to the address to start copying from.
 * @param[in]  num  number of bytes to be copied.
 * @warning No additional checks are performed, `source` + `size` shall not overflow a page.
 */
void memcpy_toExtended(uint32_t destination, const void* source, size_t num);

/** Read byte from any memory address
 *
 * Compiler doesn't support MLX16 extended memory access yet (20-bit addresses).
 * This function allows data read from a location >64kB
 *
 * @note The function will not attempt to access the memory, if the given address
 * is not within range of the chip.
 *
 * @param [in] dstFarPointer 20-bit address to be read, needs to be byte aligned
 * @return 8-bit data value located at *dstFarPointer
 */
STATIC INLINE uint8_t memoryReadByte(uint32_t dstFarPointer);

/** Read word from any memory address
 *
 * Compiler doesn't support MLX16 extended memory access yet (20-bit addresses).
 * This function allows data read from a location >64kB
 *
 * @note The function will not attempt to access the memory, if the given address
 * is not within range of the chip.
 *
 * @param [in] dstFarPointer 20-bit address to be read, needs to be word aligned
 * @return 16-bit data value located at *dstFarPointer
 */
STATIC INLINE uint16_t memoryReadWord(uint32_t dstFarPointer);

/** Read long from any memory address
 *
 * Compiler doesn't support MLX16 extended memory access yet (20-bit addresses).
 * This function allows data read from a location >64kB
 *
 * @note The function will not attempt to access the memory, if the given address
 * is not within range of the chip.
 *
 * @param [in] dstFarPointer 20-bit address to be read, needs to be word aligned
 * @return 32-bit data value located at *dstFarPointer
 */
STATIC INLINE uint32_t memoryReadLong(uint32_t dstFarPointer);

/** Write byte to any memory address
 *
 * Compiler doesn't support MLX16 extended memory access yet (20-bit addresses).
 * This function allows data write to a location >64kB
 *
 * @note The function will not attempt to access the memory, if the given address
 * is not within range of the chip.
 *
 * @note The function does not take care of memory pre-configuration needed for writing,
 * as may be needed for e.g. flash or eeprom. It just initiates a write to the given
 * address.
 *
 * @param [in] dstFarPointer 20-bit address to be written, needs to be byte aligned
 * @param [in] value 8-bit data value to be written at *dstFarPointer
 */
STATIC INLINE void memoryWriteByte(uint32_t dstFarPointer, uint8_t value);

/** Write word to any memory address
 *
 * Compiler doesn't support MLX16 extended memory access yet (20-bit addresses).
 * This function allows data write to a location >64kB
 *
 * @note The function will not attempt to access the memory, if the given address
 * is not within range of the chip.
 *
 * @note The function does not take care of memory pre-configuration needed for writing,
 * as may be needed for e.g. flash or eeprom. It just initiates a write to the given
 * address.
 *
 * @param [in] dstFarPointer 20-bit address to be written, needs to be word aligned
 * @param [in] value 16-bit data value to be written at *dstFarPointer
 */
STATIC INLINE void memoryWriteWord(uint32_t dstFarPointer, uint16_t value);

/** Write long to any memory address
 *
 * Compiler doesn't support MLX16 extended memory access yet (20-bit addresses).
 * This function allows data write to a location >64kB
 *
 * @note The function will not attempt to access the memory, if the given address
 * is not within range of the chip.
 *
 * @note The function does not take care of memory pre-configuration needed for writing,
 * as may be needed for e.g. flash or eeprom. It just initiates a write to the given
 * address.
 *
 * @param [in] dstFarPointer 20-bit address to be written, needs to be word aligned
 * @param [in] value 32-bit data value to be written at *dstFarPointer
 */
STATIC INLINE void memoryWriteLong(uint32_t dstFarPointer, uint32_t value);

#if !defined(UNITTEST)
#include "memory_inline_impl.h"
#endif

/// @}
#endif

