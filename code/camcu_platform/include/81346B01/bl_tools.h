/**
 * @file
 * @brief Provides a set of functions used for the bootloaders and can be used in the Software Application in FLASH
 * @internal
 *
 * @copyright (C) 2015-2017 Melexis N.V.
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
 * @ingroup bl_tools
 * @addtogroup bl_tools
 * @ingroup library
 *
 * @brief Provides a set of functions used for the bootloaders and can be used in the Software Application in FLASH
 * @details
 * @{
 */

#ifndef BL_TOOLS_H
#define BL_TOOLS_H

#include <stdint.h>
#include <stdbool.h>
#include "bl_bist.h"
#include "compiler_abstraction.h"

void dataCopy64(const uint16_t* src, uint16_t* dest, uint8_t size);
void dataFill16( uint16_t* dest,  uint16_t pattern, uint8_t size);
NO_RETURN void MLX16_RESET(BistResetInfo_t reason);
NO_RETURN void MLX16_RESET_SIGNED(BistResetInfo_t reason);
void SetRstatFlag(void);
void ResetRstatFlag(void);

/** Cold MLX16 reset */
STATIC INLINE NO_RETURN void MLX16_RESET_COLD(void);

/** Warm MLX16 reset */
STATIC INLINE NO_RETURN void MLX16_RESET_WARM(void);

/** Reset the MLX16 with defining the reason. Reason is one of the values pre-defined in the bl_bist.h
 * @param[in] reason specifies the declared reason which is copied into the bistResetInfo RAM cell.
 */
STATIC INLINE NO_RETURN void MLX16_RESET_EPM(BistResetInfo_t reason);

/** Copies up to 16 words of data from src into dest area ascending
 * @param[in] *src source data to copy
 * @param[in] *dest destination for the copying
 * @param[in] size data size in 16bit words.
 * @warning The data size should be within the [1..15]. In case the size=0 or 16 - this call will copy 16*2 bytes
 */
STATIC INLINE void dataCopy16(const uint16_t* src, uint16_t* dest, uint8_t size);

/** Legacy declaration */
#define dataCopy16_(src, dest, size) dataCopy16(src, dest, size)

/** Copies the data in 16bit words from the data above 64k addresses to the buffer
 *
 * Allows to copy the data placed in the addresses above 64kb
 *
 * @param[in] *src_extended_address source data to copy, address is >64kb
 * @param[in] *dest destination for the copying
 * @param[in] size the size of data to be copied in 16bit words
 * @note No additional checks were performed, Data size shall be 16-bit aligned.
 * @warning Since MOVSW instruction is used, the memory extension will be valid only for the first read access.
 *          You can only copy data from an area above 64KB to another area below 64KB.
 */
void dataCopy16_extended(uint32_t src_extended_address,  uint16_t* dest, uint16_t size);

/** Copies the data in 32bit words from the data above 64k addresses to the buffer
 *
 * Allows to copy the data placed in the addresses above 64kb
 *
 * @param[in] *src_extended_address source data to copy, address is >64kb
 * @param[in] *dest destination for the copying
 * @param[in] size the size of data to be copied in 32bit words
 * @note No additional checks were performed, Data size shall be 32-bit aligned.
 */
__attribute__ ((deprecated("Removed and can be replaced by for ex dataCopy16_extended")))
void dataCopy32_extended(uint32_t src_extended_address,
                         uint16_t* dest,
                         uint16_t size);

/** Checks wether the argument is inside the region or not.
 * @param[in] arg the value to be compared. (32bit)
 * @param[in] min the smallest value to fit in the region.
 * @param[in] max the beggest value to fit in the region.
 * @retval true  the arg value is within the region.
 * @retval false the arg value is outside the region.
 */
STATIC INLINE bool within_including_u32(uint32_t arg, uint32_t min, uint32_t max);

/** Checks wether the argument is inside the region or not.
 * @param[in] arg the value to be compared. (16bit)
 * @param[in] min the smallest value to fit in the region.
 * @param[in] max the beggest value to fit in the region.
 * @retval true  the arg value is within the region.
 * @retval false the arg value is outside the region.
 */
STATIC INLINE bool within_including_u16(uint16_t arg, uint16_t min, uint16_t max);

/** Checks wether the argument is inside the region or not.
 * @param[in] arg the value to be compared. (8bit)
 * @param[in] min the smallest value to fit in the region.
 * @param[in] max the beggest value to fit in the region.
 * @retval true  the arg value is within the region.
 * @retval false the arg value is outside the region.
 */
STATIC INLINE bool within_including_u8(uint8_t arg, uint8_t min, uint8_t max);

/** The universal comparison macros, without any checks and constraints */
#define WITHIN_INCLUDING(arg, min, max)  (((arg) <= (max)) && ((arg) >= (min)))

#ifndef UNITTEST
#include "bl_tools_inline_impl.h"
#endif

#endif /* BL_TOOLS_H */
/// @}
