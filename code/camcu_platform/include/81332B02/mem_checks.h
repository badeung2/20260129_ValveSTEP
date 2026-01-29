/**
 * @file
 * @brief Provides a checksum calculation functions
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
 * @ingroup memory
 *
 * @brief Provides a checksum calculation functions
 * @details
 *
 * @{
 */


#ifndef MEM_CHECKS_H
#define MEM_CHECKS_H

#include <stdint.h>
#include "compiler_abstraction.h"
#include "static_assert.h"

#ifndef MEM_CHECKS_PREFIX
/** Defines memory region to be used for memory checks functions */
#define MEM_CHECKS_PREFIX
#endif

/** Initial value of the CRC16 CCITT check. 0xFFFF or 0x1D0F could be used. 0xFFFF is standard but 0x1D0F is used by SREC_CAT tool. */
#define CCITT_SEED            0x1D0F
/** Expected CRC result after calculation the checksum and included checksum-value within the region (checksum value should be adapted to have NVRAM_CORRECT_CRC result */
#define NVRAM_CORRECT_CRC     0x00FFu

/** RAM_BIST_REGULAR is the HW key which runs the 'regular' RAM BIST which spoils the RAM data when executed */
#define RAM_BIST_REGULAR        (0x0F59U)
/** RAM_BIST_TRANSPARENT is the HW key which runs the 'transparent' RAM BIST which performs the RAM-reading and calculating its signature */
#define RAM_BIST_TRANSPARENT    (0x0A6EU)

/** RAM_BIST_BUF_SIZE is the buffer for non-destructive RAM BIST data storage. It shall be <=64 words and multiply by 4 words (word = 16bit) */
#define RAM_BIST_BUF_SIZE   (0x10U)

/** RAM_BIST_AREA_FOR_STACK is the buffer for temporary stack.
 *
 * Stack is adjustable to make the ramBistConfiguration_t area to stay aligned to make the RAM area covered with non-destructive RAM test
 */
#define RAM_BIST_AREA_FOR_STACK    (14U)

/** Assigns a RAM buffer for non-destructive temporary storage to make the check possible */
typedef struct {
    uint16_t ramBistStartOffset; /**< Start of RAM_BIST area under check */
    uint16_t ramBistIterations; /**< The size of RAM_BIST area under check. The number if ramBistBufferSize iterations */
    uint16_t ram_stack[RAM_BIST_AREA_FOR_STACK]; /**< Stack-size provided for RAM BIST code. SW code uses 4 words (max) of stack. The size of area ensures the overall ramBistConfiguration_t should be multiplied by the RAM_BIST_BUF_SIZE */
    uint16_t temp_buffer[RAM_BIST_BUF_SIZE]; /**< a buffer which is used as a temporary storage, should be of size ramBistBufferSize+10, in 16bit words */
} ramBistConfiguration_t;

ASSERT((sizeof(ramBistConfiguration_t) % (RAM_BIST_BUF_SIZE * sizeof(uint16_t))) == 0);


/** Test if the CRC at the end of an area in the memory is correct
 *
 * @note This function gives a valid result only when called to check regions that are CRC protected.
 *       It will return false when called over regions that are not CRC protected. All regions in the
 *       memory that are CRC protected by Melexis are checked with this function during the startup
 *       procedure.
 *
 * Usage example:
 * @code
 * #define ADDR_EE_APP_TRIM        ((const uint16_t)0x09B0)
 * #define SIZE_EE_APP_TRIM        (0x50u)
 * bool areaCheck = nvram_CalcCRC((uint16_t *)ADDR_EE_APP_TRIM, (SIZE_EE_APP_TRIM >> 1);
 * @endcode
 *
 * @param[in] *pu16BeginAddress Start-address (16-bit aligned)
 * @param[in] u16Length Length (in 16-bit words)
 * @return a 8-bit (extended to 16-bit) checksum (Sum with carry) for the specified area.
 */
uint16_t nvram_CalcCRC(const uint16_t* pu16BeginAddress, const uint16_t u16Length);

/** Performs a CRC16-CCITT calculation for the data region selected
 *
 * @param[in]  startAddress  Address to the data under test.
 * @param[in]  size  length of the region to calculate the crc for (in number of bytes).
 * @param[in]  seed  seed used for the calculation.
 * @return  16-bit calculated CRC16-CCITT for the region specified.
 */
MEM_CHECKS_PREFIX uint16_t memchecks_CalculateCrc16Ccitt(uint32_t startAddress, uint16_t size, const uint16_t seed);

#ifdef COLIN_ROM_START
/** Performs a Colin Rom bist 24 bit crc calculation for the data region selected
 *
 * @param[in]  startAddress  Address to the data in extended memory (only available when colin ROM present).
 * @param[in]  size  length of the region to calculate the crc for (in number of bytes).
 * @param[in]  *startCrc   Last intermediate result (or null pointer for first step).
 * @return  32-bit calculated Colin Rom bist intermediate checksum for the region specified.
 *          When the calculation is complete this result must be limited to 24 bit
 */
MEM_CHECKS_PREFIX uint32_t memchecks_CalculateCrc24RomBist(uint32_t startAddress,
                                                           uint16_t size,
                                                           const uint32_t* startCrc);
#endif /* COLIN_ROM_START */

#ifdef UNITTEST
/* As this function does weird things to ram, we don't want the real implementation in unit test,
 * we always want to use the mock. Having the prototype here forces generation of a mock. The implementation
 * of the function has ifndef unittest in order to never use the actual implementation in unit test.
 */
void HAL_BIST_RamNonDestructiveTestInt(ramBistConfiguration_t* ramBistConfiguration);
#endif /* UNITTEST */

/// @}
#endif /* MEM_CHECKS_H */


