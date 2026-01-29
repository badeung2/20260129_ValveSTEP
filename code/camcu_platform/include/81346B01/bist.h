/**
 * @file
 * @brief Provides a set of functions used for the bootloaders and can be used in the Software Application in FLASH
 * @internal
 *
 * @copyright (C) 2015-2018 Melexis N.V.
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
 * @ingroup bist
 * @addtogroup bist BIST
 * @ingroup peripheral_drivers
 *
 * @brief Bist functions provided for memory checks
 * @details
 * @{
 */

#ifndef BIST_H
#define BIST_H

#include <stdint.h>
#include <stdbool.h>
#include "compiler_abstraction.h"
#include "sys_tools.h"
#include "memory.h"
#include "memory_map.h"

typedef enum {
    BIST_FAILED = 0,        /** Occurs when the BIST has failed */
    BIST_OK                 /** The BIST is OK */
} bist_Result_t;

/** Start the HW BIST-check for the ROM and compares the signature with the expected.
 *
 * @param[in]  from  start data pointer (should be 32bit aligned).
 * @param[in]  to  end data pointer (should be 32bit aligned).
 * @param[in]  signature  the signature expected.
 * @retval  BIST_OK  The provided BIST signature is equal to the calculated value.
 * @retval  BIST_FAIL  The provided BIST signature is not equal to the calculated value.
 */
bist_Result_t bist_TestRom(uint32_t from, uint32_t to, uint32_t signature);
#define RomBistTest bist_TestRom __attribute__ ((deprecated("Renamed to bist_TestRom")))

/** Sets the BIST HW with the settings to check the requested FLASH memory area
 *
 * Sets the end point to the latest data in the pages area specified by the pages_number,
 * excluding the latest 32bits.
 * The latest 32bits in the area specified are used to handle the BIST signature expected.
 * The functions assumes the FLASH will have ECC bits after each 64 bit of data in array.
 *
 * @note the expected signature is reseted to 0.
 * @note Start & Stop addresses support the full 20-bit address range.
 *
 * @param[in]  from  start address to start calculating the BIST signature.
 * @param[in]  pages_number  specifies the FLASH pages number being processed with the BIST HW execution.
 */
void bist_PrepareFlashBist(address_t from, uint16_t pages_number);
#ifndef LEGACY_H
#define FLASH_BistPrepare bist_PrepareFlashBist __attribute__ ((deprecated("Renamed to bist_PrepareFlashBist")))
#endif

/** Runs the HW BIST with the pre-set configuration
 *
 * @warning The BIST check stops the MLX16 CPU, no interrupts, DMA access are not executed.
 *          (DMA-access will rise the corresponding error flag)
 */
void bist_RunFlashBist(void);
#ifndef LEGACY_H
#define FLASH_BistRun bist_RunFlashBist __attribute__ ((deprecated("Renamed to bist_RunFlashBist")))
#endif

/** Checks the HW BIST FLASH signature
 *
 * The function compares the BIST signature calculated with the latest 32bit value.
 * In addition, the functions checks the DATA_CURRUPTED (2-bit error) flag after the test.
 *
 * @param[in]  from  start address to start calculating the BIST signature.
 * @param[in] pages_number specifies the FLASH pages number being processed with the BIST HW execution
 * @retval 1 The BIST signature placed at the end of the specified area (latest 32bit area)
 *           is equal to calculated and the double-bit error is not raised during the check.
 * @retval 0 The BIST signature placed at the end of the specified area (latest 32bit area)
 *           is not equal to calculated OR the double-bit error is raised during the check.
 * @warning The BIST check stops the MLX16 CPU, no interrupts, DMA access are not executed.
 *          (DMA-access will rise the corresponding error flag)
 */
bist_Result_t bist_CheckFlashBist(address_t from, uint16_t pages_number);

/** Configures and runs the HW BIST check for some defined area of the FLASH memory
 *
 * After running the HW BIST check this function will compare the signature stored in the latest 32bit area of
 * the specified memory range with the calculated value. The flash double-bit errors will be considered and
 * reported as BIST_FAIL on occurrence.
 *
 * @param[in]  from  start address to start calculating the BIST signature.
 * @param[in]  pages_number  specifies the FLASH pages number being processed with the BIST HW execution
 * @retval  BIST_OK  No BIST errors where detected.
 * @retval  BIST_FAIL  A BIST signature or flash double-bit error was detected.
 * @warning The BIST check stops the MLX16 CPU, no interrupts, DMA access are not executed. (DMA-access will rise
 *          the corresponding error flag)
 */
bist_Result_t bist_TestFlash(address_t from, uint16_t pages_number);
#ifndef LEGACY_H
#define FLASH_BistTest bist_TestFlash __attribute__ ((deprecated("Renamed to bist_TestFlash")))
#endif
#define verifyFlashBist bist_TestFlash __attribute__ ((deprecated("Renamed to bist_TestFlash")))

/** Configures and runs the HW BIST check for application FLASH memory
 *
 * This function shall calculate the BIST starting at the flash application interrupt vectors and ranging the
 * requested amount of pages as defined in the `FW_FLASH_BIST_PAGES_COUNT`.
 * After running the HW BIST check this function will compare the signature stored in the latest 32bit area of
 * the specified memory range with the calculated value. The flash double-bit errors will be considered and
 * reported as BIST_FAIL on occurrence.
 *
 * @retval  BIST_OK  No BIST errors where detected.
 * @retval  BIST_FAIL  A BIST signature or flash double-bit error was detected.
 * @warning  The BIST check stops the MLX16 CPU, no interrupts or DMA accesses are executed. (DMA-access will rise
 *           the corresponding error flag)
 */
bist_Result_t bist_TestApplicationFlash(void);

#ifdef BL_PHY_USE_UDS_ON_LIN
/** Configures and runs the HW BIST check for UDS FLASH memory
 *
 * This function shall calculate the BIST starting at the UDS start offset and ranging the
 * requested amount of pages as defined in the `UDS_FLASH_BIST_PAGES_COUNT`.
 * After running the HW BIST check this function will compare the signature stored in the latest 32bit area of
 * the specified memory range with the calculated value. The flash double-bit errors will be considered and
 * reported as BIST_FAIL on occurrence.
 *
 * @retval  BIST_OK  No BIST errors where detected.
 * @retval  BIST_FAIL  A BIST signature or flash double-bit error was detected.
 * @warning  The BIST check stops the MLX16 CPU, no interrupts or DMA accesses are executed. (DMA-access will rise
 *           the corresponding error flag)
 */
bist_Result_t bist_TestUdsFlash(void);
#endif  /* BL_PHY_USE_UDS_ON_LIN */

/** Configures and runs the HW BIST check for entire FLASH memory
 *
 * After running the HW BIST check this function will compare the signature stored in the latest 32bit area of
 * the specified memory range with the calculated value. The flash double-bit errors will be considered and
 * reported as BIST_FAIL on occurrence.
 *
 * @retval  BIST_OK  No BIST errors where detected.
 * @retval  BIST_FAIL  A BIST signature or flash double-bit error was detected.
 * @warning  The BIST check stops the MLX16 CPU, no interrupts or DMA accesses are executed. (DMA-access will rise
 *           the corresponding error flag)
 */
bist_Result_t bist_TestFullFlash(void);
#ifndef LEGACY_H
#define FLASH_BistTestAuto bist_TestFullFlash __attribute__ ((deprecated("Renamed to bist_TestFullFlash")))
#endif

/** Check if ROM interrupt vector remapping is enabled.
 *
 * @retval true remapping is enabled.
 * @retval false otherwise
 */
STATIC INLINE bool bist_IsRomVectorRemapEnabled(void);

/** Read the flash application vector offset
 *
 * @returns the offset of the flash application interrupt vectors in respect to the flash memory start.
 */
STATIC INLINE uint16_t bist_FlashFwVectorsOffset(void);

/** Execute a quick check on the flash application memory.
 *
 * The function will check if the flash has the correct product specific `FW_FLASH_KEY` stored in
 * flash memory. It does not do any other validations on the rest of the memory!
 * @retval true flash memory is validated as "valid".
 * @retval false otherwise.
 */
STATIC INLINE bool bist_FlashFwFastCheck(void);

/** Read the flash bist start offset in number of sectors from the start of the physical flash memory.
 *
 * This function will read the flash bist start offset from the in flash stored `FW_FLASH_BIST_PAGES_COUNT`.
 * There are no validations done to the number or the flash itself.
 * @returns the flash bist start offset in number of sectors.
 */
STATIC INLINE uint16_t bist_FlashFwBistOffset(void);

/** Read the flash bist physical start address.
 *
 * This function uses the in flash stored `FW_FLASH_BIST_PAGES_COUNT` value to calculate the flash
 * bist start address. There are no validations done to the number or the flash itself.
 * @returns the flash bist physical start address.
 */
STATIC INLINE address_t bist_FlashFwBistStart(void);

/** Read the number of pages for which to check the bist.
 *
 * This function will read the flash bist length from the in flash stored `FW_FLASH_BIST_PAGES_COUNT`.
 * There are no validations done to the number or the flash itself.
 * @returns the number of pages to be checked by the flash bist
 */
STATIC INLINE uint16_t bist_FlashFwBistPages(void);

#define FW_FAST_CHECK bist_FlashFwFastCheck __attribute__ ((deprecated("Renamed to bist_FlashFwFastCheck")))
#define FW_FLASH_BIST_OFFSET bist_FlashFwBistOffset __attribute__ ((deprecated("Renamed to bist_FlashFwBistOffset")))
#define FW_FLASH_BIST_START bist_FlashFwBistStart __attribute__ ((deprecated("Renamed to bist_FlashFwBistStart")))
#define FW_FLASH_BIST_PAGES bist_FlashFwBistPages __attribute__ ((deprecated("Renamed to bist_FlashFwBistPages")))

#ifdef BL_PHY_USE_UDS_ON_LIN
/** Execute a quick check on the flash uds memory.
 *
 * The function will check if the flash has the correct product specific `UDS_FLASH_KEY` stored in
 * flash memory. It does not do any other validations on the rest of the memory!
 * @retval true flash memory is validated as "valid".
 * @retval false otherwise.
 */
STATIC INLINE bool bist_FlashUdsFastCheck(void);

/** Read the UDS flash bist start offset in number of sectors from the start of the physical flash memory.
 *
 * This function will read the flash bist start offset from the in flash stored `UDS_FLASH_BIST_PAGES_COUNT`.
 * There are no validations done to the number or the flash itself.
 * @returns the flash bist start offset in number of sectors.
 */
STATIC INLINE uint16_t bist_FlashUdsBistOffset(void);

/** Read the UDS flash bist physical start address.
 *
 * This function uses the in flash stored `UDS_FLASH_BIST_PAGES_COUNT` value to calculate the flash
 * bist start address. There are no validations done to the number or the flash itself.
 * @returns the flash bist physical start address.
 */
STATIC INLINE address_t  bist_FlashUdsBistStart(void);

/** Read the number of pages for which to check the UDS flash bist.
 *
 * This function will read the flash bist length from the in flash stored `UDS_FLASH_BIST_PAGES_COUNT`.
 * There are no validations done to the number or the flash itself.
 * @returns the number of pages to be checked by the flash bist
 */
STATIC INLINE uint16_t bist_FlashUdsBistPages(void);
#endif  /* BL_PHY_USE_UDS_ON_LIN */

/** Set flags to confirm the RAM memory is completely valid and initialized
 *
 * After next reboot the chip could decide to execute a warm restart with maintaining the RAM content.
 * This function shall be called by the application after full initialization of the system and right before
 * entering the main loop.
 */
STATIC INLINE void allowWarmReboot(void);

/** Clear flags which confirm the RAM memory was completely valid and initialized
 *
 * After next reboot the chip will execute a cold start.
 */
STATIC INLINE void disallowWarmReboot(void);

#if !defined(UNITTEST) /* for unit test, mocks will be generated */
#include "bist_inline_impl.h"
#endif

/// @}
#endif /* BIST_H */
