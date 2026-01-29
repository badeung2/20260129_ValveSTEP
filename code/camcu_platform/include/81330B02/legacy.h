/**
 * @file
 * @brief Legacy functions still existing in some ROMs
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
 * @ingroup legacy
 * @addtogroup legacy Legacy Deprecated Functions
 * @ingroup library
 * @brief Legacy functions still existing in some ROMs
 * @details Functions here are deprecated because of some deficiencies and were replaced by existing cleaner interface.
 *          Because they are still present in old ROMS and can be reused by FLASH applications of that ROMs, they are
 *          still present, but should not be used. If you want to use this just include legacy.h into your application.
 * @deprecated Functions in this group are deprecated and should not be used.
 * @{
 */

#ifndef LEGACY_H
#define LEGACY_H

#include <stdint.h>
#include "lib_clock.h"

typedef enum {
    FL_BIST_FAILED = 0,    /* Occurs when the Flash BIST has failed */
    FL_BIST_OK             /* The BIST is OK */
} fd_FlashBistRes_t;

/** FLASH_BistPrepare sets the BIST HW with the settings to check the FLASH content
 *
 * Sets the start point to start of FLASH,
 * Sets the end point to the latest data in the pages area specified by the pages_number,
 * excluding the latest 32bits.
 * The latest 32bits in the area specified are used to handle the BIST signature expected.
 * The functions assumes the FLASH will have ECC bits after each 64 bit of data in array.
 *
 * @note the expected signature is resetted to 0.
 * @note Start & Stop addresses support the full 20-bit address range.
 *
 * @param[in] pages_number specifies the FLASH pages number being processed with the BIST HW execution
 *
 * @deprecated This function still exists in some ROMs but it is replaced by the new more flexible bist interface in
 * bist.h, which allows performing bist from any start address.
 */
void FLASH_BistPrepare(uint16_t pages_number);


/** FLASH_BistRun runs the HW BIST with the pre-set configuration and checks the signature
 *
 * The function runs the HW BIST and compares the BIST signature calculated with the latest 32bit value.
 * In addition, the functions checks the DATA_CURRUPTED (2-bit error) flag after the test.
 *
 * @param[in] pages_number specifies the FLASH pages number being processed with the BIST HW execution
 * @retval 1 The BIST signature placed at the end of the specified area (latest 32bit area)
 *           is equal to calculated and the double-bit error is not raised during the check.
 * @retval 0 The BIST signature placed at the end of the specified area (latest 32bit area)
 *           is not equal to calculated OR the double-bit error is raised during the check.
 * @warning The BIST check stops the MLX16 CPU, no interrupts, DMA access are not executed.
 *          (DMA-access will rise the corresponding error flag)
 *
 * @deprecated This function still exists in some ROMs but it is replaced by the new more flexible bist interface in
 * bist.h, which allows performing bist from any start address.
 */
fd_FlashBistRes_t FLASH_BistRun(uint16_t pages_number);

/** FLASH_BistTest Configures and runs the HW BIST check for FLASH
 * @param[in] pages_number specifies the FLASH pages number being processed with the BIST HW execution
 * @retval FL_BIST_OK The BIST signature placed at the end of the specified area (latest 32bit area) is equal to calculated and
 * the double-bit error is not raised during the check.
 * @retval FL_BIST_FAIL The BIST signature placed at the end of the specified area (latest 32bit area) is no equal to calculated OR
 * the double-bit error is raised during the check.
 * @warning The BIST check stops the MLX16 CPU, no interrupts, DMA access are not executed. (DMA-access will rise the corresponding error flag)
 *
 * @deprecated This function still exists in some ROMs but it is replaced by the new more flexible bist interface in
 * bist.h. Function is now

 */
fd_FlashBistRes_t FLASH_BistTest(uint16_t pages_number);


/** FLASH_BistTestAuto Configures and runs the HW BIST check for entire FLASH array
 * @retval FL_BIST_OK The BIST signature placed at the end of the specified area (latest 32bit area) is equal to calculated and
 * the double-bit error is not raised during the check.
 * @retval FL_BIST_FAIL The BIST signature placed at the end of the specified area (latest 32bit area) is no equal to calculated OR
 * the double-bit error is raised during the check.
 * @warning The BIST check stops the MLX16 CPU, no interrupts, DMA access are not executed. (DMA-access will rise the corresponding error flag)
 *
 * @deprecated This function still exists in some ROMs but it is replaced by the new bist interface in bist.h
 */
fd_FlashBistRes_t FLASH_BistTestAuto(void);


/** ldt_mem_Copy Copy source memory into destination buffer.
 *
 * @note This function should never be used to copy itself or an system error will occur.
 * Cause this function is placed in boot flash, the boot_flash should not be copied with this function.
 * @param[in]   _dst    destination address
 * @param[in]   _src    source address
 * @param[in]   len     data length
 * @return      destination address
 *
 * @warning Regular memcpy is now used
 *
 * @deprecated This function still exists in some ROMs but is replaced in new ROMs by memcpy
 */
void* ldt_mem_Copy(uint8_t* _dst, const uint8_t* _src, uint16_t len);


/** Byte wise memory initialization with word aligned write.
 *
 * Can be used to init single bytes in areas with word write access.
 * @param[in]   _dst    first destination address
 * @param[in]   val     data byte value
 * @param[in]   len     data length
 * @return      destination address
 *
 * @warning Regular memset is now used
 *
 * @deprecated This function still exists in some ROMs but is replaced in new ROMs by memset
 */
void* ldt_mem_Set(uint8_t* _dst, uint8_t val, uint16_t len);


/** Sets the RCO, without FLASH/EEPROM wait-states configuration
 *
 * Usage example:
 * @code
 * RC_Settings_t tmp;
 * tmp.u = EE_MS_TRIM6_VALUE;
 * SetSystemSpeedNoPredivider(tmp);
 * @endcode
 *
 * @note The FLASH/EEPROM wait-states configuration is initialized to zero
 * @param[in] trim_cfg The RCO trimming and FLASH+EEPROM wait-states combined in one 16bit value.
 */
void SetSystemSpeedNoPredivider(const RC_Settings_t trim_cfg);

#endif
/** @} */
