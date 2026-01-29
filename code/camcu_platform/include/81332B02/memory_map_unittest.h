/**
 * @file
 * @brief CAMCU memory map for unit tests
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
 * @ingroup library
 *
 * @brief CAMCU memory map for unit tests
 * @details
 * @{
 */

#ifndef MEMORY_MAP_UNIT_TEST_H
#define MEMORY_MAP_UNIT_TEST_H

#ifdef UNITTEST

#define MEM_EEPROM_START            (0x1000U)
#define MEM_EEPROM_CS_START         (0x1200U)
#define MEM_MAIN_EEPROM_SIZE        (0x0200U)
#define MEM_EEPROM_SIZE             (0x0240U)

#define MEM_FLASH_CS_START          (0x1500U)
#define MEM_FLASH_CS_SIZE           (0x0080)

#define MEM_FLASH_START             (0x3800)
#define MEM_FLASH_SIZE              (0x1000U)
#define MEM_FLASH_PAGE_SIZE         (0x0080U)

#define MEM_UDS_FLASH_START         (0x3800)
#define MEM_UDS_FLASH_SIZE          (0x0080)

#ifndef MEM_EEPROM_WR_SIZE
/** The area that is available to write to (for unit-tests) */
#define MEM_EEPROM_WR_SIZE          (MEM_EEPROM_PAGE_SIZE * 54U)
#endif

#define MEM_RAM_START               (0x2000U)
#define MEM_RAM_SIZE                (0x1000U)

#define MEM_ROM_START               (0x4000)
#define MEM_ROM_SIZE                (0xA600U)

#define  MEM_COLIN_ROM_START        (0xF800U)
#define  MEM_COLIN_ROM_SIZE         (0x01800U)

#define MEM_FLASH_VECTORS_OFFSET    (0x80)
#endif // UNITTEST

#endif /* MEMORY_MAP_UNIT_TEST_H */
/// @}
