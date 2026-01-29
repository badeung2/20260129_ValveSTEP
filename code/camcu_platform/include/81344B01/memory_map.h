/**
 * @file
 * @brief Memory map description
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
 * @ingroup CAMCU_platform
 *
 * @details
 */

#ifndef MEMORY_MAP_H
#define MEMORY_MAP_H

#include "memory_map_vectors.h"

#define MEM_EEPROM_PAGE_SIZE       (0x0008U)
#define MEM_FLASH_PAGES_IN_SECTOR   16u

#ifdef UNITTEST
    #include "memory_map_unittest.h"
#else

#ifndef MEM_FLASH_START
#define MEM_FLASH_START             (0x8000)
#define MEM_FLASH_SIZE              (0x10000LU)
#define MEM_FLASH_PAGE_SIZE         (0x0080U)
#endif

#if !defined(MEM_UDS_FLASH_START) && defined(UDS_FLASH_START)
#define MEM_UDS_FLASH_START         UDS_FLASH_START
#define MEM_UDS_FLASH_SIZE          UDS_FLASH_SIZE
#endif

#ifndef MEM_FLASH_CS_START
#define MEM_FLASH_CS_START          (0x7C98)
#define MEM_FLASH_CS_SIZE           (0x0080)
#endif

#ifndef  MEM_ROM_START
#define  MEM_ROM_START              (0x2C00)
#define  MEM_ROM_SIZE               (0x5000U)
#endif

#ifndef  MEM_EEPROM_START
#define  MEM_EEPROM_START           ((uint16_t)EEPROM_START)
#define  MEM_EEPROM_SIZE            ((uint16_t)EEPROM_SIZE)
#define  MEM_EEPROM_CAL_START       (MEM_EEPROM_START + 0x01B0U)
#define  MEM_MAIN_EEPROM_SIZE       (0x0200U)

#ifdef SHIFT_EE_CS_DATA /* SHIFTING DATA workaround. Uses the shift of CS EEPROM into the main-area to avoid EEPROM CS accessing */
#define  MEM_EEPROM_CS_START        (MEM_EEPROM_START + 0x0080U)
#warning "EEPROM CS DATA is shifted to the MA area!!!"
#else
#define  MEM_EEPROM_CS_START        (MEM_EEPROM_START + MEM_MAIN_EEPROM_SIZE)
#endif /* SHIFT_EE_CS_DATA */
#endif

#ifdef HAS_EE_WR_AREA_CTRL
#define  MEM_EEPROM_WR_SIZE         MEM_MAIN_EEPROM_SIZE
#else
#define  MEM_EEPROM_WR_SIZE         (MEM_EEPROM_PAGE_SIZE * 54U) /* The area that is available to write to */
#endif /* HAS_EE_WR_AREA_CTRL */

#ifndef  MEM_RAM_START
#define  MEM_RAM_START              (0x0C00U)
#define  MEM_RAM_SIZE               (0x1000U)
#endif

#endif /* UNITTEST */

#define MEM_EEPROM_CS_LAST_PAGE     (MEM_EEPROM_START + MEM_EEPROM_SIZE - 8u)
#define MEM_FLASH_SECTORS           (MEM_FLASH_SIZE / MEM_FLASH_PAGE_SIZE / MEM_FLASH_PAGES_IN_SECTOR)

/* MLX4 memories map definition */
#ifndef  MEM_COLIN_RAM_START
#define  MEM_COLIN_RAM_START        (0x0A00)
#define  MEM_COLIN_RAM_SIZE         (0x0200)
#endif

#ifndef  MEM_COLIN_ROM_START
#define  MEM_COLIN_ROM_START        (COLIN_ROM_START)
#define  MEM_COLIN_ROM_SIZE         (COLIN_ROM_SIZE)
#endif /* MEM_COLIN_ROM_START */

/* FLASH shell definition */
#ifndef  MEM_FLASH_LATCH_START
#define  MEM_FLASH_LATCH_START      (0x7C00)
#define  MEM_FLASH_LATCH_SIZE       (0x0080)
#endif

#define MEM_ROM_VECTORS_OFFSET      (0x300)

#ifndef MEM_FLASH_VECTORS_OFFSET
/** Offset of the flash interrupt vectors from the start of the physical flash memory */
#ifdef MLX81344A01
#define MEM_FLASH_VECTORS_OFFSET    (0x0000)
#else
#define MEM_FLASH_VECTORS_OFFSET    (0x1000)
#endif
#endif

/** Start of rom interrupt vectors */
#define MEM_ROM_VECTORS_START       (MEM_ROM_START)

/** Start of remapped rom interrupt vectors */
#define MEM_ROM_VECTORS_REMAP_START ((uint16_t)MEM_ROM_START + (uint16_t)MEM_ROM_VECTORS_OFFSET)

/** Start of flash interrupt vectors */
#define MEM_FLASH_VECTORS_START     (MEM_FLASH_START)

/** Start of remapped flash interrupt vectors */
#define MEM_FLASH_VECTORS_REMAP_START   (MEM_FLASH_START + MEM_FLASH_VECTORS_OFFSET)


/** Start of flash patch area */
#define MEM_FLASH_PATCH_START       (MEM_FLASH_VECTORS_START + MEM_FLASH_VECTORS_SIZE)

/** Size of flash patch */
#define MEM_FLASH_PATCH_SIZE        (0x0010)

/** Start of flash patch 1 */
#define MEM_FLASH_PATCH_1           (MEM_FLASH_PATCH_START)

/** Start of flash patch 2 */
#define MEM_FLASH_PATCH_2           (MEM_FLASH_PATCH_1 + MEM_FLASH_PATCH_SIZE)

/** Start of flash patch 3 */
#define MEM_FLASH_PATCH_3           (MEM_FLASH_PATCH_2 + MEM_FLASH_PATCH_SIZE)

/** Start of flash patch 4 */
#define MEM_FLASH_PATCH_4           (MEM_FLASH_PATCH_3 + MEM_FLASH_PATCH_SIZE)


/** Start of remapped flash patch area */
#define MEM_FLASH_REMAP_PATCH_START (MEM_FLASH_VECTORS_REMAP_START + MEM_FLASH_VECTORS_SIZE)

/** Start of remapped flash patch 1 */
#define MEM_FLASH_PATCH_1_REMAP     (MEM_FLASH_REMAP_PATCH_START)

/** Start of remapped flash patch 2 */
#define MEM_FLASH_PATCH_2_REMAP     (MEM_FLASH_PATCH_1_REMAP + MEM_FLASH_PATCH_SIZE)

/** Start of remapped flash patch 3 */
#define MEM_FLASH_PATCH_3_REMAP     (MEM_FLASH_PATCH_2_REMAP + MEM_FLASH_PATCH_SIZE)

/** Start of remapped flash patch 4 */
#define MEM_FLASH_PATCH_4_REMAP     (MEM_FLASH_PATCH_3_REMAP + MEM_FLASH_PATCH_SIZE)


#define FW_APP_VERSION_ADDR         (0x17FECLU)
#define FW_PLTF_VERSION_ADDR        (0x17FF0LU)
#define FW_PROT_KEY_ADDR            (0x17FF4LU)


#endif
