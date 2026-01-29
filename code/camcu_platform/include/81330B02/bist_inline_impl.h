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
 *
 * @details
 */

#ifndef BIST_INLINE_IMPL_H
#define BIST_INLINE_IMPL_H

#include "io.h"
#include "compiler_abstraction.h"
#include "bl_bist.h"
#include "bl_tools.h"
#include "version.h"
#include "memory_map.h"

/** Prepare a BIST cycle
 *
 * @param from Start address of the memory to check (should be 32 bit aligned)
 * @param to End address of the memory to check (should be 32 bit aligned)
 * @param seed Seed value for the BIST calculation
 * @param ecc_pos ECC position
 */
STATIC INLINE void bist_InternalPrepare(uint32_t from, uint32_t to, uint32_t seed, uint8_t ecc_pos)
{
    IO_SET(ROM_BIST, ECC_POSITION, ecc_pos,
           BIST,            0u,
           SINGLE_RAMP,     1u,
           MASK_SIG_ERR,    0u);

    IO_SET(ROM_BIST, ADD_START_L,     (uint16_t)from);
    IO_SET(ROM_BIST, ADD_START_H,     (uint16_t)(from >> 16));


    /* Safety concept revision: SW test, ROM BIST and other bandwidth consuming tests requires another strategy */
    IO_SET(ROM_BIST, ADD_STOP_L,       (uint16_t)to);
    IO_SET(ROM_BIST, ADD_STOP_H,       (uint16_t)(to >> 16));

    IO_SET(ROM_BIST, SIG_EXPECTED_L, 0x0u);
    IO_SET(ROM_BIST, SIG_EXPECTED_H, 0x0u);

    IO_SET(ROM_BIST, SIG_RECEIVED_L, (uint16_t)seed);
    IO_SET(ROM_BIST, SIG_RECEIVED_H, (uint16_t)(seed >> 16));
}

/** Start a BIST cycle
 *
 * @param  key  key for starting the bist module
 */
STATIC INLINE void bist_InternalStart(uint16_t key)
{
    /* Run the ROM bist */
    IO_SET(ROM_BIST, START_BIST, key);
    /* CPU will stop here until the BIST's finishes his job. */

    NOP(); /* Inserted to let the HW BIST to not to skip the signature check in FX */
    NOP();
}

STATIC INLINE bool bist_IsRomVectorRemapEnabled(void)
{
#if defined (HAS_ROM_VECTOR_REMAP)
    const uint16_t* flash_key_addr =
        (uint16_t*)((uint16_t)MEM_FLASH_START + (uint16_t)MEM_FLASH_OFFSET_FW_FLASH_KEY);
    const uint16_t* flash_key_remap_addr =
        (uint16_t*)((uint16_t)MEM_FLASH_START + (uint16_t)MEM_FLASH_OFFSET_FW_FLASH_KEY +
                    (uint16_t)MEM_FLASH_VECTORS_OFFSET);

    if ((*flash_key_addr != (FLASH_FW_PATTERN ^ PRODUCT_VERSION_16)) &&
        (*flash_key_remap_addr == (FLASH_FW_PATTERN ^ PRODUCT_VERSION_16))) {
        return (true);
    } else {
        return (false);
    }
#else
    return (false);
#endif
}

STATIC INLINE uint16_t bist_FlashFwVectorsOffset(void)
{
    uint16_t retval = MEM_FLASH_VECTORS_OFFSET;

#if defined (HAS_ROM_VECTOR_REMAP)
    if (!bist_IsRomVectorRemapEnabled()) {
        retval = 0u;
    }
#endif

    return (retval);
}

STATIC INLINE bool bist_FlashFwFastCheck(void)
{
    const uint16_t* flash_key_addr = (uint16_t*)((uint16_t)MEM_FLASH_START + bist_FlashFwVectorsOffset() +
                                                 (uint16_t)MEM_FLASH_OFFSET_FW_FLASH_KEY);
    return (*flash_key_addr == (FLASH_FW_PATTERN ^ PRODUCT_VERSION_16));
}

STATIC INLINE uint16_t bist_FlashFwBistOffset(void)
{
    const uint16_t* flash_bist_addr = (uint16_t*)((uint16_t)MEM_FLASH_START + bist_FlashFwVectorsOffset() +
                                                  (uint16_t)MEM_FLASH_OFFSET_FW_FLASH_BIST_PAGES_COUNT);
    return ((*flash_bist_addr & 0xF000u) >> 12);
}

STATIC INLINE address_t  bist_FlashFwBistStart(void)
{
    return ((address_t)MEM_FLASH_START +
            ((address_t)bist_FlashFwBistOffset() * MEM_FLASH_PAGES_IN_SECTOR * MEM_FLASH_PAGE_SIZE));
}

STATIC INLINE uint16_t bist_FlashFwBistPages(void)
{
    const uint16_t* flash_bist_addr = (uint16_t*)((uint16_t)MEM_FLASH_START + bist_FlashFwVectorsOffset() +
                                                  (uint16_t)MEM_FLASH_OFFSET_FW_FLASH_BIST_PAGES_COUNT);
    return (*flash_bist_addr & 0x0FFFu);
}

#ifdef BL_PHY_USE_UDS_ON_LIN
STATIC INLINE bool bist_FlashUdsFastCheck(void)
{
    extern CONST uint16_t uds_flash_key;
    return (uds_flash_key == (FLASH_UDS_PATTERN ^ PRODUCT_VERSION_16));
}

STATIC INLINE uint16_t bist_FlashUdsBistOffset(void)
{
    extern CONST uint16_t uds_flash_bist_page_count;
    return ((uds_flash_bist_page_count & 0xF000u) >> 12);
}

STATIC INLINE address_t  bist_FlashUdsBistStart(void)
{
    return ((address_t)MEM_FLASH_START +
            ((address_t)bist_FlashUdsBistOffset() * MEM_FLASH_PAGES_IN_SECTOR * MEM_FLASH_PAGE_SIZE));
}

STATIC INLINE uint16_t bist_FlashUdsBistPages(void)
{
    extern CONST uint16_t uds_flash_bist_page_count;
    return (uds_flash_bist_page_count & 0x0FFFu);
}
#endif  /* BL_PHY_USE_UDS_ON_LIN */

STATIC INLINE void allowWarmReboot(void)
{
    bistHeader = C_CHIP_HEADER;
    bistResetInfo = C_CHIP_STATE_CMD_RESET;
    SetRstatFlag();
}

STATIC INLINE void disallowWarmReboot(void)
{
    ResetRstatFlag();
    bistHeader = C_CHIP_HEADER_INVALID;
}

#endif
