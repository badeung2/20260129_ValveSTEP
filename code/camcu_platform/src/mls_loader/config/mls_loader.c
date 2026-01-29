/**
 * @file
 * @brief Melexis LIN slave loader in ROM mode
 * @internal
 *
 * @copyright (C) 2020 Melexis N.V.
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
 * @ingroup mls_loader
 *
 * @details This file contains Melexis LIN slave loader in ROM mode.
 */

#include <stdbool.h>
#include <string.h>
#include "bist.h"
#include "bl_main.h"
#include "bl_tools.h"
#include "bl_tables.h"
#include "eeprom_drv.h"
#include "flash_drv.h"
#include "lib_wdg.h"
#include "lin_ldr.h"
#include "mls_api.h"
#include "version.h"

#include "mls_loader.h"


/** The Read by ID SID value */
#define MD_READ_BY_ID_SID 0xB2u

/** The Read by ID Response SID value */
#define MD_READ_BY_ID_RSID (MD_READ_BY_ID_SID + 0x40u)

/** Minimal data expected into the Read by ID request (only data bytes w/o SID) */
#define MD_MIN_RBI_DATA_LEN 0x05u


/** Read by ID: Melexis device ID response message structure for Enter LIN Programming mode */
typedef struct PACKED {
    uint16_t prod_id_low; /**< Melexis project ID (low part) */
    uint16_t prod_id_high; /**< Melexis project ID (high part) */
    uint8_t version; /**< PLTF version */
} mls_loader_EnterLinProgModeRes_t;
ASSERT(sizeof(mls_loader_EnterLinProgModeRes_t) == MD_MIN_RBI_DATA_LEN);

#if (LDT_HAS_FLASH_FUNCTIONS == 1) || (LDT_HAS_NVRAM_FUNCTIONS == 1) || (LDT_HAS_EEPROM_FUNCTIONS == 1) || (LDT_HAS_RAM_FUNCTIONS == 1)
/** Return the timeout function for any write operations
 * @return  timeout in ms
 */
STATIC uint8_t mls_loader_GetWriteTimeout_ms(void);
#endif

/** Callout function.
 * Ask application permission to enter programming mode.
 * This is default implementation which always does permit.
 * @return bool:
 *  - true - application is stopped and programming mode can be entered
 *  - false - programming mode entering isn't possible
 */
__attribute__((weak)) bool fw_lepm_ApplicationStop(void);

/** Enters LIN Loader programming mode.
 * @note Function perform device reset.
 */
STATIC void mls_loader_EnterProgMode(void);

/** LIN TL handler.
 * Prepare the response with positive response to confirm entering of the programming mode.
 * @param[in] transfer  LIN TL transmission buffer
 */
STATIC void mls_loader_BuildPositiveResponseEnterProg(LINDiagTransfer_t* transfer);

#if (LDT_HAS_FLASH_FUNCTIONS == 1) || (LDT_HAS_NVRAM_FUNCTIONS == 1) || (LDT_HAS_EEPROM_FUNCTIONS == 1) || (LDT_HAS_RAM_FUNCTIONS == 1)
/** Default read the memory
 * @param[in] dst_add_16bits    pointer to destination memory block
 * @param[in] p_src_add_20bits  pointer to source memory block address
 * @param[in] data_size         memory block size
 * @return  operation result
 */
STATIC bool mls_loader_ReadMem(uint8_t* dst_add_16bits,
                               const uint32_t* p_src_add_20bits,
                               uint16_t data_size);
#endif

#if (LDT_HAS_RAM_FUNCTIONS == 1)
/** Default write the RAM function
 * @param[in] p_dest_addr_20bits    pointer to destination memory block address which is word aligned
 * @param[in] data_size             memory block size
 * @param[in] data_buffer           pointer to source memory block
 * @return  operation result
 */
STATIC bool mls_loader_RamWrite(const uint32_t* p_dest_addr_20bits,
                                uint16_t data_size,
                                const uint8_t* data_buffer);
#endif

#if (LDT_HAS_EEPROM_FUNCTIONS == 1)
/** Default write the EEPROM function
 * @param[in] p_dest_addr_20bits    pointer to destination memory block address which is word aligned
 * @param[in] data_size             memory block size
 * @param[in] data_buffer           pointer to source memory block
 * @return  operation result
 */
STATIC bool mls_loader_EepromWrite(const uint32_t* p_dest_addr_20bits,
                                   uint16_t data_size,
                                   const uint8_t* data_buffer);
#endif

#if (LDT_HAS_FLASH_FUNCTIONS == 1)
/** Default write the Flash function
 * @param[in] p_dest_addr_20bits    pointer to destination memory block address which is word aligned
 * @param[in] data_size             memory block size
 * @param[in] data_buffer           pointer to source memory block
 * @return  operation result
 */
STATIC bool mls_loader_FlashWrite(const uint32_t* p_dest_addr_20bits,
                                  uint16_t data_size,
                                  const uint8_t* data_buffer);

/** Initiate sector flash erase.
 * @param[in] p_dest_addr_20bits   Flash destination sector address to erase;
 *                                 FLASH_ERASE_ALL_SECTORS value is used to erase full Flash.
 */
STATIC void mls_loader_FlashErase(const uint32_t* p_dest_addr_20bits);
#endif

#if (LDR_HAS_PROTOCOL_EXTENSION == 1)
/** Default CRC function on memory block
 *
 * Optimized implementation of CRC16 using the CCITT polynomial 1021
 * (X^16 + X^12 + X^5 + 1)
 *
 * The function is based on 'mathlib.h -> crc_ccit'.
 * @note: the 16-bit addressed memory is supported only.
 *
 * @param[in] memoryAddress_20bits  pointer to source memory block address
 * @param[in] size                  memory block size
 *
 * @return  CRC calculated on specified addresses
 */
STATIC uint16_t mls_loader_CalculateCRC16(const uint32_t* memoryAddress_20bits, uint16_t size);
#endif

/** LIN Loader private RAM */
STATIC LDR_t blLinLdr;

/** Flash and eeprom write keys */
fl_ctrl_keys_t bl_fl_ctrl_keys;

/** Adapter for memory functions.
 * It allows to replace the memory functions by rewriting this RAM area */
STATIC CONST ldr_RamFunctions_t ldrRamFunctions = {
#if (LDT_HAS_FLASH_FUNCTIONS == 1) || (LDT_HAS_NVRAM_FUNCTIONS == 1) || (LDT_HAS_EEPROM_FUNCTIONS == 1) || (LDT_HAS_RAM_FUNCTIONS == 1)
    .ldr_ReadMem = mls_loader_ReadMem,
    .ldr_GetWriteTimeout_ms = mls_loader_GetWriteTimeout_ms,
#else
    .ldr_ReadMem = NULL,
    .ldr_GetWriteTimeout_ms = NULL,
#endif
#if (LDT_HAS_RAM_FUNCTIONS == 1)
    .ldr_RamWrite = mls_loader_RamWrite,
#else
    .ldr_RamWrite = NULL,
#endif
#if (LDT_HAS_EEPROM_FUNCTIONS == 1)
    .ldr_EepromWrite = mls_loader_EepromWrite,
#else
    .ldr_EepromWrite = NULL,
#endif
#if (LDT_HAS_FLASH_FUNCTIONS == 1)
    .ldr_FlashWrite = mls_loader_FlashWrite,
    .ldr_FlashErase = mls_loader_FlashErase,
#else
    .ldr_FlashWrite = NULL,
    .ldr_FlashErase = NULL,
#endif
#if (LDR_HAS_PROTOCOL_EXTENSION == 1)
    .ldr_pe_CalculateCRC16 = mls_loader_CalculateCRC16,
#else
    .ldr_pe_CalculateCRC16 = NULL,
#endif
    .ldr_ApplicationStop = fw_lepm_ApplicationStop,
    .ldr_EnterProgMode = mls_loader_EnterProgMode,
    .ldr_BuildPositiveResponseEnterProg = mls_loader_BuildPositiveResponseEnterProg,
};

void mls_loader_Init(void)
{
    /* clear flash access keys */
    bl_fl_ctrl_keys.Erase_key = 0u;
    bl_fl_ctrl_keys.Unlock_key = 0u;
    bl_fl_ctrl_keys.Write_key = 0u;
    bl_fl_ctrl_keys.EepromWriteKey = 0u;
    bl_fl_ctrl_keys.bllrPattern = 0u;

    /* Enable Melexis LIN Loader */
    ldr_InitRamTable((ldr_RamFunctions_t*)&ldrRamFunctions);

    /* Enable only "Enter programming mode" message detection */
    blLinLdr.MlxLinLdrSession = false;
    ldr_Init(&blLinLdr);
}

/** Reset the MLX16 */
void ldr_ResetMlx16(void)
{
    disallowWarmReboot();
    MLX16_RESET_COLD();
}

#if (LDT_HAS_FLASH_FUNCTIONS == 1) || (LDT_HAS_NVRAM_FUNCTIONS == 1) || (LDT_HAS_EEPROM_FUNCTIONS == 1) || (LDT_HAS_RAM_FUNCTIONS == 1)
STATIC uint8_t mls_loader_GetWriteTimeout_ms(void)
{
    uint8_t timeout = 0u;

    if ((blLinLdr.memoryAddress >= (uint32_t)MEM_FLASH_START) &&
        (blLinLdr.memoryAddress < ((uint32_t)MEM_FLASH_START + (uint32_t)MEM_FLASH_SIZE))) {
        timeout = (uint8_t)mulU16_U8byU8(7u, blLinLdr.memorySize / LDR_FLASH_PAGE_BYTE_SIZE);
    } else if ((blLinLdr.memoryAddress >= (uint32_t)MEM_EEPROM_START) &&
               (blLinLdr.memoryAddress < ((uint32_t)MEM_EEPROM_START + (uint32_t)MEM_EEPROM_SIZE))) {
        timeout = (uint8_t)mulU16_U8byU8(15u, blLinLdr.memorySize / LDR_EEPROM_PAGE_SIZE);
    }

    return timeout;
}
#endif

bool fw_lepm_ApplicationStop(void)
{
    /* Always allow enter programming mode by default */
    return true;
}

STATIC void mls_loader_EnterProgMode(void)
{
    blLinLdr.MlxLinLdrSession = true;
}

STATIC void mls_loader_BuildPositiveResponseEnterProg(LINDiagTransfer_t* transfer)
{
    transfer->response.respSId = MD_READ_BY_ID_RSID;
    transfer->response.dataLen = MD_MIN_RBI_DATA_LEN;

    mls_loader_EnterLinProgModeRes_t* pEnterLinProgRes = (mls_loader_EnterLinProgModeRes_t*)transfer->response.data;
    pEnterLinProgRes->prod_id_low = (uint16_t)PRODUCT_VERSION_32;
    pEnterLinProgRes->prod_id_high = (uint16_t)(PRODUCT_VERSION_32 >> 16);
    pEnterLinProgRes->version = LDR_STATE;
}

#if (LDT_HAS_FLASH_FUNCTIONS == 1) || (LDT_HAS_NVRAM_FUNCTIONS == 1) || (LDT_HAS_EEPROM_FUNCTIONS == 1) || (LDT_HAS_RAM_FUNCTIONS == 1)
STATIC bool mls_loader_ReadMem(uint8_t* dst_add_16bits,
                               const uint32_t* p_src_add_20bits,
                               uint16_t data_size)
{
    const uint8_t* local_dst_add_16bits = dst_add_16bits;
    void* local_dst = __builtin_assume_aligned(local_dst_add_16bits, 2u);
    (void)memcpy(local_dst, (const void*)((uint16_t)*p_src_add_20bits), (size_t)data_size);
    return true;
}
#endif

#if (LDT_HAS_RAM_FUNCTIONS == 1)
STATIC bool mls_loader_RamWrite(const uint32_t* p_dest_addr_20bits,
                                uint16_t data_size,
                                const uint8_t* data_buffer)
{
    /* Following parameters checks are done before the function call:
     * 1. MLX16ERROR_OUT_OF_BOUNDS (less or more)
     * 2. MLX16ERROR_WRITE_EXCEEDS_PAGE
     *
     * Note: the alignment check isn't done; return the false if any input values violations
     */

    if (*p_dest_addr_20bits != (uint16_t)&rf_empty_func) {
        /* Note: memcpy can not write to memory above 64k address range! */
        void* local_dst = __builtin_assume_aligned((void*)(uint16_t)(*p_dest_addr_20bits), 2u);
        (void)memcpy(local_dst, (const void*)data_buffer, (size_t)data_size);
    } else {
        (void)memcpy(&bl_fl_ctrl_keys, (const void*)data_buffer, (size_t)data_size);
    }

    return true;
}
#endif /* LDT_HAS_RAM_FUNCTIONS == 1 */

#if (LDT_HAS_EEPROM_FUNCTIONS == 1)
STATIC bool mls_loader_EepromWrite(const uint32_t* p_dest_addr_20bits,
                                   uint16_t data_size,
                                   const uint8_t* data_buffer)
{
    bool retval = true;
    uint16_t l_data_size = data_size;
    /* Following parameters checks are done before the function call:
     * 1. MLX16ERROR_OUT_OF_BOUNDS (less or more)
     * 2. MLX16ERROR_WRITE_EXCEEDS_PAGE (max supported data size up the LIN TL buffer)
     *
     * Note: the alignment check isn't done; return the false if any input values violations
     */

    /* Check the destination address alignment on 8 bytes */
    if (((*p_dest_addr_20bits % LDR_EEPROM_PAGE_SIZE) == 0u) && ((data_size % LDR_EEPROM_PAGE_SIZE) == 0)) {
        /* The function implementation supports data length up to LIN TL buffer
         * NOTE: Only full EEPROM pages are written */
        uint8_t eePageCnt = 0;
        while (l_data_size >= LDR_EEPROM_PAGE_SIZE) {
            uint16_t word_buffer[LDR_EEPROM_PAGE_SIZE / 2] __attribute__ ((aligned (2))); /* Only fit one Flash page */
            uint8_t* byte_buffer = (uint8_t*)word_buffer;
            uint16_t p_dest_addr_16bits = (uint16_t)*p_dest_addr_20bits;
            p_dest_addr_16bits += eePageCnt * LDR_EEPROM_PAGE_SIZE;

            /* Helper copying procedure to prepare word aligned data for the default EEPROM library */
            (void)memcpy(byte_buffer, &data_buffer[eePageCnt * LDR_EEPROM_PAGE_SIZE], LDR_EEPROM_PAGE_SIZE);

            EEPROM_WriteWord64_blocking(p_dest_addr_16bits, word_buffer, bl_fl_ctrl_keys.EepromWriteKey);

            l_data_size -= LDR_EEPROM_PAGE_SIZE;
            eePageCnt++;
        }
    } else {
        retval = false;
    }

    return retval;
}
#endif /* LDT_HAS_EEPROM_FUNCTIONS == 1 */

#if (LDT_HAS_FLASH_FUNCTIONS == 1)
STATIC bool mls_loader_FlashWrite(const uint32_t* p_dest_addr_20bits,
                                  uint16_t data_size,
                                  const uint8_t* data_buffer)
{
    /* Following parameters checks are done before the function call:
     * 1. MLX16ERROR_OUT_OF_BOUNDS (less or more)
     * 2. MLX16ERROR_WRITE_EXCEEDS_PAGE
     * 3. MLX16ERROR_START_ADDRESS_NOT_PAGE_ALIGNED
     *
     * Return the false if any additional input values violations
     */
    bool blRetval = false;

    if ((data_size % LDR_FLASH_PAGE_BYTE_SIZE) == 0u) {
        /* Helper word aligned buffer to use default Flash library */
        FLASH_WriteStruct_t BlFlashWriteCfg;
        uint16_t word_buffer[LDR_FLASH_PAGE_BYTE_SIZE / 2] __attribute__ ((aligned (2))); /* Only fit one Flash page */
        uint8_t* byte_buffer = (uint8_t*)word_buffer;

        /* Helper copying procedure to prepare word aligned data for the default Flash library */
        (void)memcpy(byte_buffer, data_buffer, LDR_FLASH_PAGE_BYTE_SIZE);

        /* Relative to the Flash start destination address divided on 128 to get the FLASH's page number */
        uint16_t page_number = (uint16_t)((*p_dest_addr_20bits - (uint32_t)MEM_FLASH_START) >> 7);

        /* setup flash for page write */
        BlFlashWriteCfg.func = &bl_fl_ctrl_keys;
        BlFlashWriteCfg.data = word_buffer;
        BlFlashWriteCfg.page_number = page_number;
        BlFlashWriteCfg.write_mode = FL_WNV_PAE;
        BlFlashWriteCfg.write_state = flIdle;

        /* notify flash driver about application specific flash write configuration table */
        flash_state = &BlFlashWriteCfg;

        /* waiting until the flash is available */
        WaitForEEandFL(&BlFlashWriteCfg);

        (void) FLASH_Write();

        /* wait for page write operation to finish */
        WaitForEEandFL(&BlFlashWriteCfg);

        /* Uninitilize global pointer */
        BlFlashWriteCfg.data = NULL;
        flash_state = NULL;

        blRetval = true;
    } else {
        blRetval = false;
    }

    return blRetval;
}

STATIC void mls_loader_FlashErase(const uint32_t* p_dest_addr_20bits)
{
    FLASH_WriteStruct_t BlFlashPreWriteCfg;

    /* setup flash for sector pre-write */
    BlFlashPreWriteCfg.func = &bl_fl_ctrl_keys;
    BlFlashPreWriteCfg.data = NULL;
    BlFlashPreWriteCfg.page_number = ((uint16_t)((*p_dest_addr_20bits - (uint32_t)MEM_FLASH_START) >> 7));
    BlFlashPreWriteCfg.write_mode = FL_WNV_SEC;
    BlFlashPreWriteCfg.write_state = flIdle;

    /* notify flash driver about application specific flash write configuration table */
    flash_state = &BlFlashPreWriteCfg;

    /* waiting until the Flash is available */
    WaitForEEandFL(&BlFlashPreWriteCfg);

    (void) FLASH_Write();

    /* waiting until the Flash is available again */
    WaitForEEandFL(&BlFlashPreWriteCfg);

    flash_state = NULL;
}
#endif /* LDT_HAS_FLASH_FUNCTIONS == 1 */

#if LDR_HAS_PROTOCOL_EXTENSION == 1
STATIC uint16_t mls_loader_CalculateCRC16(const uint32_t* memoryAddress_20bits, uint16_t size)
{
    /* Should wait until the Flash is ready for read */

    uint16_t i;
    uint16_t crc = 0xFFFFu;

    const uint8_t* src = (uint8_t*) ((uint16_t) *memoryAddress_20bits);

    for (i = 0; i < size; i++) {
        crc = (uint16_t) (crc >> 8) | (crc << 8);
        crc ^= *src;
        src++;
        crc ^= (uint16_t) (crc & 0xFFu) >> 4;
        crc ^= (crc << 8) << 4;
        crc ^= ((crc & 0xFFu) << 4) << 1;

        if ((i & 0x00FFu) == 0u) {
            WDG_conditionalAwdRefresh();
        }
    }

    return crc;
}
#endif
