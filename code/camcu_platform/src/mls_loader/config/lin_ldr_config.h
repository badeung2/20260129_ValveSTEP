/**
 * @file
 * @brief LIN Loader module configurations
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
 * @ingroup mlx_lin_ldr
 *
 * @details
 */

#ifndef LIN_LDR_CONFIG_H_
#define LIN_LDR_CONFIG_H_

#include <stdint.h>
#include <compiler_abstraction.h>

#include "mls_api.h"
#include "lib_miscio.h"
#include "memory_map.h"
#include "flash_defines.h"

/** The ML_HAS_* options values:
 * - 0: disabled
 * - 1: enabled
 */

/*
 * Options
 */

/* Keep it for compatibility reasons, the PC programming tool can distinguish new chip generation
 * from the previous one
 * MULAN2: 0-3 (loader states)
 * MULAN3: 4, 0 (loader modes)
 * CAMCU: 5 (fixed, to make the difference only, has no special sense for that) */
#define LDR_STATE 5

/** Variables attributes without DP section */
#ifndef LDR_NODP_ATTR
    #if defined(ML_HAS_NO_DP_VAR_ATTR)
        #define LDR_NODP_ATTR
    #else
        #define LDR_NODP_ATTR    __attribute__((dp))
    #endif
#endif

/** Only respond with single frame as required by EOL flash process */
#ifndef LDR_HAS_STUB_READ_MEM
#define LDR_HAS_STUB_READ_MEM 0
#endif

/** Support for Flash reading/programming messages */
#ifndef LDT_HAS_FLASH_FUNCTIONS
#define LDT_HAS_FLASH_FUNCTIONS 0
#endif

/** Support for NVRAM reading/programming messages */
#ifndef LDT_HAS_NVRAM_FUNCTIONS
#define LDT_HAS_NVRAM_FUNCTIONS 0
#endif

/** Support for EEPROM reading/programming messages */
#ifndef LDT_HAS_EEPROM_FUNCTIONS
#define LDT_HAS_EEPROM_FUNCTIONS 0
#endif

/** Support for RAM reading/programming messages */
#ifndef LDT_HAS_RAM_FUNCTIONS
#if (LDT_HAS_FLASH_FUNCTIONS == 1) || (LDT_HAS_EEPROM_FUNCTIONS == 1)
#define LDT_HAS_RAM_FUNCTIONS 1
#else
#define LDT_HAS_RAM_FUNCTIONS 0
#endif
#endif

/** Disable support for RAM execution messages */
#ifndef LDT_DISABLE_RAM_EXEC_FUNCTIONS
#define LDT_DISABLE_RAM_EXEC_FUNCTIONS 1
#endif

/** Support for tables reading/programming messages */
#ifndef LDT_HAS_TABLE_FUNCTIONS
#define LDT_HAS_TABLE_FUNCTIONS 0
#endif

/** Optimization: don't provide negative responses
 * NOTE: when option is disable the LIN Loader doesn't change the RSID, which is important
 * for internal logic to recognise that message has been used for the LIN Loader */
#ifndef LDR_HAS_NEGATIVE_RESPONSES
#define LDR_HAS_NEGATIVE_RESPONSES 1
#endif

/** Protocol extension functionality */
#ifndef LDR_HAS_PROTOCOL_EXTENSION
#define LDR_HAS_PROTOCOL_EXTENSION 1
#endif

/** Max flexible FAST baudrate [Kbaud] */
#ifndef LDR_MAX_FLEX_FAST_BAUDRATE_K
#define LDR_MAX_FLEX_FAST_BAUDRATE_K    250
#endif /* LDR_MAX_FLEX_FAST_BAUDRATE_K */

/* Define minimal Fast loader baudrate value[kBd] (round up) */
#ifndef LDR_MIN_FAST_BAUDRATE_K
#define LDR_MIN_FAST_BAUDRATE_K     (((MLX4_FPLL * 10UL) / (635u * 4u * 2u * 3u)) + 1u)
#endif /* LDR_MIN_FAST_BAUDRATE_K */

/**
 * Debug output for the LIN Loader.
 * Use general project settings.
 */
#define LDR_DEBUG_OUTPUT(info) DEBUG_OUTPUT(info)


/*
 * Constants
 */

/** Flash page size */
#define LDR_FLASH_PAGE_BYTE_SIZE       MEM_FLASH_PAGE_SIZE

#if (LDR_FLASH_PAGE_BYTE_SIZE + 2) > LDT_MAX_DATA_IN_SEGMENTED_TRANSFER
#error "The LIN TL buffer can't fit the LDR_FLASH_PAGE_BYTE_SIZE"
#endif

/** Max EEPROM data size used for ldr_EepromWrite()
 * NOTE: the ldr_EepromWrite() implementation can put additional constraints on that */
#define LDR_MAX_EEPROM_DATA_BYTE_SIZE  LDR_FLASH_PAGE_BYTE_SIZE

/** Max RAM data size used for ldr_RamWrite()
 * NOTE: the ldr_RamWrite() implementation can put additional constraints on that */
#define LDR_MAX_RAM_DATA_BYTE_SIZE    LDR_FLASH_PAGE_BYTE_SIZE

/** MLX4 LIN Clock frequency [KHz] */
#define LDR_MLX4_FPLL   MLX4_FPLL

/*
 * Programmable EEPROM area definitions
 */
#ifndef LDR_PROG_EEPROM_START_INDEX
#define LDR_PROG_EEPROM_START_INDEX               ((uint32_t)MEM_EEPROM_START)
#define LDR_PROG_EEPROM_END_INDEX                 ((uint32_t)(MEM_EEPROM_START + MEM_EEPROM_SIZE))
#endif
#define LDF_PROG_EEPROM_WRITE_PROT_START_INDEX    ((uint32_t)MEM_EEPROM_CAL_START)
#define LDR_EEPROM_PAGE_SIZE                      (MEM_EEPROM_PAGE_SIZE)

/*
 * Programmable Flash area definitions
 */
#ifndef LDR_PROG_FLASH_START_INDEX
#define LDR_PROG_FLASH_START_INDEX  ((uint32_t)MEM_FLASH_START)
#define LDR_PROG_FLASH_END_INDEX    ((uint32_t)(MEM_FLASH_START + MEM_FLASH_SIZE))
#endif

/*
 * Programmable RAM area definitions
 */
#define LDR_PROG_RAM_START_INDEX  ((uint32_t)MEM_RAM_START)
#define LDR_PROG_RAM_END_INDEX    ((uint32_t)(MEM_RAM_START + MEM_RAM_SIZE))

/* FLASH settings */
#define FL_FILLING_PATTERN      0xFF

/** Parity check for read/write commands */
#define LDR_HAS_PARITY_CHECK 1

#endif /* LIN_CONFIG_H_ */
