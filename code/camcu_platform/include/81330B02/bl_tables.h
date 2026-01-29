/**
 * @file
 * @brief The RAM-content expected for the Bootloader and/or startup sequence
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
 * @ingroup bootloaders
 *
 * @details
 * The RAM-content expected for the Bootloader and/or startup sequence
 * @{
 */

#ifndef BL_TABLES_H
#define BL_TABLES_H

#include <stdint.h>
#include "bl_defs.h"

#ifndef AWD_PERIOD
#error "'AWD_PERIOD' has to be configured in accordance with Polarion Amalthea Configuration Requirement"
#endif

#ifndef BL_AWD_TIMEOUT
/** Sets the timeout period in milliseconds for the bootloader (BL). When no communication with device performed in
 * empty FLASH or Bootloader mode, IC counts the number of AWD opened-window events and resets the IC
 * after this specified timeout. When set to 0 the timeout is disabled.
 */
#define BL_AWD_TIMEOUT 2000u
#endif

#define BL_TABLE_KEY            0x05154832UL
#define BL_TABLE_NAD            0x01u

#ifndef MAX_RESET_COUNTER
#define MAX_RESET_COUNTER       0u
#endif /* MAX_RESET_COUNTER */

#include "flash_defines.h"
/**
 * This structure defines a set of shared parameters, needed to be stored, re-inited and analized after the each IC reset.
 */
typedef struct {
    uint32_t key;           /**< Key which should be set to determine the data integrity. */
    uint16_t NAD;           /**< Personal address. Used to store the NAD from Application mode. */
    bl_Mode_t prog_mode;    /**< Programming-mode bit. When - the system is working in programming mode, i.e. data parser executes the BL-commands instead of Empty-flash's. */
    volatile uint16_t reset_counter;    /**< Calculates the 'BAD'-resets for the FW application. Turns to the "Empty-FLASH" mode if this counter reaches the limit. Should be reseted by the FW. */
#if (BL_AWD_TIMEOUT > 0)
    uint16_t awd_counter;   /**< Number of AWD time periods (until the window is opened for acknowledge) */
#endif  /* BL_AWD_TIMEOUT */
#ifdef HAS_UDS_BL_TABLE
    uint8_t expect_response_req;    /**< Request ID of the response frame to be transmitted after reboot. */
    uint8_t expect_response_data;   /**< Extra response frame data to be sent */
#endif
} bl_table_t;

/** Describe the Flash / EEPROM protection keys */
typedef struct {
    fl_ctrl_keys_t fl_ctrl_keys;
    uint16_t unused[2];
} rf_empty_func_t;


/* Shared structure for the ROM-based API and libraries. Shares the data between reset-states and Application vs Bootloader modes. The bist flags are also here... */
extern bl_table_t bl_table;
extern rf_empty_func_t rf_empty_func;

typedef uint16_t (* mel_callback_func_t)(void); /* Common callback function type definition */

/// @}
#endif

