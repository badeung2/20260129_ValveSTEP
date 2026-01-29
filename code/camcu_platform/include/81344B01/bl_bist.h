/**
 * @file
 * @brief Reset BIST codes
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
 * @ingroup bist
 * @ingroup bl_bist
 * @addtogroup bl_bist
 * @ingroup library
 *
 * @brief Reset BIST codes
 * @details
 * @{
 */

#ifndef BL_BIST_H
#define BL_BIST_H

#include <syslib.h>
#include "io.h"
#include "lib_bist_config.h"


/* Default EPM command settings. Normally they are assigned by project-specific settings from the "lib_bist_config.h" file */
#ifndef C_CHIP_PROJECT_STATE_PPM_CMD_EPM
#define C_CHIP_PROJECT_STATE_PPM_CMD_EPM    (0xB065u)
#endif /* C_CHIP_PROJECT_STATE_PPM_CMD_EPM */
#ifndef C_CHIP_PROJECT_STATE_LIN_CMD_EPM
#define C_CHIP_PROJECT_STATE_LIN_CMD_EPM    (0xB070u)
#endif /* C_CHIP_PROJECT_STATE_LIN_CMD_EPM */
#ifndef C_CHIP_PROJECT_STATE_MEL_CMD_EPM
#define C_CHIP_PROJECT_STATE_MEL_CMD_EPM    (0xB06Du)
#endif /* C_CHIP_PROJECT_STATE_MEL_CMD_EPM */
#ifndef C_CHIP_PROJECT_STATE_UDS_CMD_EPM
#define C_CHIP_PROJECT_STATE_UDS_CMD_EPM    (0xB075u)
#endif /* C_CHIP_PROJECT_STATE_UDS_CMD_EPM */

/* Warm-Reset Command default definition */
#ifndef C_CHIP_PROJECT_STATE_CMD_RESET
#define C_CHIP_PROJECT_STATE_CMD_RESET      (0xB252u)
#endif /* C_CHIP_PROJECT_STATE_CMD_RESET */

/** BistHeader_t enumeration holds 16-bit signs to define the startup expectations for the bistHeader */
typedef enum bist_header_e {
    C_CHIP_HEADER = 0xB598u,                       /**< Used for simple safety check of data integrity. Should be placed BEFORE the Bist Info data, in bistHeader */
    C_CHIP_HEADER_INVALID = 0x0000u,               /**< Set the bistHeader to an invalid value */
} BistHeader_t;

/** BistResetInfo_t enumeration holds 16-bit signs to define the startup expectations for the bistResetInfo*/
typedef enum BistResetInfo_e {
    C_CHIP_STATE_COLD_START = 0x0000u,             /**< "Cold-start" sign. Normally it replaces all undefined states in this enum as RAM can be unitialized */
    C_CHIP_STATE_CMD_RESET = C_CHIP_PROJECT_STATE_CMD_RESET,              /**< Warm reset sign */
    /* Bootloaders masks and signs */
    C_CHIP_STATE_CMD_MASK = 0x801Fu,                         /**< The mask of any "enter-bootloader" command. Enter-BL mode commands ara coded by 5 bits [4:0] */
    C_CHIP_STATE_BL_SIGN = 0x3060u,                /**< Enter-programming mode (EPM) sign. Checked by mask C_CHIP_STATE_CMD_MASK */
    C_CHIP_STATE_PPM_CMD_EPM = C_CHIP_PROJECT_STATE_PPM_CMD_EPM,            /**< PPM EPM sign */
    C_CHIP_STATE_LIN_CMD_EPM = C_CHIP_PROJECT_STATE_LIN_CMD_EPM,            /**< LIN EPM sign */
    C_CHIP_STATE_MEL_CMD_EPM = C_CHIP_PROJECT_STATE_MEL_CMD_EPM,            /**< Melibu EPM sign */
    C_CHIP_STATE_UDS_CMD_EPM = C_CHIP_PROJECT_STATE_UDS_CMD_EPM,            /**< UDS EPM sign */
    C_CHIP_STATE_FLASH_CMD_EPM = 0xB066u,          /**< FLASH BLs EPM sign. Treated as warm reset by just jumping to FLASH, but has the info for customer's bootloader. */
    /* Note: Values 0bx0110000011xxxxx are assigned for Enter Programming mode values */
} BistResetInfo_t;

/** bistHeader specifies the constant signature for the data in RAM. This sign is used to validate the data below */
extern volatile BistHeader_t bistHeader __attribute__((section(".bist_stat.bistHeader")));
/** bistResetInfo specifies the command for the bootloader to perform the Warm reset, Cold reset or Enter programming-mode */
extern volatile BistResetInfo_t bistResetInfo __attribute__((section(".bist_stat.bistResetInfo")));
/** bistError reserved for the application. */
extern volatile uint16_t bistError __attribute__((section(".bist_stat.bistError")));
/** bistErrorInfo reserved for the application. */
extern volatile uint16_t bistErrorInfo __attribute__((section(".bist_stat.bistErrorInfo")));

#endif
/// @}
