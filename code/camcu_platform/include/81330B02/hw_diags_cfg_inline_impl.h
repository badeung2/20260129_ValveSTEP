/**
 * @file
 * @brief Error hanlders definition file. Project-specific configuration
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
 * @ingroup hw_diag
 *
 * @details
 * Automatic generation by MlxCCT 1.13.0
 * Template version 1.0.0
 *
 */

#ifndef HW_DIAGS_CFG_INLINE_IMPL_H
#define HW_DIAGS_CFG_INLINE_IMPL_H

#include "error_handler_ids.h"
#include "error_handler_types.h"

STATIC INLINE void ErrH_Handler(ErrH_Identifier information)
{
    switch (information) {
        case FAIL_CPU_COPROCESSOR:
            ErrH_ResetInformed(information);
            break;

        case FAIL_EEPROM_REG0_BIST:
            ErrH_Nop(information);
            break;

        case FAIL_EEPROM_REG1_BIST:
            ErrH_Fatal(information);
            break;

        case FAIL_EEPROM_REG2_BIST:
            ErrH_Fatal(information);
            break;

        case FAIL_EEPROM_REG3_BIST:
            ErrH_Fatal(information);
            break;

        case FAIL_FLASH_REG0_BIST:
            ErrH_Fatal(information);
            break;

        case FAIL_FLASH_REG1_BIST:
            ErrH_Nop(information);
            break;

        case FAIL_FLASH_REG2_BIST:
            ErrH_Nop(information);
            break;

        case FAIL_FLASH_REG3_BIST:
            ErrH_Nop(information);
            break;

        case FAIL_RAM_REG0_BIST:
            ErrH_Fatal(information);
            break;

        case FAIL_RAM_REG1_BIST:
            ErrH_Nop(information);
            break;

        case FAIL_RAM_REG2_BIST:
            ErrH_Nop(information);
            break;

        case FAIL_RAM_REG3_BIST:
            ErrH_Nop(information);
            break;

        case FAIL_ROM_REG0_BIST:
            ErrH_Fatal(information);
            break;

        case FAIL_ROM_REG1_BIST:
            ErrH_Nop(information);
            break;

        case FAIL_ROM_REG2_BIST:
            ErrH_Nop(information);
            break;

        case FAIL_ROM_REG3_BIST:
            ErrH_Nop(information);
            break;

        case FAIL_UNKNOWN_REASON:
            ErrH_Fatal(information);
            break;

        default:
            ErrH_Nop(information);
            break;
    }
}

#endif /* HW_DIAGS_CFG_INLINE_IMPL_H */

/* EOF */
