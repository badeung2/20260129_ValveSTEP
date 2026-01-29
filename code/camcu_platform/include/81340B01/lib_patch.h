/**
 * @file
 * @brief ROM patch support library
 * @internal
 *
 * @copyright (C) 2021 Melexis N.V.
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
 * @addtogroup patch Patching
 *
 * @brief ROM patch support library
 * @details
 * @{
 */

#ifndef PATCH_H
#define PATCH_H

#include <stdint.h>

/** Value to write in DBG_CONTROLx for enabling a particular patch */
#define PATCH_CTRL_ENABLE 0x0410u

/** Patch 0 : instruction to jump to fw patch 0 location */
extern const uint16_t OPCODE_JMP_FP_PATCH_FLASH_0;

/** Patch 1 : instruction to jump to fw patch 1 location */
extern const uint16_t OPCODE_JMP_FP_PATCH_FLASH_1;

/** Patch 2 : instruction to jump to fw patch 2 location */
extern const uint16_t OPCODE_JMP_FP_PATCH_FLASH_2;

/** Patch 3 : instruction to jump to fw patch 3 location */
extern const uint16_t OPCODE_JMP_FP_PATCH_FLASH_3;

#if defined (HAS_ROM_VECTOR_REMAP)
/** Patch 0 : instruction to jump to fw patch 0 remapped location */
extern const uint16_t OPCODE_JMP_FP_PATCH_FLASH_0_REMAP;

/** Patch 1 : instruction to jump to fw patch 1 remapped location */
extern const uint16_t OPCODE_JMP_FP_PATCH_FLASH_1_REMAP;

/** Patch 2 : instruction to jump to fw patch 2 remapped location */
extern const uint16_t OPCODE_JMP_FP_PATCH_FLASH_2_REMAP;

/** Patch 3 : instruction to jump to fw patch 3 remapped location */
extern const uint16_t OPCODE_JMP_FP_PATCH_FLASH_3_REMAP;
#endif

/** Copy the patch data into the IO-ports to enable the patches
 *
 * This routine uses the PATCH_COUNT patches from EEPROM and checks whether they are enabled or not.
 * If certain patch is enabled (LSB in address = 1), the routine configures the debug HW in patch mode and sets the patch
 */
void loadPatches(void);

#endif /* PATCH_H */
/// @}

