/**
 * @file
 * @brief ROM patches for the COLIN CPU
 * @internal
 *
 * @copyright (C) 2020-2021 Melexis N.V.
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
 * @ingroup patch_colin
 *
 * @details This file contains ROM patches for the COLIN CPU.
 */

#include <stdint.h>
#include <string.h>
#include "mls_config.h"

#include "patch_colin.h"

/** COLIN patch area start address */
extern uint16_t _mlx4_patch_area_start;

#if (ML_MLX4_VERSION == 413)
/** COLIN patch data */
STATIC const uint16_t colin_patch[] = {
    0xB34E,     /* Patch0: PLTF-910: FAST protocol patch on address 0x0E1 */
    0xA784,
    0x1101,
    0x20DA,
    0xBF88,
    0x20D8      /* Patch0: ----*/
};

/** Patch0 address: FAST protocol patch on address 0x0E1 */
#define COLIN_PATCH0_ADDR 0x0E1u

/** Patch0 instruction: jump to offset 0x1F88 (0x1F88/2 + instr:0x2000) or mlx4_patch_area_start */
#define COLIN_PATCH0_INSTR 0x2FC4u
#endif

void patch_colin_Init(void)
{
#if (ML_MLX4_VERSION == 413)
    memcpy((uint16_t*)&_mlx4_patch_area_start, colin_patch, sizeof(colin_patch));
    IO_SET(COLIN, ADDR_PATCH0, COLIN_PATCH0_ADDR);
    IO_SET(COLIN, DATA_PATCH0, COLIN_PATCH0_INSTR);
    IO_SET(COLIN, CTRL_PATCH0, 0x1u);
#endif /* (ML_MLX4_VERSION == 413) */
}
