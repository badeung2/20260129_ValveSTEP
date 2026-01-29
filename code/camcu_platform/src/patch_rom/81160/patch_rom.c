/**
 * @file
 * @brief ROM patches for the MLX16 CPU
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
 * @ingroup patch_colin
 *
 * @details This file contains ROM patches for the MLX16 CPU.
 */

#include <stdint.h>
#include "io.h"
#include "lib_patch.h"

#include "patch_rom.h"

#if defined(MLX81160A01)
/** Patch 3 : address to patch */
#define MLX16_PATCH3_ADDR 0x4d92u
#endif

void patch_rom_Init(void)
{
#if defined(MLX81160A01)
    /* patch 3 : MLX81160-167 */
    IO_SET(MLX16, DBG_ADDRESS3, MLX16_PATCH3_ADDR);
    IO_SET(MLX16, DBG_DATA3, (uint16_t)&OPCODE_JMP_FP_PATCH_FLASH_3);
    IO_SET(MLX16, DBG_CONTROL3, PATCH_CTRL_ENABLE);
#endif /* MLX81160A01 */
}
