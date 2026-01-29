/**
 * @file
 * @brief Patch data for 81160AA
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
 * @ingroup module_name
 *
 * Patches description:
 * v.1.0 :
 * ROM function ml_ErrorEvent not working correctly
 * https://jira.melexis.com/jira/browse/MLX81160-167
 *
 * @details
 */

#include <memory_map.h>

.section .fw_patches,"xa",@progbits

.org (MEM_FLASH_PATCH_SIZE * 3)

    global  __patch_mlx81160_167
__patch_mlx81160_167:
    lsr X, #2
    lsr X, #2
    cmp X, #5
    jmpf 0x4d94/2

/* EOF */
