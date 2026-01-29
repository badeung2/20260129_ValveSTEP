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
 * @addtogroup patch_rom MLX16 CPU ROM patches
 * @{
 * @brief   MLX16 CPU ROM patches
 * @details Module for loading correct patches for the MLX16 CPU.
 */

#ifndef PATCH_MLX16_H_
#define PATCH_MLX16_H_

/** Initialize the MLX16 ROM patch module
 *
 * This function will load the patches for the MLX16 CPU. Before calling this
 * function the MLX16 CPU needs to be stopped otherwise memory error will be
 * thrown.
 */
extern void patch_rom_Init(void);

/** @} */
#endif /* PATCH_MLX16_H_ */

