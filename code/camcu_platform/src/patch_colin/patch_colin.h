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
 * @addtogroup patch_colin COLIN CPU ROM patches
 * @{
 * @brief   COLIN CPU ROM patches
 * @details Module for loading correct patches for the COLIN CPU.
 */

#ifndef PATCH_COLIN_H_
#define PATCH_COLIN_H_

/** Initialize the COLIN ROM patch module
 *
 * This function will load the patches for the COLIN CPU. Before calling this
 * function the COLIN CPU needs to be stopped otherwise memory error will be
 * thrown.
 */
extern void patch_colin_Init(void);

/** @} */
#endif /* PATCH_COLIN_H_ */

