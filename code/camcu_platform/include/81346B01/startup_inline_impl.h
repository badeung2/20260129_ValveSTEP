/**
 * @file
 * @brief ROM startup routines
 * @internal
 *
 * @copyright (C) 2015-2020 Melexis N.V.
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
 * @ingroup startup
 *
 * @brief ROM startup routines
 * @details
 * Startup common settings and definitions. Used to handle the low-level IC management and IC startup.
 * @{
 */

#ifndef STARTUP_INLINE_IMPL_H
#define STARTUP_INLINE_IMPL_H

#if defined (HAS_ROM_VECTOR_REMAP)
#include "bist.h"
#endif

/** FW startup function type */
typedef
    void (*fw_startup_t)(void) __attribute__ ((__noreturn__));

/** Call FLASH FW. No matter call or jump. FW will assign its stack itself. */
STATIC INLINE void callFlash(void)
{
#if defined (HAS_ROM_VECTOR_REMAP)
    if (bist_IsRomVectorRemapEnabled()) {
        /* enable remapped ROM interrupt vectors */
        IO_SET(MLX16, CPU_FP0ADR, ((uint16_t)MEM_ROM_VECTORS_REMAP_START >> 8));

#if defined (__MLX16_FX__)
        ((fw_startup_t)(MEM_FLASH_VECTORS_REMAP_START >> 1))();
#else
        ((fw_startup_t)(MEM_FLASH_VECTORS_REMAP_START))();
#endif /* __MLX16_FX__ */
    }
#endif

#if defined (__MLX16_FX__)
    ((fw_startup_t)(MEM_FLASH_VECTORS_START >> 1))();
#else
    ((fw_startup_t)(MEM_FLASH_VECTORS_START))();
#endif /* __MLX16_FX__ */
}

#endif /* STARTUP_INLINE_IMPL_H */
/// @}
