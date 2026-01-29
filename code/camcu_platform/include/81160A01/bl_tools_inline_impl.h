/**
 * @file
 * @brief Provides a set of functions used for the bootloaders and can be used in the Software Application in FLASH
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
 * @ingroup bl_tools
 * @addtogroup bl_tools
 * @ingroup library
 *
 * @brief Provides a set of functions used for the bootloaders and can be used in the Software Application in FLASH
 * @details
 * @{
 */

#ifndef BL_TOOLS_INLINE_IMPL_H
#define BL_TOOLS_INLINE_IMPL_H

#include <stdint.h>
#include "compiler_abstraction.h"
#include "bl_defs.h"
#include "bl_bist.h"
#include "io.h"
#include "sys_tools.h"
#include "memory.h"


#ifdef UNITTEST
#include "bl_tools.h"
#endif
#if defined(__COVERITY__) || defined(__POLYSPACE__)
#include <string.h>
#endif

STATIC INLINE NO_RETURN void MLX16_RESET_COLD(void)
{
    MLX16_RESET(C_CHIP_STATE_COLD_START);
}

STATIC INLINE NO_RETURN void MLX16_RESET_WARM(void)
{
    MLX16_RESET_SIGNED(C_CHIP_STATE_CMD_RESET);
}

STATIC INLINE NO_RETURN void MLX16_RESET_EPM(BistResetInfo_t reason)
{
    MLX16_RESET_SIGNED(reason);
}

STATIC INLINE void dataCopy16(const uint16_t* src, uint16_t* dest, uint8_t size)
{
    (void)memcpy_word((void*)dest, (const void*)src, (size_t)size * 2u);
}

STATIC INLINE bool within_including_u32(uint32_t arg, uint32_t min, uint32_t max)
{
    return ((arg <= max) && (arg >= min));
}

STATIC INLINE bool within_including_u16(uint16_t arg, uint16_t min, uint16_t max)
{
    return ((arg <= max) && (arg >= min));
}

STATIC INLINE bool within_including_u8(uint8_t arg, uint8_t min, uint8_t max)
{
    return ((arg <= max) && (arg >= min));
}

#endif /* BL_TOOLS_INLINE_IMPL_H */
/// @}
