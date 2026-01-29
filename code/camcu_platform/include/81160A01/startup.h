/**
 * @file
 * @brief ROM startup routines
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
 * @ingroup startup
 * @addtogroup startup ROM Startup
 *
 * @brief ROM startup routines
 * @details
 * Startup common settings and definitions. Used to handle the low-level IC management and IC startup.
 * @{
 */

#ifndef STARTUP_H
#define STARTUP_H

#include "static_assert.h"
#include "compiler_abstraction.h"
#include "memory_map.h"
#include "mem_checks.h"
#include "bl_bist.h"
#include "bl_tools.h"

/* functions prefix which manages the library's functions memory mapping */
#ifndef STARTUP_PREFIX
#define STARTUP_PREFIX
#endif


/** Startup execution types */
typedef enum {
    smColdResetMode = 0u,        /**< Cold reset Mode - running the application after all memory checks */
    smWarmResetMode,            /**< Warm reset Mode - running the application with skipping all the memory checks */
    smBootloaderMode,           /**< Bootloader Mode - used to enter the communication mode and receive Bootloader's commands */
    smEmptyFlashMode,           /**< Empty Flash Mode Mode - triggers the routines to enter the programming mode with the special Enter-programming mode commands/sequences, conditions. */
} startMode_t;

void start (void);                      /* Function is called from the INT vectors table, Power-On-Reset vector for the ROM */

void rom_ram_section_init(void);
void start2(void);

/** Jump to the FLASH SW. Now it is in MEM_FLASH_START offset */
STATIC INLINE void callFlash(void);


#ifndef UNITTEST
#include "startup_inline_impl.h"
#endif /* UNITTEST */

#ifdef HAS_NO_SKIP_STARTUP_CHECKS
void HAL_BIST_RamNonDestructiveTestInt(ramBistConfiguration_t* ramBistConfiguration);
#endif /* HAS_NO_SKIP_STARTUP_CHECKS */


#endif
/// @}

