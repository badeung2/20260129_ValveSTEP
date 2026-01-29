/**
 * @file
 * @brief Describes the global variables and fields in FLASH and RAM to be used during the startup code and interrupts handling
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
 * @ingroup library
 *
 * @details
 * Describes the global variables and fields in FLASH and RAM to be used during the startup code and interrupts handling
 * @{
 */

#ifndef BL_DEFS_H
#define BL_DEFS_H

/*
 * User functions prototypes
 * These function shall be defined in user application. And they will be called from the function pointers' table
 */


#ifndef BL_PREFIX
#define BL_PREFIX                   /** Defines the prefix for each bootloader function */
#endif

#include <stdint.h>
#include "compiler_abstraction.h"
#include "sys_tools.h"

/** Possible values for ml_driver_mode
 *
 * @note: Values selection is driven by the goal to make their hamming distance as much as possible. These values should
 *     not hold values 0x55 and 0xAA as one of possible values left after the RAM BIST check.
 * */
typedef enum {
    blm_AppMode = 0x00U,        /**< Application mode sign. */
    blm_PpmLoaderMode = 0x58U,  /**< PPM mode sign. Should differ from 0x55 or 0xAA which can be left after the RAM BIST */
    blm_LinLoaderMode = 0x3BU,  /**< LIN Bootloader mode sign */
    blm_MelLoaderMode = 0xC4U,  /**< MEL Bootloader mode sign */
    blm_UartLoaderMode = 0x39U, /**< UART Bootloader mode sign */
    blm_SpiLoaderMode = 0x66U,  /**< SPI Bootloader mode sign */
    blm_UdsLoaderMode = 0xD5U,  /**< UDS Bootloader mode sign */
    blm_EmptyFlashMode = 0xA7U, /**< Empty FLASH mode sign - should be deprecated soon with blm_InvalidFlashMode */
    blm_InvalidFlashMode = blm_EmptyFlashMode  /**< Invalid FLASH mode sign */
} bl_Mode_t;

/** Simple function type */
typedef void (* main_t)(void);

#ifdef UNITTEST /* Intentionally defined in here for unit-test to be mocked externally from the component under the test */
STATIC INLINE void callFlash(void);
#endif /* UNITTEST */

void bl_start(void);

#endif
/// @}

