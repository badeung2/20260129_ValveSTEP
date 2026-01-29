/**
 * @file
 * @brief The common bootloader's interface.
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
 * @ingroup bootloaders
 * @addtogroup bootloaders Bootloader
 *
 * @brief Initialization and tick layer for bootloaders
 * @details
 * The PHY-layer wrapper used to manage the Melibus, fast-protocol LIN and the SPI physical interfaces.
 * All functions from the interfaces should be provided externally in case if they will be used.
 * @{
 */

#ifndef BL_MAIN_H
#define BL_MAIN_H

#include <stdbool.h>
#include <syslib.h>
#include "bl_tables.h"

/** Acknowledge watchdog and check the timeout.
 *
 * Check if Analog watchdog window is open and confirm the watchdog, so that cleanup can be performed in the main loop.
 *
 * @retval true Analog watchdog overflow. Analog watchdog counter is higher than BL_AWD_TIMEOUT.
 * @retval false Analog watchdog window is not opened or counter is still not higher than BL_AWD_TIMEOUT.
 */
extern bool bl_CheckWatchdog(void);
extern bl_table_t bl_table;

/** The default multiplexer state used to 'deinit' the interface */
#define DEFAULT_PIN 0u

#ifdef HAS_HW_PPM_5V_TOLERANCE
#define XCFG_ENABLE_LIN_REVPROT (((uint16_t)1u) << 11)
#define XCFG_SEL_RXD_ATDI       (((uint16_t)1u) << 10)
#endif /* HAS_HW_PPM_5V_TOLERANCE */

#ifdef HAS_HW_PPM_5V_TOLERANCE_WITH_INVERSION
#define XCFG_CXPI_DIS_WU        (((uint16_t)1u) << 13)
#define XCFG_ENABLE_LIN_REVPROT (((uint16_t)1u) << 12)
#define XCFG_SEL_RXD_ATDI       (((uint16_t)1u) << 11)
#define XCFG_LIN_INVERT_RX      (((uint16_t)1u) << 10)
#endif /* HAS_HW_PPM_5V_TOLERANCE_WITH_INVERSION */

/* PORT_LIN_XCFG bits description */
#define XCFG_DISTERM     (((uint16_t)1u) << 9)
#define XCFG_BYPASS      (((uint16_t)1u) << 8)
#define XCFG_HSM         (((uint16_t)1u) << 7)
#define XCFG_LSM         (((uint16_t)1u) << 6)
#define XCFG_EN          (((uint16_t)1u) << 5)
#define XCFG_XCFG_USE    (((uint16_t)1u) << 4)
#define XCFG_LIN_IN_SOFT (((uint16_t)1u) << 3)
#define XCFG_RXD         (((uint16_t)1u) << 2)
#define XCFG_INVERT_TX   (((uint16_t)1u) << 1)
#define XCFG_XTX         (((uint16_t)1u) << 0)

/**
 * Debug output for the LIN Loader
 */
#define BL_DEBUG_OUTPUT(info) DEBUG_OUTPUT(info)   /* use general project settings */

/** bl_getLoaderMode returns the bootloader's mode state according the ROM-supported bootloaders.
 *
 *  @retval blm_EmptyFlashMode if the KEY is wrong or bistResetInfo does not exact the bootloader's mask
 *  @retval blm_LinLoaderMode Lin Loader mode request detected
 *  @retval blm_PpmLoaderMode Ppm Loader mode request detected
 *  @retval blm_MelLoaderMode Melibu Loader mode request detected
 *  @retval blm_AppMode External (FLASH) bootloader mode request detected or the warm-startup
 */
bl_Mode_t bl_getLoaderMode(void);

#ifdef UNITTEST
#ifdef BL_PHY_USE_LIN
/** LIN bootloader initialization.
 *
 * Initialize LIN, register for response on LIN diagnostic frames and initialize LIN loader.
 */
void bl_RunLinMode(void);
#endif /* BL_PHY_USE_LIN */

#ifdef BL_PHY_USE_PPM
/** PPM bootloader initialization.
 *
 * Initialize ppm and set the appropriate state.
 */
void bl_RunPpmMode(void);

#if (PPM_EPM_ENABLED == 1)
/** Run FLASH bist and wait for 2.5ms for the ppm pattern.
 *
 * Confirm the PPM pattern on the pins to enable entry to PPM mode. If ppm pattern is accepted, then switch the
 * programming mode table to blm_PpmLoaderMode which will then run/initialize the ppmBootloader as if we restarted in
 * the ppm bootloader mode.
 *
 * @warning The function is blocking for flash bist and 2.5m timer (all in PPM_FLASH_BistTest)
 */
void bl_RunEmptyPpmMode(void);
#endif /* PPM_EPM_ENABLED */
#endif /* BL_PHY_USE_PPM */

#ifdef BL_PHY_USE_MELIBUS
/** MeliBUS bootloader initialization.
 *
 * Initialize Melibus and initialize Melibus bootloader.
 */
void bl_RunMelMode(void);
#endif /* BL_PHY_USE_MELIBUS */

#ifdef BL_PHY_USE_UDS_ON_LIN
/** UDS bootloader initialization.
 *
 * Initialize UDS bootloader
 *
 * @note Assumes LIN is already initialized in bl_init array
 */
void bl_RunUdsMode(void);
#endif /* BL_PHY_USE_UDS_ON_LIN */

/** Performs the Init
 *
 * Run the BL's *_Init() functions.
 *
 * @note Uses global variable bl_table.prog_mode as initialization functions have possibility to change bootloader mode
 * (for example: ppm entry mode pattern) while initialization. bl_init array should be organized accordingly.
 */
void bl_Init(void);

/** Performs the Idle loop
 *
 * Triggers the BL's *_Tick() function and checks the timeout.
 * Exits when the Timeout exceeds
 */
void bl_idleLoop(bl_Mode_t programming_mode);
#endif /* UNITTEST */

/// @}
#endif /* BL_MAIN_H */
