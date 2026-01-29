/**
 * @file
 * @brief The flash firmware startup definitions.
 * @internal
 *
 * @copyright (C) 2018-2019 Melexis N.V.
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
 * @ingroup fw_user_startup
 *
 * @details This file contains the definitions of the flash firmware startup module.
 */

#ifndef FW_STARTUP_H_
#define FW_STARTUP_H_


/* ---------------------------------------------
 * Global Function Declarations
 * --------------------------------------------- */

/** User FW entry point.
 * The function defines stack, executes low level initialization of the hardware,
 * C-runtime initialization and executes main function of the application.
 */
extern void fw_start(void);

/** RAM initialization.
 * Initializes .bss and .data RAM sections.
 */
extern void fw_ram_section_init(void);

/** Function premain() is invoked by start-up code just before calling main().
 * At this point all runtime-initialization is done thus variables/objects
 * get their initial values.
 */
extern void fw_premain(void);

/** Application specific premain routines.
 * Initiates clocks, wait_states to it's optimal states for the application.
 * @note Default implementation can be overwritten by application implementation.
 */
extern void fw_low_level_init(void);

#endif /* FW_STARTUP_H_ */

