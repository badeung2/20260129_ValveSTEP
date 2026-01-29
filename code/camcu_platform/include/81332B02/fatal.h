/**
 * @file
 * @brief default fatal routine
 * @internal
 *
 * @copyright (C) 2020 Melexis N.V.
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
 * @brief default fatal routine
 * @details
 * @{
 */

#ifndef FATAL_H
#define FATAL_H

#include "compiler_abstraction.h"

#ifndef UNITTEST
/** erases the bistHeader and stays forever in a loop
 *
 * sets the bistHeader=C_CHIP_HEADER_INVALID which indicates that program was terminated
 * unexpectedly and triggers a cold-reset.
 *
 * @note interrupts are still handled in the never-ending loop.
 */
void _fatal (void) NO_RETURN;
#else
/** erases the bistHeader and stays forever in a loop
 *
 * sets the bistHeader=C_CHIP_HEADER_INVALID which indicates that program was terminated
 * unexpectedly and triggers a cold-reset.
 *
 * @note interrupts are still handled in the never-ending loop.
 */
void _fatal (void);
#endif

#endif
/// @}

