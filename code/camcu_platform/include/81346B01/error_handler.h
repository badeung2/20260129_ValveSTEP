/**
 * @file
 * @brief Public API for the error-handler module
 * @internal
 *
 * @copyright (C) 2019 Melexis N.V.
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
 * @ingroup error_handler
 * @addtogroup error_handler Error Handler
 *
 * @brief Public API for the error-handler module
 * @details A library to provide the several type of errors definition during the code execution
 * @{
 */


#ifndef ERROR_HANDLER_H
#define ERROR_HANDLER_H

#include <stdint.h>
#include <stdbool.h>
#include "compiler_abstraction.h"
#include "error_handler_ids.h"
#include "error_handler_types.h"

/** Initializes the bistError and bistErrorInfo for a new cold boot */
STATIC INLINE void ErrH_InitializeBistError(void);

#ifdef HAS_EE_WR_AREA_CTRL
/** Specifies whether the ROM should use "safe EPM feature" during the startup, even if FLASH is considered as OK.
 *
 */
extern bool fFlagRunRom;
#endif /* HAS_EE_WR_AREA_CTRL */

#ifndef UNITTEST
/** Use non-informed approach to call the _fatal()
 *
 * @param[in]  information  is not used, left for compatibility
 */
STATIC INLINE void ErrH_Fatal(ErrH_Identifier information) NO_RETURN;
#else
/** Use non-informed approach to call the _fatal()
 *
 * @param[in]  information  is not used, left for compatibility
 */
STATIC INLINE void ErrH_Fatal(ErrH_Identifier information);
#endif

/** Trigger an immediate reset after storing failure id
 * During the reset the Y-register represents the current stack-pointer value used.
 *
 * @param[in]  information  holds the reason of failure to indicate
 */
STATIC INLINE void ErrH_ResetInformed (ErrH_Identifier information);

/** Jumps to the _fatal() after storing failure id
 *
 * Sets the BistErrorInfo to desired value and stores the PC. The function's call is not a call but jump to _fatal()
 * During the reset the Y-register represents the current stack-pointer value used.
 *
 * @param[in]  information  holds the reason of failure to indicate
 *
 * @note: assumes the RAM is still accessible to write to.
 */
STATIC INLINE void ErrH_FatalInformed (ErrH_Identifier information);

/** Skips the failure detected. Does nothing
 *
 * @param[in]  information  is not used, left for compatibility
 */
STATIC INLINE void ErrH_Nop(ErrH_Identifier information);

/** Stores the failure id
 *
 * @param[in]  information  is not used, left for compatibility
 */
STATIC INLINE void ErrH_NopInformed(ErrH_Identifier information);
/** In-place project-specific function which provides the proper failure handling, according to the project configuration
 *
 * @param[in]  information  specifies the Failure identifier
 */
STATIC INLINE void ErrH_Handler(ErrH_Identifier information);

#ifdef HAS_EE_WR_AREA_CTRL
/** Forces the startup code to stay in ROM's (empty-FLASH) mode, considering several cases that may occur
 *
 * @param[in]  information  specifies the Failure identifier
 */
STATIC INLINE void ErrH_TriggerRomModeInformed(ErrH_Identifier information);
#endif /* HAS_EE_WR_AREA_CTRL */

#ifndef UNITTEST
#include "error_handler_inline_impl.h"
#include "hw_diags_cfg.h"
#endif /* UNITTEST */

#endif /* ERROR_HANDLER_H */
/// @}

