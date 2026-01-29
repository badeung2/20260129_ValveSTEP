/**
 * @file
 * @brief Melexis LIN slave diagnostics in ROM mode
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
 * @ingroup mls_diag
 * @defgroup mls_diag MLX LIN slave diagnostics
 * @{
 * @brief   MLX LIN slave diagnostics
 * @details Melexis LIN slave diagnostics in ROM mode.
 * @}
 *
 * @details This file contains Melexis LIN slave diagnostics in ROM mode.
 */

#ifndef MLS_DIAG_H_
#define MLS_DIAG_H_

#include <stdint.h>


/* ---------------------------------------------
 * Global Function Declarations
 * --------------------------------------------- */

/** Enable Melexis LIN slave diagnostics */
extern void mls_diag_Init(void);

#endif /* MLS_DIAG_H_ */

