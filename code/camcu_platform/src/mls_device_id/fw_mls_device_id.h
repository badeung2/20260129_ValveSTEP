/**
 * @file
 * @brief Melexis specific LIN frames support in application mode.
 * @internal
 *
 * @copyright (C) 2018-2020 Melexis N.V.
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
 * @ingroup fw_mls_device_id
 *
 * @details Melexis specific LIN frames support used for device identification and entering programming mode in application mode.
 */

#ifndef FW_LIN_DEVICE_ID_H_
#define FW_LIN_DEVICE_ID_H_

#include <stdint.h>
#include "mls_api.h"
#include "mls_diag_transfer.h"


/* ---------------------------------------------
 * Global Function Declarations
 * --------------------------------------------- */

/** Enable Melexis specific LIN frames support */
extern void fw_lepm_Init(void);

#endif /* FW_LIN_DEVICE_ID_H_ */

