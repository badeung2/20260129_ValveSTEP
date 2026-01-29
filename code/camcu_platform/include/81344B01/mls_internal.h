/**
 * @file mls_internal.h
 * @brief Shared RAM description
 * @internal
 *
 * @copyright (C) 2016-2018 Melexis N.V.
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
 * @ingroup mlx_lin_slave_api
 *
 * @details Shared RAM description between MLX16 CPU and LIN Slave module.
 */

#ifndef MLS_INTERNAL_H_
#define MLS_INTERNAL_H_

#include <stdint.h>


/*-----------------------------------------------------------------------------
 * Declaration of variables             (scope: module-local)
 */

/*
 * Shared RAM interface
 */
extern volatile uint16_t LinResp;
extern volatile uint16_t LinCmnd;
extern volatile uint16_t LinMess;
extern volatile uint8_t LinMess2;
extern volatile uint8_t LinProtectedID;
extern volatile uint16_t LinFrame[4];

#endif /* MLS_INTERNAL_H_ */
