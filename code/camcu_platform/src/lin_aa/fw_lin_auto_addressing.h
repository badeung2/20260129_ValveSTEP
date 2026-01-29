/**
 * @file
 * @brief LIN auto addressing library
 * @internal
 *
 * @copyright (C) 2020-2021 Melexis N.V.
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
 * @addtogroup fw_lin_aa LIN auto addressing Module
 * @{
 * @brief   LIN auto addressing Module
 * @details Module for executing LIN auto addressing sequences.
 */

#ifndef FW_LIN_AUTO_ADDRESSING_H_
#define FW_LIN_AUTO_ADDRESSING_H_

#include <stdint.h>

/** Initialize the LIN auto addressing module */
void fw_linaa_Init(void);

/** LIN auto addressing timeout tick 
 *
 * @param  percentTO  percentage of the LIN AA timeout which has passed since last call.
 */
void fw_linaa_Tick(uint8_t percentTO);

/** Handle the LINAA frame timeout
 *
 * The application needs to implement the Timer0 Int3 interrupt handler and call this
 * methode when LINAA is ongoing.
 */
void fw_linaa_FrameTimoutHandler(void);

/** @} */
#endif /* FW_LIN_AUTO_ADDRESSING_H_ */
