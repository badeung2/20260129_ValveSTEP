/**
 * @file
 * @brief Melexis LIN slave initialization in ROM mode
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
 * @ingroup mls_init
 * @defgroup mls_init MLX LIN slave initialization
 * @{
 * @brief   MLX LIN slave initialization
 * @details Melexis LIN slave initialization in ROM mode.
 * @}
 *
 * @details This file contains MLX LIN slave initialization in ROM mode.
 *
 */

#ifndef MLS_INIT_H_
#define MLS_INIT_H_


/* ---------------------------------------------
 * Global Function Declarations
 * --------------------------------------------- */

/** Melexis LIN slave initialization in ROM mode. */
extern void mls_Init(void);

/** Polling function to process LIN event. */
extern void mls_Tick(void);


#endif /* MLS_INIT_H_ */

