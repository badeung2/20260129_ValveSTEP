/**
 * @file
 * @brief Additional macro definitions and checks
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
 * @ingroup mlx_lin_slave_api
 *
 * @details Additional macro definitions for backward compatibility and checks
 */

#ifndef MLS_COMPAT_H_
#define MLS_COMPAT_H_

#ifdef UNITTEST
#include "mls_config.h"
#endif


/*-----------------------------------------------------------------------------
 * Macro            (scope: module-local)
 */

/* Insure backward compatibility */
#ifndef ML_HAS_EXPOSED_MEMBER_SYMBOLS
#define ML_HAS_EXPOSED_MEMBER_SYMBOLS 1
#endif

#if (ML_HAS_EXPOSED_MEMBER_SYMBOLS == 1)

/* Define access to exposed variables */
#define ML_DATA_LIN_FRAME_DATA_BUFFER LinFrameDataBuffer
#define ML_DATA_LIN_BAUD LINBaud
#define ML_DATA_LIN_PRESC LINPresc
#define ML_DATA_LIN_STATUS LinStatus
#define ML_DATA_LIN_MESSAGE LinMessage
#define ML_DATA_LIN_DISCARDED_FRAME_FLAG LinDiscardedFrameFlag

#else /* ML_HAS_EXPOSED_MEMBER_SYMBOLS */

/* Define access to exposed variables */
#define ML_DATA_LIN_FRAME_DATA_BUFFER ml_Data.LinFrameDataBuffer
#define ML_DATA_LIN_BAUD ml_Data.LINBaud
#define ML_DATA_LIN_PRESC ml_Data.LINPresc
#define ML_DATA_LIN_STATUS ml_Data.LinStatus
#define ML_DATA_LIN_MESSAGE ml_Data.LinMessage
#define ML_DATA_LIN_DISCARDED_FRAME_FLAG ml_Data.LinDiscardedFrameFlag

#endif /* ML_HAS_EXPOSED_MEMBER_SYMBOLS */

/* Insure macro availability (external dependencies to be provided via mls_config.h or via -D compiler flag) */
#ifndef __ASSEMBLER__
#if !defined(STATIC) || \
    !defined(NOP) || \
    !defined(ASSERT) || \
    !defined(LDT_DEBUG_OUTPUT)
#error "The configuration isn't sufficient; these options values shall be provided explicitly"
#endif
#endif

#if !defined(MLXCOMP_369_WORKAROUND) || \
    !defined(ML_MLX4_VERSION) || \
    !defined(ML_SECTION_MLX4_SHARED_RAM) || \
    !defined(ML_HAS_MLX4_SEND_CMD_RETRY) || \
    !defined(ML_HAS_MLX4_CMD_ACK_TIMEOUT) || \
    !defined(ML_MLX4_SLEEP_MODE) || \
    !defined(ML_HAS_MLX4_INIT_TIMEOUT) || \
    !defined(ML_VAR_ATTR) || \
    !defined(ML_MLX4_INIT_TIMEOUT_MS) || \
    !defined(ML_HAS_FAST_LIN) || \
    !defined(ML_HAS_LIN_AUTOADDRESSING) || \
    !defined(ML_HAS_LIN_EVENT_TABLE_IN_RAM) || \
    !defined(ML_HAS_EXPOSED_MEMBER_SYMBOLS) || \
    !defined(LDT_HAS_CALLABLE_STATIC) || \
    !defined(STATIC_CALLABLE) || \
    !defined(LDT_HAS_TRANSPORT_LAYER) || \
    !defined(LDT_MAX_DATA_IN_SEGMENTED_TRANSFER) || \
    !defined(LDT_HAS_N_AS_TIMEOUT) || \
    !defined(LDT_HAS_N_CR_TIMEOUT) || \
    !defined(LDT_HAS_CONSECUTIVE_FRAMES) || \
    !defined(LDT_DEFAULT_NAD) || \
    !defined(LDT_STRUCT_COMPAT_MODE)
#error "The configuration isn't sufficient; these options values shall be provided explicitly"
#endif

#endif /* MLS_COMPAT_H_ */
