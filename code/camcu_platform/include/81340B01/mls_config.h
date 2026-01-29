/**
 * @file mls_config.h
 * @brief MLX LIN API configurations
 * @internal
 *
 * @copyright (C) 2017-2018 Melexis N.V.
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
 * @details
 */

#ifndef LIN_CONFIG_H_
#define LIN_CONFIG_H_


#include <compiler_abstraction.h>

#ifndef __ASSEMBLER__
#include <mathlib.h>
#include <sys_tools.h>
#include <syslib.h>

/**
 * Debug output for the LIN TL.
 * Use general project settings.
 */
#define LDT_DEBUG_OUTPUT(info) DEBUG_OUTPUT(info)

#endif /* __ASSEMBLER__ */


/*
 * Melexis specific definitions (not part of the MLX LIN API options description)
 */

/** Workaround for MLXCOMP_369 issue */
#ifndef MLXCOMP_369_WORKAROUND
    #define MLXCOMP_369_WORKAROUND 0
    #if MLXCOMP_369_WORKAROUND == 1
        #define MLXCOMP_369_PADDING_CONCAT2_(a, b) a ## b
        #define MLXCOMP_369_PADDING_CONCAT1_(a, b) MLXCOMP_369_PADDING_CONCAT2_(a, b)
        #define MLXCOMP_369_PADDING   \
    uint8_t MLXCOMP_369_PADDING_CONCAT1_(paddingField, __LINE__)
        #define MLXCOMP_369_GAP 1
    #else /* MLXCOMP_369_WORKAROUND */
        #define MLXCOMP_369_PADDING
        #define MLXCOMP_369_GAP 0
    #endif /* MLXCOMP_369_WORKAROUND */
#endif /* MLXCOMP_369_WORKAROUND */


/*-----------------------------------------------------------------------------
 * Constants and macros                 (scope: module-local)
 */

/** The ML_HAS_* options values:
 * - 0: disabled
 * - 1: enabled
 */

/**
 * The MLX4 version:
 * - 400 (v4.0.0)
 * - 410 (v4.1.0)
 * - 411 (v4.1.1)
 * - 412 (v4.1.2)
 * - 413 (v4.1.3)
 * - 414 (v4.1.4)
 */
#ifndef ML_MLX4_VERSION
#if defined(MLX81340A01) || defined(MLX81340B01)
#define ML_MLX4_VERSION 413
#else
#define ML_MLX4_VERSION 414
#endif
#endif

/* Special section names */
#ifndef ML_SECTION_MLX4_SHARED_RAM
#define ML_SECTION_MLX4_SHARED_RAM .special
#endif

/**
 * Retry the communication with MLX4 one more time if it's failed
 */
#ifndef ML_HAS_MLX4_SEND_CMD_RETRY
#define ML_HAS_MLX4_SEND_CMD_RETRY  1
#endif

/**
 * Return the timeout error when MLX4 isn't answering instead of forever loop
 */
#ifndef ML_HAS_MLX4_CMD_ACK_TIMEOUT
#define ML_HAS_MLX4_CMD_ACK_TIMEOUT 1
#endif

/**
 * Define the number of cycles for the acknowledgement timeout
 */
#if (ML_HAS_MLX4_CMD_ACK_TIMEOUT == 1)
#define MLX4_CMD_ACK_TIMEOUT    1000
#endif /* ML_HAS_MLX4_CMD_ACK_TIMEOUT */

/** MLX4 sleep configurations
 * - 0: light sleep
 * - 1: deep sleep
 */
#ifndef ML_MLX4_SLEEP_MODE
#define ML_MLX4_SLEEP_MODE 0
#endif

/** MLX4 HW initialization timeout
 * - 0: disabled
 * - 1: enabled
 */
#ifndef ML_HAS_MLX4_INIT_TIMEOUT
#define ML_HAS_MLX4_INIT_TIMEOUT 0
#endif

/** Variables attributes without DP section */
#ifndef ML_VAR_ATTR
    #if defined(ML_HAS_NO_DP_VAR_ATTR)
        #define ML_VAR_ATTR    __attribute__((section(".lin_ram")))
    #else
        #define ML_VAR_ATTR    __attribute__((dp, section(".lin_ram")))
    #endif
#endif

/* TODO: issue #9: the ML_HAS_ONLY_FAST_LIN options is always enabled into the lin_consts.h */
#if 0
#ifndef ML_HAS_ONLY_FAST_LIN
#define ML_HAS_ONLY_FAST_LIN 0
#endif
#endif

/** The timeout [ms] during the MLX4 initialization (acatual only when the ML_HAS_MLX4_INIT_TIMEOUT is enabled) */
#ifndef ML_MLX4_INIT_TIMEOUT_MS
#define ML_MLX4_INIT_TIMEOUT_MS 20
#endif

/** Optimize the library size by removing Fast LIN functionality.
 * Criteria: in the case MLX LIN API has been moved to the ROM the Fast LIN functionality has
 * to be available for use by some functions added to Flash (the ROM functions aren't allowed
 * to be replaced for current optimization purposes) */
#ifndef ML_HAS_FAST_LIN
#define ML_HAS_FAST_LIN 1
#endif

/** Indicates LIN Autoaddressing support */
#ifndef ML_HAS_LIN_AUTOADDRESSING
#define ML_HAS_LIN_AUTOADDRESSING 1
#endif

/** Reserve RAM for the Event Table */
#ifndef ML_HAS_LIN_EVENT_TABLE_IN_RAM
#define ML_HAS_LIN_EVENT_TABLE_IN_RAM 0
#endif

/** Don't group LIN variables in the structure in order to expose all of them for linker.
 * 0 - variables are grouped into the structure
 * 1 - all variable are exposed */
#ifndef ML_HAS_EXPOSED_MEMBER_SYMBOLS
#define ML_HAS_EXPOSED_MEMBER_SYMBOLS  0
#endif


/*
 * LIN Transport layer (TL) definitions
 */

/** Indicates that call-able functions are required for static functions */
#ifndef LDT_HAS_CALLABLE_STATIC
#define LDT_HAS_CALLABLE_STATIC 0
#endif

/** It can be STATIC, but for some cases call-able functions are required,
 * e.g. optimal overwrite of LIN TL ROM functions from Flash */
#ifndef STATIC_CALLABLE
    #if LDT_HAS_CALLABLE_STATIC == 0
        #define STATIC_CALLABLE STATIC
    #else
        #define STATIC_CALLABLE extern
    #endif
#endif

/** Indicates the LIN TL support */
#ifndef LDT_HAS_TRANSPORT_LAYER
#define LDT_HAS_TRANSPORT_LAYER 1
#endif

/** LIN segmented transfer allows data size of 4094 (without reqSID).
 * In this implementation it is reduced to the allowed minimum:
 * 128Bytes(FlashPage) + 1byte(blockSequenceCounter);
 * 128Bytes(NVRAMPage) + 2byte(Read/WriteDataID).
 */
#ifndef LDT_MAX_DATA_IN_SEGMENTED_TRANSFER
#define LDT_MAX_DATA_IN_SEGMENTED_TRANSFER (128 + 2)
#endif

/** Indicates P2 response timeout presence */
#ifndef LDT_HAS_N_AS_TIMEOUT
#define LDT_HAS_N_AS_TIMEOUT 1
#endif

/** Indicates CR response timeout presence */
#ifndef LDT_HAS_N_CR_TIMEOUT
#define LDT_HAS_N_CR_TIMEOUT 1
#endif

/** Indicates Consecutive Frames support */
#ifndef LDT_HAS_CONSECUTIVE_FRAMES
#define LDT_HAS_CONSECUTIVE_FRAMES 1
#endif

/** Default NAD for the LIN TL */
#ifndef LDT_DEFAULT_NAD
#define LDT_DEFAULT_NAD 1u
#endif

/** LDT structure compatibility mode */
#ifndef LDT_STRUCT_COMPAT_MODE
#define LDT_STRUCT_COMPAT_MODE 0
#endif

/** Indicates that LIN TL data buffer length can be changed on run-time. */
#ifndef LDT_HAS_FLEX_BUFFER_LENGTH
#define LDT_HAS_FLEX_BUFFER_LENGTH 0
#endif

#endif /* LIN_CONFIG_H_ */
