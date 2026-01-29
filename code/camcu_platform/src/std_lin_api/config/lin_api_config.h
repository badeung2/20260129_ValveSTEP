/**
 * @file lin_api_config.h
 * @brief Platform / project dependent configurations for STD LIN API
 * @internal
 *
 * @copyright (C) 2018 Melexis N.V.
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
 * @ingroup lin_api
 *
 * @details The file describes the STD LIN API dependencies from other libraries, platform, project.
 * @note The file can be replaced by application specific from application folder.
 */

#ifndef LIN_API_CONFIG_H
#define LIN_API_CONFIG_H

#include <stdint.h>
#include "static_assert.h"
#include "builtin_mlx16.h"
#include "sys_tools.h"


/*
 * General definitions
 */

/** \cond */
#ifndef ATTR_PACKED
#define ATTR_PACKED __attribute__((packed))
#endif
/** \endcond */


/*
 * Definitions related to Melexis LIN Slave API
 */

#include "mls_api.h"

/** Enable the default l_sys_irq_disable() and l_sys_irq_restore() functions implementation.
 * Recommended value is zero to increase the application awareness about STD LIN API interrupt control */
#ifndef LIN_HAS_DEFAULT_IRQ_CALLOUTS
#define LIN_HAS_DEFAULT_IRQ_CALLOUTS 1
#endif

/** Shared LIN API resources blocking CPU priority level. The highest priority
 * is set (all non-system interrupts are disabled during accessing the LIN API data) */
#ifndef LINAPI_RESOURCES_BLOCKING_PRIORITY
#define LINAPI_RESOURCES_BLOCKING_PRIORITY 0
#endif

/* The LinFrameDataBuffer shall be word-aligned to support both byte/word access. */
#define ML_SLAVE_FRAME_DATA_BUFFER (ML_DATA_LIN_FRAME_DATA_BUFFER)
#define ML_SLAVE_BUS_ACTIVITY_STATUS (ML_DATA_LIN_STATUS)

/* Adapter for First Set Bit function */
#define STD_LIN_FSB(v) builtin_mlx16_fsb(v)

/** Compiler bug workaround */
#define MLXCOMP_354_WA volatile


#endif /* LIN_API_CONFIG_H */

/* EOF */
