/**
 * @file
 * @brief MLX LIN API platform specifics
 * @internal
 *
 * @copyright (C) 2017-2020 Melexis N.V.
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
 * @ingroup mls_support
 *
 * @details MLX LIN API platform specific support functions.
 */

#ifndef MLS_SUPPORT_H_
#define MLS_SUPPORT_H_

#include <stdint.h>

#include "compiler_abstraction.h"
#include "io.h"
#include "mls_api.h"
#include "mls_config.h"
#include "memory_map.h"

/** Set the SLVIT LIN Module value
 *
 * @param[in]  slvit  value
 */
void ml_SetSLVIT(uint8_t slvit);

/** Get the SLVIT LIN Module value
 *
 * @return  SLVIT  value
 */
uint8_t ml_GetSLVIT(void);

/** Set the SLVCMD LIN Module value
 *
 * @param[in]  slvcmd  value
 */
void ml_SetSLVCMD(uint8_t slvcmd);

/** Get the SLVCMD LIN Module value
 *
 * @return  SLVCMD  value
 */
uint8_t ml_GetSLVCMD(void);

/** Clean the LIN Module interrupt */
void ml_CleanDrvInt(void);

/** Reset the LIN Module (reinitialize) */
void ml_ResetDrv(void);

/** Start the LIN Module */
void ml_StartDrv(void);

/** Initialized LIN PHY to enable Fast LIN */
void ml_EnableFastPhy(void);

#if ML_MLX4_VERSION >= 412
/* Calculate and set sleep TO.
 * The functions calculates the input arguments for ml_SetSleepTo() and calls it with them.
 * @param linVersion  version of the LIN Standard
 * @param baudRate    expected baudrate (isn't relevant for LIN 2.x)
 */
ml_Status_t ml_CalculateAndSetSleepTo(ml_LinVersion_t linVersion, uint16_t baudRate);
#endif

/** Set the MLX4 clock divider on top of MLX16 clock */
STATIC INLINE void mls_SetMlx4ClockDivider(void);

#if (ML_MLX4_VERSION == 412) || (ML_MLX4_VERSION == 413) || (ML_MLX4_VERSION == 414)
#define COLIN_LIN_STATUS_OFFSET 0x2F
#else
#error "COLIN LINstatus variable offset is unknown"
#endif

#ifndef COLIN_LIN_STATUS_ADDR
#define COLIN_LIN_STATUS_ADDR MEM_COLIN_RAM_START + COLIN_LIN_STATUS_OFFSET
#endif

/** COLIN LINstatus structure */
typedef struct COLIN_LINstatus_s {
    uint8_t bus_activity : 1;       /**< LIN bus activity detected */
    uint8_t buffer_used : 1;        /**< 0 : buffer free; 1 : buffer not free */
    uint8_t reserved : 1;           /**< reserved */
    uint8_t event_overflow : 1;     /**< event overflow occured */
} COLIN_LINstatus_t;                /**< COLIN LINstatus type */

/** COLIN LINstatus
 * This variable is located in COLIN private memory and as such, it can not be written!
 */
extern volatile const COLIN_LINstatus_t COLIN_LINstatus __attribute__((addr(COLIN_LIN_STATUS_ADDR)));

#if !defined (UNITTEST)
#include "mls_support_inline_impl.h"
#endif /* UNITTEST */

#endif /* MLS_SUPPORT_H_ */
