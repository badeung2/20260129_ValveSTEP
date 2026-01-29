/**
 * @file
 * @brief Interface with Melexis Slave LIN driver
 * @internal
 *
 * @copyright (C) 2005-2018 Melexis N.V.
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

#ifndef MLS_API_H_
#define MLS_API_H_

#include "mls_config.h"
#include "mls_macro.h"
#include "mls_types.h"
#include "mls_diag_transfer.h"
#include "mls_event_table.h"


/*-----------------------------------------------------------------------------
 * Versions
 */

/* MLX LIN API version */
#define MLX_LIN_API_VERSION_MAJOR   1
#define MLX_LIN_API_VERSION_MINOR   5
#define MLX_LIN_API_VERSION_PATCH   3
#define MLX_LIN_API_VERSION_WIP     0 /* 0: released; 1: work in process */


/*-----------------------------------------------------------------------------
 * Declaration of variables
 */

#if (ML_HAS_EXPOSED_MEMBER_SYMBOLS == 1)

/* LIN global variables */
extern ML_VAR_ATTR volatile uint8_t LinFrameDataBuffer[8];
extern ML_VAR_ATTR volatile uint8_t LINBaud;
extern ML_VAR_ATTR volatile uint8_t LINPresc;
extern ML_VAR_ATTR volatile uint8_t LinStatus;
extern ML_VAR_ATTR uint16_t LinMessage;
extern ML_VAR_ATTR volatile bool LinDiscardedFrameFlag;

#else /* ML_HAS_EXPOSED_MEMBER_SYMBOLS */

/* LIN global variables */
extern ML_VAR_ATTR ml_Data_t ml_Data;

#endif /* ML_HAS_EXPOSED_MEMBER_SYMBOLS */


/*-----------------------------------------------------------------------------
 * Exported function prototypes
 */

/* LIN API Commands ----------------------------------------------------------------    */
/* Software and Flash Loader version */
extern uint16_t ml_LinModuleVersion(void);

/* Bus Timing */
extern ml_Status_t ml_SetBaudRate(uint8_t caPresc, uint8_t caBaud);
extern ml_Status_t ml_SetAutoBaudRateMode(ml_AutoBaudrateMode_t mode);
extern ml_Status_t ml_SetFixedBaudRate (uint16_t baudRate, uint16_t mlx4Freq);
extern ml_Status_t ml_SetFastBaudRate (uint8_t fastBaudRate, uint16_t mlx4Freq);

/* Software and Hardware options */
extern ml_Status_t ml_SetSlewRate(ml_SlewRate_t slewRate);

#if ML_MLX4_VERSION >= 412
/** Set the sleep timeout due to bus inactivity
 *
 * sleep_to(sec) = 1/Fosc * 2^(pre+9) * (17 + cnt) * 256/inc
 * where:
 *      Fosc - value in the range of 250-300kHz
 *      pre, cnt, inc - are input parameters
 *
 * If the inc value is set to 0 the LIN SW never enter in sleep state due to bus inactivity.
 *
 * @param pre see formula, input range [0, 7]
 * @param cnt see formula, input range [0,15]
 * @param inc see formula, input range [0,15]
 * @return Status
 */
extern ml_Status_t ml_SetSleepTo(uint8_t pre, uint8_t cnt, uint8_t inc);

/** Configure the LIN Module software
 *
 * @param idStopBitLength 0 (default)/1/2/3 -> 1 / 1.5 / 2 / 2.5 stop bits
 * @param txStopBitLength 0 (default)/1     -> 1 / 2 stop bits
 * @param stateChangeSignal enabled (default)/disabled
 * @param sleepMode
 *        ML_LIGHTSLEEP - firmware wakeup detection with timeout
 *        ML_DEEPSLEEP  - hardware wakeup detection with timeout
 * @param linVersion
 *        ML_VER_DEFAULT - it's defined by the MLX4 constant tables
 *        ML_VER_1_3 - version LIN 1.3 selection (timeouts are affected)
 *        ML_VER_2_x - version LIN 2.x selection (timeouts are affected)
 * @return Status
 */
extern ml_Status_t ml_SetOptions (
    uint8_t idStopBitLength,
    uint8_t txStopBitLength,
    ml_StateChangeEvent_t stateChangeEvent,
    ml_SleepMode_t sleepMode,
    ml_LinVersion_t linVersion);
#else /* ML_MLX4_VERSION >= 412 */
/* Configure the LIN Module software
 *   idStopBitLength 0 (default)/1/2/3 -> 1 / 1.5 / 2 / 2.5 stop bits
 *   txStopBitLength 0 (default)/1     -> 1 / 2 stop bits
 *   stateChangeSignal enabled (default)/disabled
 *   sleepMode
 *        ML_LIGHTSLEEP - firmware wakeup detection with timeout
 *        ML_DEEPSLEEP  - hardware wakeup detection with timeout
 */
extern ml_Status_t ml_SetOptions (
    uint8_t idStopBitLength,
    uint8_t txStopBitLength,
    ml_StateChangeEvent_t stateChangeEvent,
    ml_SleepMode_t sleepMode );
#endif /* ML_MLX4_VERSION >= 412 */

/* Task Control and Task Status */
#if ML_HAS_LIN_EVENT_TABLE_IN_RAM == 0
extern void ml_InitLinEventTable (LINEventTable_t *resLINEventTable);
#endif /* ML_HAS_LIN_EVENT_TABLE_IN_RAM */

extern ml_Status_t ml_Init (ml_BaudrateMode_t baudRateMode,
                            uint16_t baudRate,
                            ml_LinVersion_t linVersion,
                            uint16_t mlx4Freq);
extern ml_Status_t ml_InitLinModule(void);
extern ml_Status_t ml_Connect(void);
extern ml_Status_t ml_Disconnect(void);
extern ml_Status_t ml_GotoSleep(void);
extern ml_Status_t ml_WakeUp(void);
extern uint16_t    ml_GetBaudRate(uint16_t mlx4Freq);

/* Specific functions for Fast LIN */
#if ML_HAS_FAST_LIN == 1
extern ml_Status_t ml_InitFast (uint8_t fastBaudRate, uint16_t mlx4Freq);
extern ml_Status_t ml_SwitchToFast(void);
#endif /* ML_HAS_FAST_LIN == 1 */

/*
 * This command retrieves the current state and updates status variable of the LIN Module.
 *
 * Preconditions:
 *  1. The ml_GetState shall be called atomically from the background main loop
 *     because it could create event racing between MLX4 (LIN Module) and MLX16 (see details
 *     in PLTF-733 on Jira)
 *
 * \param   bits_to_reset   LIN bus activity bits to be cleared
 *
 * Output (global):
 *      ML_DATA_LIN_STATUS   LIN bus activity status
 *
 * \return  ml_LinState_t     current LIN state
 */
extern ml_LinState_t ml_GetState(ml_BitClear_t bits_to_reset);

/* ML_DATA_LIN_STATUS
 * bit[0] - LIN bus activity
 * bit[1] - 0: buffer free; 1: buffer not free
 * bit[2] - reserved
 * bit[3] - event overflow occurred
 */
#define ML_LIN_BUS_ACTIVITY     (1U << 0)    /* Getting LIN bus activity bit */
#define ML_LIN_BUFFER_NOT_FREE  (1U << 1)
#define ML_LIN_CMD_OVERFLOW     (1U << 3)    /* Mask for getting LIN overflow event bit */

/* Get LIN internals */
extern uint8_t ml_GetProtectedID(void);
extern uint16_t ml_SetLinStatus(uint16_t registerA);
extern void ml_SetDFFlag(void);
extern volatile uint8_t *ml_GetLINFrameDataBuffer(void);

/* Message Management */
/*
 * Note: These functions shall be called in Disconnected state only
 */
extern ml_Status_t ml_AssignFrameToMessageID(ml_MessageID_t messageIndex, ml_FrameID_t frameID);
extern ml_Status_t ml_EnableMessage(ml_MessageID_t messageIndex);
extern ml_Status_t ml_DisableMessage(ml_MessageID_t messageIndex);

/* Data Transfer */
extern ml_Status_t ml_DiscardFrame(void);
extern ml_Status_t ml_ContFrame(ml_Bool_t blEnable);
extern ml_Status_t ml_ReleaseBuffer(void);
extern ml_Status_t ml_ReleaseBufferProg(ml_Bool_t blProgMode);
extern ml_Status_t ml_DataReady(ml_DataTransmittedEvent_t dataTransmittedEvent);

/* Auto-addressing */
extern ml_Status_t ml_AutoAddressingConfig(ml_AutoAddressingMode_t autoAddressingMode);

/* LIN Module (Mlx4) interrupt handler */
extern void ml_LinInterruptHandler (void);

/* LIN Module (Mlx4) polling functions */
extern void ml_GetLinEventData (void);     /* Copies LIN event data from the shared memory to the private structures */
extern void ml_ProcessLinEvent (void);     /* Processes LIN event and invokes mlu_ call-back functions */

/* LIN Module (Mlx4) control functions (these functions shall be implemented on the project / platform) side */
extern void ml_SetSLVIT(uint8_t slvit);
extern uint8_t ml_GetSLVIT(void);

extern void ml_SetSLVCMD(uint8_t slvcmd);
extern uint8_t ml_GetSLVCMD(void);

extern void ml_CleanDrvInt(void);

extern void ml_ResetDrv(void);
extern void ml_StartDrv(void);

extern void ml_EnableFastPhy(void);

/* Time delay in ms (it shall be implemented on the project / platform) side */
extern void ml_TimeDelayMs(uint16_t delay_ms);

#endif /* MLS_API_H_ */
