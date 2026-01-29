/* *************************************************************************** *
 *! \file       Triaxis_MLX90422_426
 *  \brief      MLX90422 or MLX90426 Triaxis (SENT) support
 *
 * \note        project MLX813xx
 *
 * \author      Marcel Braat
 *
 * \date        2023-04-05 (MMP230405-1)
 *
 * \version     2.0
 *
 *
 * MELEXIS Microelectronic Integrated Systems
 *
 * Copyright (C) 2023-2023 Melexis N.V.
 * The Software is being delivered 'AS IS' and Melexis, whether explicitly or
 * implicitly, makes no warranty as to its Use or performance.
 * The user accepts the Melexis Firmware License Agreement.
 *
 * Melexis confidential & proprietary
 *
 * *************************************************************************** */

#ifndef SENSE_LIB_TRIAXIS_MLX90422_426_H
#define SENSE_LIB_TRIAXIS_MLX90422_426_H

/*!*************************************************************************** */
/*                           INCLUDES                                          */
/* *************************************************************************** */
#include "AppBuild.h"                                                           /* Application Build */

#if (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE)

/*!*************************************************************************** */
/*                           DEFINITIONS                                       */
/* *************************************************************************** */
/* Triaxis Errors */
#define ERR_TRIAXIS_OK                  0x00U                                   /*!< Triaxis Error: OK */
#define ERR_TRIAXIS_CRC                 0x01U                                   /*!< Triaxis Error: CRC */
#define ERR_TRIAXIS_NODATA              0x02U                                   /*!< Triaxis Error: No Data */
#define ERR_TRIAXIS_INVCMD              0x03U                                   /*!< Triaxis Error: Invalid Command */
#define ERR_TRIAXIS_INVLEN              0x04U                                   /*!< Triaxis Error: Invalid Length */
#define ERR_TRIAXIS_UNKNOWN             0x05U                                   /*!< Triaxis Error: Unknown */
#define ERR_TRIAXIS_INVREPLY            0x06U                                   /*!< Triaxis Error: Invalid Reply */
#define ERR_TRIAXIS_MALFUNCTION         0x07U                                   /*!< Triaxis Error: Mall Function */
#define ERR_TRIAXIS_DIAGNOSTIC_INFO     0x08U                                   /*!< Triaxis Error: Diagnostics Info */

#define C_SENT_MSG_LEN                  8U                                      /* SENT/SPC Message length; depends on Frame-Format (max: 8) */
#define C_TICK                          3U                                      /* Tick-time: 3us */
#define C_TYP_SYNC_PERIOD_TICKS         56U                                     /* Sync period: 56T */
#define C_MIN_SYNC_PERIOD_CNT ((uint16_t)(((C_TYP_SYNC_PERIOD_TICKS * C_TICK) * (PLL_FREQ / 1000000UL)) * 0.97))
#define C_MAX_SYNC_PERIOD_CNT ((uint16_t)(((C_TYP_SYNC_PERIOD_TICKS * C_TICK) * (PLL_FREQ / 1000000UL)) * 1.03))
#define C_MIN_DATA_NIBBLE_TICKS         12U                                     /* Minimum Data Nibble: 12T */
#define C_MAX_DATA_NIBBLE_TICKS         27U                                     /* Maximum Data Nibble: 27T */

#define C_TRIAXIS_SENT_FRAME_FORMAT     3U                                      /*!< SENT Frame Format; See data-sheet for details
                                                                                 * 1: A.1: Throttle positions sensor (H.1)
                                                                                 * 3: A.3: Single Secure sensor format (H.4)
                                                                                 */
#define C_TRIAXIS_SENT_SLOW_CHANNEL     /*FALSE*/ TRUE                          /*!< SENT Slow Channel format:
                                                                                 * TRUE: Enhanced Slow Channel format (8-bit ID, 128-bit value)
                                                                                 */

/* #define C_SENT_NONE                  0x00U */
#define C_SENT_SYNC                     0x01U
#define C_SENT_MSG                      0x02U
#define C_SENT_ERROR_CRC                0x80U
#define C_SENT_ERROR_OTHER              0x81U

/* Triaxis BGN to END = 320 degrees */
#define C_TRIAXIS_0DEG        (uint16_t)(((0UL * 65536U) + 180U) / 360U)        /*!< Triaxis 0 degree offset */
#define C_TRIAXIS_APP_BGN     (uint16_t)(((0UL * 65536U) + 180U) / 360U)        /*!< Begin application at 20 degrees */
#define C_TRIAXIS_APP_END     (uint16_t)(((360UL * 65536U) + 180U) / 360U) - 1U  /*!< End application at 340 degrees */
#define C_TRIAXIS_APP_RNG               (C_TRIAXIS_APP_END - C_TRIAXIS_APP_BGN) /*!< Application angle-range */

/*!*************************************************************************** */
/*                           GLOBAL VARIABLES                                  */
/* *************************************************************************** */
#pragma space dp                                                                /* __TINY_SECTION__ */
#pragma space none                                                              /* __TINY_SECTION__ */

#pragma space nodp                                                              /* __NEAR_SECTION__ */
extern uint16_t g_u16ShaftAngle;                                                /*!< Shaft Angle; Triaxis angle corrected with Sense-magnet offset */
extern uint16_t g_u16TriaxisErrorCode;                                          /*!< Triaxis Error Code */
extern uint8_t g_u8TriaxisStatus;                                               /*!< Triaxis Status */
#pragma space none                                                              /* __NEAR_SECTION__ */

/*!*************************************************************************** */
/*                           GLOBAL FUNCTIONS                                  */
/* *************************************************************************** */
extern uint16_t Triaxis_Init(void);                                             /*!< Initialise the triaxis Sent frame interface */
extern void Triaxis_Start(void);                                                /*!< Start receiving Triaxis Sent frames */
extern void Triaxis_Stop(void);                                                 /*!< Stop receiving Triaxis Sent frames */
#if (C_TRIAXIS_SENT_SLOW_CHANNEL != FALSE)
extern uint16_t Triaxis_GetSlowChannelInfo(uint16_t u16SerialID);               /*!< Retrieve Enhanced Slow-channel information */
#endif /* (C_TRIAXIS_SENT_SLOW_CHANNEL != FALSE) */
extern uint16_t Triaxis_Data(void);                                             /*!< Get Triaxis Data (angle or error-code) */
extern uint16_t GetTriaxisAngle(void);                                          /*!< Get Triaxis Angle */
extern uint16_t GetShaftAngle(void);                                            /*!< Get Shaft Angle */
#if (LIN_COMM != FALSE)
extern void HandleTriaxisStatus(void);                                          /*!< Reply LIN response */
#endif /* (LIN_COMM != FALSE) */

#endif /* (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) */

#endif /* SENSE_LIB_TRIAXIS_MLX90422_426_H */

/* EOF */
