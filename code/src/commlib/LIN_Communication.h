/*!*************************************************************************** *
 * \file        LIN_Communication.h
 * \brief       MLX813xx LIN communication handling
 *
 * \note        project MLX813xx
 *
 * \author      Marcel Braat
 *
 * \date        2017-08-15
 *
 * \version     2.0
 *
 *
 * \copyright   MELEXIS Microelectronic Integrated Systems
 *
 * Copyright (C) 2017-2022 Melexis N.V.
 * The Software is being delivered 'AS IS' and Melexis, whether explicitly or
 * implicitly, makes no warranty as to its Use or performance.
 * The user accepts the Melexis Firmware License Agreement.
 *
 * Melexis confidential & proprietary
 *
 * *************************************************************************** */

#ifndef LIN_COMMUNICATION_H
#define LIN_COMMUNICATION_H

/*!*************************************************************************** */
/*                           INCLUDES                                          */
/* *************************************************************************** */
#include "AppBuild.h"                                                           /* Application Build */

#if (LIN_COMM != FALSE)

#if ((LINPROT & LINX) == LIN2) || (LINPROT == LIN13_HVACTB)
#include "commlib/LIN_Diagnostics.h"                                            /* LIN Diagnostics support */
#endif /* ((LINPROT & LINX) == LIN2) || (LINPROT == LIN13_HVACTB) */
#if (LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52)
#include "commlib/LIN_2x_HVAC.h"                                                /* LIN HVAC support */
#elif (LINPROT == LIN2X_AIRVENT12)
#include "commlib/LIN_2x_AirVent.h"                                             /* LIN AirVent support */
#elif (LINPROT == LIN22_SIMPLE_PCT)
#include "commlib/LIN_2x_SIMPLE_PCT.h"                                          /* LIN Simple percentage support */
#elif (LINPROT == LIN2X_AGS)
#include "commlib/LIN_2x_AGS.h"                                                 /* LIN AGS support */
#elif (LINPROT == LIN2X_FAN01)
#include "commlib/LIN_2x_FAN.h"                                                 /* LIN FAN and pump support */
#elif (LINPROT == LIN2X_RELAY)
#include "commlib/LIN_2x_Relay.h"                                               /* LIN Relay support */
#elif (LINPROT == LIN2X_SOLENOID)
#include "commlib/LIN_2x_Solenoid.h"                                            /* LIN Solenoid support */
#elif (LINPROT == LIN13_HVACTB)
#include "commlib/LIN_13_HVAC_TB.h"                                             /* LIN 1.3 HVAC support */
#else
#error "Error: LIN-AppLayer include file missing."
#endif /* (LINPROT == LIN22_SIMPLE_PCT) */
#if (_SUPPORT_LIN_AA != FALSE)
#include "commlib/LIN_AutoAddressing.h"                                         /* LIN Auto-Addressing support */
#endif /* (_SUPPORT_LIN_AA != FALSE) */

#include <fwversion.h>
#include <lib_miscio.h>
#include <mls_api.h>                                                            /* Melexis LIN module (MMP180430-1) */
#if defined (__MLX81350__)
/* New/updated platform */
#include <mls_internal.h>                                                       /* ml_GetLinEventData/ml_ProcessLinEvent & LinProtectedID & LinFrame[] */
#endif /* defined (__MLX81350__) */
#include <mls_support.h>                                                        /* COLIN_LINstatus */


#if defined(ML_HAS_NO_DP_VAR_ATTR) && (ML_HAS_NO_DP_VAR_ATTR == 1)
extern ml_Data_t ml_Data __attribute__ ((dp, section(".lin_ram")));
#else  /* defined(ML_HAS_NO_DP_VAR_ATTR) && (ML_HAS_NO_DP_VAR_ATTR == 1) */
extern ml_Data_t ml_Data __attribute__ ((section(".lin_ram")));
#endif /* defined(ML_HAS_NO_DP_VAR_ATTR) && (ML_HAS_NO_DP_VAR_ATTR == 1) */

/*!*************************************************************************** */
/*                           DEFINITIONS                                       */
/* *************************************************************************** */
/* LIN Communication */
#define LIN_BR                  19200U                                          /*!< 19200 LIN Baudrate [kBaud] */
#if ((((PLL_FREQ / (1 << (1 + 1))) + (LIN_BR >> 1)) / LIN_BR) < 200)
#define LIN_BR_PRESCALER        1                                               /*!< LIN Baudrate pre-scaler */
#elif ((((PLL_FREQ / (1 << (1 + 2))) + (LIN_BR >> 1)) / LIN_BR) < 200)
#define LIN_BR_PRESCALER        2                                               /*!< LIN Baudrate pre-scaler */
#elif ((((PLL_FREQ / (1 << (1 + 3))) + (LIN_BR >> 1)) / LIN_BR) < 200)
#define LIN_BR_PRESCALER        3                                               /*!< LIN Baudrate pre-scaler */
#elif ((((PLL_FREQ / (1 << (1 + 4))) + (LIN_BR >> 1)) / LIN_BR) < 200)
#define LIN_BR_PRESCALER        4                                               /*!< LIN Baudrate pre-scaler */
#else
#error "ERROR: LIN Pre-scaler not set"
#endif
#define LIN_BR_DIV              ((((1000UL * MLX4_FPLL) / (1U << (1U + LIN_BR_PRESCALER))) + (LIN_BR >> 1)) / LIN_BR)   /*!< FPLL is given by command-line as N x 250kHz, eg 80 x 250kHz for 20MHz */
#if (LIN_BR_DIV < 99) || (LIN_BR_DIV > 200)
#error "ERROR: Wrong LinBaudrate pre-scaler; Please adapt pre-scaler value so the LinBaudrate is between 99 and 200."
#endif /* (LinBaudrate < 99) || (LinBaudrate > 200) */
#define C_DEF_DEVICE_ID         0x3FU                                           /*!< Default Device ID */

#define C_LIN_IN_FREE           0x00U                                           /*!< LIN input frame-buffer is free */
#define C_LIN_IN_FULL           0x01U                                           /*!< LIN input frame-buffer is full; It's not allowed to be overwritten (could be processed) */
#define C_LIN_IN_POSTPONE       0x02U                                           /*!< LIN input frame-buffer is full, but allowed to be overwritten by new LIN message */

/*! LININBUF union */
typedef union _LININBUF
{
#if ((LINPROT & LINX) == LIN2) || (LINPROT == LIN13_HVACTB)
    DFR_DIAG Diag;                                                              /**< LIN Diagnostics Demand Frame */
#if (LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52)
    HVAC_CTRL Ctrl;                                                             /**< Application Control Frame (HVAC) */
#elif (LINPROT == LIN2X_AIRVENT12)
    AIRVENT_CTRL Ctrl;                                                          /**< Application Control Frame (AirVent) */
#elif (LINPROT == LIN2X_AGS)
    ACT_CTRL Ctrl;                                                              /**< Application Control Frame (AGS) */
#elif (LINPROT == LIN22_SIMPLE_PCT)
    CONTROL_FRM Ctrl;                                                           /**< Application Control Frame (Simple Percentage) */
#if (_SUPPORT_SPECIAL_COMM_FIELD != FALSE)
    CONTROL_SPECIAL CtrlSpecial;                                                /**< Application special Control Frame (Simple Percentage, with extra data-fields) */
#endif /* (_SUPPORT_SPECIAL_COMM_FIELD != FALSE) */
#elif (LINPROT == LIN2X_FAN01)
    FAN_CTRL Ctrl;                                                              /**< Application Control Frame (FAN) */
#elif (LINPROT == LIN2X_RELAY)
    RELAY_CTRL Ctrl;                                                            /**< Application Control Frame (Relay) */
#elif (LINPROT == LIN2X_SOLENOID)
    SOLENOID_CTRL Ctrl;                                                         /**< Application Control Frame (Solenoid) */
#elif (LINPROT == LIN13_HVACTB)
    HVAC_CTRL Ctrl;                                                             /**< Application Control Frame (HVAC) */
#else
#error "Error: LIN Control Message structure missing."
#endif /* (LINPROT) */
#endif /* ((LINPROT & LINX) == LIN2) */
#if (_SUPPORT_MLX_CHIP_STATUS != FALSE)
    DFR_CHIP_CONTROL ChipCtrl;                                                  /**< Chip Control frame (DPI) */
#endif /* (_SUPPORT_MLX_CHIP_STATUS != FALSE) */
} LININBUF;

/*! LINOUTBUF union */
typedef union _LINOUTBUF
{
#if ((LINPROT & LINX) == LIN2) || (LINPROT == LIN13_HVACTB)
    RFR_DIAG DiagResponse;                                                      /**< LIN Diagnostics Response Frame */
#if (LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52)
    HVAC_STATUS Status;                                                         /**< Application status Frame (HVAC) */
#elif (LINPROT == LIN2X_AIRVENT12)
    AIRVENT_STATUS Status;                                                      /**< Application status Frame (AirVent) */
    AIRVENT_STATUS2 Status2;                                                    /**< Application status2 Frame (AirVent) */
#elif (LINPROT == LIN2X_AGS)
    ACT_STATUS Status;                                                          /**< Application status Frame (AGS) */
#elif (LINPROT == LIN22_SIMPLE_PCT)
    STATUS_FRM Status;                                                          /**< Application status Frame (Simple Percentage) */
#elif (LINPROT == LIN2X_FAN01)
    FAN_STATUS Status;                                                          /**< Application status Frame (FAN) */
#elif (LINPROT == LIN2X_RELAY)
    RELAY_STATUS Status;                                                        /**< Application status Frame (Relay) */
#elif (LINPROT == LIN2X_SOLENOID)
    SOLENOID_STATUS Status;                                                     /**< Application status Frame (Solenoid) */
#elif (LINPROT == LIN13_HVACTB)
    HVAC_STATUS Status;                                                         /**< Application Status Frame (HVAC) */
#else
#error "Error: LIN Status Message structure missing."
#endif /* (LINPROT) */
#endif /* ((LINPROT & LINX) == LIN2) */
#if (_SUPPORT_MLX_CHIP_STATUS != FALSE)
    RFR_CHIP_STATUS ChipStatus;                                                 /**< Chip Status frame (DPI) */
#endif /* (_SUPPORT_MLX_CHIP_STATUS != FALSE) */
} LINOUTBUF;
#define PLINOUTBUF (LINOUTBUF *)                                                /*!< Pointer type of LINOUTBUF */
#define INVALID 0                                                               /*!< BufferOut Status "INVALID" */
#define VALID   1                                                               /*!< BufferOut Status "VALID" */

#define C_COMM_EVENT_NONE           ((uint8_t)0x00U)                            /*!< No Communication Event */
#define C_COMM_EVENT_CHIPRESET      ((uint8_t)(1U << 0))                        /*!< Chip-Reset Communication Event */
#define C_COMM_EVENT_STALL          ((uint8_t)(1U << 1))                        /*!< Stall Communication Event */
#define C_COMM_EVENT_EMRUN          ((uint8_t)(1U << 2))                        /*!< Emergency-run Communication Event */
#define C_COMM_EVENT_LINERROR       ((uint8_t)(1U << 3))                        /*!< LIN-error Communication Event */

/*!*************************************************************************** */
/*                           GLOBAL VARIABLES                                  */
/* *************************************************************************** */
#pragma space dp                                                                /* __TINY_SECTION__ */
extern uint8_t g_u8BufferOutID;                                                 /*!< LIN output buffer is invalid */
extern LININBUF g_LinCmdFrameBuffer;                                            /*!< (Copy of) LIN input frame-buffer */
#if ((LINPROT & LINX) == LIN2) || (LINPROT == LIN13_HVACTB)
extern RFR_DIAG g_DiagResponse;                                                 /*!< LIN Diagnostics Response Buffer */
#endif /* ((LINPROT & LINX) == LIN2) || (LINPROT == LIN13_HVACTB) */
#pragma space none                                                              /* __TINY_SECTION__ */

#pragma space nodp                                                              /* __NEAR_SECTION__ */
extern volatile uint8_t g_u8LinInFrameBufState;                                 /*!< LIN input frame-buffer status */
extern volatile uint8_t g_u8ErrorCommunication;                                 /*!< Communication error */
#if (_SUPPORT_BUSTIMEOUT != FALSE)
extern volatile uint8_t g_u8ErrorCommBusTimeout;                                /*!< Flag indicate of LIN bus time-out occurred */
#endif /* (_SUPPORT_BUSTIMEOUT != FALSE) */
extern uint8_t g_byCommEvent;                                                   /*!< Communication Event */
#pragma space none                                                              /* __NEAR_SECTION__ */

/* MLX4 RAM Buffer LinFrame (MMP191207-1) */
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
extern volatile uint8_t LinProtectedID __attribute((nodp, addr(0x0E47)));  /*lint !e526 */
extern volatile uint8_t LinFrame[8] __attribute((nodp, addr(0x0E48)));  /*lint !e526 */
#elif defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
extern volatile uint8_t LinProtectedID __attribute((nodp, addr(0x0A47)));  /*lint !e526 */
extern volatile uint8_t LinFrame[8] __attribute((nodp, addr(0x0A48)));  /*lint !e526 */
#endif

/*!*************************************************************************** */
/*                           GLOBAL FUNCTIONS                                  */
/* *************************************************************************** */
extern void LIN_Init(void);                                                     /*!< LIN communication initialisation */
#if ((CAN_COMM != FALSE) || (I2C_COMM != FALSE) || (PWM_COMM != FALSE) || (SPI_COMM != FALSE) || \
    ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR)))
extern void LIN_Stop(void);                                                     /*!< Stop LIN Communication */
#endif /* ((CAN_COMM != FALSE) || (I2C_COMM != FALSE) || (PWM_COMM != FALSE) || (SPI_COMM != FALSE) || ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR))) */
extern void HandleLinInMsg(void);                                               /*!< Handle LIN input frame buffer */
extern ml_Status_t mlu_ApplicationStop(void);                                   /*!< Application Stop */
extern uint16_t p_ml_GetBaudRate(uint16_t u16Mlx4Clk);
extern uint16_t p_ml_GetLastBaudRate(uint16_t u16Mlx4Clk);
extern uint16_t p_ml_GetAutoBaudRate(uint16_t u16Mlx4Clk);

#endif /* (LIN_COMM != FALSE) */

#endif /* LIN_COMMUNICATION_H */

/* EOF */
