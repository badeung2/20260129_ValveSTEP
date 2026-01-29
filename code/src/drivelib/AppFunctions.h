/*!*************************************************************************** *
 * \file        AppFunctions.h
 * \brief       MLX8133x Application support functions
 *
 * \note        project MLX8133x
 *
 * \author      Marcel Braat
 *
 * \date        2017-08-15
 *
 * \version     2.0
 *
 * A list of functions:
 *  - Global Functions:
 *              -# Get_AmbientTemperature()
 *              -# Set_Mlx4ErrorState()
 *              -# ClearMlx4CheckPeriodCount()
 *
 * \copyright   MELEXIS Microelectronic Integrated Systems
 *
 * Copyright (C) 2017-2022 Melexis N.V.
 * The Software is being delivered 'AS IS' and Melexis, whether explicitly or
 * implicitly, makes no warranty as to its Use or performance.
 * The user accepts the Melexis Firmware License Agreement.
 *
 * Melexis confidential & proprietary
 * *************************************************************************** */

#ifndef DRIVE_LIB_APPFUNCTIONS_H
#define DRIVE_LIB_APPFUNCTIONS_H

/*!*************************************************************************** */
/*                           INCLUDES                                          */
/* *************************************************************************** */
#include "AppBuild.h"                                                           /* Application Build */

#include "drivelib/Timer.h"                                                     /* Simple Timer support */

/*!*************************************************************************** *
 *                            DEFINES                                          *
 * *************************************************************************** */
/* Intelligent Watchdog */
#define C_IWD_DIV   5U                                                          /*!< WDC_CLK = MCU_CLK / 2^(5+2*DIV) */
#define C_IWD_TO    (uint8_t)((100UL * FPLL) / (1U << (5U + (2U * C_IWD_DIV))))  /*!< 100ms (0...255) */

#define C_SELFHEAT_COMP_PERIOD      500U                                        /*!< Self-heat compensation period: 500 (x 500us) */
#define C_SELFHEAT_INTEGRATOR       64946U                                      /*!< Self-heat integrator constant: 64946 */
#if defined (__MLX81160__)
#define C_SELFHEAT_CONST            180U                                        /*!< Self-heat constant: 180/65536 (Socket-board) */
#else  /* defined (__MLX81160__) */
#define C_SELFHEAT_CONST            85U                                         /*!< Self-heat constant: 85/65536 (EVB board) */
#endif /* defined (__MLX81160__) */
#define C_SELFHEAT_IC               2U                                          /*!< Self-heat offset: 2C */
#if defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#define C_SELFHEAT_IC_IDLE          150                                         /*!< Self-heat IC (  Idle:  7.0mA): VS/1.5 (MMP220128-2) */
#else  /* defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */
#define C_SELFHEAT_IC_IDLE          200                                         /*!< Self-heat IC (  Idle:  6.0mA): VS/2 (MMP200616-1) */
#endif /* defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */
#define C_SELFHEAT_IC_ACTIVE        80                                          /*!< Self-heat IC (Active: 13.5mA): VS/0.8 (MMP200616-1) */

#define C_UV_FILTER_COUNT           C_PI_TICKS_500MS                            /*!< Core-Timer ticks per 500ms */
#define C_OV_FILTER_COUNT           C_PI_TICKS_500MS                            /*!< Core-Timer ticks per 500ms */

#define C_MLX4_STATE_TIMEOUT        (300U * PI_TICKS_PER_MILLISECOND)           /*!< 300 ms */
#define C_MLX4_STATE_ERROR_THRSHLD  4U                                          /*!< After 'n' LIN MLX4 state error, re-initialise LIN interface: 4*300ms = 1.2s */
#define C_MLX4_STATE_IMMEDIATE_RST  0x80U                                       /*!< MLX4 Immediate reset request */

/* Baudrate check (9600+/-10%, 10417+/-10% and 19200+/-10%) */
#define C_LIN_19200_MAX             (uint16_t)(19200 * 1.1)                     /*!< LIN Baudrate 19k2 + 10% */
#define C_LIN_19200_MIN             (uint16_t)(19200 * 0.9)                     /*!< LIN Baudrate 19k2 - 10% */
#define C_LIN_10417_MAX             (uint16_t)(10417 * 1.1)                     /*!< LIN Baudrate 10k4 + 10% */
#define C_LIN_10417_MIN             (uint16_t)(10417 * 0.9)                     /*!< LIN Baudrate 10k4 - 10% */
#define C_LIN_9600_MAX              (uint16_t)(9600 * 1.1)                      /*!< LIN Baudrate  9k6 + 10% */
#define C_LIN_9600_MIN              (uint16_t)(9600 * 0.9)                      /*!< LIN Baudrate  9k6 - 10% */

#define C_CHIP_CODE(x, y, z) \
    ( (uint16_t)( (((uint16_t)(x) - (uint16_t)'@') << 10) | (((uint16_t)(y) - (uint16_t)'@') << 5) | \
                  ((uint16_t)(z) - (uint16_t)'@') ) )                           /*!< Chip Code macro */
#define C_CHIP_STATE_FATAL_RECOVER_ENA      C_CHIP_CODE('F','R','E')            /*!< Fatal Recovery Enabled */
#define C_CHIP_STATE_FATAL_CRASH_RECOVERY   C_CHIP_CODE('F','C','R')            /*!< Fatal Crash Recovery */

/*!*************************************************************************** */
/*                           GLOBAL VARIABLES                                  */
/* *************************************************************************** */
#pragma space nodp                                                              /* __NEAR_SECTION__ */
extern uint16_t l_u16ReversePolarityVdrop;                                      /*!< Reverse polarity diode voltage drop */
#if (LIN_COMM != FALSE)
extern uint8_t g_u8MLX4_RAM_Dynamic_CRC;                                        /*!< MLX4-RAM Dynamic Frame-IDs CRC */
#if (_SUPPORT_MLX_CHIP_STATUS != FALSE)
extern uint32_t g_u32FlashBist;
extern uint32_t g_u32EepromBist;
#endif /* (_SUPPORT_MLX_CHIP_STATUS != FALSE) */
#endif /* (LIN_COMM != FALSE) */
#pragma space none                                                              /* __NEAR_SECTION__ */

/*!*************************************************************************** */
/*                           GLOBAL FUNCTIONS                                  */
/* *************************************************************************** */
extern void AppInit(void);                                                      /*!< Initialise Application */
extern void AppResetFlags(void);                                                /*!< Clear (reset) LIN status flags */
extern void AppDegradedCheck(void);                                             /*!< Application degraded check */
extern void AppPeriodicTimerEvent(uint16_t u16Period);                          /*!< Perform Application support Functions Periodic Timer Event updates. */
extern void AppBackgroundHandler(void);                                         /*!< Handle Generic Back-ground tasks */
extern void AppStop(void);                                                      /*!< Stop the application */
extern void AppSleepWithWakeUpTimer(void);                                      /*!< Application into Sleep-mode with Wake-up Timer activated */
extern void AppSleep(void);                                                     /*!< Application into Sleep-mode */
extern void AppReset(void);                                                     /*!< Application and IC reset */

/*!*************************************************************************** *
 * Get_AmbientTemperature
 * \brief   Get variable l_i16AmbientTemperature
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  int16_t l_i16AmbientTemperature
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline int16_t Get_AmbientTemperature(void)
{
    extern int16_t l_i16AmbientTemperature;                                     /* Ambient Temperature */
    return (l_i16AmbientTemperature);
} /* End of Get_AmbientTemperature() */

/*!*************************************************************************** *
 * Set_Mlx4ErrorState
 * \brief   Set variable l_u8Mlx4ErrorState
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u8Value: New value for l_u8Mlx4ErrorState
 * \return  -
 * *************************************************************************** *
 * \details in-line function to set 'local' variable
 * *************************************************************************** */
static inline void Set_Mlx4ErrorState(uint8_t u8Value)
{
    extern uint8_t l_u8Mlx4ErrorState;                                          /* Number of MLX4 Error states occurred */
    l_u8Mlx4ErrorState = u8Value;
} /* End of Set_Mlx4ErrorState() */

/*!*************************************************************************** *
 * ClearMlx4CheckPeriodCount
 * \brief   Clear (zero) variable l_u16Mlx4CheckPeriodCount
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details in-line function to clear 'local' variable
 * *************************************************************************** */
static inline void ClearMlx4CheckPeriodCount(void)
{
    extern uint16_t l_u16Mlx4CheckPeriodCount;
    l_u16Mlx4CheckPeriodCount &= (uint8_t) ~C_MLX4_STATE_IMMEDIATE_RST;
} /* End of ClearMlx4CheckPeriodCount() */

#endif /* DRIVE_LIB_APPFUNCTIONS_H */

/*EOF */

