/*!*************************************************************************** *
 * \file        LIN_AutoAddressing.c
 * \brief       MLX8133x/40 LIN Auto Addressing handling
 *
 * \note        project MLX8133x/40
 *
 * \author      Marcel Braat
 *
 * \date        2018-01-02
 *
 * \version     2.0
 *
 * A list of functions:
 *  - Global Functions:
 *           -# mlu_AutoAddressingStep()
 *           -# ml_SetSlaveNotAddressed()
 *           -# ml_SetSlaveAddressed()
 *           -# ml_InitAutoAddressing()
 *           -# ml_StopAutoAddressing()
 *           -# ml_GetAutoaddressingStatus()
 *           -# AutoAddressingReadADCResult()
 *           -# ClearAAData()
 *
 * \copyright   MELEXIS Microelectronic Integrated Systems
 *
 * Copyright (C) 2018-2022 Melexis N.V.
 * The Software is being delivered 'AS IS' and Melexis, whether explicitly or
 * implicitly, makes no warranty as to its Use or performance.
 * The user accepts the Melexis Firmware License Agreement.
 *
 * Melexis confidential & proprietary
 *
 *                          RAM     Flash
 * _SUPPORT_LIN_AA:         324 B   2028 B (including Screening)
 * _SUPPORT_LIN_AA:          64 B   1652 B (excluding Screening)
 * *************************************************************************** */

/*!*************************************************************************** *
 *                              I N C L U D E S                                *
 * *************************************************************************** */
#include "AppBuild.h"                                                           /* Application Build */

#if (LIN_COMM != FALSE)

#include <mls_api.h>                                                            /* Melexis LIN module (MMP180430-1) */

#if (_SUPPORT_LIN_AA != FALSE)

#include "drivelib/ADC.h"                                                       /* ADC support */
#include "drivelib/NV_Functions.h"                                              /* Non Volatile Memory Functions & Layout */
#include "drivelib/Timer.h"                                                     /* Simple Timer support */
#include "camculib/private_mathlib.h"                                           /* Private Math Library support */

#include "commlib/LIN_AutoAddressing.h"                                         /* LIN Auto-Addressing support */
#include "commlib/LIN_Communication.h"                                          /* LIN Communication support */

#include <plib.h>                                                               /* Product libraries */
#include <string.h>                                                             /* memcpy() */

#ifndef LIN_AA_EXT_VOLTAGE
#define LIN_AA_EXT_VOLTAGE      FALSE                                           /* LIN AA Extended Voltage Range support */
#endif /* LIN_AA_EXT_VOLTAGE */

/* *************************************************************************** */
/*                           DEFINITIONS                                       */
/* *************************************************************************** */
#define C_LINAA_RBOND               60U                                         /*!< Bondwire resistance [mR] (QFN) */
#if (LIN_AA_NV == FALSE)
/* 8:6 = M_PORT_LINAA2_LCD_SEL_LINAA, 5:0 = M_TRIM_MISC_TRIM_LCD_LINAA */
#define C_LINAA_CS_112              (1U << 6) | 61U                             /*!< Selection and Trim for 1.12mA */
#define C_LINAA_CS_205              (3U << 6) | 55U                             /*!< Selection and Trim for 2.05mA */
#define C_LINAA_CS_462              (7U << 6) | 00U                             /*!< Selection and Trim for 4.62mA */
#define C_LINAA_SC_GAIN             (0U << 4 | C_SAGAIN)                        /*!< 2de amplifier Gain setting */
#define C_LINAA_GDM                 C_GLAA                                      /*!< Default LIN-AA Differential-Mode Gain */
#define C_LINAA_SAADMCM             0U                                          /*!< No Software compensation */
/* Assume internal shunt 900mR (design specification) */
#define C_LINAA_ISHUNT              900U                                        /*!< Internal LIN-AA Shunt [mR] */
#if (LINPROT == LIN2X_HVAC52)
#define C_LINAA_ESHUNT              200U                                        /*!< External LIN-AA Shunt [mR] */
#else  /* (LINPROT == LIN2X_HVAC52) */
#define C_LINAA_ESHUNT              0U                                          /*!< No external LIN-AA shunt */
#endif /* (LINPROT == LIN2X_HVAC52) */
#else  /* (LIN_AA_NV == FALSE) */
#if (LINPROT == LIN2X_HVAC52)
#define C_LINAA_ESHUNT              ((ENH_LIN_PARAMS_t *)ADDR_NV_ENH_LIN_1)->u8LINAA_EShunt   /*!< External LIN-AA Shunt value (Non Volatile Memory) */
#else  /* (LINPROT == LIN2X_HVAC52) */
#define C_LINAA_ESHUNT              0U                                          /*!< No external LIN-AA shunt */
#endif /* (LINPROT == LIN2X_HVAC52) */
#define C_LINAA_ISHUNT              ((MLX_CALIB_t *)ADDR_NV_MLX_CALIB)->u16APP_TRIM12_LINAA_ISHUNT   /*!< LIN-AA Internal (IC) shunt */
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81350__)
#define C_LINAA_GDM                 ((int16_t)(0 - (((MLX_CALIB_t *)ADDR_NV_MLX_CALIB)->u16APP_TRIM15_SAADM)))    /*!< LIN-AA Gain Differential-mode */
#define C_LINAA_SAADMCM             ((MLX_CALIB_t *)ADDR_NV_MLX_CALIB)->i16APP_TRIM14_AASDMCM   /*!< LIN-AA Common-mode to Differential-mode Gain */
#elif defined (__MLX81340__)
#define C_LINAA_GDM                 ((int16_t)(((MLX_CALIB_t *)ADDR_NV_MLX_CALIB)->u16APP_TRIM15_SAADM))    /*!< LIN-AA Gain Differential-mode */
#define C_LINAA_SAADMCM             ((int16_t)(0 - ((MLX_CALIB_t *)ADDR_NV_MLX_CALIB)->i16APP_TRIM14_AASDMCM))    /*!< LIN-AA Common-mode to Differential-mode Gain */
#endif
#endif /* (LIN_AA_NV == FALSE) */

#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
#define C_LIN_AA_CURR_DIVISOR       (int16_t)((4096L * 50 * 8) / 100)           /*!< Gain * 4096, 50LSB/mA, 8 DM-Sa, 100 [10uA/mA] (MMP200408-1) */
#define C_LIN_AA_CURR_SDIV          14                                          /*!< Shift-divisor (N32 >> 14) (MMP221101-1) */
#define C_LIN_AA_SAADMCM_DIVISOR    (int16_t)(32768L / 8)                       /*!< (2^15)/8 (MMP200408-1) */
#define C_LIN_AA_SAADMCM_SDIV       12                                          /*!< Shift-divisor (N32 >> 12) (MMP221101-1) */
#elif defined (__MLX81340__)
#define C_LIN_AA_CURR_DIVISOR       (int16_t)(((4096L * 50 * 16) / 100) - 1)    /*!< Gain * 4096, 50LSB/mA, 16 DM-Sa, 100 [10uA/mA] (MMP200408-1) */
#define C_LIN_AA_CURR_SDIV          15                                          /*!< Shift-divisor (N32 >> 15) (MMP221101-1) */
#define C_LIN_AA_SAADMCM_DIVISOR    (int16_t)(32768L / 16)                      /*!< (2^15)/16 */
#define C_LIN_AA_SAADMCM_SDIV       11                                          /*!< Shift-divisor (N32 >> 11) (MMP221101-1) */
#elif defined (__MLX81350__)
#define C_LIN_AA_CURR_DIVISOR       (int16_t)((4096L * 50 * 8) / 100)           /*!< Gain * 4096, 50LSB/mA, 8 DM-Sa, 100 [10uA/mA] (TODO[MMP50]) */
#define C_LIN_AA_CURR_SDIV          14                                          /*!< Shift-divisor (N32 >> 14) (TODO[MMP50]) */
#define C_LIN_AA_SAADMCM_DIVISOR    (int16_t)(32768L / 8)                       /*!< (2^15)/8 (TODO[MMP50]) */
#define C_LIN_AA_SAADMCM_SDIV       12                                          /*!< Shift-divisor (N32 >> 12) (TODO[MMP50]) */
#endif
#define C_CURRENT_SOURCE            240U                                        /*!< (Total) Current Source at Final Selection */

#define C_SLOW_BAUDRATE_DELAY   (uint16_t)((40UL * FPLL) / C_DELAY_CONST)       /*!< 40us delay */
#define C_T_FRAME_MAX (uint32_t)((1000000UL / CT_PERIODIC_RATE) * 1.4 * (34 + 10 * (8 + 1)))  /*!< LIN-Message time-out */

#define C_LIN_KEY                   0xB2A3U                                     /*!< LIN-key to access LIN-AA */

#define _LINAA_ASM                  TRUE                                        /*!< Use of ASM instruction for time-critical routines */

/*!*************************************************************************** */
/*                           GLOBAL & LOCAL VARIABLES                          */
/* *************************************************************************** */
#pragma space dp
static volatile uint8_t l_u8AutoAddressingFlags = (uint8_t)0x00U;               /*!< Reset all auto addressing flags (MMP180801-2) */
volatile uint8_t g_u8LinAAMode = 0U;                                            /*!< Not in LIN-AA mode */
#pragma space none

#pragma space nodp
#if (LIN_AA_TIMEOUT != FALSE)
uint8_t g_u8LinAATimeout = 0U;                                                  /*!< LIN-AA Timeout counter (seconds) */
uint16_t g_u16LinAATicker = 0U;                                                 /*!< LIN-AA Ticker counter */
#if (LIN_AA_LINFRAME_TIMEOUT != FALSE)
uint16_t l_u16LinFrameMaxTime = 0U;                                             /*!< LIN-AA Frame Time [S_TIMER ticks] */
uint16_t l_u16LinFrameTimeOut = 0U;                                             /*!< LIN-AA Frame Time-out Counter */
#endif /* (LIN_AA_LINFRAME_TIMEOUT != FALSE) */
#endif /* (LIN_AA_TIMEOUT != FALSE) */
#if (LIN_AA_BSM_SNPD_R1p0 != FALSE)
uint16_t l_u16SlowBaudrateAdjustment = 0U;                                      /*!< Minimum time */
uint16_t l_u16LinAATimerPeriod;                                                 /*!< LIN-AA Time-out period */
#if (LIN_AA_INFO != FALSE)
PSNPD_DATA pSNPD_Data;                                                          /*!< LIN-AA Screening/test data-structure pointer */
#endif /* (LIN_AA_INFO != FALSE) */
#endif /* (LIN_AA_BSM_SNPD_R1p0 != FALSE) */
#if (_SUPPORT_LINAA_VS_COMPENSATION != FALSE)
static volatile uint16_t l_u16AutoAddressingVS = 0U;                            /*!< Supply Voltage ADC result [ADC-LSB] (MMP240214-1) */
#endif /* (_SUPPORT_LINAA_VS_COMPENSATION != FALSE) */
static uint16_t l_u16AutoAddressingCM_0 = 0U;                                   /*!< Running sum of up to 2 Common mode ADC results [ADC-LSB] */
static uint16_t l_u16AutoAddressingDM_0 = 0;                                    /*!< Running sum of up to 8 Differential mode ADC results [ADC-LSB] */
static volatile uint16_t l_u16AutoAddressingCM = 0U;                            /*!< Running sum of up to 2 Common mode ADC results [ADC-LSB] */
static volatile uint16_t l_u16AutoAddressingDM = 0U;                            /*!< Running sum of up to 8 Differential mode ADC results [ADC-LSB] */
static int16_t l_i16LINAA_PreSelCurr;                                           /*!< LIN-AA pre-selection currents [10uA] */
static int16_t l_i16LINAA_FinalSelCurr;                                         /*!< LIN-AA final-selection currents [10uA] */
#if (PROJECT_ID == 0x050F) || (PROJECT_ID == 0x0510) || (PROJECT_ID == 0x0511) || /* MLX81330B1 SO8-001|QFN|SO8-101 */ \
    (PROJECT_ID == 0x070A) || (PROJECT_ID == 0x070B) || (PROJECT_ID == 0x070C)    /* MLX81332B1 QFN|SO8-001|SO8-101 */
uint16_t l_u16PreLinAA_MotorDrvState;                                           /*!< Motor-driver supply-state (MMP190214-1) */
#endif /* (PROJECT_ID) */
#if (LIN_AA_BSM_SNPD_R1p0 == FALSE)
volatile uint8_t l_e8AutoAddressingState = AUTOADDRESSING_IDLE;                 /*!< LIN-Slave auto-addressing state */
volatile uint8_t l_u8AutoAddressingPulse = 0U;                                  /*!< Local copy of pulse variable set by MLX4 LIN API */
uint8_t l_u8Step;                                                               /*!< LIN-AA step number */
#endif /* (LIN_AA_BSM_SNPD_R1p0 == FALSE) */
#if (LIN_AA_INFO != FALSE)
/* LIN Auto-addressing info */
SNPD_DATA l_aSNPD_Data[LIN_AA_INFO_SZ];                                         /*!< LIN-AA Screening/test data structure buffer */
static uint8_t l_u8SNPD_CycleCount;                                             /*!< LIN-AA Screening/test data structure index */
uint8_t g_u8SNPD_CycleCountComm = 0U;                                           /*!< Communication Cycle counter LIN-AA info */
#endif /* (LIN_AA_INFO != FALSE) */
volatile ADC_LINAA LinAutoAddressing;  /*lint !e552 */                          /*!< LIN Auto Addressing measurement results */
#if defined (__MLX81332B02__) || defined (__MLX81334__)
static uint16_t l_u16TrimBG;                                                    /*!< Storage Trim Band Gap */
static uint16_t l_u16TrimVdd;                                                   /*!< Storage Trim VDD */
#endif /* defined (__MLX81332B02__) || defined (__MLX81334__) */

#pragma space none

#if (LIN_AA_RESOURCE == LIN_AA_RESOURCE_CTIMER0)
/************************ AUTO ADDRESSING ***********************/
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
/*! ADC SBASE channel configuration for LIN-AA Common-mode (CTIMER0) */
#define C_ADC_LINAA_CM_CT0                                                      \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_LINAA_CM,                                     \
            .u3AdcVref = C_ADC_VREF_2_50_V,                                     \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_CTIMER0_INT3,                      \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }
/*! ADC SBASE channel configuration for LIN-AA Differential-mode (CTIMER0) */
#define C_ADC_LINAA_DM_CT0                                                      \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_LINAA_DM,                                     \
            .u3AdcVref = C_ADC_VREF_2_50_V,                                     \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_CTIMER0_INT3,                      \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }
#if (_SUPPORT_LINAA_VS_COMPENSATION != FALSE)
/*! ADC SBASE channel configuration for VS (CTIMER0) */
#define C_ADC_VS_CT0                                                            \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_VS_HV,                                        \
            .u3AdcVref = C_ADC_VREF_2_50_V,                                     \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_CTIMER0_INT3,                      \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }
#endif /* (_SUPPORT_LINAA_VS_COMPENSATION != FALSE) */
#elif defined (__MLX81340__)
/*! ADC SBASE channel configuration for LIN-AA Common-mode (CTIMER0) */
#define C_ADC_LINAA_CM_CT0                                                      \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_LINAA_CM,                                     \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_CTIMER0_INT3                       \
        }                                                                       \
    }
/*! ADC SBASE channel configuration for LIN-AA Differential-mode (CTIMER1) */
#define C_ADC_LINAA_DM_CT0                                                      \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_LINAA_DM,                                     \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_CTIMER0_INT3                       \
        }                                                                       \
    }
#if (_SUPPORT_LINAA_VS_COMPENSATION != FALSE)
/*! ADC SBASE channel configuration for VS (CTIMER0) */
#define C_ADC_VS_CT0                                                            \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_VS_HV,                                        \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_CTIMER0_INT3                       \
        }                                                                       \
    }
#endif /* (_SUPPORT_LINAA_VS_COMPENSATION != FALSE) */
#elif defined (__MLX81350__)
/*! ADC SBASE channel configuration for LIN-AA Common-mode (CTIMER0) */
#define C_ADC_LINAA_CM_CT0                                                      \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_LINAA_CM,                                     \
            .u3AdcReserved = 0U,                                                \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_CTIMER0_INT3,                      \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }
/*! ADC SBASE channel configuration for LIN-AA Differential-mode (CTIMER0) */
#define C_ADC_LINAA_DM_CT0                                                      \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_LINAA_DM,                                     \
            .u3AdcReserved = 0U,                                                \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_CTIMER0_INT3,                      \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }
#if (_SUPPORT_LINAA_VS_COMPENSATION != FALSE)
/*! ADC SBASE channel configuration for VS (CTIMER0) */
#define C_ADC_VS_CT0                                                            \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_VS_HV,                                        \
            .u3AdcReserved = 0U,                                                \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_CTIMER0_INT3,                      \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }
#endif /* (_SUPPORT_LINAA_VS_COMPENSATION != FALSE) */
#endif
/*! LIN-AA ADC channel array */
static ADC_SDATA_t const SBASE_LIN[] = {
#if (_SUPPORT_LINAA_VS_COMPENSATION != FALSE)
    C_ADC_VS_CT0,                                                               /* Supply Compensation (MMP240214-1) */
#endif /* (_SUPPORT_LINAA_VS_COMPENSATION != FALSE) */
    C_ADC_LINAA_CM_CT0,                                                         /* Channel 7 : LIN-AA Common-mode, #1 */
    C_ADC_LINAA_CM_CT0,                                                         /* Channel 7 : LIN-AA Common-mode, #2 */
    C_ADC_LINAA_DM_CT0,                                                         /* Channel 6 : LIN-AA Differential, #1 */
    C_ADC_LINAA_DM_CT0,                                                         /* Channel 6 : LIN-AA Differential, #2 */
    C_ADC_LINAA_DM_CT0,                                                         /* Channel 6 : LIN-AA Differential, #3 */
    C_ADC_LINAA_DM_CT0,                                                         /* Channel 6 : LIN-AA Differential, #4 */
    C_ADC_LINAA_DM_CT0,                                                         /* Channel 6 : LIN-AA Differential, #5 */
    C_ADC_LINAA_DM_CT0,                                                         /* Channel 6 : LIN-AA Differential, #6 */
    C_ADC_LINAA_DM_CT0,                                                         /* Channel 6 : LIN-AA Differential, #7 */
    C_ADC_LINAA_DM_CT0,                                                         /* Channel 6 : LIN-AA Differential, #8 */
    C_ADC_LINAA_DM_CT0,                                                         /* Channel 6 : LIN-AA Differential, #9 */
    C_ADC_LINAA_DM_CT0,                                                         /* Channel 6 : LIN-AA Differential, #10 */
    C_ADC_LINAA_DM_CT0,                                                         /* Channel 6 : LIN-AA Differential, #11 */
    C_ADC_LINAA_DM_CT0,                                                         /* Channel 6 : LIN-AA Differential, #12 */
    C_ADC_LINAA_DM_CT0,                                                         /* Channel 6 : LIN-AA Differential, #13 */
    C_ADC_LINAA_DM_CT0,                                                         /* Channel 6 : LIN-AA Differential, #14 */
    C_ADC_LINAA_DM_CT0,                                                         /* Channel 6 : LIN-AA Differential, #15 */
    C_ADC_LINAA_DM_CT0,                                                         /* Channel 6 : LIN-AA Differential, #16 */
    {.u16 = C_ADC_EOS}
};                                                                              /* End-of-Sequence */

/* **************************************************************************** *
 *  LIN API event: mlu_AutoAddressingStep
 * **************************************************************************** */
/* Timer2 is used for LIN Auto-Addressing ADC trigger; Timer1 could also be used, as the motor is not running */
#define LINAA_TIMER_RESET()         {IO_CTIMER0_CTRL = B_CTIMER0_STOP; }
#define LINAA_TIMER_SETUP()         {IO_CTIMER0_CTRL = C_CTIMER0_DIV_CPU | C_CTIMER0_MODE_TIMER; }
#define LINAA_TIMER_START()         {IO_CTIMER0_CTRL = B_CTIMER0_START; }
#define LINAA_TIMER_STOP()          {IO_CTIMER0_CTRL = B_CTIMER0_STOP; }
#define LINAA_TIMER_PERIOD(p)       {IO_CTIMER0_TREGB = p; }
#define LINAA_TIMER_IRQ_DIS()       {IO_MLX16_ITC_MASK1 &= ~B_MLX16_ITC_MASK1_CTIMER0_3; }
#elif (LIN_AA_RESOURCE == LIN_AA_RESOURCE_CTIMER1)
/************************ AUTO ADDRESSING ***********************/
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
/*! ADC SBASE channel configuration for LIN-AA Common-mode (CTIMER1) */
#define C_ADC_LINAA_CM_CT1                                                      \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_LINAA_CM,                                     \
            .u3AdcVref = C_ADC_VREF_2_50_V,                                     \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_CTIMER1_INT3,                      \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }
/*! ADC SBASE channel configuration for LIN-AA Differential-mode (CTIMER1) */
#define C_ADC_LINAA_DM_CT1                                                      \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_LINAA_DM,                                     \
            .u3AdcVref = C_ADC_VREF_2_50_V,                                     \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_CTIMER1_INT3,                      \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }
#if (_SUPPORT_LINAA_VS_COMPENSATION != FALSE)
/*! ADC SBASE channel configuration for VS (CTIMER1) */
#define C_ADC_VS_CT1                                                            \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_VS_HV,                                        \
            .u3AdcVref = C_ADC_VREF_2_50_V,                                     \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_CTIMER1_INT3,                      \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }
#endif /* (_SUPPORT_LINAA_VS_COMPENSATION != FALSE) */
#elif defined (__MLX81340__)
/*! ADC SBASE channel configuration for LIN-AA Common-mode (CTIMER1) */
#define C_ADC_LINAA_CM_CT1                                                      \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_LINAA_CM,                                     \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_CTIMER1_INT3                       \
        }                                                                       \
    }
/*! ADC SBASE channel configuration for LIN-AA Differential-mode (CTIMER1) */
#define C_ADC_LINAA_DM_CT1                                                      \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_LINAA_DM,                                     \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_CTIMER1_INT3                       \
        }                                                                       \
    }
#if (_SUPPORT_LINAA_VS_COMPENSATION != FALSE)
/*! ADC SBASE channel configuration for VS (CTIMER1) */
#define C_ADC_VS_CT1                                                            \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_VS_HV,                                        \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_CTIMER1_INT3                       \
        }                                                                       \
    }
#endif /* (_SUPPORT_LINAA_VS_COMPENSATION != FALSE) */
#elif defined (__MLX81350__)
/*! ADC SBASE channel configuration for LIN-AA Common-mode (CTIMER1) */
#define C_ADC_LINAA_CM_CT1                                                      \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_LINAA_CM,                                     \
            .u3AdcReserved = 0U,                                                \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_CTIMER1_INT3,                      \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }
/*! ADC SBASE channel configuration for LIN-AA Differential-mode (CTIMER1) */
#define C_ADC_LINAA_DM_CT1                                                      \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_LINAA_DM,                                     \
            .u3AdcReserved = 0U,                                                \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_CTIMER1_INT3,                      \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }
#if (_SUPPORT_LINAA_VS_COMPENSATION != FALSE)
/*! ADC SBASE channel configuration for VS (CTIMER1) */
#define C_ADC_VS_CT1                                                            \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_VS_HV,                                        \
            .u3AdcReserved = 0U,                                                \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_CTIMER1_INT3,                      \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }
#endif /* (_SUPPORT_LINAA_VS_COMPENSATION != FALSE) */
#endif
/*! LIN-AA ADC channel array */
static ADC_SDATA_t const SBASE_LIN[] = {
#if (_SUPPORT_LINAA_VS_COMPENSATION != FALSE)
    C_ADC_VS_CT1,                                                               /* Supply Compensation (MMP240214-1) */
#endif /* (_SUPPORT_LINAA_VS_COMPENSATION != FALSE) */
    C_ADC_LINAA_CM_CT1,                                                         /* Channel 7 : LIN-AA Common-mode, #1 */
    C_ADC_LINAA_CM_CT1,                                                         /* Channel 7 : LIN-AA Common-mode, #2 */
    C_ADC_LINAA_DM_CT1,                                                         /* Channel 6 : LIN-AA Differential, #1 */
    C_ADC_LINAA_DM_CT1,                                                         /* Channel 6 : LIN-AA Differential, #2 */
    C_ADC_LINAA_DM_CT1,                                                         /* Channel 6 : LIN-AA Differential, #3 */
    C_ADC_LINAA_DM_CT1,                                                         /* Channel 6 : LIN-AA Differential, #4 */
    C_ADC_LINAA_DM_CT1,                                                         /* Channel 6 : LIN-AA Differential, #5 */
    C_ADC_LINAA_DM_CT1,                                                         /* Channel 6 : LIN-AA Differential, #6 */
    C_ADC_LINAA_DM_CT1,                                                         /* Channel 6 : LIN-AA Differential, #7 */
    C_ADC_LINAA_DM_CT1,                                                         /* Channel 6 : LIN-AA Differential, #8 */
    C_ADC_LINAA_DM_CT1,                                                         /* Channel 6 : LIN-AA Differential, #9 */
    C_ADC_LINAA_DM_CT1,                                                         /* Channel 6 : LIN-AA Differential, #10 */
    C_ADC_LINAA_DM_CT1,                                                         /* Channel 6 : LIN-AA Differential, #11 */
    C_ADC_LINAA_DM_CT1,                                                         /* Channel 6 : LIN-AA Differential, #12 */
    C_ADC_LINAA_DM_CT1,                                                         /* Channel 6 : LIN-AA Differential, #13 */
    C_ADC_LINAA_DM_CT1,                                                         /* Channel 6 : LIN-AA Differential, #14 */
    C_ADC_LINAA_DM_CT1,                                                         /* Channel 6 : LIN-AA Differential, #15 */
    C_ADC_LINAA_DM_CT1,                                                         /* Channel 6 : LIN-AA Differential, #16 */
    {.u16 = C_ADC_EOS}
};                                                                              /* End-of-Sequence */

/* **************************************************************************** *
 *  LIN API event: mlu_AutoAddressingStep
 * **************************************************************************** */
/* Timer2 is used for LIN Auto-Addressing ADC trigger; Timer1 could also be used, as the motor is not running */
#define LINAA_TIMER_RESET()         {IO_CTIMER1_CTRL = B_CTIMER1_STOP; }        /*!< Reset LIN-AA Timer */
#define LINAA_TIMER_SETUP()         {IO_CTIMER1_CTRL = C_CTIMER1_DIV_CPU | C_CTIMER1_MODE_TIMER; }  /*!< Setup LIN-AA Timer */
#define LINAA_TIMER_START()         {IO_CTIMER1_CTRL = B_CTIMER1_START; }       /*!< Start LIN-AA Timer */
#define LINAA_TIMER_STOP()          {IO_CTIMER1_CTRL = B_CTIMER1_STOP; }        /*!< Stop LIN-AA Timer */
#define LINAA_TIMER_PERIOD(p)       {IO_CTIMER1_TREGB = p; }                    /*!< Set LIN-AA Timer period */
#define LINAA_TIMER_IRQ_DIS()       {IO_MLX16_ITC_MASK1_S &= ~B_MLX16_ITC_MASK1_CTIMER1_3; } /*!< Disable LIN-AA Timer IRQ */
#elif (LIN_AA_RESOURCE == LIN_AA_RESOURCE_PWM_MASTER1)
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
/*! ADC SBASE channel configuration for LIN-AA Common-mode (PWM MASTER1) */
#define C_ADC_LINAA_CM_PWM_M1C                                                  \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_LINAA_CM,                                     \
            .u3AdcVref = C_ADC_VREF_2_50_V,                                     \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,                         \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }
/*! ADC SBASE channel configuration for LIN-AA Differential-mode (PWM MASTER1) */
#define C_ADC_LINAA_DM_PWM_M1C                                                  \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_LINAA_DM,                                     \
            .u3AdcVref = C_ADC_VREF_2_50_V,                                     \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,                         \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }
#if (_SUPPORT_LINAA_VS_COMPENSATION != FALSE)
/*! ADC SBASE channel configuration for VS (PWM MASTER1) */
#define C_ADC_VS_PWM_M1C                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_VS_HV,                                        \
            .u3AdcVref = C_ADC_VREF_2_50_V,                                     \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,                         \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }
#endif /* (_SUPPORT_LINAA_VS_COMPENSATION != FALSE) */
#elif defined (__MLX81340__)
/*! ADC SBASE channel configuration for LIN-AA Common-mode (PWM MASTER1) */
#define C_ADC_LINAA_CM_PWM_M1C                                                  \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_LINAA_CM,                                     \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT                          \
        }                                                                       \
    }
/*! ADC SBASE channel configuration for LIN-AA Differential-mode (PWM MASTER1) */
#define C_ADC_LINAA_DM_PWM_M1C                                                  \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_LINAA_DM,                                     \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT                          \
        }                                                                       \
    }
#if (_SUPPORT_LINAA_VS_COMPENSATION != FALSE)
/*! ADC SBASE channel configuration for VS (PWM MASTER1) */
#define C_ADC_VS_PWM_M1C                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_VS_HV,                                        \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT                          \
        }                                                                       \
    }
#endif /* (_SUPPORT_LINAA_VS_COMPENSATION != FALSE) */
#elif defined (__MLX81350__)
/*! ADC SBASE channel configuration for LIN-AA Common-mode (PWM MASTER1) */
#define C_ADC_LINAA_CM_PWM_M1C                                                  \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_LINAA_CM,                                     \
            .u3AdcReserved = 0U,                                                \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,                         \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }
/*! ADC SBASE channel configuration for LIN-AA Differential-mode (PWM MASTER1) */
#define C_ADC_LINAA_DM_PWM_M1C                                                  \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_LINAA_DM,                                     \
            .u3AdcReserved = 0U,                                                \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,                         \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }
#if (_SUPPORT_LINAA_VS_COMPENSATION != FALSE)
/*! ADC SBASE channel configuration for VS (PWM MASTER1) */
#define C_ADC_VS_PWM_M1C                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_VS_HV,                                        \
            .u3AdcReserved = 0U,                                                \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,                         \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }
#endif /* (_SUPPORT_LINAA_VS_COMPENSATION != FALSE) */
#endif
/*! LIN-AA ADC channel array */
static ADC_SDATA_t const SBASE_LIN[] = {
#if (_SUPPORT_LINAA_VS_COMPENSATION != FALSE)
    C_ADC_VS_PWM_M1C,                                                           /* Supply Compensation (MMP240214-1) */
#endif /* (_SUPPORT_LINAA_VS_COMPENSATION != FALSE) */
    C_ADC_LINAA_CM_PWM_M1C,                                                     /* Channel 7 : LIN-AA Common-mode, #1 */
    C_ADC_LINAA_CM_PWM_M1C,                                                     /* Channel 7 : LIN-AA Common-mode, #2 */
    C_ADC_LINAA_DM_PWM_M1C,                                                     /* Channel 6 : LIN-AA Differential, #1 */
    C_ADC_LINAA_DM_PWM_M1C,                                                     /* Channel 6 : LIN-AA Differential, #2 */
    C_ADC_LINAA_DM_PWM_M1C,                                                     /* Channel 6 : LIN-AA Differential, #3 */
    C_ADC_LINAA_DM_PWM_M1C,                                                     /* Channel 6 : LIN-AA Differential, #4 */
    C_ADC_LINAA_DM_PWM_M1C,                                                     /* Channel 6 : LIN-AA Differential, #5 */
    C_ADC_LINAA_DM_PWM_M1C,                                                     /* Channel 6 : LIN-AA Differential, #6 */
    C_ADC_LINAA_DM_PWM_M1C,                                                     /* Channel 6 : LIN-AA Differential, #7 */
    C_ADC_LINAA_DM_PWM_M1C,                                                     /* Channel 6 : LIN-AA Differential, #8 */
    C_ADC_LINAA_DM_PWM_M1C,                                                     /* Channel 6 : LIN-AA Differential, #9 */
    C_ADC_LINAA_DM_PWM_M1C,                                                     /* Channel 6 : LIN-AA Differential, #10 */
    C_ADC_LINAA_DM_PWM_M1C,                                                     /* Channel 6 : LIN-AA Differential, #11 */
    C_ADC_LINAA_DM_PWM_M1C,                                                     /* Channel 6 : LIN-AA Differential, #12 */
    C_ADC_LINAA_DM_PWM_M1C,                                                     /* Channel 6 : LIN-AA Differential, #13 */
    C_ADC_LINAA_DM_PWM_M1C,                                                     /* Channel 6 : LIN-AA Differential, #14 */
    C_ADC_LINAA_DM_PWM_M1C,                                                     /* Channel 6 : LIN-AA Differential, #15 */
    C_ADC_LINAA_DM_PWM_M1C,                                                     /* Channel 6 : LIN-AA Differential, #16 */
    {.u16 = C_ADC_EOS}
};                                                                              /* End-of-Sequence */

/* **************************************************************************** *
 *  LIN API event: mlu_AutoAddressingStep
 * **************************************************************************** */
/* Timer2 is used for LIN Auto-Addressing ADC trigger; Timer1 could also be used, as the motor is not running */
#define LINAA_TIMER_RESET()         {IO_PWM_MASTER1_CTRL = B_PWM_MASTER1_STOP; }
#define LINAA_TIMER_SETUP()         {IO_PWM_MASTER1_CTRL = (0U << 8) | C_PWM_MASTER1_MODE_MIRROR; }
#define LINAA_TIMER_START()         {IO_PWM_MASTER1_CTRL = B_PWM_MASTER1_START; }
#define LINAA_TIMER_STOP()          {IO_PWM_MASTER1_CTRL = B_PWM_MASTER1_STOP; }
#define LINAA_TIMER_PERIOD(p)       {IO_PWM_MASTER1_PER = p; }
#endif /* LIN_AA_RESOURCE */

/*                          +-------+
 *  LIN-IN []--------+------| 200mR |------+--------[] LIN-OUT
 *                   |      +-------+      |
 *                  +-+                   +-+
 *                  | |                   | |
 *                  +-+                   +-+
 *                   |                     |
 *            +------o---------------------o----+
 *            | IC   |      +-------+      |    |
 *            |      +------| 900mR |------+    |
 *            |             +-------+           |
 */
#define LIN_SET_XKEY()              {IO_PORT_LIN_XKEY_S = C_LIN_KEY; }          /*!< Set LIN-AA Key (enable access) */
#define LIN_CLR_XKEY()              {IO_PORT_LIN_XKEY_S = 0x0000U; }            /*!< Clear LIN-AA Key (disable access) */

/*!*************************************************************************** *
 * LINAA_RESET
 * \brief   Initialise LIN AA Interface
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Initialise LIN-AA block for LIN-AA sequence
 * *************************************************************************** *
 * - Call Hierarchy: ml_InitAutoAddressing()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static inline void LINAA_RESET(void)
{
    /* Initialise for LIN-AA */
#if (LIN_AA_EXT_VOLTAGE != FALSE)
    IO_PORT_LINAA1 = B_PORT_LINAA1_LINAA_EN |
                     B_PORT_LINAA1_LINAA_5V_ENABLE;
#else  /* (LIN_AA_EXT_VOLTAGE != FALSE) */
    IO_PORT_LINAA1 = B_PORT_LINAA1_LINAA_EN;
#endif /* (LIN_AA_EXT_VOLTAGE != FALSE) */
    IO_PORT_LINAA2 = B_PORT_LINAA2_LCD_DIS_LINAA;
} /* End of LINAA_RESET() */

/*!*************************************************************************** *
 * LINAA_STOP
 * \brief   Stop/abort LIN AA Interface
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Stop/abort LIN-AA block for LIN-AA sequence
 * *************************************************************************** *
 * - Call Hierarchy: mlu_AutoAddresingStep()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static inline void LINAA_STOP(void)
{
    /* Stop LIN-AA */
#if (LIN_AA_EXT_VOLTAGE != FALSE)
    IO_PORT_LINAA1 = B_PORT_LINAA1_LINAA_5V_ENABLE;
#else  /* (LIN_AA_EXT_VOLTAGE != FALSE) */
    IO_PORT_LINAA1 = 0U;
#endif /* (LIN_AA_EXT_VOLTAGE != FALSE) */
    IO_PORT_LINAA2 = B_PORT_LINAA2_LCD_DIS_LINAA;
} /* End of LINAA_STOP() */

/*!*************************************************************************** *
 * LINAA_InitMeasurement
 * \brief   Setup LIN-AA Bus-shunt current measurement without current source
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16CMSuppGain: 4-bit variable Gain2 setting
 *                              5-bit Common mode suppression adjustments
 * \return  -
 * *************************************************************************** *
 * \details Setup for LIN Bus-shunt current measurement
 * *************************************************************************** *
 * - Call Hierarchy: mlu_AutoAddresingStep()
 * - Cyclomatic Complexity: 0/1+1
 * - Nesting: 0/1
 * - Function calling: 0
 * *************************************************************************** */
static inline void LINAA_InitMeasurement(uint16_t u16CMSuppGain)
{
    /* Offset measurement */
    IO_PORT_LINAA1 = (B_PORT_LINAA1_LINAA_EN |                                  /* Enable OpAmp of the LINAA amplifier */
                      B_PORT_LINAA1_LINAA_RST1 |                                /* Reset first amplifier gain */
                      B_PORT_LINAA1_LINAA_RST2 |                                /* Reset second variable gain amplifier */
#if (LIN_AA_EXT_VOLTAGE != FALSE)
                      B_PORT_LINAA1_LINAA_5V_ENABLE |
#endif /* (LIN_AA_EXT_VOLTAGE != FALSE) */
                      (u16CMSuppGain & (M_PORT_LINAA1_LINAA_DIV | M_PORT_LINAA1_LINAA_GAIN)));  /* Common mode suppression adjustments bits & Gain control bits of the variable gain amp */
#if defined (__MLX81350__)
    if ( (IO_PORT_MISC_OUT & B_PORT_MISC_OUT_SWITCH_VDDA_TO_5V) != 0U)
    {
        IO_PORT_LINAA2 = B_PORT_LINAA2_LINAA_SELREF_VDDA;                       /* VDDA is set to 5.0V (MMP240527-1) */
    }
#endif /* defined (__MLX81350__) */
} /* End of LINAA_InitMeasurement() */

/*!*************************************************************************** *
 * LINAA_SetCurrentSource
 * \brief   Setup LIN-AA Bus-shunt current measurement with current source
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16CurrentLevel:
 *     Others:   [8:6]: 3-bit current-selection + [5:0]: 6-bit current selection trim
 *     MLX81350: [7:5]: 3-bit current-selection + [4:0]: 5-bit current selection trim
 * \return  -
 * *************************************************************************** *
 * \details Setup for LIN Bus-shunt current measurement and current source
 *          MLX81350: Selection & Trim in single PORT: IO_PORT_LINAA2
 *          Others: Selection in PORT: IO_PORT_LINAA2, Trim in PORT: IO_TRIM_MISC
 * *************************************************************************** *
 * - Call Hierarchy: mlu_AutoAddresingStep()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static INLINE void LINAA_SetCurrentSource(uint16_t u16CurrentLevel)
{
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
    uint16_t u16LinAA_Selection = ((u16CurrentLevel >> 6) & M_PORT_LINAA2_LCD_SEL_LINAA);  /* EE-bit 8:6 --> IO-bit 2:0 */
    uint16_t u16LinAA_SelTrim = (u16CurrentLevel & M_TRIM_MISC_TRIM_LCD_LINAA);  /* EE-bit 5:0 --> IO-bit 5:0 */
#elif defined (__MLX81340__)
    uint16_t u16LinAA_Selection = ((u16CurrentLevel >> 6) & M_PORT_LINAA2_LCD_SEL_LINAA);  /* EE-bit 8:6 --> IO-bit 2:0 */
    uint16_t u16LinAA_SelTrim = ((u16CurrentLevel << 8) & M_TRIM_MISC_TRIM_LCD_LINAA);  /* EE-bit 5:0 --> IO-bit 13:8 */
#elif defined (__MLX81350__)
    uint16_t u16LinAA_SelectionAndTrim = (u16CurrentLevel & (M_PORT_LINAA2_LCD_SEL_LINAA | M_PORT_LINAA2_TRIM_LCD_LINAA));  /* EE-bit 7:0 --> IO-bit 7:0 (MMP240527-1) */
#endif
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81340__)
    /* Enable and set current level */
    IO_PORT_LINAA2 = (IO_PORT_LINAA2 & ~(B_PORT_LINAA2_LCD_DIS_LINAA | M_PORT_LINAA2_LCD_SEL_LINAA)) |
                     (u16LinAA_Selection |                                      /* LIN-AA Current selection */
                      B_PORT_LINAA2_LCD_ON_LINAA);                              /* Turn-in LCDIAG_ON */
    IO_TRIM_MISC = (IO_TRIM_MISC & ~M_TRIM_MISC_TRIM_LCD_LINAA) | u16LinAA_SelTrim; /* LIN-AA Selection Trim */
#elif defined (__MLX81350__)
    /* Enable and set current level (MMP240527-1) */
    IO_PORT_LINAA2 = (IO_PORT_LINAA2 & ~(B_PORT_LINAA2_LCD_DIS_LINAA | M_PORT_LINAA2_LCD_SEL_LINAA | M_PORT_LINAA2_TRIM_LCD_LINAA)) |
                     (u16LinAA_SelectionAndTrim |                               /* LIN-AA Current selection */
                      B_PORT_LINAA2_LCD_ON_LINAA);                              /* Turn-in LCDIAG_ON */
#endif
    IO_PORT_LINAA1 |= B_PORT_LINAA1_LINAA_CDOUTEN;                              /* Switch of the monitor current for LINAA */
} /* End of LINAA_SetCurrentSource() */

/*!*************************************************************************** *
 * AutoAddressingReadADCResult
 * \brief   Setup LIN-AA Bus-shunt current measurement with current source
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  LIN Bus-shunt current [0.1 mA]
 * *************************************************************************** *
 * \details Calculate the LIN-shunt current in 0.1mA units.
 * *************************************************************************** *
 * - Call Hierarchy: mlu_AutoAddresingStep()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
static void AutoAddressingReadADCResult(void)
{
    LINAA_TIMER_STOP();                                                         /* Stop LIN-AA Timer */

    asm ("mov x, #_LinAutoAddressing");
#if (_SUPPORT_LINAA_VS_COMPENSATION != FALSE)
    asm ("mov a, [x++]");                                                       /* Supply (VS) Compensation (MMP240214-1) */
    asm ("mov _l_u16AutoAddressingVS, a");                                      /* LIN-AA VS */
#endif /* (_SUPPORT_LINAA_VS_COMPENSATION != FALSE) */
    asm ("mov a, [x++]");                                                       /* i16Sum =  *(int16_t *) &LinAutoAddressing.Result_LinShunt1_CommonMode */
    asm ("add a, [x++]");                                                       /* i16Sum += *(int16_t *) &LinAutoAddressing.Result_LinShunt2_CommonMode */
    asm ("lsr a, #1");                                                          /* l_u16AutoAddressingCM = (i16Sum >> 1) */
    asm ("mov _l_u16AutoAddressingCM, a");                                      /* LIN-AA Common-Mode */
    asm ("mov a, [x++]");                                                       /* i16Sum =  *(int16_t *) &LinAutoAddressing.Result_LinShunt1_mode1 */
    asm ("add a, [x++]");                                                       /* i16Sum += *(int16_t *) &LinAutoAddressing.Result_LinShunt2_mode1 */
    asm ("add a, [x++]");                                                       /* i16Sum += *(int16_t *) &LinAutoAddressing.Result_LinShunt3_mode1 */
    asm ("add a, [x++]");                                                       /* i16Sum += *(int16_t *) &LinAutoAddressing.Result_LinShunt4_mode1 */
    asm ("add a, [x++]");                                                       /* i16Sum += *(int16_t *) &LinAutoAddressing.Result_LinShunt5_mode1 */
    asm ("add a, [x++]");                                                       /* i16Sum += *(int16_t *) &LinAutoAddressing.Result_LinShunt6_mode1 */
    asm ("add a, [x++]");                                                       /* i16Sum += *(int16_t *) &LinAutoAddressing.Result_LinShunt7_mode1 */
    asm ("add a, [x++]");                                                       /* i16Sum += *(int16_t *) &LinAutoAddressing.Result_LinShunt8_mode1 */
    asm ("add a, [x++]");                                                       /* i16Sum += *(int16_t *) &LinAutoAddressing.Result_LinShunt9_mode1 */
    asm ("add a, [x++]");                                                       /* i16Sum += *(int16_t *) &LinAutoAddressing.Result_LinShunt10_mode1 */
    asm ("add a, [x++]");                                                       /* i16Sum += *(int16_t *) &LinAutoAddressing.Result_LinShunt11_mode1 */
    asm ("add a, [x++]");                                                       /* i16Sum += *(int16_t *) &LinAutoAddressing.Result_LinShunt12_mode1 */
    asm ("add a, [x++]");                                                       /* i16Sum += *(int16_t *) &LinAutoAddressing.Result_LinShunt13_mode1 */
    asm ("add a, [x++]");                                                       /* i16Sum += *(int16_t *) &LinAutoAddressing.Result_LinShunt14_mode1 */
    asm ("add a, [x++]");                                                       /* i16Sum += *(int16_t *) &LinAutoAddressing.Result_LinShunt15_mode1 */
    asm ("add a, [x++]");                                                       /* i16Sum += *(int16_t *) &LinAutoAddressing.Result_LinShunt16_mode1 */
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
    asm ("lsr a, #1");                                                          /* DM 3-bits more (8x) */
#endif /* defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) */
    asm ("mov _l_u16AutoAddressingDM, a");                                      /* LIN-AA Differential Mode (x8) */
} /* End of AutoAddressingReadADCResult() */

/*!*************************************************************************** *
 * mlu_AutoAddressingStep
 * \brief   LIN Auto Addressing Bus shunt method measurement steps
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] StepNumber
 *                1: Setup LIN-AA for current-offset measurement
 *                2: Perform current-offset measurement
 *                3: Setup LIN-AA for pre-selection measurement
 *                4: Perform pre-selection measurement
 *                5: Setup LIN-AA for final-selection measurement
 *                6: Perform final-selection measurement
 *                7: Restore LIN interface
 * \return  -
 * *************************************************************************** *
 * \details Perform LIN-Auto Addressing via LIN Bus-shunt method
 * NOTE: As this function is called by mlx4 callback, it is assumed that no Non Volatile Memory
 *       Write is busy.
 * *************************************************************************** *
 * - Call Hierarchy: LIN_Init()
 * - Cyclomatic Complexity: 21+1
 * - Nesting: 8
 * - Function calling: 3 (LINAA_InitMeasurement(),
 *                        AutoAddressingReadADCResult(),
 *                        LINAA_SetCurrentSource())
 * *************************************************************************** */
void mlu_AutoAddressingStep(uint8_t StepNumber)
{
#if (_DEBUG_LINAA != FALSE)
    DEBUG_SET_IO_A();
#endif /* (_DEBUG_LINAA != FALSE) */
#if (LIN_AA_BSM_SNPD_R1p0 == FALSE)
    l_u8AutoAddressingPulse = StepNumber;
#else  /* (LIN_AA_BSM_SNPD_R1p0 == FALSE) */

    if (StepNumber == 1U)
    {
        uint8_t u8AutoAddressingFlags;

        /* *** Step 1: Setup LIN-AA for current-offset measurement *** */
        /* configure Auto addressing hardware cell on all slaves*/
#if (_SUPPORT_APP_USER_MODE != FALSE)
        ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        IO_PORT_LIN_XCFG_S |= B_PORT_LIN_XCFG_DISTERM;                          /* Disable LIN pull-up resistor */
#if (_SUPPORT_APP_USER_MODE != FALSE)
        EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */

        u8AutoAddressingFlags = l_u8AutoAddressingFlags;
        u8AutoAddressingFlags &= (uint8_t) ~(SLAVEFINALSTEP |                   /* Not the final step */
                                             LASTSLAVE |                        /* LASTSLAVE flag is cleared */
                                             WAITINGFORBREAK);                  /* Break occurred (MMP180801-2) */
        l_u8AutoAddressingFlags = u8AutoAddressingFlags;

        if ( (u8AutoAddressingFlags & SLAVEADDRESSED) == 0U)
        {
            /* Initialise offset measurement */
#if (LIN_AA_NV != FALSE)
#if defined (__MLX81332A01__)
            LINAA_InitMeasurement(CalibrationParams.u8APP_TRIM17_SC_GAIN_TRIM);
#else
            LINAA_InitMeasurement( ((CalibrationParams.u8APP_TRIM05_LINAA_TRIM & 0x1FU) << 4) |
                                   (CalibrationParams.u8APP_TRIM17_SC_GAIN & 0x0FU));
#endif
#else  /* (LIN_AA_NV != FALSE) */
            LINAA_InitMeasurement(C_LINAA_SC_GAIN);
#endif /* (LIN_AA_NV != FALSE) */

            /* Configure LIN-AA Timer to be used for ADC LIN shunt current measurement. */
            LINAA_TIMER_SETUP();                                                /* Setup LIN-AA Timer: Divider = 1, Mode = 0 (Timer Mode) */
            LINAA_TIMER_PERIOD(l_u16LinAATimerPeriod);

            /* Set AdcCtrl to start CM, continuous */

#if ((LIN_AA_INFO != FALSE) && (LIN_AA_SCREENTEST != FALSE))
            pSNPD_Data = &l_aSNPD_Data[l_u8SNPD_CycleCount];
#endif /* ((LIN_AA_INFO != FALSE) && (LIN_AA_SCREENTEST != FALSE)) */
            IO_ADC_SBASE = (uint16_t)&l_au16AdcSource[0];                       /* Switch ADC input source to LIN Shunt */
        }
#if (LIN_AA_LINFRAME_TIMEOUT != FALSE)
        l_u16LinFrameTimeOut = l_u16LinFrameMaxTime;
#endif /* (LIN_AA_LINFRAME_TIMEOUT != FALSE) */
    }
    else if ( (l_u8AutoAddressingFlags & (SLAVEADDRESSED | WAITINGFORBREAK)) == 0U)  /* MMP180801-2 */
    {
        if (StepNumber == 2U)
        {
            /* *** Step 2: Perform current-offset measurement *** */
            if (l_u16SlowBaudrateAdjustment != 0U)
            {
                DELAY(l_u16SlowBaudrateAdjustment);  /*lint !e522 */
            }
            DELAY(C_DELAY_5US);                                                 /* 5 us */
            IO_PORT_LINAA1 &= ~B_PORT_LINAA1_LINAA_RST1;                        /* Deactivate Reset first amplifier gain */
            DELAY(C_DELAY_10US);                                                /* 10 us */
            IO_PORT_LINAA1 &= ~B_PORT_LINAA1_LINAA_RST2;                        /* Deactivate Reset second variable gain amplifier */

            /* Test-module: No ADC-measurement, only current source */
            HAL_ADC_Start();                                                    /* Start ADC */
            LINAA_TIMER_START();                                                /* Start LIN-shunt current measurement (LIN-AA Timer) */
#if (_DEBUG_LINAA != FALSE)
            DEBUG_CLR_IO_B();
#endif /* (_DEBUG_LINAA != FALSE) */
            /* Clear variables */
            l_u16AutoAddressingCM = 0U;
            l_u16AutoAddressingDM = 0U;
            while ( (IO_ADC_CTRL & B_ADC_STOP) == 0U) {}
#if (_DEBUG_LINAA != FALSE)
            DEBUG_SET_IO_B();
#endif /* (_DEBUG_LINAA != FALSE) */
            AutoAddressingReadADCResult();                                      /* Calculate LIN shunt offset current */
        }
        else if (StepNumber == 3U)
        {
            /* *** Step 3: Setup LIN-AA for pre-selection measurement *** */
            if (l_u16SlowBaudrateAdjustment != 0U)
            {
                DELAY(l_u16SlowBaudrateAdjustment);  /*lint !e522 */            /* MMP200527-1 */
            }

            /* Setup 2nd current measurement; Either enable pull-up or enable current-source of 0.45mA */
#if (LIN_AA_CURRSRC_ONLY != FALSE)
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
            if (CalibrationParams.u8APP_TRIM03_CalibVersion >= C_NV_MLX_VER_3)
#elif defined (__MLX81340__) || defined (__MLX81350__)
            if (CalibrationParams.u8APP_TRIM03_CalibVersion >= C_NV_MLX_VER_1)
#endif
            {
                /* Turn on current source value of 0.45mA */
                LINAA_SetCurrentSource(CalibrationParams.u8APP_TRIM16_IAA_Trim045mA);  /* MLX81350: 7:5: LCD_SEL_LINAA, 4:0: TRIM_LCD_LINAA; Others: 8:6: LCD_SEL_LINAA, 5:0: TRIM_LCD_LINAA */
            }
            else
#endif /* (LIN_AA_CURRSRC_ONLY != FALSE) */
            {
                /* switch on LIN Pull up resistor */
#if (_SUPPORT_APP_USER_MODE != FALSE)
                ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
                IO_PORT_LIN_XCFG_S &= ~B_PORT_LIN_XCFG_DISTERM;                 /* Enable LIN pull-up resistor (LIN 1.3/Cooling) */
#if (_SUPPORT_APP_USER_MODE != FALSE)
                EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
            }
            IO_ADC_SBASE = (uint16_t)&l_au16AdcSource[0];                       /* Switch ADC input source to LIN Shunt */
        }
        else if (StepNumber == 4U)
        {
            /* *** Step 4: Perform pre-selection measurement *** */

            /* Test-module: No ADC-measurement, only current source */
            HAL_ADC_Start();                                                    /* Start ADC */
            LINAA_TIMER_START();                                                /* Start Timer as ADC HW-trigger */
#if (_DEBUG_LINAA != FALSE)
            DEBUG_CLR_IO_B();
#endif /* (_DEBUG_LINAA != FALSE) */
            l_u16AutoAddressingCM_0 = l_u16AutoAddressingCM;
            l_u16AutoAddressingDM_0 = l_u16AutoAddressingDM;
#if ((LIN_AA_INFO != FALSE) && (LIN_AA_SCREENTEST != FALSE))
            pSNPD_Data->u16CM_1 = l_u16AutoAddressingCM_0;
            pSNPD_Data->u16DM_1 = l_u16AutoAddressingDM_0;
#endif /* ((LIN_AA_INFO != FALSE) && (LIN_AA_SCREENTEST != FALSE)) */
            /* Clear variables */
            l_u16AutoAddressingCM = 0U;
            l_u16AutoAddressingDM = 0U;
            while ( (IO_ADC_CTRL & B_ADC_STOP) == 0U) {}
#if (_DEBUG_LINAA != FALSE)
            DEBUG_SET_IO_B();
#endif /* (_DEBUG_LINAA != FALSE) */
            AutoAddressingReadADCResult();                                      /* Calculate LIN shunt pre-selection current */
#if (PROJECT_ID == 0x0503) || (PROJECT_ID == 0x050B) ||                            /* MLX81330A  SO8|QFN */ \
    (PROJECT_ID == 0x050F) || (PROJECT_ID == 0x0510) || (PROJECT_ID == 0x0511) || /* MLX81330B1 SO8-001|QFN|SO8-101 */ \
    (PROJECT_ID == 0x0514) || (PROJECT_ID == 0x0515) || (PROJECT_ID == 0x0516) || /* MLX81330B2 SO8-002|QFN|SO8-102 */ \
    (PROJECT_ID == 0x0701) || (PROJECT_ID == 0x0705) ||                         /* MLX81332A QFN|SO8-001 */ \
    (PROJECT_ID == 0x070A) || (PROJECT_ID == 0x070B) || (PROJECT_ID == 0x070C) || /* MLX81332B1 QFN|SO8-001|SO8-101 */ \
    (PROJECT_ID == 0x070F) || (PROJECT_ID == 0x0710) || (PROJECT_ID == 0x0711) || /* MLX81332B2 QFN|SO8-002|SO8-102 */ \
    (PROJECT_ID == 0x0901)                                                      /* MLX81334AA QFN */
            /* IC with internal LIN Bus Shunt (MMP200408-1) */
            {
#if defined (C_LIN_AA_SAADMCM_SDIV)
                int16_t i16DM = ((int16_t)(l_u16AutoAddressingDM_0 - l_u16AutoAddressingDM)) +
                                (int16_t) (p_MulI32_I16byI16( (int16_t)(l_u16AutoAddressingCM - l_u16AutoAddressingCM_0),
                                                           C_LINAA_SAADMCM) >> C_LIN_AA_SAADMCM_SDIV);
#else
                int16_t i16DM = ((int16_t)(l_u16AutoAddressingDM_0 - l_u16AutoAddressingDM)) +
                                p_MulDivI16_I16byI16byI16( (int16_t)(l_u16AutoAddressingCM - l_u16AutoAddressingCM_0),
                                                           C_LINAA_SAADMCM,
                                                           C_LIN_AA_SAADMCM_DIVISOR);
#endif
#if defined (C_LIN_AA_CURR_SDIV)
                l_i16LINAA_PreSelCurr =
                    (int16_t) (p_MulI32_I16byI16(i16DM, C_LINAA_GDM) >> C_LIN_AA_CURR_SDIV);
#else
                l_i16LINAA_PreSelCurr = p_MulDivI16_I16byI16byI16(i16DM,
                                                                  C_LINAA_GDM,  /*lint !e501 */
                                                                  C_LIN_AA_CURR_DIVISOR);  /* 10uA */
#endif
#if (LINPROT == LIN2X_HVAC52)
                if (C_LINAA_ESHUNT != 0U)
                {
                    l_i16LINAA_PreSelCurr = p_MulDivI16_I16byI16byI16(l_i16LINAA_PreSelCurr,
                                                                      (C_LINAA_ISHUNT + C_LINAA_ESHUNT),
                                                                      C_LINAA_ESHUNT);  /* 10uA */
#if (_SUPPORT_LINAA_VS_COMPENSATION != FALSE)                                   /* (MMP240214-1) */
                    uint16_t u16AutoAddressingVS =
                        (uint16_t)((p_MulU32_U16byU16(l_u16AutoAddressingVS,
                                    Get_HighVoltGain()) +
                                    (C_VOLTGAIN_DIV / 2)) / C_VOLTGAIN_DIV);    /* [10mV] */
                    /* VS compensation is 0.067 [mA/V] */
                    int16_t i16VsComp =                                         /* [10uA] */
                        (int16_t)(p_MulI32_I16byI16((int16_t)(1200U - u16AutoAddressingVS),  /* [10mV] */
                                                  67) >> 10);                   /* [1uA/V], divided by ~[mV/V] */
                    l_i16LINAA_PreSelCurr += i16VsComp;
#endif /* (_SUPPORT_LINAA_VS_COMPENSATION != FALSE) */
                }
#endif /* (LINPROT == LIN2X_HVAC52) */
            }
#elif (PROJECT_ID == 0x0502) || (PROJECT_ID == 0x050A)
            /* IC without internal LIN Bus Shunt */
            /* I = U / R
             * I[10uA] = (U[mV] / R[mR]) * 100000 [A/10uA]
             * I[10uA] = (((DM[LSB] / 8) * (C_LINAA_GDM/4096[LSB/mV] / 50mV)) / C_LINAA_ESHUNT) * 100000
             * I[10uA] = DM[LSB] * C_LINAA_GDM * 100000 / (4096 * 50 * 8 * C_LINAA_ESHUNT)
             * 100000 / (4096 * 50 * 8) = 125/2048 = 1/16.384 = ~1/16
             * I[10uA] = (DM[LSB] * C_LINAA_GDM) / (C_LINAA_ESHUNT * 16)
             */
/*            int16_t i16Divisor = (int16_t) (p_MulU32_U16byU16( C_LINAA_GDM, C_LINAA_ESHUNT) >> 5); */
/*            l_i16LINAA_PreSelCurr = p_MulDivI16_I16byI16byI16( (int16_t) (l_u16AutoAddressingDM_0 - l_u16AutoAddressingDM), */
/*                                                               32000, i16Divisor); */
            int16_t i16Divisor = p_MulDivI16_I16byI16byI16(C_LINAA_ESHUNT, 4096, 250);
            l_i16LINAA_PreSelCurr = p_MulDivI16_I16byI16byI16(
                (int16_t)(l_u16AutoAddressingDM_0 - l_u16AutoAddressingDM),
                C_LINAA_GDM,
                i16Divisor);
#elif (PROJECT_ID == 0x0A01) || (PROJECT_ID == 0x0A03) || (PROJECT_ID == 0x0A05)  /* MLX81340A|B QFN32 */
            /* IC with internal LIN Bus Shunt */
            {
#if defined (C_LIN_AA_SAADMCM_SDIV)
                int16_t i16DM = ((int16_t)(l_u16AutoAddressingDM - l_u16AutoAddressingDM_0)) +
                                (int16_t) (p_MulI32_I16byI16( (int16_t)(l_u16AutoAddressingCM - l_u16AutoAddressingCM_0),
                                                           C_LINAA_SAADMCM) >> C_LIN_AA_SAADMCM_SDIV);
#else
                int16_t i16DM = ((int16_t)(l_u16AutoAddressingDM - l_u16AutoAddressingDM_0)) +
                                p_MulDivI16_I16byI16byI16( (int16_t)(l_u16AutoAddressingCM - l_u16AutoAddressingCM_0),
                                                           C_LINAA_SAADMCM,
                                                           C_LIN_AA_SAADMCM_DIVISOR);
#endif
#if defined (C_LIN_AA_CURR_SDIV)
                l_i16LINAA_PreSelCurr =
                    (int16_t) (p_MulI32_I16byI16(i16DM, C_LINAA_GDM) >> C_LIN_AA_CURR_SDIV);
#else
                l_i16LINAA_PreSelCurr = p_MulDivI16_I16byI16byI16(i16DM,
                                                                  C_LINAA_GDM,  /*lint !e501 */
                                                                  C_LIN_AA_CURR_DIVISOR);
#endif
            }
#elif (PROJECT_ID == 0x2601) || (PROJECT_ID == 0x2602) || (PROJECT_ID == 0x2604)  /* MLX81350AA S08-001, S08-101, QFN24 */
            {
#if defined (C_LIN_AA_SAADMCM_SDIV)
                int16_t i16DM = ((int16_t)(l_u16AutoAddressingDM - l_u16AutoAddressingDM_0)) +
                                (int16_t) (p_MulI32_I16byI16( (int16_t)(l_u16AutoAddressingCM - l_u16AutoAddressingCM_0),
                                                           C_LINAA_SAADMCM) >> C_LIN_AA_SAADMCM_SDIV);
#else
                int16_t i16DM = ((int16_t)(l_u16AutoAddressingDM - l_u16AutoAddressingDM_0)) +
                                p_MulDivI16_I16byI16byI16( (int16_t)(l_u16AutoAddressingCM - l_u16AutoAddressingCM_0),
                                                           C_LINAA_SAADMCM,
                                                           C_LIN_AA_SAADMCM_DIVISOR);
#endif
#if defined (C_LIN_AA_CURR_SDIV)
                l_i16LINAA_FinalSelCurr =
                    (int16_t) (p_MulI32_I16byI16(i16DM, C_LINAA_GDM) >> C_LIN_AA_CURR_SDIV);
#else
                l_i16LINAA_FinalSelCurr = p_MulDivI16_I16byI16byI16(i16DM,
                                                                    C_LINAA_GDM,  /*lint !e501 */
                                                                    C_LIN_AA_CURR_DIVISOR);
#endif
#if (LINPROT == LIN2X_HVAC52)
                if (C_LINAA_ESHUNT != 0U)
                {
                    l_i16LINAA_PreSelCurr = p_MulDivI16_I16byI16byI16(l_i16LINAA_PreSelCurr,
                                                                      (C_LINAA_ISHUNT + C_LINAA_ESHUNT),
                                                                      C_LINAA_ESHUNT);  /* 10uA */
#if (_SUPPORT_LINAA_VS_COMPENSATION != FALSE)                                   /* (MMP240214-1) */
                    uint16_t u16AutoAddressingVS =
                        (uint16_t)((p_MulU32_U16byU16(l_u16AutoAddressingVS,
                                    Get_HighVoltGain()) +
                                    (C_VOLTGAIN_DIV / 2)) / C_VOLTGAIN_DIV);    /* [10mV] */
                    /* VS compensation is 0.067 [mA/V] */
                    int16_t i16VsComp =                                         /* [10uA] */
                        (int16_t)(p_MulI32_I16byI16((int16_t)(1200U - u16AutoAddressingVS),  /* [10mV] */
                                                  67) >> 10);                   /* [1uA/V], divided by ~[mV/V] */
                    l_i16LINAA_PreSelCurr += i16VsComp;
#endif /* (_SUPPORT_LINAA_VS_COMPENSATION != FALSE) */
                }
#endif /* (LINPROT == LIN2X_HVAC52) */
            }
#else
#error "LIN-AA fails to work"
#endif /* (PROJECT_ID) */
        }
        else if (StepNumber == 5U)
        {
            /* *** Step 5: Pre-select: Check I2-I1 < threshold1 then Setup for current-source measurement, otherwise "quit" (wait) *** */
            if (l_u16SlowBaudrateAdjustment != 0U)
            {
                DELAY(l_u16SlowBaudrateAdjustment);  /*lint !e522 */
            }

            if (l_i16LINAA_PreSelCurr > C_LIN13AA_dI_1)                         /* Calculate difference Ishunt2 - Ishunt1 */
            {
                /* If Ishunt2 - Ishunt1 > It_1, disable standard pull up and goto wait state */
                LINAA_STOP();
#if (_SUPPORT_APP_USER_MODE != FALSE)
                ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
                IO_PORT_LIN_XCFG_S |= B_PORT_LIN_XCFG_DISTERM;                  /* Disable LIN pull-up resistor */
#if (_SUPPORT_APP_USER_MODE != FALSE)
                EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */

#if ((LIN_AA_INFO != FALSE) && (LIN_AA_SCREENTEST != FALSE))
                pSNPD_Data->u16CM_2 = l_u16AutoAddressingCM;
                pSNPD_Data->u16DM_2 = l_u16AutoAddressingDM;
#endif /* ((LIN_AA_INFO != FALSE) && (LIN_AA_SCREENTEST != FALSE)) */
            }
            else
            {
                /* Setup 3th current measurement; Either enable pull-up + current-source of 2.05mA or enable current-source of 2.40mA */
#if (LIN_AA_CURRSRC_ONLY != FALSE)
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                if (CalibrationParams.u8APP_TRIM03_CalibVersion >= C_NV_MLX_VER_3)
#elif defined (__MLX81340__) || defined (__MLX81350__)
                if (CalibrationParams.u8APP_TRIM03_CalibVersion >= C_NV_MLX_VER_1)
#endif
                {
                    /* Turn on current source value of 2.40mA */
#if defined (__MLX81350__)
                    LINAA_SetCurrentSource(CalibrationParams.u8APP_TRIM17_IAA_Trim240mA);  /* 7:5: LCD_SEL_LINAA, 4:0: TRIM_LCD_LINAA (MMP240527-1) */
#else  /* defined (__MLX81350__) */
                    LINAA_SetCurrentSource(0x100U | CalibrationParams.u8APP_TRIM17_IAA_Trim240mA); /* 8:6: LCD_SEL_LINAA, 5:0: TRIM_LCD_LINAA */
#endif /* defined (__MLX81350__) */
                }
                else
#endif /* (LIN_AA_CURRSRC_ONLY != FALSE) */
                {
                    /* Turn on current source value of 2.05mA */
#if (LIN_AA_NV != FALSE)
                    LINAA_SetCurrentSource(CalibrationParams.u8APP_TRIM16_IAA_Trim205mA);  /* MLX81350: 7:5: LCD_SEL_LINAA, 4:0: TRIM_LCD_LINAA; Others: 8:6: LCD_SEL_LINAA, 5:0: TRIM_LCD_LINAA */
#else  /* (LIN_AA_NV != FALSE) */
                    LINAA_SetCurrentSource(C_LINAA_CS_205);
#endif /* (LIN_AA_NV != FALSE) */
                }
                IO_ADC_SBASE = (uint16_t)&l_au16AdcSource[0];                   /* Switch ADC input source to LIN Shunt */
                l_u8AutoAddressingFlags |= SLAVEFINALSTEP;
            }
        }
        else if ( (StepNumber == 6U) && ((l_u8AutoAddressingFlags & SLAVEFINALSTEP) != 0U) )
        {
            /* *** Step 6: Perform current source measurement *** */
            if (l_u16SlowBaudrateAdjustment != 0U)
            {
                DELAY(l_u16SlowBaudrateAdjustment);  /*lint !e522 */
            }
            /* Test-module: No ADC-measurement, only current source */
            HAL_ADC_Start();                                                    /* Start ADC */
            LINAA_TIMER_START();                                                /* Start LIN-shunt current measurement (LIN-AA Timer) */
#if (_DEBUG_LINAA != FALSE)
            DEBUG_CLR_IO_B();
#endif /* (_DEBUG_LINAA != FALSE) */
#if ((LIN_AA_INFO != FALSE) && (LIN_AA_SCREENTEST != FALSE))
            pSNPD_Data->u16CM_2 = l_u16AutoAddressingCM;
            pSNPD_Data->u16DM_2 = l_u16AutoAddressingDM;
#endif /* ((LIN_AA_INFO != FALSE) && (LIN_AA_SCREENTEST != FALSE)) */
            /* Clear variables */
            l_u16AutoAddressingCM = 0U;
            l_u16AutoAddressingDM = 0U;
            while ( (IO_ADC_CTRL & B_ADC_STOP) == 0U) {}
#if (_DEBUG_LINAA != FALSE)
            DEBUG_SET_IO_B();
#endif /* (_DEBUG_LINAA != FALSE) */
            AutoAddressingReadADCResult();                                      /* Calculate LIN shunt final-selection current */
#if (PROJECT_ID == 0x0503) || (PROJECT_ID == 0x050B) ||                            /* MLX81330A  SO8|QFN */ \
    (PROJECT_ID == 0x050F) || (PROJECT_ID == 0x0510) || (PROJECT_ID == 0x0511) || /* MLX81330B1 SO8-001|QFN|SO8-101 */ \
    (PROJECT_ID == 0x0514) || (PROJECT_ID == 0x0515) || (PROJECT_ID == 0x0516) || /* MLX81330B2 SO8-002|QFN|SO8-102 */ \
    (PROJECT_ID == 0x0701) || (PROJECT_ID == 0x0705) ||                         /* MLX81332A QFN|SO8-001 */ \
    (PROJECT_ID == 0x070A) || (PROJECT_ID == 0x070B) || (PROJECT_ID == 0x070C) || /* MLX81332B1 QFN|SO8-001|SO8-101 */ \
    (PROJECT_ID == 0x070F) || (PROJECT_ID == 0x0710) || (PROJECT_ID == 0x0711) || /* MLX81332B2 QFN|SO8-002|SO8-102 */ \
    (PROJECT_ID == 0x0901)                                                      /* MLX81334AA QFN */
            /* IC with internal LIN Bus Shunt (MMP200408-1) */
            {
#if defined (C_LIN_AA_SAADMCM_SDIV)
                int16_t i16DM = ((int16_t)(l_u16AutoAddressingDM_0 - l_u16AutoAddressingDM)) +
                                (int16_t) (p_MulI32_I16byI16( (int16_t)(l_u16AutoAddressingCM - l_u16AutoAddressingCM_0),
                                                           C_LINAA_SAADMCM) >> C_LIN_AA_SAADMCM_SDIV);
#else
                int16_t i16DM = ((int16_t)(l_u16AutoAddressingDM_0 - l_u16AutoAddressingDM)) +
                                p_MulDivI16_I16byI16byI16( (int16_t)(l_u16AutoAddressingCM - l_u16AutoAddressingCM_0),
                                                           C_LINAA_SAADMCM,
                                                           C_LIN_AA_SAADMCM_DIVISOR);
#endif
#if defined (C_LIN_AA_CURR_SDIV)
                l_i16LINAA_FinalSelCurr =
                    (int16_t) (p_MulI32_I16byI16(i16DM, C_LINAA_GDM) >> C_LIN_AA_CURR_SDIV);
#else
                l_i16LINAA_FinalSelCurr = p_MulDivI16_I16byI16byI16(i16DM,
                                                                    C_LINAA_GDM,  /*lint !e501 */
                                                                    C_LIN_AA_CURR_DIVISOR);  /* [10 uA] */
#endif
#if (LINPROT == LIN2X_HVAC52)
                if (C_LINAA_ESHUNT != 0U)
                {
                    /* Bond-wire correction = 2.3mA * Rbond/((C_LINAA_ISHUNT - Rbond) + C_LINAA_ESHUNT) * 100 [10uA/mA] */
                    l_i16LINAA_FinalSelCurr += p_MulDivI16_I16byI16byI16(C_LINAA_RBOND,
                                                                         C_CURRENT_SOURCE,
                                                                         ((C_LINAA_ISHUNT - C_LINAA_RBOND) +
                                                                          C_LINAA_ESHUNT));  /* [10 uA] */
                    l_i16LINAA_FinalSelCurr = p_MulDivI16_I16byI16byI16(l_i16LINAA_FinalSelCurr,
                                                                        (C_LINAA_ISHUNT + C_LINAA_ESHUNT),
                                                                        C_LINAA_ESHUNT);  /* [10 uA] */
#if (_SUPPORT_LINAA_VS_COMPENSATION != FALSE)                                   /* (MMP240214-1) */
                    uint16_t u16AutoAddressingVS =
                        (uint16_t)((p_MulU32_U16byU16(l_u16AutoAddressingVS,
                                                      Get_HighVoltGain()) +
                                    (C_VOLTGAIN_DIV / 2)) / C_VOLTGAIN_DIV);    /* [10mV] */
                    /* VS compensation is 0.067 [mA/V] */
                    int16_t i16VsComp =                                         /* [10uA] */
                        (int16_t)(p_MulI32_I16byI16((int16_t)(1200U - u16AutoAddressingVS),  /* [10mV] */
                                                  67) >> 10);                   /* [1uA/V], divided by ~[mV/V] */
                    l_i16LINAA_FinalSelCurr += i16VsComp;
#endif /* (_SUPPORT_LINAA_VS_COMPENSATION != FALSE) */
                }
#endif /* (LINPROT == LIN2X_HVAC52) */
            }
#elif (PROJECT_ID == 0x0502) || (PROJECT_ID == 0x050A)
            /* IC without internal LIN Bus Shunt */
            /* I = U / R
             * I[10uA] = (U[mV] / R[mR]) / 100000 [A/10uA]
             * I[10uA] = (((DM[LSB] / 8) / (C_LINAA_GDM/4096[LSB/mV] * 50mV)) / C_LINAA_ESHUNT) / 100000
             * I[10uA] = DM[LSB] * ((4096 * 100000)/(50*8)) / (C_LINAA_GDM * C_LINAA_ESHUNT)
             * Divide by 32
             * I[10uA] = (DM[LSB] * 32000) / ((C_LINAA_GDM * C_LINAA_ESHUNT) / 32)
             */
            int16_t i16Divisor = p_MulDivI16_I16byI16byI16(C_LINAA_ESHUNT, 4096, 250);
            l_i16LINAA_FinalSelCurr =
                p_MulDivI16_I16byI16byI16( (int16_t)(l_u16AutoAddressingDM_0 - l_u16AutoAddressingDM),
                                           C_LINAA_GDM, i16Divisor);
#elif (PROJECT_ID == 0x0A01) || (PROJECT_ID == 0x0A03) || (PROJECT_ID == 0x0A05)  /* MLX81340A|B QFN32 */
            {
#if defined (C_LIN_AA_SAADMCM_SDIV)
                int16_t i16DM = ((int16_t)(l_u16AutoAddressingDM - l_u16AutoAddressingDM_0)) +
                                (int16_t) (p_MulI32_I16byI16( (int16_t)(l_u16AutoAddressingCM - l_u16AutoAddressingCM_0),
                                                           C_LINAA_SAADMCM) >> C_LIN_AA_SAADMCM_SDIV);
#else
                int16_t i16DM = ((int16_t)(l_u16AutoAddressingDM - l_u16AutoAddressingDM_0)) +
                                p_MulDivI16_I16byI16byI16( (int16_t)(l_u16AutoAddressingCM - l_u16AutoAddressingCM_0),
                                                           C_LINAA_SAADMCM,
                                                           C_LIN_AA_SAADMCM_DIVISOR);
#endif
#if defined (C_LIN_AA_CURR_SDIV)
                l_i16LINAA_FinalSelCurr =
                    (int16_t) (p_MulI32_I16byI16(i16DM, C_LINAA_GDM) >> C_LIN_AA_CURR_SDIV);
#else
                l_i16LINAA_FinalSelCurr = p_MulDivI16_I16byI16byI16(i16DM,
                                                                    C_LINAA_GDM,  /*lint !e501 */
                                                                    C_LIN_AA_CURR_DIVISOR);
#endif
            }
#elif (PROJECT_ID == 0x2601) || (PROJECT_ID == 0x2602) || (PROJECT_ID == 0x2604)  /* MLX81350AA S08-001, S08-101, QFN24 */
            {
#if defined (C_LIN_AA_SAADMCM_SDIV)
                int16_t i16DM = ((int16_t)(l_u16AutoAddressingDM - l_u16AutoAddressingDM_0)) +
                                (int16_t) (p_MulI32_I16byI16( (int16_t)(l_u16AutoAddressingCM - l_u16AutoAddressingCM_0),
                                                           C_LINAA_SAADMCM) >> C_LIN_AA_SAADMCM_SDIV);
#else
                int16_t i16DM = ((int16_t)(l_u16AutoAddressingDM - l_u16AutoAddressingDM_0)) +
                                p_MulDivI16_I16byI16byI16( (int16_t)(l_u16AutoAddressingCM - l_u16AutoAddressingCM_0),
                                                           C_LINAA_SAADMCM,
                                                           C_LIN_AA_SAADMCM_DIVISOR);
#endif
#if defined (C_LIN_AA_CURR_SDIV)
                l_i16LINAA_FinalSelCurr =
                    (int16_t) (p_MulI32_I16byI16(i16DM, C_LINAA_GDM) >> C_LIN_AA_CURR_SDIV);
#else
                l_i16LINAA_FinalSelCurr = p_MulDivI16_I16byI16byI16(i16DM,
                                                                    C_LINAA_GDM,  /*lint !e501 */
                                                                    C_LIN_AA_CURR_DIVISOR);
#endif
#if (LINPROT == LIN2X_HVAC52)
                if (C_LINAA_ESHUNT != 0U)
                {
                    /* Bond-wire correction = 2.3mA * Rbond/((C_LINAA_ISHUNT - Rbond) + C_LINAA_ESHUNT) * 100 [10uA/mA] */
                    l_i16LINAA_FinalSelCurr += p_MulDivI16_I16byI16byI16(C_LINAA_RBOND,
                                                                         C_CURRENT_SOURCE,
                                                                         ((C_LINAA_ISHUNT - C_LINAA_RBOND) +
                                                                          C_LINAA_ESHUNT));  /* [10 uA] */
                    l_i16LINAA_FinalSelCurr = p_MulDivI16_I16byI16byI16(l_i16LINAA_FinalSelCurr,
                                                                        (C_LINAA_ISHUNT + C_LINAA_ESHUNT),
                                                                        C_LINAA_ESHUNT);  /* [10 uA] */
#if (_SUPPORT_LINAA_VS_COMPENSATION != FALSE)                                   /* (MMP240214-1) */
                    uint16_t u16AutoAddressingVS =
                        (uint16_t)((p_MulU32_U16byU16(l_u16AutoAddressingVS,
                                                      Get_HighVoltGain()) +
                                    (C_VOLTGAIN_DIV / 2)) / C_VOLTGAIN_DIV);    /* [10mV] */
                    /* VS compensation is 0.067 [mA/V] */
                    int16_t i16VsComp =                                         /* [10uA] */
                        (int16_t)(p_MulI32_I16byI16((int16_t)(1200U - u16AutoAddressingVS),  /* [10mV] */
                                                  67) >> 10);                   /* [1uA/V], divided by ~[mV/V] */
                    l_i16LINAA_FinalSelCurr += i16VsComp;
#endif /* (_SUPPORT_LINAA_VS_COMPENSATION != FALSE) */
                }
#endif /* (LINPROT == LIN2X_HVAC52) */
            }
#else
#error "LIN-AA fails to work"
#endif /* (PROJECT_ID) */
        }
    }
    if (COLIN_LINstatus.event_overflow != 0U)
    {
        /* LIN Overflow; Abort LIN-AA */
        StepNumber = 8U;
        l_u8AutoAddressingFlags &= (uint8_t) ~(SLAVEFINALSTEP | LASTSLAVE);     /* LASTSLAVE flag is cleared */
        (void)ml_GetState(ML_CLR_LIN_CMD_OVERFLOW);
    }                                                                           /* MMP180801-1 */
    if ( (StepNumber == 0U) || (StepNumber >= 7U) )                             /* MMP180801-3 */
    {
        /* All slaves switch off current source and switch on pull up */
        LINAA_TIMER_RESET();                                                    /* Reset LIN-AA timer */
        l_u8AutoAddressingFlags |= (uint8_t)WAITINGFORBREAK;                    /* MMP180801-2 */

        if ( (StepNumber == 7U) && ((l_u8AutoAddressingFlags & SLAVEFINALSTEP) != 0U) )
        {
            /* *** Step 6 is only updated for slaves that did not skip any step before;
             *   Last-slave?: Check I3-I1 < threshold2, then last-slave otherwise not  *** */
            if (l_i16LINAA_FinalSelCurr < C_LIN13AA_dI_2)                       /* Check current difference between Ishunt3 and Ishunt1 */
            {
#if (_DEBUG_LINAA != FALSE)
                DEBUG_CLR_IO_B();
#endif /* (_DEBUG_LINAA != FALSE) */
                l_u8AutoAddressingFlags = (l_u8AutoAddressingFlags & (uint8_t) ~WAITINGFORBREAK) |
                                          (uint8_t)LASTSLAVE;                   /* This LIN-slave is the last slave: take the address and set SLAVEADDRESSED flag */
#if (_DEBUG_LINAA != FALSE)
                DEBUG_SET_IO_B();
#endif /* (_DEBUG_LINAA != FALSE) */
            }

#if ((LIN_AA_INFO != FALSE) && (LIN_AA_SCREENTEST != FALSE))
            pSNPD_Data->u16CM_3 = l_u16AutoAddressingCM;
            pSNPD_Data->u16DM_3 = (uint16_t)l_u16AutoAddressingDM;
#endif /* ((LIN_AA_INFO != FALSE) && (LIN_AA_SCREENTEST != FALSE)) */
        }

        if (l_u16SlowBaudrateAdjustment != 0U)
        {
            DELAY(l_u16SlowBaudrateAdjustment);  /*lint !e522 */
        }
        LINAA_STOP();
#if (_SUPPORT_APP_USER_MODE != FALSE)
        ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        IO_PORT_LIN_XCFG_S &= ~B_PORT_LIN_XCFG_DISTERM;                         /* Enable LIN pull-up resistor */
#if (_SUPPORT_APP_USER_MODE != FALSE)
        EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */

#if (LIN_AA_INFO != FALSE)
        if (StepNumber == 7U)
        {
            pSNPD_Data->byStepAndFlags = (7U << 3) | (l_u8AutoAddressingFlags & 0x87U);
            pSNPD_Data->byIshunt1 = (uint8_t)0U;
            pSNPD_Data->byIshunt2 = (uint8_t)((l_i16LINAA_PreSelCurr + 5U) / 10U);
            pSNPD_Data->byIshunt3 = (uint8_t)((l_i16LINAA_FinalSelCurr + 5U) / 10U);
            if (l_u8SNPD_CycleCount < (LIN_AA_INFO_SZ - 1U) )                   /* Don't increase index in case last AA-structure index */
            {
                l_u8SNPD_CycleCount++;
            }
        }
#endif /* (LIN_AA_INFO != FALSE) */
    }
#endif /* (LIN_AA_BSM_SNPD_R1p0 == FALSE) */
#if (_DEBUG_LINAA != FALSE)
    DEBUG_CLR_IO_A();
#endif /* (_DEBUG_LINAA != FALSE) */
} /* End of mlu_AutoAddressingStep() */

/*!*************************************************************************** *
 * ml_SetSlaveNotAddressed
 * \brief   Set Slave status to "Not Addressed"
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Resets the SLAVEADDRESSED flag. The actuator will try to
 *          acquire a new address next sync break.
 * *************************************************************************** *
 * - Call Hierarchy: DfrDiagAssignNAD() - C_SNPD_SUBFUNC_START
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
void ml_SetSlaveNotAddressed(void)
{
    l_u8AutoAddressingFlags &= ~SLAVEADDRESSED;                                 /* LIN-AA Start: Slave has lost it's address */
} /* End of ml_SetSlaveNotAddressed() */

/*!*************************************************************************** *
 * ml_SetSlaveAddressed
 * \brief   Set Slave status to "Addressed"
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Sets the SLAVEADDRESSED flag. The actuator will assume it is
 *          already addressed and not execute all auto-addressing steps.
 * *************************************************************************** *
 * - Call Hierarchy: DfrDiagAssignNAD() - C_SNPD_SUBFUNC_ADDR
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
void ml_SetSlaveAddressed(void)
{
    l_u8AutoAddressingFlags |= SLAVEADDRESSED;                                  /* LIN-AA Last-slave is set; Slave has received new address */
} /* End of ml_SetSlaveAddressed */

#if (LIN_AA_BSM_SNPD_R1p0 != FALSE)
/*!*************************************************************************** *
 * ml_InitAutoAddressing
 * \brief   Set LIN (and ADC) for LIN-AA
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Determine last LIN message Baudrate, to be used to set ADC sample rate
 *          and optional delays (only in case baudrate < 12000 Baud).
 *          Stop ADC and reconfigure to LIN-AA usage. Application (Motor) should
 *          have stopped before. From now on, no supply, temperature or other ADC
 *          channels can be measured.
 *          Initialise the LIN-AA ADC trigger source (CTimer or PWM-Module)
 *          Allow switching of LIN Pull-up resistance, therefore the LIN XKEY must
 *          be set (remove protection)
 *          Reset the LIN-AA IP block
 *          Set LIN-AA flag "Waiting for Break"
 *          Calculate the LIN-AA ADC sampling rate (l_u16LinAATimerPeriod)
 *          Enable MLX4 Auto-Addressing feature (extra call-back to implement/support
 *          LIN-AA: mlu_AutoAddressingStep())
 * *************************************************************************** *
 * - Call Hierarchy: DfrDiagAssignNAD() - C_SNPD_SUBFUNC_START
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 1
 * - Function calling: 3 (p_ml_GetBaudRate(), HAL_ADC_StopSafe(),
 *                        ml_AutoAddressingConfig())
 * *************************************************************************** */
void ml_InitAutoAddressing(void)
{
    /* Calculate the detected MLX4 LIN Baudrate, based on baudrate divider and pre-scaler */
    uint16_t u16LinBaudrate = p_ml_GetBaudRate(MLX4_FPLL);
    if ( (u16LinBaudrate != 0U) && (u16LinBaudrate < 12000U) )
    {
        l_u16SlowBaudrateAdjustment = C_SLOW_BAUDRATE_DELAY;                    /* Time at 9600 Baud */
    }
#if (LIN_AA_LINFRAME_TIMEOUT != FALSE)
    l_u16LinFrameMaxTime = p_DivU16_U32byU16(C_T_FRAME_MAX, u16LinBaudrate);
    l_u16LinFrameTimeOut = 0U;
#endif /* (LIN_AA_LINFRAME_TIMEOUT != FALSE) */

#if (PROJECT_ID == 0x050F) || (PROJECT_ID == 0x0510) || (PROJECT_ID == 0x0511) || /* MLX81330B1 SO8-001|QFN|SO8-101 */ \
    (PROJECT_ID == 0x070A) || (PROJECT_ID == 0x070B) || (PROJECT_ID == 0x070C)    /* MLX81332B1 QFN|SO8-001|SO8-101 */
    /* The LIN-AA need the motor driver 5.25V, and there-for the motor-driver must be enabled (MMP190214-1) */
    l_u16PreLinAA_MotorDrvState = (IO_PORT_DRV_OUT & B_PORT_DRV_OUT_ENABLE_DRVSUP);
    IO_PORT_DRV_OUT |= B_PORT_DRV_OUT_ENABLE_DRVSUP;
#endif /* (PROJECT_ID) */

    /* LIN-AA will claim the ADC and a Timer */
    /* Disable the ADC & IRQ */
    HAL_ADC_StopSafe();
    l_au16AdcSource[0] = (uint16_t)&LinAutoAddressing;
    memcpy( (void *)&l_au16AdcSource[1], (const void *)&SBASE_LIN[0], sizeof(SBASE_LIN));
    HAL_ADC_Setup(/*B_ADC_ADC_WIDTH |*/                                         /* Setup LIN-shunt current measurement */
            C_ADC_ASB_NEVER |                                                   /* Auto Standby: Never */
#if (_DEBUG_IO_ADC != FALSE)
            C_ADC_INT_SCHEME_EOC |                                              /* Message interrupt: End-of-Frame */
#else  /* (_DEBUG_IO_ADC != FALSE) */
            C_ADC_INT_SCHEME_NOINT |                                            /* Message interrupt: No */
#endif /* (_DEBUG_IO_ADC != FALSE) */
            B_ADC_SATURATE |                                                    /* Saturation: Enabled */
            B_ADC_NO_INTERLEAVE |                                               /* Interleave: No */
            C_ADC_SOC_SOURCE_HARD_CTRIG |                                       /* Start Of Conversion (SOC) triggered by: Hardware (HARD_CTRIG) */
            C_ADC_SOS_SOURCE_2ND_HARD_CTRIG);                                   /* Start Of Sequence (SOS) triggered: 2nd Hardware */
    HAL_ADC_ClearErrors();                                                      /* Prior to start, first clear any error flag and enable Triggers */
    l_u8AdcMode = (uint8_t)C_ADC_MODE_LINAA;

#if defined (__MLX81332B02__) || defined (__MLX81334__)
    /* Trim VDDA to Max level (Test) (MMP210712-1) */
    l_u16TrimBG = IO_TRIM_BG_BIAS;
    IO_TRIM_BG_BIAS = (l_u16TrimBG & ~(M_TRIM_BG_BIAS_PRE_TR_BIAS | M_TRIM_BG_BIAS_PRE_TR_BGA)) | ((0x20U << 8) | 8U);
    l_u16TrimVdd = IO_TRIM_VDD;
    IO_TRIM_VDD = l_u16TrimVdd & ~M_TRIM_VDD_PRE_TR_VDDA;
#endif /* defined (__MLX81332B02__) || defined (__MLX81334__) */

    /* Disable Timer2 & IRQ */
    LINAA_TIMER_RESET();                                                        /* Reset timer */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#if (LIN_AA_RESOURCE == LIN_AA_RESOURCE_CTIMER0) || (LIN_AA_RESOURCE == LIN_AA_RESOURCE_CTIMER1)
    LINAA_TIMER_IRQ_DIS();                                                      /* Disable LIN-AA Timer interrupt */
#endif /* (LIN_AA_RESOURCE == LIN_AA_RESOURCE_CTIMER0) || (LIN_AA_RESOURCE == LIN_AA_RESOURCE_CTIMER1) */

    LIN_SET_XKEY();                                                             /* Set XKEY to allow LIN Pull-up switching */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#if (LIN_AA_VDDA_5V != FALSE) && ((_SUPPORT_VDDA_5V == FALSE) && (_SUPPORT_SENSOR_VDDA_5V == FALSE))
#if (PROJECT_ID == 0x0503) || (PROJECT_ID == 0x050B) ||                            /* MLX81330A  SO8|QFN */ \
    (PROJECT_ID == 0x0502) || (PROJECT_ID == 0x050A) ||                            /* MLX81330A SO8| QFN wo INT-SHUNT */ \
    (PROJECT_ID == 0x050F) || (PROJECT_ID == 0x0510) || (PROJECT_ID == 0x0511) ||  /* MLX81330B1 SO8-001|QFN|SO8-101 */ \
    (PROJECT_ID == 0x0514) || (PROJECT_ID == 0x0515) || (PROJECT_ID == 0x0516) ||  /* MLX81330B2 SO8-002|QFN|SO8-102 */ \
    (PROJECT_ID == 0x0701) || (PROJECT_ID == 0x0705) ||                            /* MLX81332A QFN|SO8-001 */ \
    (PROJECT_ID == 0x070A) || (PROJECT_ID == 0x070B) || (PROJECT_ID == 0x070C) ||  /* MLX81332B1 QFN|SO8-001|SO8-101 */ \
    (PROJECT_ID == 0x070F) || (PROJECT_ID == 0x0710) || (PROJECT_ID == 0x0711) ||  /* MLX81332B2 QFN|SO8-002|SO8-102 */ \
    (PROJECT_ID == 0x0901) ||                                                      /* MLX81334AA QFN */ \
    (PROJECT_ID == 0x0A05) || (PROJECT_ID == 0x0A05) || (PROJECT_ID == 0x0A05) ||  /* MLX81340A QFN32|MLX81340B QFN32 */ \
    (PROJECT_ID == 0x2601) || (PROJECT_ID == 0x2602) || (PROJECT_ID == 0x2604)     /* MLX81350AA S08-001, S08-101, QFN24 */
    /* MLX81330A must switch to 5V; MLX81330B not */
    IO_PORT_MISC_OUT |= B_PORT_MISC_OUT_SWITCH_VDDA_TO_5V;
    DELAY(C_DELAY_250US);
#endif /* (PROJECT_ID == 0x05xx) || (PROJECT_ID == 0x07xx) */
#endif /* (LIN_AA_VDDA_5V != FALSE) && ((_SUPPORT_VDDA_5V == FALSE) && (_SUPPORT_SENSOR_VDDA_5V == FALSE)) */
    LINAA_RESET();
    l_u8AutoAddressingFlags |= (uint8_t)WAITINGFORBREAK;                        /* MMP180801-2 */

#if ((LINPROT & LINXX) == LIN13)
    /* LIN 1.3: 18T @ 9600 Baud or 36T @ 19200T, Measurement = 3.5-4T @ 9600 => 350us --> max 18us */
    l_u16LinAATimerPeriod = (PLL_FREQ / 71428U);                                /* Set to 71.4kHz rate, 14us (266-280us) */
#else /* ((LINPROT & LINXX) == LIN13) */
    /* LIN 2.x: 13T, Measurement = 3T; At 9600Baud: 312.5us --> max 16us, At 19200 Baud: 156us --> max 8us */
    if (l_u16SlowBaudrateAdjustment == C_SLOW_BAUDRATE_DELAY)
    {
        /* Slow Baudrate (9600-10415); 3 x Tbit = 288-312us */
        l_u16LinAATimerPeriod = (PLL_FREQ / 83333U);                            /* Set to 83.3kHz rate, 12us (228-240us) */
    }
    else
    {
        /* Fast Baudrate (19200); 3 x Tbit = 156us */
        l_u16LinAATimerPeriod = (PLL_FREQ / 166667U);                           /* Set to 167kHz rate, 6us (108-114us) */
    }
#endif /* ((LINPROT & LINXX) == LIN13) */

#if defined (ML_HAS_LIN_AUTOADDRESSING)
    /* enable the MLX4 auto addressing pulses */
    (void)ml_AutoAddressingConfig(ML_AA_ENABLED);
#endif /* ML_HAS_LIN_AUTOADDRESSING */
#if (_DEBUG_LINAA != FALSE)
    DEBUG_CLR_IO_A();
    DEBUG_SET_IO_B();
#endif /* (_DEBUG_LINAA != FALSE) */
} /* End of ml_InitAutoAddressing() */

/*!*************************************************************************** *
 * ml_StopAutoAddressing
 * \brief   Stop LIN-AA
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Stop/abort LIN-AA
 *          Disable MLX4 Auto-Addressing feature
 *          Restore LIN Protection (Remove XKEY)
 *          Release ADC for other (application) use
 * *************************************************************************** *
 * - Call Hierarchy: DfrDiagAssignNAD(), LIinAATimeoutControl(
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 1 (ml_AutoAddressingConfig())
 * *************************************************************************** */
void ml_StopAutoAddressing(void)
{
#if (LIN_AA_EXT_VOLTAGE != FALSE)
    IO_PORT_LINAA1 = 0U;
#endif /* (LIN_AA_EXT_VOLTAGE != FALSE) */
#if defined (ML_HAS_LIN_AUTOADDRESSING)
    /* enable the MLX4 auto addressing pulses */
    (void)ml_AutoAddressingConfig(ML_AA_DISABLED);
#endif /* ML_HAS_LIN_AUTOADDRESSING */
#if (LIN_AA_VDDA_5V != FALSE) && ((_SUPPORT_VDDA_5V == FALSE) && (_SUPPORT_SENSOR_VDDA_5V == FALSE))
#if (PROJECT_ID == 0x0503) || (PROJECT_ID == 0x050B) ||                            /* MLX81330A  SO8|QFN */ \
    (PROJECT_ID == 0x0502) || (PROJECT_ID == 0x050A) ||                            /* MLX81330A SO8| QFN wo INT-SHUNT */ \
    (PROJECT_ID == 0x050F) || (PROJECT_ID == 0x0510) || (PROJECT_ID == 0x0511) ||  /* MLX81330B1 SO8-001|QFN|SO8-101 */ \
    (PROJECT_ID == 0x0514) || (PROJECT_ID == 0x0515) || (PROJECT_ID == 0x0516) ||  /* MLX81330B2 SO8-002|QFN|SO8-102 */ \
    (PROJECT_ID == 0x0701) || (PROJECT_ID == 0x0705) ||                            /* MLX81332A QFN|SO8-001 */ \
    (PROJECT_ID == 0x070A) || (PROJECT_ID == 0x070B) || (PROJECT_ID == 0x070C) ||  /* MLX81332B1 QFN|SO8-001|SO8-101 */ \
    (PROJECT_ID == 0x070F) || (PROJECT_ID == 0x0710) || (PROJECT_ID == 0x0711) ||  /* MLX81332B2 QFN|SO8-002|SO8-102 */ \
    (PROJECT_ID == 0x0901) ||                                                      /* MLX81334AA QFN */ \
    (PROJECT_ID == 0x0A01) || (PROJECT_ID == 0x0A03) || (PROJECT_ID == 0x0A05) ||  /* MLX81340A QFN32|MLX81340B QFN32 */ \
    (PROJECT_ID == 0x2601) || (PROJECT_ID == 0x2602) || (PROJECT_ID == 0x2604)     /* MLX81350AA S08-001, S08-101, QFN24 */
    IO_PORT_MISC_OUT &= ~B_PORT_MISC_OUT_SWITCH_VDDA_TO_5V;
    DELAY(C_DELAY_250US);
#endif /* (PROJECT_ID == 0x05xx) || (PROJECT_ID == 0x07xx) */
#endif /* (LIN_AA_VDDA_5V != FALSE) && ((_SUPPORT_VDDA_5V == FALSE) && (_SUPPORT_SENSOR_VDDA_5V == FALSE)) */
#if (PROJECT_ID == 0x050F) || (PROJECT_ID == 0x0510) || (PROJECT_ID == 0x0511) || /* MLX81330B1 SO8-001|QFN|SO8-101 */ \
    (PROJECT_ID == 0x070A) || (PROJECT_ID == 0x070B) || (PROJECT_ID == 0x070C)    /* MLX81332B1 QFN|SO8-001|SO8-101 */
    IO_PORT_DRV_OUT = (IO_PORT_DRV_OUT & ~B_PORT_DRV_OUT_ENABLE_DRVSUP) | l_u16PreLinAA_MotorDrvState;  /* MMP190214-1 */
#endif /* (PROJECT_ID) */
#if defined (__MLX81332B02__) || defined (__MLX81334__)
    IO_TRIM_BG_BIAS = l_u16TrimBG;
    IO_TRIM_VDD = l_u16TrimVdd;
#endif /* defined (__MLX81332B02__) || defined (__MLX81334__) */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    LIN_CLR_XKEY();
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    l_u8AdcMode = (uint8_t)C_ADC_MODE_IDLE;
} /* End of ml_StopAutoAddressing() */
#endif /* (LIN_AA_BSM_SNPD_R1p0 != FALSE) */

/*!*************************************************************************** *
 * ml_GetAutoaddressingStatus
 * \brief   Get LIN-AA Status
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t Status: 0 = Not Last Slave
 *                           1 = Last Slave
 * *************************************************************************** *
 * \details Return the status according the LIN-AA flag LASTSLAVE (optionally set
 *          during Step 7 of the LIN-AA sequence)
 * *************************************************************************** *
 * - Call Hierarchy: DfrDiagAssignNAD()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
uint16_t ml_GetAutoaddressingStatus(void)
{
    uint16_t u16Result = FALSE;

    /* The slave is addressed in this cycle if the flag in step 6 is set */
    if ( (l_u8AutoAddressingFlags & LASTSLAVE) != 0U)
    {
        u16Result = TRUE;
    }
    return (u16Result);
} /* End of ml_GetAutoaddressingstatus() */

#if (LIN_AA_INFO != FALSE)
/*!*************************************************************************** *
 * ClearAAData
 * \brief   Clear LIN-AA data (debugging)
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Clear the LIN-AA optional info for debug/screening-purpose.
 * *************************************************************************** *
 * - Call Hierarchy: DfrDiagAssignNAD()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
void ClearAAData(void)
{
    uint16_t i;
    uint16_t *pLinAAInfo = (uint16_t *)&l_aSNPD_Data[0];
    for (i = (sizeof(l_aSNPD_Data) / sizeof(uint16_t)); i > 0U; i--)
    {
        *pLinAAInfo = 0U;
        pLinAAInfo++;
    }
    l_u8SNPD_CycleCount = 0U;
    g_u8SNPD_CycleCountComm = 0U;
} /* End of ClearAAData() */
#endif /* (LIN_AA_INFO != FALSE) */

#if (LIN_AA_TIMEOUT != FALSE)
/*!*************************************************************************** *
 * LinAATimeoutControl
 * \brief   LIN-AutoAddressing time-out control
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details If the LIN-AA mode is active for more than the time-out value
 *          (40s), the original (Non Volatile Memory) saved NAD will be restored and the
 *          LIN-AA mode will be cancelled.
 * NOTE: As this function is called by LIN call-back mlu_LinSleepMode IRQ, it is
 *       assumed that no Non Volatile Memory Write is busy.
 * *************************************************************************** *
 * - Call Hierarchy: HandleBusTimeout(), LinAutoAddressingTimer()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 1 (ml_StopAutoAddressing())
 * *************************************************************************** */
void LinAATimeoutControl(void)
{
    STD_LIN_PARAMS_t *pStdLin = (STD_LIN_PARAMS_t *)ADDR_NV_STD_LIN_1;

    /* LIN-AA takes too long time */
#if (LIN_AA_BSM_SNPD_R1p0 != FALSE)
    ml_StopAutoAddressing();
#endif /* (LIN_AA_BSM_SNPD_R1p0 != FALSE) */
    g_u8NAD = pStdLin->u8NAD;                                                   /* Restore original NAD */
    g_u16LinAATicker = 0U;                                                      /* Stop LIN-AA timeout counter */
    g_u8LinAAMode = 0U;                                                         /* Cancel LIN-AA mode */
#if (LIN_AA_LINFRAME_TIMEOUT != FALSE)
    l_u16LinFrameTimeOut = 0U;
#endif /* (LIN_AA_LINFRAME_TIMEOUT != FALSE) */
/*  mlu_AutoAddressingStep( 8);                                                 / * Cancel pending LIN-AA process too * / */
} /* End of LinAATimeoutControl() */
#endif /* (LIN_AA_TIMEOUT != FALSE) */

#if (LIN_AA_LINFRAME_TIMEOUT != FALSE)
/*!*************************************************************************** *
 * ClearLinFrameTimeOut
 * \brief   Clear LIN Frame Time-out period
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details During LIN-AA, LIN Frame time-out is done as frame has been received.
 * *************************************************************************** *
 * - Call Hierarchy: mlu_MessageReceived()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
void ClearLinFrameTimeOut(void)
{
    l_u16LinFrameTimeOut = 0U;
} /* End of ClearLinFrameTimeOut() */
#endif /* (LIN_AA_LINFRAME_TIMEOUT != FALSE) */

/*!*************************************************************************** *
 * LinAutoAddressingTimer
 * \brief   Periodic LIN Auto Addressing timer event
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details If the LIN-AA mode is active for more than the time-out value
 *          (40s), the original (Non Volatile Memory) saved NAD will be restored and the
 *          LIN-AA mode will be cancelled.
 * *************************************************************************** *
 * - Call Hierarchy: TIMER_IT()
 * - Cyclomatic Complexity: 4+1
 * - Nesting: 3
 * - Function calling: 2 (LinAATimeoutControl(), p_AwdAck())
 * *************************************************************************** */
void LinAutoAddressingTimer(void)
{
    if (g_u16LinAATicker != 0U)                                                 /* LIN-AA on going */
    {
        --g_u16LinAATicker;
        if (g_u16LinAATicker == 0U)                                             /* LIN-AutoAddresing time-out counter (seconds) */
        {
            --g_u8LinAATimeout;
            if (g_u8LinAATimeout == 0U)
            {
                LinAATimeoutControl();
            }
            else
            {
                g_u16LinAATicker = PI_TICKS_PER_SECOND;
            }
        }
    }

#if (LIN_AA_LINFRAME_TIMEOUT != FALSE)
    {
        uint16_t u16Mlx4Reconnect = FALSE;                                      /* MMP201113-1: LIN-AA LIN Bus-timeout fix (Begin) */
        ENTER_SECTION(ATOMIC_KEEP_MODE); /*lint !e534 */
        if (l_u16LinFrameTimeOut != 0U)
        {
            l_u16LinFrameTimeOut--;                                             /* MMP200507-1: Decrease atomically */
            if (l_u16LinFrameTimeOut == 0U)
            {
                u16Mlx4Reconnect = TRUE;
            }
        }
        EXIT_SECTION(); /*lint !e438 */
        if (u16Mlx4Reconnect != FALSE)
        {
            /* This will re-connect MLX4; This resets the LIN Bus-timeout and LIN wait for 3C data */
            (void)ml_Disconnect();
            (void)ml_Connect();
        }                                                                       /* MMP201113-1: LIN-AA LIN Bus-timeout fix (End) */
    }
#endif /* (LIN_AA_LINFRAME_TIMEOUT != FALSE) */

    if ( (l_u8AutoAddressingFlags & WAITINGFORBREAK) != 0U)
    {
        /* Start performing "background" watchdog acknowledges, during LIN auto-addressing (waiting for the LIN break-pulse */
        p_AwdAck(); /*lint !e522 */                                             /* Acknowledge Analogue Watchdog .. (MMP200625-2) */
#if (_SUPPORT_DWD != FALSE)
        WDG_conditionalIwdRefresh(C_IWD_DIV, C_IWD_TO);                         /* .. acknowledge the digital watchdog */
#endif /* (_SUPPORT_DWD != FALSE) */
    }
} /* End of LinAutoAddressingTimer() */

#else  /* (_SUPPORT_LIN_AA != FALSE) */

/*!*************************************************************************** *
 * mlu_AutoAddressingStep
 * \brief   LIN Auto Addressing Bus shunt method measurement steps
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] StepNumber
 *                1: Setup LIN-AA for current-offset measurement
 *                2: Perform current-offset measurement
 *                3: Setup LIN-AA for pre-selection measurement
 *                4: Perform pre-selection measurement
 *                5: Setup LIN-AA for final-selection measurement
 *                6: Perform final-selection measurement
 *                7: Restore LIN interface
 * \return  -
 * *************************************************************************** *
 * \details Perform LIN-Auto Addressing via LIN Bus-shunt method
 * *************************************************************************** *
 * - Call Hierarchy: LIN_Init()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
void mlu_AutoAddressingStep(uint8_t StepNumber)
{
    (void)StepNumber;
} /* End of mlu_AutoAddressingStep() */
#endif /* (_SUPPORT_LIN_AA != FALSE) */

#endif /* (LIN_COMM != FALSE) */

/* EOF */
