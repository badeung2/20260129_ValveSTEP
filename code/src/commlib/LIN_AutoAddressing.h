/*!*************************************************************************** *
 * \file        LIN_AutoAddressing.h
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
 * *************************************************************************** */

#ifndef ACT_PLTFRM_LIN_AUTOADDRESSING_H
#define ACT_PLTFRM_LIN_AUTOADDRESSING_H

/*!*************************************************************************** */
/*                           INCLUDES                                          */
/* *************************************************************************** */
#include "AppBuild.h"                                                           /* Application Build */

#if (LIN_COMM != FALSE) && (_SUPPORT_LIN_AA != FALSE)

/* *************************************************************************** */
/*                           DEFINITIONS                                       */
/* *************************************************************************** */
#define C_LINAA_TIMEOUT                     40U                                 /*!< LIN-AutoAddressing time-out of 40 seconds */

#define C_DELAY_5US             (uint16_t)((5UL * FPLL) / C_DELAY_CONST)        /*!<   5us delay */
#define C_DELAY_10US            (uint16_t)((10UL * FPLL) / C_DELAY_CONST)       /*!<  10us delay */
#define C_DELAY_250US           (uint16_t)((250UL * FPLL) / C_DELAY_CONST)      /*!< 250us delay */

/* flags in AutoAddressingFlags */
#define SLAVEADDRESSED                  0x01                                    /*!< 0: Slave still in the running; 1: slave is addressed */
#define SLAVEFINALSTEP                  0x02                                    /*!< 0: Don't measure Ishunt3; 1: Measure Ishunt3 */
#define WAITINGFORBREAK                 0x04                                    /*!< 0: Step 1 (LIN Break) received; 1: Waiting for LIN Break (MMP180801-2) */
#define LASTSLAVE                       0x80                                    /*!< 0: Slave is not the  last in line; 1: slave is the last in line. */

/*! LIN Auto Addressing ADC data */
typedef struct _ADC_LINAA
{
#if (_SUPPORT_LINAA_VS_COMPENSATION != FALSE)
    uint16_t u16VS_LinAA;                                                       /**< LIN-AA Supply compensation */
#endif /* (_SUPPORT_LINAA_VS_COMPENSATION != FALSE) */
    uint16_t au16Result_LinAA[18];                                              /**< LIN Auto Addressing data */
    uint16_t u16ADC_CRC;                                                        /**< LIN-AA data CRC */
} ADC_LINAA;
#define PADC_LINAA (ADC_LINAA *)                                                /*!< Pointer type of ADC_LINAA */

/* threshold constants for meeting the protocol */
#if (PROJECT_ID == 0x0503) || (PROJECT_ID == 0x050B) ||                            /* MLX81330A  SO8|QFN */ \
    (PROJECT_ID == 0x050F) || (PROJECT_ID == 0x0510) || (PROJECT_ID == 0x0511) ||  /* MLX81330B1 SO8-001|QFN|SO8-101 */ \
    (PROJECT_ID == 0x0514) || (PROJECT_ID == 0x0515) || (PROJECT_ID == 0x0516) ||  /* MLX81330B2 SO8-002|QFN|SO8-102 */ \
    (PROJECT_ID == 0x0701) || (PROJECT_ID == 0x0705) ||                            /* MLX81332A  QFN|SO8-001 */ \
    (PROJECT_ID == 0x070A) || (PROJECT_ID == 0x070B) || (PROJECT_ID == 0x070C) ||  /* MLX81332B1 QFN|SO8-001|SO8-101 */ \
    (PROJECT_ID == 0x070F) || (PROJECT_ID == 0x0710) || (PROJECT_ID == 0x0711) ||  /* MLX81332B2 QFN|SO8-002|SO8-102 */ \
    (PROJECT_ID == 0x0901) ||                                                      /* MLX81334AA QFN */ \
    (PROJECT_ID == 0x0A01) || (PROJECT_ID == 0x0A03) || (PROJECT_ID == 0x0A05) ||  /* MLX81340A QFN24 & QFN32 */ \
    defined (__MLX81350__)
#define C_LIN13AA_dI_1                  120                                     /*!< Pre-selection threshold: 1.20[mA] * 100 = 120 [10uA] (MMP180528-1) */
#define C_LIN13AA_dI_2                  120                                     /*!< Final-selection threshold: 1.20[mA] * 100 = 120 [10uA] (MMP180528-1) */
#define C_LIN2xAA_dI_1_BSM2             120                                     /*!< Pre-selection threshold: 1.20[mA] * 100 = 120 [10uA] (MMP180528-1) */
#define C_LIN2xAA_dI_2_BSM2             120                                     /*!< Final-selection threshold: 1.20[mA] * 100 = 120 [10uA] (MMP180528-1) */
#elif (PROJECT_ID == 0x0502) || (PROJECT_ID == 0x050A)                          /* MLX81330A  SO8|QFN without internal LIN-AA shunt */
#define C_LIN13AA_dI_1                   40                                     /*!< Pre-selection threshold: 0.40[mA] * 100 = 40 [10uA] (MMP180528-1) */
#define C_LIN13AA_dI_2                   40                                     /*!< Final-selection threshold: 0.40[mA] * 100 = 40 [10uA] (MMP180528-1) */
#define C_LIN2xAA_dI_1_BSM2              40                                     /*!< Pre-selection threshold: 0.40[mA] * 100 = 40 [10uA] (MMP180528-1) */
#define C_LIN2xAA_dI_2_BSM2              40                                     /*!< Final-selection threshold: 0.40[mA] * 100 = 40 [10uA] (MMP180528-1) */
#else
#error "LIN-AA fails to work"
#endif /* (PROJECT_ID) */

/*! Definition of auto-addressing state names */
typedef enum __attribute__((packed))
{
    AUTOADDRESSING_IDLE = 0,                                                    /**< array with pointers to all the functions */
    AUTOADDRESSING_STEP0,                                                       /**< that have to be executed in the different auto addressing steps */
    AUTOADDRESSING_STEP1,
    AUTOADDRESSING_STEP2,
    AUTOADDRESSING_STEP3,
    AUTOADDRESSING_STEP4,
    AUTOADDRESSING_STEP5,
    AUTOADDRESSING_STEP6,
    AUTOADDRESSING_WAIT,
    AUTOADDRESSING_DONE
} T_AUTOADDRESSING_STEP;

#if (LIN_AA_INFO != FALSE)
/*! LIN Auto-Addressing information structure (debugging/screening) */
typedef struct _SNPD_DATA
{
    uint8_t byStepAndFlags;                                                     /*!< 0x00: LIN-AA Step & Flags */
    uint8_t byIshunt1;                                                          /*!< 0x01: LIN-AA Offset current */
    uint8_t byIshunt2;                                                          /*!< 0x02: LIN-AA Pre-selection current */
    uint8_t byIshunt3;                                                          /*!< 0x03: LIN-AA Final selection current */
#if (LIN_AA_SCREENTEST != FALSE)
    uint16_t u16CM_1;                                                           /*!< 0x04: LIN-AA Common-mode offset */
    uint16_t u16DM_1;                                                           /*!< 0x06: LIN-AA Differential-mode offset */
    uint16_t u16CM_2;                                                           /*!< 0x08: LIN-AA Common-mode pre-selection */
    uint16_t u16DM_2;                                                           /*!< 0x0A: LIN-AA Differential-mode pre-selection */
    uint16_t u16CM_3;                                                           /*!< 0x0C: LIN-AA Common-mode final-selection */
    uint16_t u16DM_3;                                                           /*!< 0x0E: LIN-AA Differential-mode final-selection */
#endif /* (LIN_AA_SCREENTEST != FALSE) */
} SNPD_DATA;
#define PSNPD_DATA SNPD_DATA*                                                   /*!< Pointer to LIN-AA measurement structure */
#define LIN_AA_INFO_SZ                      16U                                 /*!< Number of LIN-AA measurements storage */
extern SNPD_DATA l_aSNPD_Data[LIN_AA_INFO_SZ];
#endif /* (LIN_AA_INFO != FALSE) */

/* ************************************************************************** */
/*                          GLOBAL VARIABLES                                  */
/* ************************************************************************** */
#pragma space dp
extern volatile uint8_t g_u8LinAAMode;                                          /*!< LIN-AA mode */
#pragma space none

#pragma space nodp
#if (LIN_AA_TIMEOUT != FALSE)
extern uint8_t g_u8LinAATimeout;                                                /*!< LIN-AA Timeout counter (seconds) */
extern uint16_t g_u16LinAATicker;                                               /*!< LIN-AA Ticker counter */
#endif /* (LIN_AA_TIMEOUT != FALSE) */
#if (LIN_AA_INFO != FALSE)
extern uint8_t g_u8SNPD_CycleCountComm;
#endif /* (LIN_AA_INFO != FALSE) */
#pragma space none

/*!*************************************************************************** */
/*                           GLOBAL FUNCTIONS                                  */
/* *************************************************************************** */
extern void ml_SetSlaveNotAddressed(void);                                      /*!< Set LIN-SLave status as 'not-addressed' */
extern void ml_SetSlaveAddressed(void);                                         /*!< Set LIN-SLave status as 'addressed' */
extern uint16_t ml_GetAutoaddressingStatus(void);                               /*!< Get LIN-AA Status */
#if (LIN_AA_INFO != FALSE)
extern void ClearAAData(void);                                                  /*!< Clear LIN-AA data */
#endif /* (LIN_AA_INFO != FALSE) */
#if (LIN_AA_BSM_SNPD_R1p0 != FALSE)
extern void ml_InitAutoAddressing(void);                                        /*!< Initialise LIN-AA */
extern void ml_StopAutoAddressing(void);                                        /*!< Stop LIN-AA */
#endif /* (LIN_AA_BSM_SNPD_R1p0 != FALSE) */
extern void LinAATimeoutControl(void);                                          /*!< LIN-AA time-out */
#if (LIN_AA_LINFRAME_TIMEOUT != FALSE)
extern void ClearLinFrameTimeOut(void);
#endif /* (LIN_AA_LINFRAME_TIMEOUT != FALSE) */
extern void LinAutoAddressingTimer(void);                                       /*!< LIN-AA Periodic Timer routine */

#endif /* (LIN_COMM != FALSE) && (_SUPPORT_LIN_AA != FALSE) */

#endif /* ACT_PLTFRM_LIN_AUTOADDRESSING_H */

/* EOF */
