/*!*************************************************************************** *
 * \file        MotorDriverSelfTest.c
 * \brief       MLX813xx Motor Driver Self-Test
 *
 * \note        project MLX813xx
 *
 * \author      Marcel Braat
 *
 * \date        2017-08-15
 *
 * \version     2.0
 *
 * A list of functions:
 *  - Global Functions:
 *           -# MotorDriverSelfTest()
 *  - Internal Functions:
 *           -# MotorDriverSelfTestInit()
 *           -# MotorDriverSelfTestPin()
 *           -# MotorDriverSelfTestPin2Pin()
 *           -# MotorDriverSelfTestCoilShort()
 *           -# MotorDriverSelfTestCoilOpen()
 *
 * \copyright   MELEXIS Microelectronic Integrated Systems
 *
 * Copyright (C) 2017-2022 Melexis N.V.
 * The Software is being delivered 'AS IS' and Melexis, whether explicitly or
 * implicitly, makes no warranty as to its Use or performance.
 * The user accepts the Melexis Firmware License Agreement.
 *
 * Melexis confidential & proprietary
 *                                                  RAM     Flash
 * _SUPPORT_MOTOR_SELFTEST (C_MOTOR_PHASES == 3):    0 B    1726 B
 * _SUPPORT_MOTOR_SELFTEST (C_MOTOR_PHASES == 4):    0 B    1798 B
 * *************************************************************************** */

/*!*************************************************************************** */
/*                           INCLUDES                                          */
/* *************************************************************************** */
#include "AppBuild.h"                                                           /* Application Build */

#if (_SUPPORT_MOTOR_SELFTEST != FALSE) || (_SUPPORT_OSD != FALSE)

#include "../ActADC.h"                                                          /* Application ADC support */

#include "drivelib/Diagnostic.h"                                                /* Diagnostic support */
#include "drivelib/NV_UserPage.h"                                               /* Non Volatile Memory User Application Page Layout */
#include "drivelib/ErrorCodes.h"                                                /* Error-logging support */
#include "drivelib/GlobalVars.h"                                                /* Global Variables support */
#include "drivelib/MotorDriver.h"                                               /* Motor driver support */
#include "drivelib/MotorDriverTables.h"                                         /* Wave-form vector tables */
#include "camculib/private_mathlib.h"                                           /* Private math-library */

#include <atomic.h>
#include <sys_tools.h>

/*!*************************************************************************** */
/*                           DEFINITIONS                                       */
/* *************************************************************************** */
#if (_SUPPORT_MOTOR_SELFTEST != FALSE)
#if defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
#define C_MIN_PWM_PULSE             (PWM_TIMER_CLK / 400000UL)                  /*!< PWM Pulse of 2.5us (MMP240304-1) */
#define C_ADC_TRIGGER_BEFORE_END    (PWM_TIMER_CLK / 2000000UL)                 /*!< ADC Trigger before end of pulse: 0.5us (MMP240304-1) */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#define C_MIN_PWM_PULSE             (PWM_TIMER_CLK / 250000UL)                  /*!< PWM Pulse of 4.0us */
#define C_ADC_TRIGGER_BEFORE_END    (PWM_TIMER_CLK / 2000000UL)                 /*!< 0.5us */
#endif

#define C_VSM_STABLE            39U                                             /*!< Motor-test VSM tolerance: 2.00V --> 2.00V/21 = 0.095V/2.5Vref * 1023 = 38.97 */
#endif /* (_SUPPORT_MOTOR_SELFTEST != FALSE) */

#if (_SUPPORT_OSD != FALSE) && (!defined (__MLX81160__) && !defined (__MLX81330__) && !defined (__MLX81339__) && !defined (__MLX81350__))
#if defined (__MLX81332__) || defined (__MLX81334__)
#define C_I_OSD                     0.0001                                      /*!< 100uA */
#define C_R_OSD                     19000                                       /*!< OSD 38kR//38kR */
#define C_VTH_HIGH ((int16_t)((2.5 * (C_I_OSD * C_R_OSD) * 1023) / (2.5 * 21)))    /*!< 2.5x IOSD * ROSD * 1023 / (2.5Vref * 21) */
#define C_VTH_LOW  ((int16_t)((0.25 * (C_I_OSD * C_R_OSD) * 1023) / (2.5 * 21)))   /*!< 0.25x IOSD * ROSD * 1023 / (2.5Vref * 21) */
#elif defined (__MLX81340__) || defined (__MLX81344__)
#define C_I_OSD                     0.0002                                      /*!< OSD 200uA (MMP231208-1) */
#define C_R_OSD                     17500                                       /*!< OSD 35kR//35kR (MMP231208-1) */
#define C_VTH_HIGH ((int16_t)((1.5 * (C_I_OSD * C_R_OSD) * 2048) / (1.48 * 26)))   /*!< 1.5x IOSD * ROSD * 2048 / (1.48Vref * 26) */
#define C_VTH_LOW  ((int16_t)((0.5 * (C_I_OSD * C_R_OSD) * 2048) / (1.48 * 26)))   /*!< 0.5x IOSD * ROSD * 2048 / (1.48Vref * 26) */
#elif defined (__MLX81346__)
#define C_I_OSD                     0.0002                                      /*!< OSD 200uA */
#define C_R_OSD                     17500                                       /*!< OSD 35kR//35kR */
#define C_VTH_HIGH ((int16_t)((1.5 * (C_I_OSD * C_R_OSD) * 2048) / (1.48 * 26)))   /*!< 1.5x IOSD * ROSD * 2048 / (1.48Vref * 26) */
#define C_VTH_LOW  ((int16_t)((0.5 * (C_I_OSD * C_R_OSD) * 2048) / (1.48 * 26)))   /*!< 0.5x IOSD * ROSD * 2048 / (1.48Vref * 26) */
#endif
#endif /* (_SUPPORT_OSD != FALSE) && (!defined (__MLX81160__) && !defined (__MLX81330__) && !defined (__MLX81339__) && !defined (__MLX81350__)) */

/*!*************************************************************************** */
/*                           GLOBAL & LOCAL VARIABLES                          */
/* *************************************************************************** */
#pragma space dp
#pragma space none

#pragma space nodp                                                              /* __NEAR_SECTION__ */
#if (_SUPPORT_MOTOR_SELFTEST != FALSE)
extern uint16_t l_au16AdcSource[C_ADC_SBASE_LEN];                               /*!< ADC Source buffer */
#endif /* (_SUPPORT_MOTOR_SELFTEST != FALSE) */
#if (_SUPPORT_OSD != FALSE) && (!defined (__MLX81160__) && !defined (__MLX81330__) && !defined (__MLX81339__) && !defined (__MLX81350__))
extern ADC_OSD_RESULTS AdcOsdResult;                                            /*!< Motor-driver inactive */
#endif /* (_SUPPORT_OSD != FALSE) && (!defined (__MLX81160__) && !defined (__MLX81330__) && !defined (__MLX81339__) && !defined (__MLX81350__)) */
#pragma space none                                                              /* __NEAR_SECTION__ */

#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
extern uint8_t l_u8MotorHoldingCurrState;
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */

#if (_SUPPORT_MOTOR_SELFTEST != FALSE)
#if (C_MOTOR_PHASES == 3)
/*! Set-Test - Part A: Single FET ON */
static const uint16_t c_au16DrvCfgSelfTestA[C_MOTOR_PHASES] =
{
#if defined (__MLX81160__)
#if (_SUPPORT_DUAL_BLDC != FALSE)
#error "Error: Self-test not supported"
#elif _SUPPORT_1ST_BLDC
    /* R-S-T */
    (C_PORT_DRV1_CTRL_DRV2_TRISTATE | C_PORT_DRV1_CTRL_DRV1_TRISTATE | C_PORT_DRV1_CTRL_DRV0_MASTER1),  /* PhC = Z, PhB = Z, PhA = PWM */
    (C_PORT_DRV1_CTRL_DRV2_TRISTATE | C_PORT_DRV1_CTRL_DRV1_MASTER1 | C_PORT_DRV1_CTRL_DRV0_TRISTATE),  /* PhC = Z, PhB = PWM, PhA = Z */
    (C_PORT_DRV1_CTRL_DRV2_MASTER1 | C_PORT_DRV1_CTRL_DRV1_TRISTATE | C_PORT_DRV1_CTRL_DRV0_TRISTATE)   /* PhC = PWM, PhB = Z, PhA = Z */
#else  /* _SUPPORT_1ST_BLDC */
    /* U-V-W */
    (C_PORT_DRV2_CTRL_DRV5_TRISTATE | C_PORT_DRV2_CTRL_DRV4_TRISTATE | C_PORT_DRV1_CTRL_DRV3_MASTER1),  /* PhC = Z, PhB = Z, PhA = PWM */
    (C_PORT_DRV2_CTRL_DRV5_TRISTATE | C_PORT_DRV2_CTRL_DRV4_MASTER1 | C_PORT_DRV1_CTRL_DRV3_TRISTATE),  /* PhC = Z, PhB = PWM, PhA = Z */
    (C_PORT_DRV2_CTRL_DRV5_MASTER1 | C_PORT_DRV2_CTRL_DRV4_TRISTATE | C_PORT_DRV1_CTRL_DRV3_TRISTATE)   /* PhC = PWM, PhB = Z, PhA = Z */
#endif /* _SUPPORT_1ST_BLDC */
#else
    /* U-V-W */
    (C_PORT_DRV_CTRL_DRV2_TRISTATE | C_PORT_DRV_CTRL_DRV1_TRISTATE | C_PORT_DRV_CTRL_DRV0_MASTER1),  /* PhC = Z, PhB = Z, PhA = PWM */
    (C_PORT_DRV_CTRL_DRV2_TRISTATE | C_PORT_DRV_CTRL_DRV1_MASTER1 | C_PORT_DRV_CTRL_DRV0_TRISTATE),  /* PhC = Z, PhB = PWM, PhA = Z */
    (C_PORT_DRV_CTRL_DRV2_MASTER1 | C_PORT_DRV_CTRL_DRV1_TRISTATE | C_PORT_DRV_CTRL_DRV0_TRISTATE)   /* PhC = PWM, PhB = Z, PhA = Z */
#endif
};

/* Set-Test - Part B: Single Phase-pair active, for a small period (6% duty cycle) */
static const uint16_t c_au16DrvCfgSelfTestB[C_MOTOR_PHASES * C_MOTOR_STEP_PER_PHASE] =
{
#if defined (__MLX81160__)
#if (_SUPPORT_DUAL_BLDC != FALSE)
#error "Error: Self-test not supported"
#elif _SUPPORT_1ST_BLDC
    /* R-S-T */
    (C_PORT_DRV1_CTRL_DRV3_TRISTATE | C_PORT_DRV1_CTRL_DRV2_H | C_PORT_DRV1_CTRL_DRV1_MASTER1 |
     C_PORT_DRV1_CTRL_DRV0_TRISTATE),                                                                                             /* PhC = H, PhB = PWM, PhA = Z */
    (C_PORT_DRV1_CTRL_DRV3_TRISTATE | C_PORT_DRV1_CTRL_DRV2_TRISTATE | C_PORT_DRV1_CTRL_DRV1_H |
     C_PORT_DRV1_CTRL_DRV0_MASTER1),                                                                                              /* PhC = Z, PhB = H, PhA = PWM */
    (C_PORT_DRV1_CTRL_DRV3_TRISTATE | C_PORT_DRV1_CTRL_DRV2_MASTER1 | C_PORT_DRV1_CTRL_DRV1_TRISTATE |
     C_PORT_DRV1_CTRL_DRV0_H),                                                                                                    /* PhC = PWM, PhB = Z, PhA = H */
    (C_PORT_DRV1_CTRL_DRV3_TRISTATE | C_PORT_DRV1_CTRL_DRV2_MASTER1 | C_PORT_DRV1_CTRL_DRV1_L |
     C_PORT_DRV1_CTRL_DRV0_TRISTATE),                                                                                             /* PhC = PWM, PhB = L, PhA = Z */
    (C_PORT_DRV1_CTRL_DRV3_TRISTATE | C_PORT_DRV1_CTRL_DRV2_L | C_PORT_DRV1_CTRL_DRV1_TRISTATE |
     C_PORT_DRV1_CTRL_DRV0_MASTER1),                                                                                              /* PhC = L, PhB = Z, PHA = PWM */
    (C_PORT_DRV1_CTRL_DRV3_TRISTATE | C_PORT_DRV1_CTRL_DRV2_TRISTATE | C_PORT_DRV1_CTRL_DRV1_MASTER1 |
     C_PORT_DRV1_CTRL_DRV0_L)                                                                                                     /* PhC = Z, PhB = PWM, PhA = L */
#else  /* _SUPPORT_1ST_BLDC */
    /* U-V-W */
    (C_PORT_DRV2_CTRL_DRV5_H | C_PORT_DRV2_CTRL_DRV4_MASTER1 | C_PORT_DRV1_CTRL_DRV3_TRISTATE),  /* PhC = H, PhB = PWM, PhA = Z */
    (C_PORT_DRV2_CTRL_DRV5_TRISTATE | C_PORT_DRV2_CTRL_DRV4_H | C_PORT_DRV1_CTRL_DRV3_MASTER1),  /* PhC = Z, PhB = H, PhA = PWM */
    (C_PORT_DRV2_CTRL_DRV5_MASTER1 | C_PORT_DRV2_CTRL_DRV4_TRISTATE | C_PORT_DRV1_CTRL_DRV3_H),  /* PhC = PWM, PhB = Z, PhA = H */
    (C_PORT_DRV2_CTRL_DRV5_MASTER1 | C_PORT_DRV2_CTRL_DRV4_L | C_PORT_DRV1_CTRL_DRV3_TRISTATE),  /* PhC = PWM, PhB = L, PhA = Z */
    (C_PORT_DRV2_CTRL_DRV5_L | C_PORT_DRV2_CTRL_DRV4_TRISTATE | C_PORT_DRV1_CTRL_DRV3_MASTER1),  /* PhC = L, PhB = Z, PHA = PWM */
    (C_PORT_DRV2_CTRL_DRV5_TRISTATE | C_PORT_DRV2_CTRL_DRV4_MASTER1 | C_PORT_DRV1_CTRL_DRV3_L)   /* PhC = Z, PhB = PWM, PhA = L */
#endif /* _SUPPORT_1ST_BLDC */
#elif defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
    /* U-V-W */
    (C_PORT_DRV_CTRL_DRV3_TRISTATE | C_PORT_DRV_CTRL_DRV2_H | C_PORT_DRV_CTRL_DRV1_MASTER1 |
     C_PORT_DRV_CTRL_DRV0_TRISTATE),                                                                                          /* PhC = H, PhB = PWM, PhA = Z */
    (C_PORT_DRV_CTRL_DRV3_TRISTATE | C_PORT_DRV_CTRL_DRV2_TRISTATE | C_PORT_DRV_CTRL_DRV1_H |
     C_PORT_DRV_CTRL_DRV0_MASTER1),                                                                                           /* PhC = Z, PhB = H, PhA = PWM */
    (C_PORT_DRV_CTRL_DRV3_TRISTATE | C_PORT_DRV_CTRL_DRV2_MASTER1 | C_PORT_DRV_CTRL_DRV1_TRISTATE |
     C_PORT_DRV_CTRL_DRV0_H),                                                                                                 /* PhC = PWM, PhB = Z, PhA = H */
    (C_PORT_DRV_CTRL_DRV3_TRISTATE | C_PORT_DRV_CTRL_DRV2_MASTER1 | C_PORT_DRV_CTRL_DRV1_L |
     C_PORT_DRV_CTRL_DRV0_TRISTATE),                                                                                          /* PhC = PWM, PhB = L, PhA = Z */
    (C_PORT_DRV_CTRL_DRV3_TRISTATE | C_PORT_DRV_CTRL_DRV2_L | C_PORT_DRV_CTRL_DRV1_TRISTATE |
     C_PORT_DRV_CTRL_DRV0_MASTER1),                                                                                           /* PhC = L, PhB = Z, PHA = PWM */
    (C_PORT_DRV_CTRL_DRV3_TRISTATE | C_PORT_DRV_CTRL_DRV2_TRISTATE | C_PORT_DRV_CTRL_DRV1_MASTER1 |
     C_PORT_DRV_CTRL_DRV0_L)                                                                                                  /* PhC = Z, PhB = PWM, PhA = L */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    /* U-V-W */
    (C_PORT_DRV_CTRL_DRV2_H | C_PORT_DRV_CTRL_DRV1_MASTER1 | C_PORT_DRV_CTRL_DRV0_TRISTATE),  /* PhC = H, PhB = PWM, PhA = Z */
    (C_PORT_DRV_CTRL_DRV2_TRISTATE | C_PORT_DRV_CTRL_DRV1_H | C_PORT_DRV_CTRL_DRV0_MASTER1),  /* PhC = Z, PhB = H, PhA = PWM */
    (C_PORT_DRV_CTRL_DRV2_MASTER1 | C_PORT_DRV_CTRL_DRV1_TRISTATE | C_PORT_DRV_CTRL_DRV0_H),  /* PhC = PWM, PhB = Z, PhA = H */
    (C_PORT_DRV_CTRL_DRV2_MASTER1 | C_PORT_DRV_CTRL_DRV1_L | C_PORT_DRV_CTRL_DRV0_TRISTATE),  /* PhC = PWM, PhB = L, PhA = Z */
    (C_PORT_DRV_CTRL_DRV2_L | C_PORT_DRV_CTRL_DRV1_TRISTATE | C_PORT_DRV_CTRL_DRV0_MASTER1),  /* PhC = L, PhB = Z, PHA = PWM */
    (C_PORT_DRV_CTRL_DRV2_TRISTATE | C_PORT_DRV_CTRL_DRV1_MASTER1 | C_PORT_DRV_CTRL_DRV0_L)   /* PhC = Z, PhB = PWM, PhA = L */
#endif
};

/*!*************************************************************************** *
 * MotorDriverSelfTest
 * \brief   Self test Motor Driver
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 * 1. Test FET short with Ground or Vsupply
 * 2. Test Motor-phase short
 * 3. Test Open connection with motor-phase
 * 4. Test BEMF Voltage levels
 * Note: Diagnostics must be initialised before calling this self-test.
 *
 * Error's:
 * C_ERR_SELFTEST_A: Over current occurred (IC Diagnostics ISR)
 * C_ERR_SELFTEST_B: Phase-pin voltage incorrect (Test A: Pin-test)
 * C_ERR_SELFTEST_C: Motor phase/coil voltage incorrect (Test B: Coil-test)
 * C_ERR_SELFTEST_D: Motor coil current below 5 LSB's (Test B: Coil-test)
 * C_ERR_SELFTEST_E: Phase-voltage incorrect (Test B: Coil-test)
 * C_ERR_SELFTEST_F: BEMF-voltage incorrect (Test B: Coil-test)
 * Performance: 1.75[ms]
 * *************************************************************************** *
 * - Call Hierarchy: main_Init(), MotorDriverStart()
 * - Cyclomatic Complexity: 68+1
 * - Nesting: 6
 * - Function calling: 3  (HAL_ADC_StopSafe(), MotorDriverPermanentError(),
 *                         SetLastError())
 * *************************************************************************** */
void MotorDriverSelfTest(void)
{
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
    static ADC_SDATA_t const tAdcPreSelfTest[] =
    {
        /* Driver supply voltage (21:1) */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_VSMF_HV,
                .u3AdcVref = C_ADC_VREF_2_50_V,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CMP,
                .u1AdcReserved = 0U
            }
        },
        /* Motor Phase U */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_U_HV,
                .u3AdcVref = C_ADC_VREF_2_50_V,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV1_CMP,
                .u1AdcReserved = 0U
            }
        },
        /* Motor Phase V */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_V_HV,
                .u3AdcVref = C_ADC_VREF_2_50_V,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                .u1AdcReserved = 0U
            }
        },
        /* Motor Phase W */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_W_HV,
                .u3AdcVref = C_ADC_VREF_2_50_V,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV3_CMP,
                .u1AdcReserved = 0U
            }
        },
        /* Driver supply voltage (21:1) */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_VSMF_HV,
                .u3AdcVref = C_ADC_VREF_2_50_V,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,
                .u1AdcReserved = 0U
            }
        }
    };
#elif defined (__MLX81339__) || defined (__MLX81350__)
    static ADC_SDATA_t const tAdcPreSelfTest[] =
    {
        /* Driver supply voltage (21:1) */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_VSMF_HV,
                .u3AdcReserved = 0U,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CMP,
                .u1AdcReserved = 0U
            }
        },
        /* Motor Phase U */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_U_HV,
                .u3AdcReserved = 0U,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV1_CMP,
                .u1AdcReserved = 0U
            }
        },
        /* Motor Phase V */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_V_HV,
                .u3AdcReserved = 0U,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                .u1AdcReserved = 0U
            }
        },
        /* Motor Phase W */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_W_HV,
                .u3AdcReserved = 0U,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV3_CMP,
                .u1AdcReserved = 0U
            }
        },
        /* Driver supply voltage (21:1) */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_VSMF_HV,
                .u3AdcReserved = 0U,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,
                .u1AdcReserved = 0U
            }
        }
    };
#elif defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    static ADC_SDATA_t const tAdcPreSelfTest[] =
    {
        /* Driver supply voltage (26:1) */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u1AdcSin = C_ADC_SIN_SDATA,
                .u6AdcChannel = C_ADC_VSMF_HV,
                .u1AdcType = C_ADC_TYPE_CYCLIC,
                .u6AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CMP
            }
        },
#if defined (__MLX81160__)
        /* Motor Phase R */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u1AdcSin = C_ADC_SIN_SDATA,
                .u6AdcChannel = C_ADC_PH_R_HV,
                .u1AdcType = C_ADC_TYPE_CYCLIC,
                .u6AdcTrigger = C_ADC_HW_TRIGGER_SLV1_CMP
            }
        },
        /* Motor Phase S */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u1AdcSin = C_ADC_SIN_SDATA,
                .u6AdcChannel = C_ADC_PH_S_HV,
                .u1AdcType = C_ADC_TYPE_CYCLIC,
                .u6AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP
            }
        },
        /* Motor Phase T */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u1AdcSin = C_ADC_SIN_SDATA,
                .u6AdcChannel = C_ADC_PH_T_HV,
                .u1AdcType = C_ADC_TYPE_CYCLIC,
                .u6AdcTrigger = C_ADC_HW_TRIGGER_SLV3_CMP
            }
        },
#else  /* defined (__MLX81160__) */
#if defined (__MLX81346__) && _SUPPORT_APP_48V_ADC
        /* Motor Phase U (1:52) */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u1AdcSin = C_ADC_SIN_SDATA,
                .u6AdcChannel = C_ADC_PH_U_DIV52,
                .u1AdcType = C_ADC_TYPE_CYCLIC,
                .u6AdcTrigger = C_ADC_HW_TRIGGER_SLV1_CMP
            }
        },
        /* Motor Phase V (1:52) */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u1AdcSin = C_ADC_SIN_SDATA,
                .u6AdcChannel = C_ADC_PH_V_DIV52,
                .u1AdcType = C_ADC_TYPE_CYCLIC,
                .u6AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP
            }
        },
        /* Motor Phase W (1:52) */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u1AdcSin = C_ADC_SIN_SDATA,
                .u6AdcChannel = C_ADC_PH_W_DIV52,
                .u1AdcType = C_ADC_TYPE_CYCLIC,
                .u6AdcTrigger = C_ADC_HW_TRIGGER_SLV3_CMP
            }
        },
#else  /* defined (__MLX81346__) && _SUPPORT_APP_48V_ADC */
        /* Motor Phase U */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u1AdcSin = C_ADC_SIN_SDATA,
                .u6AdcChannel = C_ADC_PH_U_HV,
                .u1AdcType = C_ADC_TYPE_CYCLIC,
                .u6AdcTrigger = C_ADC_HW_TRIGGER_SLV1_CMP
            }
        },
        /* Motor Phase V */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u1AdcSin = C_ADC_SIN_SDATA,
                .u6AdcChannel = C_ADC_PH_V_HV,
                .u1AdcType = C_ADC_TYPE_CYCLIC,
                .u6AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP
            }
        },
        /* Motor Phase W */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u1AdcSin = C_ADC_SIN_SDATA,
                .u6AdcChannel = C_ADC_PH_W_HV,
                .u1AdcType = C_ADC_TYPE_CYCLIC,
                .u6AdcTrigger = C_ADC_HW_TRIGGER_SLV3_CMP
            }
        },
#endif /* defined (__MLX81346__) && _SUPPORT_APP_48V_ADC */
#endif /* defined (__MLX81160__) */
        /* Driver supply voltage (26:1) */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u1AdcSin = C_ADC_SIN_SDATA,
                .u6AdcChannel = C_ADC_VSMF_HV,
                .u1AdcType = C_ADC_TYPE_CYCLIC,
                .u6AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT
            }
        }
    };
#endif

    /*! SelfTest "A" (Pre) */
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
    static ADC_SDATA_t const tAdcSelfTest_PreA[] =
    {
        /* Junction Temperature */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_TEMP,
                .u3AdcVref = C_ADC_VREF_2_50_V,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CMP,
                .u1AdcReserved = 0U
            }
        },
        /* Driver supply voltage (21:1) */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_VSMF_HV,
                .u3AdcVref = C_ADC_VREF_HV,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV1_CMP,
                .u1AdcReserved = 0U
            }
        }
    };
#elif defined (__MLX81339__) || defined (__MLX81350__)
    static ADC_SDATA_t const tAdcSelfTest_PreA[] =
    {
        /* Junction Temperature */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_TEMP,
                .u3AdcReserved = 0U,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CMP,
                .u1AdcReserved = 0U
            }
        },
        /* Driver supply voltage (21:1) */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_VSMF_HV,
                .u3AdcReserved = 0U,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV1_CMP,
                .u1AdcReserved = 0U
            }
        }
    };
#elif defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    static ADC_SDATA_t const tAdcSelfTest_PreA[] =
    {
        /* Junction Temperature */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u1AdcSin = C_ADC_SIN_SDATA,
                .u6AdcChannel = C_ADC_TEMP,
                .u1AdcType = C_ADC_TYPE_CYCLIC,
                .u6AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CMP
            }
        },
        /* Driver supply voltage (26:1) */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u1AdcSin = C_ADC_SIN_SDATA,
                .u6AdcChannel = C_ADC_VSMF_HV,
                .u1AdcType = C_ADC_TYPE_CYCLIC,
                .u6AdcTrigger = C_ADC_HW_TRIGGER_SLV1_CMP
            }
        }
    };
#endif

#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
    static ADC_SDATA_t const tAdcSelfTestA[] =
    {
        /* Motor Phase U */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_U_HV,
                .u3AdcVref = C_ADC_VREF_2_50_V,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                .u1AdcReserved = 0U
            }
        },
        /* Motor Phase V */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_V_HV,
                .u3AdcVref = C_ADC_VREF_2_50_V,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                .u1AdcReserved = 0U
            }
        },
        /* Motor Phase W */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_W_HV,
                .u3AdcVref = C_ADC_VREF_2_50_V,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                .u1AdcReserved = 0U
            }
        }
    };
#elif defined (__MLX81339__) || defined (__MLX81350__)
    static ADC_SDATA_t const tAdcSelfTestA[] =
    {
        /* Motor Phase U */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_U_HV,
                .u3AdcReserved = 0U,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                .u1AdcReserved = 0U
            }
        },
        /* Motor Phase V */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_V_HV,
                .u3AdcReserved = 0U,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                .u1AdcReserved = 0U
            }
        },
        /* Motor Phase W */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_W_HV,
                .u3AdcReserved = 0U,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                .u1AdcReserved = 0U
            }
        }
    };
#elif defined (__MLX81160__)
    static uint16_t const tAdcSelfTestA[3] =
    {
#if (_SUPPORT_DUAL_BLDC != FALSE)
#error "Error: Self-test not supported"
#elif _SUPPORT_1ST_BLDC
        /* Motor Phase R */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_R_HV,
                .u3AdcVref = C_ADC_VREF_2_50_V,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                .u1AdcReserved = 0U
            }
        },
        /* Motor Phase S */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_S_HV,
                .u3AdcVref = C_ADC_VREF_2_50_V,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                .u1AdcReserved = 0U
            }
        },
        /* Motor Phase T */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_T_HV,
                .u3AdcVref = C_ADC_VREF_2_50_V,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                .u1AdcReserved = 0U
            }
        },
#else  /* _SUPPORT_1ST_BLDC */
        /* Motor Phase U */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_U_HV,
                .u3AdcVref = C_ADC_VREF_2_50_V,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                .u1AdcReserved = 0U
            }
        },
        /* Motor Phase V */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_V_HV,
                .u3AdcVref = C_ADC_VREF_2_50_V,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                .u1AdcReserved = 0U
            }
        },
        /* Motor Phase W */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_W_HV,
                .u3AdcVref = C_ADC_VREF_2_50_V,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                .u1AdcReserved = 0U
            }
        }
#endif /* _SUPPORT_1ST_BLDC */
    };
#else  /* defined (__MLX81160__) */
    static ADC_SDATA_t const tAdcSelfTestA[] =
    {
#if defined (__MLX81346__) && _SUPPORT_APP_48V_ADC
        /* Motor Phase U (1:52) */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u1AdcSin = C_ADC_SIN_SDATA,
                .u6AdcChannel = C_ADC_PH_U_DIV52,
                .u1AdcType = C_ADC_TYPE_CYCLIC,
                .u6AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP
            }
        },
        /* Motor Phase V (1:52) */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u1AdcSin = C_ADC_SIN_SDATA,
                .u6AdcChannel = C_ADC_PH_V_DIV52,
                .u1AdcType = C_ADC_TYPE_CYCLIC,
                .u6AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP
            }
        },
        /* Motor Phase W (1:52) */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u1AdcSin = C_ADC_SIN_SDATA,
                .u6AdcChannel = C_ADC_PH_W_DIV52,
                .u1AdcType = C_ADC_TYPE_CYCLIC,
                .u6AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP
            }
        }
#else  /* defined (__MLX81346__) && _SUPPORT_APP_48V_ADC */
        /* Motor Phase U */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u1AdcSin = C_ADC_SIN_SDATA,
                .u6AdcChannel = C_ADC_PH_U_HV,
                .u1AdcType = C_ADC_TYPE_CYCLIC,
                .u6AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP
            }
        },
        /* Motor Phase V */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u1AdcSin = C_ADC_SIN_SDATA,
                .u6AdcChannel = C_ADC_PH_V_HV,
                .u1AdcType = C_ADC_TYPE_CYCLIC,
                .u6AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP
            }
        },
        /* Motor Phase W */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u1AdcSin = C_ADC_SIN_SDATA,
                .u6AdcChannel = C_ADC_PH_W_HV,
                .u1AdcType = C_ADC_TYPE_CYCLIC,
                .u6AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP
            }
        }
#endif /* defined (__MLX81346__) && _SUPPORT_APP_48V_ADC */
    };
#endif /* defined (__MLX81160__) */

#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
    static ADC_SDATA_t const tAdcSelfTestB[] =
    {
        /* Driver supply voltage (21:1) */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_VSMF_HV,
                .u3AdcVref = C_ADC_VREF_2_50_V,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                .u1AdcReserved = 0U
            }
        },
        /* Motor current (unfiltered) */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_MCUR,
                .u3AdcVref = C_ADC_VREF_2_50_V,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,
                .u1AdcReserved = 0U
            }
        },
        /* Motor Phase U */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_U_HV,
                .u3AdcVref = C_ADC_VREF_2_50_V,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                .u1AdcReserved = 0U
            }
        },
        /* Motor Phase U */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_U_HV,
                .u3AdcVref = C_ADC_VREF_2_50_V,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,
                .u1AdcReserved = 0U
            }
        },
        /* Motor Phase V */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_V_HV,
                .u3AdcVref = C_ADC_VREF_2_50_V,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                .u1AdcReserved = 0U
            }
        },
        /* Motor Phase V */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_V_HV,
                .u3AdcVref = C_ADC_VREF_2_50_V,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,
                .u1AdcReserved = 0U
            }
        },
        /* Motor Phase W */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_W_HV,
                .u3AdcVref = C_ADC_VREF_2_50_V,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                .u1AdcReserved = 0U
            }
        },
        /* Motor Phase W */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_W_HV,
                .u3AdcVref = C_ADC_VREF_2_50_V,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,
                .u1AdcReserved = 0U
            }
        },
        /* Motor current (unfiltered) */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_MCUR,
                .u3AdcVref = C_ADC_VREF_2_50_V,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                .u1AdcReserved = 0U
            }
        },
        /* Motor current (unfiltered) */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_MCUR,
                .u3AdcVref = C_ADC_VREF_2_50_V,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,
                .u1AdcReserved = 0U
            }
        },
        {.u16 = C_ADC_EOS}                                                      /* End-of-Sequence */
    };
#elif defined (__MLX81339__) || defined (__MLX81350__)
    static ADC_SDATA_t const tAdcSelfTestB[] =
    {
        /* Driver supply voltage (21:1) */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_VSMF_HV,
                .u3AdcReserved = 0U,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                .u1AdcReserved = 0U
            }
        },
        /* Motor current (unfiltered) */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_MCUR,
                .u3AdcReserved = 0U,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,
                .u1AdcReserved = 0U
            }
        },
        /* Motor Phase U */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_U_HV,
                .u3AdcReserved = 0U,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                .u1AdcReserved = 0U
            }
        },
        /* Motor Phase U */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_U_HV,
                .u3AdcReserved = 0U,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,
                .u1AdcReserved = 0U
            }
        },
        /* Motor Phase V */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_V_HV,
                .u3AdcReserved = 0U,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                .u1AdcReserved = 0U
            }
        },
        /* Motor Phase V */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_V_HV,
                .u3AdcReserved = 0U,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,
                .u1AdcReserved = 0U
            }
        },
        /* Motor Phase W */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_W_HV,
                .u3AdcReserved = 0U,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                .u1AdcReserved = 0U
            }
        },
        /* Motor Phase W */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_W_HV,
                .u3AdcReserved = 0U,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,
                .u1AdcReserved = 0U
            }
        },
        /* Motor current (unfiltered) */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_MCUR,
                .u3AdcReserved = 0U,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                .u1AdcReserved = 0U
            }
        },
        /* Motor current (unfiltered) */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_MCUR,
                .u3AdcReserved = 0U,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,
                .u1AdcReserved = 0U
            }
        },
        {.u16 = C_ADC_EOS}                                                      /* End-of-Sequence */
    };
#else
    static ADC_SDATA_t const tAdcSelfTestB[] =
    {
        /* Driver supply voltage (21:1) */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u1AdcSin = C_ADC_SIN_SDATA,
                .u6AdcChannel = C_ADC_VSMF_HV,
                .u1AdcType = C_ADC_TYPE_CYCLIC,
                .u6AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP
            }
        },
        /* Motor current (unfiltered) */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u1AdcSin = C_ADC_SIN_SDATA,
#if defined (__MLX81160__)
#if (_SUPPORT_DUAL_BLDC != FALSE)
#error "Error: Self-test not supported"
#elif (_SUPPORT_1ST_BLDC != FALSE)
                .u6AdcChannel = C_ADC_MCUR1,
#else  /* (_SUPPORT_1ST_BLDC != FALSE) */
                .u6AdcChannel = C_ADC_MCUR2,
#endif /* (_SUPPORT_1ST_BLDC != FALSE) */
#else  /* defined (__MLX81160__) */
                .u6AdcChannel = C_ADC_MCUR,
#endif
                .u1AdcType = C_ADC_TYPE_CYCLIC,
                .u6AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT
            }
        },
        /* Motor Phase U */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u1AdcSin = C_ADC_SIN_SDATA,
#if defined (__MLX81160__)
#if (_SUPPORT_DUAL_BLDC != FALSE)
#error "Error: Self-test not supported"
#elif (_SUPPORT_1ST_BLDC != FALSE)
                .u6AdcChannel = C_ADC_PH_R_HV,
#else  /* (_SUPPORT_1ST_BLDC != FALSE) */
                .u6AdcChannel = C_ADC_PH_U_HV,
#endif /* (_SUPPORT_1ST_BLDC != FALSE) */
#elif defined (__MLX81346__) && _SUPPORT_APP_48V_ADC
                .u6AdcChannel = C_ADC_PH_U_DIV52,
#else  /* defined (__MLX81346__) && _SUPPORT_APP_48V_ADC */
                .u6AdcChannel = C_ADC_PH_U_HV,
#endif /* defined (__MLX81346__) && _SUPPORT_APP_48V_ADC */
                .u1AdcType = C_ADC_TYPE_CYCLIC,
                .u6AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP
            }
        },
        /* Motor Phase U */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u1AdcSin = C_ADC_SIN_SDATA,
#if defined (__MLX81160__)
#if (_SUPPORT_DUAL_BLDC != FALSE)
#error "Error: Self-test not supported"
#elif (_SUPPORT_1ST_BLDC != FALSE)
                .u6AdcChannel = C_ADC_PH_R_HV,
#else  /* (_SUPPORT_1ST_BLDC != FALSE) */
                .u6AdcChannel = C_ADC_PH_U_HV,
#endif /* (_SUPPORT_1ST_BLDC != FALSE) */
#elif defined (__MLX81346__) && _SUPPORT_APP_48V_ADC
                .u6AdcChannel = C_ADC_PH_U_DIV52,
#else  /* defined (__MLX81346__) && _SUPPORT_APP_48V_ADC */
                .u6AdcChannel = C_ADC_PH_U_HV,
#endif /* defined (__MLX81346__) && _SUPPORT_APP_48V_ADC */
                .u1AdcType = C_ADC_TYPE_CYCLIC,
                .u6AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT
            }
        },
        /* Motor Phase V */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u1AdcSin = C_ADC_SIN_SDATA,
#if defined (__MLX81160__)
#if (_SUPPORT_DUAL_BLDC != FALSE)
#error "Error: Self-test not supported"
#elif (_SUPPORT_1ST_BLDC != FALSE)
                .u6AdcChannel = C_ADC_PH_S_HV,
#else  /* (_SUPPORT_1ST_BLDC != FALSE) */
                .u6AdcChannel = C_ADC_PH_V_HV,
#endif /* (_SUPPORT_1ST_BLDC != FALSE) */
#elif defined (__MLX81346__) && _SUPPORT_APP_48V_ADC
                .u6AdcChannel = C_ADC_PH_V_DIV52,
#else  /* defined (__MLX81346__) && _SUPPORT_APP_48V_ADC */
                .u6AdcChannel = C_ADC_PH_V_HV,
#endif /* defined (__MLX81346__) && _SUPPORT_APP_48V_ADC */
                .u1AdcType = C_ADC_TYPE_CYCLIC,
                .u6AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP
            }
        },
        /* Motor Phase V */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u1AdcSin = C_ADC_SIN_SDATA,
#if defined (__MLX81160__)
#if (_SUPPORT_DUAL_BLDC != FALSE)
#error "Error: Self-test not supported"
#elif (_SUPPORT_1ST_BLDC != FALSE)
                .u6AdcChannel = C_ADC_PH_S_HV,
#else  /* (_SUPPORT_1ST_BLDC != FALSE) */
                .u6AdcChannel = C_ADC_PH_V_HV,
#endif /* (_SUPPORT_1ST_BLDC != FALSE) */
#elif defined (__MLX81346__) && _SUPPORT_APP_48V_ADC
                .u6AdcChannel = C_ADC_PH_V_DIV52,
#else  /* defined (__MLX81346__) && _SUPPORT_APP_48V_ADC */
                .u6AdcChannel = C_ADC_PH_V_HV,
#endif /* defined (__MLX81346__) && _SUPPORT_APP_48V_ADC */
                .u1AdcType = C_ADC_TYPE_CYCLIC,
                .u6AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT
            }
        },
        /* Motor Phase W */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u1AdcSin = C_ADC_SIN_SDATA,
#if defined (__MLX81160__)
#if (_SUPPORT_DUAL_BLDC != FALSE)
#error "Error: Self-test not supported"
#elif (_SUPPORT_1ST_BLDC != FALSE)
                .u6AdcChannel = C_ADC_PH_T_HV,
#else  /* (_SUPPORT_1ST_BLDC != FALSE) */
                .u6AdcChannel = C_ADC_PH_W_HV,
#endif /* (_SUPPORT_1ST_BLDC != FALSE) */
#elif defined (__MLX81346__) && _SUPPORT_APP_48V_ADC
                .u6AdcChannel = C_ADC_PH_W_DIV52,
#else  /* defined (__MLX81346__) && _SUPPORT_APP_48V_ADC */
                .u6AdcChannel = C_ADC_PH_W_HV,
#endif /* defined (__MLX81346__) && _SUPPORT_APP_48V_ADC */
                .u1AdcType = C_ADC_TYPE_CYCLIC,
                .u6AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP
            }
        },
        /* Motor Phase W */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u1AdcSin = C_ADC_SIN_SDATA,
#if defined (__MLX81160__)
#if (_SUPPORT_DUAL_BLDC != FALSE)
#error "Error: Self-test not supported"
#elif (_SUPPORT_1ST_BLDC != FALSE)
                .u6AdcChannel = C_ADC_PH_T_HV,
#else  /* (_SUPPORT_1ST_BLDC != FALSE) */
                .u6AdcChannel = C_ADC_PH_W_HV,
#endif /* (_SUPPORT_1ST_BLDC != FALSE) */
#elif defined (__MLX81346__) && _SUPPORT_APP_48V_ADC
                .u6AdcChannel = C_ADC_PH_W_DIV52,
#else  /* defined (__MLX81346__) && _SUPPORT_APP_48V_ADC */
                .u6AdcChannel = C_ADC_PH_W_HV,
#endif /* defined (__MLX81346__) && _SUPPORT_APP_48V_ADC */
                .u1AdcType = C_ADC_TYPE_CYCLIC,
                .u6AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT
            }
        },
        /* Motor current (unfiltered) */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u1AdcSin = C_ADC_SIN_SDATA,
#if defined (__MLX81160__)
#if (_SUPPORT_DUAL_BLDC != FALSE)
#error "Error: Self-test not supported"
#elif (_SUPPORT_1ST_BLDC != FALSE)
                .u6AdcChannel = C_ADC_MCUR1,
#else  /* (_SUPPORT_1ST_BLDC != FALSE) */
                .u6AdcChannel = C_ADC_MCUR2,
#endif /* (_SUPPORT_1ST_BLDC != FALSE) */
#else  /* defined (__MLX81160__) */
                .u6AdcChannel = C_ADC_MCUR,
#endif
                .u1AdcType = C_ADC_TYPE_CYCLIC,
                .u6AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP
            }
        },
        /* Motor current (unfiltered) */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u1AdcSin = C_ADC_SIN_SDATA,
#if defined (__MLX81160__)
#if (_SUPPORT_DUAL_BLDC != FALSE)
#error "Error: Self-test not supported"
#elif (_SUPPORT_1ST_BLDC != FALSE)
                .u6AdcChannel = C_ADC_MCUR1,
#else  /* (_SUPPORT_1ST_BLDC != FALSE) */
                .u6AdcChannel = C_ADC_MCUR2,
#endif /* (_SUPPORT_1ST_BLDC != FALSE) */
#else  /* defined (__MLX81160__) */
                .u6AdcChannel = C_ADC_MCUR,
#endif
                .u1AdcType = C_ADC_TYPE_CYCLIC,
                .u6AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT
            }
        },
        {.u16 = C_ADC_EOS}                                                      /* End-of-Sequence */
    };
#endif

    uint16_t u16SelfTestIdx;
    ADC_PRE_TEST adcMotorPreTest;
    ADC_PIN_TEST adcMotorPinTest;
    ADC_SELFTEST adcMotorSelfTest;
    uint16_t u16PhaseVoltageZ[3] = {0U, 0U, 0U};
    uint16_t u16Pwm2Storage = IO_PWM_SLAVE2_CMP;                                /* Save PWM2 ADC trigger CMP time */
#if defined (__MLX81160__)
    uint16_t u16Vds = 135U;                                                     /* 2.0V --> (2.0V / 26) * (2048 / 1.165V) = 135.22 ADC-LSB's */
#elif defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
    uint16_t u16Vds = 39U;                                                      /* 2.0V --> (2.0V / 21) * (1023 / 2.5V) = 38.97 ADC-LSB's */
#elif defined (__MLX81339__)
    uint16_t u16Vds = 276U;                                                     /* 2.0V --> (2.0V / 18) * (4095 / 1.65V) = 275.75 ADC-LSB's */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#if defined (__MLX81346__) && _SUPPORT_APP_48V_ADC
    uint16_t u16Vds = 53U;                                                      /* 2.0V --> (2.0V / 52) * (2048 / 1.48V) = 53.22 ADC-LSB's */
#else  /* defined (__MLX81346__) && _SUPPORT_APP_48V_ADC */
    uint16_t u16Vds = 106U;                                                     /* 2.0V --> (2.0V / 26) * (2048 / 1.48V) = 106.44 ADC-LSB's */
#endif /* defined (__MLX81346__) && _SUPPORT_APP_48V_ADC */
#elif defined (__MLX81350__)
    uint16_t u16Vds = 248U;                                                     /* 2.0V --> (2.0V / 20) * (4095 / 1.65V) = 248.18 ADC-LSB's */
#endif
    uint16_t u16Vsm = 0U;
    uint16_t *pu16Src = &l_au16AdcSource[0];

    /* 1. Measure all phase voltage and Motor Driver Voltage */
#if defined (__MLX81160__)
    DRVCFG_DIS_RSTUVW();
#elif defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
    DRVCFG_DIS_UVW();
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    DRVCFG_DIS();
#endif

#if (_DEBUG_SELFTEST != FALSE)
    DEBUG_SET_IO_B();
#endif /* (_DEBUG_SELFTEST != FALSE) */
    HAL_ADC_StopSafe();                                                         /* Clear the ADC control register */
    *pu16Src++ = (uint16_t)&adcMotorPreTest.MotorDriverVoltageA;
    *pu16Src++ = tAdcPreSelfTest[0].u16;
    *pu16Src++ = tAdcPreSelfTest[1].u16;
    *pu16Src++ = tAdcPreSelfTest[2].u16;
    *pu16Src++ = tAdcPreSelfTest[3].u16;
    *pu16Src++ = tAdcPreSelfTest[4].u16;
    *pu16Src = C_ADC_EOS;

    HAL_ADC_Setup(/*B_ADC_ADC_WIDTH |*/                                         /* Setup Self Test measurement */
        C_ADC_ASB_NEVER |                                                       /* Auto Standby: Never */
#if (_DEBUG_IO_ADC != FALSE)
        C_ADC_INT_SCHEME_EOC |                                                  /* Message interrupt: End-of-Frame */
#else  /* (_DEBUG_IO_ADC != FALSE) */
        C_ADC_INT_SCHEME_NOINT |                                                /* Message interrupt: No */
#endif /* (_DEBUG_IO_ADC != FALSE) */
        B_ADC_SATURATE |                                                        /* Saturation: Enabled */
        B_ADC_NO_INTERLEAVE |                                                   /* Interleave: No */
        C_ADC_SOC_SOURCE_HARD_CTRIG |                                           /* Start Of Conversion (SOC) triggered by: Hardware (HARD_CTRIG) */
        C_ADC_SOS_SOURCE_HARD_CTRIG);                                           /* Start Of Sequence (SOS) triggered: 2nd Hardware */
    HAL_ADC_ClearErrors();                                                      /* Prior to start, first clear any error flag and enable Triggers */
    HAL_ADC_Start();                                                            /* Start ADC */
    while ( (IO_ADC_CTRL & B_ADC_START) != 0U) {}                               /* Wait for ADC result (Time-out?) */
#if (_DEBUG_SELFTEST != FALSE)
    DEBUG_CLR_IO_B();
#endif /* (_DEBUG_SELFTEST != FALSE) */

    /* 2. Check for each phase voltage is near GNDM or VSM
     *    a. Near GNDM: Create 2us high-pulse and check phase-voltage
     *    b. Near VSM: Create 2us low-pulse and check phase-voltage
     * Test-time: Approx. 9 PWM periods (@20kHz: 450us). */
    MotorDriverConfig(TRUE);
    IO_PWM_SLAVE2_CMP = (((3UL * PWM_REG_PERIOD) + 3U) / 6U) + C_PWM_DCORR;       /* 50.0% of period */

#if defined (__MLX81160__)
    IO_PORT_DRV1_CTRL =
        (C_PORT_DRV1_CTRL_DRV2_TRISTATE | C_PORT_DRV1_CTRL_DRV1_TRISTATE | C_PORT_DRV1_CTRL_DRV0_TRISTATE);
#else  /* defined (__MLX81160__) */
    IO_PORT_DRV_CTRL = (C_PORT_DRV_CTRL_DRV2_TRISTATE | C_PORT_DRV_CTRL_DRV1_TRISTATE | C_PORT_DRV_CTRL_DRV0_TRISTATE);
#endif /* defined (__MLX81160__) */
#if defined (__MLX81160__)
#if (_SUPPORT_DUAL_BLDC != FALSE)
    DRVCFG_ENA_RST_UVW();
#elif (_SUPPORT_1ST_BLDC != FALSE)
    DRVCFG_ENA_RST();
#else  /* (_SUPPORT_1ST_BLDC != FALSE) */
    DRVCFG_ENA_UVW();
#endif /* (_SUPPORT_1ST_BLDC != FALSE) */
#elif defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
    DRVCFG_ENA_UVW();
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    DRVCFG_ENA();
#endif

    /* Test for FET shortages; Note: Diagnostics configuration will switch off driver at over-current */
#if (_DEBUG_SELFTEST != FALSE)
    DEBUG_SET_IO_B();
#endif /* (_DEBUG_SELFTEST != FALSE) */
    for (u16SelfTestIdx = 0U;
         (g_e8ErrorElectric == (uint8_t)C_ERR_NONE) &&
         (u16SelfTestIdx < (sizeof(c_au16DrvCfgSelfTestA) / sizeof(c_au16DrvCfgSelfTestA[0])));
         u16SelfTestIdx++)
    {
        uint16_t u16DC;
        IO_PWM_MASTER1_CTRL = B_PWM_MASTER1_STOP;
        u16DC = ((uint16_t *)&adcMotorPreTest.PhaseU_Voltage)[u16SelfTestIdx];  /*lint !e661 !e662 */
        if ( (u16DC - Get_HighVoltOffset()) < u16Vds)
        {
            /* Near ground */
            IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR;  /* Initialise the master pre-scaler ratio (Fck/8) */
            u16DC = PWM_REG_PERIOD - C_MIN_PWM_PULSE;                           /* Minimum PWM pulse to measure Vphase */
        }
        else if (u16DC > (adcMotorPreTest.MotorDriverVoltageB - u16Vds) )
        {
            /* Near supply */
            IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR | B_PWM_MASTER1_POL;  /* Initialise the master pre-scaler ratio (Fck/8) */
            u16DC = C_MIN_PWM_PULSE;                                            /* Minimum PWM pulse to measure Vphase */
        }
        else
        {
            continue;
        }
        u16DC = u16DC / 2U;
        IO_PWM_MASTER1_LT = u16DC;                                              /* Copy the results into the PWM register for phase */
        IO_PWM_MASTER1_CTRL = B_PWM_MASTER1_START;

        HAL_ADC_StopSafe();                                                     /* Clear the ADC control register */
        pu16Src = &l_au16AdcSource[0];
        *pu16Src++ = (uint16_t)&adcMotorPinTest.Temperature;
        *pu16Src++ = tAdcSelfTest_PreA[0].u16;
        *pu16Src++ = tAdcSelfTest_PreA[1].u16;
        *pu16Src++ = tAdcSelfTestA[u16SelfTestIdx].u16;
        *pu16Src = C_ADC_EOS;

        HAL_ADC_Setup(/*B_ADC_ADC_WIDTH |*/                                     /* Setup Self Test measurement */
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
        HAL_ADC_ClearErrors();                                                  /* Prior to start, first clear any error flag and enable Triggers */

        HAL_PWM_MasterPendClear();                                              /* Clear PWM-module Master1-End IRQ's */
        HAL_PWM_MasterPendWait();                                               /* Wait for PWM Master PEND-flag */
#if defined (__MLX81160__)
#if (_SUPPORT_DUAL_BLDC != FALSE)
#error "Error: Self-test not supported"
#elif (_SUPPORT_1ST_BLDC != FALSE)
        IO_PORT_DRV1_CTRL = c_au16DrvCfgSelfTestA[u16SelfTestIdx];              /* R-S-T */
#else  /* (_SUPPORT_1ST_BLDC != FALSE) */
        IO_PORT_DRV1_CTRL = c_au16DrvCfgSelfTestA[u16SelfTestIdx] & 0xF000U;    /* U */
        IO_PORT_DRV2_CTRL = c_au16DrvCfgSelfTestA[u16SelfTestIdx] & 0x00FFU;    /* V-W */
#endif /* (_SUPPORT_1ST_BLDC != FALSE) */
#else  /* defined (__MLX81160__) */
        IO_PORT_DRV_CTRL = c_au16DrvCfgSelfTestA[u16SelfTestIdx];
#endif /* defined (__MLX81160__) */

        HAL_ADC_Start();                                                        /* Start ADC */
#if (_DEBUG_SELFTEST != FALSE)
        DEBUG_CLR_IO_B();
#endif /* (_DEBUG_SELFTEST != FALSE) */
#if (_SUPPORT_WHILE_RETRY_LIMITED != FALSE)
        uint16_t u16Retries = 100U;
        while ( (IO_ADC_CTRL & B_ADC_START) != 0U)
        {
            if (--u16Retries == 0U)
            {
#if (_SUPPORT_LOG_ERRORS != FALSE)
                SetLastError(C_ERR_TO_MDST_PIN);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                break;
            }
            DELAY_US(5);
        }                  /* Wait for ADC result (Time-out?) */
#else  /* (_SUPPORT_WHILE_RETRY_LIMITED != FALSE) */
        while ( (IO_ADC_CTRL & B_ADC_START) != 0U) {}                           /* Wait for ADC result (Time-out?) */
#endif /* _SUPPORT_WHILE_RETRY_LIMITED */
#if (_DEBUG_SELFTEST != FALSE)
        DEBUG_SET_IO_B();
#endif /* (_DEBUG_SELFTEST != FALSE) */

        if (g_e8ErrorElectric != (uint8_t)C_ERR_NONE)
        {
            /* Over-current trigger; Phase makes short with supply or Ground */
            MotorDriverPermanentError( (uint8_t)C_ERR_PERMANENT);               /* Permanent Electric Failure */
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_SELFTEST_A | (C_ERR_EXT | ((u16SelfTestIdx & 0x0FU) << 8)));   /* Over current (short) */
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            break;
        }
        u16DC = ((uint16_t *)&adcMotorPreTest.PhaseU_Voltage)[u16SelfTestIdx];   /*lint !e661 !e662 */
        if ( (u16DC - Get_HighVoltOffset()) < u16Vds)
        {
            /* Near ground */
            if (adcMotorPinTest.PhaseVoltage < (adcMotorPinTest.MotorDriverVoltage - u16Vds) )
            {
                MotorDriverPermanentError( (uint8_t)C_ERR_MOTOR_TEST_VPH);      /* Permanent Electric Failure */
#if (_SUPPORT_LOG_ERRORS != FALSE)
                SetLastError(C_ERR_SELFTEST_B | (C_ERR_EXT | ((u16SelfTestIdx & 0x0FU) << 8)));  /* Phase to ground short */
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                break;
            }
        }
        else if (u16DC > (adcMotorPreTest.MotorDriverVoltageB - u16Vds) )
        {
            if (adcMotorPinTest.PhaseVoltage > u16Vds)
            {
                MotorDriverPermanentError( (uint8_t)C_ERR_MOTOR_TEST_VPH);      /* Permanent Electric Failure */
#if (_SUPPORT_LOG_ERRORS != FALSE)
                SetLastError(C_ERR_SELFTEST_B | (C_ERR_EXT | ((u16SelfTestIdx & 0x0FU) << 8)));  /* Phase to ground short */
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                break;
            }
        }
    }
#if (_DEBUG_SELFTEST != FALSE)
    DEBUG_CLR_IO_B();
#endif /* (_DEBUG_SELFTEST != FALSE) */

    if (g_e8ErrorElectric == (uint8_t)C_ERR_NONE)
    {
        uint16_t *pu16SBase = (uint16_t *)&tAdcSelfTestB[0];
        pu16Src = &l_au16AdcSource[0];
        *pu16Src++ = (uint16_t)&adcMotorSelfTest.MotorDriverVoltage;
        *pu16Src++ = *pu16SBase++;
        *pu16Src++ = *pu16SBase++;
        *pu16Src++ = *pu16SBase++;
        *pu16Src++ = *pu16SBase++;
        *pu16Src++ = *pu16SBase++;
        *pu16Src++ = *pu16SBase++;
        *pu16Src++ = *pu16SBase++;
        *pu16Src++ = *pu16SBase++;
        *pu16Src++ = *pu16SBase++;
        *pu16Src++ = *pu16SBase++;
        *pu16Src++ = *pu16SBase++;
    }

    /* Test for open connections, damaged coil(s) */
#if (_DEBUG_SELFTEST != FALSE)
    DEBUG_SET_IO_B();
#endif /* (_DEBUG_SELFTEST != FALSE) */
    for (u16SelfTestIdx = 0U;
         (g_e8ErrorElectric == (uint8_t)C_ERR_NONE) &&
         (u16SelfTestIdx < (sizeof(c_au16DrvCfgSelfTestB) / sizeof(c_au16DrvCfgSelfTestB[0])));
         u16SelfTestIdx++)
    {
        uint16_t u16LT;
        uint16_t u16DeltaZ;

        IO_PWM_MASTER1_CTRL = B_PWM_MASTER1_STOP;
        IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR;  /* Initialise the master pre-scaler ratio (Fck/8) */
        IO_ADC_SBASE = (uint16_t)&l_au16AdcSource[0];

#if (_SUPPORT_COIL_UNIT_10mR != FALSE)
#if (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE)
        if (g_u16MotorCoilResistanceTotal_NV > 1400U)
#else  /* (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE) */
        if (NV_MOTOR_COIL_RTOT > 255U)   /*lint !e685 */
#endif /* (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE) */
#elif (_SUPPORT_COIL_UNIT_100mR != FALSE)
#if (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE)
        if (g_u16MotorCoilResistanceTotal_NV > 140U)
#else  /* (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE) */
        if (NV_MOTOR_COIL_RTOT > 140U)
#endif /* (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE) */
#else
#if (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE)
        if (g_u16MotorCoilResistanceTotal_NV > 14U)
#else  /* (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE) */
        if (NV_MOTOR_COIL_RTOT > 14U)
#endif /* (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE) */
#endif
        {
            u16LT = (PWM_REG_PERIOD >> 2);                                      /* Approximate 12.5% */
        }
#if (_SUPPORT_COIL_UNIT_10mR != FALSE)
#if (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE)
        else if (g_u16MotorCoilResistanceTotal_NV > 700U)
#else  /* (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE) */
        else if (NV_MOTOR_COIL_RTOT > 255U)   /*lint !e685 */
#endif /* (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE) */
#elif (_SUPPORT_COIL_UNIT_100mR != FALSE)
#if (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE)
        else if (g_u16MotorCoilResistanceTotal_NV > 70U)
#else  /* (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE) */
        else if (NV_MOTOR_COIL_RTOT > 70U)
#endif /* (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE) */
#else
#if (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE)
        else if (g_u16MotorCoilResistanceTotal_NV > 7U)
#else  /* (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE) */
        else if (NV_MOTOR_COIL_RTOT > 7U)
#endif /* (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE) */
#endif
        {
            u16LT = (PWM_REG_PERIOD >> 3);                                      /* Approximate 6.25% */
        }
        else
        {
            u16LT = (PWM_REG_PERIOD >> 4);                                      /* Approximate 3.125% */
        }
        if (u16SelfTestIdx >= 3U)
        {
            /* One phase PWM, One phase LOW, One phase TRI-state */
            u16LT = (PWM_REG_PERIOD >> 1) - u16LT;
        }
        IO_PWM_MASTER1_LT = u16LT;
        IO_PWM_MASTER1_CTRL = B_PWM_MASTER1_START;

#if defined (__MLX81160__)
#if (_SUPPORT_DUAL_BLDC != FALSE)
#error "Error: Self-test not supported"
#elif (_SUPPORT_1ST_BLDC != FALSE)
        IO_PORT_DRV1_CTRL = c_au16DrvCfgSelfTestB[u16SelfTestIdx];
#else  /* (_SUPPORT_1ST_BLDC != FALSE) */
        IO_PORT_DRV1_CTRL = (c_au16DrvCfgSelfTestB[u16SelfTestIdx] & M_PORT_DRV1_CTRL_DRV3_CTRL) |
                            (C_PORT_DRV1_CTRL_DRV2_TRISTATE | C_PORT_DRV1_CTRL_DRV1_TRISTATE |
                             C_PORT_DRV1_CTRL_DRV0_TRISTATE);
        IO_PORT_DRV2_CTRL =
            (c_au16DrvCfgSelfTestB[u16SelfTestIdx] & (M_PORT_DRV2_CTRL_DRV5_CTRL | M_PORT_DRV2_CTRL_DRV4_CTRL));
#endif /* (_SUPPORT_1ST_BLDC != FALSE) */
#else  /* defined (__MLX81160__) */
        IO_PORT_DRV_CTRL = c_au16DrvCfgSelfTestB[u16SelfTestIdx];
#endif /* defined (__MLX81160__) */
        DELAY(2U * C_DELAY_mPWM);                                               /* Allow phases to settle */

#if (_DEBUG_SELFTEST != FALSE)
        DEBUG_SET_IO_A();
#endif /* (_DEBUG_SELFTEST != FALSE) */
        HAL_ADC_Start();                                                        /* Start ADC */

        /* This takes about 4 Motor PWM-periods per self-test */
#if (_SUPPORT_WHILE_RETRY_LIMITED != FALSE)
        uint16_t u16Retries = 100U;
        while ( (IO_ADC_CTRL & B_ADC_START) != 0U)
        {
            if (--u16Retries == 0U)
            {
#if (_SUPPORT_LOG_ERRORS != FALSE)
                SetLastError(C_ERR_TO_MDST_COIL_SHORT);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                break;
            }
            DELAY_US(5);
        }                  /* Wait for ADC result (Time-out?) */
#else  /* (_SUPPORT_WHILE_RETRY_LIMITED != FALSE) */
        while ( (IO_ADC_CTRL & B_ADC_START) != 0U) {}                           /* Wait for ADC result (Time-out?) */
#endif /* (_SUPPORT_WHILE_RETRY_LIMITED != FALSE) */
#if (_DEBUG_SELFTEST != FALSE)
        DEBUG_CLR_IO_A();
#endif /* (_DEBUG_SELFTEST != FALSE) */

        if (u16SelfTestIdx < 3U)
        {
#if defined (__MLX81160__)
#if (_SUPPORT_DUAL_BLDC != FALSE)
#error "Error: Self-test not supported"
#elif (_SUPPORT_1ST_BLDC != FALSE)
            DRVCFG_SUP_RST();
#else  /* (_SUPPORT_1ST_BLDC != FALSE) */
            DRVCFG_SUP_UVW();
#endif /* (_SUPPORT_1ST_BLDC != FALSE) */
#else  /* defined (__MLX81160__) */
/*            DRVCFG_SUP_UVW(); */
            DRVCFG_TRI_UVW();
#endif /* defined (__MLX81160__) */
        }
        else
        {
#if defined (__MLX81160__)
#if (_SUPPORT_DUAL_BLDC != FALSE)
#error "Error: Self-test not supported"
#elif (_SUPPORT_1ST_BLDC != FALSE)
            DRVCFG_GND_RST();
#else  /* (_SUPPORT_1ST_BLDC != FALSE) */
            DRVCFG_GND_UVW();
#endif /* (_SUPPORT_1ST_BLDC != FALSE) */
#else  /* defined (__MLX81160__) */
/*          DRVCFG_GND_UVW(); */
            DRVCFG_TRI_UVW();
#endif /* defined (__MLX81160__) */
        }

        if (g_e8ErrorElectric != (uint8_t)C_ERR_NONE)                           /* Over-current ? */
        {
            /* Over-current trigger; Phase makes short with other phase */
            MotorDriverPermanentError( (uint8_t)C_ERR_PERMANENT);               /* Permanent Electric Failure */
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_SELFTEST_A | (C_ERR_EXT | ((u16SelfTestIdx & 0x0FU) << 8)));  /* Over current (short) */
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            break;
        }

        /* Motor Driver Supply check (MMP190210-1) */
        if (u16Vsm == 0U)
        {
            u16Vsm = adcMotorSelfTest.MotorDriverVoltage;
        }
        else if ( (u16Vsm > adcMotorSelfTest.MotorDriverVoltage) &&
                  ((u16Vsm - adcMotorSelfTest.MotorDriverVoltage) > C_VSM_STABLE) )
        {
            /* VSM Voltage not stable; Supply decrease by 2V */
            g_u8ForceMotorDriverSelfTest = TRUE;
            break;
        }
        else if ( (u16Vsm < adcMotorSelfTest.MotorDriverVoltage) &&
                  ((adcMotorSelfTest.MotorDriverVoltage - u16Vsm) > C_VSM_STABLE) )
        {
            /* VSM Voltage not stable; Supply increase by 2V */
            g_u8ForceMotorDriverSelfTest = TRUE;
            break;
        }
        else
        {
            /* Nothing; It's okay */
        }

        {
            uint16_t u16RefVoltage = adcMotorSelfTest.MotorDriverVoltage - u16Vds;
            if (u16SelfTestIdx < 3U)
            {
                /* Check HighVoltage Phase is above (Vsup - Vds) */
                if ( (adcMotorSelfTest.PhaseU_Voltage50 < u16RefVoltage) ||
                     (adcMotorSelfTest.PhaseV_Voltage50 < u16RefVoltage) ||
                     (adcMotorSelfTest.PhaseW_Voltage50 < u16RefVoltage) )
                {
                    /* Permanent Electric Failure */
                    MotorDriverPermanentError( (uint8_t)C_ERR_MOTOR_TEST_VPH);
#if (_SUPPORT_LOG_ERRORS != FALSE)
                    SetLastError(C_ERR_SELFTEST_C | (C_ERR_EXT | ((u16SelfTestIdx & 0x0FU) << 8)));
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                }
            }
            else
            {
                /* Check HighVoltage Phase is below Vds */
                if ( (adcMotorSelfTest.PhaseU_Voltage100 > (C_OADC_VSMF + u16Vds)) ||
                     (adcMotorSelfTest.PhaseV_Voltage100 > (C_OADC_VSMF + u16Vds)) ||
                     (adcMotorSelfTest.PhaseW_Voltage100 > (C_OADC_VSMF + u16Vds)) )
                {
                    /* Permanent Electric Failure */
                    MotorDriverPermanentError( (uint8_t)C_ERR_MOTOR_TEST_VPH);
#if (_SUPPORT_LOG_ERRORS != FALSE)
                    SetLastError(C_ERR_SELFTEST_C | (C_ERR_EXT | ((u16SelfTestIdx & 0x0FU) << 8)));
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                }
            }

            if (u16SelfTestIdx < 3U)
            {
                if (adcMotorSelfTest.MotorDriverCurrent100 > Get_CurrentZeroOffset() )
                {
                    u16DeltaZ = adcMotorSelfTest.MotorDriverCurrent100 - Get_CurrentZeroOffset();
                }
                else
                {
                    u16DeltaZ = Get_CurrentZeroOffset() - adcMotorSelfTest.MotorDriverCurrent100;
                }
            }
            else
            {
                if (adcMotorSelfTest.MotorDriverCurrent50 > Get_CurrentZeroOffset() )
                {
                    u16DeltaZ = adcMotorSelfTest.MotorDriverCurrent50 - Get_CurrentZeroOffset();
                }
                else
                {
                    u16DeltaZ = Get_CurrentZeroOffset() - adcMotorSelfTest.MotorDriverCurrent50;
                }
            }
            if (u16DeltaZ < 5U)
            {
                /* Less than 5 ADC -LSB's (< 1%) */
                MotorDriverPermanentError( (uint8_t)(C_ERR_PERMANENT | C_ERR_MOTOR_ZERO_CURRENT));  /* Permanent Electric Failure */
#if (_SUPPORT_LOG_ERRORS != FALSE)
                SetLastError(C_ERR_SELFTEST_D | (C_ERR_EXT | ((u16SelfTestIdx & 0x0FU) << 8)));
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            }

            switch (u16SelfTestIdx)
            {
                case 0U:
                    /* PhW = H, PhV = PWM, PhU = Z */
                    if ( (adcMotorSelfTest.PhaseW_Voltage100 < u16RefVoltage) ||
                         (adcMotorSelfTest.PhaseV_Voltage100 > (C_OADC_VSMF + u16Vds)) ||
                         (adcMotorSelfTest.PhaseU_Voltage100 >= adcMotorSelfTest.PhaseW_Voltage100) || /* High */
                         (adcMotorSelfTest.PhaseU_Voltage100 <= adcMotorSelfTest.PhaseV_Voltage100) ) /* PWM-Low */
                    {
                        /* Permanent Electric Failure */
                        MotorDriverPermanentError( (uint8_t)C_ERR_MOTOR_TEST_VPH);
#if (_SUPPORT_LOG_ERRORS != FALSE)
                        if (adcMotorSelfTest.PhaseV_Voltage100 < u16RefVoltage)
                        {
                            SetLastError(C_ERR_SELFTEST_E | (C_ERR_EXT | 0x0000U));
                        }
                        else if (adcMotorSelfTest.PhaseU_Voltage100 > (C_OADC_VSMF + u16Vds) )
                        {
                            SetLastError(C_ERR_SELFTEST_E | (C_ERR_EXT | 0x0800U));
                        }
                        else if (adcMotorSelfTest.PhaseW_Voltage100 >= adcMotorSelfTest.PhaseV_Voltage100)  /* High */
                        {
                            SetLastError(C_ERR_SELFTEST_E | (C_ERR_EXT | 0x1000U));
                        }
                        else if (adcMotorSelfTest.PhaseW_Voltage100 <= adcMotorSelfTest.PhaseU_Voltage100)  /* PWM-Low */
                        {
                            SetLastError(C_ERR_SELFTEST_E | (C_ERR_EXT | 0x1800U));
                        }
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                    }
                    else
                    {
                        u16PhaseVoltageZ[0] = adcMotorSelfTest.PhaseU_Voltage100;
                    }
                    break;
                case 1U:
                    /* PhW = Z, PhV = H, PhU = PWM */
                    if ( (adcMotorSelfTest.PhaseV_Voltage100 < u16RefVoltage) ||
                         (adcMotorSelfTest.PhaseU_Voltage100 > (C_OADC_VSMF + u16Vds)) ||
                         (adcMotorSelfTest.PhaseW_Voltage100 >= adcMotorSelfTest.PhaseV_Voltage100) || /* High */
                         (adcMotorSelfTest.PhaseW_Voltage100 <= adcMotorSelfTest.PhaseU_Voltage100) ) /* PWM-Low */
                    {
                        /* Permanent Electric Failure */
                        MotorDriverPermanentError( (uint8_t)C_ERR_MOTOR_TEST_VPH);
#if (_SUPPORT_LOG_ERRORS != FALSE)
                        if (adcMotorSelfTest.PhaseU_Voltage100 < u16RefVoltage)
                        {
                            SetLastError(C_ERR_SELFTEST_E | (C_ERR_EXT | 0x0100U));
                        }
                        else if (adcMotorSelfTest.PhaseW_Voltage100 > (C_OADC_VSMF + u16Vds) )
                        {
                            SetLastError(C_ERR_SELFTEST_E | (C_ERR_EXT | 0x0900U));
                        }
                        else if (adcMotorSelfTest.PhaseV_Voltage100 >= adcMotorSelfTest.PhaseU_Voltage100)  /* High */
                        {
                            SetLastError(C_ERR_SELFTEST_E | (C_ERR_EXT | 0x1100U));
                        }
                        else if (adcMotorSelfTest.PhaseV_Voltage100 <= adcMotorSelfTest.PhaseW_Voltage100)  /* PWM-Low */
                        {
                            SetLastError(C_ERR_SELFTEST_E | (C_ERR_EXT | 0x1900U));
                        }
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                    }
                    else
                    {
                        u16PhaseVoltageZ[1] = adcMotorSelfTest.PhaseW_Voltage100;
                    }
                    break;
                case 2U:
                    /* PhW = PWM, PhV = Z, PhU = H */
                    if ( (adcMotorSelfTest.PhaseU_Voltage100 < u16RefVoltage) ||
                         (adcMotorSelfTest.PhaseW_Voltage100 > (C_OADC_VSMF + u16Vds)) ||
                         (adcMotorSelfTest.PhaseV_Voltage100 >= adcMotorSelfTest.PhaseU_Voltage100) || /* High */
                         (adcMotorSelfTest.PhaseV_Voltage100 <= adcMotorSelfTest.PhaseW_Voltage100) ) /* PWM-Low */
                    {
                        /* Permanent Electric Failure */
                        MotorDriverPermanentError( (uint8_t)C_ERR_MOTOR_TEST_VPH);
#if (_SUPPORT_LOG_ERRORS != FALSE)
                        if (adcMotorSelfTest.PhaseW_Voltage100 < u16RefVoltage)
                        {
                            SetLastError(C_ERR_SELFTEST_E | (C_ERR_EXT | 0x0200U));
                        }
                        else if (adcMotorSelfTest.PhaseV_Voltage100 > (C_OADC_VSMF + u16Vds) )
                        {
                            SetLastError(C_ERR_SELFTEST_E | (C_ERR_EXT | 0x0A00U));
                        }
                        else if (adcMotorSelfTest.PhaseU_Voltage100 >= adcMotorSelfTest.PhaseW_Voltage100)  /* High */
                        {
                            SetLastError(C_ERR_SELFTEST_E | (C_ERR_EXT | 0x1200U));
                        }
                        else if (adcMotorSelfTest.PhaseU_Voltage100 <= adcMotorSelfTest.PhaseV_Voltage100)  /* PWM-Low */
                        {
                            SetLastError(C_ERR_SELFTEST_E | (C_ERR_EXT | 0x1A00U));
                        }
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                    }
                    else
                    {
                        u16PhaseVoltageZ[2] = adcMotorSelfTest.PhaseV_Voltage100;
                    }
                    break;
                case 3U:
                    /* PhW = PWM, PhV = L, PhU = Z */
                    if ( (adcMotorSelfTest.PhaseW_Voltage50 < u16RefVoltage) ||
                         (adcMotorSelfTest.PhaseV_Voltage50 > (C_OADC_VSMF + u16Vds)) ||
                         (adcMotorSelfTest.PhaseU_Voltage50 >= adcMotorSelfTest.PhaseW_Voltage50) || /* PWM-High */
                         (adcMotorSelfTest.PhaseU_Voltage50 <= adcMotorSelfTest.PhaseV_Voltage50) ) /* Low */
                    {
                        MotorDriverPermanentError( (uint8_t)C_ERR_MOTOR_TEST_VPH);   /* Permanent Electric Failure */
#if (_SUPPORT_LOG_ERRORS != FALSE)
                        if (adcMotorSelfTest.PhaseU_Voltage50 < u16RefVoltage)
                        {
                            SetLastError(C_ERR_SELFTEST_E | (C_ERR_EXT | 0x0300U));
                        }
                        else if (adcMotorSelfTest.PhaseV_Voltage50 > (C_OADC_VSMF + u16Vds) )
                        {
                            SetLastError(C_ERR_SELFTEST_E | (C_ERR_EXT | 0x0B00U));
                        }
                        else if (adcMotorSelfTest.PhaseW_Voltage50 >= adcMotorSelfTest.PhaseU_Voltage50)  /* PWM-High */
                        {
                            SetLastError(C_ERR_SELFTEST_E | (C_ERR_EXT | 0x1300U));
                        }
                        else if (adcMotorSelfTest.PhaseW_Voltage50 <= adcMotorSelfTest.PhaseV_Voltage50)  /* Low */
                        {
                            SetLastError(C_ERR_SELFTEST_E | (C_ERR_EXT | 0x1B00U));
                        }
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                    }
                    if (u16PhaseVoltageZ[0] > adcMotorSelfTest.PhaseU_Voltage50)
                    {
                        u16DeltaZ = u16PhaseVoltageZ[0] - adcMotorSelfTest.PhaseU_Voltage50;
                    }
                    else
                    {
                        u16DeltaZ = adcMotorSelfTest.PhaseU_Voltage50 - u16PhaseVoltageZ[0];
                    }
                    if (u16DeltaZ > (adcMotorSelfTest.MotorDriverVoltage >> 2) )
                    {
                        MotorDriverPermanentError( (uint8_t)C_ERR_MOTOR_TEST_VPH);  /* Permanent Electric Failure */
#if (_SUPPORT_LOG_ERRORS != FALSE)
                        SetLastError(C_ERR_SELFTEST_F | (C_ERR_EXT | 0x0300U));
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                    }
                    break;
                case 4U:
                    /* PhW = L, PhV = Z, PhU = PWM */
                    if ( (adcMotorSelfTest.PhaseU_Voltage50 < u16RefVoltage) ||
                         (adcMotorSelfTest.PhaseW_Voltage50 > (C_OADC_VSMF + u16Vds)) ||
                         (adcMotorSelfTest.PhaseV_Voltage50 >= adcMotorSelfTest.PhaseU_Voltage50) || /* PWM-High */
                         (adcMotorSelfTest.PhaseV_Voltage50 <= adcMotorSelfTest.PhaseW_Voltage50) ) /* Low */
                    {
                        MotorDriverPermanentError( (uint8_t)C_ERR_MOTOR_TEST_VPH);  /* Permanent Electric Failure */
#if (_SUPPORT_LOG_ERRORS != FALSE)
                        if (adcMotorSelfTest.PhaseW_Voltage50 < u16RefVoltage)
                        {
                            SetLastError(C_ERR_SELFTEST_E | (C_ERR_EXT | 0x0400U));
                        }
                        else if (adcMotorSelfTest.PhaseU_Voltage50 > (C_OADC_VSMF + u16Vds) )
                        {
                            SetLastError(C_ERR_SELFTEST_E | (C_ERR_EXT | 0x0C00U));
                        }
                        else if (adcMotorSelfTest.PhaseV_Voltage50 >= adcMotorSelfTest.PhaseW_Voltage50)  /* PWM-High */
                        {
                            SetLastError(C_ERR_SELFTEST_E | (C_ERR_EXT | 0x1400U));
                        }
                        else if (adcMotorSelfTest.PhaseV_Voltage50 <= adcMotorSelfTest.PhaseU_Voltage50)  /* Low */
                        {
                            SetLastError(C_ERR_SELFTEST_E | (C_ERR_EXT | 0x1C00U));
                        }
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                    }
                    if (u16PhaseVoltageZ[2] > adcMotorSelfTest.PhaseV_Voltage50)
                    {
                        u16DeltaZ = u16PhaseVoltageZ[2] - adcMotorSelfTest.PhaseV_Voltage50;
                    }
                    else
                    {
                        u16DeltaZ = adcMotorSelfTest.PhaseV_Voltage50 - u16PhaseVoltageZ[2];
                    }
                    if (u16DeltaZ > (adcMotorSelfTest.MotorDriverVoltage >> 2) )
                    {
                        MotorDriverPermanentError( (uint8_t)C_ERR_MOTOR_TEST_VPH);  /* Permanent Electric Failure */
#if (_SUPPORT_LOG_ERRORS != FALSE)
                        SetLastError(C_ERR_SELFTEST_F | (C_ERR_EXT | 0x0400U));  /* Low-ohmic phase V */
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                    }
                    break;
                case 5U:
                    /* PhW = Z, PhV = PWM, PhU = L */
                    if ( (adcMotorSelfTest.PhaseV_Voltage50 < u16RefVoltage) ||
                         (adcMotorSelfTest.PhaseU_Voltage50 > (C_OADC_VSMF + u16Vds)) ||
                         (adcMotorSelfTest.PhaseW_Voltage50 >= adcMotorSelfTest.PhaseV_Voltage50) || /* PWM-High */
                         (adcMotorSelfTest.PhaseW_Voltage50 <= adcMotorSelfTest.PhaseU_Voltage50) ) /* Low */
                    {
                        MotorDriverPermanentError( (uint8_t)C_ERR_MOTOR_TEST_VPH);  /* Permanent Electric Failure */
#if (_SUPPORT_LOG_ERRORS != FALSE)
                        if (adcMotorSelfTest.PhaseV_Voltage50 < u16RefVoltage)
                        {
                            SetLastError(C_ERR_SELFTEST_E | (C_ERR_EXT | 0x0500U));
                        }
                        else if (adcMotorSelfTest.PhaseW_Voltage50 > (C_OADC_VSMF + u16Vds) )
                        {
                            SetLastError(C_ERR_SELFTEST_E | (C_ERR_EXT | 0x0D00U));
                        }
                        else if (adcMotorSelfTest.PhaseU_Voltage50 >= adcMotorSelfTest.PhaseV_Voltage50)  /* PWM-High */
                        {
                            SetLastError(C_ERR_SELFTEST_E | (C_ERR_EXT | 0x1500U));
                        }
                        else if (adcMotorSelfTest.PhaseU_Voltage50 <= adcMotorSelfTest.PhaseW_Voltage50)  /* Low */
                        {
                            SetLastError(C_ERR_SELFTEST_E | (C_ERR_EXT | 0x1D00U));
                        }
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                    }
                    if (u16PhaseVoltageZ[1] > adcMotorSelfTest.PhaseW_Voltage50)
                    {
                        u16DeltaZ = u16PhaseVoltageZ[1] - adcMotorSelfTest.PhaseW_Voltage50;
                    }
                    else
                    {
                        u16DeltaZ = adcMotorSelfTest.PhaseW_Voltage50 - u16PhaseVoltageZ[1];
                    }
                    if (u16DeltaZ > (adcMotorSelfTest.MotorDriverVoltage >> 2) )
                    {
                        MotorDriverPermanentError( (uint8_t)C_ERR_MOTOR_TEST_VPH);  /* Permanent Electric Failure */
#if (_SUPPORT_LOG_ERRORS != FALSE)
                        SetLastError(C_ERR_SELFTEST_F | (C_ERR_EXT | 0x0500U));  /* Low-ohmic phase U */
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                    }
                    break;
            } /* End Switch */
              /* Time-constant[s] = L[H]/R[R]; t[us] = L[uH]/R[R] */
#if (_SUPPORT_COIL_UNIT_10mR != FALSE)
            DELAY_US( (uint16_t)(((100UL * C_COIL_L) / C_COIL_R) / 8U));
#elif (_SUPPORT_COIL_UNIT_100mR != FALSE)
            DELAY_US( (uint16_t)(((10UL * C_COIL_L) / C_COIL_R) / 8U));
#else  /* (_SUPPORT_COIL_UNIT_100mR != FALSE) */
            DELAY_US( (uint16_t)((C_COIL_L / C_COIL_R) / 8U) );
#endif /* (_SUPPORT_COIL_UNIT_100mR != FALSE) */
        }
    }
#if (_DEBUG_SELFTEST != FALSE)
    DEBUG_CLR_IO_B();
#endif /* (_DEBUG_SELFTEST != FALSE) */
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
    if (((g_e8ErrorElectric & (uint8_t)C_ERR_PERMANENT) != 0U) ||               /* MMP220729-1: In case of permanent error or ... */
        (l_u8MotorHoldingCurrState == FALSE) )                                  /* ... no holding current required */
    {
        MotorDriverConfig(FALSE);                                               /* Disable Motor Driver */
        l_u8MotorHoldingCurrState = FALSE;
    }
    else
    {
        /* MotorDriverInit() has already enabled MotorDriverConfig(); Remain it active */
    }
#else  /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
    MotorDriverConfig(FALSE);                                                   /* Disable Motor Driver */
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
#if (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_INDEPENDENT_GND)
    IO_PWM_SLAVE1_CTRL = B_PWM_SLAVE1_SLAVE | C_PWM_SLAVE1_MODE_MIRROR;
    IO_PWM_SLAVE2_CTRL = B_PWM_SLAVE2_SLAVE | C_PWM_SLAVE2_MODE_MIRROR | B_PWM_SLAVE2_POL;
#endif /* (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_INDEPENDENT_GND) */
    IO_PWM_MASTER1_CTRL = (PWM_PRESCALER << 8) | C_PWM_MASTER1_MODE_MIRROR;     /* Restore the PWM-master */
    IO_PWM_SLAVE1_LT = PWM_SCALE_OFFSET;                                        /* 50% PWM duty cycle for phase U */
    IO_PWM_SLAVE2_LT = PWM_SCALE_OFFSET;                                        /* 50% PWM duty cycle for phase V */
    IO_PWM_MASTER1_LT = PWM_SCALE_OFFSET;                                       /* Master must be modified at last (value is not important) */
    IO_PWM_SLAVE2_CMP = u16Pwm2Storage;                                         /* Restore PWM2 ADC trigger CMP time */

} /* End of MotorDriverSelfTest() */
#elif (C_MOTOR_PHASES == 4)
/*! ADC Pre-SelfTest structure 4-Phase */
typedef struct
{
    uint16_t PhaseVoltageU;                                                     /*!< Phase-U voltage */
    uint16_t PhaseVoltageV;                                                     /*!< Phase-V voltage */
    uint16_t PhaseVoltageW;                                                     /*!< Phase-W voltage */
    uint16_t PhaseVoltageT;                                                     /*!< Phase-T voltage */
    uint16_t MotorDriverVoltage;                                                /*!< Filtered Motor Driver Voltage */
    uint16_t AdcCRC;                                                            /*!< ADC CRC */
} T_ADC_PRE_SELFTEST_4PH;

/*! ADC SelfTest structure 4-Phase (A) */
typedef struct
{
    uint16_t IntTemperatureSensor;                                              /*!< IC Junction Temperature (dummy) */
    uint16_t MotorDriverVoltage;                                                /*!< Unfiltered Motor Driver Voltage */
    uint16_t NeighbourPhaseVoltage;                                             /*!< Neighbour phase voltage */
    uint16_t PhaseVoltage;                                                      /*!< Phase-voltage */
} T_ADC_SELFTEST_4PH_A;

/*! ADC SelfTest structure 4-Phase (B) */
typedef struct
{
    uint16_t IntTemperatureSensor;                                              /*!< IC Junction Temperature (dummy) */
    uint16_t PhaseVoltage;                                                      /*!< Phase-voltage */
    uint16_t MotorDriverVoltage;                                                /*!< Unfiltered Motor Driver Voltage */
} T_ADC_SELFTEST_4PH_B;

/*! ADC SelfTest structure 4-Phase (C) */
typedef struct
{
    uint16_t IntTemperatureSensor;                                              /*!< IC Junction Temperature (dummy) */
    uint16_t Phase_HighVoltage;                                                 /*!<  50%: Phase High-voltage */
    uint16_t Phase_LowVoltage;                                                  /*!< 100%: Phase Low-voltage */
    uint16_t MotorDriverVoltage;                                                /*!<  50%: Unfiltered Motor Driver Voltage */
    uint16_t MotorDriverCurrent;                                                /*!< 100%: Unfiltered Motor Driver Current */
} T_ADC_SELFTEST_4PH_C;

/*! Pre-test - Single pin short check */
static const uint16_t c_au16DrvCfgPreSelfTest[4] =
{
    (C_PORT_DRV_CTRL_DRV3_TRISTATE | C_PORT_DRV_CTRL_DRV2_TRISTATE | C_PORT_DRV_CTRL_DRV1_TRISTATE |
     C_PORT_DRV_CTRL_DRV0_MASTER1),                                             /* PhD = Z, PhC = Z, PhB = Z, PhA = PWM */
    (C_PORT_DRV_CTRL_DRV3_TRISTATE | C_PORT_DRV_CTRL_DRV2_TRISTATE | C_PORT_DRV_CTRL_DRV1_MASTER1 |
     C_PORT_DRV_CTRL_DRV0_TRISTATE),                                            /* PhD = Z, PhC = Z, PhB = PWM, PhA = Z */
    (C_PORT_DRV_CTRL_DRV3_TRISTATE | C_PORT_DRV_CTRL_DRV2_MASTER1 | C_PORT_DRV_CTRL_DRV1_TRISTATE |
     C_PORT_DRV_CTRL_DRV0_TRISTATE),                                            /* PhD = Z, PhC = PWM, PhB = Z, PhA = Z */
    (C_PORT_DRV_CTRL_DRV3_MASTER1 | C_PORT_DRV_CTRL_DRV2_TRISTATE | C_PORT_DRV_CTRL_DRV1_TRISTATE |
     C_PORT_DRV_CTRL_DRV0_TRISTATE)                                             /* PhD = PWM, PhC = Z, PhB = Z, PhA = Z */
};

/*! Set-Test - Part A: Single FET ON */
static const uint16_t c_au16DrvCfgSelfTestA4[8] =
{
    (C_PORT_DRV_CTRL_DRV3_TRISTATE | C_PORT_DRV_CTRL_DRV2_L | C_PORT_DRV_CTRL_DRV1_TRISTATE |
     C_PORT_DRV_CTRL_DRV0_TRISTATE),                                            /* PhD = Z, PhC = L, PhB = Z, PhA = Z */
    (C_PORT_DRV_CTRL_DRV3_TRISTATE | C_PORT_DRV_CTRL_DRV2_H | C_PORT_DRV_CTRL_DRV1_TRISTATE |
     C_PORT_DRV_CTRL_DRV0_TRISTATE),                                            /* PhD = Z, PhC = H, PhB = Z, PhA = Z */
    (C_PORT_DRV_CTRL_DRV3_TRISTATE | C_PORT_DRV_CTRL_DRV2_TRISTATE | C_PORT_DRV_CTRL_DRV1_L |
     C_PORT_DRV_CTRL_DRV0_TRISTATE),                                            /* PhD = Z, PhC = Z, PhB = L, PhA = Z */
    (C_PORT_DRV_CTRL_DRV3_TRISTATE | C_PORT_DRV_CTRL_DRV2_TRISTATE | C_PORT_DRV_CTRL_DRV1_H |
     C_PORT_DRV_CTRL_DRV0_TRISTATE),                                            /* PhD = Z, PhC = Z, PhB = H, PhA = Z */
    (C_PORT_DRV_CTRL_DRV3_TRISTATE | C_PORT_DRV_CTRL_DRV2_TRISTATE | C_PORT_DRV_CTRL_DRV1_TRISTATE |
     C_PORT_DRV_CTRL_DRV0_L),                                                   /* PhD = Z, PhC = Z, PhB = Z, PhA = L */
    (C_PORT_DRV_CTRL_DRV3_TRISTATE | C_PORT_DRV_CTRL_DRV2_TRISTATE | C_PORT_DRV_CTRL_DRV1_TRISTATE |
     C_PORT_DRV_CTRL_DRV0_H),                                                   /* PhD = Z, PhC = Z, PhB = Z, PhA = H */
    (C_PORT_DRV_CTRL_DRV3_L | C_PORT_DRV_CTRL_DRV2_TRISTATE | C_PORT_DRV_CTRL_DRV1_TRISTATE |
     C_PORT_DRV_CTRL_DRV0_TRISTATE),                                            /* PhD = L, PhC = Z, PhB = Z, PhA = Z */
    (C_PORT_DRV_CTRL_DRV3_H | C_PORT_DRV_CTRL_DRV2_TRISTATE | C_PORT_DRV_CTRL_DRV1_TRISTATE |
     C_PORT_DRV_CTRL_DRV0_TRISTATE)                                             /* PhD = H, PhC = Z, PhB = Z, PhA = Z */
};

#if (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UW_VT)
/*! Coil-short test (MMP201104-1) */
static const uint16_t c_au16DrvCfgSelfTestB4[4] =
{
    (C_PORT_DRV_CTRL_DRV3_TRISTATE | C_PORT_DRV_CTRL_DRV2_L | C_PORT_DRV_CTRL_DRV1_TRISTATE |
     C_PORT_DRV_CTRL_DRV0_MASTER1),                                             /* PhD = Z, PhC = L, PhB = Z, PhA = PWM */
    (C_PORT_DRV_CTRL_DRV3_L | C_PORT_DRV_CTRL_DRV2_TRISTATE | C_PORT_DRV_CTRL_DRV1_MASTER1 |
     C_PORT_DRV_CTRL_DRV0_TRISTATE),                                            /* PhD = L, PhC = Z, PhB = PWM, PhA = Z */
    (C_PORT_DRV_CTRL_DRV3_TRISTATE | C_PORT_DRV_CTRL_DRV2_MASTER1 | C_PORT_DRV_CTRL_DRV1_TRISTATE |
     C_PORT_DRV_CTRL_DRV0_L),                                                   /* PhD = Z, PhC = PWM, PhB = Z, PhA = L */
    (C_PORT_DRV_CTRL_DRV3_MASTER1 | C_PORT_DRV_CTRL_DRV2_TRISTATE | C_PORT_DRV_CTRL_DRV1_L |
     C_PORT_DRV_CTRL_DRV0_TRISTATE)                                             /* PhD = PWM, PhC = Z, PhB = L, PhA = Z */
};

/*! Coil-open test (MMP201104-1) */
static const uint16_t c_au16DrvCfgSelfTestC4[10] =
{
    (C_PORT_DRV_CTRL_DRV3_TRISTATE | C_PORT_DRV_CTRL_DRV2_H | C_PORT_DRV_CTRL_DRV1_TRISTATE |
     C_PORT_DRV_CTRL_DRV0_MASTER1),                                             /* PhD = Z, PhC = H, PhB = Z, PhA = PWM */
    (C_PORT_DRV_CTRL_DRV3_H | C_PORT_DRV_CTRL_DRV2_TRISTATE | C_PORT_DRV_CTRL_DRV1_MASTER1 |
     C_PORT_DRV_CTRL_DRV0_TRISTATE),                                            /* PhD = H, PhC = Z, PhB = PWM, PhA = Z */
    (C_PORT_DRV_CTRL_DRV3_TRISTATE | C_PORT_DRV_CTRL_DRV2_L | C_PORT_DRV_CTRL_DRV1_TRISTATE |
     C_PORT_DRV_CTRL_DRV0_MASTER1),                                             /* PhD = Z, PhC = L, PhB = Z, PhA = PWM */
    (C_PORT_DRV_CTRL_DRV3_L | C_PORT_DRV_CTRL_DRV2_TRISTATE | C_PORT_DRV_CTRL_DRV1_MASTER1 |
     C_PORT_DRV_CTRL_DRV0_TRISTATE),                                            /* PhD = L, PhC = Z, PhB = PWM, PhA = Z */
    (C_PORT_DRV_CTRL_DRV3_TRISTATE | C_PORT_DRV_CTRL_DRV2_MASTER1 | C_PORT_DRV_CTRL_DRV1_TRISTATE |
     C_PORT_DRV_CTRL_DRV0_H),                                                   /* PhD = Z, PhC = PWM, PhB = Z, PhA = H */
    (C_PORT_DRV_CTRL_DRV3_MASTER1 | C_PORT_DRV_CTRL_DRV2_TRISTATE | C_PORT_DRV_CTRL_DRV1_H |
     C_PORT_DRV_CTRL_DRV0_TRISTATE),                                            /* PhD = PWM, PhC = Z, PhB = H, PhA = Z */
    (C_PORT_DRV_CTRL_DRV3_TRISTATE | C_PORT_DRV_CTRL_DRV2_MASTER1 | C_PORT_DRV_CTRL_DRV1_TRISTATE |
     C_PORT_DRV_CTRL_DRV0_L),                                                   /* PhD = Z, PhC = PWM, PhB = Z, PhA = L */
    (C_PORT_DRV_CTRL_DRV3_MASTER1 | C_PORT_DRV_CTRL_DRV2_TRISTATE | C_PORT_DRV_CTRL_DRV1_L |
     C_PORT_DRV_CTRL_DRV0_TRISTATE),                                            /* PhD = PWM, PhC = Z, PhB = L, PhA = Z */
    (C_PORT_DRV_CTRL_DRV3_H | C_PORT_DRV_CTRL_DRV2_H | C_PORT_DRV_CTRL_DRV1_SLAVE1 | C_PORT_DRV_CTRL_DRV0_MASTER1),
    (C_PORT_DRV_CTRL_DRV3_SLAVE3 | C_PORT_DRV_CTRL_DRV2_SLAVE2 | C_PORT_DRV_CTRL_DRV1_H | C_PORT_DRV_CTRL_DRV0_H)
};
#elif (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_WT) || (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_TW)
/*! Coil-short test (MMP201104-1) */
static const uint16_t c_au16DrvCfgSelfTestB4[4] =
{
    (C_PORT_DRV_CTRL_DRV3_TRISTATE | C_PORT_DRV_CTRL_DRV2_TRISTATE | C_PORT_DRV_CTRL_DRV1_L |
     C_PORT_DRV_CTRL_DRV0_MASTER1),                                             /* PhD = Z, PhC = Z, PhB = L, PhA = PWM */
    (C_PORT_DRV_CTRL_DRV3_TRISTATE | C_PORT_DRV_CTRL_DRV2_TRISTATE | C_PORT_DRV_CTRL_DRV1_MASTER1 |
     C_PORT_DRV_CTRL_DRV0_L),                                                   /* PhD = Z, PhC = Z, PhB = PWM, PhA = L */
    (C_PORT_DRV_CTRL_DRV3_L | C_PORT_DRV_CTRL_DRV2_MASTER1 | C_PORT_DRV_CTRL_DRV1_TRISTATE |
     C_PORT_DRV_CTRL_DRV0_TRISTATE),                                            /* PhD = L, PhC = PWM, PhB = Z, PhA = Z */
    (C_PORT_DRV_CTRL_DRV3_MASTER1 | C_PORT_DRV_CTRL_DRV2_L | C_PORT_DRV_CTRL_DRV1_TRISTATE |
     C_PORT_DRV_CTRL_DRV0_TRISTATE)                                             /* PhD = PWM, PhC = Z, PhB = Z, PhA = Z */
};

/*! Coil-open test (MMP201104-1) */
static const uint16_t c_au16DrvCfgSelfTestC4[10] =
{
    (C_PORT_DRV_CTRL_DRV3_TRISTATE | C_PORT_DRV_CTRL_DRV2_TRISTATE | C_PORT_DRV_CTRL_DRV1_H |
     C_PORT_DRV_CTRL_DRV0_MASTER1),                                             /* PhD = Z, PhC = Z, PhB = H, PhA = PWM */
    (C_PORT_DRV_CTRL_DRV3_H | C_PORT_DRV_CTRL_DRV2_MASTER1 | C_PORT_DRV_CTRL_DRV1_TRISTATE |
     C_PORT_DRV_CTRL_DRV0_TRISTATE),                                            /* PhD = H, PhC = PMW, PhB = Z, PhA = Z */
    (C_PORT_DRV_CTRL_DRV3_TRISTATE | C_PORT_DRV_CTRL_DRV2_TRISTATE | C_PORT_DRV_CTRL_DRV1_L |
     C_PORT_DRV_CTRL_DRV0_MASTER1),                                             /* PhD = Z, PhC = Z, PhB = L, PhA = PWM */
    (C_PORT_DRV_CTRL_DRV3_L | C_PORT_DRV_CTRL_DRV2_MASTER1 | C_PORT_DRV_CTRL_DRV1_TRISTATE |
     C_PORT_DRV_CTRL_DRV0_TRISTATE),                                            /* PhD = L, PhC = PWM, PhB = Z, PhA = Z */
    (C_PORT_DRV_CTRL_DRV3_TRISTATE | C_PORT_DRV_CTRL_DRV2_TRISTATE | C_PORT_DRV_CTRL_DRV1_MASTER1 |
     C_PORT_DRV_CTRL_DRV0_H),                                                   /* PhD = Z, PhC = PWM, PhB = Z, PhA = H (MMP130916-2) */
    (C_PORT_DRV_CTRL_DRV3_MASTER1 | C_PORT_DRV_CTRL_DRV2_H | C_PORT_DRV_CTRL_DRV1_TRISTATE |
     C_PORT_DRV_CTRL_DRV0_TRISTATE),                                            /* PhD = PWM, PhC = Z, PhB = H, PhA = Z (MMP130916-2) */
    (C_PORT_DRV_CTRL_DRV3_TRISTATE | C_PORT_DRV_CTRL_DRV2_TRISTATE | C_PORT_DRV_CTRL_DRV1_MASTER1 |
     C_PORT_DRV_CTRL_DRV0_L),                                                   /* PhD = Z, PhC = PWM, PhB = Z, PhA = L (MMP130916-2) */
    (C_PORT_DRV_CTRL_DRV3_MASTER1 | C_PORT_DRV_CTRL_DRV2_L | C_PORT_DRV_CTRL_DRV1_TRISTATE |
     C_PORT_DRV_CTRL_DRV0_TRISTATE),                                            /* PhD = PWM, PhC = Z, PhB = L, PhA = Z (MMP130916-2) */
    (C_PORT_DRV_CTRL_DRV3_H | C_PORT_DRV_CTRL_DRV2_SLAVE2 | C_PORT_DRV_CTRL_DRV1_H | C_PORT_DRV_CTRL_DRV0_MASTER1),
    (C_PORT_DRV_CTRL_DRV3_SLAVE3 | C_PORT_DRV_CTRL_DRV2_H | C_PORT_DRV_CTRL_DRV1_SLAVE1 | C_PORT_DRV_CTRL_DRV0_H)
};
#elif (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UT_VW)
/*! Coil-short test (MMP201104-1) */
static const uint16_t c_au16DrvCfgSelfTestB4[4] =
{
    (C_PORT_DRV_CTRL_DRV3_L | C_PORT_DRV_CTRL_DRV2_TRISTATE | C_PORT_DRV_CTRL_DRV1_TRISTATE |
     C_PORT_DRV_CTRL_DRV0_MASTER1),                                             /* PhD = L, PhC = Z, PhB = Z, PhA = PWM */
    (C_PORT_DRV_CTRL_DRV3_TRISTATE | C_PORT_DRV_CTRL_DRV2_L | C_PORT_DRV_CTRL_DRV1_MASTER1 |
     C_PORT_DRV_CTRL_DRV0_TRISTATE),                                            /* PhD = Z, PhC = 0, PhB = PWM, PhA = Z */
    (C_PORT_DRV_CTRL_DRV3_TRISTATE | C_PORT_DRV_CTRL_DRV2_MASTER1 | C_PORT_DRV_CTRL_DRV1_L |
     C_PORT_DRV_CTRL_DRV0_TRISTATE),                                            /* PhD = Z, PhC = PWM, PhB = 0, PhA = Z */
    (C_PORT_DRV_CTRL_DRV3_MASTER1 | C_PORT_DRV_CTRL_DRV2_TRISTATE | C_PORT_DRV_CTRL_DRV1_TRISTATE |
     C_PORT_DRV_CTRL_DRV0_L)                                                    /* PhD = PWM, PhC = Z, PhB = Z, PhA = 0 */
};

/*! Coil-open test (MMP201104-1) */
static const uint16_t c_au16DrvCfgSelfTestC4[10] =
{
    (C_PORT_DRV_CTRL_DRV3_H | C_PORT_DRV_CTRL_DRV2_TRISTATE | C_PORT_DRV_CTRL_DRV1_TRISTATE |
     C_PORT_DRV_CTRL_DRV0_MASTER1),                                             /* PhD = Z, PhC = Z, PhB = H, PhA = PWM */
    (C_PORT_DRV_CTRL_DRV3_TRISTATE | C_PORT_DRV_CTRL_DRV2_H | C_PORT_DRV_CTRL_DRV1_MASTER1 |
     C_PORT_DRV_CTRL_DRV0_TRISTATE),                                            /* PhD = H, PhC = PMW, PhB = Z, PhA = Z */
    (C_PORT_DRV_CTRL_DRV3_L | C_PORT_DRV_CTRL_DRV2_TRISTATE | C_PORT_DRV_CTRL_DRV1_TRISTATE |
     C_PORT_DRV_CTRL_DRV0_MASTER1),                                             /* PhD = Z, PhC = Z, PhB = L, PhA = PWM */
    (C_PORT_DRV_CTRL_DRV3_TRISTATE | C_PORT_DRV_CTRL_DRV2_L | C_PORT_DRV_CTRL_DRV1_MASTER1 |
     C_PORT_DRV_CTRL_DRV0_TRISTATE),                                            /* PhD = L, PhC = PWM, PhB = Z, PhA = Z */
    (C_PORT_DRV_CTRL_DRV3_MASTER1 | C_PORT_DRV_CTRL_DRV2_TRISTATE | C_PORT_DRV_CTRL_DRV1_TRISTATE |
     C_PORT_DRV_CTRL_DRV0_H),                                                   /* PhD = Z, PhC = PWM, PhB = Z, PhA = H (MMP130916-2) */
    (C_PORT_DRV_CTRL_DRV3_TRISTATE | C_PORT_DRV_CTRL_DRV2_MASTER1 | C_PORT_DRV_CTRL_DRV1_H |
     C_PORT_DRV_CTRL_DRV0_TRISTATE),                                            /* PhD = PWM, PhC = Z, PhB = H, PhA = Z (MMP130916-2) */
    (C_PORT_DRV_CTRL_DRV3_MASTER1 | C_PORT_DRV_CTRL_DRV2_TRISTATE | C_PORT_DRV_CTRL_DRV1_TRISTATE |
     C_PORT_DRV_CTRL_DRV0_L),                                                   /* PhD = Z, PhC = PWM, PhB = Z, PhA = L (MMP130916-2) */
    (C_PORT_DRV_CTRL_DRV3_TRISTATE | C_PORT_DRV_CTRL_DRV2_MASTER1 | C_PORT_DRV_CTRL_DRV1_L |
     C_PORT_DRV_CTRL_DRV0_TRISTATE),                                            /* PhD = PWM, PhC = Z, PhB = L, PhA = Z (MMP130916-2) */
    (C_PORT_DRV_CTRL_DRV3_H | C_PORT_DRV_CTRL_DRV2_H | C_PORT_DRV_CTRL_DRV1_SLAVE1 | C_PORT_DRV_CTRL_DRV0_MASTER1),
    (C_PORT_DRV_CTRL_DRV3_SLAVE3 | C_PORT_DRV_CTRL_DRV2_SLAVE2 | C_PORT_DRV_CTRL_DRV1_H | C_PORT_DRV_CTRL_DRV0_H)
};
#endif /* (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UW_VT) */

/*! Coil short test (MMP201104-1) */
static const ADC_SDATA_t tAdcSelfTest4B[4] =
{
    /* (DRV_CFG_T_TRISTATE | DRV_CFG_W_0 | DRV_CFG_V_TRISTATE | DRV_CFG_U_PWM) */
    /* Motor Phase U */
    {
        {
            .u2AdcMarker = C_ADC_NO_SIGN,
            .u5AdcChannel = C_ADC_PH_U_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
            .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
            .u3AdcReserved = 0U,
#endif
            .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
            .u1AdcReserved = 0U
        }
    },
    /* Motor Phase V */
    {
        {
            .u2AdcMarker = C_ADC_NO_SIGN,
            .u5AdcChannel = C_ADC_PH_V_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
            .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
            .u3AdcReserved = 0U,
#endif
            .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
            .u1AdcReserved = 0U
        }
    },
    /* Motor Phase W */
    {
        {
            .u2AdcMarker = C_ADC_NO_SIGN,
            .u5AdcChannel = C_ADC_PH_W_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
            .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
            .u3AdcReserved = 0U,
#endif
            .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
            .u1AdcReserved = 0U
        }
    },
    /* Motor Phase T */
    {
        {
            .u2AdcMarker = C_ADC_NO_SIGN,
            .u5AdcChannel = C_ADC_PH_T_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
            .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
            .u3AdcReserved = 0U,
#endif
            .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
            .u1AdcReserved = 0U
        }
    }
};

/*!*************************************************************************** *
 * MotorDriverSelfTestInit
 * \brief   Perform driver Self-test
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t error-code
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: MotorDriverSelfTest()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 2 (MeasureVsupplyAndTemperature(), ADC_Conv_Vmotor())
 * *************************************************************************** */
static uint16_t MotorDriverSelfTestInit(void)
{
    uint16_t u16VdsThreshold;
    ADC_MeasureVsupplyAndTemperature();
    (void)ADC_Conv_Vmotor();
#if FALSE
    /* VDS Threshold not stored in EE */
    if (NV_VDS_THRESHOLD != 0U)
    {
        u16VdsThreshold = NV_VDS_THRESHOLD;
    }
    else
#endif /* FALSE */
    {
        u16VdsThreshold = 200U;                                                 /* 2.00V */
    }

    /* Convert Vds-voltage (10mV units) to ADC-LSB */
    u16VdsThreshold = p_MulDivU16_U16byU16byU16(u16VdsThreshold, C_VOLTGAIN_DIV, Get_MotorVoltGainF());
    return (u16VdsThreshold);
} /* End of MotorDriverSelfTestInit() */

/*!*************************************************************************** *
 * MotorDriverSelfTestPin
 * \brief   Perform driver Self-test - Pin check
 * \author  mmp
 * *************************************************************************** *
 * \param   u16VdsThreshold: VDS threshold (10mV)
 * \return  -
 * *************************************************************************** *
 * \details Self test Motor Driver (Pin)
 * Note: Diagnostics must be initialised before calling this self-test.
 *
 * 1. Measure all phase voltage and Motor Driver Voltage
 * 2. Check for each phase voltage is near GNDM or VSM
 *    a. Near GNDM: Create 2us high-pulse and check phase-voltage
 *    b. Near VSM: Create 2us low-pulse and check phase-voltage
 * Test-time: Approx. 9 PWM periods (@20kHz: 450us).
 * *************************************************************************** *
 * - Call Hierarchy: MotorDriverSelfTest()
 * - Cyclomatic Complexity: 8+1
 * - Nesting: 3
 * - Function calling: 1 (SetLastError())
 * *************************************************************************** */
static void MotorDriverSelfTestPin(uint16_t u16VdsThreshold)
{
    /* Pre-test (MMP180409-1) */
    static const ADC_SDATA_t tAdcPreSelfTest4[6] =
    {
        /* Motor Phase U */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_U_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)
                .u3AdcReserved = 0U,
#endif
                .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CMP,
                .u1AdcReserved = 0U
            }
        },
        /* Motor Phase V */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_V_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                .u3AdcReserved = 0U,
#endif
                .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV1_CMP,
                .u1AdcReserved = 0U
            }
        },
        /* Motor Phase W */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_W_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                .u3AdcReserved = 0U,
#endif
                .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                .u1AdcReserved = 0U
            }
        },
        /* Motor Phase T */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_U_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                .u3AdcReserved = 0U,
#endif
                .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV3_CMP,
                .u1AdcReserved = 0U
            }
        },
        /* Motor Supply */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_VSM_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                .u3AdcReserved = 0U,
#endif
                .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,
                .u1AdcReserved = 0U
            }
        },
        {.u16 = C_ADC_EOS}
    };
    static const ADC_SDATA_t JTEMP =
    {
        {
            .u2AdcMarker = C_ADC_NO_SIGN,
            .u5AdcChannel = C_ADC_TEMP,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
            .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
            .u3AdcReserved = 0U,
#endif
            .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CMP,
            .u1AdcReserved = 0U
        }
    };

    uint16_t u16PhaseIdx;
    T_ADC_PRE_SELFTEST_4PH sMotorPreSelfTest;                                   /* Pre-test */
    T_ADC_SELFTEST_4PH_B sMotorSelfTestB;                                       /* Pin short test */

#if (_DEBUG_SELFTEST != FALSE)
    DEBUG_SET_IO_B();
#endif /* (_DEBUG_SELFTEST != FALSE) */

    HAL_ADC_StopSafe();                                                         /* Stop ADC */
    DRVCFG_DIS_TUVW();
    uint16_t *pu16Src = &l_au16AdcSource[0];
    *pu16Src++ = (uint16_t)&sMotorPreSelfTest;
    *pu16Src++ = tAdcPreSelfTest4[0].u16;
    *pu16Src++ = tAdcPreSelfTest4[1].u16;
    *pu16Src++ = tAdcPreSelfTest4[2].u16;
    *pu16Src++ = tAdcPreSelfTest4[3].u16;
    *pu16Src++ = tAdcPreSelfTest4[4].u16;
    *pu16Src++ = tAdcPreSelfTest4[5].u16;

    HAL_ADC_Setup(/*B_ADC_ADC_WIDTH |*/                                         /* Setup Self Test measurement */
        C_ADC_ASB_NEVER |                                                       /* Auto Standby: Never */
#if (_DEBUG_IO_ADC != FALSE)
        C_ADC_INT_SCHEME_EOC |                                                  /* Message interrupt: End-of-Frame */
#else  /* (_DEBUG_IO_ADC != FALSE) */
        C_ADC_INT_SCHEME_NOINT |                                                /* Message interrupt: No */
#endif /* (_DEBUG_IO_ADC != FALSE) */
        B_ADC_SATURATE |                                                        /* Saturation: Enabled */
        B_ADC_NO_INTERLEAVE |                                                   /* Interleave: No */
        C_ADC_SOC_SOURCE_HARD_CTRIG |                                           /* Start Of Conversion (SOC) triggered by: Hardware (HARD_CTRIG) */
        C_ADC_SOS_SOURCE_2ND_HARD_CTRIG);                                       /* Start Of Sequence (SOS) triggered: 2nd Hardware */
    HAL_ADC_ClearErrors();                                                      /* Prior to start, first clear any error flag and enable Triggers */

    HAL_ADC_Start();                                                            /* Start ADC */
#if (_SUPPORT_WHILE_RETRY_LIMITED != FALSE)
    uint16_t u16Retries = 100U;
    while ( (IO_ADC_CTRL & B_ADC_START) != 0U)
    {
        if (--u16Retries == 0U)
        {
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_TO_MDST_PIN);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            break;
        }
        DELAY_US(5);
    }                  /* Wait for ADC result (Time-out?) */
#else  /* (_SUPPORT_WHILE_RETRY_LIMITED != FALSE) */
    while ( (IO_ADC_CTRL & B_ADC_START) != 0U) {}                               /* Wait for ADC result (Time-out?) */
#endif /* (_SUPPORT_WHILE_RETRY_LIMITED != FALSE) */

    /* Check phase by phase */
    IO_PWM_SLAVE2_CMP = (((3UL * PWM_REG_PERIOD) + 3U) / 6U) + ((C_MIN_PWM_PULSE / 2U) - C_ADC_TRIGGER_BEFORE_END); /* Sample phase voltage near to end-of the phase pulse */

    for (u16PhaseIdx = 0U; (g_e8ErrorElectric == (uint8_t)C_ERR_NONE) && (u16PhaseIdx < 4U); u16PhaseIdx++)
    {
        uint16_t u16DC;
        u16DC = ((uint16_t *)&sMotorPreSelfTest.PhaseVoltageU)[u16PhaseIdx];  /*lint !e661 !e662 */
        if (u16DC < u16VdsThreshold)
        {
            /* Near ground */
            u16DC = PWM_REG_PERIOD - C_MIN_PWM_PULSE;                           /* Minimum PWM pulse to measure Vphase */
        }
        else if (u16DC > (sMotorPreSelfTest.MotorDriverVoltage - u16VdsThreshold) )
        {
            /* Near supply */
            u16DC = C_MIN_PWM_PULSE;                                            /* Minimum PWM pulse to measure Vphase */
        }
        else
        {
            continue;
        }
        /* Independent mode */
        u16DC = u16DC / 2U;
        IO_PWM_MASTER1_HT = PWM_REG_PERIOD - u16DC;                             /* MMP201104-1 */
        IO_PWM_MASTER1_LT = u16DC;                                              /* Copy the results into the PWM register for phase U */

        HAL_ADC_StopSafe();                                                     /* Clear the ADC control register */
        pu16Src = &l_au16AdcSource[0];
        *pu16Src++ = (uint16_t)&sMotorSelfTestB;
        *pu16Src++ = JTEMP.u16;
        *pu16Src++ = tAdcSelfTest4B[u16PhaseIdx].u16;                           /* Phase Voltage measured at SLV2 trigger, 50% of period */
        *pu16Src++ = tAdcPreSelfTest4[4].u16;                                   /* Motor Supply */
        *pu16Src++ = C_ADC_EOS;                                                 /* MMP201104-1 - End */

        HAL_ADC_Setup(/*B_ADC_ADC_WIDTH |*/                                     /* Setup Self Test measurement */
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
        HAL_ADC_ClearErrors();                                                  /* Prior to start, first clear any error flag and enable Triggers */

        HAL_PWM_MasterPendClear();                                              /* Clear PWM-module Master1-End IRQ's */
        HAL_PWM_MasterPendWait();                                               /* Wait for PWM Master PEND-flag */
        DRVCFG_CNFG_TUVW( (uint16_t)c_au16DrvCfgPreSelfTest[u16PhaseIdx]);      /* Single coil switch active */
        DRVCFG_ENA_TUVW();

        HAL_ADC_Start();                                                        /* Start ADC */

        /* This takes about 4 Motor PWM-periods per self-test */
#if (_SUPPORT_WHILE_RETRY_LIMITED != FALSE)
        uint16_t u16Retries = 100U;
        while ( (IO_ADC_CTRL & B_ADC_START) != 0U)
        {
            if (--u16Retries == 0U)
            {
#if (_SUPPORT_LOG_ERRORS != FALSE)
                SetLastError(C_ERR_TO_MDST_PIN);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                break;
            }
            DELAY_US(5);
        }                  /* Wait for ADC result (Time-out?) */
#else  /* (_SUPPORT_WHILE_RETRY_LIMITED != FALSE) */
        while ( (IO_ADC_CTRL & B_ADC_START) != 0U) {}                           /* Wait for ADC result (Time-out?) */
#endif /* (_SUPPORT_WHILE_RETRY_LIMITED != FALSE) */

        DRVCFG_DIS_TUVW();

        if (g_e8ErrorElectric != C_ERR_NONE)
        {
            /* Over-current trigger; Phase makes short with supply or Ground */
            g_e8ErrorElectric |= (uint8_t)C_ERR_PERMANENT;
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_SELFTEST_A);                           /* Over-current */
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            break;
        }

        if (u16DC == C_MIN_PWM_PULSE)
        {
            if (sMotorSelfTestB.PhaseVoltage > u16VdsThreshold)
            {
                /* Pin short (Vds Bottom-FET) */
                g_e8ErrorElectric = (uint8_t)(C_ERR_PERMANENT | C_ERR_VDS);
#if (_SUPPORT_LOG_ERRORS != FALSE)
                SetLastError(C_ERR_SELFTEST_B | (C_ERR_EXT + (u16PhaseIdx << 8)));
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                break;
            }
        }
        else if (sMotorSelfTestB.PhaseVoltage < (sMotorSelfTestB.MotorDriverVoltage - u16VdsThreshold) )
        {
            /* Pin short (Vds Top-FET) */
            g_e8ErrorElectric = (uint8_t)(C_ERR_PERMANENT | C_ERR_VDS);
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_SELFTEST_B | (C_ERR_EXT + (u16PhaseIdx << 8)));
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            break;
        }
    }

    IO_PWM_SLAVE2_CMP = (((3UL * PWM_REG_PERIOD) + 3U) / 6U);

#if (_DEBUG_SELFTEST != FALSE)
    DEBUG_CLR_IO_B();
#endif /* (_DEBUG_SELFTEST != FALSE) */
} /* End of MotorDriverSelfTestPin() */

/*!*************************************************************************** *
 * MotorDriverSelfTestPin2Pin
 * \brief   Perform driver Self-test - Pin-to-Pin check
 * \author  mmp
 * *************************************************************************** *
 * \param   u16VdsThreshold: VDS threshold (10mV)
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: MotorDriverSelfTest()
 * - Cyclomatic Complexity: 9+1
 * - Nesting: 3
 * - Function calling: 1 (SetLastError())
 * *************************************************************************** */
static void MotorDriverSelfTestPin2Pin(uint16_t u16VdsThreshold)
{
    /* Pin-test, including neighbour phase (MMP180409-1/MMP201104-1) */
    static const ADC_SDATA_t tAdcSelfTest4A[4][2] =
    {
        {
#if (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_WT)
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_PH_V_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV3_CMP,
                    .u1AdcReserved = 0U
                }
            },
#elif (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UT_VW)
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_PH_T_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV3_CMP,
                    .u1AdcReserved = 0U
                }
            },
#elif (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UW_VT)
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_PH_V_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV3_CMP,
                    .u1AdcReserved = 0U
                }
            },
#endif /* (_SUPPORT_BIPOLAR_MODE) */
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_PH_W_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,
                    .u1AdcReserved = 0U
                }
            },
        },
        {
#if (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_WT)
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_PH_W_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV3_CMP,
                    .u1AdcReserved = 0U
                }
            },
#elif (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UT_VW)
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_PH_U_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV3_CMP,
                    .u1AdcReserved = 0U
                }
            },
#elif (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UW_VT)
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_PH_U_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV3_CMP,
                    .u1AdcReserved = 0U
                }
            },
#endif /* (_SUPPORT_BIPOLAR_MODE) */
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_PH_V_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,
                    .u1AdcReserved = 0U
                }
            }
        },
        {
#if (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_WT)
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_PH_W_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV3_CMP,
                    .u1AdcReserved = 0U
                }
            },
#elif (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UT_VW)
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_PH_V_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV3_CMP,
                    .u1AdcReserved = 0U
                }
            },
#elif (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UW_VT)
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_PH_V_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV3_CMP,
                    .u1AdcReserved = 0U
                }
            },
#endif /* (_SUPPORT_BIPOLAR_MODE) */
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_PH_U_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,
                    .u1AdcReserved = 0U
                }
            }
        },
        {
#if (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_WT)
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_PH_U_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV3_CMP,
                    .u1AdcReserved = 0U
                }
            },
#elif (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UT_VW)
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_PH_W_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV3_CMP,
                    .u1AdcReserved = 0U
                }
            },
#elif (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UW_VT)
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_PH_W_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV3_CMP,
                    .u1AdcReserved = 0U
                }
            },
#endif /* (_SUPPORT_BIPOLAR_MODE) */
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_PH_T_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,
                    .u1AdcReserved = 0U
                }
            }
        }
    };
    static const ADC_SDATA_t JTEMP =
    {
        {
            .u2AdcMarker = C_ADC_NO_SIGN,
            .u5AdcChannel = C_ADC_TEMP,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
            .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
            .u3AdcReserved = 0U,
#endif
            .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CMP,
            .u1AdcReserved = 0U
        }
    };
    static const ADC_SDATA_t VSM =
    {
        {
            .u2AdcMarker = C_ADC_NO_SIGN,
            .u5AdcChannel = C_ADC_VSM_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
            .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
            .u3AdcReserved = 0U,
#endif
            .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
            .u1AdcReserved = 0U
        }
    };

    uint16_t u16SelfTestIdx;
    uint16_t u16NeighbourPhaseRisk = FALSE;                                     /* MMP170726-2: Phase-to-phase check */
    T_ADC_SELFTEST_4PH_A sMotorSelfTestA;                                       /* Pin-test */

#if (_DEBUG_SELFTEST != FALSE)
    DEBUG_SET_IO_B();
#endif /* (_DEBUG_SELFTEST != FALSE) */

    /* Pin-Test for FET shortages; Note: Diagnostics configuration will switch off driver at over-current (MMP170421-2) */
    for (u16SelfTestIdx = 0U;
         (g_e8ErrorElectric == (uint8_t)C_ERR_NONE) &&
         (u16SelfTestIdx < (sizeof(c_au16DrvCfgSelfTestA4) / sizeof(c_au16DrvCfgSelfTestA4[0])));
         u16SelfTestIdx++)
    {
        HAL_ADC_StopSafe();                                                     /* Clear the ADC control register */
        uint16_t *pu16Src = &l_au16AdcSource[0];
        *pu16Src++ = (uint16_t)&sMotorSelfTestA;
        *pu16Src++ = JTEMP.u16;
        *pu16Src++ = VSM.u16;
        *pu16Src++ = tAdcSelfTest4A[u16SelfTestIdx >> 1][0].u16;
        *pu16Src++ = tAdcSelfTest4A[u16SelfTestIdx >> 1][1].u16;
        *pu16Src++ = C_ADC_EOS;

        HAL_ADC_Setup(/*B_ADC_ADC_WIDTH |*/                                     /* Setup Self Test measurement */
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
        HAL_ADC_ClearErrors();                                                  /* Prior to start, first clear any error flag and enable Triggers */

        HAL_PWM_MasterPendClear();                                              /* Clear PWM-module Master1-End IRQ's */
        HAL_PWM_MasterPendWait();                                               /* Wait for PWM Master PEND-flag */
        DRVCFG_CNFG_TUVW( (uint16_t)c_au16DrvCfgSelfTestA4[u16SelfTestIdx]);    /* Single coil switch active */
        DRVCFG_ENA_TUVW();

        HAL_ADC_Start();                                                        /* Start ADC */

        /* This takes about 4 Motor PWM-periods per self-test */
#if (_SUPPORT_WHILE_RETRY_LIMITED != FALSE)
        uint16_t u16Retries = 100U;
        while ( (IO_ADC_CTRL & B_ADC_START) != 0U)
        {
            if (--u16Retries == 0U)
            {
#if (_SUPPORT_LOG_ERRORS != FALSE)
                SetLastError(C_ERR_TO_MDST_PIN2PIN);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                break;
            }
            DELAY_US(5);
        }                  /* Wait for ADC result (Time-out?) */
#else  /* (_SUPPORT_WHILE_RETRY_LIMITED != FALSE) */
        while ( (IO_ADC_CTRL & B_ADC_START) != 0U) {}                           /* Wait for ADC result (Time-out?) */
#endif /* (_SUPPORT_WHILE_RETRY_LIMITED != FALSE) */

        DRVCFG_DIS_TUVW();

        if (g_e8ErrorElectric != C_ERR_NONE)
        {
            /* Over-current trigger; Phase makes short with supply or Ground */
            g_e8ErrorElectric |= (uint8_t)C_ERR_PERMANENT;
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_SELFTEST_A);                                     /* Over-current */
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            break;
        }
        if ( (u16SelfTestIdx & 1U) == 0U)
        {
            /* Step #0: Phase-voltage is LOW */
            if (sMotorSelfTestA.PhaseVoltage > u16VdsThreshold)
            {
                g_e8ErrorElectric |= (uint8_t)(C_ERR_PERMANENT | C_ERR_VDS);
#if (_SUPPORT_LOG_ERRORS != FALSE)
                SetLastError(C_ERR_SELFTEST_B | (C_ERR_EXT + (u16SelfTestIdx << 8)));
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                break;
            }

            /*  MMP170726-2: Neighbour phase voltage check (floating) */
            if (sMotorSelfTestA.NeighbourPhaseVoltage < u16VdsThreshold)
            {
                /* Neighbour phase is low (below VDS-threshold); This is at risk (possible short) */
                u16NeighbourPhaseRisk = TRUE;
            }
            else
            {
                /* Neighbour phase is high (above VDS-threshold); No risk = no short */
                u16NeighbourPhaseRisk = FALSE;
            }
        }
        else
        {
            /* Step #1: Phase-voltage is HIGH */
            if (sMotorSelfTestA.PhaseVoltage < (sMotorSelfTestA.MotorDriverVoltage - u16VdsThreshold) )
            {
                g_e8ErrorElectric |= (uint8_t)(C_ERR_PERMANENT | C_ERR_VDS);
#if (_SUPPORT_LOG_ERRORS != FALSE)
                SetLastError(C_ERR_SELFTEST_B | (C_ERR_EXT + (u16SelfTestIdx << 8)));   /* Phase to ground short */
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                break;
            }

            /*  MMP170726-2: Neighbour phase voltage check (floating) */
            if ( (u16NeighbourPhaseRisk != FALSE) &&                            /* Possible short (Neighbour phase was Low) */
                 (sMotorSelfTestA.NeighbourPhaseVoltage > (sMotorSelfTestA.MotorDriverVoltage - u16VdsThreshold)) ) /* Neighbour phase is high */
            {
                /* Phase-to-Phase short; Neighbour phase follows test-phase */
                g_e8ErrorElectric |= (uint8_t)(C_ERR_PERMANENT | C_ERR_VDS);
#if (_SUPPORT_LOG_ERRORS != FALSE)
                SetLastError(C_ERR_SELFTEST_F | (C_ERR_EXT + (u16SelfTestIdx << 8)));   /* Phase to Phase short */
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                break;
            }
        }
    }

    (void)tAdcSelfTest4A;

#if (_DEBUG_SELFTEST != FALSE)
    DEBUG_CLR_IO_B();
#endif /* (_DEBUG_SELFTEST != FALSE) */
} /* End of MotorDriverSelfTestPin2Pin() */

/*!*************************************************************************** *
 * MotorDriverSelfTestCoilShort
 * \brief   Perform driver Self-test - Coil-Short
 * \author  mmp
 * *************************************************************************** *
 * \param   u16VdsThreshold: VDS threshold (10mV)
 * \return  -
 * *************************************************************************** *
 * \details Self test Motor Driver (Coil Short)
 * Note: Diagnostics must be initialised before calling this self-test.
 *
 * Test-time: Approx. 8 PWM periods (@20kHz: 400us).
 * (20240301: Approx. 12 PWM periods (@20kHz: 600us).
 * *************************************************************************** *
 * - Call Hierarchy: MotorDriverSelfTest()
 * - Cyclomatic Complexity: 7+1
 * - Nesting: 3
 * - Function calling: 1 (SetLastError())
 * *************************************************************************** */
static void MotorDriverSelfTestCoilShort(uint16_t u16VdsThreshold)
{
    static const ADC_SDATA_t JTEMP =
    {
        {
            .u2AdcMarker = C_ADC_NO_SIGN,
            .u5AdcChannel = C_ADC_TEMP,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
            .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
            .u3AdcReserved = 0U,
#endif
            .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CMP,
            .u1AdcReserved = 0U
        }
    };
    static const ADC_SDATA_t VSM =
    {
        {
            .u2AdcMarker = C_ADC_NO_SIGN,
            .u5AdcChannel = C_ADC_VSM_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
            .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
            .u3AdcReserved = 0U,
#endif
            .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,
            .u1AdcReserved = 0U
        }
    };

    uint16_t u16SelfTestIdx;
    T_ADC_SELFTEST_4PH_B sMotorSelfTestB;                                       /* Coil short test */

#if (_DEBUG_SELFTEST != FALSE)
    DEBUG_SET_IO_B();
#endif /* (_DEBUG_SELFTEST != FALSE) */

    /* Coil short test (MMP170903-1) */
    if (g_e8ErrorElectric == (uint8_t)C_ERR_NONE)
    {
        /* Shift ADC 50% trigger a bit to 50%+ */
        uint16_t u16DC = PWM_REG_PERIOD - C_MIN_PWM_PULSE;                      /* Minimum PWM pulse to measure Vphase */
        IO_PWM_SLAVE2_CMP = (((3UL * PWM_REG_PERIOD) + 3U) / 6U) + ((C_MIN_PWM_PULSE / 2U) - C_ADC_TRIGGER_BEFORE_END); /* Sample phase voltage near to end-of the phase pulse */
        /* Independent-mode */
        u16DC = u16DC / 2U;
        IO_PWM_MASTER1_HT = PWM_REG_PERIOD - u16DC;                             /* MMP201104-1 */
        IO_PWM_MASTER1_LT = u16DC;                                              /* Copy the results into the PWM register for phase U */
    }

    for (u16SelfTestIdx = 0U;
         (g_e8ErrorElectric == (uint8_t)C_ERR_NONE) &&
         (u16SelfTestIdx < (sizeof(c_au16DrvCfgSelfTestB4) / sizeof(c_au16DrvCfgSelfTestB4[0])));
         u16SelfTestIdx++)
    {
        HAL_ADC_StopSafe();                                                     /* Clear the ADC control register */
        uint16_t *pu16Src = &l_au16AdcSource[0];
        *pu16Src++ = (uint16_t)&sMotorSelfTestB;
        *pu16Src++ = JTEMP.u16;                                                 /* IC Junction temperature measured at MSTR1_CMP trigger, 12.5% of period */
        *pu16Src++ = tAdcSelfTest4B[u16SelfTestIdx].u16;                        /* Phase Voltage measured at SLV2 trigger, 50% of period */
        *pu16Src++ = VSM.u16;                                                   /* Motor voltage measured at MSTR1_CNT, end of period */
        *pu16Src++ = C_ADC_EOS;

        HAL_ADC_Setup(/*B_ADC_ADC_WIDTH |*/                                     /* Setup Self Test measurement */
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
        HAL_ADC_ClearErrors();                                                  /* Prior to start, first clear any error flag and enable Triggers */

        HAL_PWM_MasterPendClear();                                              /* Clear PWM-module Master1-End IRQ's */
        HAL_PWM_MasterPendWait();                                               /* Wait for PWM Master PEND-flag */
        DRVCFG_CNFG_TUVW( (uint16_t)c_au16DrvCfgSelfTestB4[u16SelfTestIdx]);    /* Single coil switch active */
        DRVCFG_ENA_TUVW();

        HAL_ADC_Start();                                                        /* Start ADC */

        /* This takes about 4 Motor PWM-periods per self-test */
#if (_SUPPORT_WHILE_RETRY_LIMITED != FALSE)
        uint16_t u16Retries = 100U;
        while ( (IO_ADC_CTRL & B_ADC_START) != 0U)
        {
            if (--u16Retries == 0U)
            {
#if (_SUPPORT_LOG_ERRORS != FALSE)
                SetLastError(C_ERR_TO_MDST_COIL_SHORT);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                break;
            }
            DELAY_US(5);
        }                  /* Wait for ADC result (Time-out?) */
#else  /* (_SUPPORT_WHILE_RETRY_LIMITED != FALSE) */
        while ( (IO_ADC_CTRL & B_ADC_START) != 0U) {}                           /* Wait for ADC result (Time-out?) */
#endif /* (_SUPPORT_WHILE_RETRY_LIMITED != FALSE) */

        DRVCFG_DIS_TUVW();

        if (g_e8ErrorElectric != C_ERR_NONE)
        {
            /* Over-current trigger; Phase makes short with supply or Ground */
            g_e8ErrorElectric |= (uint8_t)C_ERR_PERMANENT;
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_SELFTEST_A);                                     /* Over-current */
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            break;
        }

        if (sMotorSelfTestB.PhaseVoltage < (sMotorSelfTestB.MotorDriverVoltage - u16VdsThreshold) )
        {
            /* Coil short (Vds P-FET) */
            g_e8ErrorElectric |= (uint8_t)(C_ERR_PERMANENT | C_ERR_VDS);
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_SELFTEST_D);                                     /* Coil Short */
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            break;
        }
    }

    IO_PWM_SLAVE2_CMP = (((3UL * PWM_REG_PERIOD) + 3U) / 6U);

    (void)tAdcSelfTest4B;

#if (_DEBUG_SELFTEST != FALSE)
    DEBUG_CLR_IO_B();
#endif /* (_DEBUG_SELFTEST != FALSE) */
} /* End of MotorDriverSelfTestCoilShort() */

/*!*************************************************************************** *
 * MotorDriverSelfTestCoilOpen
 * \brief   Perform driver Self-test - Coil-Open
 * \author  mmp
 * *************************************************************************** *
 * \param   u16VdsThreshold: VDS threshold (10mV)
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: MotorDriverSelfTest()
 * - Cyclomatic Complexity: 10+1
 * - Nesting: 2
 * - Function calling: 1 (SetLastError())
 * *************************************************************************** */
static void MotorDriverSelfTestCoilOpen(uint16_t u16VdsThreshold)
{
    /* Coil-open test (MMP180409-1)/(MMP201104-1) */
    static const ADC_SDATA_t tAdcSelfTest4C_H[4][4] =
    {
        {
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_PH_U_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                    .u1AdcReserved = 0U
                }
            },
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_PH_U_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,
                    .u1AdcReserved = 0U
                }
            },
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_VSMF_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                    .u1AdcReserved = 0U
                }
            },
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_MCUR,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,
                    .u1AdcReserved = 0U
                }
            }
        },
        {
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_PH_V_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                    .u1AdcReserved = 0U
                }
            },
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_PH_V_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,
                    .u1AdcReserved = 0U
                }
            },
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_VSMF_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                    .u1AdcReserved = 0U
                }
            },
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_MCUR,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,
                    .u1AdcReserved = 0U
                }
            }
        },
        {
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_PH_W_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                    .u1AdcReserved = 0U
                }
            },
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_PH_W_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,
                    .u1AdcReserved = 0U
                }
            },
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_VSMF_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                    .u1AdcReserved = 0U
                }
            },
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_MCUR,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,
                    .u1AdcReserved = 0U
                }
            }
        },
        {
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_PH_T_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                    .u1AdcReserved = 0U
                }
            },
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_PH_T_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,
                    .u1AdcReserved = 0U
                }
            },
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_VSMF_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                    .u1AdcReserved = 0U
                }
            },
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_MCUR,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,
                    .u1AdcReserved = 0U
                }
            }
        }
    };
    static const ADC_SDATA_t tAdcSelfTest4C_L[4][4] =
    {
        {
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_PH_U_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                    .u1AdcReserved = 0U
                }
            },
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_PH_U_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,
                    .u1AdcReserved = 0U
                }
            },
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_MCUR,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                    .u1AdcReserved = 0U
                }
            },
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_VSMF_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,
                    .u1AdcReserved = 0U
                }
            }
        },
        {
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_PH_V_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                    .u1AdcReserved = 0U
                }
            },
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_PH_V_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,
                    .u1AdcReserved = 0U
                }
            },
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_MCUR,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                    .u1AdcReserved = 0U
                }
            },
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_VSMF_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,
                    .u1AdcReserved = 0U
                }
            }
        },
        {
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_PH_W_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                    .u1AdcReserved = 0U
                }
            },
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_PH_W_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,
                    .u1AdcReserved = 0U
                }
            },
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_MCUR,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                    .u1AdcReserved = 0U
                }
            },
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_VSMF_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,
                    .u1AdcReserved = 0U
                }
            }
        },
        {
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_PH_T_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                    .u1AdcReserved = 0U
                }
            },
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_PH_T_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,
                    .u1AdcReserved = 0U
                }
            },
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_MCUR,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                    .u1AdcReserved = 0U
                }
            },
            {
                {
                    .u2AdcMarker = C_ADC_NO_SIGN,
                    .u5AdcChannel = C_ADC_VSMF_HV,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
                    .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)                          /* MMP240328-2 */
                    .u3AdcReserved = 0U,
#endif
                    .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,
                    .u1AdcReserved = 0U
                }
            }
        }
    };
    static const uint16_t tAdcSelfTest4C[10] =
    {
#if (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UW_VT)
        (uint16_t)&tAdcSelfTest4C_H[0][0],                                      /* Phase U = PWM High */
        (uint16_t)&tAdcSelfTest4C_H[1][0],                                      /* Phase V = PWM High */
        (uint16_t)&tAdcSelfTest4C_L[0][0],                                      /* Phase U = PWM Low */
        (uint16_t)&tAdcSelfTest4C_L[1][0],                                      /* Phase V = PWM Low */
        (uint16_t)&tAdcSelfTest4C_H[2][0],                                      /* Phase W = PWM High */
        (uint16_t)&tAdcSelfTest4C_H[3][0],                                      /* Phase T = PWM High */
        (uint16_t)&tAdcSelfTest4C_L[2][0],                                      /* Phase W = PWM Low */
        (uint16_t)&tAdcSelfTest4C_L[3][0],                                      /* Phase T = PWM Low */
        (uint16_t)&tAdcSelfTest4C_H[0][0],                                      /* Phase = High */
        (uint16_t)&tAdcSelfTest4C_H[2][0]                                       /* Phase = High */
#elif (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_WT)
        (uint16_t)&tAdcSelfTest4C_H[0][0],                                      /* Phase U = PWM High */
        (uint16_t)&tAdcSelfTest4C_H[2][0],                                      /* Phase W = PWM High */
        (uint16_t)&tAdcSelfTest4C_L[0][0],                                      /* Phase U = PWM Low */
        (uint16_t)&tAdcSelfTest4C_L[2][0],                                      /* Phase W = PWM Low */
        (uint16_t)&tAdcSelfTest4C_H[1][0],                                      /* Phase V = PWM High */
        (uint16_t)&tAdcSelfTest4C_H[3][0],                                      /* Phase T = PWM High */
        (uint16_t)&tAdcSelfTest4C_L[1][0],                                      /* Phase V = PWM Low */
        (uint16_t)&tAdcSelfTest4C_L[3][0],                                      /* Phase T = PWM Low */
        (uint16_t)&tAdcSelfTest4C_H[0][0],                                      /* Phase = High */
        (uint16_t)&tAdcSelfTest4C_H[1][0]                                       /* Phase = High */
#elif (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UT_VW)
        (uint16_t)&tAdcSelfTest4C_H[0][0],                                      /* Phase U = PWM High */
        (uint16_t)&tAdcSelfTest4C_H[1][0],                                      /* Phase V = PWM High */
        (uint16_t)&tAdcSelfTest4C_L[0][0],                                      /* Phase U = PWM Low */
        (uint16_t)&tAdcSelfTest4C_L[1][0],                                      /* Phase V = PWM Low */
        (uint16_t)&tAdcSelfTest4C_H[3][0],                                      /* Phase T = PWM High */
        (uint16_t)&tAdcSelfTest4C_H[2][0],                                      /* Phase W = PWM High */
        (uint16_t)&tAdcSelfTest4C_L[3][0],                                      /* Phase T = PWM Low */
        (uint16_t)&tAdcSelfTest4C_L[2][0],                                      /* Phase W = PWM Low */
        (uint16_t)&tAdcSelfTest4C_H[0][0],                                      /* Phase = High */
        (uint16_t)&tAdcSelfTest4C_H[3][0]                                       /* Phase = High */
#endif
    };
    static const ADC_SDATA_t JTEMP =
    {
        {
            .u2AdcMarker = C_ADC_NO_SIGN,
            .u5AdcChannel = C_ADC_TEMP,
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
            .u3AdcVref = C_ADC_VREF_2_50_V,
#elif defined (__MLX81339__) || defined (__MLX81350__)
            .u3AdcReserved = 0U,
#endif
            .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CMP,
            .u1AdcReserved = 0U
        }
    };

    uint16_t u16SelfTestIdx;
    T_ADC_SELFTEST_4PH_C sMotorSelfTestC;                                       /* Coil connect/open test */

#if (_DEBUG_SELFTEST != FALSE)
    DEBUG_SET_IO_B();
#endif /* (_DEBUG_SELFTEST != FALSE) */

    for (u16SelfTestIdx = 0U;
         (g_e8ErrorElectric == (uint8_t)C_ERR_NONE) &&
         (u16SelfTestIdx < (sizeof(c_au16DrvCfgSelfTestC4) / sizeof(c_au16DrvCfgSelfTestC4[0])));
         u16SelfTestIdx++)
    {
        /* Test for open connections, damaged coil(s) */
        uint16_t u16Vsm;
        uint16_t u16VphH;
        uint16_t u16VphL;
        uint16_t u16Vds;
        uint16_t u16MotorCoilCurrent;
        register uint16_t u16DC;
        if ( (u16SelfTestIdx & 0x02U) != 0U)
        {
            /* Phase LOW + phase -PWM */
            u16DC = (PWM_REG_PERIOD >> 1);                                      /* Approx. 50% */
        }
        else
        {
            /* Phase HIGH + phase PWM */
            u16DC = PWM_REG_PERIOD - (PWM_REG_PERIOD >> 1);                     /* Approx. 50% */
        }
        /* Independent-mode */
        u16DC = u16DC / 2U;
        IO_PWM_SLAVE1_HT = PWM_REG_PERIOD - u16DC;
        IO_PWM_SLAVE1_LT = u16DC;                                               /* Copy the results into the PWM register for phase V */
        IO_PWM_SLAVE2_HT = PWM_REG_PERIOD - u16DC;
        IO_PWM_SLAVE2_LT = u16DC;                                               /* Copy the results into the PWM register for phase W */
        IO_PWM_SLAVE3_HT = PWM_REG_PERIOD - u16DC;
        IO_PWM_SLAVE3_LT = u16DC;                                               /* Copy the results into the PWM register for phase T */
        IO_PWM_MASTER1_HT = PWM_REG_PERIOD - u16DC;
        IO_PWM_MASTER1_LT = u16DC;                                              /* Copy the results into the PWM register for phase U */

        HAL_ADC_StopSafe();                                                     /* Clear the ADC control register */
        uint16_t *pu16Src = &l_au16AdcSource[0];
        uint16_t *pu16X = (uint16_t *)tAdcSelfTest4C[u16SelfTestIdx];
        *pu16Src++ = (uint16_t)&sMotorSelfTestC;
        *pu16Src++ = JTEMP.u16;
        *pu16Src++ = pu16X[0];
        *pu16Src++ = pu16X[1];
        *pu16Src++ = pu16X[2];
        *pu16Src++ = pu16X[3];
        *pu16Src++ = C_ADC_EOS;

        HAL_ADC_Setup(/*B_ADC_ADC_WIDTH |*/                                     /* Setup Self Test measurement */
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
        HAL_ADC_ClearErrors();                                                  /* Prior to start, first clear any error flag and enable Triggers */

        HAL_PWM_MasterPendClear();                                              /* Clear PWM-module Master1-End IRQ's */
        HAL_PWM_MasterPendWait();                                               /* Wait for PWM Master PEND-flag */
        DRVCFG_CNFG_TUVW( (uint16_t)c_au16DrvCfgSelfTestC4[u16SelfTestIdx]);
        DRVCFG_ENA_TUVW();

        DELAY_US( (C_COIL_L >> 5) + 50U);                                       /* Delay of (dI * L) / U, with dI > 10 ADC-LSB's = ~60mA, U = 12V */

        HAL_ADC_Start();                                                        /* Start ADC (2 PWM-periods) */

        /* This takes about 4 Motor PWM-periods per self-test */
#if (_SUPPORT_WHILE_RETRY_LIMITED != FALSE)
        uint16_t u16Retries = 100U;
        while ( (IO_ADC_CTRL & B_ADC_START) != 0U)
        {
            if (--u16Retries == 0U)
            {
#if (_SUPPORT_LOG_ERRORS != FALSE)
                SetLastError(C_ERR_TO_MDST_COIL_OPEN);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                break;
            }
            DELAY_US(5);
        }                  /* Wait for ADC result (Time-out?) */
#else  /* (_SUPPORT_WHILE_RETRY_LIMITED != FALSE) */
        while ( (IO_ADC_CTRL & B_ADC_START) != 0U) {}                           /* Wait for ADC result (Time-out?) */
#endif /* (_SUPPORT_WHILE_RETRY_LIMITED != FALSE) */

        if (g_e8ErrorElectric != (uint8_t)C_ERR_NONE)                           /* Over-current ? */
        {
            /* Over-current trigger; Phase makes short with other phase */
            g_e8ErrorElectric |= (uint8_t)C_ERR_PERMANENT;
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_SELFTEST_B | (C_ERR_EXT + (u16SelfTestIdx << 8)));
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            break;
        }

        DRVCFG_TRI_TUVW();

        if ( (u16SelfTestIdx & 2U) != 0U)
        {
            /* Use tAdcSelfTest4B */
            u16Vsm = sMotorSelfTestC.MotorDriverCurrent;                        /* Current becomes voltage */
            u16MotorCoilCurrent = sMotorSelfTestC.MotorDriverVoltage;
        }
        else
        {
            /* Use tAdcSelfTest4A */
            u16Vsm = sMotorSelfTestC.MotorDriverVoltage;
            u16MotorCoilCurrent = sMotorSelfTestC.MotorDriverCurrent;
        }

        u16VphH = sMotorSelfTestC.Phase_HighVoltage;
        u16VphL = sMotorSelfTestC.Phase_LowVoltage;

        if (u16Vsm > u16VphH)
        {
            u16Vds = u16Vsm - u16VphH;
        }
        else
        {
            u16Vds = u16VphH - u16Vsm;
        }
        if (u16Vds > u16VdsThreshold)
        {
            g_e8ErrorElectric |= (uint8_t)(C_ERR_PERMANENT | C_ERR_VDS);
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_SELFTEST_D | (C_ERR_EXT + (u16SelfTestIdx << 8)));
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            break;
        }
        if (u16VphL > u16VdsThreshold)
        {
            g_e8ErrorElectric |= (uint8_t)(C_ERR_PERMANENT | C_ERR_VDS);
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_SELFTEST_E | (C_ERR_EXT + (u16SelfTestIdx << 8)));
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            break;
        }
        if ( (int16_t)(u16MotorCoilCurrent - Get_CurrentZeroOffset()) < (int16_t)C_MIN_MOTORCURRENT)
        {
            /* No current (less than 10 LSB's); Coil Open */
            g_e8ErrorElectric |= (uint8_t)(C_ERR_PERMANENT | C_ERR_MOTOR_ZERO_CURRENT);
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_SELFTEST_C | (C_ERR_EXT + (u16SelfTestIdx << 8)));
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            break;
        }

        /* Time to allow the coil current to recirculate */
        DELAY_US( (C_COIL_L >> 8) + 50U);
    }

    (void)tAdcSelfTest4C_H;
    (void)tAdcSelfTest4C_L;
#if (_DEBUG_SELFTEST != FALSE)
    DEBUG_CLR_IO_B();
#endif /* (_DEBUG_SELFTEST != FALSE) */
} /* End of MotorDriverSelfTestCoilOpen() */

/*!*************************************************************************** *
 * MotorDriverSelfTest
 * \brief   Perform driver Self-test
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Self test Motor Driver
 * Note: Diagnostics must be initialised before calling this self-test.
 * Total time: Approximate: 4.5 ms
 * 1. Pre-test:        0.4 ms
 * 2. Pin-test:        0.8 ms
 * 3. Coil-short test: 0.4 ms
 * 4. Coil-open test:  1.5 ms + 1.0ms delay
 *    Over head:        12%
 *
 * 1. Test FET short with Ground or Vsupply
 * 2. Test Motor-phase short
 * 3. Test Open connection with motor-phase
 * 4. Test BEMF Voltage levels
 * *************************************************************************** *
 * - Call Hierarchy: mainInit()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 6 (MotorDriverSelfTestInit(), MotorDriverSelfTestPin(),
 *                      MotorDriverSelfTestPin2Pin(), MotorDriverSelfTestCoilShort(),
 *                      MotorDriverSelfTestCoilOpen(), MotorDriverConfig())
 * *************************************************************************** */
void MotorDriverSelfTest(void)
{
    uint16_t u16VdsThreshold;                                                   /* MMP130919-1/MMP140403-1 */

#if (_DEBUG_SELFTEST != FALSE)
    DEBUG_SET_IO_A();                                                           /* Start of Self-Test */
#endif /* (_DEBUG_SELFTEST != FALSE) */

    MotorDriverConfig(TRUE);

    u16VdsThreshold = MotorDriverSelfTestInit();

    /* Pre-check phase-pins (floating) */
    MotorDriverSelfTestPin(u16VdsThreshold);

    /* Pin-test for FET-shortage */
    MotorDriverSelfTestPin2Pin(u16VdsThreshold);

    /* Coil-short test */
    MotorDriverSelfTestCoilShort(u16VdsThreshold);

    /* Coil open test */
    MotorDriverSelfTestCoilOpen(u16VdsThreshold);

    DRVCFG_DIS_TUVW();
    IO_PWM_SLAVE1_LT = PWM_SCALE_OFFSET;                                        /* Copy the results into the PWM register for phase V */
    IO_PWM_SLAVE2_LT = PWM_SCALE_OFFSET;                                        /* Copy the results into the PWM register for phase W */
    IO_PWM_SLAVE3_LT = PWM_SCALE_OFFSET;                                        /* Copy the results into the PWM register for phase T */
    IO_PWM_MASTER1_LT = PWM_SCALE_OFFSET;                                       /* Copy the results into the PWM register for phase U */

#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
    if (((g_e8ErrorElectric & (uint8_t)C_ERR_PERMANENT) != 0U) ||               /* MMP220729-1: In case of permanent error or ... */
        (l_u8MotorHoldingCurrState == FALSE) )                                  /* ... no holding current required */
    {
        MotorDriverConfig(FALSE);                                               /* Disable Motor Driver */
        l_u8MotorHoldingCurrState = FALSE;
    }
    else
    {
        /* MotorDriverInit() has already enabled MotorDriverConfig(); Remain it active */
    }
#else  /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
    MotorDriverConfig(FALSE);                                                   /* Disable Motor Driver */
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */

#if (_DEBUG_SELFTEST != FALSE)
    DEBUG_CLR_IO_A();                                                           /* End of Self-Test */
#endif /* (_DEBUG_SELFTEST != FALSE) */

} /* End of MotorDriverSelfTest() */
#else
#error "ERROR: No Motor SelfTest available (DC/SCF)"
#endif /* (C_MOTOR_PHASES == 3) */

#endif /* (_SUPPORT_MOTOR_SELFTEST != FALSE) */

#if (_SUPPORT_OSD != FALSE) && (!defined (__MLX81160__) && !defined (__MLX81330__) && !defined (__MLX81339__) && !defined (__MLX81350__))
/*!*************************************************************************** *
 * MotorDriverTest_OSD
 * \brief   Motor Driver Test - Off State Diagnostics
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details The OSD can only be called when the motor driver is not activated.
 * *************************************************************************** *
 * - Call Hierarchy: AppInit()
 * - Cyclomatic Complexity: 5/8+1
 * - Nesting: 2
 * - Function calling: 3 (ADC_OSD_Start(), MotorDriverPermanentError(), SetLastError())
 * *************************************************************************** */
void MotorDriverTest_OSD(void)
{
    /* OSD works only when driver has been disabled (and no current is in the motor coils) */
#if (_DEBUG_OSD != FALSE)
    DEBUG_SET_IO_A();
#endif /* (_DEBUG_OSD != FALSE) */
#if defined (__MLX81332__) || defined (__MLX81334__)
    IO_PORT_MISC2_OUT = IO_PORT_MISC2_OUT | M_PORT_MISC2_OUT_ENA_OSD_DRV;       /* Current Source U & T: ON */
#if (_DEBUG_OSD != FALSE)
    DEBUG_CLR_IO_A();
#endif /* (_DEBUG_OSD == FALSE) */
    DELAY_US(25U);                                                              /* Minimum 50us delay to settle */
#if (_DEBUG_OSD != FALSE)
    DEBUG_SET_IO_A();
#endif /* (_DEBUG_OSD == FALSE) */
    ADC_OSD_Start();
    IO_PORT_MISC2_OUT = IO_PORT_MISC2_OUT & ~M_PORT_MISC2_OUT_ENA_OSD_DRV;      /* Current Source U & T: OFF */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    IO_PORT_MISC2_OUT = IO_PORT_MISC2_OUT | M_PORT_MISC2_OUT_ENABLE_OSD_DRV;    /* Current Source U: ON and Pull-down on V & W of 35kR */
#if (_DEBUG_OSD != FALSE)
    DEBUG_CLR_IO_A();
#endif /* (_DEBUG_OSD != FALSE) */
    DELAY_US(2000U);                                                            /* Minimum 2000us delay to settle */
#if (_DEBUG_OSD != FALSE)
    DEBUG_SET_IO_A();
#endif /* (_DEBUG_OSD != FALSE) */
    ADC_OSD_Start();
    IO_PORT_MISC2_OUT = IO_PORT_MISC2_OUT & ~M_PORT_MISC2_OUT_ENABLE_OSD_DRV;   /* Current Source U & T: OFF*/
#endif
#if (_DEBUG_OSD != FALSE)
    DEBUG_CLR_IO_A();
#endif /* (_DEBUG_OSD == FALSE) */
    {
        int16_t i16VphU = AdcOsdResult.u16AdcVphU - Get_VsmOffset();
        int16_t i16VphV = AdcOsdResult.u16AdcVphV - Get_VsmOffset();
        int16_t i16VphW = AdcOsdResult.u16AdcVphW - Get_VsmOffset();
#if (C_MOTOR_PHASES == 3U)
        /* Check open connection U */
        if ( (i16VphU > C_VTH_HIGH) ||                                          /* Phase U is disconnected or short to supply */
             (i16VphV > C_VTH_HIGH) ||                                          /* Phase V is short to supply */
             (i16VphW > C_VTH_HIGH) )                                           /* Phase W is short to supply */
        {
            if ( (i16VphV < C_VTH_LOW) && (i16VphW < C_VTH_LOW) )
            {
                /* Motor absent or Phase U is disconnected or Phase short to GND */
                MotorDriverPermanentError(C_ERR_PERMANENT | C_ERR_MOTOR_ZERO_CURRENT);
#if (_SUPPORT_LOG_ERRORS != FALSE)
                SetLastError(C_ERR_COIL_ZERO_CURRENT | C_ERR_EXT | 0x0100U);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            }
            else
            {
                /* Phase U and/or V and/or W short to supply */
                MotorDriverPermanentError(C_ERR_PERMANENT | C_ERR_MOTOR_OVER_CURRENT);
#if (_SUPPORT_LOG_ERRORS != FALSE)
                SetLastError(C_ERR_COIL_OVER_CURRENT | C_ERR_EXT | 0x0200U);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            }
        }
        else if (i16VphU < C_VTH_LOW)
        {
            /* Phase U is short to Ground */
            MotorDriverPermanentError(C_ERR_PERMANENT | C_ERR_MOTOR_OVER_CURRENT);
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_COIL_OVER_CURRENT | C_ERR_EXT | 0x0300U);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
        }
        if ( (i16VphV < C_VTH_LOW) && (i16VphU < C_VTH_HIGH) )
        {
            /* Phase V is short to Ground, or disconnected */
            MotorDriverPermanentError(C_ERR_PERMANENT | C_ERR_MOTOR_ZERO_CURRENT | C_ERR_MOTOR_OVER_CURRENT);
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_COIL_OVER_CURRENT | C_ERR_EXT | 0x0400U);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
        }
        if ( (i16VphW < C_VTH_LOW) && (i16VphU < C_VTH_HIGH) )
        {
            /* Phase W is short to Ground, or disconnected */
            MotorDriverPermanentError(C_ERR_PERMANENT | C_ERR_MOTOR_ZERO_CURRENT | C_ERR_MOTOR_OVER_CURRENT);
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_COIL_OVER_CURRENT | C_ERR_EXT | 0x0500U);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
        }
#elif (C_MOTOR_PHASES == 4U)
        int16_t i16VphT = AdcOsdResult.u16AdcVphT - Get_VsmOffset();
        /* Check open connection U */
#if (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_WT)
        if ( (i16VphU > C_VTH_HIGH) ||                                          /* Phase U is disconnected or short to supply */
             (i16VphV > C_VTH_HIGH) )                                           /* Phase V is short to supply */
        {
            if (i16VphV < C_VTH_LOW)
            {
                /* Motor absent or Phase U is disconnected */
                MotorDriverPermanentError(C_ERR_PERMANENT | C_ERR_MOTOR_ZERO_CURRENT);
#if (_SUPPORT_LOG_ERRORS != FALSE)
                SetLastError(C_ERR_COIL_ZERO_CURRENT | C_ERR_EXT | 0x0100U);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            }
            else
            {
                /* Phase U and/or V short to supply */
                MotorDriverPermanentError(C_ERR_PERMANENT | C_ERR_MOTOR_OVER_CURRENT);
#if (_SUPPORT_LOG_ERRORS != FALSE)
                SetLastError(C_ERR_COIL_OVER_CURRENT | C_ERR_EXT | 0x0200U);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            }
        }
#elif (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UW_VT)
        if ( (i16VphU > C_VTH_HIGH) ||                                          /* Phase U is disconnected or short to supply */
             (i16VphW > C_VTH_HIGH) )                                           /* Phase W is short to supply */
        {
            if (i16VphW < C_VTH_LOW)
            {
                /* Motor absent or Phase U is disconnected */
                MotorDriverPermanentError(C_ERR_PERMANENT | C_ERR_MOTOR_ZERO_CURRENT);
#if (_SUPPORT_LOG_ERRORS != FALSE)
                SetLastError(C_ERR_COIL_ZERO_CURRENT | C_ERR_EXT | 0x0100U);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            }
            else
            {
                /* Phase U and/or W short to supply */
                MotorDriverPermanentError(C_ERR_PERMANENT | C_ERR_MOTOR_OVER_CURRENT);
#if (_SUPPORT_LOG_ERRORS != FALSE)
                SetLastError(C_ERR_COIL_OVER_CURRENT | C_ERR_EXT | 0x0200U);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            }
        }
#endif
        else if (i16VphU < C_VTH_LOW)
        {
            /* Phase U is short to Ground */
            MotorDriverPermanentError(C_ERR_PERMANENT | C_ERR_MOTOR_OVER_CURRENT);
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_COIL_OVER_CURRENT | C_ERR_EXT | 0x0300U);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
        }
#if (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_WT)
        if ( (i16VphV < C_VTH_LOW) && (i16VphU < C_VTH_HIGH) )
        {
            /* Phase V is short to Ground, or disconnected */
            MotorDriverPermanentError(C_ERR_PERMANENT | C_ERR_MOTOR_ZERO_CURRENT | C_ERR_MOTOR_OVER_CURRENT);
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_COIL_OVER_CURRENT | C_ERR_EXT | 0x0400U);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
        }
#elif (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UW_VT)
        if ( (i16VphW < C_VTH_LOW) && (i16VphU < C_VTH_HIGH) )
        {
            /* Phase W is short to Ground, or disconnected */
            MotorDriverPermanentError(C_ERR_PERMANENT | C_ERR_MOTOR_ZERO_CURRENT | C_ERR_MOTOR_OVER_CURRENT);
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_COIL_OVER_CURRENT | C_ERR_EXT | 0x0500U);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
        }
#endif

        /* Check open connection T */
#if (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_WT)
        if ( (i16VphT > C_VTH_HIGH) ||                                          /* Phase T is disconnected or short to supply */
             (i16VphW > C_VTH_HIGH) )                                           /* Phase W is short to supply */
        {
            if (i16VphW < C_VTH_LOW)
            {
                /* Motor absent or Phase U is disconnected */
                MotorDriverPermanentError(C_ERR_PERMANENT | C_ERR_MOTOR_ZERO_CURRENT);
#if (_SUPPORT_LOG_ERRORS != FALSE)
                SetLastError(C_ERR_COIL_ZERO_CURRENT | C_ERR_EXT | 0x0100U);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            }
            else
            {
                /* Phase T and/or W short to supply */
                MotorDriverPermanentError(C_ERR_PERMANENT | C_ERR_MOTOR_OVER_CURRENT);
#if (_SUPPORT_LOG_ERRORS != FALSE)
                SetLastError(C_ERR_COIL_OVER_CURRENT | C_ERR_EXT | 0x0200U);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            }
        }
#elif (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UW_VT)
        if ( (i16VphT > C_VTH_HIGH) ||                                          /* Phase T is disconnected or short to supply */
             (i16VphV > C_VTH_HIGH) )                                           /* Phase V is short to supply */
        {
            if (i16VphV < C_VTH_LOW)
            {
                /* Motor absent or Phase T is disconnected */
                MotorDriverPermanentError(C_ERR_PERMANENT | C_ERR_MOTOR_ZERO_CURRENT);
#if (_SUPPORT_LOG_ERRORS != FALSE)
                SetLastError(C_ERR_COIL_ZERO_CURRENT | C_ERR_EXT | 0x0100U);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            }
            else
            {
                /* Phase T and/or V short to supply */
                MotorDriverPermanentError(C_ERR_PERMANENT | C_ERR_MOTOR_OVER_CURRENT);
#if (_SUPPORT_LOG_ERRORS != FALSE)
                SetLastError(C_ERR_COIL_OVER_CURRENT | C_ERR_EXT | 0x0200U);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            }
        }
#endif
        else if (i16VphT < C_VTH_LOW)
        {
            /* Phase T is short to Ground */
            MotorDriverPermanentError(C_ERR_PERMANENT | C_ERR_MOTOR_OVER_CURRENT);
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_COIL_OVER_CURRENT | C_ERR_EXT | 0x0300U);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
        }
#if (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_WT)
        if ( (i16VphW < C_VTH_LOW) && (i16VphT < C_VTH_HIGH) )
        {
            /* Phase W is short to Ground, or disconnected */
            MotorDriverPermanentError(C_ERR_PERMANENT | C_ERR_MOTOR_ZERO_CURRENT | C_ERR_MOTOR_OVER_CURRENT);
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_COIL_OVER_CURRENT | C_ERR_EXT | 0x0400U);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
        }
#elif (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UW_VT)
        if ( (i16VphV < C_VTH_LOW) && (i16VphT < C_VTH_HIGH) )
        {
            /* Phase V is short to Ground, or disconnected */
            MotorDriverPermanentError(C_ERR_PERMANENT | C_ERR_MOTOR_ZERO_CURRENT | C_ERR_MOTOR_OVER_CURRENT);
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_COIL_OVER_CURRENT | C_ERR_EXT | 0x0500U);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
        }
#endif
#endif /* (C_MOTOR_PHASES == 4U) */
    }
} /* End of MotorDriverTest_OSD() */
#endif /* (_SUPPORT_OSD != FALSE) && (!defined (__MLX81160__) && !defined (__MLX81330__) && !defined (__MLX81339__) && !defined (__MLX81350__)) */

#endif /* (_SUPPORT_MOTOR_SELFTEST != FALSE) || (_SUPPORT_OSD != FALSE) */

/* EOF */
