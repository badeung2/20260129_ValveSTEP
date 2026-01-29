/*!*************************************************************************** *
 * \file        MotorDriverTables.h
 * \brief       MLX813xx Motor Driver Waveform Tables
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
 * *************************************************************************** */

#ifndef DRIVE_LIB_MOTOR_DRIVERTABLES_H
#define DRIVE_LIB_MOTOR_DRIVERTABLES_H

/*!*************************************************************************** */
/*                           INCLUDES                                          */
/* *************************************************************************** */
#include "AppBuild.h"                                                           /* Application Build */

#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
/*!*************************************************************************** */
/*                           DEFINITIONS                                       */
/* *************************************************************************** */
#if (C_MOTOR_PHASES != 1)
#define SZ_MICRO_VECTOR_TABLE_3PH   ((5U * (6U * C_MICROSTEP_PER_FULLSTEP)) / 3U)  /*!< Vector table size */
#define SZ_MICRO_VECTOR_TABLE_4PH   (5U * C_MICROSTEP_PER_FULLSTEP)             /*!< Table-size: 2*2 * 16uStep * (1 + 1/4) */
#define SZ_MICRO_VECTOR_TABLE_3PH_2COIL (6U * C_MICROSTEP_PER_FULLSTEP)         /*!< Table-size: Dual-coil, three-phase */

#if (_SUPPORT_MULTI_VECTOR_WAVEFORM != FALSE)
#define C_SPACE_VECTOR_SINE_ID          0
#define C_SPACE_VECTOR_TRIPLE_SINE_ID   1
#define C_SPACE_VECTOR_FIFTH_SINE_ID    2
#define C_SPACE_VECTOR_TRAPEZOIDAL_ID   3                                       /* Trapezoidal */
extern int16_t const c_ai16MicroStepVector4PH[4][SZ_MICRO_VECTOR_TABLE_4PH];
#else  /* (_SUPPORT_MULTI_VECTOR_WAVEFORM != FALSE) */
/* Select max 1 space-vector wave-form */
#define SPACE_VECTOR_SINE               TRUE                                    /*!< Single Sine: SIN(x) */
#define SPACE_VECTOR_TRIPLE_SINE        FALSE                                   /*!< Triple Sine: 1.149571 * {SIN(x) + 0.2 * SIN(3 * x)}; Current increase by 21-29% */
#define SPACE_VECTOR_FIFTH_SINE         FALSE                                   /*!< Fifth Sine: 1.0373751 * {Triple Sinus "B" + 0.04 * SIN(5 * x)} */
#define SPACE_VECTOR_TRAPIZE            FALSE                                   /*!< Trapezoidal */
#define SPACE_VECTOR_BLOCK              FALSE                                   /*!< Block */
extern int16_t const c_ai16MicroStepVector4PH[SZ_MICRO_VECTOR_TABLE_4PH];       /*!< Micro-step waveform table */
#endif /* (_SUPPORT_MULTI_VECTOR_WAVEFORM != FALSE) */

#if (_SUPPORT_PWM_MODE == BIPOLAR_TRIPHASE_TWOPWM_INDEPENDENT_GND)
/* Tables for Bipolar as 3-phase, 2-phase PWM and GND */
extern uint16_t const c_au16MicroStepVector3PH_DualCoil_A[SZ_MICRO_VECTOR_TABLE_3PH_2COIL];  /*!< Micro-step Vector-table (3-phase) for Dual-coil actuator "A" */
extern uint16_t const c_au16MicroStepVector3PH_DualCoil_B[SZ_MICRO_VECTOR_TABLE_3PH_2COIL];  /*!< Micro-step Vector-table (3-phase) for Dual-coil actuator "B" */
#elif (_SUPPORT_PWM_MODE == BIPOLAR_TRIPHASE_ALLPWM_MIRROR)
/* Tables for Bipolar as 3-phase, 3-phase PWM Mirror mode */
extern int16_t const c_ai16MicroStepVector3PH_DualCoil_A[SZ_MICRO_VECTOR_TABLE_3PH_2COIL];
extern int16_t const c_ai16MicroStepVector3PH_DualCoil_B[SZ_MICRO_VECTOR_TABLE_3PH_2COIL];
#else  /* (_SUPPORT_PWM_MODE == BIPOLAR_TRIPHASE_TWOPWM_INDEPENDENT_GND) || (_SUPPORT_PWM_MODE == BIPOLAR_TRIPHASE_ALLPWM_MIRROR) */
#if (_SUPPORT_PWM_MODE == TRIPLEPHASE_ALLPWM_MIRROR)
extern int16_t const c_ai16MicroStepVector3PH[SZ_MICRO_VECTOR_TABLE_3PH + C_MICROSTEP_PER_FULLSTEP];
#else  /* (_SUPPORT_PWM_MODE == TRIPLEPHASE_ALLPWM_MIRROR) */
extern uint16_t const c_au16MicroStepVector3PH[SZ_MICRO_VECTOR_TABLE_3PH + C_MICROSTEP_PER_FULLSTEP];
#endif /* (_SUPPORT_PWM_MODE == TRIPLEPHASE_ALLPWM_MIRROR) */
#if (_SUPPORT_BOOST_MODE == BOOST_MODE_WAVEFORM)
extern uint16_t const c_au16MicroStepVector3PH_Boost[SZ_MICRO_VECTOR_TABLE_3PH + C_MICROSTEP_PER_FULLSTEP];
#endif /* (_SUPPORT_BOOST_MODE == BOOST_MODE_WAVEFORM) */
#endif /* (_SUPPORT_PWM_MODE == BIPOLAR_TRIPHASE_TWOPWM_INDEPENDENT_GND) || (_SUPPORT_PWM_MODE == BIPOLAR_TRIPHASE_ALLPWM_MIRROR) */

#if ((_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (C_MOTOR_PHASES != 1U)) || (_SUPPORT_STALLDET_LA != FALSE) || (_SUPPORT_ANTICOGGING != FALSE) || (_DEBUG_PWM_TO_IO != FALSE)
#if (_SUPPORT_SINCOS_TABLE_SZ == 192)
extern int16_t const c_ai16MicroStepVector3PH_SinCos192[192U + 48U];            /*!< SinCos-table */
#define C_COS_OFFSET            (192U / 4U)                                     /*!< Co-sinus-offset in micro-steps */
#elif (_SUPPORT_SINCOS_TABLE_SZ == 256)
extern int16_t const c_ai16MicroStepVector3PH_SinCos256[256U + 64U];            /*!< SinCos-table */
#define C_COS_OFFSET            (256U / 4U)                                     /*!< Co-sinus-offset in micro-steps */
#elif (_SUPPORT_SINCOS_TABLE_SZ == 1024)
extern int16_t const c_ai16MicroStepVector3PH_SinCos256[384U + 96U];            /*!< SinCos-table */
#define C_COS_OFFSET            (1024U / 4U)                                    /*!< Co-sinus-offset in micro-steps */
#endif
#endif /* ((_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (C_MOTOR_PHASES != 1U)) || (_SUPPORT_STALLDET_LA != FALSE) || (_SUPPORT_ANTICOGGING != FALSE) || (_DEBUG_PWM_TO_IO != FALSE) */

extern uint16_t const c_au16DrvCfg[C_MOTOR_PHASES * C_MOTOR_STEP_PER_PHASE];    /*!< Motor-Drive configuration table */

#else  /* (C_MOTOR_PHASES != 1) */
extern uint16_t const c_au16DrvCfgDC_AB[2];
extern uint16_t const c_au16DrvCfgDC_A[2];
extern uint16_t const c_au16DrvCfgDC_B[2];

extern uint16_t const c_au16DrvCfgDC1[2];

/* Set-Test - Part A: Single FET ON */
extern uint16_t const c_au8DrvCfgSelfTestA[8];

#if (C_MICROSTEP_PER_FULLSTEP != 0)
#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR)
#define SPACE_VECTOR_SINE               TRUE                                    /*!< Waveform choice: SINE */
#define SPACE_VECTOR_ASYM_SINE          FALSE                                   /*!< Waveform choice: A-SYMmetric SINE */
#define SPACE_VECTOR_BEMF_SHAPE         FALSE                                   /*!< Waveform choice: BEMF */
#define SPACE_VECTOR_DOUBLE_SINE        FALSE                                   /*!< Waveform choice: DOUBLE_SINE */
#define SPACE_VECTOR_TRIPLE_SINE        FALSE                                   /*!< Waveform choice: TRIPPLE_SINE */
#define SPACE_VECTOR_FIFTH_SINE         FALSE                                   /*!< Waveform choice: FIFTH_SINE */
#define SPACE_VECTOR_TRAPIZE            FALSE                                   /*!< Waveform choice: TRAPIZE */
#define SPACE_VECTOR_BLOCK              FALSE                                   /*!< Waveform choice: BLOCK */
#else  /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) */
#define SPACE_VECTOR_SINE               FALSE                                   /*!< Waveform choice: SINE */
#define SPACE_VECTOR_BEMF_SHAPE         FALSE                                   /*!< Waveform choice: BEMF */
#define SPACE_VECTOR_DOUBLE_SINE        FALSE                                   /*!< Waveform choice: DOUBLE_SINE */
#define SPACE_VECTOR_TRIPLE_SINE        FALSE                                   /*!< Waveform choice: TRIPPLE_SINE */
#define SPACE_VECTOR_FIFTH_SINE         FALSE                                   /*!< Waveform choice: FIFTH_SINE */
#define SPACE_VECTOR_TRAPIZE            FALSE                                   /*!< Waveform choice: TRAPIZE */
#define SPACE_VECTOR_BLOCK              TRUE                                    /*!< Waveform choice: BLOCK */
#endif /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) */

extern uint16_t const c_au16MicroStepVector[C_MICROSTEP_PER_FULLSTEP];
#if (_SUPPORT_HALL_LATCH == FALSE)
extern uint16_t const c_au16AlignMicroStepVector[C_MICROSTEP_PER_FULLSTEP];
#endif /* (_SUPPORT_HALL_LATCH == FALSE) */
#endif /* (C_MICROSTEP_PER_FULLSTEP != 0) */
#endif /* (C_MOTOR_PHASES != 1) */

#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT) */

#endif /* DRIVE_LIB_MOTOR_DRIVERTABLES_H */

/* EOF */

