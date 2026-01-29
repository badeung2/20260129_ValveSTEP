/*!************************************************************************** *
 * \file        HAL_ADC_channels.h
 * \brief       Hardware Abstraction Layer for ADC channels
 *
 * \note        project MLX81160/33x/34x/35x
 *
 * \author      Marcel Braat
 *
 * \date        2024-03-24
 *
 * \version     2.0
 *
 * A list of functions:
 *  - Inline Functions:
 *
 *
 * MELEXIS Microelectronic Integrated Systems
 *
 * Copyright (C) 2024-2024 Melexis N.V.
 * The Software is being delivered 'AS IS' and Melexis, whether explicitly or
 * implicitly, makes no warranty as to its Use or performance.
 * The user accepts the Melexis Firmware License Agreement.
 *
 * Melexis confidential & proprietary
 *
 * ************************************************************************** */

#ifndef HAL_LIB_ADC_CHANNELS_H
#define HAL_LIB_ADC_CHANNELS_H

/*!************************************************************************** */
/*                          INCLUDES                                          */
/* ************************************************************************** */
#include "../AppBuild.h"                                                        /* Application Build */

/*!*************************************************************************** */
/*                           DEFINITIONS                                       */
/* *************************************************************************** */
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
#if (_SUPPORT_ADC_REF_HV_CALIB != FALSE)                                        /* MMP190523-2 */
#define C_ADC_VREF_HV       C_ADC_VREF_1_50_V
#else  /* (_SUPPORT_ADC_REF_HV_CALIB != FALSE) */                               /* MMP190523-2 */
#define C_ADC_VREF_HV       C_ADC_VREF_2_50_V
#endif /* (_SUPPORT_ADC_REF_HV_CALIB != FALSE) */                               /* MMP190523-2 */

#if (_SUPPORT_ADC_REF_MC_CALIB != FALSE)
#define C_ADC_VREF_MCUR     C_ADC_VREF_1_50_V
#else  /* (_SUPPORT_ADC_REF_MC_CALIB != FALSE) */
#define C_ADC_VREF_MCUR     C_ADC_VREF_2_50_V
#endif /* (_SUPPORT_ADC_REF_MC_CALIB != FALSE) */

/* IO[0] HV */
#define C_ADC_IO0_HV_EOC                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_IO0_HV,                                       \
            .u3AdcVref = C_ADC_VREF_HV,                                         \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

/* IO[0] LV */
#define C_ADC_IO0_LV_EOC                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_IO0_LV,                                       \
            .u3AdcVref = C_ADC_VREF_2_50_V,                                     \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

/* IO[1] LV */
#define C_ADC_IO1_LV_EOC                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_IO1_LV,                                       \
            .u3AdcVref = C_ADC_VREF_2_50_V,                                     \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

/* IO[2] LV */
#define C_ADC_IO2_LV_EOC                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_IO2_LV,                                       \
            .u3AdcVref = C_ADC_VREF_2_50_V,                                     \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

/* IO[3] LV */
#define C_ADC_IO3_LV_EOC                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_IO3_LV,                                       \
            .u3AdcVref = C_ADC_VREF_2_50_V,                                     \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

/* IO[4] LV */
#define C_ADC_IO4_LV_EOC                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_IO4_LV,                                       \
            .u3AdcVref = C_ADC_VREF_2_50_V,                                     \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

/* IO[5] LV */
#define C_ADC_IO5_LV_EOC                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_IO5_LV,                                       \
            .u3AdcVref = C_ADC_VREF_2_50_V,                                     \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

/* IO[6] LV */
#define C_ADC_IO6_LV_EOC                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_IO6_LV,                                       \
            .u3AdcVref = C_ADC_VREF_2_50_V,                                     \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

/* IO[7] LV */
#define C_ADC_IO7_LV_EOC                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_IO7_LV,                                       \
            .u3AdcVref = C_ADC_VREF_2_50_V,                                     \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

#if (_SUPPORT_POTI_IO == PIN_FUNC_IO_0)
#define C_ADC_POTI_IOX_EOC C_ADC_IO0_LV_EOC
#elif (_SUPPORT_POTI_IO == PIN_FUNC_IO_1)
#define C_ADC_POTI_IOX_EOC C_ADC_IO1_LV_EOC
#elif (_SUPPORT_POTI_IO == PIN_FUNC_IO_2)
#define C_ADC_POTI_IOX_EOC C_ADC_IO2_LV_EOC
#elif (_SUPPORT_POTI_IO == PIN_FUNC_IO_3)
#define C_ADC_POTI_IOX_EOC C_ADC_IO3_LV_EOC
#elif !defined (__MLX81330__) && (_SUPPORT_POTI_IO == PIN_FUNC_IO_4)
#define C_ADC_POTI_IOX_EOC C_ADC_IO4_LV_EOC
#elif (_SUPPORT_POTI_IO == PIN_FUNC_IO_O_HV)
#define C_ADC_POTI_IOX_EOC C_ADC_IO0_HV_EOC
#else
#error "Error: POTI ADC I/O not configured"
#endif

#if (_SUPPORT_RESOLVER_X_IO == PIN_FUNC_IO_0)
#define C_ADC_RESOLVER_IOX_LV_EOC C_ADC_IO0_LV_EOC
#elif (_SUPPORT_RESOLVER_X_IO == PIN_FUNC_IO_1)
#define C_ADC_RESOLVER_IOX_LV_EOC C_ADC_IO1_LV_EOC
#elif (_SUPPORT_RESOLVER_X_IO == PIN_FUNC_IO_2)
#define C_ADC_RESOLVER_IOX_LV_EOC C_ADC_IO2_LV_EOC
#elif (_SUPPORT_RESOLVER_X_IO == PIN_FUNC_IO_3)
#define C_ADC_RESOLVER_IOX_LV_EOC C_ADC_IO3_LV_EOC
#elif (_SUPPORT_RESOLVER_X_IO == PIN_FUNC_IO_4)
#define C_ADC_RESOLVER_IOX_LV_EOC C_ADC_IO4_LV_EOC
#elif (_SUPPORT_RESOLVER_X_IO == PIN_FUNC_IO_5)
#define C_ADC_RESOLVER_IOX_LV_EOC C_ADC_IO5_LV_EOC
#elif (_SUPPORT_RESOLVER_X_IO == PIN_FUNC_IO_6)
#define C_ADC_RESOLVER_IOX_LV_EOC C_ADC_IO6_LV_EOC
#elif (_SUPPORT_RESOLVER_X_IO == PIN_FUNC_IO_7)
#define C_ADC_RESOLVER_IOX_LV_EOC C_ADC_IO7_LV_EOC
#else
#error "Error: TRIAXIS MLX9038x OUT1 not configured"
#endif

#if (_SUPPORT_RESOLVER_X_IO == PIN_FUNC_IO_0)
#define C_ADC_RESOLVER_IOY_LV_EOC C_ADC_IO0_LV_EOC
#elif (_SUPPORT_RESOLVER_Y_IO == PIN_FUNC_IO_1)
#define C_ADC_RESOLVER_IOY_LV_EOC C_ADC_IO1_LV_EOC
#elif (_SUPPORT_RESOLVER_Y_IO == PIN_FUNC_IO_2)
#define C_ADC_RESOLVER_IOY_LV_EOC C_ADC_IO2_LV_EOC
#elif (_SUPPORT_RESOLVER_Y_IO == PIN_FUNC_IO_3)
#define C_ADC_RESOLVER_IOY_LV_EOC C_ADC_IO3_LV_EOC
#elif (_SUPPORT_RESOLVER_Y_IO == PIN_FUNC_IO_4)
#define C_ADC_RESOLVER_IOY_LV_EOC C_ADC_IO4_LV_EOC
#elif (_SUPPORT_RESOLVER_Y_IO == PIN_FUNC_IO_5)
#define C_ADC_RESOLVER_IOY_LV_EOC C_ADC_IO5_LV_EOC
#elif (_SUPPORT_RESOLVER_Y_IO == PIN_FUNC_IO_6)
#define C_ADC_RESOLVER_IOY_LV_EOC C_ADC_IO6_LV_EOC
#elif (_SUPPORT_RESOLVER_Y_IO == PIN_FUNC_IO_7)
#define C_ADC_RESOLVER_IOY_LV_EOC C_ADC_IO7_LV_EOC
#else
#error "Error: TRIAXIS MLX9038x OUT2 not configured"
#endif

#if (_SUPPORT_NTC_IO == PIN_FUNC_IO_0)
#define C_ADC_NTC_IO_LV_EOC C_ADC_IO0_LV_EOC
#elif (_SUPPORT_NTC_IO == PIN_FUNC_IO_1)
#define C_ADC_NTC_IO_LV_EOC C_ADC_IO1_LV_EOC
#elif (_SUPPORT_NTC_IO == PIN_FUNC_IO_2)
#define C_ADC_NTC_IO_LV_EOC C_ADC_IO2_LV_EOC
#elif (_SUPPORT_NTC_IO == PIN_FUNC_IO_3)
#define C_ADC_NTC_IO_LV_EOC C_ADC_IO3_LV_EOC
#elif (_SUPPORT_NTC_IO == PIN_FUNC_IO_4)
#define C_ADC_NTC_IO_LV_EOC C_ADC_IO4_LV_EOC
#elif (_SUPPORT_NTC_IO == PIN_FUNC_IO_5)
#define C_ADC_NTC_IO_LV_EOC C_ADC_IO5_LV_EOC
#elif (_SUPPORT_NTC_IO == PIN_FUNC_IO_6)
#define C_ADC_NTC_IO_LV_EOC C_ADC_IO6_LV_EOC
#elif (_SUPPORT_NTC_IO == PIN_FUNC_IO_7)
#define C_ADC_NTC_IO_LV_EOC C_ADC_IO7_LV_EOC
#else
#error "Error: NTC I/O not configured"
#endif

/* Motor Current */
#define C_ADC_MCUR_EOC                                                          \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_MCUR,                                         \
            .u3AdcVref = C_ADC_VREF_MCUR,                                       \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }
#define C_ADC_MCUR_SLV1                                                         \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_MCUR,                                         \
            .u3AdcVref = C_ADC_VREF_MCUR,                                       \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV1_CMP,                          \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }
#define C_ADC_MCUR_SLV2                                                         \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_MCUR,                                         \
            .u3AdcVref = C_ADC_VREF_MCUR,                                       \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,                          \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }
#define C_ADC_MCUR_SLV3                                                         \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_MCUR,                                         \
            .u3AdcVref = C_ADC_VREF_MCUR,                                       \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV3_CMP,                          \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }
#define C_ADC_MCUR_MSTR2                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_MCUR,                                         \
            .u3AdcVref = C_ADC_VREF_MCUR,                                       \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR2_CMP,                         \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }
#define C_ADC_MCURF_EOC                                                         \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_MCURF,                                        \
            .u3AdcVref = C_ADC_VREF_MCUR,                                       \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

#define C_ADC_TEMP_EOC                                                          \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_TEMP,                                         \
            .u3AdcVref = C_ADC_VREF_2_50_V,                                     \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

#define C_ADC_VAUX_EOC                                                          \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_VAUX,                                         \
            .u3AdcVref = C_ADC_VREF_2_50_V,                                     \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

#define C_ADC_VBGD_EOC                                                          \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_VBGD,                                         \
            .u3AdcVref = C_ADC_VREF_2_50_V,                                     \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCpls16xADC_CLOCK,                \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

#define C_ADC_VBGD_EOC_1V5                                                      \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_VBGD,                                         \
            .u3AdcVref = C_ADC_VREF_1_50_V,                                     \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCpls16xADC_CLOCK,                \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

#define C_ADC_VDDA_EOC                                                          \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_VDDA,                                         \
            .u3AdcVref = C_ADC_VREF_2_50_V,                                     \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

#define C_ADC_VDDD_EOC                                                          \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_VDDD,                                         \
            .u3AdcVref = C_ADC_VREF_2_50_V,                                     \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

/* IC Supply */
#define C_ADC_VS_EOC                                                            \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_VS_HV,  /* Chip supply voltage (21:1) */      \
            .u3AdcVref = C_ADC_VREF_HV,                                         \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }
#define C_ADC_VS_MSTR1_CMP                                                      \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_VS_HV,                                        \
            .u3AdcVref = C_ADC_VREF_HV,                                         \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CMP,                         \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

/* Motor Driver Supply */
#define C_ADC_VSMF_EOC                                                          \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_VSMF_HV,                                      \
            .u3AdcVref = C_ADC_VREF_HV,                                         \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

/* Motor Driver Phase U Voltage */
#define C_ADC_VPHU_EOC                                                          \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_PH_U_HV,                                      \
            .u3AdcVref = C_ADC_VREF_HV,                                         \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

/* Motor Driver Phase V Voltage */
#define C_ADC_VPHV_EOC                                                          \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_PH_V_HV,                                      \
            .u3AdcVref = C_ADC_VREF_HV,                                         \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

/* Motor Driver Phase W Voltage */
#define C_ADC_VPHW_EOC                                                          \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_PH_W_HV,                                      \
            .u3AdcVref = C_ADC_VREF_HV,                                         \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

/* Motor Driver Phase T Voltage */
#define C_ADC_VPHT_EOC                                                          \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_PH_T_HV,                                      \
            .u3AdcVref = C_ADC_VREF_HV,                                         \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

/* LIN Voltage */
#define C_ADC_LIN_EOC                                                           \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_LIN,                                          \
            .u3AdcVref = C_ADC_VREF_2_50_V,                                     \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

#elif defined (__MLX81339__) || defined (__MLX81350__)

/* IO[0] HV */
#define C_ADC_IO0_HV_EOC                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_IO0_HV,                                       \
            .u3AdcReserved = 0U,                                                \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }
/* IO[0] LV */
#define C_ADC_IO0_LV_EOC                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_IO0_LV,                                       \
            .u3AdcReserved = 0U,                                                \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

/* IO[1] LV */
#define C_ADC_IO1_LV_EOC                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_IO1_LV,                                       \
            .u3AdcReserved = 0U,                                                \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

/* IO[2] LV */
#define C_ADC_IO2_LV_EOC                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_IO2_LV,                                       \
            .u3AdcReserved = 0U,                                                \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

/* IO[3] LV */
#define C_ADC_IO3_LV_EOC                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_IO3_LV,                                       \
            .u3AdcVref = C_ADC_VREF_2_50_V,                                     \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

#if !defined (__MLX81350__)
/* IO[4] LV */
#define C_ADC_IO4_LV_EOC                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_IO4_LV,                                       \
            .u3AdcReserved = 0U,                                                \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

/* IO[5] LV */
#define C_ADC_IO5_LV_EOC                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_IO5_LV,                                       \
            .u3AdcReserved = 0U,                                                \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

/* IO[6] LV */
#define C_ADC_IO6_LV_EOC                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_IO6_LV,                                       \
            .u3AdcReserved = 0U,                                                \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

/* IO[7] LV */
#define C_ADC_IO7_LV_EOC                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_IO7_LV,                                       \
            .u3AdcReserved = 0U,                                                \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }
#endif /* !define (__MLX81350__) */

#if (_SUPPORT_RESOLVER_X_IO == PIN_FUNC_IO_0)
#define C_ADC_RESOLVER_IOX_LV_EOC C_ADC_IO0_LV_EOC
#elif (_SUPPORT_RESOLVER_X_IO == PIN_FUNC_IO_1)
#define C_ADC_RESOLVER_IOX_LV_EOC C_ADC_IO1_LV_EOC
#elif (_SUPPORT_RESOLVER_X_IO == PIN_FUNC_IO_2)
#define C_ADC_RESOLVER_IOX_LV_EOC C_ADC_IO2_LV_EOC
#elif (_SUPPORT_RESOLVER_X_IO == PIN_FUNC_IO_3)
#define C_ADC_RESOLVER_IOX_LV_EOC C_ADC_IO3_LV_EOC
#elif (_SUPPORT_RESOLVER_X_IO == PIN_FUNC_IO_4)
#define C_ADC_RESOLVER_IOX_LV_EOC C_ADC_IO4_LV_EOC
#elif (_SUPPORT_RESOLVER_X_IO == PIN_FUNC_IO_5)
#define C_ADC_RESOLVER_IOX_LV_EOC C_ADC_IO5_LV_EOC
#elif (_SUPPORT_RESOLVER_X_IO == PIN_FUNC_IO_6)
#define C_ADC_RESOLVER_IOX_LV_EOC C_ADC_IO6_LV_EOC
#elif (_SUPPORT_RESOLVER_X_IO == PIN_FUNC_IO_7)
#define C_ADC_RESOLVER_IOX_LV_EOC C_ADC_IO7_LV_EOC
#else
#error "Error: TRIAXIS MLX9038x OUT1 not configured"
#endif

#if (_SUPPORT_RESOLVER_X_IO == PIN_FUNC_IO_0)
#define C_ADC_RESOLVER_IOY_LV_EOC C_ADC_IO0_LV_EOC
#elif (_SUPPORT_RESOLVER_Y_IO == PIN_FUNC_IO_1)
#define C_ADC_RESOLVER_IOY_LV_EOC C_ADC_IO1_LV_EOC
#elif (_SUPPORT_RESOLVER_Y_IO == PIN_FUNC_IO_2)
#define C_ADC_RESOLVER_IOY_LV_EOC C_ADC_IO2_LV_EOC
#elif (_SUPPORT_RESOLVER_Y_IO == PIN_FUNC_IO_3)
#define C_ADC_RESOLVER_IOY_LV_EOC C_ADC_IO3_LV_EOC
#elif (_SUPPORT_RESOLVER_Y_IO == PIN_FUNC_IO_4)
#define C_ADC_RESOLVER_IOY_LV_EOC C_ADC_IO4_LV_EOC
#elif (_SUPPORT_RESOLVER_Y_IO == PIN_FUNC_IO_5)
#define C_ADC_RESOLVER_IOY_LV_EOC C_ADC_IO5_LV_EOC
#elif (_SUPPORT_RESOLVER_Y_IO == PIN_FUNC_IO_6)
#define C_ADC_RESOLVER_IOY_LV_EOC C_ADC_IO6_LV_EOC
#elif (_SUPPORT_RESOLVER_Y_IO == PIN_FUNC_IO_7)
#define C_ADC_RESOLVER_IOY_LV_EOC C_ADC_IO7_LV_EOC
#else
#error "Error: TRIAXIS MLX9038x OUT2 not configured"
#endif

#if (_SUPPORT_NTC_IO == PIN_FUNC_IO_0)
#define C_ADC_NTC_IO_LV_EOC C_ADC_IO0_LV_EOC
#elif (_SUPPORT_NTC_IO == PIN_FUNC_IO_1)
#define C_ADC_NTC_IO_LV_EOC C_ADC_IO1_LV_EOC
#elif (_SUPPORT_NTC_IO == PIN_FUNC_IO_2)
#define C_ADC_NTC_IO_LV_EOC C_ADC_IO2_LV_EOC
#elif (_SUPPORT_NTC_IO == PIN_FUNC_IO_3)
#define C_ADC_NTC_IO_LV_EOC C_ADC_IO3_LV_EOC
#elif (_SUPPORT_NTC_IO == PIN_FUNC_IO_4)
#define C_ADC_NTC_IO_LV_EOC C_ADC_IO4_LV_EOC
#elif (_SUPPORT_NTC_IO == PIN_FUNC_IO_5)
#define C_ADC_NTC_IO_LV_EOC C_ADC_IO5_LV_EOC
#elif (_SUPPORT_NTC_IO == PIN_FUNC_IO_6)
#define C_ADC_NTC_IO_LV_EOC C_ADC_IO6_LV_EOC
#elif (_SUPPORT_NTC_IO == PIN_FUNC_IO_7)
#define C_ADC_NTC_IO_LV_EOC C_ADC_IO7_LV_EOC
#else
#error "Error: NTC I/O not configured"
#endif

/* Motor Current */
#define C_ADC_MCUR_EOC                                                          \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_MCUR,                                         \
            .u3AdcReserved = 0U,                                                \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }
#define C_ADC_MCUR_SLV1                                                         \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_MCUR,                                         \
            .u3AdcReserved = 0U,                                                \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV1_CMP,                          \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }
#define C_ADC_MCUR_SLV2                                                         \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_MCUR,                                         \
            .u3AdcReserved = 0U,                                                \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,                          \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }
#define C_ADC_MCUR_SLV3                                                         \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_MCUR,                                         \
            .u3AdcReserved = 0U,                                                \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV3_CMP,                          \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }
#define C_ADC_MCUR_MSTR2                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_MCUR,                                         \
            .u3AdcReserved = 0U,                                                \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR2_CMP,                         \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }
#define C_ADC_MCURF_EOC                                                         \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_MCURF,                                        \
            .u3AdcReserved = 0U,                                                \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

#define C_ADC_TEMP_EOC                                                          \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_TEMP,                                         \
            .u3AdcReserved = 0U,                                                \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

#define C_ADC_VAUX_EOC                                                          \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_VAUX,                                         \
            .u3AdcReserved = 0U,                                                \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

#define C_ADC_VBGD_EOC                                                          \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_VBGD,                                         \
            .u3AdcReserved = 0U,                                                \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCpls16xADC_CLOCK,                \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

#define C_ADC_VDDA_EOC                                                          \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_VDDA,                                         \
            .u3AdcReserved = 0U,                                                \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

#define C_ADC_VDDD_EOC                                                          \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_VDDD,                                         \
            .u3AdcReserved = 0U,                                                \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

/* IC Supply */
#define C_ADC_VS_EOC                                                            \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_VS_HV,  /* Chip supply voltage (21:1) */      \
            .u3AdcReserved = 0U,                                                \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }
#define C_ADC_VS_MSTR1_CMP                                                      \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_VS_HV,                                        \
            .u3AdcReserved = 0U,                                                \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CMP,                         \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

/* Motor Driver Supply */
#define C_ADC_VSMF_EOC                                                          \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_VSMF_HV,                                      \
            .u3AdcReserved = 0U,                                                \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

/* Motor Driver Phase U Voltage */
#define C_ADC_VPHU_EOC                                                          \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_PH_U_HV,                                      \
            .u3AdcReserved = 0U,                                                \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

/* Motor Driver Phase V Voltage */
#define C_ADC_VPHV_EOC                                                          \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_PH_V_HV,                                      \
            .u3AdcReserved = 0U,                                                \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

/* Motor Driver Phase W Voltage */
#define C_ADC_VPHW_EOC                                                          \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_PH_W_HV,                                      \
            .u3AdcReserved = 0U,                                                \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

/* Motor Driver Phase T Voltage */
#define C_ADC_VPHT_EOC                                                          \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_PH_T_HV,                                      \
            .u3AdcReserved = 0U,                                                \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }

#if !defined (__MLX81339__)
/* LIN Voltage */
#define C_ADC_LIN_EOC                                                           \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u5AdcChannel = C_ADC_LIN,                                          \
            .u3AdcReserved = 0U,                                                \
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,                 \
            .u1AdcReserved = 0U                                                 \
        }                                                                       \
    }
#endif /* !defined (__MLX81339__) */

#elif defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
/* IO[0] HV */
#define C_ADC_IO0_HV_EOC                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_IO0_HV,                                       \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK                  \
        }                                                                       \
    }

/* IO[0] LV */
#define C_ADC_IO0_LV_EOC                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_IO0_LV,                                       \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK                  \
        }                                                                       \
    }

/* IO[1] HV */
#define C_ADC_IO1_HV_EOC                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_IO1_HV,                                       \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK                  \
        }                                                                       \
    }

/* IO[1] LV */
#define C_ADC_IO1_LV_EOC                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_IO1_LV,                                       \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK                  \
        }                                                                       \
    }

/* IO[2] HV */
#define C_ADC_IO2_HV_EOC                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_IO2_HV,                                       \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK                  \
        }                                                                       \
    }

/* IO[2] LV */
#define C_ADC_IO2_LV_EOC                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_IO2_LV,                                       \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK                  \
        }                                                                       \
    }

/* IO[3] LV */
#define C_ADC_IO3_LV_EOC                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_IO3_LV,                                       \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK                  \
        }                                                                       \
    }

/* IO[4] LV */
#define C_ADC_IO4_LV_EOC                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_IO4_LV,                                       \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK                  \
        }                                                                       \
    }

/* IO[5] LV */
#define C_ADC_IO5_LV_EOC                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_IO5_LV,                                       \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK                  \
        }                                                                       \
    }

/* IO[6] LV */
#define C_ADC_IO6_LV_EOC                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_IO6_LV,                                       \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK                  \
        }                                                                       \
    }

/* IO[7] LV */
#define C_ADC_IO7_LV_EOC                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_IO7_LV,                                       \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK                  \
        }                                                                       \
    }

/* IO[8] LV */
#define C_ADC_IO8_LV_EOC                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_IO8_LV,                                       \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK                  \
        }                                                                       \
    }

/* IO[9] LV */
#define C_ADC_IO9_LV_EOC                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_IO9_LV,                                       \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK                  \
        }                                                                       \
    }

/* IO[10] LV */
#define C_ADC_IO10_LV_EOC                                                       \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_IO10_LV,                                      \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK                  \
        }                                                                       \
    }

/* IO[11] LV */
#define C_ADC_IO11_LV_EOC                                                       \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_IO11_LV,                                      \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK                  \
        }                                                                       \
    }

#if (_SUPPORT_POTI_IO == PIN_FUNC_IO_0)
#define C_ADC_POTI_IOX_EOC C_ADC_IO0_LV_EOC
#elif (_SUPPORT_POTI_IO == PIN_FUNC_IO_1)
#define C_ADC_POTI_IOX_EOC C_ADC_IO1_LV_EOC
#elif (_SUPPORT_POTI_IO == PIN_FUNC_IO_2)
#define C_ADC_POTI_IOX_EOC C_ADC_IO2_LV_EOC
#elif (_SUPPORT_POTI_IO == PIN_FUNC_IO_3)
#define C_ADC_POTI_IOX_EOC C_ADC_IO3_LV_EOC
#elif defined (PIN_FUNC_IO_4) && (_SUPPORT_POTI_IO == PIN_FUNC_IO_4)
#define C_ADC_POTI_IOX_EOC C_ADC_IO4_LV_EOC
#elif (_SUPPORT_POTI_IO == PIN_FUNC_IO_O_HV)
#define C_ADC_POTI_IOX_EOC C_ADC_IO0_HV_EOC
#else
#error "Error: POTI ADC I/O not configured"
#endif

#if (_SUPPORT_RESOLVER_X_IO == PIN_FUNC_IO_0)
#define C_ADC_RESOLVER_IOX_LV_EOC C_ADC_IO0_LV_EOC
#elif (_SUPPORT_RESOLVER_X_IO == PIN_FUNC_IO_1)
#define C_ADC_RESOLVER_IOX_LV_EOC C_ADC_IO1_LV_EOC
#elif (_SUPPORT_RESOLVER_X_IO == PIN_FUNC_IO_2)
#define C_ADC_RESOLVER_IOX_LV_EOC C_ADC_IO2_LV_EOC
#elif (_SUPPORT_RESOLVER_X_IO == PIN_FUNC_IO_3)
#define C_ADC_RESOLVER_IOX_LV_EOC C_ADC_IO3_LV_EOC
#elif (_SUPPORT_RESOLVER_X_IO == PIN_FUNC_IO_4)
#define C_ADC_RESOLVER_IOX_LV_EOC C_ADC_IO4_LV_EOC
#elif (_SUPPORT_RESOLVER_X_IO == PIN_FUNC_IO_5)
#define C_ADC_RESOLVER_IOX_LV_EOC C_ADC_IO5_LV_EOC
#elif (_SUPPORT_RESOLVER_X_IO == PIN_FUNC_IO_6)
#define C_ADC_RESOLVER_IOX_LV_EOC C_ADC_IO6_LV_EOC
#elif (_SUPPORT_RESOLVER_X_IO == PIN_FUNC_IO_7)
#define C_ADC_RESOLVER_IOX_LV_EOC C_ADC_IO7_LV_EOC
#elif (_SUPPORT_RESOLVER_X_IO == PIN_FUNC_IO_8)
#define C_ADC_RESOLVER_IOX_LV_EOC C_ADC_IO8_LV_EOC
#elif (_SUPPORT_RESOLVER_X_IO == PIN_FUNC_IO_9)
#define C_ADC_RESOLVER_IOX_LV_EOC C_ADC_IO9_LV_EOC
#elif (_SUPPORT_RESOLVER_X_IO == PIN_FUNC_IO_10)
#define C_ADC_RESOLVER_IOX_LV_EOC C_ADC_IO10_LV_EOC
#elif (_SUPPORT_RESOLVER_X_IO == PIN_FUNC_IO_11)
#define C_ADC_RESOLVER_IOX_LV_EOC C_ADC_IO11_LV_EOC
#else
#error "Error: TRIAXIS MLX9038x OUT1 not configured"
#endif

#if (_SUPPORT_RESOLVER_X_IO == PIN_FUNC_IO_0)
#define C_ADC_RESOLVER_IOY_LV_EOC C_ADC_IO0_LV_EOC
#elif (_SUPPORT_RESOLVER_Y_IO == PIN_FUNC_IO_1)
#define C_ADC_RESOLVER_IOY_LV_EOC C_ADC_IO1_LV_EOC
#elif (_SUPPORT_RESOLVER_Y_IO == PIN_FUNC_IO_2)
#define C_ADC_RESOLVER_IOY_LV_EOC C_ADC_IO2_LV_EOC
#elif (_SUPPORT_RESOLVER_Y_IO == PIN_FUNC_IO_3)
#define C_ADC_RESOLVER_IOY_LV_EOC C_ADC_IO3_LV_EOC
#elif (_SUPPORT_RESOLVER_Y_IO == PIN_FUNC_IO_4)
#define C_ADC_RESOLVER_IOY_LV_EOC C_ADC_IO4_LV_EOC
#elif (_SUPPORT_RESOLVER_Y_IO == PIN_FUNC_IO_5)
#define C_ADC_RESOLVER_IOY_LV_EOC C_ADC_IO5_LV_EOC
#elif (_SUPPORT_RESOLVER_Y_IO == PIN_FUNC_IO_6)
#define C_ADC_RESOLVER_IOY_LV_EOC C_ADC_IO6_LV_EOC
#elif (_SUPPORT_RESOLVER_Y_IO == PIN_FUNC_IO_7)
#define C_ADC_RESOLVER_IOY_LV_EOC C_ADC_IO7_LV_EOC
#elif (_SUPPORT_RESOLVER_Y_IO == PIN_FUNC_IO_8)
#define C_ADC_RESOLVER_IOY_LV_EOC C_ADC_IO8_LV_EOC
#elif (_SUPPORT_RESOLVER_Y_IO == PIN_FUNC_IO_9)
#define C_ADC_RESOLVER_IOY_LV_EOC C_ADC_IO9_LV_EOC
#elif (_SUPPORT_RESOLVER_Y_IO == PIN_FUNC_IO_10)
#define C_ADC_RESOLVER_IOY_LV_EOC C_ADC_IO10_LV_EOC
#elif (_SUPPORT_RESOLVER_Y_IO == PIN_FUNC_IO_11)
#define C_ADC_RESOLVER_IOY_LV_EOC C_ADC_IO11_LV_EOC
#else
#error "Error: TRIAXIS MLX9038x OUT2 not configured"
#endif

#if (_SUPPORT_NTC_IO == PIN_FUNC_IO_0)
#define C_ADC_NTC_IO_LV_EOC C_ADC_IO0_LV_EOC
#elif (_SUPPORT_NTC_IO == PIN_FUNC_IO_1)
#define C_ADC_NTC_IO_LV_EOC C_ADC_IO1_LV_EOC
#elif (_SUPPORT_NTC_IO == PIN_FUNC_IO_2)
#define C_ADC_NTC_IO_LV_EOC C_ADC_IO2_LV_EOC
#elif (_SUPPORT_NTC_IO == PIN_FUNC_IO_3)
#define C_ADC_NTC_IO_LV_EOC C_ADC_IO3_LV_EOC
#elif (_SUPPORT_NTC_IO == PIN_FUNC_IO_4)
#define C_ADC_NTC_IO_LV_EOC C_ADC_IO4_LV_EOC
#elif (_SUPPORT_NTC_IO == PIN_FUNC_IO_5)
#define C_ADC_NTC_IO_LV_EOC C_ADC_IO5_LV_EOC
#elif (_SUPPORT_NTC_IO == PIN_FUNC_IO_6)
#define C_ADC_NTC_IO_LV_EOC C_ADC_IO6_LV_EOC
#elif (_SUPPORT_NTC_IO == PIN_FUNC_IO_7)
#define C_ADC_NTC_IO_LV_EOC C_ADC_IO7_LV_EOC
#else
#error "Error: NTC I/O not configured"
#endif

/* Motor Current */
#if defined (__MLX81160__)
#define C_ADC_MCUR1_EOC                                                         \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_MCUR1,                                        \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK                  \
        }                                                                       \
    }
#define C_ADC_MCUR1_SLV1                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_MCUR1,                                        \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_SLV1_CMP                           \
        }                                                                       \
    }
#define C_ADC_MCUR1_SLV2                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_MCUR1,                                        \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP                           \
        }                                                                       \
    }
#define C_ADC_MCUR1_SLV3                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_MCUR1,                                        \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_SLV3_CMP                           \
        }                                                                       \
    }
#define C_ADC_MCUR1_MSTR2                                                       \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_MCUR1,                                        \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_MSTR2_CMP                          \
        }                                                                       \
    }
#define C_ADC_MCUR2_EOC                                                         \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_MCUR2,                                        \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK                  \
        }                                                                       \
    }
#define C_ADC_MCUR2_SLV1                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_MCUR2,                                        \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_SLV1_CMP                           \
        }                                                                       \
    }
#define C_ADC_MCUR2_SLV2                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_MCUR2,                                        \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP                           \
        }                                                                       \
    }
#define C_ADC_MCUR2_SLV3                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_MCUR2,                                        \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_SLV3_CMP                           \
        }                                                                       \
    }
#define C_ADC_MCUR2_MSTR2                                                       \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_MCUR2,                                        \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_MSTR2_CMP                          \
        }                                                                       \
    }
#else  /* defined (__MLX81160__) */
#define C_ADC_MCUR_EOC                                                          \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_MCUR,                                         \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK                  \
        }                                                                       \
    }
#define C_ADC_MCUR_SLV1                                                         \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_MCUR,                                         \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_SLV1_CMP                           \
        }                                                                       \
    }
#define C_ADC_MCUR_SLV2                                                         \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_MCUR,                                         \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP                           \
        }                                                                       \
    }
#define C_ADC_MCUR_SLV3                                                         \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_MCUR,                                         \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_SLV3_CMP                           \
        }                                                                       \
    }
#define C_ADC_MCUR_MSTR1                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_MCUR,                                         \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT                          \
        }                                                                       \
    }
#define C_ADC_MCUR_MSTR2                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_MCUR,                                         \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_MSTR2_CMP                          \
        }                                                                       \
    }
#endif /* defined (__MLX81160__) */

#define C_ADC_TEMP_EOC                                                          \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_TEMP,                                         \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK                  \
        }                                                                       \
    }

#define C_ADC_VAUX_EOC                                                          \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_VAUX,                                         \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK                  \
        }                                                                       \
    }

#define C_ADC_VBGD_EOC                                                          \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_VBGD,                                         \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_EOCpls15xADC_CLOCK                 \
        }                                                                       \
    }

#if defined (__MLX81346__) && _SUPPORT_APP_48V_ADC
#define C_ADC_VBOOST_EOC                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_VBOOST_DIV64,                                 \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_EOCpls15xADC_CLOCK                 \
        }                                                                       \
    }
#else  /* defined (__MLX81346__) && _SUPPORT_APP_48V_ADC */
#define C_ADC_VBOOST_EOC                                                        \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_VBOOST_HV,                                    \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK                  \
        }                                                                       \
    }
#endif /* defined (__MLX81346__) && _SUPPORT_APP_48V_ADC */

#define C_ADC_VDDA_EOC                                                          \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_VDDA,                                         \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_EOCpls15xADC_CLOCK                 \
        }                                                                       \
    }

#define C_ADC_VDDD_EOC                                                          \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_VDDD,                                         \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK                  \
        }                                                                       \
    }

/* IC Supply */
#define C_ADC_VS_EOC                                                            \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_VS_HV,                                        \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK                  \
        }                                                                       \
    }
#define C_ADC_VS_MSTR1_CMP                                                      \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_VS_HV,                                        \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CMP                          \
        }                                                                       \
    }

/* Motor Driver Supply */
#define C_ADC_VSMF_EOC                                                          \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_VSMF_HV,                                      \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK                  \
        }                                                                       \
    }

#if defined (__MLX81160__)
/* Motor Driver Phase R Voltage */
#define C_ADC_VPHR_EOC                                                          \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_PH_R_HV,                                      \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK                  \
        }                                                                       \
    }

/* Motor Driver Phase S Voltage */
#define C_ADC_VPHS_EOC                                                          \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_PH_S_HV,                                      \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK                  \
        }                                                                       \
    }

/* Motor Driver Phase T Voltage */
#define C_ADC_VPHT_EOC                                                          \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_PH_T_HV,                                      \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK                  \
        }                                                                       \
    }
#endif /* defined (__MLX81160__) */

/* Motor Driver Phase U Voltage */
#define C_ADC_VPHU_EOC                                                          \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_PH_U_HV,                                      \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK                  \
        }                                                                       \
    }

/* Motor Driver Phase V Voltage */
#define C_ADC_VPHV_EOC                                                          \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_PH_V_HV,                                      \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK                  \
        }                                                                       \
    }

/* Motor Driver Phase W Voltage */
#define C_ADC_VPHW_EOC                                                          \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_PH_W_HV,                                      \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK                  \
        }                                                                       \
    }

#if defined (__MLX81160__)
#define C_ADC_AGND_EOC                                                          \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_AGND,                                         \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK                  \
        }                                                                       \
    }
#else  /* defined (__MLX81160__) */
#define C_ADC_AGND_EOC                                                          \
    {                                                                           \
        {                                                                       \
            .u2AdcMarker = C_ADC_NO_SIGN,                                       \
            .u1AdcSin = C_ADC_SIN_SDATA,                                        \
            .u6AdcChannel = C_ADC_TA0_LV,                                       \
            .u1AdcType = C_ADC_TYPE_CYCLIC,                                     \
            .u6AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK                  \
        }                                                                       \
    }
#endif /* defined (__MLX81160__) */
#endif

#define C_ADC_MIN           0x000U                                              /*!< Minimum-value ADC */
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
#define C_ADC_MAX           0x3FFU                                              /*!< Maximum-value ADC; 10-bit ADC */
#elif defined (__MLX81339__) || defined (__MLX81350__)
#define C_ADC_MAX           0x0FFFU                                             /*!< Maximum-value ADC; 12-bit (unsigned) ADC */
#elif defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#define C_ADC_MAX           0x0FFFU                                             /*!< Maximum-value ADC; 12-bit (signed) ADC */
#endif
#define C_ADC_MID           ((C_ADC_MAX - C_ADC_MIN) / 2U)                      /*!< Mid-value ADC */

#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
#define C_ADC_HV_DIV        21U                                                 /*!< HV channel divider */
#elif defined (__MLX81339__)
#define C_ADC_HV_DIV        18U                                                 /*!< HV channel divider */
#elif defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#if defined (__MLX81346__) && _SUPPORT_APP_48V_ADC
#define C_ADC_HV_DIV        52U                                                 /*!< HV channel divider */
#else  /* defined (__MLX81346__) && _SUPPORT_APP_48V_ADC */
#define C_ADC_HV_DIV        26U                                                 /*!< HV channel divider */
#endif /* defined (__MLX81346__) && _SUPPORT_APP_48V_ADC */
#elif defined (__MLX81350__)
#define C_ADC_HV_DIV        20U                                                 /*!< HV channel divider */
#endif

#endif /* HAL_LIB_ADC_CHANNELS_H */

/* EOF */
