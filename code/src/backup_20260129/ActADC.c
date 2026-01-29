/*!*************************************************************************** *
 * \file        ActADC.c
 * \brief       MLX8133x ADC handling
 *
 * \note        project MLX8133x
 *
 * \author      Marcel Braat
 *
 * \date        2017-08-14
 *
 * \version     2.0
 *
 * A list of functions:
 *  - Global Functions:
 *           -# ADC_Start()
 *           -# ADC_ISR()
 *           -# CalcMotorCurrentAngleAndPeak()
 *           -# ADC_GetRawMotorDriverCurrent()
 *           -# ADC_MeasureVsupplyAndTemperature()
 *           -# ADC_GetNewSampleVsupply();
 *           -# ADC_MeasureMotorCurrent()
 *           -# MeasureResolverPos()
 *           -# GetResolverPosition()
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

/*!*************************************************************************** */
/*                           INCLUDES                                          */
/* *************************************************************************** */
#include "AppBuild.h"                                                           /* Application Build */

#include "ActADC.h"                                                             /* Application ADC support */

#include "drivelib/ErrorCodes.h"                                                /* Error-logging support */
#include "drivelib/NV_UserPage.h"                                               /* Non Volatile Memory User Application Page Layout */
#if (_APP_BURN_IN != FALSE) || (_APP_DMA_STRESS_TEST != FALSE) || (_DEBUG_RESOLVER != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || (_SUPPORT_STALLDET_BZC != FALSE)
#include "drivelib/GlobalVars.h"                                                /* Global Variables support */
#endif /* (_APP_BURN_IN != FALSE) || (_APP_DMA_STRESS_TEST != FALSE) || (_DEBUG_RESOLVER != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || (_SUPPORT_STALLDET_BZC != FALSE) */
#include "drivelib/MotorDriver.h"                                               /* Motor driver support */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) || (_SUPPORT_STALLDET_LA != FALSE) || ((_SUPPORT_PWM_MODE != BIPOLAR_PWM_SINGLE_INDEPENDENT_GND) && (_SUPPORT_PWM_MODE != BIPOLAR_FULL_STEP_BEMF) && (_SUPPORT_PWM_MODE != BIPOLAR_HALF_STEP_BEMF) && (_SUPPORT_APP_TYPE != C_APP_SOLENOID) && (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM))
#include "drivelib/MotorDriverTables.h"                                         /* Wave-form vector tables */
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) || (_SUPPORT_STALLDET_LA != FALSE) || ((_SUPPORT_PWM_MODE != BIPOLAR_PWM_SINGLE_INDEPENDENT_GND) && (_SUPPORT_PWM_MODE != BIPOLAR_FULL_STEP_BEMF) && (_SUPPORT_PWM_MODE != BIPOLAR_HALF_STEP_BEMF) && (_SUPPORT_APP_TYPE != C_APP_SOLENOID) && (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM)) */
#include "camculib/private_mathlib.h"                                           /* Private Math Library support */

#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
#include "senselib/Triaxis_MLX9038x.h"                                          /* Triaxis MLX90380 support */
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */

#include <sys_tools.h>

/*!*************************************************************************** */
/*                           DEFINITIONS                                       */
/* *************************************************************************** */
#if (_SUPPORT_STALLDET_BZC != FALSE)
#define C_MIN_HCOMMUT_TIME         75                                           /*!< Minimum Half-commutation time */
#define C_MAX_HCOMMUT_TIME      16383                                           /*!< Maximum Half-commutation time */

#if (_SUPPORT_ADC_REF_HV_CALIB == FALSE)
#define C_1VOLT                 19U                                             /*!< 1.00V --> 1.00V/21 = 0.0476V/2.5Vref * 1023 = 19.49 */
#define C_100MV                 2U                                              /*!< 100mV --> 2 ADC-LSB's */
#else  /* (_SUPPORT_ADC_REF_HV_CALIB == FALSE) */
#define C_1VOLT                 32U                                             /*!< 1.00V --> 1.00V/21 = 0.0476V/1.5Vref * 1023 = 32.48 */
#define C_100MV                 3U                                              /*!< 100mV --> 3 ADC-LSB's */
#endif /* (_SUPPORT_ADC_REF_HV_CALIB == FALSE) */
#endif /* (_SUPPORT_STALLDET_BZC != FALSE) */

/*!*************************************************************************** */
/*                           GLOBAL & LOCAL VARIABLES                          */
/* *************************************************************************** */
#pragma space dp                                                                /* __TINY_SECTION__ */
#if (C_MOTOR_PHASES != 1)
int16_t g_i16MotorCurrentCoilA = 0;                                             /*!< Phase current 'A' without offset (g_i16MotorCurrentCoilX) */
#if (_SUPPORT_PWM_MODE != BIPOLAR_FULL_STEP_BEMF) && (_SUPPORT_PWM_MODE != BIPOLAR_HALF_STEP_BEMF)
int16_t g_i16MotorCurrentCoilB = 0;                                             /*!< Phase current 'B' without offset */
#if (C_MOTOR_PHASES == 3)
int16_t g_i16MotorCurrentCoilC = 0;                                             /*!< Phase current 'C' without offset */
int16_t g_i16MotorCurrentCoilY = 0;                                             /*!< Phase current 'Y' (combination of 'B' + 'C') */
#endif /* (C_MOTOR_PHASES == 3) */
#endif /* (_SUPPORT_PWM_MODE != BIPOLAR_FULL_STEP_BEMF) && (_SUPPORT_PWM_MODE != BIPOLAR_HALF_STEP_BEMF) */
#endif /* (C_MOTOR_PHASES != 1) */
uint16_t g_u16MotorCurrentPeak = 0U;                                            /*!< Motor Current Peak [ADC-LSB] (MMP230803-1) */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) || (_SUPPORT_STALLDET_LA != FALSE)
int16_t g_i16MotorCurrentAngle = 0;                                             /*!< Motor Current Angle [0..65535 = 0..2pi] (MMP230803-1) */
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) || (_SUPPORT_STALLDET_LA != FALSE) */
#if (_SUPPORT_STALLDET_BRI != FALSE)
uint32_t l_u32MotorBemfVoltage_Integration = 0U;                                /* Motor BEMF Voltage (integration) */
uint16_t l_u16MotorBemf_IntegrationCount = 0U;                                  /* Motor BEMF Voltage integration-counter */
uint16_t l_u16MotorBemf_IgnoreCount = 0U;                                       /* Motor BEMF Sense Ignore period (PWM-periods) */
uint16_t g_u16MotorBemfVoltage = 0U;                                            /* Motor BEMF Voltage [ADC-LSB] */
#endif /* _(_SUPPORT_STALLDET_BRI != FALSE) */
#if (ANA_COMM != FALSE)
uint16_t g_u16ReferencePotiPos;                                                 /*!< Actual Reference position */
#endif /* (ANA_COMM != FALSE) */
#pragma space none                                                              /* __TINY_SECTION__ */

#pragma space nodp                                                              /* __NEAR_SECTION__ */
volatile ADC_RESULTS l_AdcResult;                                               /*!< Motor-driver active */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_VARIABLE_PWM != FALSE)
volatile ADC_RESULTS_3 l_AdcResult3;                                            /*!< ADC results Stepper mode (3-phase current) (MMP220815-1) */
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_VARIABLE_PWM != FALSE) */
#if (_SUPPORT_STALLDET_BZC != FALSE)
volatile ADC_MOTORRUN_BEMF l_AdcMotorRunBemf;                                   /*!< ADC results BEMF mode */
uint16_t l_u16FullStepIndex = 0U;                                               /*!< Motor full-step index */
uint16_t l_u16ZeroCrossVoltage = 0U;                                            /*!< Zero-crossing voltage level */
uint16_t l_u16ZcTime = 0U;                                                      /*!< Timer between two Zero-crossings */
#endif /* (_SUPPORT_STALLDET_BZC != FALSE) */
#if (ANA_COMM != FALSE)
static uint16_t l_au16PosRefSamples[C_POS_MOVAVG_SZ] = {0U};                    /*!< Moving-Average filter Reference (Potentiometer) Position samples */
static uint16_t l_u16PosRefMovAgvIdx = 0U;                                      /*!< Reference Position Moving-Average filter index */
static uint16_t l_u16PosRefMovAvgxN = 0U;                                       /*!< Reference Position Moving-Average Sum */
/*static uint16_t l_u16InvRefPos = 0U;*/                                        /* MMP170509-2 */
#endif /* (ANA_COMM != FALSE) */
#if (_SUPPORT_POTI != FALSE)
static uint16_t l_au16PosActSamples[C_POS_MOVAVG_SZ] = {0};                     /*!< Moving-Average filter Actuator Potentiometer Position samples */
static uint16_t l_u16PosActMovAgvIdx = 0U;                                      /*!< Actuator Position Moving-Average filter index */
static uint16_t l_u16PosActMovAvgxN = 0U;                                       /*!< Actuator Position Moving-Average Sum */
#endif /* (_SUPPORT_POTI != FALSE) */
#pragma space none                                                              /* __NEAR_SECTION__ */

#if (_SUPPORT_STALLDET_BRI != FALSE)
extern int16_t GetRawMotorBemfVoltage(void);
#endif /* (_SUPPORT_STALLDET_BRI != FALSE) */

/*!*************************************************************************** *
 *                           ADC TABLES                                        *
 * *************************************************************************** */
/*! ADC Table for active/running actuator */
static ADC_SDATA_t const SBASE_MOTOR_RUN[] = /*lint !e578 */                       /* ADC Automated measurements */
{
#if (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
    /* Solenoid
     *              MASTER1      SLAVE1       SLAVE2       SLAVE3       MASTER2
     * Triggers at: 1/6 (16.7%)  2/6 (33.3%)  3/6 (50.0%)  3/4 (75.0%)  -
     * Measurement: VSMF         Temperature  M-Current    VS           -
     * Solenoid Current must be add 50% when using Mirror PWM.
     * Max. 8 ADC channels per PWM-period of 20kHz (6.25us/ADC-channel @ 8MHz ADC-Clock)
     * (See also: SolenoidDriver.c)
     */
    C_ADC_VS_MSTR1_CMP,
    C_ADC_VDDA_EOC,
    C_ADC_VDDD_EOC,
    C_ADC_MCUR_SLV2,
    C_ADC_VSMF_EOC,
    C_ADC_TEMP_EOC,
#else  /* (_SUPPORT_APP_TYPE == C_APP_SOLENOID) */

#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM)
    /* DC-Motor
     *                  MASTER1       SLAVE1       SLAVE2    SLAVE3                 MASTER2
     * Triggers at    : 15.0%         30.0%        45.0%     55.0%                  80.0%
     * Measurement (1): VS,VDDA,VDDD  -            M-CurrF   M-CurrF,VSMF,Tj,IO3    -       (IO3 = Potentiometer)
     *             (2): VS            -            M-CurrF   M-CurrF,VSMF,Tj,IO0HV  -       (IO0 = Analogue input)
     *             (3): VS,IO[2:0]    -            M-CurrF   M-CurrF,VSMF,Tj        -       (IO[2:0] = Command/position input)
     * DC-Motor Current must be add 50% when using Mirror PWM (current-ripple/spike).
     * Max. 8 ADC channels per PWM-period of 20kHz (6.25us/ADC-channel @ 8MHz ADC-Clock)
     * (See also: MotorDriver.c)
     */
    C_ADC_VS_MSTR1_CMP,
    C_ADC_VDDA_EOC,
    C_ADC_VDDD_EOC,
    {
        {
            .u2AdcMarker = C_ADC_NO_SIGN,
            .u5AdcChannel = C_ADC_MCURF,
            .u3AdcVref = C_ADC_VREF_MCUR,
            .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
            .u1AdcReserved = 0U
        }
    },
    {
        {
            .u2AdcMarker = C_ADC_NO_SIGN,
            .u5AdcChannel = C_ADC_MCURF,
            .u3AdcVref = C_ADC_VREF_MCUR,
            .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV3_CMP,
            .u1AdcReserved = 0U
        }
    },
    C_ADC_VSMF_EOC,
    C_ADC_TEMP_EOC,
#if (_SUPPORT_POTI != FALSE)
    C_ADC_POTI_IOX_EOC,
#elif (_SUPPORT_IO_DUT_SELECT_HVIO != FALSE) && (HVIO_DUT_SELECT == PIN_FUNC_IO_0)
    C_ADC_IO0_HV_EOC,
#endif /* (_SUPPORT_POTI != FALSE) */
    C_ADC_VS_EOC,
#if (_SUPPORT_IO_CMD_SELECT_LVIO != FALSE)
#if (HVIO_DUT_SELECT_A == PIN_FUNC_IO_0)
    C_ADC_IO0_LV_EOC,
#elif (HVIO_DUT_SELECT_A == PIN_FUNC_IO_1)
    C_ADC_IO1_LV_EOC,
#elif (HVIO_DUT_SELECT_A == PIN_FUNC_IO_2)
    C_ADC_IO2_LV_EOC,
#elif (HVIO_DUT_SELECT_A == PIN_FUNC_IO_3)
    C_ADC_IO3_LV_EOC,
#elif (HVIO_DUT_SELECT_A == PIN_FUNC_IO_4)
    C_ADC_IO4_LV_EOC,
#elif (HVIO_DUT_SELECT_A == PIN_FUNC_IO_5)
    C_ADC_IO5_LV_EOC,
#elif (HVIO_DUT_SELECT_A == PIN_FUNC_IO_6)
    C_ADC_IO6_LV_EOC,
#elif (HVIO_DUT_SELECT_A == PIN_FUNC_IO_7)
    C_ADC_IO7_LV_EOC,
#endif /* (HVIO_DUT_SELECT_A) */
#if (_SUPPORT_NR_OF_IO_SELECT >= 2U)
#if (HVIO_DUT_SELECT_B == PIN_FUNC_IO_1)
    C_ADC_IO1_LV_EOC,
#elif (HVIO_DUT_SELECT_B == PIN_FUNC_IO_2)
    C_ADC_IO2_LV_EOC,
#elif (HVIO_DUT_SELECT_B == PIN_FUNC_IO_3)
    C_ADC_IO3_LV_EOC,
#elif (HVIO_DUT_SELECT_B == PIN_FUNC_IO_4)
    C_ADC_IO4_LV_EOC,
#elif (HVIO_DUT_SELECT_B == PIN_FUNC_IO_5)
    C_ADC_IO5_LV_EOC,
#elif (HVIO_DUT_SELECT_B == PIN_FUNC_IO_6)
    C_ADC_IO6_LV_EOC,
#elif (HVIO_DUT_SELECT_B == PIN_FUNC_IO_7)
    C_ADC_IO7_LV_EOC,
#endif /* (HVIO_DUT_SELECT_B) */
#if (_SUPPORT_NR_OF_IO_SELECT >= 3U)
#if (HVIO_DUT_SELECT_C == PIN_FUNC_IO_2)
    C_ADC_IO2_LV_EOC,
#elif (HVIO_DUT_SELECT_C == PIN_FUNC_IO_3)
    C_ADC_IO3_LV_EOC,
#elif (HVIO_DUT_SELECT_C == PIN_FUNC_IO_4)
    C_ADC_IO4_LV_EOC,
#elif (HVIO_DUT_SELECT_C == PIN_FUNC_IO_5)
    C_ADC_IO5_LV_EOC,
#elif (HVIO_DUT_SELECT_C == PIN_FUNC_IO_6)
    C_ADC_IO6_LV_EOC,
#elif (HVIO_DUT_SELECT_C == PIN_FUNC_IO_7)
    C_ADC_IO7_LV_EOC,
#endif /* (HVIO_DUT_SELECT_C) */
#endif /* (_SUPPORT_NR_OF_IO_SELECT >= 3U) */
#endif /* (_SUPPORT_NR_OF_IO_SELECT >= 2U) */
#endif /* (_SUPPORT_IO_CMD_SELECT_LVIO != FALSE) */
    {
        {
            .u2AdcMarker = C_ADC_NO_SIGN,
            .u5AdcChannel = C_ADC_MCURF,
            .u3AdcVref = C_ADC_VREF_MCUR,
            .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
            .u1AdcReserved = 0U
        }
    },
    {
        {
            .u2AdcMarker = C_ADC_NO_SIGN,
            .u5AdcChannel = C_ADC_MCURF,
            .u3AdcVref = C_ADC_VREF_MCUR,
            .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV3_CMP,
            .u1AdcReserved = 0U
        }
    },
    C_ADC_VSMF_EOC,
    C_ADC_TEMP_EOC
#if (ANA_COMM != FALSE) && (C_ANA_IO == PIN_FUNC_IO_0)
    C_ADC_IO0_HV_EOC,
#endif /* (ANA_COMM != FALSE) && (C_ANA_IO == PIN_FUNC_IO_0) */

#elif (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR)
    /* Single Coil Fan/Pump
     *               MASTER1       SLAVE1           SLAVE2                      SLAVE3
     * Triggers at : 1/8 (12.5%)   2/6+D (33.3%+D)  3/6+D (50.0%+D)             4/6+D (66.7%+D)
     * Measurement : VS,VDDA,VDDD  -                M-Curr,VSMF,Tj              -
     * With VBGD:
     * Measurement : VS,VSMF,Tj    -                M-Curr,VDDA,VDDD,VAUX,VBGD  -
     * Note 1: Single Coil Current must be add 50% when using Mirror PWM
     * (See also: MotorDriver.c)
     * Note 2: VBGD must be measured after VAUX with max ADC delay of 4.0us for the best result.
     * Max. 10 ADC channels per PWM-period of 20kHz (4.5us/ADC-channel @ 4MHz ADC-Clock)
     */
#if (_SUPPORT_ADC_BGD != FALSE)                                                 /* MMP220307-1 */
    C_ADC_VS_MSTR1_CMP,
    C_ADC_VSMF_EOC,
    C_ADC_TEMP_EOC,
    C_ADC_MCUR_SLV2,
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE)
    C_ADC_VDDA_EOC,
    C_ADC_VDDD_EOC,
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) */
    C_ADC_VAUX_EOC,
    C_ADC_VBGD_EOC,
#else  /* (_SUPPORT_ADC_BGD != FALSE) */
    C_ADC_VS_MSTR1_CMP,
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE)
    C_ADC_VDDA_EOC,
    C_ADC_VDDD_EOC,
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) */
    C_ADC_MCUR_SLV2,
    C_ADC_VSMF_EOC,
    C_ADC_TEMP_EOC,
#endif /* (_SUPPORT_ADC_BGD != FALSE) */

#elif (_SUPPORT_PWM_MODE == BIPOLAR_FULL_STEP_BEMF) || (_SUPPORT_PWM_MODE == BIPOLAR_HALF_STEP_BEMF)
    /* Bi-Polar BEMF Zero-Cross
     *               MASTER1       SLAVE1       SLAVE2                 SLAVE3       MASTER2
     * Triggers at : 1/8 (12.5%)   2/6 (33.3%)  3/6+D (50.0%+D)        4/6 (66.7%)  0+D (0%+D)
     * Measurement : VS,VDDA,VDDD  -            M-Current,VSMF,Tj      -            -
     * BEMF_BRI    : VS,VDDA,VDDD, BEMF_1       M-Curr, BEMF_2, VSMF, Tj            -
     * Coil Current must be add 50% when using Mirror PWM
     * Max. 8 ADC channels per PWM-period of 20kHz (6.25us/ADC-channel @ 8MHz ADC-Clock)
     * (See also: MotorDriver.c)
     */
    C_ADC_VS_MSTR1_CMP,
    C_ADC_VDDA_EOC,
    C_ADC_VDDD_EOC,
#if (_SUPPORT_STALLDET_BRI != FALSE)                                            /* (MMP230802-1) */
    C_ADC_VPHU_EOC,
#endif /* (_SUPPORT_STALLDET_BRI != FALSE) */
    C_ADC_MCUR_SLV2,
#if (_SUPPORT_STALLDET_BRI != FALSE)                                            /* (MMP230802-1) */
#if (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_WT) || (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_TW)
    C_ADC_VPHV_EOC,
#elif (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UW_VT)
    C_ADC_VPHW_EOC,
#elif (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UT_VW)
    C_ADC_VPHT_EOC,
#else  /* (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_WT) || (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_TW) */
#error "ERROR: Bi-Polar Full-step not implemented"
#endif /* (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_WT) || (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_TW) */
#endif /* (_SUPPORT_STALLDET_BRI != FALSE) */
    C_ADC_VSMF_EOC,
    C_ADC_TEMP_EOC,

#elif (_SUPPORT_PWM_MODE == BIPOLAR_PWM_SINGLE_INDEPENDENT_GND)
    /* Bi-Polar Stepper
     *                MASTER1            SLAVE1       SLAVE2                       SLAVE3       MASTER2
     * Triggers at  : 1/8 (12.5%)        2/6 (33.3%)  3/6+D (50.0%+D)              4/6 (66.7%)  0+D (0%+D)
     * Measurement  : VS,VDDA,VDDD       -            M-CurrA,VSMF,Tj              -            M-CurrB
     * or Resolver  : VS,IO-X,IO-Y       -            M-CurrA,VSMF,Tj              -            M-CurrB
     * With VBGD:
     * Measurement  : VS,VSMF,Tj,(Opt)   -            M-CurrA,VDDA,VDDD,VAUX,VBGD  -            M-CurrB
     * or Resolver  : VS,VSMF,IO-X,IO-Y  -            M-CurrA,Tj,VDDA,VAUX,VBGD    -            M-CurrB
     *
     * Note 1: Coil Current must be measured at 50% (Coil "A") and 100% (Coil "B") when using Mirror PWM/Inverse Mirror-PWM
     * (See also: MotorDriver.c)
     * Note 2: VBGD must be measured after VAUX with max ADC delay of 4.0us for the best result.
     * Max. 10 ADC channels per PWM-period of 20kHz (4.5us/ADC-channel @ 4MHz ADC-Clock)
     */
#if (_SUPPORT_ADC_BGD != FALSE)                                                 /* MMP220307-1 */
    C_ADC_VS_MSTR1_CMP,
    C_ADC_VSMF_EOC,
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    C_ADC_RESOLVER_IOX_LV_EOC,
    C_ADC_RESOLVER_IOY_LV_EOC,
#else  /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    C_ADC_TEMP_EOC,
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    C_ADC_MCUR_SLV2,
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    C_ADC_TEMP_EOC,
    C_ADC_VDDA_EOC,
#else  /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    C_ADC_VDDA_EOC,
    C_ADC_VDDD_EOC,
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    C_ADC_VAUX_EOC,
    C_ADC_VBGD_EOC,
    C_ADC_MCUR_MSTR2
#else  /* (_SUPPORT_ADC_BGD != FALSE) */
    C_ADC_VS_MSTR1_CMP,
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    C_ADC_RESOLVER_IOX_LV_EOC,
    C_ADC_RESOLVER_IOY_LV_EOC,
#else  /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    C_ADC_VDDA_EOC,
    C_ADC_VDDD_EOC,
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    C_ADC_MCUR_SLV2,
    C_ADC_VSMF_EOC,
    C_ADC_TEMP_EOC,
    C_ADC_MCUR_MSTR2
#endif /* (_SUPPORT_ADC_BGD != FALSE) */

#elif (_SUPPORT_PWM_MODE == BIPOLAR_TRIPHASE_ALLPWM_MIRROR)
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
    /* Bi-Polar stepper as 3-phase actuator (FOC Based)
     *                MASTER1      SLAVE1                SLAVE2           SLAVE3           MASTER2
     * Triggers at  : 1/8 (12.5%)  2/8+D (25.0%+D)       4/8+D (50.0%+D)  6/8+D (75.0%+D)  -
     * Measurement  : VS           M-CurrA,VDDA,VSMF,Tj  -                M-CurrB
     * or (Middle)  : -            M-CurrA,VSMF,Tj       -                M-CurrB,VS
     * or (Min-Max) : -            M-CurrA,VSMF,Tj,      -                M-CurrB,VS,VDDA,VDDD
     * Coil Current must be measured at +/-25% (Coil "A") and +/-75% (Coil "B") when using All Mirror PWM
     * Max. 8 ADC channels per PWM-period of 20kHz (6.25us/ADC-channel @ 8MHz ADC-Clock)
     * Dual Motor Current sampling at 25-50% and 75-100% (ADC_TRIGGER_PWM_MIN_MAX) or 12.5-37.5%
     *  and 62.5-87.5% (ADC_TRIGGER_PWM_MID). In case of ADC_TRIGGER_PWM_MIN_MAX, between
     *  the two Motor-Current sampling fits 2-3 other ADC channel sampling.
     * (See also: MotorDriver.c)
     */
#if (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW)
    C_ADC_VS_EOC,
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX)
    C_ADC_VDDA_EOC,
    C_ADC_VDDD_EOC,
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX) */
    C_ADC_MCUR_SLV1,
#else  /* (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW) */
    C_ADC_VS_MSTR1_CMP,
    C_ADC_MCUR_SLV1,
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX)
    C_ADC_VDDA_EOC,
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX) */
#endif /* (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW) */
    C_ADC_VSMF_EOC,
    C_ADC_TEMP_EOC,
    C_ADC_MCUR_SLV2,

#else   /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
    /* Bi-Polar stepper as 3-phase actuator (Stepper)
     *                MASTER1      SLAVE1                    SLAVE2           SLAVE3           MASTER2
     * Triggers at  : 1/8 (12.5%)  2/8+D (25.0%+D)           4/8+D (50.0%+D)  6/8+D (75.0%+D)  -
     * Measurement  : VS           M-CurrA,VDDA,VSMF,Tj      -                M-CurrB
     * or (Resolver): VS           M-CurrA,IO-X,IO-Y,VSMF,Tj -                M-CurrB
     * or (SW-Trig) : -            M-CurrA,VDDA,VSMF,Tj      -                M-CurrB,VS,VDDA,VDDD
     * or (Resolver): -            M-CurrA,VDDA,VSMF,Tj      -                M-CurrB,VS,IO-X,IO-Y
     * Coil Current must be measured at +/-25% (Coil "A") and +/-75% (Coil "B") when using All Mirror PWM
     * Max. 8 ADC channels per PWM-period of 20kHz (6.25us/ADC-channel @ 8MHz ADC-Clock)
     * Dual Motor Current sampling at 25-50% and 75-100% (ADC_TRIGGER_PWM_MIN_MAX) or 12.5-37.5%
     *  and 62.5-87.5% (ADC_TRIGGER_PWM_MID). In case of ADC_TRIGGER_PWM_MIN_MAX, between
     *  the two Motor-Current sampling fits 2-3 other ADC channel sampling.
     * (See also: MotorDriver.c)
     */
#if (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW)
    C_ADC_VS_EOC,
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    C_ADC_RESOLVER_IOX_LV_EOC,
    C_ADC_RESOLVER_IOY_LV_EOC,
#elif (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX)
    C_ADC_VDDA_EOC,
    C_ADC_VDDD_EOC,
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    C_ADC_MCUR_SLV1,
#else  /* (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW) */
    C_ADC_VS_MSTR1_CMP,
    C_ADC_MCUR_SLV1,
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    C_ADC_RESOLVER_IOX_LV_EOC,
    C_ADC_RESOLVER_IOY_LV_EOC,
#elif (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX)
    C_ADC_VDDA_EOC,
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX) */
#endif /* (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW) */
    C_ADC_VSMF_EOC,
    C_ADC_TEMP_EOC,
    C_ADC_MCUR_SLV2,
#endif  /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */

#elif (_SUPPORT_PWM_MODE == TRIPLEPHASE_ALLPWM_MIRROR)
#if (_SUPPORT_STALLDET_BZC != FALSE)
    /* BLDC (3-Phase) BEMF Zero-Cross
     *                MASTER1               SLAVE1          SLAVE2       SLAVE3           MASTER2
     * Triggers at  : 1/8 (12.5%)           2/8+D (25.0%+D) 4/8 (50.0%)  6/8+D (75.0%+D)  -
     * Measurement  : VS,VDDA,VDDD,VSMF,Tj  -               -            M-Curr
     * Coil Current must be measured at +/-25% (Coil "A") or +/-75% (Coil "B") when using All Mirror PWM
     * Max. 8 ADC channels per PWM-period of 20kHz (6.25us/ADC-channel @ 8MHz ADC-Clock)
     * (See also: MotorDriver.c)
     */
    C_ADC_VS_MSTR1_CMP,
    C_ADC_VDDA_EOC,
    C_ADC_VDDD_EOC,
    C_ADC_VSMF_EOC,
    C_ADC_TEMP_EOC,
    C_ADC_MCUR_SLV3,

#elif (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
    /* BLDC (3-Phase) FOC-Based
     *                MASTER1      SLAVE1                SLAVE2       SLAVE3           MASTER2
     * Triggers at  : 1/8 (12.5%)  2/8+D (25.0%+D)       4/8 (50.0%)  6/8+D (75.0%+D)  -
     * Measurement  : VS           M-CurrA,VDDA,VSMF,Tj  -            M-CurrB
     * or (SW-Trig) :              M-CurrA,VSMF,Tj       -            M-CurrB,VS,VDDA,VDDD
     * Coil Current must be measured at +/-25% (Coil "A") or +/-75% (Coil "B") when using All Mirror PWM
     * Max. 8 ADC channels per PWM-period of 20kHz (6.25us/ADC-channel @ 8MHz ADC-Clock)
     * Dual Motor Current sampling at 25-50% and 75-100% (ADC_TRIGGER_PWM_MIN_MAX) or 12.5-37.5%
     *  and 62.5-87.5% (ADC_TRIGGER_PWM_MID). In case of ADC_TRIGGER_PWM_MIN_MAX, between
     *  the two Motor-Current sampling fits 2-3 other ADC channel sampling.
     * (See also: MotorDriver.c)
     */
#if (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW)
    C_ADC_VS_EOC,
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX)
    C_ADC_VDDA_EOC,
    C_ADC_VDDD_EOC,
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX) */
    C_ADC_MCUR_SLV1,
#else  /* (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW) */
    C_ADC_VS_MSTR1_CMP,
    C_ADC_MCUR_SLV1,
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX)
    C_ADC_VDDA_EOC,
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX) */
#endif /* (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW) */
    C_ADC_VSMF_EOC,
    C_ADC_TEMP_EOC,
    C_ADC_MCUR_SLV2,

#else
    /* BLDC (3-Phase) Non-FOC/BSZ-Based
     *                MASTER1      SLAVE1                     SLAVE2       SLAVE3           MASTER2
     * Triggers at  : 1/8 (12.5%)  2/8+D (25.0%+D)            4/8 (50.0%)  6/8+D (75.0%+D)  -
     * Measurement  : VS,          M-CurrA,VDDA,VSMF,Tj       -            M-CurrB
     * or (Resolver): VS,          M-CurrA,IO-X,IO-Y,VSMF,Tj  -            M-CurrB
     * or (SW-Trig) :              M-CurrA,VSMF,Tj            -            M-CurrB,VS,VDDA,VDDD
     * or (Resolver):              M-CurrA,VDDA,VSMF,Tj       -            M-CurrB,VS,IO-X,IO-Y
     * Coil Current must be measured at +/-25% (Coil "A") or +/-75% (Coil "B") when using All Mirror PWM
     * Max. 8 ADC channels per PWM-period of 20kHz (4.5us/ADC-channel @ 4MHz ADC-Clock)
     * Dual Motor Current sampling at 25-50% and 75-100% (ADC_TRIGGER_PWM_MIN_MAX) or 12.5-37.5%
     *  and 62.5-87.5% (ADC_TRIGGER_PWM_MID). In case of ADC_TRIGGER_PWM_MIN_MAX, between
     *  the two Motor-Current sampling fits 2-3 other ADC channel sampling.
     * (See also: MotorDriver.c)
     */
#if (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW)
    C_ADC_VS_EOC,
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    C_ADC_RESOLVER_IOX_LV_EOC,
    C_ADC_RESOLVER_IOY_LV_EOC,
#elif (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX)
    C_ADC_VDDA_EOC,
    C_ADC_VDDD_EOC,
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    C_ADC_MCUR_SLV1,
#else  /* (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW) */
    C_ADC_VS_MSTR1_CMP,
    C_ADC_MCUR_SLV1,
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    C_ADC_RESOLVER_IOX_LV_EOC,
    C_ADC_RESOLVER_IOY_LV_EOC,
#elif (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX)
    C_ADC_VDDA_EOC,
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX) */
#endif /* (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW) */
    C_ADC_VSMF_EOC,
    C_ADC_TEMP_EOC,
    C_ADC_MCUR_SLV2,
#endif

#elif (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_MIRROR_SUP)
    /* BLDC (3-Phase) Two Mirror PWM to Supply
     *                MASTER1                              SLAVE1       SLAVE2       SLAVE3       MASTER2
     * Triggers at  : 1/8 (12.5%)                          2/6 (33.3%)  3/6 (50.0%)  3/4 (75.0%)  0+D (0%+D)
     * Measurement  : VS,VDDA,VDDD,VSMF,Tj                 -            -            -            M-Curr
     * or (Resolver): VS,VDDA,VDDD,VSMF,Tj,X-IO,Y-IO       -            -            -            M-Curr
     * With VBGD:
     * Measurement  : VS,VSMF,Tj,VDDA,VDDD,VAUX,VBGD       -            -            -            M-Curr
     * or (Resolver): VS,VSMF,X-IO,Y-IO,Tj,VDDA,VAUX,VBGD  -            -            -            M-Curr
     * Coil Current must be measured at 0% when using Two Mirror PWM to Supply
     * Max. 10 ADC channels per PWM-period of 20kHz (4.5us/ADC-channel @ 4MHz ADC-Clock)
     * (See also: MotorDriver.c)
     */
#if (_SUPPORT_ADC_BGD != FALSE)                                                 /* MMP220307-1 */
    C_ADC_VS_MSTR1_CMP,
    C_ADC_VSMF_EOC,
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    C_ADC_RESOLVER_IOX_LV_EOC,
    C_ADC_RESOLVER_IOY_LV_EOC,
    C_ADC_TEMP_EOC,
    C_ADC_VDDA_EOC,
    C_ADC_VAUX_EOC,
    C_ADC_VBGD_EOC,
#else  /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    C_ADC_TEMP_EOC,
    C_ADC_VDDA_EOC,
    C_ADC_VDDD_EOC,
    C_ADC_VAUX_EOC,
    C_ADC_VBGD_EOC,
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */

#else  /* (_SUPPORT_ADC_BGD != FALSE) */
    C_ADC_VS_MSTR1_CMP,
    C_ADC_VDDA_EOC,
    C_ADC_VDDD_EOC,
    C_ADC_VSMF_EOC,
    C_ADC_TEMP_EOC,
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    C_ADC_RESOLVER_IOX_LV_EOC,
    C_ADC_RESOLVER_IOY_LV_EOC,
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    C_ADC_MCUR_MSTR2
#endif /* (_SUPPORT_ADC_BGD != FALSE) */

#elif (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_MIRROR_GND)
    /* BLDC (3-Phase) Two Mirror PWM to Ground
     *                MASTER1            SLAVE1       SLAVE2                      SLAVE3       MASTER2
     * Triggers at  : 1/8 (12.5%)        2/6 (33.3%)  3/6+D (50.0%+D)             4/6 (66.7%)  -
     * Measurement  : VS,VDDA,VDDD       -            M-Curr,VSMF,Tj              -
     * or (Resolver): VS,VDDA,VDDD       -            M-Curr,VSMF,Tj,X-IO,Y-IO    -
     * With VBGD:
     * Measurement  : VS,VMSF,Tj         -            M-Curr,VDDA,VDDD,VAUX,VBGD  -
     * or (Resolver): VS,VSMF,X-IO,Y-IO  -            M-Curr,Tj,VDDA,VAUX,VBGD    -
     * Coil Current must be measured at 50% when using Two Mirror PWM to Supply
     * Max. 10 ADC channels per PWM-period of 20kHz (4.5us/ADC-channel @ 4MHz ADC-Clock)
     * (See also: MotorDriver.c)
     */
#if (_SUPPORT_ADC_BGD != FALSE)                                                 /* MMP220307-1 */
    C_ADC_VS_MSTR1_CMP,
    C_ADC_VSMF_EOC,
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    C_ADC_RESOLVER_IOX_LV_EOC,
    C_ADC_RESOLVER_IOY_LV_EOC,
#else  /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    C_ADC_TEMP_EOC,
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    C_ADC_MCUR_SLV2,
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    C_ADC_TEMP_EOC,
    C_ADC_VDDA_EOC,
#else  /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    C_ADC_VDDA_EOC,
    C_ADC_VDDD_EOC,
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    C_ADC_VAUX_EOC,
    C_ADC_VBGD_EOC,
#else  /* (_SUPPORT_ADC_BGD != FALSE) */
    C_ADC_VS_MSTR1_CMP,
    C_ADC_VDDA_EOC,
    C_ADC_VDDD_EOC,
    C_ADC_MCUR_SLV2,
    C_ADC_VSMF_EOC,
    C_ADC_TEMP_EOC,
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    C_ADC_RESOLVER_IOX_LV_EOC,
    C_ADC_RESOLVER_IOY_LV_EOC,
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
#endif /* (_SUPPORT_ADC_BGD != FALSE) */

#elif (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_INDEPENDENT_GND) || \
    (_SUPPORT_PWM_MODE == BIPOLAR_TRIPHASE_TWOPWM_INDEPENDENT_GND)
    /* BLDC (3-Phase) Mirror/Inverse-Mirror PWM to Ground or Bi-Polar stepper as 3-phase actuator
     *                MASTER1            SLAVE1       SLAVE2                       SLAVE3       MASTER2
     * Triggers at  : 1/8 (12.5%)        2/6 (33.3%)  3/6+D (50.0%+D)              4/6 (66.7%)  0+D (0%+D)
     * Measurement  : VS,VDDA,VDDD       -            M-CurrA,VSMF,Tj              -            M-CurrB
     * or (Resolver): VS,IO-X,IO-Y       -            M-CurrA,VSMF,Tj,VDDA,VDDA    -            M-CurrB
     * With VBGD:
     * Measurement  : VS,VMSF,Tj,(Opt)   -            M-CurrA,VDDA,VDDD,VAUX,VBGD  -            M-CurrB
     * or Resolver  : VS,VMSF,IO-X,IO-Y  -            M-CurrA,Tj,VDDA,VAUX,VBGD    -            M-CurrB
     * or 2xResolver: VS,VMSF,IO-X,IO-Y  -            M-CurrA,Tj,IO-X,IO-Y         -            M-CurrB
     *
     * Note 1: Coil Current must be measured at 50% & 100% (Two of the three currents)
     * (See also: MotorDriver.c)
     * Note 2: VBGD must be measured after VAUX with max ADC delay of 4.0us for the best result.
     * Note 3: (Opt) = IO0-HV or IOx-LV
     * Max. 10 ADC channels per PWM-period of 20kHz (4.5us/ADC-channel @ 4MHz ADC-Clock)
     */
#if (_SUPPORT_ADC_BGD != FALSE)                                                 /* MMP220307-1 */
    /* Chip supply voltage (21:1) */
    {
        {
            .u2AdcMarker = C_ADC_NO_SIGN,
            .u5AdcChannel = C_ADC_VS_HV,
            .u3AdcVref = C_ADC_VREF_HV,
            .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CMP,
            .u1AdcReserved = 0U
        }
    },
    /* Driver supply voltage (21:1) */
    C_ADC_VSMF_EOC,
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    C_ADC_RESOLVER_IOX_LV_EOC,
    C_ADC_RESOLVER_IOY_LV_EOC,
#else  /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    C_ADC_TEMP_EOC,
#if (ANA_COMM != FALSE) && (C_ANA_IO == PIN_FUNC_IO_4)
    /* IO[4] */
    {
        {
            .u2AdcMarker = C_ADC_NO_SIGN,
            .u5AdcChannel = C_ADC_IO4_LV,
            .u3AdcVref = C_ADC_VREF_2_50_V,
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,
            .u1AdcReserved = 0U
        }
    },
#elif (_SUPPORT_IO_DUT_SELECT_HVIO != FALSE) && (HVIO_DUT_SELECT == PIN_FUNC_IO_0)
    /* IO[0]_HV */
    {
        {
            .u2AdcMarker = C_ADC_NO_SIGN,
            .u5AdcChannel = C_ADC_IO0_HV,
            .u3AdcVref = C_ADC_VREF_2_50_V,
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,
            .u1AdcReserved = 0U
        }
    },
#endif
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    /* Motor current (unfiltered) */
    C_ADC_MCUR_SLV2,
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    C_ADC_TEMP_EOC,
#if (_SUPPORT_RESOLVER2_X_IO != PIN_FUNC_IO_NONE) && (_SUPPORT_RESOLVER2_Y_IO != PIN_FUNC_IO_NONE)
    /* IO[2] */
    {
        {
            .u2AdcMarker = C_ADC_NO_SIGN,
            .u5AdcChannel = C_ADC_IO2_LV,
            .u3AdcVref = C_ADC_VREF_2_50_V,
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,
            .u1AdcReserved = 0U
        }
    },
    /* IO[3] */
    {
        {
            .u2AdcMarker = C_ADC_NO_SIGN,
            .u5AdcChannel = C_ADC_IO3_LV,
            .u3AdcVref = C_ADC_VREF_2_50_V,
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,
            .u1AdcReserved = 0U
        }
    },
#else  /* (_SUPPORT_RESOLVER2_X_IO != PIN_FUNC_IO_NONE) && (_SUPPORT_RESOLVER2_Y_IO != PIN_FUNC_IO_NONE) */
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE)
    C_ADC_VDDA_EOC,
    C_ADC_VAUX_EOC,
    C_ADC_VBGD_EOC,
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
#endif /* (_SUPPORT_RESOLVER2_X_IO != PIN_FUNC_IO_NONE) && (_SUPPORT_RESOLVER2_Y_IO != PIN_FUNC_IO_NONE) */
#else  /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE)
    C_ADC_VDDA_EOC,
    C_ADC_VDDD_EOC,
    C_ADC_VAUX_EOC,
    C_ADC_VBGD_EOC,
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) */
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    /* Motor current (unfiltered) */
    {
        {
            .u2AdcMarker = C_ADC_NO_SIGN,
            .u5AdcChannel = C_ADC_MCUR,
            .u3AdcVref = C_ADC_VREF_MCUR,
#if (_SUPPORT_ADC_PWM_DELAY != FALSE)
            .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR2_CMP,
#else  /* (_SUPPORT_ADC_PWM_DELAY != FALSE) */
            .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,
#endif /* (_SUPPORT_ADC_PWM_DELAY != FALSE) */
            .u1AdcReserved = 0U
        }
    }
#else  /* (_SUPPORT_ADC_BGD != FALSE) */
    C_ADC_VS_MSTR1_CMP,
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    C_ADC_RESOLVER_IOX_LV_EOC,
    C_ADC_RESOLVER_IOY_LV_EOC,
#elif (_SUPPORT_ADC_VDDA_VDDD != FALSE)
    C_ADC_VDDA_EOC,
    C_ADC_VDDD_EOC,
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    C_ADC_MCUR_SLV2,
    C_ADC_VSMF_EOC,
    C_ADC_TEMP_EOC,
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
#if (_SUPPORT_RESOLVER2_X_IO != PIN_FUNC_IO_NONE) && (_SUPPORT_RESOLVER2_Y_IO != PIN_FUNC_IO_NONE)
    /* Second Resolver X */
    {
        {
            .u2AdcMarker = C_ADC_NO_SIGN,
            .u5AdcChannel = C_ADC_IO2_LV,
            .u3AdcVref = C_ADC_VREF_2_50_V,
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,
            .u1AdcReserved = 0U
        }
    },
    /* Second Resolver Y */
    {
        {
            .u2AdcMarker = C_ADC_NO_SIGN,
            .u5AdcChannel = C_ADC_IO3_LV,
            .u3AdcVref = C_ADC_VREF_2_50_V,
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,
            .u1AdcReserved = 0U
        }
    },
#elif (_SUPPORT_ADC_VDDA_VDDD != FALSE)
    C_ADC_VDDA_EOC,
    C_ADC_VDDD_EOC,
#endif /* (_SUPPORT_RESOLVER2_X_IO != PIN_FUNC_IO_NONE) && (_SUPPORT_RESOLVER2_Y_IO != PIN_FUNC_IO_NONE) */
#elif (ANA_COMM != FALSE) && (C_ANA_IO == PIN_FUNC_IO_4)
    {
        {
            .u2AdcMarker = C_ADC_NO_SIGN,
            .u5AdcChannel = C_ADC_IO4_LV,
            .u3AdcVref = C_ADC_VREF_2_50_V,
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,
            .u1AdcReserved = 0U
        }
    },
#elif (_SUPPORT_IO_DUT_SELECT_HVIO != FALSE) && (HVIO_DUT_SELECT == PIN_FUNC_IO_0)
    {
        {
            .u2AdcMarker = C_ADC_NO_SIGN,
            .u5AdcChannel = C_ADC_IO0_HV,
            .u3AdcVref = C_ADC_VREF_2_50_V,
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,
            .u1AdcReserved = 0U
        }
    },
#elif (_SUPPORT_NTC != FALSE)
    C_ADC_NTC_IO_LV_EOC,
#elif (_SUPPORT_POTI != FALSE)
    C_ADC_POTI_IOX_EOC,
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    {
        {
            .u2AdcMarker = C_ADC_NO_SIGN,
            .u5AdcChannel = C_ADC_MCUR,
#if !defined (__MLX81339__) && !defined (__MLX81350__)
            .u3AdcVref = C_ADC_VREF_MCUR,
#else  /* !defined (__MLX81339__) && !defined (__MLX81350__) */
            .u3AdcReserved = 0U,
#endif /* !defined (__MLX81339__) && !defined (__MLX81350__) */
#if (_SUPPORT_ADC_PWM_DELAY != FALSE)
            .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR2_CMP,
#else  /* (_SUPPORT_ADC_PWM_DELAY != FALSE) */
            .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,
#endif /* (_SUPPORT_ADC_PWM_DELAY != FALSE) */
            .u1AdcReserved = 0U
        }
    }
#endif /* (_SUPPORT_ADC_BGD != FALSE) */

#elif (_SUPPORT_PWM_MODE == TRIPLEPHASE_FULL_STEP_BEMF)
    /* BLDC (3-Phase) Full Step BEMF
     *                MASTER1       SLAVE1       SLAVE2                SLAVE3       MASTER2
     * Triggers at  : 1/8 (12.5%)   2/6 (33.3%)  4/8 (50.0%)           6/8 (75.0%)  -
     * Measurement  : VS,VDDA,VDDD  -            M-Curr,U|V|W,VSMF,Tj  -
     * Coil Current and BEMF-Voltage must be measured at 50%
     * Max. 8 ADC channels per PWM-period of 20kHz (6.25us/ADC-channel @ 8MHz ADC-Clock)
     * (See also: MotorDriver.c)
     */
    C_ADC_VS_MSTR1_CMP,
    C_ADC_VDDA_EOC,
    C_ADC_VDDD_EOC,
    C_ADC_MCUR_SLV2,
#if (_SUPPORT_STALLDET_BRI != FALSE)
    {
        {
            .u2AdcMarker = C_ADC_NO_SIGN,
            .u5AdcChannel = C_ADC_PH_U_HV,
            .u3AdcVref = C_ADC_VREF_2_50_V,
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,
            .u1AdcReserved = 0U
        }
    },
#endif /* (_SUPPORT_STALLDET_BRI != FALSE) */
    C_ADC_VSMF_EOC,
    C_ADC_TEMP_EOC,
#endif
#endif /* (_SUPPORT_APP_TYPE == C_APP_SOLENOID) */
};

#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_VARIABLE_PWM != FALSE)
/** ADC SBASE for motor run, based on 3-PWM (MMP220815-1) */
static const ADC_SDATA_t SBASE_INIT_3PWM[] =                                       /* ADC Automated measurements */
{                                                                               /* Ch  Trigger    Vref  Description */
    {.u16 = (uint16_t)&l_AdcResult3.u16AdcVs},
    C_ADC_VS_MSTR1_CMP,
    C_ADC_MCUR_SLV1,
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE)
    C_ADC_VDDA_EOC,
#if (_SUPPORT_ADC_BGD != FALSE)
    C_ADC_VBGD_EOC,
#endif /* (_SUPPORT_ADC_BGD != FALSE) */
    {
        {
            .u2AdcMarker = C_ADC_NO_SIGN,
            .u5AdcChannel = C_ADC_VDDD,
            .u3AdcVref = C_ADC_VREF_2_50_V,
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,
            .u1AdcReserved = 0U
        }
    },
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) */
    C_ADC_MCUR_SLV3,
    C_ADC_VSMF_EOC,
    C_ADC_TEMP_EOC,
#if (_SUPPORT_ADC_VBOOST != FALSE)
    C_ADC_VBOOST_EOC,
#endif /* (_SUPPORT_ADC_VBOOST != FALSE) */
    C_ADC_MCUR_MSTR2
};
#endif /* (_SUPPORT_VARIABLE_PWM != FALSE) */

/*! ADC Table for idle (optional: holding) actuator */
static ADC_SDATA_t const SBASE_MOTOR_IDLE[] = /*lint !e578 */                   /* ADC Automated measurements */
{
#if (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
    /* Solenoid */
    C_ADC_VS_EOC,
    C_ADC_VDDA_EOC,
    C_ADC_VDDD_EOC,
    C_ADC_MCUR_EOC,
    C_ADC_VSMF_EOC,
    C_ADC_TEMP_EOC,
#else  /* (_SUPPORT_APP_TYPE == C_APP_SOLENOID) */
#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM)
    /* DC-Motor */
    C_ADC_VS_EOC,
    C_ADC_VDDA_EOC,
    C_ADC_VDDD_EOC,
    C_ADC_MCUR_EOC,
    C_ADC_MCUR_EOC,
    C_ADC_VSMF_EOC,
    C_ADC_TEMP_EOC,
#if (_SUPPORT_POTI != FALSE)
    C_ADC_POTI_IOX_EOC,
#elif (_SUPPORT_IO_DUT_SELECT_HVIO != FALSE) && (HVIO_DUT_SELECT == PIN_FUNC_IO_0)
    C_ADC_IO0_HV_EOC,
#endif /* (_SUPPORT_POTI != FALSE) */
    C_ADC_VS_EOC,
#if (_SUPPORT_IO_CMD_SELECT_LVIO != FALSE)
#if (HVIO_DUT_SELECT_A == PIN_FUNC_IO_0)
    C_ADC_IO0_LV_EOC,
#elif (HVIO_DUT_SELECT_A == PIN_FUNC_IO_1)
    C_ADC_IO1_LV_EOC,
#elif (HVIO_DUT_SELECT_A == PIN_FUNC_IO_2)
    C_ADC_IO2_LV_EOC,
#elif (HVIO_DUT_SELECT_A == PIN_FUNC_IO_3)
    C_ADC_IO3_LV_EOC,
#elif (HVIO_DUT_SELECT_A == PIN_FUNC_IO_4)
    C_ADC_IO4_LV_EOC,
#elif (HVIO_DUT_SELECT_A == PIN_FUNC_IO_5)
    C_ADC_IO5_LV_EOC,
#elif (HVIO_DUT_SELECT_A == PIN_FUNC_IO_6)
    C_ADC_IO6_LV_EOC,
#elif (HVIO_DUT_SELECT_A == PIN_FUNC_IO_7)
    C_ADC_IO7_LV_EOC,
#endif /* (HVIO_DUT_SELECT_A) */
#if (_SUPPORT_NR_OF_IO_SELECT >= 2U)
#if (HVIO_DUT_SELECT_B == PIN_FUNC_IO_1)
    C_ADC_IO1_LV_EOC,
#elif (HVIO_DUT_SELECT_B == PIN_FUNC_IO_2)
    C_ADC_IO2_LV_EOC,
#elif (HVIO_DUT_SELECT_B == PIN_FUNC_IO_3)
    C_ADC_IO3_LV_EOC,
#elif (HVIO_DUT_SELECT_B == PIN_FUNC_IO_4)
    C_ADC_IO4_LV_EOC,
#elif (HVIO_DUT_SELECT_B == PIN_FUNC_IO_5)
    C_ADC_IO5_LV_EOC,
#elif (HVIO_DUT_SELECT_B == PIN_FUNC_IO_6)
    C_ADC_IO6_LV_EOC,
#elif (HVIO_DUT_SELECT_B == PIN_FUNC_IO_7)
    C_ADC_IO7_LV_EOC,
#endif /* (HVIO_DUT_SELECT_B) */
#if (_SUPPORT_NR_OF_IO_SELECT >= 3U)
#if (HVIO_DUT_SELECT_C == PIN_FUNC_IO_2)
    C_ADC_IO2_LV_EOC,
#elif (HVIO_DUT_SELECT_C == PIN_FUNC_IO_3)
    C_ADC_IO3_LV_EOC,
#elif (HVIO_DUT_SELECT_C == PIN_FUNC_IO_4)
    C_ADC_IO4_LV_EOC,
#elif (HVIO_DUT_SELECT_C == PIN_FUNC_IO_5)
    C_ADC_IO5_LV_EOC,
#elif (HVIO_DUT_SELECT_C == PIN_FUNC_IO_6)
    C_ADC_IO6_LV_EOC,
#elif (HVIO_DUT_SELECT_C == PIN_FUNC_IO_7)
    C_ADC_IO7_LV_EOC,
#endif /* (HVIO_DUT_SELECT_C) */
#endif /* (_SUPPORT_NR_OF_IO_SELECT >= 3U) */
#endif /* (_SUPPORT_NR_OF_IO_SELECT >= 2U) */
#endif /* (_SUPPORT_IO_CMD_SELECT_LVIO != FALSE) */
    C_ADC_MCUR_EOC,
    C_ADC_MCUR_EOC,
    C_ADC_VSMF_EOC,
    C_ADC_TEMP_EOC
#if (ANA_COMM != FALSE) && (C_ANA_IO == PIN_FUNC_IO_0)
    C_ADC_IO0_HV_EOC,
#endif /* (ANA_COMM != FALSE) && (C_ANA_IO == PIN_FUNC_IO_0) */

#elif (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR)
    /* Single Coil Fan/Pump */
#if (_SUPPORT_ADC_BGD != FALSE)                                                 /* MMP220307-1 */
    C_ADC_VS_EOC,
    C_ADC_VSMF_EOC,
    C_ADC_TEMP_EOC,
    C_ADC_MCUR_EOC,
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE)
    C_ADC_VDDA_EOC,
    C_ADC_VDDD_EOC,
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) */
    C_ADC_VAUX_EOC,
    C_ADC_VBGD_EOC,
#else  /* (_SUPPORT_ADC_BGD != FALSE) */
    C_ADC_VS_EOC,
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE)
    C_ADC_VDDA_EOC,
    C_ADC_VDDD_EOC,
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) */
    C_ADC_MCUR_EOC,
    C_ADC_VSMF_EOC,
    C_ADC_TEMP_EOC,
#endif /* (_SUPPORT_ADC_BGD != FALSE) */

#elif (_SUPPORT_PWM_MODE == BIPOLAR_FULL_STEP_BEMF) || (_SUPPORT_PWM_MODE == BIPOLAR_HALF_STEP_BEMF)
    C_ADC_VS_EOC,
    C_ADC_VDDA_EOC,
    C_ADC_VDDD_EOC,
#if (_SUPPORT_STALLDET_BRI != FALSE)                                            /* (MMP230802-1) */
    C_ADC_VPHU_EOC,
#endif /* (_SUPPORT_STALLDET_BRI != FALSE) */
    C_ADC_MCUR_EOC,
#if (_SUPPORT_STALLDET_BRI != FALSE)                                            /* (MMP230802-1) */
#if (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_WT) || (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_TW)
    C_ADC_VPHV_EOC,
#elif (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UW_VT)
    C_ADC_VPHW_EOC,
#elif (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UT_VW)
    C_ADC_VPHT_EOC,
#else  /* (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_WT) || (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_TW) */
#error "ERROR: Bi-Polar Full-step not implemented"
#endif /* (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_WT) || (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_TW) */
#endif /* (_SUPPORT_STALLDET_BRI != FALSE) */
    C_ADC_VSMF_EOC,
    C_ADC_TEMP_EOC,

#elif (_SUPPORT_PWM_MODE == BIPOLAR_PWM_SINGLE_INDEPENDENT_GND)
#if (_SUPPORT_ADC_BGD != FALSE)                                                 /* MMP220307-1 */
    C_ADC_VS_EOC,
    C_ADC_VSMF_EOC,
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    C_ADC_RESOLVER_IOX_LV_EOC,
    C_ADC_RESOLVER_IOY_LV_EOC,
#else  /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    C_ADC_TEMP_EOC,
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    C_ADC_MCUR_EOC,
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    C_ADC_TEMP_EOC,
    C_ADC_VDDA_EOC,
#else  /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    C_ADC_VDDA_EOC,
    C_ADC_VDDD_EOC,
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    C_ADC_VAUX_EOC,
    C_ADC_VBGD_EOC,
#else  /* (_SUPPORT_ADC_BGD != FALSE) */
    C_ADC_VS_EOC,
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    C_ADC_RESOLVER_IOX_LV_EOC,
    C_ADC_RESOLVER_IOY_LV_EOC,
#else  /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    C_ADC_VDDA_EOC,
    C_ADC_VDDD_EOC,
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    C_ADC_MCUR_EOC,
    C_ADC_VSMF_EOC,
    C_ADC_TEMP_EOC,
#endif  /* (_SUPPORT_ADC_BGD != FALSE) */

#elif (_SUPPORT_PWM_MODE == BIPOLAR_TRIPHASE_ALLPWM_MIRROR)
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
    /** ADC SBASE for motor run, based on 2-PWM */
    C_ADC_VS_EOC,
#if (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW)
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX)
    C_ADC_VDDA_EOC,
    C_ADC_VDDD_EOC,
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX) */
    C_ADC_MCUR_EOC,
#else  /* (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW) */
    C_ADC_MCUR_EOC,
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX)
    C_ADC_VDDA_EOC,
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX) */
#endif /* (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW) */
    C_ADC_VSMF_EOC,
    C_ADC_TEMP_EOC,

#else   /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
    C_ADC_VS_EOC,
#if (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW)
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    C_ADC_RESOLVER_IOX_LV_EOC,
    C_ADC_RESOLVER_IOY_LV_EOC,
#elif (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX)
    C_ADC_VDDA_EOC,
    C_ADC_VDDD_EOC,
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    C_ADC_MCUR_EOC,
#else  /* (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW) */
    C_ADC_MCUR_EOC,
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    C_ADC_RESOLVER_IOX_LV_EOC,
    C_ADC_RESOLVER_IOY_LV_EOC,
#elif (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX)
    C_ADC_VDDA_EOC,
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX) */
#endif /* (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW) */
    C_ADC_VSMF_EOC,
    C_ADC_TEMP_EOC,
#endif  /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */

#elif (_SUPPORT_PWM_MODE == TRIPLEPHASE_ALLPWM_MIRROR)
#if (_SUPPORT_STALLDET_BZC != FALSE)
    /* BLDC BEMF Zero-Cross */
    C_ADC_VS_EOC,
    C_ADC_VDDA_EOC,
    C_ADC_VDDD_EOC,
#if (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_MIRROR_GND) || (_SUPPORT_PWM_MODE == BIPOLAR_PWM_SINGLE_INDEPENDENT_GND)
    C_ADC_MCUR_EOC,
#endif /* (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_MIRROR_GND) || (_SUPPORT_PWM_MODE == BIPOLAR_PWM_SINGLE_INDEPENDENT_GND) */
    C_ADC_VSMF_EOC,
    C_ADC_TEMP_EOC,
    C_ADC_MCUR_EOC,

#elif (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
    /** ADC SBASE for motor run, based on 2-PWM */
    C_ADC_VS_EOC,
#if (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW)
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX)
    C_ADC_VDDA_EOC,
    C_ADC_VDDD_EOC,
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX) */
    C_ADC_MCUR_EOC,
#else  /* (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW) */
    C_ADC_MCUR_EOC,
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX)
    C_ADC_VDDA_EOC,
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX) */
#endif /* (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW) */
    C_ADC_VSMF_EOC,
    C_ADC_TEMP_EOC,

#else
    C_ADC_VS_EOC,
#if (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW)
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    C_ADC_RESOLVER_IOX_LV_EOC,
    C_ADC_RESOLVER_IOY_LV_EOC,
#elif (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX)
    C_ADC_VDDA_EOC,
    C_ADC_VDDD_EOC,
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    C_ADC_MCUR_EOC,
#else  /* (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW) */
    C_ADC_MCUR_EOC,
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    C_ADC_RESOLVER_IOX_LV_EOC,
    C_ADC_RESOLVER_IOY_LV_EOC,
#elif (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX)
    C_ADC_VDDA_EOC,
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX) */
#endif /* (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW) */
    C_ADC_VSMF_EOC,
    C_ADC_TEMP_EOC,
#endif

#elif (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_MIRROR_SUP)
#if (_SUPPORT_ADC_BGD != FALSE)                                                 /* MMP220307-1 */
    C_ADC_VS_EOC,
    C_ADC_VSMF_EOC,
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    C_ADC_RESOLVER_IOX_LV_EOC,
    C_ADC_RESOLVER_IOY_LV_EOC,
    C_ADC_TEMP_EOC,
    C_ADC_VDDA_EOC,
#else  /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    C_ADC_VDDA_EOC,
    C_ADC_VDDD_EOC,
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    C_ADC_VAUX_EOC,
    C_ADC_VBGD_EOC,
#else  /* (_SUPPORT_ADC_BGD != FALSE) */
    C_ADC_VS_EOC,
    C_ADC_VDDA_EOC,
    C_ADC_VDDD_EOC,
    C_ADC_VSMF_EOC,
    C_ADC_TEMP_EOC,
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    C_ADC_RESOLVER_IOX_LV_EOC,
    C_ADC_RESOLVER_IOY_LV_EOC,
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    C_ADC_MCUR_EOC,
#endif /* (_SUPPORT_ADC_BGD != FALSE) */

#elif (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_MIRROR_GND)
#if (_SUPPORT_ADC_BGD != FALSE)
    C_ADC_VS_EOC,
    C_ADC_VSMF_EOC,
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    C_ADC_RESOLVER_IOX_LV_EOC,
    C_ADC_RESOLVER_IOY_LV_EOC,
#else  /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    C_ADC_TEMP_EOC,
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    C_ADC_MCUR_EOC,
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    C_ADC_TEMP_EOC,
    C_ADC_VDDA_EOC,
#else  /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    C_ADC_VDDA_EOC,
    C_ADC_VDDD_EOC,
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    C_ADC_VAUX_EOC,
    C_ADC_VBGD_EOC,
#else  /* (_SUPPORT_ADC_BGD != FALSE) */
    C_ADC_VS_EOC,
    C_ADC_VDDA_EOC,
    C_ADC_VDDD_EOC,
    C_ADC_MCUR_EOC,
    C_ADC_VSMF_EOC,
    C_ADC_TEMP_EOC,
#endif /* (_SUPPORT_ADC_BGD != FALSE) */

#elif (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_INDEPENDENT_GND)
#if (_SUPPORT_ADC_BGD != FALSE)
    C_ADC_VS_EOC,
    C_ADC_VSMF_EOC,
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    C_ADC_RESOLVER_IOX_LV_EOC,
    C_ADC_RESOLVER_IOY_LV_EOC,
#else  /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    C_ADC_TEMP_EOC,
#if (ANA_COMM != FALSE) && (C_ANA_IO == PIN_FUNC_IO_4)
    C_ADC_IO4_LV_EOC,
#elif (_SUPPORT_IO_DUT_SELECT_HVIO != FALSE) && (HVIO_DUT_SELECT == PIN_FUNC_IO_0)
    C_ADC_IO0_HV_EOC,
#endif
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    C_ADC_MCUR_EOC,
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    C_ADC_TEMP_EOC,
#if (_SUPPORT_RESOLVER2_X_IO != PIN_FUNC_IO_NONE) && (_SUPPORT_RESOLVER2_Y_IO != PIN_FUNC_IO_NONE)
    C_ADC_IO2_LV_EOC,
    C_ADC_IO3_LV_EOC,
#else  /* (_SUPPORT_RESOLVER2_X_IO != PIN_FUNC_IO_NONE) && (_SUPPORT_RESOLVER2_Y_IO != PIN_FUNC_IO_NONE) */
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE)
    C_ADC_VDDA_EOC,
    C_ADC_VAUX_EOC,
    C_ADC_VBGD_EOC,
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
#endif /* (_SUPPORT_RESOLVER2_X_IO != PIN_FUNC_IO_NONE) && (_SUPPORT_RESOLVER2_Y_IO != PIN_FUNC_IO_NONE) */
#else  /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE)
    C_ADC_VDDA_EOC,
    C_ADC_VDDD_EOC,
    C_ADC_VAUX_EOC,
    C_ADC_VBGD_EOC,
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) */
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
#else  /* (_SUPPORT_ADC_BGD != FALSE) */
    {
        {
            .u2AdcMarker = C_ADC_NO_SIGN,
            .u5AdcChannel = C_ADC_VS_HV,                                        /* Chip supply voltage (21:1) */
#if !defined (__MLX81339__) && !defined (__MLX81350__)
            .u3AdcVref = C_ADC_VREF_HV,
#else  /* !defined (__MLX81339__) && !defined (__MLX81350__) */
            .u3AdcReserved = 0U,
#endif /* !defined (__MLX81339__) && !defined (__MLX81350__) */
            .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,
            .u1AdcReserved = 0U
        }
    },
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    C_ADC_RESOLVER_IOX_LV_EOC,
    C_ADC_RESOLVER_IOY_LV_EOC,
#elif (_SUPPORT_ADC_VDDA_VDDD != FALSE)
    C_ADC_VDDA_EOC,
    C_ADC_VDDD_EOC,
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    C_ADC_MCUR_EOC,
    C_ADC_VSMF_EOC,
    C_ADC_TEMP_EOC,
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
#if (_SUPPORT_RESOLVER2_X_IO != PIN_FUNC_IO_NONE) && (_SUPPORT_RESOLVER2_Y_IO != PIN_FUNC_IO_NONE)
    C_ADC_IO2_LV_EOC,
    C_ADC_IO3_LV_EOC,
#elif (_SUPPORT_ADC_VDDA_VDDD != FALSE)
    C_ADC_VDDA_EOC,
    C_ADC_VDDD_EOC,
#endif /* (_SUPPORT_RESOLVER2_X_IO != PIN_FUNC_IO_NONE) && (_SUPPORT_RESOLVER2_Y_IO != PIN_FUNC_IO_NONE) */
#elif (ANA_COMM != FALSE) && (C_ANA_IO == PIN_FUNC_IO_4)
    C_ADC_IO4_LV_EOC,
#elif (_SUPPORT_IO_DUT_SELECT_HVIO != FALSE) && (HVIO_DUT_SELECT == PIN_FUNC_IO_0)
    C_ADC_IO0_HV_EOC,
#elif (_SUPPORT_NTC != FALSE)
    C_ADC_NTC_IO_LV_EOC,
#elif (_SUPPORT_POTI != FALSE)
    C_ADC_POTI_IOX_EOC,
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    C_ADC_MCUR_EOC,
#endif /* (_SUPPORT_ADC_BGD != FALSE) */

#elif (_SUPPORT_PWM_MODE == TRIPLEPHASE_FULL_STEP_BEMF)
    C_ADC_VS_EOC,
    C_ADC_VDDA_EOC,
    C_ADC_VDDD_EOC,
    C_ADC_MCUR_EOC,
#if (_SUPPORT_STALLDET_BRI != FALSE)
    C_ADC_VPHU_EOC,
#endif /* (_SUPPORT_STALLDET_BRI != FALSE) */
    C_ADC_VSMF_EOC,
    C_ADC_TEMP_EOC,
#endif
#endif /* (_SUPPORT_APP_TYPE == C_APP_SOLENOID) */
};

#if (_SUPPORT_PWM_MODE == TRIPLEPHASE_FULL_STEP_BEMF) || (_SUPPORT_PWM_MODE == BIPOLAR_FULL_STEP_BEMF)
/*!*************************************************************************** *
 * ADC_Start
 * \brief   Start ADC (with waiting for pending ADC conversions to be finished)
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16FullStep: Full-step index
 * \param   [in] u16IrqEna: FALSE Disable IRQ, TRUE: Enable IRQ (MMP230919-1)
 * \return  -
 * *************************************************************************** *
 * \details Start ADC by using hardware triggers
 *          ADC ISR priority: None (6, not used)
 * *************************************************************************** *
 * - Call Hierarchy: MotordriverStart()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 1 (HAL_ADC_StopSafe())
 * *************************************************************************** */
void ADC_Start(uint16_t u16FullStep, uint16_t u16IrqEna)
#elif (_SUPPORT_PWM_MODE == BIPOLAR_HALF_STEP_BEMF)
/*!*************************************************************************** *
 * ADC_Start
 * \brief   Start ADC (with waiting for pending ADC conversions to be finished)
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16HalfStep: Full-step index
 * \param   [in] u16IrqEna: FALSE Disable IRQ, TRUE: Enable IRQ (MMP230919-1)
 * \return  -
 * *************************************************************************** *
 * \details Start ADC by using hardware triggers
 *          ADC ISR priority: None (6, not used)
 * *************************************************************************** *
 * - Call Hierarchy: MotordriverStart()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 1 (HAL_ADC_StopSafe())
 * *************************************************************************** */
void ADC_Start(uint16_t u16HalfStep, uint16_t u16IrqEna)
#else  /* (_SUPPORT_PWM_MODE == TRIPLEPHASE_FULL_STEP_BEMF) || (_SUPPORT_PWM_MODE == BIPOLAR_FULL_STEP_BEMF) || (_SUPPORT_PWM_MODE == BIPOLAR_HALF_STEP_BEMF) */
/*!*************************************************************************** *
 * ADC_Start
 * \brief   Start ADC (with waiting for pending ADC conversions to be finished)
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16IrqEna: FALSE Disable IRQ, TRUE: Enable IRQ (MMP230919-1)
 * \return  -
 * *************************************************************************** *
 * \details Start ADC by using hardware triggers
 *          ADC ISR priority: None (6, not used)
 * *************************************************************************** *
 * - Call Hierarchy: MotordriverStart()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 1 (HAL_ADC_StopSafe())
 * *************************************************************************** */
void ADC_Start(uint16_t u16IrqEna)
#endif /* (_SUPPORT_PWM_MODE == TRIPLEPHASE_FULL_STEP_BEMF) || (_SUPPORT_PWM_MODE == BIPOLAR_FULL_STEP_BEMF) || (_SUPPORT_PWM_MODE == BIPOLAR_HALF_STEP_BEMF) */
{
#if ((_SUPPORT_PWM_MODE == TRIPLEPHASE_FULL_STEP_BEMF) || (_SUPPORT_PWM_MODE == BIPOLAR_FULL_STEP_BEMF)) && (_SUPPORT_STALLDET_BZC != FALSE)
    (void) u16FullStep;
    if (l_u8AdcMode != C_ADC_MODE_ON_STEPPER)                                   /* MMP181123-2 */
#elif (_SUPPORT_PWM_MODE == BIPOLAR_HALF_STEP_BEMF) && (_SUPPORT_STALLDET_BZC != FALSE)
    (void) u16HalfStep;
    if (l_u8AdcMode != C_ADC_MODE_ON_STEPPER)                                   /* MMP181123-2 */
#else  /* ((_SUPPORT_PWM_MODE == TRIPLEPHASE_FULL_STEP_BEMF) || (_SUPPORT_PWM_MODE == BIPOLAR_FULL_STEP_BEMF) || (_SUPPORT_PWM_MODE == BIPOLAR_HALF_STEP_BEMF)) && (_SUPPORT_STALLDET_BZC != FALSE) */
    if (l_u8AdcMode != C_ADC_MODE_RUN_HW)                                       /* MMP181123-2 */
#endif /* ((_SUPPORT_PWM_MODE == TRIPLEPHASE_FULL_STEP_BEMF) || (_SUPPORT_PWM_MODE == BIPOLAR_FULL_STEP_BEMF) || (_SUPPORT_PWM_MODE == BIPOLAR_HALF_STEP_BEMF)) && (_SUPPORT_STALLDET_BZC != FALSE) */
    {
        HAL_ADC_StopSafe();

        /* Set SBASE structure in RAM, to prevent MCU performance losses due to ADC DMA accesses from Flash */
        {
            uint16_t *pu16Src = &l_au16AdcSource[0];
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_VARIABLE_PWM != FALSE)    /* MMP220815-1 */
            if (g_u8TriplePWM != FALSE)
            {
                /* Slow speed: 3 PWM's */
                uint16_t *pu16SBase = (uint16_t *)&SBASE_INIT_3PWM[0];
                do
                {
                    *pu16Src = *pu16SBase;
                    pu16Src++;
                    pu16SBase++;
                } while (pu16SBase < &SBASE_INIT_3PWM[sizeof(SBASE_INIT_3PWM) / sizeof(uint16_t)]);
            }
            else
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_VARIABLE_PWM != FALSE) */
            {
                uint16_t *pu16SBase = (uint16_t *)&SBASE_MOTOR_RUN[0];
                *pu16Src++ = (uint16_t)&l_AdcResult;                            /* DBASE */
                pu16Src = p_MemCpyU16(pu16Src, pu16SBase, (sizeof(SBASE_MOTOR_RUN) / sizeof(uint16_t)));  /* MMP230512-1: Code size reduction */
            }
            *pu16Src++ = C_ADC_EOF;                                             /* End-of-Frame */
            *pu16Src = (uint16_t)&l_au16AdcSource[0];                           /* Repeat forever */
#if (_SUPPORT_STALLDET_BRI != FALSE)
#if (C_MOTOR_COILS == 3U)
            if ( (u16FullStep == 1U) || (u16FullStep == 4U) )
            {
                ADC_SDATA_t u16PhV = C_ADC_VPHV_EOC;

                l_au16AdcSource[5] = u16PhV.u16;                                /* 13  50.0%+D+3n 2.5V  Motor BEMF-Voltage (unfiltered) */
            }
            else if ( (u16FullStep == 2U) || (u16FullStep == 5U) )
            {
                ADC_SDATA_t u16PhW = C_ADC_VPHW_EOC;

                l_au16AdcSource[5] = u16PhW.u16;                                /* 14  50.0%+D+3n 2.5V  Motor BEMF-Voltage (unfiltered) */
            }
            else
            {
                /* Nothing */
            }
#elif (C_MOTOR_COILS == 2U) && (C_MOTOR_PHASES == 4U)
#if (_SUPPORT_PWM_MODE == BIPOLAR_FULL_STEP_BEMF)
            if ( (u16FullStep == 0U) || (u16FullStep == 2U) )
#else  /* (_SUPPORT_PWM_MODE == BIPOLAR_FULL_STEP_BEMF) */
            if ( (u16HalfStep == 0U) || (u16HalfStep == 4U) )
#endif /* (_SUPPORT_PWM_MODE == BIPOLAR_FULL_STEP_BEMF) */
            {
#if (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_WT) || (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_TW)
                ADC_SDATA_t u16PhT = C_ADC_VPHT_EOC;
                ADC_SDATA_t u16PhW = C_ADC_VPHW_EOC;

                l_au16AdcSource[4] = u16PhW.u16;                                /* 14  50.0%+D+3n 2.5V  Motor BEMF-Voltage (unfiltered) (MMP230802-1) */
                l_au16AdcSource[6] = u16PhT.u16;                                /* 15  50.0%+D+4n 2.5V  Motor BEMF-Voltage (unfiltered) (MMP230802-1) */
#elif (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UW_VT)
                ADC_SDATA_t u16PhT = C_ADC_VPHT_EOC;
                ADC_SDATA_t u16PhV = C_ADC_VPHV_EOC;

                l_au16AdcSource[4] = u16PhV.u16;                                /* 14  50.0%+D+3n 2.5V  Motor BEMF-Voltage (unfiltered) (MMP231017-1) */
                l_au16AdcSource[6] = u16PhT.u16;                                /* 15  50.0%+D+4n 2.5V  Motor BEMF-Voltage (unfiltered) (MMP231017-1) */
#elif (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UT_VW)
                ADC_SDATA_t u16PhV = C_ADC_VPHV_EOC;
                ADC_SDATA_t u16PhW = C_ADC_VPHW_EOC;

                l_au16AdcSource[4] = u16PhV.u16;                                /* 14  50.0%+D+3n 2.5V  Motor BEMF-Voltage (unfiltered) (MMP231017-1) */
                l_au16AdcSource[6] = u16PhW.u16;                                /* 15  50.0%+D+4n 2.5V  Motor BEMF-Voltage (unfiltered) (MMP231017-1) */
#else  /* (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_WT) || (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_TW) */
#error "ERROR: Bi-Polar Full-step not implemented"
#endif /* (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_WT) || (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_TW) */
            }
#endif /* C_MOTOR_COILS */
#endif /* (_SUPPORT_STALLDET_BRI != FALSE) */
        }

        /* Configure ADC */
        HAL_ADC_Setup(/*B_ADC_ADC_WIDTH |*/                                     /* Setup Motor Drive measurement */
            C_ADC_ASB_NEVER |                                                   /* Auto Standby: Never */
#if ((_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM_BIPOLAR)) || /* FOC-based, none single-coil */ \
    (_SUPPORT_PWM_MODE == TRIPLEPHASE_FULL_STEP_BEMF) || (_SUPPORT_PWM_MODE == BIPOLAR_FULL_STEP_BEMF) || (_SUPPORT_PWM_MODE == BIPOLAR_HALF_STEP_BEMF) || /* BEMF-based */ \
    (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM)                                      /* DC-motor */
            C_ADC_INT_SCHEME_EOF |                                              /* Message interrupt: End-of-Frame */
#elif (_DEBUG_IO_ADC != FALSE)
            C_ADC_INT_SCHEME_EOC |                                              /* Message interrupt: End-of-Frame */
#else  /* (_DEBUG_IO_ADC != FALSE) */
            C_ADC_INT_SCHEME_NOINT |                                            /* Message interrupt: No */
#endif /* (_DEBUG_IO_ADC != FALSE) */
            B_ADC_SATURATE |                                                    /* Saturation: Enabled */
            B_ADC_NO_INTERLEAVE |                                               /* Interleave: No */
            C_ADC_SOC_SOURCE_HARD_CTRIG |                                       /* Start Of Conversion (SOC) triggered by: Hardware (HARD_CTRIG) */
#if (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_HW)
            C_ADC_SOS_SOURCE_2ND_HARD_CTRIG);                                   /* Start Of Sequence (SOS) triggered: 2nd Hardware */
#else  /* (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_HW) */
            C_ADC_SOS_SOURCE_SOFT_TRIG);                                        /* Start Of Sequence (SOS) triggered: By Software */
#endif /* (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_HW) */
        HAL_ADC_ClearErrors();                                                  /* Prior to start, first clear any error flag and enable Triggers */

#if (_DEBUG_IO_ADC != FALSE) ||                                                 /* ADC debugging */ \
    ((_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM_BIPOLAR)) || /* FOC-based, none single-coil */ \
    (_SUPPORT_PWM_MODE == TRIPLEPHASE_FULL_STEP_BEMF) || (_SUPPORT_PWM_MODE == BIPOLAR_FULL_STEP_BEMF) || (_SUPPORT_PWM_MODE == BIPOLAR_HALF_STEP_BEMF) || /* BEMF-based */ \
    (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM)                                      /* DC-motor */
        if (u16IrqEna != FALSE)                                                 /* MMP230919-1 */
        {
#if (_DEBUG_IO_ADC != FALSE)
            HAL_ADC_SetPrioAndEnableIRQ(C_MLX16_ITC_PRIO2_ADC_PRIO6);
#else  /* (_DEBUG_IO_ADC != FALSE) */
            HAL_ADC_SetPrioAndEnableIRQ(C_MLX16_ITC_PRIO2_ADC_PRIO4);
#endif /* (_DEBUG_IO_ADC != FALSE) */
        }
#else  /* (_DEBUG_IO_ADC != FALSE) */
        (void)u16IrqEna;
#endif /* (_DEBUG_IO_ADC != FALSE) */
        HAL_ADC_Start();                                                        /* Start ADC */
#if (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW)
        DELAY_US(C_ADC_SETTLING_TIME);                                          /* Allow some delay for the ADC to load first SBASE channel selection */
        if ( (IO_ADC_STATUS & M_ADC_STATE) == C_ADC_STATE_MEM_TRANSFER)
        {
            HAL_ADC_Stop();                                                     /* Stop ADC (and restart) */
        }
        DELAY_US(C_ADC_SETTLING_TIME);                                          /* Setting time */
        HAL_ADC_SoftTrigger();                                                  /* Send software-trigger now */
#endif /* (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW) */

#if (_SUPPORT_STALLDET_BZC != FALSE)
        l_u8AdcMode = (uint8_t)C_ADC_MODE_ON_STEPPER;
#else  /* (_SUPPORT_STALLDET_BZC != FALSE) */
        l_u8AdcMode = (uint8_t)C_ADC_MODE_RUN_HW;                               /* MMP181123-2 */
#endif /* (_SUPPORT_STALLDET_BZC != FALSE) */
    }
#if (_SUPPORT_STALLDET_BRI != FALSE)
    else
    {
#if (C_MOTOR_COILS == 3U)
        if ( (u16FullStep == 0U) || (u16FullStep == 3U) )
        {
            ADC_SDATA_t u16PhU = C_ADC_VPHU_EOC;

            l_au16AdcSource[5] = u16PhU.u16;                                    /* 12  50.0%+D+3n 2.5V  Motor BEMF-Voltage (unfiltered) */
        }
        else if ( (u16FullStep == 1U) || (u16FullStep == 4U) )
        {
            ADC_SDATA_t u16PhV = C_ADC_VPHV_EOC;

            l_au16AdcSource[5] = u16PhV.u16;                                    /* 13  50.0%+D+3n 2.5V  Motor BEMF-Voltage (unfiltered) */
        }
        else /* if ( (u16FullStep == 2U) || (u16FullStep == 5U) ) */
        {
            ADC_SDATA_t u16PhW = C_ADC_VPHW_EOC;

            l_au16AdcSource[5] = u16PhW.u16;                                    /* 14  50.0%+D+3n 2.5V  Motor BEMF-Voltage (unfiltered) */
        }
#elif (C_MOTOR_COILS == 2U) && (C_MOTOR_PHASES == 4U)
#if (_SUPPORT_PWM_MODE == BIPOLAR_FULL_STEP_BEMF)
        if ( (u16FullStep == 0U) || (u16FullStep == 2U) )
#else  /* (_SUPPORT_PWM_MODE == BIPOLAR_FULL_STEP_BEMF) */
        if ( (u16HalfStep == 0U) || (u16HalfStep == 4U) )
#endif /* (_SUPPORT_PWM_MODE == BIPOLAR_FULL_STEP_BEMF) */
        {
#if (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_WT) || (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_TW)
            ADC_SDATA_t u16PhT = C_ADC_VPHT_EOC;
            ADC_SDATA_t u16PhW = C_ADC_VPHW_EOC;

            l_au16AdcSource[4] = u16PhW.u16;                                    /* 14  50.0%+D+3n 2.5V  Motor BEMF-Voltage (unfiltered) (MMP230802-1) */
            l_au16AdcSource[6] = u16PhT.u16;                                    /* 15  50.0%+D+4n 2.5V  Motor BEMF-Voltage (unfiltered) (MMP230802-1) */
#elif (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UW_VT)
            ADC_SDATA_t u16PhT = C_ADC_VPHT_EOC;
            ADC_SDATA_t u16PhV = C_ADC_VPHV_EOC;

            l_au16AdcSource[4] = u16PhV.u16;                                    /* 14  50.0%+D+3n 2.5V  Motor BEMF-Voltage (unfiltered) (MMP231017-1) */
            l_au16AdcSource[6] = u16PhT.u16;                                    /* 15  50.0%+D+4n 2.5V  Motor BEMF-Voltage (unfiltered) (MMP231017-1) */
#elif (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UT_VW)
            ADC_SDATA_t u16PhV = C_ADC_VPHV_EOC;
            ADC_SDATA_t u16PhW = C_ADC_VPHW_EOC;

            l_au16AdcSource[4] = u16PhV.u16;                                    /* 14  50.0%+D+3n 2.5V  Motor BEMF-Voltage (unfiltered) (MMP231017-1) */
            l_au16AdcSource[6] = u16PhW.u16;                                    /* 15  50.0%+D+4n 2.5V  Motor BEMF-Voltage (unfiltered) (MMP231017-1) */
#endif
        }
        else
        {
            ADC_SDATA_t u16PhU = C_ADC_VPHU_EOC;
#if (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_WT) || (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UV_TW)
            ADC_SDATA_t u16PhV = C_ADC_VPHV_EOC;

            l_au16AdcSource[6] = u16PhV.u16;                                    /* 13  50.0%+D+4n 2.5V  Motor BEMF-Voltage (unfiltered) (MMP230802-1) */
#elif (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UW_VT)
            ADC_SDATA_t u16PhW = C_ADC_VPHW_EOC;

            l_au16AdcSource[6] = u16PhW.u16;                                    /* 13  50.0%+D+4n 2.5V  Motor BEMF-Voltage (unfiltered) (MMP230802-1) */
#elif (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_UT_VW)
            ADC_SDATA_t u16PhT = C_ADC_VPHT_EOC;

            l_au16AdcSource[6] = u16PhT.u16;                                    /* 13  50.0%+D+4n 2.5V  Motor BEMF-Voltage (unfiltered) (MMP230802-1) */
#endif
            l_au16AdcSource[4] = u16PhU.u16;                                    /* 12  50.0%+D+3n 2.5V  Motor BEMF-Voltage (unfiltered) (MMP230802-1) */
        }
#endif
    }
    if (l_u16MotorBemf_IntegrationCount != 0U)
    {
        g_u16MotorBemfVoltage = p_DivU16_U32byU16(l_u32MotorBemfVoltage_Integration, l_u16MotorBemf_IntegrationCount);
    }
    else
    {
        g_u16MotorBemfVoltage = 0U;
    }
    l_u32MotorBemfVoltage_Integration = 0U;
    l_u16MotorBemf_IntegrationCount = 0U;
    l_u16MotorBemf_IgnoreCount = ((PWM_FREQ + (800U / 2)) / 800U);              /* 1s/800 = 1250us delay */
#endif /* (_SUPPORT_STALLDET_BRI != FALSE) */
} /* End of ADC_Start() */

#if (_SUPPORT_MOTION_DET == C_MOTION_DET_BEMF) || (_SUPPORT_MOTION_DET == C_MOTION_DET_SENSOR)
/*!*************************************************************************** *
 * ADC_MovementDetection
 * \brief   Start ADC for movement detection
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Start ADC by using hardware triggers
 *          ADC ISR priority: None (6, not used)
 * *************************************************************************** *
 * - Call Hierarchy: MovementDetectorInit()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 1 (HAL_ADC_StopSafe())
 * *************************************************************************** */
void ADC_MovementDetection(void)
{
#if (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_INDEPENDENT_GND) || \
    (_SUPPORT_PWM_MODE == BIPOLAR_TRIPHASE_TWOPWM_INDEPENDENT_GND)
    /** ADC SBASE for movement detection, on Phase 'U' and 'W'
     *                MASTER1            SLAVE1       SLAVE2                       SLAVE3       MASTER2
     * Triggers at  : 1/8 (12.5%)        2/6 (33.3%)  3/6+D (50.0%+D)              4/6 (66.7%)  0+D (0%+D)
     * Measurement  : VS,VDDA,BGD,VDDD   -            VPHU,VSMF,Tj,VPHV            -            VPHW
     *
     * Max. 10 ADC channels per PWM-period of 20kHz (4.5us/ADC-channel @ 4MHz ADC-Clock)
     */
    static const ADC_SDATA_t SBASE_MOTION[] = /*lint !e578 */                   /* ADC Automated measurements */
    {                                                                           /* Ch  Trigger    Vref  Description */
        {.u16 = (uint16_t)&l_AdcResult.u16AdcVs},
        C_ADC_VS_MSTR1_CMP,
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE)
        C_ADC_VDDA_EOC,
#if (_SUPPORT_ADC_BGD != FALSE)
        C_ADC_VBGD_EOC,
        C_ADC_VAUX_EOC,
#endif /* (_SUPPORT_ADC_BGD != FALSE) */
        C_ADC_VDDD_EOC,
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_U_HV,
                .u3AdcVref = C_ADC_VREF_HV,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                .u1AdcReserved = 0U
            }
        },
        C_ADC_VSMF_EOC,
        C_ADC_TEMP_EOC,
        C_ADC_VPHV_EOC,
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
        C_ADC_RESOLVER_IOX_LV_EOC,
        C_ADC_RESOLVER_IOY_LV_EOC,
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_W_HV,
                .u3AdcVref = C_ADC_VREF_HV,
#if (_SUPPORT_ADC_PWM_DELAY != FALSE)
                .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR2_CMP,
#else  /* (_SUPPORT_ADC_PWM_DELAY != FALSE) */
                .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,
#endif /* (_SUPPORT_ADC_PWM_DELAY != FALSE) */
                .u1AdcReserved = 0U
            }
        },
        {.u16 = C_ADC_EOF},                                                     /* End-of-Frame */
        {.u16 = (uint16_t)&l_au16AdcSource[0]}                                  /* Repeat forever */
    };
#else
#error "ERROR: _SUPPORT_PWM_MODE not supported."
#endif

    if (l_u8AdcMode != (uint8_t)C_ADC_MODE_MOV_DET)
    {
        HAL_ADC_StopSafe();

        /* Set SBASE structure in RAM, to prevent MCU performance losses due to ADC DMA accesses from Flash */
        {
            uint16_t *pu16Src = &l_au16AdcSource[0];
            ADC_SDATA_t *pu16SBase = &SBASE_MOTION[0];
            do
            {
                *pu16Src = pu16SBase->u16;
                pu16Src++;
                pu16SBase++;
            } while (pu16SBase < (ADC_SDATA_t *)&SBASE_MOTION[sizeof(SBASE_MOTION) / sizeof(uint16_t)]);
        }

        HAL_ADC_Setup(/*B_ADC_ADC_WIDTH |*/                                     /* Setup Movement Detection measurement */
            C_ADC_ASB_NEVER |                                                   /* Auto Standby: Never */
            C_ADC_INT_SCHEME_EOF |                                              /* Message interrupt: End-of-Frame */
            B_ADC_SATURATE |                                                    /* Saturation: Enabled */
            B_ADC_NO_INTERLEAVE |                                               /* Interleave: No */
            C_ADC_SOC_SOURCE_HARD_CTRIG |                                       /* Start Of Conversion (SOC) triggered by: Hardware (HARD_CTRIG) */
#if (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_HW)
            C_ADC_SOS_SOURCE_2ND_HARD_CTRIG);                                   /* Start Of Sequence (SOS) triggered: 2nd Hardware */
#else  /* (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_HW) */
            C_ADC_SOS_SOURCE_SOFT_TRIG);                                        /* Start Of Sequence (SOS) triggered: By Software */
#endif /* (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_HW) */
        HAL_ADC_ClearErrors();                                                  /* Prior to start, first clear any error flag and enable Triggers */
        HAL_ADC_EnableIRQ();                                                    /* Enable ADC Interrupt */
        HAL_ADC_Start();                                                        /* Start ADC */
        l_u8AdcMode = (uint8_t)C_ADC_MODE_MOV_DET;
    }
} /* End of ADC_MovementDetection() */
#endif /* (_SUPPORT_MOTION_DET == C_MOTION_DET_BEMF) || (_SUPPORT_MOTION_DET == C_MOTION_DET_SENSOR) */

#if (_SUPPORT_STALLDET_BZC != FALSE)
/*!*************************************************************************** *
 * ADC_BEMF_Start
 * \brief   Start ADC (with waiting for pending ADC conversions to be finished)
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16Idx
 * \return  -
 * *************************************************************************** *
 * \details Start ADC by using hardware triggers
 *          ADC ISR priority: 4
 * *************************************************************************** *
 * - Call Hierarchy: ISR_CTIMER0_3()
 * - Cyclomatic Complexity: 6+1
 * - Nesting: 3
 * - Function calling: 1 (HAL_ADC_StopSafe())
 * *************************************************************************** */
void ADC_BEMF_Start(uint16_t u16Idx)
{
    static const ADC_SDATA_t SBASE_INIT_BEMF_U[] =                              /* ADC Automated measurements */
    {                                                                           /* Ch  Trigger    Vref  Description */
        {.u16 = (uint16_t)&l_AdcMotorRunBemf.u16AdcVs},
        C_ADC_VS_MSTR1_CMP,
        C_ADC_VSMF_EOC,
#if (_SUPPORT_BEMF_STARPOINT != FALSE)
        C_ADC_IO0_HV_EOC,
#endif /* (_SUPPORT_BEMF_STARPOINT != FALSE) */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_U_HV,
                .u3AdcVref = C_ADC_VREF_HV,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                .u1AdcReserved = 0U
            }
        },
        C_ADC_MCUR_EOC,
        C_ADC_TEMP_EOC,
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE)
        C_ADC_VDDA_EOC,
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_U_HV,
                .u3AdcVref = C_ADC_VREF_HV,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,
                .u1AdcReserved = 0U
            }
        },
        {.u16 = C_ADC_EOF},                                                     /* End-of-table marker */
        {.u16 = (uint16_t)&l_au16AdcSource[0]}                                  /* Repeat forever */
    };
    static const ADC_SDATA_t SBASE_INIT_BEMF_V[] =                              /* ADC Automated measurements */
    {                                                                           /* Ch  Trigger    Vref  Description */
        {.u16 = (uint16_t)&l_AdcMotorRunBemf.u16AdcVs},
        C_ADC_VS_MSTR1_CMP,
        C_ADC_VSMF_EOC,
#if (_SUPPORT_BEMF_STARPOINT != FALSE)
        C_ADC_IO0_HV_EOC,
#endif /* (_SUPPORT_BEMF_STARPOINT != FALSE) */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_V_HV,
                .u3AdcVref = C_ADC_VREF_HV,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                .u1AdcReserved = 0U
            }
        },
        C_ADC_MCUR_EOC,
        C_ADC_TEMP_EOC,
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE)
        C_ADC_VDDA_EOC,
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_V_HV,
                .u3AdcVref = C_ADC_VREF_HV,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,
                .u1AdcReserved = 0U
            }
        },
        {.u16 = C_ADC_EOF},                                                     /* End-of-table marker */
        {.u16 = (uint16_t)&l_au16AdcSource[0]}                                  /* Repeat forever */
    };
    static const ADC_SDATA_t SBASE_INIT_BEMF_W[] =                              /* ADC Automated measurements */
    {                                                                           /* Ch  Trigger    Vref  Description */
        {.u16 = (uint16_t)&l_AdcMotorRunBemf.u16AdcVs},
        C_ADC_VS_MSTR1_CMP,
        C_ADC_VSMF_EOC,
#if (_SUPPORT_BEMF_STARPOINT != FALSE)
        C_ADC_IO0_HV_EOC,
#endif /* (_SUPPORT_BEMF_STARPOINT != FALSE) */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_W_HV,
                .u3AdcVref = C_ADC_VREF_HV,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP,
                .u1AdcReserved = 0U
            }
        },
        C_ADC_MCUR_EOC,
        C_ADC_TEMP_EOC,
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE)
        C_ADC_VDDA_EOC,
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_W_HV,
                .u3AdcVref = C_ADC_VREF_HV,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CNT,
                .u1AdcReserved = 0U
            }
        },
        {.u16 = C_ADC_EOF},                                                     /* End-of-table marker */
        {.u16 = (uint16_t)&l_au16AdcSource[0]}                                  /* Repeat forever */
    };

    l_u16FullStepIndex = u16Idx;
    if (l_u8AdcMode != (uint8_t)C_ADC_MODE_ON_BEMF)
    {
        /* Change ADC Mode; Stop it first */
        HAL_ADC_StopSafe();                                                     /* Clear the ADC control register */

#if (_SUPPORT_PWM_MODE == TRIPLEPHASE_ALLPWM_MIRROR)
        IO_PWM_SLAVE2_CMP = (((4UL * PWM_REG_PERIOD) + 4U) / 8U);               /* 50.0% of period (MMP230512-2) */
#endif /* (_SUPPORT_PWM_MODE == TRIPLEPHASE_ALLPWM_MIRROR) */

        /* Set SBASE structure in RAM, to prevent MCU performance losses due to ADC DMA accesses from Flash */
        {
            const ADC_SDATA_t *pu16SBase;
            if ( (u16Idx == 0U) || (u16Idx == 3U) )
            {
                pu16SBase = &SBASE_INIT_BEMF_U[0];
            }
            else if ( (u16Idx == 1U) || (u16Idx == 4U) )
            {
                pu16SBase = &SBASE_INIT_BEMF_V[0];
            }
            else /* if ( (u16Idx == 2U) || (u16Idx == 5U) ) */
            {
                pu16SBase = &SBASE_INIT_BEMF_W[0];
            }
            p_MemCpyU16(&l_au16AdcSource[0], (uint16_t *)pu16SBase,
                        (sizeof(SBASE_INIT_BEMF_U) / sizeof(uint16_t)));        /* MMP230512-1: Code size reduction */
        }
#if (_SUPPORT_BEMF_STARPOINT != FALSE)
        IO_PORT_IO_ENABLE = (IO_PORT_IO_ENABLE & ~B_PORT_IO_ENABLE_IO_ENABLE_0) |
                            B_PORT_IO_ENABLE_IO_DISREC_0;
        IO_PORT_IO_OUT_EN = (IO_PORT_IO_OUT_EN & ~(B_PORT_IO_OUT_EN_IO_LV_ENABLE_0 |
                                                   B_PORT_IO_OUT_EN_IO_HS_ENABLE_0));
#endif /* (_SUPPORT_BEMF_STARPOINT != FALSE) */

        /* Configure ADC */
        HAL_ADC_Setup(/*B_ADC_ADC_WIDTH |*/                                     /* Setup BEMF measurement */
            C_ADC_ASB_NEVER |                                                   /* Auto Standby: Never */
            C_ADC_INT_SCHEME_EOF |                                              /* Message interrupt: EOF */
            B_ADC_SATURATE |                                                    /* Saturation: Enabled */
            B_ADC_NO_INTERLEAVE |                                               /* Interleave: No */
            C_ADC_SOC_SOURCE_HARD_CTRIG |                                       /* Start Of Conversion (SOC) triggered by: Hardware (HARD_CTRIG) */
            C_ADC_SOS_SOURCE_2ND_HARD_CTRIG);                                   /* Start Of Sequence (SOS) triggered: 2nd Hardware */
        HAL_ADC_ClearErrors();                                                  /* Prior to start, first clear any error flag and enable Triggers */
        HAL_ADC_EnableIRQ();                                                    /* Enable ADC Interrupt */
        HAL_ADC_Start();                                                        /* Start ADC */
        l_u8AdcMode = (uint8_t)C_ADC_MODE_ON_BEMF;
    }
    else
    {
        const ADC_SDATA_t *pu16SBase;
        if ( (u16Idx == 0U) || (u16Idx == 3U) )
        {
            pu16SBase = &SBASE_INIT_BEMF_U[0];
        }
        else if ( (u16Idx == 1U) || (u16Idx == 4U) )
        {
            pu16SBase = &SBASE_INIT_BEMF_V[0];
        }
        else /* if ( (u16Idx == 2U) || (u16Idx == 5U) ) */
        {
            pu16SBase = &SBASE_INIT_BEMF_W[0];
        }
#if (_SUPPORT_BEMF_STARPOINT != FALSE)
        u16Idx = 4U;                                                            /* Set initial index; Skip: DBASE, VS, VSMF, VIO0HV */
#else  /* (_SUPPORT_BEMF_STARPOINT != FALSE) */
        u16Idx = 3U;                                                            /* Set initial index; Skip: DBASE, VS, VSMF */
#endif /* (_SUPPORT_BEMF_STARPOINT != FALSE) */
        l_au16AdcSource[u16Idx] = pu16SBase[u16Idx].u16;                        /* xx    66.7%  2.5V    Motor-driver Phase-voltage X-phase (divided by 21) */
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE)
        u16Idx += 4U;                                                           /* Update index, skip: MCUR, TEMP, VDDA */
#else  /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) */
        u16Idx += 3U;                                                           /* Update index, skip: MCUR, TEMP */
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) */
        l_au16AdcSource[u16Idx] = pu16SBase[u16Idx].u16;                        /* xx   100.0%  2.5V    Motor-driver Phase-voltage X-phase (divided by 21) */
    }

} /* End of ADC_BEMF_Start() */

/*!*************************************************************************** *
 * CalcCommutPeriod
 * \brief   Calculate Commutation time
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16CommutToZeroCrossTime
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: ADC_ISR()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 2
 * - Function calling: 0
 * *************************************************************************** */
static uint16_t CalcCommutPeriod(uint16_t u16CommutToZeroCrossTime)
{
    uint16_t u16Period;
    uint16_t u16HalfCommutTimerPeriod = (l_u16ZcTime + u16CommutToZeroCrossTime) >> 1;  /* Average over last commutation period and new commutation period */
    l_u16ZcTime = u16CommutToZeroCrossTime;                                     /* New ZC-Time */
    if (u16HalfCommutTimerPeriod < C_MIN_HCOMMUT_TIME)
    {
        u16HalfCommutTimerPeriod = C_MIN_HCOMMUT_TIME;
    }
    else if (u16HalfCommutTimerPeriod > C_MAX_HCOMMUT_TIME)
    {
        u16HalfCommutTimerPeriod = C_MAX_HCOMMUT_TIME;
    }

    u16Period = l_u16ZcTime + u16HalfCommutTimerPeriod;                         /* No lead-angle (30 + 30) */
#if (_SUPPORT_BEMF_LEADANGLE_COMPENSATE != FALSE)
    u16Period -= p_MulU32_U16byU16(u16HalfCommutTimerPeriod, NV_LEADANGLE_COMPENSATION) >> 4;
#endif /* (_SUPPORT_BEMF_LEADANGLE_COMPENSATE != FALSE) */
    return (u16Period);
} /* End of CalcCommutPeriod() */
#endif /* (_SUPPORT_STALLDET_BZC != FALSE) */

#if ((_SUPPORT_FOC_MODE == FOC_MODE_NONE) || (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR)) && \
    (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM)
/*!*************************************************************************** *
 * ADC_ISR
 * \brief   ADC Interrupt Service Routine
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Toggle IO at end-of-conversion (debug-mode)
 * *************************************************************************** *
 * - Call Hierarchy: IRQ (EOC)
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
__attribute__((interrupt)) void ADC_ISR(void)
{
#if (_DEBUG_IO_ADC != FALSE)
    DEBUG_TOG_IO_E();
#endif /* (_DEBUG_IO_ADC != FALSE) */

#if (_SUPPORT_STALLDET_BRI != FALSE)
    if (l_u16MotorBemf_IgnoreCount == 0U)
    {
#if (_DEBUG_BEMF_SENSE != FALSE)
        DEBUG_SET_IO_D();
#endif /* (_DEBUG_BEMF_SENSE != FALSE) */
        uint16_t u16BemfVoltage = p_Abs16(GetRawMotorBemfVoltage());
        l_u32MotorBemfVoltage_Integration += u16BemfVoltage;
        l_u16MotorBemf_IntegrationCount++;
#if (_DEBUG_MOTOR_CURRENT_FLT != FALSE) && (_DEBUG_BEMF_SENSE != FALSE)
/*      if ( g_u16DebugBufWrIdx < C_DEBUG_BUF_SZ ) */
        {
            uint8_t *pBfr = &g_au8DebugBuf[g_u16DebugBufWrIdx];
            pBfr[0] = (uint8_t)(u16BemfVoltage >> 2);
            g_u16DebugBufWrIdx += 1U;
            if (g_u16DebugBufWrIdx >= C_DEBUG_BUF_SZ)
            {
                g_u16DebugBufWrIdx = 0U;
            }
        }
#endif /* (_DEBUG_MOTOR_CURRENT_FLT != FALSE) && (_DEBUG_BEMF_SENSE != FALSE) */
#if (_DEBUG_BEMF_SENSE != FALSE)
        DEBUG_CLR_IO_D();
#endif /* (_DEBUG_BEMF_SENSE != FALSE) */
    }
    else
    {
        l_u16MotorBemf_IgnoreCount--;
    }
#endif /* (_SUPPORT_STALLDET_BRI != FALSE) */

#if (_SUPPORT_STALLDET_BZC != FALSE)
#if (_DEBUG_COMMUT_ISR != FALSE)
    DEBUG_SET_IO_C();
#endif /* (_DEBUG_COMMUT_ISR != FALSE) */
    if (l_u8AdcMode == (uint8_t)C_ADC_MODE_ON_BEMF)
    {
        /* Handle BEMF Zero-cross. */
        /* JE-Method: Measure BEMF at 25% of the period & 75% of the period and interpolate the ZC */
        /* Condition: Phase voltage must be between: GND+1V and VSM-1V */
        uint16_t u16CommutToZeroCrossTime = IO_CTIMER0_TCNT;                    /* Take a copy of the commutation timer value */
        if (g_e8ZcDetectorState == (uint8_t)ZC_RESET)
        {
            /* Ignore first ADC-IRQ after ZC_RESET */
            g_e8ZcDetectorState = (uint8_t)ZC_INIT;
        }
        else if ( (g_e8ZcDetectorState & (uint8_t)ZC_FOUND) == 0U)              /* Zero-cross not jet found */
        {
            uint16_t u16VsmVoltage = l_AdcMotorRunBemf.u16AdcVsmF;
#if (_SUPPORT_BEMF_STARPOINT != FALSE)
            uint16_t u16StarPointVoltage = l_AdcMotorRunBemf.u16AdcVio0HV;
#endif /* (_SUPPORT_BEMF_STARPOINT != FALSE) */
            uint16_t u16BemfVoltage = ((l_AdcMotorRunBemf.u16AdcVphA +
                                        l_AdcMotorRunBemf.u16AdcVphB) >> 1);
#if (_SUPPORT_BEMF_STARPOINT != FALSE)
            /* Use Star-point voltage as Zero-crossing reference voltage */
            if (l_u16ZeroCrossVoltage != 0U)
            {
                l_u16ZeroCrossVoltage = ((l_u16ZeroCrossVoltage + u16StarPointVoltage) >> 1);
            }
            else
            {
                l_u16ZeroCrossVoltage = u16StarPointVoltage;
            }
#else  /* (_SUPPORT_BEMF_STARPOINT != FALSE) */
            /* Use (VSMF / 2) voltage as Zero-crossing reference voltage */
            if (l_u16ZeroCrossVoltage != 0U)
            {
                l_u16ZeroCrossVoltage = ((l_u16ZeroCrossVoltage + (u16VsmVoltage >> 1)) >> 1);
            }
            else
            {
                l_u16ZeroCrossVoltage = (u16VsmVoltage >> 1);
            }
#endif /* (_SUPPORT_BEMF_STARPOINT != FALSE) */
            if ( ((l_u16FullStepIndex ^ g_e8MotorDirectionCCW) & 1U) == 0U)
            {
                /* Raising BEMF */
                if (u16BemfVoltage < (u16VsmVoltage - C_1VOLT) )
                {
                    /* BEMF-shape */
                    if ( (g_e8ZcDetectorState & (uint8_t)ZC_START) == 0U)
                    {
                        /* ZC detector has not started */
                        if (u16BemfVoltage < l_u16ZeroCrossVoltage)
                        {
                            /* Start ZC correctly */
                            g_e8ZcDetectorState = (uint8_t)ZC_START;
                        }
                        else
                        {
                            /* Start ZC, although BEMF is above ZC-level.
                             * Ignore first BEMF PWM-period and thread this PWM-period as a start */
                            g_e8ZcDetectorState = (uint8_t)(ZC_START | ZC_ERROR);
                        }
                    }
                    else
                    {
                        /* ZC Detector has started */
                        if (u16BemfVoltage > (l_u16ZeroCrossVoltage + C_100MV) )
                        {
                            /* ZC Found */
#if (_DEBUG_COMMUT_ISR != FALSE)
                            DEBUG_SET_IO_B();
#endif /* (_DEBUG_COMMUT_ISR != FALSE) */
                            IO_CTIMER0_TREGB = CalcCommutPeriod(u16CommutToZeroCrossTime);   /* Set new commutation period */
                            g_e8ZcDetectorState = (uint8_t)ZC_FOUND;
#if (_DEBUG_COMMUT_ISR != FALSE)
                            DEBUG_CLR_IO_B();
#endif /* (_DEBUG_COMMUT_ISR != FALSE) */
                        }
                    }
                }
                else
                {
                    /* Fly-back pulse */
                }
            }
            else
            {
                /* Falling BEMF */
                if (u16BemfVoltage > C_1VOLT)
                {
                    /* BEMF-shape */
                    if ( (g_e8ZcDetectorState & (uint8_t)ZC_START) == 0U)
                    {
                        /* ZC detector has not started */
                        if (u16BemfVoltage > l_u16ZeroCrossVoltage)
                        {
                            /* Start ZC correctly */
                            g_e8ZcDetectorState = (uint8_t)ZC_START;
                        }
                        else
                        {
                            /* Start ZC, although BEMF is below ZC-level.
                             * Ignore first BEMF PWM-period and thread this PWM-period as a start */
                            g_e8ZcDetectorState = (uint8_t)(ZC_START | ZC_ERROR);
                        }
                    }
                    else
                    {
                        /* ZC Detector has started */
                        if (u16BemfVoltage < (l_u16ZeroCrossVoltage - C_100MV) )
                        {
                            /* ZC Found */
#if (_DEBUG_COMMUT_ISR != FALSE)
                            DEBUG_SET_IO_B();
#endif /* (_DEBUG_COMMUT_ISR != FALSE) */
                            IO_CTIMER0_TREGB = CalcCommutPeriod(u16CommutToZeroCrossTime);   /* Set new commutation period */
                            g_e8ZcDetectorState = (uint8_t)ZC_FOUND;
#if (_DEBUG_COMMUT_ISR != FALSE)
                            DEBUG_CLR_IO_B();
#endif /* (_DEBUG_COMMUT_ISR != FALSE) */
                        }
                    }
                }
                else
                {
                    /* Fly-back pulse */
                }
            }

#if (_DEBUG_MOTOR_CURRENT_FLT != FALSE) && (_DEBUG_MOTOR_BEMF != FALSE)
            extern uint8_t g_au8DebugBuf[C_DEBUG_BUF_SZ];                       /* Raw (unfiltered) motor current measurement */
            extern uint16_t g_u16DebugBufWrIdx;                                 /* Motor current measurement index */
#if (_DEBUG_MCUR_CYCLIC == FALSE)
            if (g_u16DebugBufWrIdx < C_DEBUG_BUF_SZ)
#endif /* (_DEBUG_MCUR_CYCLIC == FALSE) */
            {
                uint8_t *pu8Ptr = &g_au8DebugBuf[g_u16DebugBufWrIdx];
                *pu8Ptr++ = (uint8_t)((u16BemfVoltage >> 1U) & 0xFFU);
                *pu8Ptr++ = (uint8_t)((l_u16ZeroCrossVoltage >> 1U) & 0xFFU);
                g_u16DebugBufWrIdx += 2U;
#if (_DEBUG_MCUR_CYCLIC != FALSE)
                if (g_u16DebugBufWrIdx >= C_DEBUG_BUF_SZ)
                {
                    g_u16DebugBufWrIdx = 0U;
                }
#endif /* (_DEBUG_MCUR_CYCLIC != FALSE) */
            }
#endif /* (_DEBUG_MOTOR_CURRENT_FLT != FALSE) && (_DEBUG_MOTOR_BEMF != FALSE) */
        }
    }
#if (_DEBUG_COMMUT_ISR != FALSE)
    DEBUG_CLR_IO_C();
#endif /* (_DEBUG_COMMUT_ISR != FALSE) */
#endif /* (_SUPPORT_STALLDET_BZC != FALSE) */

#if (_SUPPORT_MOTION_DET != C_MOTION_DET_NONE)
    if (l_u8AdcMode == (uint8_t)C_ADC_MODE_MOV_DET)
    {
        /* Movement Detector */
#if (_SUPPORT_MOTION_DET == C_MOTION_DET_BEMF)
        MotionDetectorBEMF();
#elif (_SUPPORT_MOTION_DET == C_MOTION_DET_SENSOR)
        MotionDetectorSensor();
#endif /* (_SUPPORT_MOTION_DET == C_MOTION_DET_SENSOR) */
        return;
    }
#endif /* (_SUPPORT_MOTION_DET != C_MOTION_DET_NONE) */
} /* End of ADC_ISR() */
#endif /* (_SUPPORT_FOC_MODE == FOC_MODE_NONE) && (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM) */

#if ((_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM_BIPOLAR)) && \
    (_SUPPORT_RAM_FUNC != FALSE)
uint16_t ADC_GetRawMotorDriverCurrent(uint16_t u16MicroStepIdx) __attribute__ ((section(".ramfunc"))) __attribute__ ((aligned(8)));
#elif ((_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM_BIPOLAR)) && \
    (_SUPPORT_OPTIMIZE_FOR_SPEED != FALSE)
uint16_t ADC_GetRawMotorDriverCurrent(uint16_t u16MicroStepIdx) __attribute__((aligned(8)));
#endif /* ((_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM_BIPOLAR)) && _SUPPORT_OPTIMIZE_FOR_SPEED */

#if (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM_BIPOLAR) && (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM) && \
    (_SUPPORT_APP_TYPE != C_APP_SOLENOID) && (_SUPPORT_PWM_MODE != BIPOLAR_FULL_STEP_BEMF) && (_SUPPORT_PWM_MODE != BIPOLAR_HALF_STEP_BEMF) && \
    ((_SUPPORT_FOC_MODE != FOC_MODE_NONE) || (_SUPPORT_STALLDET_LA != FALSE) || \
     ((_SUPPORT_PWM_MODE != BIPOLAR_PWM_SINGLE_INDEPENDENT_GND) && (_SUPPORT_PWM_MODE != BIPOLAR_FULL_STEP_BEMF) && (_SUPPORT_PWM_MODE != BIPOLAR_HALF_STEP_BEMF)))
/*!*************************************************************************** *
 * CalcMotorCurrentAngleAndPeak
 * \brief   Calculate the Motor Current Angle and Peak
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details [in] g_i16MotorCurrentCoilA : (BLDC/Stepper) Motor Current Coil 'A'
 *          [in] g_i16MotorCurrentCoilY : (BLDC) Motor Current Coil 'B' & 'C', (Stepper) Motor Current Coil 'B'
 *          [out] g_i16MotorCurrentAngle: Motor Current Vector angle (Clarke).
 *          [out] g_u16MotorCurrentPeak : Motor Current Vector amplitude (Clarke).
 * *************************************************************************** *
 * - Call Hierarchy: GetRawMotorCurrent_2PWM()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 1 (p_atan2I16())
 * *************************************************************************** */
static inline void CalcMotorCurrentAngleAndPeak(void)
{
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) || (_SUPPORT_STALLDET_LA != FALSE)
    /* Motor-current Angle; 2^16 = 360 deg or 2pi */
#if (_SUPPORT_FAST_ATAN2 != FALSE)
    g_i16MotorCurrentAngle = (int16_t)p_atan2I16(g_i16MotorCurrentCoilA, g_i16MotorCurrentCoilY);
#else  /* (_SUPPORT_FAST_ATAN2 != FALSE) */
    g_i16MotorCurrentAngle = (int16_t)atan2I16(g_i16MotorCurrentCoilA, g_i16MotorCurrentCoilY);
#endif /* (_SUPPORT_FAST_ATAN2 != FALSE) */
#if (_SUPPORT_BEMF_SINUSOIDAL == FALSE)
    /* ROM-variant: g_u16MotorCurrentPeak = fm_HypotenusePythagoreanNonInlined( g_i16MotorCurrentCoilA, g_i16MotorCurrentCoilY); */
    g_u16MotorCurrentPeak = p_AproxSqrtU16_I16byI16(g_i16MotorCurrentCoilA, g_i16MotorCurrentCoilY);
#else  /* (_SUPPORT_BEMF_SINUSOIDAL == FALSE) */
#if (_SUPPORT_SINCOS_TABLE_SZ == 256)
    uint16_t u16Idx = ((uint16_t)g_i16MotorCurrentAngle / 256);
    int16_t *pi16SinCos = (int16_t *) &c_ai16MicroStepVector3PH_SinCos256[u16Idx];
#elif (_SUPPORT_SINCOS_TABLE_SZ == 192)
    uint16_t u16Idx = (uint16_t)p_MulU16hi_U16byU16((uint16_t)g_i16MotorCurrentAngle, l_u16MotorMicroStepsPerElecRotation);
    int16_t *pi16SinCos = (int16_t *) &c_ai16MicroStepVector3PH_SinCos192[u16Idx];
#elif (_SUPPORT_SINCOS_TABLE_SZ == 1024)
    int16_t *pi16SinCos = p_SinDirectLutInlined_2_5k(g_i16MotorCurrentAngle);
#endif /* _SUPPORT_SINCOS_TABLE_SZ */
    g_u16MotorCurrentPeak = p_MulI16_I16bypQ15( g_i16MotorCurrentCoilA, pi16SinCos) +
                            p_MulI16_I16bypQ15( g_i16MotorCurrentCoilY, (pi16SinCos + C_COS_OFFSET));
#endif /* (_SUPPORT_BEMF_SINUSOIDAL == FALSE) */
#else  /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) || (_SUPPORT_STALLDET_LA != FALSE) */
    /* Current angle is unknown, use SQRT */
    g_u16MotorCurrentPeak = p_AproxSqrtU16_I16byI16(g_i16MotorCurrentCoilA, g_i16MotorCurrentCoilY);
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) || (_SUPPORT_STALLDET_LA != FALSE) */
} /* End of CalcMotorCurrentAngleAndPeak() */
#endif /* (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM_BIPOLAR) && (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM) && (_SUPPORT_APP_TYPE != C_APP_SOLENOID) && (_SUPPORT_PWM_MODE != BIPOLAR_FULL_STEP_BEMF) && (_SUPPORT_PWM_MODE != BIPOLAR_HALF_STEP_BEMF) && ((_SUPPORT_FOC_MODE != FOC_MODE_NONE) || (_SUPPORT_STALLDET_LA != FALSE) || ((_SUPPORT_PWM_MODE != BIPOLAR_PWM_SINGLE_INDEPENDENT_GND) && (_SUPPORT_PWM_MODE != BIPOLAR_FULL_STEP_BEMF))) */

#if (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
/*!*************************************************************************** *
 * ADC_GetRawMotorDriverCurrent()
 * \brief   Get Motor Driver Current [ADC-LSB]
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (uint16_t) Motor Driver current in ADC-LSB
 * *************************************************************************** *
 * \details Get the Motor Driver current in ADC-LSB
 * *************************************************************************** *
 * - Call Hierarchy: ADC_ConvMotorDriverCurrent(), MotorDriverCurrentMeasurement()
 * - Cyclomatic Complexity: 3+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
uint16_t ADC_GetRawMotorDriverCurrent(void)
{
    uint16_t u16Current = l_AdcResult.u16AdcCurr;
    if (u16Current > l_u16CurrentZeroOffset)
    {
        g_u16MotorCurrentPeak = u16Current - Get_CurrentZeroOffset();
    }
    else
    {
        g_u16MotorCurrentPeak = 0U;
    }
    return (g_u16MotorCurrentPeak);
} /* End of ADC_GetRawMotorDriverCurrent() */
#else  /* (_SUPPORT_APP_TYPE == C_APP_SOLENOID) */

#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM)
/*!*************************************************************************** *
 * ADC_GetRawMotorDriverCurrent
 * \brief   Get Motor Driver Current [ADC-LSB]
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16MicroStepIdx: Unused Micro-step index
 * \return  (uint16_t) Motor Driver current in ADC-LSB
 * *************************************************************************** *
 * \details Get the Motor Driver current in ADC-LSB
 * *************************************************************************** *
 * - Call Hierarchy: ADC_ConvMotorDriverCurrent(), MotorDriverCurrentMeasurement()
 * - Cyclomatic Complexity: 3+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
uint16_t ADC_GetRawMotorDriverCurrent(uint16_t u16MicroStepIdx)
{
    uint16_t u16Current = ((l_AdcResult.u16AdcCurrA_1 + l_AdcResult.u16AdcCurrB_1) +
                           (l_AdcResult.u16AdcCurrA_2 + l_AdcResult.u16AdcCurrB_2));  /* Copy the ADC current measurement not calibrated */
    uint16_t u16ZeroOffset = (4U * Get_CurrentZeroOffset());
    if (u16Current > u16ZeroOffset)
    {
        u16Current = u16Current - u16ZeroOffset;
    }
    else
    {
        u16Current = 0U;
    }
    g_u16MotorCurrentPeak = u16Current;
    (void)u16MicroStepIdx;
    return (u16Current);
} /* End of ADC_GetRawMotorDriverCurrent() */

#elif (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR)
/*!*************************************************************************** *
 * ADC_GetRawMotorDriverCurrent()
 * \brief   Get Motor Driver Current [ADC-LSB]
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16MicroStepIdx: Micro-step index
 * \return  (uint16_t) Motor Driver current in ADC-LSB
 * *************************************************************************** *
 * \details Get the Motor Driver current in ADC-LSB
 * *************************************************************************** *
 * - Call Hierarchy: ADC_ConvMotorDriverCurrent(), MotorDriverCurrentMeasurement()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
uint16_t ADC_GetRawMotorDriverCurrent(uint16_t u16MicroStepIdx)
{
    uint16_t u16Current = l_AdcResult.u16AdcCurr;
    (void)u16MicroStepIdx;

    if (u16Current > Get_CurrentZeroOffset() )
    {
        u16Current -= Get_CurrentZeroOffset();
    }
    else
    {
        u16Current = 0U;
    }
    return (u16Current);
} /* End of ADC_GetRawMotorDriverCurrent() */

#elif (_SUPPORT_PWM_MODE == BIPOLAR_FULL_STEP_BEMF) || (_SUPPORT_PWM_MODE == BIPOLAR_HALF_STEP_BEMF)
/*!*************************************************************************** *
 * ADC_GetRawMotorDriverCurrent
 * \brief   Get Motor Driver Current [ADC-LSB]
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16MicroStepIdx: Micro-step index
 * \return  (uint16_t) Motor Driver current in ADC-LSB
 * *************************************************************************** *
 * \details Get the Motor Driver current in ADC-LSB
 * *************************************************************************** *
 * - Call Hierarchy: ADC_ConvMotorDriverCurrent(), MotorDriverCurrentMeasurement()
 * - Cyclomatic Complexity: 3+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
uint16_t ADC_GetRawMotorDriverCurrent(uint16_t u16MicroStepIdx)
{
    (void)u16MicroStepIdx;
    g_i16MotorCurrentCoilA = (int16_t)(l_AdcResult.u16AdcCurrA - Get_CurrentZeroOffset());
    g_u16MotorCurrentPeak = g_i16MotorCurrentCoilA;
    return (g_u16MotorCurrentPeak);
} /* End of ADC_GetRawMotorDriverCurrent() */

#elif (_SUPPORT_PWM_MODE == BIPOLAR_TRIPHASE_TWOPWM_INDEPENDENT_GND)
/*!*************************************************************************** *
 * ADC_GetRawMotorDriverCurrent()
 * \brief   Get Motor Driver Current [ADC-LSB]
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16MicroStepIdx: Micro-step index
 * \return  (uint16_t) Motor Driver current in ADC-LSB
 * *************************************************************************** *
 * \details Get the Motor Driver current in ADC-LSB
 * *************************************************************************** *
 * - Call Hierarchy: ADC_ConvMotorDriverCurrent(), MotorDriverCurrentMeasurement()
 * - Cyclomatic Complexity: 3+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
uint16_t ADC_GetRawMotorDriverCurrent(uint16_t u16MicroStepIdx)
{
    register uint16_t u16Offset = Get_CurrentZeroOffset();
    if (u16MicroStepIdx >= ((9U * C_MICROSTEP_PER_FULLSTEP) / 4U) )
    {
        /* Step 18 .. 47 or 135 to 360 degrees */
        g_i16MotorCurrentCoilB = (l_AdcResult.u16AdcCurrB - u16Offset);
    }
    else
    {
        /* Step 0 .. 18 or 0 to 135 degrees */
        g_i16MotorCurrentCoilB = ((2U * u16Offset) - (l_AdcResult.u16AdcCurrB + l_AdcResult.u16AdcCurrA));
    }
    if (u16MicroStepIdx <= ((15U * C_MICROSTEP_PER_FULLSTEP) / 4U) )
    {
        /* Step 0 ... 30 or 0 to 225 degrees */
        g_i16MotorCurrentCoilA = (l_AdcResult.u16AdcCurrA - u16Offset);
    }
    else
    {
        /* Step 30 .. 47 or 225 to 360 degrees */
        g_i16MotorCurrentCoilA = ((2U * u16Offset) - (l_AdcResult.u16AdcCurrB + l_AdcResult.u16AdcCurrA));
    }
    CalcMotorCurrentAngleAndPeak();
    return (g_u16MotorCurrentPeak);
} /* End of ADC_GetRawMotorDriverCurrent() */

#elif (_SUPPORT_PWM_MODE == BIPOLAR_PWM_SINGLE_INDEPENDENT_GND)
/*!*************************************************************************** *
 * ADC_GetRawMotorDriverCurrent
 * \brief   Get Motor Driver Current [ADC-LSB]
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16MicroStepIdx: Micro-step index
 * \return  (uint16_t) Motor Driver current in ADC-LSB
 * *************************************************************************** *
 * \details Get the Motor Driver current in ADC-LSB
 * *************************************************************************** *
 * - Call Hierarchy: ADC_ConvMotorDriverCurrent(), MotorDriverCurrentMeasurement()
 * - Cyclomatic Complexity: 3+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
uint16_t ADC_GetRawMotorDriverCurrent(uint16_t u16MicroStepIdx)
{
#if (_SUPPORT_STALLDET_BZC != FALSE)
    u16Current = 0U;
#endif /* (_SUPPORT_STALLDET_BZC != FALSE) */

#if (_SUPPORT_FOC_MODE == FOC_MODE_NONE)
    (void)u16MicroStepIdx;
#endif /* (_SUPPORT_FOC_MODE == FOC_MODE_NONE) */
    g_i16MotorCurrentCoilA = (int16_t)(l_AdcResult.u16AdcCurrA - Get_CurrentZeroOffset());
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) || (_SUPPORT_STALLDET_LA != FALSE)
    if (u16MicroStepIdx >= (2U * C_MICROSTEP_PER_FULLSTEP) )
    {
        g_i16MotorCurrentCoilA = -g_i16MotorCurrentCoilA;
    }
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
#if (_SUPPORT_STALLDET_BZC != FALSE)
    if (g_i16MotorCurrentCoilA > 0)
    {
        u16Current = (uint16_t)g_i16MotorCurrentCoilA;
    }
#endif /* (_SUPPORT_STALLDET_BZC != FALSE) */
    g_i16MotorCurrentCoilB = (int16_t)(l_AdcResult.u16AdcCurrB - Get_CurrentZeroOffset());
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) || (_SUPPORT_STALLDET_LA != FALSE)
    if ( (u16MicroStepIdx >= C_MICROSTEP_PER_FULLSTEP) && (u16MicroStepIdx < (3U * C_MICROSTEP_PER_FULLSTEP)) )
    {
        g_i16MotorCurrentCoilB = -g_i16MotorCurrentCoilB;
    }
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
#if (_SUPPORT_STALLDET_BZC != FALSE)
    if (g_i16MotorCurrentCoilB > 0)
    {
        u16Current += (uint16_t)g_i16MotorCurrentCoilB;
    }
    g_u16MotorCurrentPeak = u16Current;
    return (g_u16MotorCurrentPeak);
#else  /* (_SUPPORT_STALLDET_BZC != FALSE) */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) || (_SUPPORT_STALLDET_LA != FALSE)     /* (MMP230824-1) */
    CalcMotorCurrentAngleAndPeak();
#else  /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) || (_SUPPORT_STALLDET_LA != FALSE) */
    g_u16MotorCurrentPeak = p_AproxSqrtU16_I16byI16(g_i16MotorCurrentCoilA, g_i16MotorCurrentCoilB);
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) || (_SUPPORT_STALLDET_LA != FALSE) */
    return (g_u16MotorCurrentPeak);
#endif /* (_SUPPORT_STALLDET_BZC != FALSE) */
} /* End of ADC_GetRawMotorDriverCurrent() */

#elif (_SUPPORT_PWM_MODE == BIPOLAR_TRIPHASE_ALLPWM_MIRROR)
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
/*!*************************************************************************** *
 * ADC_GetRawMotorDriverCurrent
 * \brief   Get Raw Motor current (2-PWM based)
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16MicroStepIdx: Micro-step index
 * \return  (uint16_t) g_u16MotorCurrentPeak
 * *************************************************************************** *
 * \details Get (raw) Motor Driver Current [ADC-LSB], based on 2 PWM's
 *          Two PWM's gives two coil-current; Re-construct third coil-current.
 *          The two coil-current's 'A' and 'B' depends on the micro-step-index and PWM-state.
 * *************************************************************************** *
 * - Call Hierarchy: GetRawMotorDriverCurrent()
 * - Cyclomatic Complexity: 6+1
 * - Nesting: 5
 * - Function calling: 1 (p_AproxSqrtU16_I16byI16())
 * *************************************************************************** */
uint16_t ADC_GetRawMotorDriverCurrent(uint16_t u16MicroStepIdx)
{
    register uint16_t u16Offset = Get_CurrentZeroOffset();

    if (u16MicroStepIdx < ((3U * C_MICROSTEP_PER_FULLSTEP) / 4U) )
    {
        /* Micro-step 0 .. 6 of 48 or 0 to 45 degrees */
        g_i16MotorCurrentCoilC = (u16Offset - l_AdcResult.u16AdcCurrA);
        g_i16MotorCurrentCoilB = (l_AdcResult.u16AdcCurrB - u16Offset);
        g_i16MotorCurrentCoilA = (0 - (g_i16MotorCurrentCoilB + g_i16MotorCurrentCoilC));
    }
    else if (u16MicroStepIdx < ((9U * C_MICROSTEP_PER_FULLSTEP) / 4U) )
    {
        /* Micro-step 6 .. 18 of 48 or 14 to 135 degrees */
        g_i16MotorCurrentCoilC = (u16Offset - l_AdcResult.u16AdcCurrA);
        g_i16MotorCurrentCoilA = (l_AdcResult.u16AdcCurrB - u16Offset);
        g_i16MotorCurrentCoilB = (0 - (g_i16MotorCurrentCoilA + g_i16MotorCurrentCoilC));
    }
    else if (u16MicroStepIdx < ((12U * C_MICROSTEP_PER_FULLSTEP) / 4U) )
    {
        /* Micro-step 18 .. 24 of 48 or 135 to 180 degrees */
        g_i16MotorCurrentCoilB = (u16Offset - l_AdcResult.u16AdcCurrA);
        g_i16MotorCurrentCoilA = (l_AdcResult.u16AdcCurrB - u16Offset);
        g_i16MotorCurrentCoilC = (0 - (g_i16MotorCurrentCoilA + g_i16MotorCurrentCoilB));
    }
    else if (u16MicroStepIdx < ((15U * C_MICROSTEP_PER_FULLSTEP) / 4U) )
    {
        /* Micro-step 24 .. 30 of 48 or 180 to 225 degrees */
        g_i16MotorCurrentCoilB = (u16Offset - l_AdcResult.u16AdcCurrA);
        g_i16MotorCurrentCoilC = (l_AdcResult.u16AdcCurrB - u16Offset);
        g_i16MotorCurrentCoilA = (0 - (g_i16MotorCurrentCoilB + g_i16MotorCurrentCoilC));
    }
    else if (u16MicroStepIdx < ((21U * C_MICROSTEP_PER_FULLSTEP) / 4U) )
    {
        /* Micro-step 30 .. 42 of 48 or 225 to 315 degrees */
        g_i16MotorCurrentCoilA = (u16Offset - l_AdcResult.u16AdcCurrA);
        g_i16MotorCurrentCoilC = (l_AdcResult.u16AdcCurrB - u16Offset);
        g_i16MotorCurrentCoilB = (0 - (g_i16MotorCurrentCoilA + g_i16MotorCurrentCoilC));
    }
    else
    {
        /* Micro-step 42 .. 48 of 48 or 315 to 360 degrees */
        g_i16MotorCurrentCoilA = (u16Offset - l_AdcResult.u16AdcCurrA);
        g_i16MotorCurrentCoilB = (l_AdcResult.u16AdcCurrB - u16Offset);
        g_i16MotorCurrentCoilC = (0 - (g_i16MotorCurrentCoilA + g_i16MotorCurrentCoilB));
    }
    g_u16MotorCurrentPeak = p_AproxSqrtU16_I16byI16(g_i16MotorCurrentCoilA, g_i16MotorCurrentCoilC);
    return (g_u16MotorCurrentPeak);
} /* End of ADC_GetRawMotorDriverCurrent() */

#else   /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
/*!*************************************************************************** *
 * ADC_GetRawMotorDriverCurrent()
 * \brief   Get Motor Driver Current [ADC-LSB]
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16MicroStepIdx: Micro-step index
 * \return  (uint16_t) Motor Driver current in ADC-LSB
 * *************************************************************************** *
 * \details Get the Motor Driver current in ADC-LSB
 * *************************************************************************** *
 * - Call Hierarchy: ADC_ConvMotorDriverCurrent(), MotorDriverCurrentMeasurement()
 * - Cyclomatic Complexity: 3+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
uint16_t ADC_GetRawMotorDriverCurrent(uint16_t u16MicroStepIdx)
{
    register uint16_t u16Offset = Get_CurrentZeroOffset();
    if (u16MicroStepIdx < ((3U * C_MICROSTEP_PER_FULLSTEP) / 4U) )
    {
        /* Micro-step 0 .. 6 of 48 or 0 to 45 degrees */
        g_i16MotorCurrentCoilC = (u16Offset - l_AdcResult.u16AdcCurrA);
        g_i16MotorCurrentCoilB = (l_AdcResult.u16AdcCurrB - u16Offset);
        g_i16MotorCurrentCoilA = (0 - (g_i16MotorCurrentCoilB + g_i16MotorCurrentCoilC));
    }
    else if (u16MicroStepIdx < ((9U * C_MICROSTEP_PER_FULLSTEP) / 4U) )
    {
        /* Micro-step 6 .. 18 of 48 or 14 to 135 degrees */
        g_i16MotorCurrentCoilC = (u16Offset - l_AdcResult.u16AdcCurrA);
        g_i16MotorCurrentCoilA = (l_AdcResult.u16AdcCurrB - u16Offset);
        g_i16MotorCurrentCoilB = (0 - (g_i16MotorCurrentCoilA + g_i16MotorCurrentCoilC));
    }
    else if (u16MicroStepIdx < ((12U * C_MICROSTEP_PER_FULLSTEP) / 4U) )
    {
        /* Micro-step 18 .. 24 of 48 or 135 to 180 degrees */
        g_i16MotorCurrentCoilB = (u16Offset - l_AdcResult.u16AdcCurrA);
        g_i16MotorCurrentCoilA = (l_AdcResult.u16AdcCurrB - u16Offset);
        g_i16MotorCurrentCoilC = (0 - (g_i16MotorCurrentCoilA + g_i16MotorCurrentCoilB));
    }
    else if (u16MicroStepIdx < ((15U * C_MICROSTEP_PER_FULLSTEP) / 4U) )
    {
        /* Micro-step 24 .. 30 of 48 or 180 to 225 degrees */
        g_i16MotorCurrentCoilB = (u16Offset - l_AdcResult.u16AdcCurrA);
        g_i16MotorCurrentCoilC = (l_AdcResult.u16AdcCurrB - u16Offset);
        g_i16MotorCurrentCoilA = (0 - (g_i16MotorCurrentCoilB + g_i16MotorCurrentCoilC));
    }
    else if (u16MicroStepIdx < ((21U * C_MICROSTEP_PER_FULLSTEP) / 4U) )
    {
        /* Micro-step 30 .. 42 of 48 or 225 to 315 degrees */
        g_i16MotorCurrentCoilA = (u16Offset - l_AdcResult.u16AdcCurrA);
        g_i16MotorCurrentCoilC = (l_AdcResult.u16AdcCurrB - u16Offset);
        g_i16MotorCurrentCoilB = (0 - (g_i16MotorCurrentCoilA + g_i16MotorCurrentCoilC));
    }
    else
    {
        /* Micro-step 42 .. 48 of 48 or 315 to 360 degrees */
        g_i16MotorCurrentCoilA = (u16Offset - l_AdcResult.u16AdcCurrA);
        g_i16MotorCurrentCoilB = (l_AdcResult.u16AdcCurrB - u16Offset);
        g_i16MotorCurrentCoilC = (0 - (g_i16MotorCurrentCoilA + g_i16MotorCurrentCoilB));
    }
    g_u16MotorCurrentPeak = p_AproxSqrtU16_I16byI16(g_i16MotorCurrentCoilA, g_i16MotorCurrentCoilC);
    return (g_u16MotorCurrentPeak);
} /* End of ADC_GetRawMotorDriverCurrent() */
#endif  /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */

#elif (_SUPPORT_PWM_MODE == TRIPLEPHASE_ALLPWM_MIRROR)
#if (_SUPPORT_STALLDET_BZC != FALSE)
/*!*************************************************************************** *
 * ADC_GetRawMotorDriverCurrent
 * \brief   Get Motor Driver Current [ADC-LSB]
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16MicroStepIdx: Micro-step index
 * \return  (uint16_t) Motor Driver current in ADC-LSB
 * *************************************************************************** *
 * \details Get the Motor Driver current in ADC-LSB
 * *************************************************************************** *
 * - Call Hierarchy: ADC_ConvMotorDriverCurrent(), MotorDriverCurrentMeasurement()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
uint16_t ADC_GetRawMotorDriverCurrent(uint16_t u16MicroStepIdx)
{
    uint16_t u16Current = 0U;
    if (l_u8AdcMode == (uint8_t)C_ADC_MODE_ON_BEMF)
    {
        u16Current = l_AdcMotorRunBemf.u16AdcCurr;
    }
    else
    {
        u16Current = l_AdcResult.u16AdcCurr;
    }
    if (u16Current > Get_CurrentZeroOffset() )
    {
        u16Current = u16Current - Get_CurrentZeroOffset();
    }
    else
    {
        u16Current = 0U;
    }
    g_u16MotorCurrentPeak = u16Current;
    (void)u16MicroStepIdx;
    return (u16Current);
} /* End of ADC_GetRawMotorDriverCurrent() */

#elif (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
/*!*************************************************************************** *
 * ADC_GetRawMotorDriverCurrent
 * \brief   Get Raw Motor current (2-PWM based)
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16MicroStepIdx: Micro-step index
 * \return  (uint16_t) g_u16MotorCurrentPeak
 * *************************************************************************** *
 * \details Get (raw) Motor Driver Current [ADC-LSB], based on 2 PWM's
 *          Two PWM's gives two coil-current; Re-construct third coil-current.
 *          The two coil-current's 'A' and 'B' depends on the micro-step-index and PWM-state.
 * *************************************************************************** *
 * - Call Hierarchy: GetRawMotorDriverCurrent()
 * - Cyclomatic Complexity: 6+1
 * - Nesting: 5
 * - Function calling: 1 (p_AproxSqrtU16_I16byI16())
 * *************************************************************************** */
uint16_t ADC_GetRawMotorDriverCurrent(uint16_t u16MicroStepIdx)
{
    register uint16_t u16Offset = Get_CurrentZeroOffset();

    uint16_t u16PhaseIdx = (u16MicroStepIdx + (C_MICROSTEP_PER_FULLSTEP / 2U)) &
                           ~(C_MICROSTEP_PER_FULLSTEP - 1U);
    if (u16PhaseIdx == (1U * C_MICROSTEP_PER_FULLSTEP) )
    {
        /* 0.5*FS ... 1.5*FS */
        g_i16MotorCurrentCoilA = (l_AdcResult.u16AdcCurrB - u16Offset);
        g_i16MotorCurrentCoilC = (u16Offset - l_AdcResult.u16AdcCurrA);
        g_i16MotorCurrentCoilB = (0 - (g_i16MotorCurrentCoilA + g_i16MotorCurrentCoilC));
    }
    else if (u16PhaseIdx == (2U * C_MICROSTEP_PER_FULLSTEP) )
    {
        /* 1.5*FS ... 2.5*FS */
        g_i16MotorCurrentCoilA = (l_AdcResult.u16AdcCurrB - u16Offset);
        g_i16MotorCurrentCoilB = (u16Offset - l_AdcResult.u16AdcCurrA);
        g_i16MotorCurrentCoilC = (0 - (g_i16MotorCurrentCoilA + g_i16MotorCurrentCoilB));
    }
    else if (u16PhaseIdx == (3U * C_MICROSTEP_PER_FULLSTEP) )
    {
        /* 2.5*FS ... 3.5*FS */
        g_i16MotorCurrentCoilC = (l_AdcResult.u16AdcCurrB - u16Offset);
        g_i16MotorCurrentCoilB = (u16Offset - l_AdcResult.u16AdcCurrA);
        g_i16MotorCurrentCoilA = (0 - (g_i16MotorCurrentCoilB + g_i16MotorCurrentCoilC));
    }
    else if (u16PhaseIdx == (4U * C_MICROSTEP_PER_FULLSTEP) )
    {
        /* 3.5*FS ... 4.5*FS */
        g_i16MotorCurrentCoilC = (l_AdcResult.u16AdcCurrB - u16Offset);
        g_i16MotorCurrentCoilA = (u16Offset - l_AdcResult.u16AdcCurrA);
        g_i16MotorCurrentCoilB = (0 - (g_i16MotorCurrentCoilA + g_i16MotorCurrentCoilC));
    }
    else if (u16PhaseIdx == (5U * C_MICROSTEP_PER_FULLSTEP) )
    {
        /* 4.5*FS ... 5.5*FS */
        g_i16MotorCurrentCoilB = (l_AdcResult.u16AdcCurrB - u16Offset);
        g_i16MotorCurrentCoilA = (u16Offset - l_AdcResult.u16AdcCurrA);
        g_i16MotorCurrentCoilC = (0 - (g_i16MotorCurrentCoilA + g_i16MotorCurrentCoilB));
    }
    else
    {
        /* 0.0*FS ... 0.5*FS and 5.5*FS ... 6.0*FS */
        g_i16MotorCurrentCoilB = (l_AdcResult.u16AdcCurrB - u16Offset);
        g_i16MotorCurrentCoilC = (u16Offset - l_AdcResult.u16AdcCurrA);
        g_i16MotorCurrentCoilA = (0 - (g_i16MotorCurrentCoilB + g_i16MotorCurrentCoilC));
    }
    g_i16MotorCurrentCoilY = p_MulI16_I16byQ15( (g_i16MotorCurrentCoilB - g_i16MotorCurrentCoilC), C_INV_SQRT3);
    CalcMotorCurrentAngleAndPeak();
    return (g_u16MotorCurrentPeak);
} /* End of ADC_GetRawMotorDriverCurrent() */

#else
/*!*************************************************************************** *
 * ADC_GetRawMotorDriverCurrent()
 * \brief   Get Motor Driver Current [ADC-LSB]
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16MicroStepIdx: Micro-step index
 * \return  (uint16_t) Motor Driver current in ADC-LSB
 * *************************************************************************** *
 * \details Get the Motor Driver current in ADC-LSB
 * *************************************************************************** *
 * - Call Hierarchy: ADC_ConvMotorDriverCurrent(), MotorDriverCurrentMeasurement()
 * - Cyclomatic Complexity: 3+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
uint16_t ADC_GetRawMotorDriverCurrent(uint16_t u16MicroStepIdx)
{
    register uint16_t u16Offset = Get_CurrentZeroOffset();
    uint16_t u16PhaseIdx = (u16MicroStepIdx + (C_MICROSTEP_PER_FULLSTEP / 2U)) &
                           ~(C_MICROSTEP_PER_FULLSTEP - 1U);
    if (u16PhaseIdx == (1U * C_MICROSTEP_PER_FULLSTEP) )
    {
        /* 0.5*FS ... 1.5*FS */
        g_i16MotorCurrentCoilA = (l_AdcResult.u16AdcCurrB - u16Offset);
        g_i16MotorCurrentCoilC = (u16Offset - l_AdcResult.u16AdcCurrA);
        g_i16MotorCurrentCoilB = (0 - (g_i16MotorCurrentCoilA + g_i16MotorCurrentCoilC));
    }
    else if (u16PhaseIdx == (2U * C_MICROSTEP_PER_FULLSTEP) )
    {
        /* 1.5*FS ... 2.5*FS */
        g_i16MotorCurrentCoilA = (l_AdcResult.u16AdcCurrB - u16Offset);
        g_i16MotorCurrentCoilB = (u16Offset - l_AdcResult.u16AdcCurrA);
        g_i16MotorCurrentCoilC = (0 - (g_i16MotorCurrentCoilA + g_i16MotorCurrentCoilB));
    }
    else if (u16PhaseIdx == (3U * C_MICROSTEP_PER_FULLSTEP) )
    {
        /* 2.5*FS ... 3.5*FS */
        g_i16MotorCurrentCoilC = (l_AdcResult.u16AdcCurrB - u16Offset);
        g_i16MotorCurrentCoilB = (u16Offset - l_AdcResult.u16AdcCurrA);
        g_i16MotorCurrentCoilA = (0 - (g_i16MotorCurrentCoilB + g_i16MotorCurrentCoilC));
    }
    else if (u16PhaseIdx == (4U * C_MICROSTEP_PER_FULLSTEP) )
    {
        /* 3.5*FS ... 4.5*FS */
        g_i16MotorCurrentCoilC = (l_AdcResult.u16AdcCurrB - u16Offset);
        g_i16MotorCurrentCoilA = (u16Offset - l_AdcResult.u16AdcCurrA);
        g_i16MotorCurrentCoilB = (0 - (g_i16MotorCurrentCoilA + g_i16MotorCurrentCoilC));
    }
    else if (u16PhaseIdx == (5U * C_MICROSTEP_PER_FULLSTEP) )
    {
        /* 4.5*FS ... 5.5*FS */
        g_i16MotorCurrentCoilB = (l_AdcResult.u16AdcCurrB - u16Offset);
        g_i16MotorCurrentCoilA = (u16Offset - l_AdcResult.u16AdcCurrA);
        g_i16MotorCurrentCoilC = (0 - (g_i16MotorCurrentCoilA + g_i16MotorCurrentCoilB));
    }
    else
    {
        /* 0.0*FS ... 0.5*FS and 5.5*FS ... 6.0*FS */
        g_i16MotorCurrentCoilB = (l_AdcResult.u16AdcCurrB - u16Offset);
        g_i16MotorCurrentCoilC = (u16Offset - l_AdcResult.u16AdcCurrA);
        g_i16MotorCurrentCoilA = (0 - (g_i16MotorCurrentCoilB + g_i16MotorCurrentCoilC));
    }
    g_i16MotorCurrentCoilY = p_MulI16_I16byQ15( (g_i16MotorCurrentCoilB - g_i16MotorCurrentCoilC), C_INV_SQRT3);
    CalcMotorCurrentAngleAndPeak();
    return (g_u16MotorCurrentPeak);
} /* End of ADC_GetRawMotorDriverCurrent() */
#endif

#elif (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_MIRROR_SUP)
/*!*************************************************************************** *
 * ADC_GetRawMotorDriverCurrent
 * \brief   Get Motor Driver Current [ADC-LSB]
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16MicroStepIdx: Micro-step index
 * \return  (uint16_t) Motor Driver current in ADC-LSB
 * *************************************************************************** *
 * \details Get the Motor Driver current in ADC-LSB
 * *************************************************************************** *
 * - Call Hierarchy: ADC_ConvMotorDriverCurrent(), MotorDriverCurrentMeasurement()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
uint16_t ADC_GetRawMotorDriverCurrent(uint16_t u16MicroStepIdx)
{
    uint16_t u16Current = 0U;
    if (l_u8AdcMode == (uint8_t)C_ADC_MODE_ON_BEMF)
    {
        u16Current = l_AdcMotorRunBemf.u16AdcCurr;
    }
    else
    {
        u16Current = l_AdcResult.u16AdcCurr;
    }
    if (u16Current > Get_CurrentZeroOffset() )
    {
        u16Current = u16Current - Get_CurrentZeroOffset();
    }
    else
    {
        u16Current = 0U;
    }
    g_u16MotorCurrentPeak = u16Current;
    (void)u16MicroStepIdx;
    return (u16Current);
} /* End of ADC_GetRawMotorDriverCurrent() */

#elif (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_MIRROR_GND)
/*!*************************************************************************** *
 * ADC_GetRawMotorDriverCurrent
 * \brief   Get Motor Driver Current [ADC-LSB]
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16MicroStepIdx: Micro-step index
 * \return  (uint16_t) Motor Driver current in ADC-LSB
 * *************************************************************************** *
 * \details Get the Motor Driver current in ADC-LSB
 * *************************************************************************** *
 * - Call Hierarchy: ADC_ConvMotorDriverCurrent(), MotorDriverCurrentMeasurement()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
uint16_t ADC_GetRawMotorDriverCurrent(uint16_t u16MicroStepIdx)
{
    uint16_t u16Current = 0U;
    if (l_u8AdcMode == (uint8_t)C_ADC_MODE_ON_BEMF)
    {
        u16Current = l_AdcMotorRunBemf.u16AdcCurr;
    }
    else
    {
        u16Current = l_AdcResult.u16AdcCurr;
    }
    if (u16Current > Get_CurrentZeroOffset() )
    {
        u16Current = u16Current - Get_CurrentZeroOffset();
    }
    else
    {
        u16Current = 0U;
    }
    g_u16MotorCurrentPeak = u16Current;
    (void)u16MicroStepIdx;
    return (u16Current);
} /* End of ADC_GetRawMotorDriverCurrent() */

#elif (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_INDEPENDENT_GND)
#if (_SUPPORT_VARIABLE_PWM != FALSE)
/*!*************************************************************************** *
 * GetRawMotorCurrent_3PWM
 * \brief   Get Raw Motor current (3-PWM based)
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Get the Motor Driver current in ADC-LSB (MMP220815-1)
 * *************************************************************************** *
 * - Call Hierarchy: GetRawMotorDriverCurrent()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
void GetRawMotorCurrent_3PWM(void)
{
/*    extern int16_t g_i16ZeroCurrentCoilA; */
/*    extern int16_t g_i16ZeroCurrentCoilB; */
/*    extern int16_t g_i16ZeroCurrentCoilC; */
    register uint16_t u16Offset = Get_CurrentZeroOffset();

    g_i16MotorCurrentCoilA = (int16_t)(l_AdcResult3.u16AdcCurrA - u16Offset);   /* + (g_i16ZeroCurrentCoilA >> 8))); */
    g_i16MotorCurrentCoilB = (int16_t)(l_AdcResult3.u16AdcCurrB - u16Offset);   /* + (g_i16ZeroCurrentCoilB >> 8))); */
    g_i16MotorCurrentCoilC = (int16_t)(l_AdcResult3.u16AdcCurrC - u16Offset);   /* + (g_i16ZeroCurrentCoilC >> 8))); */
    g_i16MotorCurrentCoilY = p_MulI16_I16byQ15( (g_i16MotorCurrentCoilB - g_i16MotorCurrentCoilC), C_INV_SQRT3);

    CalcMotorCurrentAngleAndPeak();
} /* End of GetRawMotorCurrent_3PWM() */
#endif /* (_SUPPORT_VARIABLE_PWM != FALSE) */

/*!*************************************************************************** *
 * ADC_GetRawMotorDriverCurrent()
 * \brief   Get Motor Driver Current [ADC-LSB]
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16MicroStepIdx: Micro-step index
 * \return  (uint16_t) Motor Driver current in ADC-LSB
 * *************************************************************************** *
 * \details Get the Motor Driver current in ADC-LSB
 * *************************************************************************** *
 * - Call Hierarchy: ADC_ConvMotorDriverCurrent(), MotorDriverCurrentMeasurement()
 * - Cyclomatic Complexity: 3+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
uint16_t ADC_GetRawMotorDriverCurrent(uint16_t u16MicroStepIdx)
{
#if (_SUPPORT_VARIABLE_PWM != FALSE)                                            /* MMP220815-1 */
    if (g_u8TriplePWM == FALSE)
#endif /* (_SUPPORT_VARIABLE_PWM != FALSE) */
    {
        register uint16_t u16Offset = Get_CurrentZeroOffset();
#if (_SUPPORT_VOLTAGE_SHAPE != VOLTAGE_SHAPE_DSVM_ALT) /* MMP210524-1 */
        int16_t i16CurrAB = (int16_t)(l_AdcResult.u16AdcCurrA - u16Offset);
        int16_t i16CurrBC = (int16_t)(l_AdcResult.u16AdcCurrB - u16Offset);
        g_i16MotorCurrentCoilB = i16CurrBC;
        g_i16MotorCurrentCoilC = i16CurrBC;
        i16CurrBC = (0 - (i16CurrAB + i16CurrBC));
        {
            uint16_t u16PhaseIdx = (u16MicroStepIdx + (C_MICROSTEP_PER_FULLSTEP / 2U)) &
                                   ~((2U * C_MICROSTEP_PER_FULLSTEP) - 1U);
            if (u16PhaseIdx == (4U * C_MICROSTEP_PER_FULLSTEP) )
            {
                /* Small U-phase low period --> U-phase to Low */
                g_i16MotorCurrentCoilA = i16CurrBC;
                g_i16MotorCurrentCoilB = i16CurrAB;
            }
            else
            {
                g_i16MotorCurrentCoilA = i16CurrAB;
                if (u16PhaseIdx == (2U * C_MICROSTEP_PER_FULLSTEP) )
                {
                    /* Small V-phase low period --> V-phase to Low */
                    g_i16MotorCurrentCoilB = i16CurrBC;
                }
                else
                {
                    /* Small W-phase low period --> W-phase to Low */
                    g_i16MotorCurrentCoilC = i16CurrBC;
                }
            }
        }
        g_i16MotorCurrentCoilY = p_MulI16_I16byQ15( (g_i16MotorCurrentCoilB - g_i16MotorCurrentCoilC), C_INV_SQRT3);
        CalcMotorCurrentAngleAndPeak();
#if (_APP_BURN_IN != FALSE)
        g_u16MotorCurrentPeak =
            p_MulDivU16_U16byU16byU16(g_u16MotorCurrentPeak, l_u16CorrectionRatio, (PWM_REG_PERIOD << 4));
#endif /* (_APP_BURN_IN != FALSE) */
#else  /* (_SUPPORT_VOLTAGE_SHAPE != VOLTAGE_SHAPE_DSVM_ALT) */ /* MMP210524-1 */
#if (_SUPPORT_OPTIMIZE_FOR_SPEED != FALSE)                                      /* MMP231018-2 */
        /* Fastest version (24.87 us; Size: 204)
         *                 (24.72 us; Size: 208) */
        int16_t i16MotorCurrentCoilB, i16MotorCurrentCoilC;
        switch (u16MicroStepIdx / C_MICROSTEP_PER_FULLSTEP)
        {
            /*case 5U:*/
            default:
                /* g_i16MotorCurrentCoilC = (u16Offset - l_AdcResult.u16AdcCurrB);
                 * g_i16MotorCurrentCoilA = (u16Offset - l_AdcResult.u16AdcCurrA);
                 * g_i16MotorCurrentCoilB = (0 - (g_i16MotorCurrentCoilA + g_i16MotorCurrentCoilC)); */
                __asm__ (
                    "lod A, X \n\t"                                             /* A = u16Offset */
                    "sub A, %w3 \n\t"                                           /* A = u16Offset - l_AdcResult.u16AdcCurrA */
                    "mov dp:_g_i16MotorCurrentCoilA, A \n\t"                    /* g_i16MotorCurrentCoilA = A */
                    "sub X, %w4 \n\t"                                           /* X = u16Offset - l_AdcResult.u16AdcCurrB = g_i16MotorCurrentCoilC */
                    "add A, X \n\t"                                             /* A = g_i16MotorCurrentCoilA + g_i16MotorCurrentCoilC */
                    "neg A \n\t"                                                /* A = (0 - (g_i16MotorCurrentCoilA + g_i16MotorCurrentCoilC)) = g_i16MotorCurrentCoilB */
                    : "=a" (i16MotorCurrentCoilB), "=x" (i16MotorCurrentCoilC)
                    : "x" (u16Offset), "" (l_AdcResult.u16AdcCurrA), "" (l_AdcResult.u16AdcCurrB)
                    );
                break;
            case 0U:
                /* g_i16MotorCurrentCoilA = (l_AdcResult.u16AdcCurrB - u16Offset);
                 * g_i16MotorCurrentCoilB = (l_AdcResult.u16AdcCurrA - u16Offset);
                 * g_i16MotorCurrentCoilC = (0 - (g_i16MotorCurrentCoilA + g_i16MotorCurrentCoilB)); */
                __asm__ (
                    "lod A, %w3 \n\t"                                           /* A = l_AdcResult.u16AdcCurrA */
                    "sub A, X \n\t"                                             /* A = l_AdcResult.u16AdcCurrA - u16Offset = g_i16MotorCurrentCoilB */
                    "neg X \n\t"                                                /* X = -u16Offset */
                    "add X, %w4 \n\t"                                           /* X = l_AdcResult.u16AdcCurrB - u16Offset */
                    "mov dp:_g_i16MotorCurrentCoilA, X \n\t"                    /* g_i16MotorCurrentCoilA = X */
                    "add X, A \n\t"                                             /* X = g_i16MotorCurrentCoilA + g_i16MotorCurrentCoilB */
                    "neg X \n\t"                                                /* X = (0 - (g_i16MotorCurrentCoilA + g_i16MotorCurrentCoilB)) = g_i16MotorCurrentCoilC */
                    : "=a" (i16MotorCurrentCoilB), "=x" (i16MotorCurrentCoilC)
                    : "x" (u16Offset), "" (l_AdcResult.u16AdcCurrA), "" (l_AdcResult.u16AdcCurrB)
                    );
                break;
            case 1U:
                /* g_i16MotorCurrentCoilB = (u16Offset - l_AdcResult.u16AdcCurrB);
                 * g_i16MotorCurrentCoilC = (u16Offset - l_AdcResult.u16AdcCurrA);
                 * g_i16MotorCurrentCoilA = (0 - (g_i16MotorCurrentCoilB + g_i16MotorCurrentCoilC)); */
                __asm__ (
                    "lod A, X \n\t"                                             /* A = u16Offset */
                    "sub A, %w4 \n\t"                                           /* A = u16Offset - l_AdcResult.u16AdcCurrB = g_i16MotorCurrentCoilB */
                    "sub X, %w3 \n\t"                                           /* X = u16Offset - l_AdcResult.u16AdcCurrB = g_i16MotorCurrentCoilC */
                    "mov Y, A \n\t"                                             /* Y = g_i16MotorCurrentCoilB */
                    "add Y, X \n\t"                                             /* Y = g_i16MotorCurrentCoilB + g_i16MotorCurrentCoilC */
                    "neg Y \n\t"                                                /* Y = (0 - (g_i16MotorCurrentCoilB + g_i16MotorCurrentCoilC)) */
                    "mov dp:_g_i16MotorCurrentCoilA, Y \n\t"                    /* g_i16MotorCurrentCoilA = Y */
                    : "=a" (i16MotorCurrentCoilB), "=x" (i16MotorCurrentCoilC)
                    : "x" (u16Offset), "" (l_AdcResult.u16AdcCurrA), "" (l_AdcResult.u16AdcCurrB)
                    : "Y"
                    );
                break;
            case 2U:
                /* g_i16MotorCurrentCoilC = (l_AdcResult.u16AdcCurrB - u16Offset);
                 * g_i16MotorCurrentCoilA = (l_AdcResult.u16AdcCurrA - u16Offset);
                 * g_i16MotorCurrentCoilB = (0 - (g_i16MotorCurrentCoilA + g_i16MotorCurrentCoilC)); */
                __asm__ (
                    "lod A, %w3 \n\t"                                           /* A = l_AdcResult.u16AdcCurrA */
                    "sub A, X \n\t"                                             /* A = l_AdcResult.u16AdcCurrA - u16Offset */
                    "mov dp:_g_i16MotorCurrentCoilA, A \n\t"                    /* g_i16MotorCurrentCoilA = A */
                    "neg X \n\t"                                                /* X = -u16Offset */
                    "add X, %w4 \n\t"                                           /* X = l_AdcResult.u16AdcCurrB - u16Offset = g_i16MotorCurrentCoilC */
                    "add A, X \n\t"                                             /* A = g_i16MotorCurrentCoilA + g_i16MotorCurrentCoilC */
                    "neg A \n\t"                                                /* A = (0 - (g_i16MotorCurrentCoilA + g_i16MotorCurrentCoilC)) = g_i16MotorCurrentCoilB */
                    : "=a" (i16MotorCurrentCoilB), "=x" (i16MotorCurrentCoilC)
                    : "x" (u16Offset), "" (l_AdcResult.u16AdcCurrA), "" (l_AdcResult.u16AdcCurrB)
                    );
                break;
            case 3U:
                /* g_i16MotorCurrentCoilA = (u16Offset - l_AdcResult.u16AdcCurrB);
                 * g_i16MotorCurrentCoilB = (u16Offset - l_AdcResult.u16AdcCurrA);
                 * g_i16MotorCurrentCoilC = (0 - (g_i16MotorCurrentCoilA + g_i16MotorCurrentCoilB)); */
                __asm__ (
                    "lod A, X \n\t"                                             /* A = u16Offset */
                    "sub A, %w3 \n\t"                                           /* A = u16Offset - l_AdcResult.u16AdcCurrA = g_i16MotorCurrentCoilB */
                    "sub X, %w4 \n\t"                                           /* X = u16Offset - l_AdcResult.u16AdcCurrB */
                    "mov dp:_g_i16MotorCurrentCoilA, X \n\t"                    /* g_i16MotorCurrentCoilA = X */
                    "add X, A \n\t"                                             /* X = g_i16MotorCurrentCoilA + g_i16MotorCurrentCoilB */
                    "neg X \n\t"                                                /* X = (0 - (g_i16MotorCurrentCoilA + g_i16MotorCurrentCoilB)) = g_i16MotorCurrentCoilC */
                    : "=a" (i16MotorCurrentCoilB), "=x" (i16MotorCurrentCoilC)
                    : "x" (u16Offset), "" (l_AdcResult.u16AdcCurrA), "" (l_AdcResult.u16AdcCurrB)
                    );
                break;
            case 4U:
                /* g_i16MotorCurrentCoilB = (l_AdcResult.u16AdcCurrB - u16Offset);
                 * g_i16MotorCurrentCoilC = (l_AdcResult.u16AdcCurrA - u16Offset);
                 * g_i16MotorCurrentCoilA = (0 - (g_i16MotorCurrentCoilB + g_i16MotorCurrentCoilC)); */
                __asm__ (
                    "lod A, %w4 \n\t"                                           /* A = l_AdcResult.u16AdcCurrB */
                    "sub A, X \n\t"                                             /* A = l_AdcResult.u16AdcCurrB - u16Offset = g_i16MotorCurrentCoilB */
                    "neg X \n\t"                                                /* X = -u16Offset */
                    "add X, %w3 \n\t"                                           /* X = l_AdcResult.u16AdcCurrA - u16Offset = g_i16MotorCurrentCoilC */
                    "mov Y, A \n\t"                                             /* Y = g_i16MotorCurrentCoilB */
                    "add Y, X \n\t"                                             /* Y = g_i16MotorCurrentCoilB + g_i16MotorCurrentCoilC */
                    "neg Y \n\t"                                                /* Y = (0 - (g_i16MotorCurrentCoilB + g_i16MotorCurrentCoilC)) */
                    "mov dp:_g_i16MotorCurrentCoilA, Y \n\t"                    /* g_i16MotorCurrentCoilA = Y */
                    : "=a" (i16MotorCurrentCoilB), "=x" (i16MotorCurrentCoilC)
                    : "x" (u16Offset), "" (l_AdcResult.u16AdcCurrA), "" (l_AdcResult.u16AdcCurrB)
                    : "Y"
                    );
                break;
        }
        g_i16MotorCurrentCoilB = i16MotorCurrentCoilB;
        g_i16MotorCurrentCoilC = i16MotorCurrentCoilC;
        g_i16MotorCurrentCoilY = p_MulI16_I16byQ15( (i16MotorCurrentCoilB - i16MotorCurrentCoilC), C_INV_SQRT3);
#elif TRUE
        /* Switch-case; Reduced code-size and faster (25.18 us; Size: 212)
         *                                           (24.97 us; Size: 214) */
        int16_t i16MotorCurrentCoilB, i16MotorCurrentCoilC;
        switch (u16MicroStepIdx / C_MICROSTEP_PER_FULLSTEP)
        {
            /*case 5U:*/
            default:
                i16MotorCurrentCoilC = (u16Offset - l_AdcResult.u16AdcCurrB);
                g_i16MotorCurrentCoilA = (u16Offset - l_AdcResult.u16AdcCurrA);
                i16MotorCurrentCoilB = (0 - (g_i16MotorCurrentCoilA + i16MotorCurrentCoilC));
                break;
            case 0U:
                g_i16MotorCurrentCoilA = (l_AdcResult.u16AdcCurrB - u16Offset);
                i16MotorCurrentCoilB = (l_AdcResult.u16AdcCurrA - u16Offset);
                i16MotorCurrentCoilC = (0 - (g_i16MotorCurrentCoilA + i16MotorCurrentCoilB));
                break;
            case 1U:
                i16MotorCurrentCoilB = (u16Offset - l_AdcResult.u16AdcCurrB);
                i16MotorCurrentCoilC = (u16Offset - l_AdcResult.u16AdcCurrA);
                g_i16MotorCurrentCoilA = (0 - (i16MotorCurrentCoilB + i16MotorCurrentCoilC));
                break;
            case 2U:
                i16MotorCurrentCoilC = (l_AdcResult.u16AdcCurrB - u16Offset);
                g_i16MotorCurrentCoilA = (l_AdcResult.u16AdcCurrA - u16Offset);
                i16MotorCurrentCoilB = (0 - (g_i16MotorCurrentCoilA + i16MotorCurrentCoilC));
                break;
            case 3U:
                g_i16MotorCurrentCoilA = (u16Offset - l_AdcResult.u16AdcCurrB);
                i16MotorCurrentCoilB = (u16Offset - l_AdcResult.u16AdcCurrA);
                i16MotorCurrentCoilC = (0 - (g_i16MotorCurrentCoilA + i16MotorCurrentCoilB));
                break;
            case 4U:
                i16MotorCurrentCoilB = (l_AdcResult.u16AdcCurrB - u16Offset);
                i16MotorCurrentCoilC = (l_AdcResult.u16AdcCurrA - u16Offset);
                g_i16MotorCurrentCoilA = (0 - (i16MotorCurrentCoilB + i16MotorCurrentCoilC));
                break;
        }
        g_i16MotorCurrentCoilB = i16MotorCurrentCoilB;
        g_i16MotorCurrentCoilC = i16MotorCurrentCoilC;
        g_i16MotorCurrentCoilY = p_MulI16_I16byQ15( (i16MotorCurrentCoilB - i16MotorCurrentCoilC), C_INV_SQRT3);
#else
        /* Original (25.43 us; Size: 230)
         *          (25.10 us; Size: 212) */
        int16_t i16MotorCurrentCoilB, i16MotorCurrentCoilC;
        uint16_t u16PhaseIdx = (u16MicroStepIdx / C_MICROSTEP_PER_FULLSTEP);
        if (u16PhaseIdx == 0)
        {
            g_i16MotorCurrentCoilA = (l_AdcResult.u16AdcCurrB - u16Offset);
            i16MotorCurrentCoilB = (l_AdcResult.u16AdcCurrA - u16Offset);
            i16MotorCurrentCoilC = (0 - (g_i16MotorCurrentCoilA + i16MotorCurrentCoilB));
        }
        else if (u16PhaseIdx == 1)
        {
            i16MotorCurrentCoilB = (u16Offset - l_AdcResult.u16AdcCurrB);
            i16MotorCurrentCoilC = (u16Offset - l_AdcResult.u16AdcCurrA);
            g_i16MotorCurrentCoilA = (0 - (i16MotorCurrentCoilB + i16MotorCurrentCoilC));
        }
        else if (u16PhaseIdx == 2)
        {
            i16MotorCurrentCoilC = (l_AdcResult.u16AdcCurrB - u16Offset);
            g_i16MotorCurrentCoilA = (l_AdcResult.u16AdcCurrA - u16Offset);
            i16MotorCurrentCoilB = (0 - (g_i16MotorCurrentCoilA + i16MotorCurrentCoilC));
        }
        else if (u16PhaseIdx == 3)
        {
            g_i16MotorCurrentCoilA = (u16Offset - l_AdcResult.u16AdcCurrB);
            i16MotorCurrentCoilB = (u16Offset - l_AdcResult.u16AdcCurrA);
            i16MotorCurrentCoilC = (0 - (g_i16MotorCurrentCoilA + i16MotorCurrentCoilB));
        }
        else if (u16PhaseIdx == 4)
        {
            i16MotorCurrentCoilC = (l_AdcResult.u16AdcCurrA - u16Offset);
            i16MotorCurrentCoilB = (l_AdcResult.u16AdcCurrB - u16Offset);
            g_i16MotorCurrentCoilA = (0 - (i16MotorCurrentCoilB + i16MotorCurrentCoilC));
        }
        else /* if ( u16PhaseIdx == 5 ) */
        {
            i16MotorCurrentCoilC = (u16Offset - l_AdcResult.u16AdcCurrB);
            g_i16MotorCurrentCoilA = (u16Offset - l_AdcResult.u16AdcCurrA);
            i16MotorCurrentCoilB = (0 - (g_i16MotorCurrentCoilA + i16MotorCurrentCoilC));
        }
        g_i16MotorCurrentCoilB = i16MotorCurrentCoilB;
        g_i16MotorCurrentCoilC = i16MotorCurrentCoilC;
        g_i16MotorCurrentCoilY = p_MulI16_I16byQ15( (i16MotorCurrentCoilB - i16MotorCurrentCoilC), C_INV_SQRT3);
#endif
        CalcMotorCurrentAngleAndPeak();
#endif /* (_SUPPORT_VOLTAGE_SHAPE != VOLTAGE_SHAPE_DSVM_ALT) */ /* MMP210524-1 */
    }
#if (_SUPPORT_VARIABLE_PWM != FALSE)                                            /* MMP220815-1 */
    else
    {
        /* Three phase currents; Correct for zero-current offset (per phase) */
        GetRawMotorCurrent_3PWM();
    }
#endif /* (_SUPPORT_VARIABLE_PWM != FALSE) */

    return (g_u16MotorCurrentPeak);
} /* End of ADC_GetRawMotorDriverCurrent() */

#elif (_SUPPORT_PWM_MODE == TRIPLEPHASE_FULL_STEP_BEMF)
/*!*************************************************************************** *
 * ADC_GetRawMotorDriverCurrent()
 * \brief   Get Motor Driver Current [ADC-LSB]
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16MicroStepIdx: Micro-step index
 * \return  (uint16_t) Motor Driver current in ADC-LSB
 * *************************************************************************** *
 * \details Get the Motor Driver current in ADC-LSB
 * *************************************************************************** *
 * - Call Hierarchy: ADC_ConvMotorDriverCurrent(), MotorDriverCurrentMeasurement()
 * - Cyclomatic Complexity: 3+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
uint16_t ADC_GetRawMotorDriverCurrent(uint16_t u16MicroStepIdx)
{
#if (_SUPPORT_STALLDET_BZC != FALSE)
    if (l_u8AdcMode == (uint8_t)C_ADC_MODE_ON_BEMF)
    {
        g_i16MotorCurrentCoilA = (l_AdcMotorRunBemf.u16AdcCurr - Get_CurrentZeroOffset());
    }
    else
#endif /* (_SUPPORT_STALLDET_BZC != FALSE) */
    {
        g_i16MotorCurrentCoilA = (l_AdcResult.u16AdcCurrA - Get_CurrentZeroOffset());
    }
    if (g_i16MotorCurrentCoilA > 0)
    {
        g_u16MotorCurrentPeak = (uint16_t)g_i16MotorCurrentCoilA;
    }
    else
    {
        g_u16MotorCurrentPeak = 0U;
    }
    (void)u16MicroStepIdx;
    return (g_u16MotorCurrentPeak);
} /* End of ADC_GetRawMotorDriverCurrent() */
#endif
#endif /* (_SUPPORT_APP_TYPE == C_APP_SOLENOID) */

#if (_SUPPORT_STALLDET_BRI != FALSE)
/*!*************************************************************************** *
 * ADC_GetRawMotorBemfVoltage
 * \brief   Get Motor BEMF Voltage [ADC-LSB]
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (int16_t) Motor BEMF Voltage in ADC-LSB
 * *************************************************************************** *
 * \details Get the Motor BEMF Voltage in ADC-LSB
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 3+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
int16_t GetRawMotorBemfVoltage(void)
{
#if (C_MOTOR_COILS == 3U)
    uint16_t u16MotorSupplyVoltage = Get_RawVmotorF();
    int16_t i16MotorBemfVoltage = (int16_t)l_AdcResult.u16AdcVbemf;
#if (_DEBUG_MOTOR_CURRENT_FLT != FALSE) && (_DEBUG_BEMF_SENSE != FALSE)
/*  if ( g_u16DebugBufWrIdx < C_DEBUG_BUF_SZ ) */
    {
        uint8_t *pBfr = &g_au8DebugBuf[g_u16DebugBufWrIdx];
        pBfr[0] = (uint8_t)(u16MotorSupplyVoltage >> 2);
        pBfr[1] = (uint8_t)(i16MotorBemfVoltage >> 2);
        g_u16DebugBufWrIdx += 2U;
        if (g_u16DebugBufWrIdx >= C_DEBUG_BUF_SZ)
        {
            g_u16DebugBufWrIdx = 0U;
        }
    }
#endif /* _DEBUG_MOTOR_CURRENT_FLT && (_DEBUG_BEMF_SENSE != FALSE) */
    i16MotorBemfVoltage = (i16MotorBemfVoltage - (u16MotorSupplyVoltage / 2U));
#elif (C_MOTOR_COILS == 2U) && (C_MOTOR_PHASES == 4U)
    int16_t i16MotorBemfVoltage = (int16_t)(l_AdcResult.u16AdcVbemfA - l_AdcResult.u16AdcVbemfB);
#endif /* C_MOTOR_COILS */
    return (i16MotorBemfVoltage);
} /* End of GetRawMotorBemfVoltage() */
#endif /* (_SUPPORT_STALLDET_BRI != FALSE) */

/*!*************************************************************************** *
 * ADC_MeasureVsupplyAndTemperature
 * \brief   Measure IC supply and Temperature with ADC
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Measure the IC supply, Junction Temperature
 * *************************************************************************** *
 * - Call Hierarchy: AppCurrentCheck(), HandleDiagnosticsOT(), HandleDiagnosticsUVOV(),
 *                   main_Init()
 * - Cyclomatic Complexity: 3+1
 * - Nesting: 3
 * - Function calling: 1 (HAL_ADC_StartSoftTrig())
 * *************************************************************************** */
void ADC_MeasureVsupplyAndTemperature(void)
{
    HAL_ADC_StopSafe();

    /* Copy (Flash) Table into RAM */
    {
        uint16_t *pu16Src = &l_au16AdcSource[0];
        uint16_t *pu16SBase = (uint16_t *)&SBASE_MOTOR_IDLE[0];
        *pu16Src++ = (uint16_t)&l_AdcResult;                                    /* DBASE */
        pu16Src = p_MemCpyU16(pu16Src, (uint16_t *)pu16SBase,
                              (sizeof(SBASE_MOTOR_IDLE) / sizeof(uint16_t)));   /* MMP230512-1: Code size reduction */
        *pu16Src = C_ADC_EOS;                                                   /* End-of-Sequence */
    }

    (void)HAL_ADC_StartSoftTrig(C_ADC_STATE_IDLE);
    if ( (IO_PORT_DRV_OUT & B_PORT_DRV_OUT_ENABLE_CSA) == 0U)                   /* MMP200818-1: Check CSA; Correct CurrA */
    {
        /* Motor Current Sense Amplifier is disabled; Motor Current measurement is incorrect */
#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) && (_SUPPORT_APP_TYPE != C_APP_SOLENOID)
        l_AdcResult.u16AdcCurrA_1 = Get_CurrentZeroOffset();                    /* Overwrite u16AdcCurrA_1 with Offset */
#elif (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) || (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_MIRROR_GND)
        l_AdcResult.u16AdcCurr = Get_CurrentZeroOffset();                       /* Overwrite DriverCurrent with Offset */
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
        l_AdcResult.u16AdcCurr = Get_CurrentZeroOffset();                       /* Overwrite u16AdcCurrA with Offset */
#elif (_SUPPORT_PWM_MODE == TRIPLEPHASE_ALLPWM_MIRROR) && (_SUPPORT_STALLDET_BZC != FALSE)
        l_AdcResult.u16AdcCurr = Get_CurrentZeroOffset();                       /* Overwrite u16AdcCurrA with Offset */
#else  /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) */
        l_AdcResult.u16AdcCurrA = Get_CurrentZeroOffset();                      /* Overwrite u16AdcCurrA with Offset */
#endif /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) */
    }
#if (_SUPPORT_APP_TYPE != C_APP_SOLENOID) && (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM_BIPOLAR) && \
    (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM) && (_SUPPORT_PWM_MODE != TRIPLEPHASE_TWOPWM_MIRROR_GND) && \
    (_SUPPORT_PWM_MODE != TRIPLEPHASE_FULL_STEP_BEMF) && (_SUPPORT_PWM_MODE != BIPOLAR_FULL_STEP_BEMF) && (_SUPPORT_PWM_MODE != BIPOLAR_HALF_STEP_BEMF) && \
    ((_SUPPORT_PWM_MODE != TRIPLEPHASE_ALLPWM_MIRROR) || (_SUPPORT_STALLDET_BZC == FALSE))
    l_AdcResult.u16AdcCurrB = Get_CurrentZeroOffset();                          /* MMP200818-1: This will clear the ADC-CRC value written in u16AdcCurrA */
#endif /* (_SUPPORT_APP_TYPE != C_APP_SOLENOID) && (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM_BIPOLAR) && (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM) && (_SUPPORT_PWM_MODE != TRIPLEPHASE_TWOPWM_MIRROR_GND) */
} /* End of ADC_MeasureVsupplyAndTemperature() */

/*!*************************************************************************** *
 * ADC_GetNewSampleVsupply
 * \brief   Measure IC supply
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (uint16_t) IC supply [ADC-LSB]
 * *************************************************************************** *
 * \details Measure the IC supply
 * *************************************************************************** *
 * - Call Hierarchy: NV_WriteBlock()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 1 (HAL_ADC_StartSoftTrig())
 * *************************************************************************** */
uint16_t ADC_GetNewSampleVsupply(void)
{
    uint16_t u16Vsupply;

    static const ADC_SDATA_t VS = C_ADC_VS_EOC;                                 /*  Chip supply voltage (divided by 21) */

    if (l_u8AdcMode == C_ADC_MODE_RUN_HW)
    {
        /* ADC is active; Just take a latest sample */
#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) && (_SUPPORT_APP_TYPE != C_APP_SOLENOID)
        u16Vsupply = l_AdcResult.u16AdcVs_1;
#else  /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) */
        u16Vsupply = l_AdcResult.u16AdcVs;
#endif /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) */
    }
    else
    {
        /* ADC is inactive; Sample VS */
        uint16_t *pu16Src = &l_au16AdcSource[0];
#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) && (_SUPPORT_APP_TYPE != C_APP_SOLENOID)
        *pu16Src++ = (uint16_t)&l_AdcResult.u16AdcVs_1;
#else  /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) */
        *pu16Src++ = (uint16_t)&l_AdcResult.u16AdcVs;
#endif /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) */
        *pu16Src++ = VS.u16;
        *pu16Src = C_ADC_EOS;                                                   /* End-of-Sequence */

        (void)HAL_ADC_StartSoftTrig(C_ADC_STATE_IDLE);
#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) && (_SUPPORT_APP_TYPE != C_APP_SOLENOID)
        u16Vsupply = l_AdcResult.u16AdcVs_1;
#else  /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) */
        u16Vsupply = l_AdcResult.u16AdcVs;
#endif /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) */
    }
    return (u16Vsupply);
} /* End of ADC_GetNewSampleVsupply() */

#if (ANA_COMM != FALSE)
/*!*************************************************************************** *
 * ADC_ReferencePosition
 * \brief   Get Reference (potentiometer) position
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (uint16_t) Potentiometer Position
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
uint16_t ADC_ReferencePosition(void)
{
    uint16_t u16NewRefPos = l_AdcResult.u16AdcRefPos;
    if ( (u16NewRefPos != C_ADC_MIN) && (u16NewRefPos != C_ADC_MAX) )
    {
        uint16_t *pu16Pos = &l_au16PosRefSamples[l_u16PosRefMovAgvIdx];
        uint16_t u16PrevPos = *pu16Pos;
        l_u16PosRefMovAgvIdx = (l_u16PosRefMovAgvIdx + 1U) & (C_POS_MOVAVG_SZ - 1U);
        l_u16PosRefMovAvgxN -= u16PrevPos;
        l_u16PosRefMovAvgxN += u16NewRefPos;
        *pu16Pos = u16NewRefPos;

        g_u16ReferencePotiPos = (l_u16PosRefMovAvgxN + (C_POS_MOVAVG_SZ / 2U)) >> C_POS_MOVAVG_SSZ;
    }

    return (g_u16ReferencePotiPos);
} /* End of ADC_ReferencePosition() */
#endif /* (ANA_COMM != FALSE) */

#if ((_SUPPORT_POTI != FALSE) && (_SUPPORT_POTI_MODE == C_POTI_MODE_CLOSED_LOOP)) || (ANA_COMM != FALSE)
/*!*************************************************************************** *
 * ADC_ActRefPositionInit
 * \brief   Actuator and/or reference position initialisation.
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 5+1
 * - Nesting: 3
 * - Function calling: 2 (HAL_ADC_StopSafe(), HAL_ADC_StartSoftTrig())
 * *************************************************************************** */
void ADC_ActRefPositionInit(void)
{
    static const ADC_SDATA_t SBASE_ADC_POS[] =                                  /* ADC Automated measurements */
    {                                                                           /* Ch  Trigger    Vref  Description */
        C_ADC_VS_EOC,
#if (_SUPPORT_POTI != FALSE)
        C_ADC_IO3_LV_EOC,
#endif /* (_SUPPORT_POTI != FALSE) */
#if (ANA_COMM != FALSE) && (C_ANA_IO == PIN_FUNC_IO_0)
        C_ADC_IO0_HV_EOC,
#endif /* (ANA_COMM != FALSE) && (C_ANA_IO == PIN_FUNC_IO_0) */
    };

    ADC_POS AdcPos;
#if (_SUPPORT_POTI != FALSE) && (ANA_COMM != FALSE)
    uint16_t u16Done = 0x0000U;
#endif /* (_SUPPORT_POTI != FALSE) && (ANA_COMM != FALSE) */

    HAL_ADC_StopSafe();

    {
        uint16_t *pu16Src = &l_au16AdcSource[0U];
        *pu16Src++ = (uint16_t)&AdcPos.u16AdcVs;
        *pu16Src++ = SBASE_ADC_POS[0].u16;
#if (_SUPPORT_POTI != FALSE)
        *pu16Src++ = SBASE_ADC_POS[1].u16;
#endif /* (_SUPPORT_POTI != FALSE) */
#if (ANA_COMM != FALSE) && (C_ANA_IO == PIN_FUNC_IO_0)
        *pu16Src++ = SBASE_ADC_POS[2].u16;
#endif /* (ANA_COMM != FALSE) && (C_ANA_IO == PIN_FUNC_IO_0) */
        *pu16Src = C_ADC_EOS;                                                   /* End-of-Sequence */
    }

    do
    {
        uint16_t u16NewPos;

        DELAY(C_DELAY_1MS);  /*lint !e522 */

        HAL_ADC_Setup( /*B_ADC_ADC_WIDTH |*/                                    /* Setup for Reference Position Measurement */
                       C_ADC_ASB_NEVER |                                        /* Auto Standby: Never */
#if (_DEBUG_IO_ADC != FALSE)
                       C_ADC_INT_SCHEME_EOC |                                   /* Message interrupt: End-of-Frame */
#else  /* (_DEBUG_IO_ADC != FALSE) */
                       C_ADC_INT_SCHEME_NOINT |                                 /* Message interrupt: No */
#endif /* (_DEBUG_IO_ADC != FALSE) */
                       B_ADC_SATURATE |                                         /* Saturation: Enabled */
                       B_ADC_NO_INTERLEAVE |                                    /* Interleave: No */
                       C_ADC_SOC_SOURCE_HARD_CTRIG |                            /* Start Of Conversion (SOC) triggered by: Hardware */
                       C_ADC_SOS_SOURCE_SOFT_TRIG);                             /* Start Of Sequence (SOS) triggered: Software */
        (void)HAL_ADC_StartSoftTrig(C_ADC_STATE_IDLE);

#if (_SUPPORT_POTI != FALSE)
#if (C_POTI_CCW == FALSE)
        u16NewPos = AdcPos.u16AdcPotiPos;
#else  /* (C_POTI_CCW == FALSE) */
        u16NewPos = (C_ADC_MAX - AdcPos.u16AdcPotiPos);
#endif /* (C_POTI_CCW == FALSE) */
        if ( (u16NewPos != C_ADC_MIN) && (u16NewPos != C_ADC_MAX) )
        {
            uint16_t *pu16Pos = &l_au16PosActSamples[l_u16PosActMovAgvIdx];
            l_u16PosActMovAgvIdx = (l_u16PosActMovAgvIdx + 1U) & (C_POS_MOVAVG_SZ - 1U);
#if (ANA_COMM != FALSE)
            if (l_u16PosActMovAgvIdx == 0U)
            {
                u16Done |= 0x0001U;
            }
#endif /* (ANA_COMM != FALSE) */
            l_u16PosActMovAvgxN += u16NewPos;
            *pu16Pos = u16NewPos;
        }
#endif /* (_SUPPORT_POTI != FALSE) */
#if (ANA_COMM != FALSE)
        u16NewPos = AdcPos.u16AdcRefPos;
        if ( (u16NewPos != C_ADC_MIN) && (u16NewPos != C_ADC_MAX) )
        {
            uint16_t *pu16Pos = &l_au16PosRefSamples[l_u16PosRefMovAgvIdx];
            l_u16PosRefMovAgvIdx = (l_u16PosRefMovAgvIdx + 1U) & (C_POS_MOVAVG_SZ - 1U);
#if (_SUPPORT_POTI != FALSE)
            if (l_u16PosRefMovAgvIdx == 0U)
            {
                u16Done |= 0x0002U;
            }
#endif /* (_SUPPORT_POTI != FALSE) */
            l_u16PosRefMovAvgxN += u16NewPos;
            *pu16Pos = u16NewPos;
        }
#endif /* (ANA_COMM != FALSE) */
#if (_SUPPORT_POTI != FALSE)
#if (ANA_COMM != FALSE)
    } while (u16Done != 0x0003U);
#else  /* (ANA_COMM != FALSE) */
    } while (l_u16PosActMovAgvIdx != 0U);
#endif /* (ANA_COMM != FALSE) */
#else  /* (_SUPPORT_POTI != FALSE) */
    } while (l_u16PosRefMovAgvIdx != 0U);
#endif /* (_SUPPORT_POTI != FALSE) */

#if (_SUPPORT_POTI != FALSE)
    g_u16ActualPotiPos = (l_u16PosActMovAvgxN + (C_POS_MOVAVG_SZ / 2U)) >> C_POS_MOVAVG_SSZ;
#endif /* (_SUPPORT_POTI != FALSE) */
#if (ANA_COMM != FALSE)
    g_u16ReferencePotiPos = (l_u16PosRefMovAvgxN + (C_POS_MOVAVG_SZ / 2U)) >> C_POS_MOVAVG_SSZ;
#endif /* (ANA_COMM != FALSE) */
} /* End of ADC_ActRefPositionInit() */
#endif /* (_SUPPORT_POTI != FALSE) || (ANA_COMM != FALSE) */

#if (_SUPPORT_POTI != FALSE)
/*!*************************************************************************** *
 * ADC_PotentiometerPosition
 * \brief   Get Potentiometer position
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (uint16_t) Potentiometer Position
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
uint16_t ADC_PotentiometerPosition(void)
{
#if (C_POTI_CCW == FALSE)
    uint16_t u16NewPos = l_AdcResult.u16AdcPotiPos;
#else  /* (C_POTI_CCW == FALSE) */
    uint16_t u16NewPos = (C_ADC_MAX - l_AdcResult.u16AdcPotiPos);
#endif /* (C_POTI_CCW == FALSE) */
    if ( (u16NewPos != C_ADC_MIN) && (u16NewPos != C_ADC_MAX) )
    {
        uint16_t *pu16Pos = &l_au16PosActSamples[l_u16PosActMovAgvIdx];
        uint16_t u16PrevPos = *pu16Pos;
        l_u16PosActMovAgvIdx = (l_u16PosActMovAgvIdx + 1U) & (C_POS_MOVAVG_SZ - 1U);
        l_u16PosActMovAvgxN -= u16PrevPos;
        l_u16PosActMovAvgxN += u16NewPos;
        *pu16Pos = u16NewPos;

        g_u16ActualPotiPos = (l_u16PosActMovAvgxN + (C_POS_MOVAVG_SZ / 2U)) >> C_POS_MOVAVG_SSZ;
    }
    return (g_u16ActualPotiPos);
} /* End of ADC_PotentiometerPosition() */
#endif /* (_SUPPORT_POTI != FALSE) */

#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
/*!*************************************************************************** *
 * MeasureResolverPos
 * \brief   Measure Resolver position via IO[0] (X) and IO[1] (Y)
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: -
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 2 (HAL_ADC_StopSafe, HAL_ADC_StartSoftTrig())
 * *************************************************************************** */
void MeasureResolverPos(void)
{
    static const ADC_SDATA_t SBASE_RESOLVER[4] =
    {
        {.u16 = (uint16_t)&l_AdcResult.u16Resolver_X},
        C_ADC_RESOLVER_IOX_LV_EOC,
        C_ADC_RESOLVER_IOY_LV_EOC,
        {.u16 = C_ADC_EOS}
    };

    HAL_ADC_StopSafe();

    {
        uint16_t *pu16Src = &l_au16AdcSource[0];
        ADC_SDATA_t *pu16SBase = (ADC_SDATA_t *)&SBASE_RESOLVER[0];
        do
        {
            *pu16Src = pu16SBase->u16;
            pu16Src++;
            pu16SBase++;
        } while (pu16SBase < (ADC_SDATA_t *)&SBASE_RESOLVER[sizeof(SBASE_RESOLVER) / sizeof(uint16_t)]);
    }

    (void)HAL_ADC_StartSoftTrig(C_ADC_STATE_IDLE);

} /* End of MeasureResolverPos() */

/* ************************************************************************** *
 * Function : GetResolverPosition                                             *
 * Purpose  : Get Resolver position (X, Y)                                    *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * \param   -
 * \return  (uint16_t) u16Result = 0: OK
 *                              <> 0: Error
 * ************************************************************************** *
 * \details Global variable 'g_u16ResolverPosX' and 'g_u16ResolverPosY' are
 *          updated
 * ************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * -vFunction calling: 1 (SetLastError())
 * ************************************************************************** */
uint16_t GetResolverPosition(void)
{
    uint16_t u16Result = C_ERR_NONE;
#if (_DEBUG_RESOLVER != FALSE)
    static uint16_t l_u16PrevAngle = 0U;
    static uint16_t l_u16PrevResolverPosX = 512U;
    static uint16_t l_u16PrevResolverPosY = 512U;
    static uint8_t l_u8ResolverPosErrorCnt = 0U;
#endif /* (_DEBUG_RESOLVER != FALSE) */

    g_u16ResolverPosX = l_AdcResult.u16Resolver_X;
    g_u16ResolverPosY = l_AdcResult.u16Resolver_Y;

    if ( ((g_u16ResolverPosX < C_ADC_RESOLVER_MIN) || (g_u16ResolverPosX > C_ADC_RESOLVER_MAX)) ||
         ((g_u16ResolverPosY < C_ADC_RESOLVER_MIN) || (g_u16ResolverPosY > C_ADC_RESOLVER_MAX)) )
    {
        u16Result = (C_ERR_TRIAXIS_INTERFACE | (C_ERR_EXT | 0x0A00U));
#if (_SUPPORT_LOG_ERRORS != FALSE)
        SetLastError(u16Result);                                                /* Triaxis/Resolver amplitude issue */
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
        g_e8ErrorElectric |= (uint8_t)C_ERR_SENSOR;
    }

#if (_DEBUG_RESOLVER != FALSE)
    while (g_u8ResolverCalibrated != FALSE)
    {
        int16_t i16Y =
            (int16_t)(p_MulI32_I16byI16( (int16_t)(g_u16ResolverPosY - g_u16ResolverOffY),
                                         (int16_t)g_u16ResolverAmplXY) >> 10);
        int16_t i16X = (int16_t)(g_u16ResolverPosX - g_u16ResolverOffX);
        uint16_t u16Angle = (uint16_t)p_atan2I16(i16Y, i16X);
        uint16_t u16ResolverPosDiff;

        /* Check X-signal noise */
        if (g_u16ResolverPosX > l_u16PrevResolverPosX)
        {
            u16ResolverPosDiff = g_u16ResolverPosX - l_u16PrevResolverPosX;
        }
        else
        {
            u16ResolverPosDiff = l_u16PrevResolverPosX - g_u16ResolverPosX;
        }
        l_u16PrevResolverPosX = g_u16ResolverPosX;
        if (u16ResolverPosDiff > 100U)
        {
            l_u8ResolverPosErrorCnt++;
            if (l_u8ResolverPosErrorCnt > 3U)
            {
                u16Result = C_ERR_TRIAXIS_X_CLIP;
#if (_SUPPORT_LOG_ERRORS != FALSE)
                SetLastError(u16Result);                                        /* Triaxis/Resolver connection issue */
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                g_e8ErrorElectric |= (uint8_t)C_ERR_SENSOR;
            }
            break;
        }

        /* Check Y-signal noise */
        if (g_u16ResolverPosY > l_u16PrevResolverPosY)
        {
            u16ResolverPosDiff = g_u16ResolverPosY - l_u16PrevResolverPosY;
        }
        else
        {
            u16ResolverPosDiff = l_u16PrevResolverPosY - g_u16ResolverPosY;
        }
        l_u16PrevResolverPosY = g_u16ResolverPosY;
        if (u16ResolverPosDiff > 100U)
        {
            l_u8ResolverPosErrorCnt++;
            if (l_u8ResolverPosErrorCnt > 3U)
            {
                u16Result = C_ERR_TRIAXIS_Y_CLIP;
#if (_SUPPORT_LOG_ERRORS != FALSE)
                SetLastError(u16Result);                                        /* Triaxis/Resolver connection issue */
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                g_e8ErrorElectric |= (uint8_t)C_ERR_SENSOR;
            }
            break;
        }

        /* Check Angle noise */
        if (g_e8MotorDirectionCCW == FALSE)
        {
            /* Clock Wise */
            if (u16Angle < l_u16PrevAngle)
            {
                l_u8ResolverPosErrorCnt++;
                if (l_u8ResolverPosErrorCnt > 3U)
                {
                    u16Result = C_ERR_TRIAXIS_DEADZONE_CW;
#if (_SUPPORT_LOG_ERRORS != FALSE)
                    SetLastError(u16Result);                                    /* Triaxis/Resolver connection issue */
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                    g_e8ErrorElectric |= (uint8_t)C_ERR_SENSOR;
                }
            }
            else
            {
                p_DecNzU8(&l_u8ResolverPosErrorCnt);
            }
        }
        else
        {
            /* Counter Clock Wise */
            if (u16Angle > l_u16PrevAngle)
            {
                l_u8ResolverPosErrorCnt++;
                if (l_u8ResolverPosErrorCnt > 3U)
                {
                    u16Result = C_ERR_TRIAXIS_DEADZONE_CCW;
#if (_SUPPORT_LOG_ERRORS != FALSE)
                    SetLastError(u16Result);                                    /* Triaxis/Resolver connection issue */
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                    g_e8ErrorElectric |= (uint8_t)C_ERR_SENSOR;
                }
            }
            else
            {
                p_DecNzU8(&l_u8ResolverPosErrorCnt);
            }
        }
        l_u16PrevAngle = u16Angle;
        break;
    }
#endif /* (_DEBUG_RESOLVER != FALSE) */
    return (u16Result);
} /* End of GetResolverPosition() */
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */

#if (ANA_COMM != FALSE)
/* ************************************************************************** *
 * Function : GetVio
 * Purpose  : Get I/O Voltage (LV)
 * Author   : mmp
 * ************************************************************************** *
 * \param   -
 * \return  (uint16_t) u16Result I/O-Voltage [ADC-LSB]
 * ************************************************************************** *
 * \details
 * ************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * -vFunction calling: 0
 * ************************************************************************** */
int16_t GetVio(void)
{
    return ((int16_t) (l_AdcResult.u16AdcIO - Get_LowVoltOffset()));
} /* End of GetVio() */
#endif /* (ANA_COMM != FALSE) */

/* EOF */
