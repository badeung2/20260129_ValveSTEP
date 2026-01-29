/*!*************************************************************************** *
 * \file        ActADC.h
 * \brief       MLX8133x ADC handling
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
 *           -# Get_RawVsupplyChip()
 *           -# Get_RawVmotorF()
 *           -# Get_RawTemperature()
 *           -# Set_RawTemperature()
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

#ifndef ACT_ADC_H_
#define ACT_ADC_H_

/*!*************************************************************************** */
/*                           INCLUDES                                          */
/* *************************************************************************** */
#include "AppBuild.h"                                                           /* Application Build */

#include "drivelib/ADC.h"

#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_VARIABLE_PWM != FALSE)
#include "drivelib/MotorDriver.h"                                               /* Motor driver support */
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_VARIABLE_PWM != FALSE) */

/*!*************************************************************************** */
/*                           DEFINITIONS                                       */
/* *************************************************************************** */
/*! ADC Mode */
typedef enum __attribute__((packed))
{
    C_ADC_MODE_ON_STEPPER = 3U,
    C_ADC_MODE_ON_BEMF
} ADC_MODE;

#if (_SUPPORT_POTI != FALSE)
#define C_POS_MOVAVG_SSZ        5U                                              /*!< Potentiometer moving-average buffer length (2^n) */
#define C_POS_MOVAVG_SZ         (1 << C_POS_MOVAVG_SSZ)                         /*!< Potentiometer moving-average buffer length */
#endif /* (_SUPPORT_POTI != FALSE) */

#if ((_SUPPORT_POTI != FALSE) && (_SUPPORT_POTI_MODE == C_POTI_MODE_CLOSED_LOOP)) || (ANA_COMM != FALSE)
#define C_DELAY_1MS             (uint16_t)((1000UL * FPLL) / C_DELAY_CONST)     /*!<  1ms delay */
#endif /* ((_SUPPORT_POTI != FALSE) && (_SUPPORT_POTI_MODE == C_POTI_MODE_CLOSED_LOOP)) || (ANA_COMM != FALSE) */

#if (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
/*! ADC Activated Solenoid data */
typedef struct _ADC_RESULTS
{
    uint16_t u16AdcVs;                                                          /**< Chip supply voltage (21:1) */
    uint16_t u16AdcVdda;                                                        /**< VDDA (2:1) */
    uint16_t u16AdcVddd;                                                        /**< VDDD (1:1) */
    uint16_t u16AdcCurr;                                                        /**< Motor driver coils current */
    uint16_t u16AdcVsmF;                                                        /**< Driver supply voltage (21:1) (Filtered) */
    uint16_t u16AdcTj;                                                          /**< Chip Junction Temperature */
    uint16_t u16CRC;                                                            /**< CRC of data */
} ADC_RESULTS;
#else  /* (_SUPPORT_APP_TYPE == C_APP_SOLENOID) */

#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM)
/*! ADC Running Motor data (DC-Motor)
 *  Dual Motor Current sampling at 50% +/- 5% */
typedef struct _ADC_RESULTS
{
    uint16_t u16AdcVs_1;                                                        /**< Chip supply voltage (21:1) */
    uint16_t u16AdcVdda;                                                        /**< VDDA (2:1) */
    uint16_t u16AdcVddd;                                                        /**< VDDD (1:1) */
    uint16_t u16AdcCurrA_1;                                                     /**< Motor driver coil 'A' current */
    uint16_t u16AdcCurrB_1;                                                     /**< Motor driver coil 'B' current */
    uint16_t u16AdcVsmF_1;                                                      /**< Driver supply voltage (21:1) */
    uint16_t u16AdcTj_1;                                                        /**< Chip Junction Temperature */
#if (_SUPPORT_POTI != FALSE)
    uint16_t u16AdcPotiPos;                                                     /**< Potentiometer Position voltage */
#elif (_SUPPORT_IO_DUT_SELECT_HVIO != FALSE) && (HVIO_DUT_SELECT == PIN_FUNC_IO_0)
    uint16_t u16AdcIO0HV;                                                       /**< HVIO DUT-Selection */
#endif /* (_SUPPORT_POTI != FALSE) */
    uint16_t u16AdcVs_2;                                                        /**< Chip supply voltage (21:1) */
#if (_SUPPORT_IO_CMD_SELECT_LVIO != FALSE)
    uint16_t u16AdcIoSelectA;                                                   /**< IO-Selection 'A' */
#if (_SUPPORT_NR_OF_IO_SELECT >= 2U)
    uint16_t u16AdcIoSelectB;                                                   /**< IO-Selection 'B' */
#if (_SUPPORT_NR_OF_IO_SELECT >= 3U)
    uint16_t u16AdcIoSelectC;                                                   /**< IO-Selection 'C' */
#endif /* (_SUPPORT_NR_OF_IO_SELECT >= 3U) */
#endif /* (_SUPPORT_NR_OF_IO_SELECT >= 2U) */
#endif /* (_SUPPORT_IO_CMD_SELECT_LVIO != FALSE) */
    uint16_t u16AdcCurrA_2;                                                     /**< Motor driver coil 'A' current */
    uint16_t u16AdcCurrB_2;                                                     /**< Motor driver coil 'B' current */
    uint16_t u16AdcVsmF_2;                                                      /**< Driver supply voltage (21:1) */
    uint16_t u16AdcTj_2;                                                        /**< Chip Junction Temperature */
#if (ANA_COMM != FALSE)
    uint16_t u16AdcRefPos;                                                      /**< HV-Potentiometer (Reference input) */
#endif /* (ANA_COMM != FALSE) */
    uint16_t u16CRC;                                                            /**< CRC of data */
} ADC_RESULTS;

#elif (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR)
/*! ADC Running Motor data (Single Coil Fan/Pump).
 *  Single Motor Current sampling at 50% */
typedef struct _ADC_RESULTS                                                     /* Mirror-mode PWM */
{
#if (_SUPPORT_ADC_BGD != FALSE)
    uint16_t u16AdcVs;                                                          /**< Vs-unfiltered (divided by 21) */
    uint16_t u16AdcVsmF;                                                        /**< Vsm-filtered (divided by 21) */
    uint16_t u16AdcTj;                                                          /**< Internal Temperature Sensor */
    uint16_t u16AdcCurr;                                                        /**< Motor-driver Current-unfiltered */
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE)
    uint16_t u16AdcVdda;                                                        /**< VDDA (2:1) */
    uint16_t u16AdcVddd;                                                        /**< VDDD (1:1) */
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) */
    uint16_t u16AdcVaux;                                                        /**< VAUX (4:1) */
    uint16_t u16AdcVbgd;                                                        /**< VBGD (1:1) */
#else  /* (_SUPPORT_ADC_BGD != FALSE) */
    uint16_t u16AdcVs;                                                          /**< Vs-unfiltered (divided by 21) */
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE)
    uint16_t u16AdcVdda;                                                        /**< VDDA (2:1) */
    uint16_t u16AdcVddd;                                                        /**< VDDD (1:1) */
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) */
    uint16_t u16AdcCurr;                                                        /**< Motor-driver Current-unfiltered */
    uint16_t u16AdcVsmF;                                                        /**< Vsm-filtered (divided by 21) */
    uint16_t u16AdcTj;                                                          /**< Internal Temperature Sensor */
#endif /* (_SUPPORT_ADC_BGD != FALSE) */
    uint16_t u16CRC;                                                            /**< CRC of data */
} ADC_RESULTS;

#elif (_SUPPORT_PWM_MODE == BIPOLAR_FULL_STEP_BEMF) || (_SUPPORT_PWM_MODE == BIPOLAR_HALF_STEP_BEMF)
/*! ADC Running Motor data (Bi-Polar BEMF Zero-Cross)
 *  Single Motor Current sampling at 50% */
typedef struct _ADC_RESULTS
{
    uint16_t u16AdcVs;                                                          /**< Chip supply voltage (21:1) */
    uint16_t u16AdcVdda;                                                        /**< VDDA (2:1) */
    uint16_t u16AdcVddd;                                                        /**< VDDD (1:1) */
#if (_SUPPORT_STALLDET_BRI != FALSE)                                            /* (MMP230802-1) */
    uint16_t u16AdcVbemfA;                                                      /**< Motor driver coil 'B' BEMF-Voltage */
#endif /* (_SUPPORT_STALLDET_BRI != FALSE) */
    uint16_t u16AdcCurrA;                                                       /**< Motor driver coil 'A' current */
#if (_SUPPORT_STALLDET_BRI != FALSE)                                            /* (MMP230802-1) */
    uint16_t u16AdcVbemfB;                                                      /**< Motor driver coil 'B' BEMF-Voltage */
#endif /* (_SUPPORT_STALLDET_BRI != FALSE) */
    uint16_t u16AdcVsmF;                                                        /**< Driver supply voltage (21:1) */
    uint16_t u16AdcTj;                                                          /**< Chip Junction Temperature */
    uint16_t u16CRC;                                                            /**< CRC of data */
} ADC_RESULTS;

#elif (_SUPPORT_PWM_MODE == BIPOLAR_PWM_SINGLE_INDEPENDENT_GND)
/*! ADC Running Motor data (Bi-Polar Stepper), with PWM in Mirror/Inverse-Mirror mode to Ground.
 *  Dual Motor Current sampling at 50% and 100% */
typedef struct _ADC_RESULTS
{
#if (_SUPPORT_ADC_BGD != FALSE)
    uint16_t u16AdcVs;                                                          /**< Chip supply voltage (21:1) */
    uint16_t u16AdcVsmF;                                                        /**< Driver supply voltage (21:1) */
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    uint16_t u16Resolver_X;                                                     /**< Resolver-X */
    uint16_t u16Resolver_Y;                                                     /**< Resolver-Y */
#else  /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    uint16_t u16AdcTj;                                                          /**< Chip Junction Temperature */
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    uint16_t u16AdcCurrA;                                                       /**< Motor driver coil 'A' current */
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    uint16_t u16AdcTj;                                                          /**< Chip Junction Temperature */
    uint16_t u16AdcVdda;                                                        /**< VDDA (2:1) */
#else  /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    uint16_t u16AdcVdda;                                                        /**< VDDA (2:1) */
    uint16_t u16AdcVddd;                                                        /**< VDDD (1:1) */
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    uint16_t u16AdcVaux;                                                        /**< VAUX (4:1) */
    uint16_t u16AdcVbgd;                                                        /**< VBGD (1:1) */
    uint16_t u16AdcCurrB;                                                       /**< Motor driver coil 'B' current */
#else  /* (_SUPPORT_ADC_BGD != FALSE) */
    uint16_t u16AdcVs;                                                          /**< Chip supply voltage (21:1) */
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    uint16_t u16Resolver_X;                                                     /**< Resolver-X */
    uint16_t u16Resolver_Y;                                                     /**< Resolver-Y */
#else  /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    uint16_t u16AdcVdda;                                                        /**< VDDA (2:1) */
    uint16_t u16AdcVddd;                                                        /**< VDDD (1:1) */
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    uint16_t u16AdcCurrA;                                                       /**< Motor driver coil 'A' current */
    uint16_t u16AdcVsmF;                                                        /**< Driver supply voltage (21:1) */
    uint16_t u16AdcTj;                                                          /**< Chip Junction Temperature */
    uint16_t u16AdcCurrB;                                                       /**< Motor driver coil 'B' current */
#endif /* (_SUPPORT_ADC_BGD != FALSE) */
    uint16_t u16CRC;                                                            /**< CRC of data */
} ADC_RESULTS;

#elif (_SUPPORT_PWM_MODE == BIPOLAR_TRIPHASE_ALLPWM_MIRROR)
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
/*! ADC Running Motor data using all Motor PWM in Mirror-mode. Dual Motor Current
 *  sampling at 25-50% and 75-100% (ADC_TRIGGER_PWM_MIN_MAX) or 12.5-37.5% and
 *  62.5-87.5% (ADC_TRIGGER_PWM_MID). In case of ADC_TRIGGER_PWM_MIN_MAX, between
 *  the two Motor-Current sampling fits 2-3 other ADC channel sampling. */
typedef struct _ADC_RESULTS                                                     /* Mirror-mode PWM */
{                                                                               /**< Structure of the ram where the ADC values are memorised in direct page */
    uint16_t u16AdcVs;                                                          /**< Chip supply voltage (21:1) */
#if (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW)
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX)
    uint16_t u16AdcVdda;                                                        /**< VDDA (2:1) */
    uint16_t u16AdcVddd;                                                        /**< VDDD (1:1) */
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX) */
    uint16_t u16AdcCurrA;                                                       /**< Motor driver coil 'A' current */
#else  /* (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW) */
    uint16_t u16AdcCurrA;                                                       /**< Motor driver coil 'A' current */
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX)
    uint16_t u16AdcVdda;                                                        /**< VDDA (2:1) */
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX) */
#endif /* (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW) */
    uint16_t u16AdcVsmF;                                                        /**< Driver supply voltage (21:1) */
    uint16_t u16AdcTj;                                                          /**< Chip Junction Temperature */
    uint16_t u16AdcCurrB;                                                       /**< Motor driver coil 'B' current */
    uint16_t u16CRC;                                                            /**< CRC of data */
} ADC_RESULTS;

#else   /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
/*! ADC Running Motor data (Bi-Polar as 3-phase) using three motor PWM in Mirror mode.
 *  Dual Motor Current sampling at +/-25% and +/-75% */
typedef struct _ADC_RESULTS
{
#if (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW)
    uint16_t u16AdcVs;                                                          /**< Chip supply voltage (21:1) */
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    uint16_t u16Resolver_X;                                                     /**< Resolver-X */
    uint16_t u16Resolver_Y;                                                     /**< Resolver-Y */
#elif (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX)
    uint16_t u16AdcVdda;                                                        /**< VDDA (2:1) */
    uint16_t u16AdcVddd;                                                        /**< VDDD (1:1) */
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    uint16_t u16AdcCurrA;                                                       /**< Motor driver coils current */
#else  /* (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW) */
    uint16_t u16AdcVs;                                                          /**< Chip supply voltage (21:1) */
    uint16_t u16AdcCurrA;                                                       /**< Motor driver coils current */
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    uint16_t u16Resolver_X;                                                     /**< Resolver-X */
    uint16_t u16Resolver_Y;                                                     /**< Resolver-Y */
#elif (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX)
    uint16_t u16AdcVdda;                                                        /**< VDDA (2:1) */
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
#endif /* (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW) */
    uint16_t u16AdcVsmF;                                                        /**< Driver supply voltage (21:1) */
    uint16_t u16AdcTj;                                                          /**< Chip Junction Temperature */
    uint16_t u16AdcCurrB;                                                       /**< Motor driver coils current */
    uint16_t u16CRC;                                                            /**< CRC of data */
} ADC_RESULTS;
#endif  /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */

#elif (_SUPPORT_PWM_MODE == TRIPLEPHASE_ALLPWM_MIRROR)
#if (_SUPPORT_STALLDET_BZC != FALSE)
/*! ADC Running Motor data (BLDC BEMF Zero-Cross) using three Motor PWM in Mirror mode.
 *  Single Motor Current sampling at +/-75% */
typedef struct _ADC_RESULTS
{
    uint16_t u16AdcVs;                                                          /**< Chip supply voltage (21:1) */
    uint16_t u16AdcVdda;                                                        /**< VDDA (2:1) */
    uint16_t u16AdcVddd;                                                        /**< VDDD (1:1) */
    uint16_t u16AdcVsmF;                                                        /**< Driver supply voltage (21:1) */
    uint16_t u16AdcTj;                                                          /**< Chip Junction Temperature */
    uint16_t u16AdcCurr;                                                        /**< Motor driver coils current */
    uint16_t u16CRC;                                                            /**< CRC of data */
} ADC_RESULTS;

#elif (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
/* FOC Based */
/*! ADC Running Motor data using all Motor PWM in Mirror-mode. Dual Motor Current
 *  sampling at 25-50% and 75-100% (ADC_TRIGGER_PWM_MIN_MAX) or 12.5-37.5% and
 *  62.5-87.5% (ADC_TRIGGER_PWM_MID). In case of ADC_TRIGGER_PWM_MIN_MAX, between
 *  the two Motor-Current sampling fits 2-3 other ADC channel sampling. */
typedef struct _ADC_RESULTS                                                     /* Mirror-mode PWM */
{                                                                               /**< Structure of the ram where the ADC values are memorised in direct page */
    uint16_t u16AdcVs;                                                          /**< Chip supply voltage (21:1) */
#if (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW)
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX)
    uint16_t u16AdcVdda;                                                        /**< VDDA (2:1) */
    uint16_t u16AdcVddd;                                                        /**< VDDD (1:1) */
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX) */
    uint16_t u16AdcCurrA;                                                       /**< Motor driver coil 'A' current */
#else  /* (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW) */
    uint16_t u16AdcCurrA;                                                       /**< Motor driver coil 'A' current */
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX)
    uint16_t u16AdcVdda;                                                        /**< VDDA (2:1) */
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX) */
#endif /* (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW) */
    uint16_t u16AdcVsmF;                                                        /**< Driver supply voltage (21:1) */
    uint16_t u16AdcTj;                                                          /**< Chip Junction Temperature */
    uint16_t u16AdcCurrB;                                                       /**< Motor driver coil 'B' current */
    uint16_t u16CRC;                                                            /**< CRC of data */
} ADC_RESULTS;

#else
/*! ADC Running Motor data using three Motor PWM in Mirror mode.
 *  Dual Motor Current sampling at +/-25% and +/-75% */
typedef struct _ADC_RESULTS
{
#if (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW)
    uint16_t u16AdcVs;                                                          /**< Chip supply voltage (21:1) */
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    uint16_t u16Resolver_X;                                                     /**< Resolver-X */
    uint16_t u16Resolver_Y;                                                     /**< Resolver-Y */
#elif (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX)
    uint16_t u16AdcVdda;                                                        /**< VDDA (2:1) */
    uint16_t u16AdcVddd;                                                        /**< VDDD (1:1) */
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    uint16_t u16AdcCurrA;                                                       /**< Motor driver coils current */
#else  /* (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW) */
    uint16_t u16AdcVs;                                                          /**< Chip supply voltage (21:1) */
    uint16_t u16AdcCurrA;                                                       /**< Motor driver coils current */
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    uint16_t u16Resolver_X;                                                     /**< Resolver-X */
    uint16_t u16Resolver_Y;                                                     /**< Resolver-Y */
#elif (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX)
    uint16_t u16AdcVdda;                                                        /**< VDDA (2:1) */
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
#endif /* (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW) */
    uint16_t u16AdcVsmF;                                                        /**< Driver supply voltage (21:1) */
    uint16_t u16AdcTj;                                                          /**< Chip Junction Temperature */
    uint16_t u16AdcCurrB;                                                       /**< Motor driver coils current */
    uint16_t u16CRC;                                                            /**< CRC of data */
} ADC_RESULTS;
#endif

#elif (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_MIRROR_SUP)
/*! ADC Running Motor data using two Motor PWM in Mirror mode to Supply.
 *  Single Motor Current sampling at 100% */
typedef struct _ADC_RESULTS
{
#if (_SUPPORT_ADC_BGD != FALSE)
    uint16_t u16AdcVs;                                                          /**< Chip supply voltage (21:1) */
    uint16_t u16AdcVsmF;                                                        /**< Driver supply voltage (21:1) */
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    uint16_t u16Resolver_X;                                                     /**< Resolver-X */
    uint16_t u16Resolver_Y;                                                     /**< Resolver-Y */
    uint16_t u16AdcTj;                                                          /**< Chip Junction Temperature */
    uint16_t u16AdcVdda;                                                        /**< VDDA (2:1) */
#else  /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    uint16_t u16AdcTj;                                                          /**< Chip Junction Temperature */
    uint16_t u16AdcVdda;                                                        /**< VDDA (2:1) */
    uint16_t u16AdcVddd;                                                        /**< VDDD (1:1) */
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    uint16_t u16AdcVaux;                                                        /**< VAUX (4:1) */
    uint16_t u16AdcVbgd;                                                        /**< VBGD (1:1) */
#else  /* (_SUPPORT_ADC_BGD != FALSE) */
    uint16_t u16AdcVs;                                                          /**< Chip supply voltage (21:1) */
    uint16_t u16AdcVdda;                                                        /**< VDDA (2:1) */
    uint16_t u16AdcVddd;                                                        /**< VDDD (1:1) */
    uint16_t u16AdcVsmF;                                                        /**< Driver supply voltage (21:1) */
    uint16_t u16AdcTj;                                                          /**< Chip Junction Temperature */
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    uint16_t u16Resolver_X;                                                     /**< Resolver-X */
    uint16_t u16Resolver_Y;                                                     /**< Resolver-Y */
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    uint16_t u16AdcCurrA;                                                       /**< Motor driver coils current */
#endif /* (_SUPPORT_ADC_BGD != FALSE) */
    uint16_t u16CRC;                                                            /**< CRC of data */
} ADC_RESULTS;

#elif (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_MIRROR_GND)
/*! ADC Running Motor data using two Motor PWM in Mirror mode to Ground.
 *  Single Motor Current sampling at 50% */
typedef struct _ADC_RESULTS
{
#if (_SUPPORT_ADC_BGD != FALSE)
    uint16_t u16AdcVs;                                                          /**< Chip supply voltage (21:1) */
    uint16_t u16AdcVsmF;                                                        /**< Driver supply voltage (21:1) */
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    uint16_t u16Resolver_X;                                                     /**< Resolver-X */
    uint16_t u16Resolver_Y;                                                     /**< Resolver-Y */
#else  /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    uint16_t u16AdcTj;                                                          /**< Chip Junction Temperature */
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    uint16_t u16AdcCurr;                                                        /**< Motor driver coils current */
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    uint16_t u16AdcTj;                                                          /**< Chip Junction Temperature */
    uint16_t u16AdcVdda;                                                        /**< VDDA (2:1) */
#else  /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    uint16_t u16AdcVdda;                                                        /**< VDDA (2:1) */
    uint16_t u16AdcVddd;                                                        /**< VDDD (1:1) */
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    uint16_t u16AdcVaux;                                                        /**< VAUX (4:1) */
    uint16_t u16AdcVbgd;                                                        /**< VBGD (1:1) */
#else  /* (_SUPPORT_ADC_BGD != FALSE) */
    uint16_t u16AdcVs;                                                          /**< Chip supply voltage (21:1) */
    uint16_t u16AdcVdda;                                                        /**< VDDA (2:1) */
    uint16_t u16AdcVddd;                                                        /**< VDDD (1:1) */
    uint16_t u16AdcCurr;                                                        /**< Motor driver coils current */
    uint16_t u16AdcVsmF;                                                        /**< Driver supply voltage (21:1) */
    uint16_t u16AdcTj;                                                          /**< Chip Junction Temperature */
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    uint16_t u16Resolver_X;                                                     /**< Resolver-X */
    uint16_t u16Resolver_Y;                                                     /**< Resolver-Y */
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
#endif /* (_SUPPORT_ADC_BGD != FALSE) */
    uint16_t u16CRC;                                                            /**< CRC of data */
} ADC_RESULTS;

#elif (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_INDEPENDENT_GND) || \
    (_SUPPORT_PWM_MODE == BIPOLAR_TRIPHASE_TWOPWM_INDEPENDENT_GND)
/*! ADC Running Motor data using two Motor PWM in Mirror/Inverse-Mirror
 *  mode or Bi-Polar as 3-phase. Dual Motor Current sampling at 50% and 100%
 *  At  ~12.5%  : VS (IC Supply)
 *  At  ~22.5%* : Resolver_X or VDDA
 *  At  ~32.5%* : Resolver_Y or VDDD
 *  At  ~50.0%  : Coil Current A
 *  At  ~60.0%* : VSMF (Motor Driver Supply, Filtered)
 *  At  ~70.0%* : Junction Temperature IC
 *  At  ~80.0%* : Resolver2_X or VDDA or LV-IO or HV-IO or BGD
 *  At  ~90.0%* : Resolver2_Y or VDDD or nothing
 *  At ~100.0%  : Coil Current B
 * With VBGD:     with Resolver          w/o Resolver
 *  At  12.5%   : VS                     VS (IC Supply)
 *  At  ~22.0%* : VSMF                   VSMF (Motor Driver Supply, Filtered)
 *  At  ~31.5%* : Resolver_X             Junction Temperature
 *  At  ~41.0%* : Resolver_Y             Optional: LV-IO or HV-IO
 *  At  ~50.0%  : Coil Current A         Coil Current A
 *  At  ~59.5%* : Junction Temperature   VDDA
 *  At  ~69.0%* : VDDA                   VDDD
 *  At  ~78.5%* : VAUX                   VAUX
 *  At  ~90.5%**: VBGD                   VBGD
 *  At ~100.0%  : Coil Current B         Coil Current B
 *  *) MLX8133x: Previous ADC-EOC (~3us) + 1.5us, MLX8134x: Previous ADC-EOC (2.9us) + 0.6us
 *  **) MLX8133x: Previous ADC-EOC (~3us) + 4.0us, MLX8134x: Previous ADC-EOC (2.9us) + 1.0us
 *  At 50us PWM Period and ADC @ 4MHz (MLX8133x): ~10 ADC conversions per PWM period
 */
typedef struct _ADC_RESULTS
{
#if (_SUPPORT_ADC_BGD != FALSE)
    uint16_t u16AdcVs;                                                          /**< Chip supply voltage (21:1) */
    uint16_t u16AdcVsmF;                                                        /**< Driver supply voltage (21:1) */
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    uint16_t u16Resolver_X;                                                     /**< Resolver-X */
    uint16_t u16Resolver_Y;                                                     /**< Resolver-Y */
#else  /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    uint16_t u16AdcTj;                                                          /**< Chip Junction Temperature */
#if (ANA_COMM != FALSE)
    uint16_t u16AdcVana;                                                        /**< Analogue input LV */
#elif (_SUPPORT_IO_DUT_SELECT_HVIO_LR != FALSE) && (IO_DUT_SELECT == PIN_FUNC_IO_0)
    uint16_t u16AdcIO0HV;                                                       /**< Analogue input HV */
#endif
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    uint16_t u16AdcCurrA;                                                       /**< Motor driver coil 'A' current */
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    uint16_t u16AdcTj;                                                          /**< Chip Junction Temperature */
#if (_SUPPORT_RESOLVER2_X_IO != PIN_FUNC_IO_NONE) && (_SUPPORT_RESOLVER2_Y_IO != PIN_FUNC_IO_NONE)
    uint16_t u16Resolver2_X;                                                    /**< Resolver-X #2 */
    uint16_t u16Resolver2_Y;                                                    /**< Resolver-Y #2 */
#else  /* (_SUPPORT_RESOLVER2_X_IO != PIN_FUNC_IO_NONE) && (_SUPPORT_RESOLVER2_Y_IO != PIN_FUNC_IO_NONE) */
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE)
    uint16_t u16AdcVdda;                                                        /**< VDDA (2:1) */
    uint16_t u16AdcVaux;                                                        /**< VAUX (4:1) */
    uint16_t u16AdcVbgd;                                                        /**< BGD Voltage */
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) */
#endif /* (_SUPPORT_RESOLVER2_X_IO != PIN_FUNC_IO_NONE) && (_SUPPORT_RESOLVER2_Y_IO != PIN_FUNC_IO_NONE) */
#else  /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE)
    uint16_t u16AdcVdda;                                                        /**< VDDA (2:1) */
    uint16_t u16AdcVddd;                                                        /**< VDDD (1:1) */
    uint16_t u16AdcVaux;                                                        /**< VAUX (4:1) */
    uint16_t u16AdcVbgd;                                                        /**< BGD Voltage */
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) */
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    uint16_t u16AdcCurrB;                                                       /**< Motor driver coil 'B' current */
#else  /* (_SUPPORT_ADC_BGD != FALSE) */
    uint16_t u16AdcVs;                                                          /**< Chip supply voltage (21:1) */
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    uint16_t u16Resolver_X;                                                     /**< Resolver-X */
    uint16_t u16Resolver_Y;                                                     /**< Resolver-Y */
#elif (_SUPPORT_ADC_VDDA_VDDD != FALSE)
    uint16_t u16AdcVdda;                                                        /**< VDDA (2:1) */
    uint16_t u16AdcVddd;                                                        /**< VDDD (1:1) */
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    uint16_t u16AdcCurrA;                                                       /**< Motor driver coil 'A' current */
    uint16_t u16AdcVsmF;                                                        /**< Driver supply voltage (21:1) */
    uint16_t u16AdcTj;                                                          /**< Chip Junction Temperature */
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
#if (_SUPPORT_RESOLVER2_X_IO != PIN_FUNC_IO_NONE) && (_SUPPORT_RESOLVER2_Y_IO != PIN_FUNC_IO_NONE)
    uint16_t u16Resolver2_X;                                                    /**< Resolver-X #2 */
    uint16_t u16Resolver2_Y;                                                    /**< Resolver-Y #2 */
#elif (_SUPPORT_ADC_VDDA_VDDD != FALSE)
    uint16_t u16AdcVdda;                                                        /**< VDDA (2:1) */
    uint16_t u16AdcVddd;                                                        /**< VDDD (1:1) */
#endif /* (_SUPPORT_RESOLVER2_X_IO != PIN_FUNC_IO_NONE) && (_SUPPORT_RESOLVER2_Y_IO != PIN_FUNC_IO_NONE) */
#elif (ANA_COMM != FALSE)
    uint16_t u16AdcVana;                                                        /**< Analogue input LV */
#elif (_SUPPORT_IO_DUT_SELECT_HVIO != FALSE) && (HVIO_DUT_SELECT == PIN_FUNC_IO_0)
    uint16_t u16AdcIO0HV;                                                       /**< Analogue input HV */
#elif (_SUPPORT_NTC != FALSE)
    uint16_t u16AdcNTC;                                                         /**< NTC voltage */
#elif (_SUPPORT_ADC_BGD != FALSE)
    uint16_t u16AdcVbgd;                                                        /**< BGD Voltage */
#elif (_SUPPORT_POTI != FALSE)
    uint16_t u16AdcPotiPos;                                                     /**< Potentiometer Voltage */
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
    uint16_t u16AdcCurrB;                                                       /**< Motor driver coil 'B' current */
#endif /* _SUPPORT_ADC_BGD */
    uint16_t u16CRC;                                                            /**< CRC of data */
} ADC_RESULTS;

#elif (_SUPPORT_PWM_MODE == TRIPLEPHASE_FULL_STEP_BEMF)
/*! ADC Running Motor data (BLDC Full-step BEMF) */
typedef struct _ADC_RESULTS
{
    uint16_t u16AdcVs;                                                          /**< Chip supply voltage (21:1) */
    uint16_t u16AdcVdda;                                                        /**< VDDA (2:1) */
    uint16_t u16AdcVddd;                                                        /**< VDDD (1:1) */
    uint16_t u16AdcCurrA;                                                       /**< Motor driver coil 'A' current */
#if (_SUPPORT_STALLDET_BRI != FALSE)
    uint16_t u16AdcVbemf;                                                       /**< BEMF Voltage */
#endif /* (_SUPPORT_STALLDET_BRI != FALSE) */
    uint16_t u16AdcVsmF;                                                        /**< Driver supply voltage (21:1) */
    uint16_t u16AdcTj;                                                          /**< Chip Junction Temperature */
    uint16_t u16CRC;                                                            /**< CRC of data */
} ADC_RESULTS;
#endif
#endif /* (_SUPPORT_APP_TYPE == C_APP_SOLENOID) */

#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_VARIABLE_PWM != FALSE)
/*! ADC Running Motor data using triple Motor PWM in Independent mode. (MMP220815-1)
 *  Current sampling at fixed 33%, 67% and 100%
 *  At   12.5%  : VS (IC Supply)
 *  At  ~19.4%* : <Free>
 *  At  ~26.3%* : <Free>
 *  At  ~33.3%  : Coil Current A (PWM_SLAVE1)
 *  At  ~41.7%* : VDDA
 *  At  ~50.0%* : VBGD
 *  At  ~58.3%* : VDDD
 *  At  ~66.7%  : Coil Current B (PWM_SLAVE3)
 *  At  ~75.0%* : VSMF (Motor Driver Supply, Filtered)
 *  At  ~83.3%* : Junction Temperature IC
 *  At  ~91.7%* : VBOOST
 *  At ~100.0%  : Coil Current C (PWM_MASTER2)
 *  *) Previous ADC-EOC (2.9us) + 0.6us
 */
typedef struct _ADC_RESULTS_3                                                   /* Independent-mode PWM */
{                                                                               /* Structure of the ram where the ADC values are memorised in direct page */
    uint16_t u16AdcVs;                                                          /**< Chip supply voltage (21:1) */
    uint16_t u16AdcCurrA;                                                       /**< Motor driver coil 'A' current */
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE)
    uint16_t u16AdcVdda;                                                        /**< Chip VDDA supply voltage (2:1) */
#if (_SUPPORT_ADC_BGD != FALSE)
    uint16_t u16AdcVbgd;                                                        /**< BGD Voltage (1:1) */
#endif /* (_SUPPORT_ADC_BGD != FALSE) */
    uint16_t u16AdcVddd;                                                        /**< Chip VDDD supply voltage (1:1) */
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) */
    uint16_t u16AdcCurrB;                                                       /**< Motor driver coil 'B' current */
    uint16_t u16AdcVsmF;                                                        /**< Driver supply voltage (21:1) */
    uint16_t u16AdcTj;                                                          /**< Chip Junction Temperature */
#if (_SUPPORT_ADC_VBOOST != FALSE)
    uint16_t u16AdcVboost;                                                      /**< Boost voltage (31:1) */
#endif /* (_SUPPORT_ADC_VBOOST != FALSE) */
    uint16_t u16AdcCurrC;                                                       /**< Motor driver coil 'C' current */
    uint16_t u16CRC;                                                            /**< CRC of data */
} ADC_RESULTS_3;
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_VARIABLE_PWM != FALSE) */

#if (_SUPPORT_STALLDET_BZC != FALSE)
/*! ADC Running Motor data (BEMF-mode) */
typedef struct _ADC_MOTORRUN_BEMF
{
    uint16_t u16AdcVs;                                                          /**< Unfiltered Supply Voltage (Vs) */
    uint16_t u16AdcVsmF;                                                        /**< Filtered Motor Driver Voltage (Vsm) */
#if (_SUPPORT_BEMF_STARPOINT != FALSE)
    uint16_t u16AdcVio0HV;                                                      /**< HV-IO[0] Voltage */
#endif /* (_SUPPORT_BEMF_STARPOINT != FALSE) */
    uint16_t u16AdcVphA;                                                        /**< Phase voltage (Vph) */
    uint16_t u16AdcCurr;                                                        /**< Unfiltered Motor Driver Current */
    uint16_t u16AdcTj;                                                          /**< Internal Temperature Sensor */
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE)
    uint16_t u16AdcVdda;                                                        /**< Chip VDDA supply voltage (2:1) */
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) */
    uint16_t u16AdcVphB;                                                        /**< Phase voltage (Vph) */
    uint16_t u16CRC;                                                            /**< CRC of data */
} ADC_MOTORRUN_BEMF;
#endif /* (_SUPPORT_STALLDET_BZC != FALSE) */

#if ((_SUPPORT_POTI != FALSE) && (_SUPPORT_POTI_MODE == C_POTI_MODE_CLOSED_LOOP)) || (ANA_COMM != FALSE)
/*! ADC Position data */
typedef struct _ADC_POS
{
    uint16_t u16AdcVs;                                                          /**<  15%: Chip supply voltage (21:1) */
#if (_SUPPORT_POTI != FALSE)
    uint16_t u16AdcPotiPos;                                                     /**<       Potentiometer Position voltage */
#endif /* (_SUPPORT_POTI != FALSE) */
#if (ANA_COMM != FALSE)
    uint16_t u16AdcRefPos;                                                      /**<       HV-Potentiometer (Reference input) */
#endif /* (ANA_COMM != FALSE) */
    uint16_t u16CRC;                                                            /**< CRC of data */
} ADC_POS;
#endif /* ((_SUPPORT_POTI != FALSE) && (_SUPPORT_POTI_MODE == C_POTI_MODE_CLOSED_LOOP)) || (ANA_COMM != FALSE) */

/*!*************************************************************************** */
/*                           GLOBAL VARIABLES                                  */
/* *************************************************************************** */
#pragma space dp                                                                /* __TINY_SECTION__ */
#if (C_MOTOR_PHASES != 1)
extern int16_t g_i16MotorCurrentCoilA;                                          /*!< Phase current 'A' without offset */
#if (_SUPPORT_PWM_MODE != BIPOLAR_FULL_STEP_BEMF) && (_SUPPORT_PWM_MODE != BIPOLAR_HALF_STEP_BEMF)
extern int16_t g_i16MotorCurrentCoilB;                                          /*!< Phase current 'B' without offset */
#if (C_MOTOR_PHASES == 3)
extern int16_t g_i16MotorCurrentCoilC;                                          /*!< Phase current 'C' without offset */
extern int16_t g_i16MotorCurrentCoilY;                                          /*!< Phase current 'Y' (combination of 'B' + 'C') */
#elif (C_MOTOR_PHASES == 4)
#define g_i16MotorCurrentCoilY  g_i16MotorCurrentCoilB                          /*!< Phase current 'Y' is the same as 'B' for a bipolar */
#endif /* (C_MOTOR_PHASES == 3) */
#endif /* (_SUPPORT_PWM_MODE != BIPOLAR_FULL_STEP_BEMF) && (_SUPPORT_PWM_MODE != BIPOLAR_HALF_STEP_BEMF) */
#endif /* (C_MOTOR_PHASES != 1) */
extern uint16_t g_u16MotorCurrentPeak;                                          /*!< Motor Current Peak [ADC-LSB] (MMP230803-1) */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) || (_SUPPORT_STALLDET_LA != FALSE)
extern int16_t g_i16MotorCurrentAngle;                                          /*!< Motor Current Angle [0..65535 = 0..2pi] (MMP230803-1) */
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) || (_SUPPORT_STALLDET_LA != FALSE) */
#if (_SUPPORT_STALLDET_BRI != FALSE)
extern uint16_t g_u16MotorBemfVoltage;                                          /*!< Motor BEMF Voltage */
#endif /* (_SUPPORT_STALLDET_BRI != FALSE) */
#if (ANA_COMM != FALSE)
extern uint16_t g_u16ReferencePotiPos;                                          /*!< Actual Reference position */
#endif /* (ANA_COMM != FALSE) */
#pragma space none                                                              /* __TINY_SECTION__ */

#pragma space nodp                                                              /* __NEAR_SECTION__ */
extern volatile ADC_RESULTS l_AdcResult;                                        /* ADC results */
#pragma space none                                                              /* __NEAR_SECTION__ */

/*!*************************************************************************** */
/*                           GLOBAL FUNCTIONS                                  */
/* *************************************************************************** */
#if (_SUPPORT_PWM_MODE == BIPOLAR_FULL_STEP_BEMF) || (_SUPPORT_PWM_MODE == BIPOLAR_HALF_STEP_BEMF) || (_SUPPORT_PWM_MODE == TRIPLEPHASE_FULL_STEP_BEMF)
extern void ADC_Start(uint16_t u16FullStep, uint16_t u16IrqEna);
#else  /* (_SUPPORT_PWM_MODE == BIPOLAR_FULL_STEP_BEMF) || (_SUPPORT_PWM_MODE == BIPOLAR_HALF_STEP_BEMF) || (_SUPPORT_PWM_MODE == TRIPLEPHASE_FULL_STEP_BEMF) */
extern void ADC_Start(uint16_t u16IrqEna);
#endif /* (_SUPPORT_PWM_MODE == BIPOLAR_FULL_STEP_BEMF) || (_SUPPORT_PWM_MODE == BIPOLAR_HALF_STEP_BEMF) || (_SUPPORT_PWM_MODE == TRIPLEPHASE_FULL_STEP_BEMF) */
#if (_SUPPORT_MOTION_DET == C_MOTION_DET_BEMF) || (_SUPPORT_MOTION_DET == C_MOTION_DET_SENSOR)
extern void ADC_MovementDetection(void);                                        /* Start ADC for movement detection */
#endif /* (_SUPPORT_MOTION_DET == C_MOTION_DET_BEMF) || (_SUPPORT_MOTION_DET == C_MOTION_DET_SENSOR) */
#if (_SUPPORT_STALLDET_BZC != FALSE)
extern void ADC_BEMF_Start(uint16_t u16Idx);
#endif /* (_SUPPORT_STALLDET_BZC != FALSE) */

#if (_SUPPORT_APP_TYPE != C_APP_SOLENOID)
extern uint16_t ADC_GetRawMotorDriverCurrent(uint16_t u16MicroStepIdx);
#else  /* (_SUPPORT_APP_TYPE != C_APP_SOLENOID) */
extern uint16_t ADC_GetRawMotorDriverCurrent(void);
#endif  /* (_SUPPORT_APP_TYPE != C_APP_SOLENOID) */
extern void ADC_MeasureVsupplyAndTemperature(void);
extern uint16_t ADC_GetNewSampleVsupply(void);
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
extern void MeasureResolverPos(void);
extern uint16_t GetResolverPosition(void);
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
#if (_SUPPORT_POTI != FALSE)
extern uint16_t ADC_PotentiometerPosition(void);
#endif /* (_SUPPORT_POTI != FALSE) */
#if (ANA_COMM != FALSE)
extern uint16_t ADC_ReferencePosition(void);
#endif /* (ANA_COMM != FALSE) */
#if (_SUPPORT_POTI != FALSE) || (ANA_COMM != FALSE)
extern void ADC_ActRefPositionInit(void);
#endif /* (_SUPPORT_POTI != FALSE) || (ANA_COMM != FALSE) */

/*!*************************************************************************** *
 * Get_RawVsupplyChip
 * \brief   Get variable l_AdcResult.u16AdcVs
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_AdcResult.u16AdcVs
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_RawVsupplyChip(void)
{
#if (_SUPPORT_STALLDET_BZC != FALSE)
    extern uint8_t l_u8AdcMode;
    extern volatile ADC_MOTORRUN_BEMF l_AdcMotorRunBemf;
#endif /* (_SUPPORT_STALLDET_BZC != FALSE) */
    extern uint16_t l_u16HighVoltOffset;

#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) && (_SUPPORT_APP_TYPE != C_APP_SOLENOID)
    return ( ((l_AdcResult.u16AdcVs_1 + l_AdcResult.u16AdcVs_2) >> 1) - l_u16HighVoltOffset);
#else  /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_VARIABLE_PWM != FALSE)    /* MMP220815-1 */
    if (g_u8TriplePWM != FALSE)
    {
        extern volatile ADC_RESULTS_3 l_AdcResult3;
        return (l_AdcResult3.u16AdcVs - l_u16HighVoltOffset);
    }
    else
#elif (_SUPPORT_STALLDET_BZC != FALSE)
    if (l_u8AdcMode == (uint8_t)C_ADC_MODE_ON_BEMF)
    {
        return (l_AdcMotorRunBemf.u16AdcVs - l_u16HighVoltOffset);
    }
    else
#endif
    {
        return (l_AdcResult.u16AdcVs - l_u16HighVoltOffset);
    }
#endif /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) */
} /* End of GetRawVsupplyChip() */

/*!*************************************************************************** *
 * Set_RawVsupplyChip
 * \brief   Set variable l_AdcResult.u16AdcVs
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16Value: New u16AdcVs value.
 * \return  -
 * *************************************************************************** *
 * \details in-line function to set 'local' variable
 * *************************************************************************** */
static inline void Set_RawVsupplyChip(uint16_t u16Value)
{
#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) && (_SUPPORT_APP_TYPE != C_APP_SOLENOID)
    l_AdcResult.u16AdcVs_1 = u16Value;
    l_AdcResult.u16AdcVs_2 = u16Value;
#else  /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && _SUPPORT_VARIABLE_PWM               /* MMP220815-1 */
    if (g_u8TriplePWM != FALSE)
    {
        extern volatile ADC_RESULTS_3 l_AdcResult3;
        l_AdcResult3.u16AdcVs = u16Value;
    }
    else
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && _SUPPORT_VARIABLE_PWM */
    {
        l_AdcResult.u16AdcVs = u16Value;
    }
#endif /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) */
} /* End of Set_RawVsupplyChip() */

/*!*************************************************************************** *
 * Get_RawVmotorF
 * \brief   Get variable l_AdcResult.u16AdcVsmF
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_AdcResult.u16AdcVsmF
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_RawVmotorF(void)
{
#if (_SUPPORT_STALLDET_BZC != FALSE)
    extern uint8_t l_u8AdcMode;
    extern volatile ADC_MOTORRUN_BEMF l_AdcMotorRunBemf;
#endif /* (_SUPPORT_STALLDET_BZC != FALSE) */

#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) && (_SUPPORT_APP_TYPE != C_APP_SOLENOID)
    return ( ((l_AdcResult.u16AdcVsmF_1 + l_AdcResult.u16AdcVsmF_2) >> 1) - Get_VsmOffset() );
#else  /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_VARIABLE_PWM != FALSE)    /* MMP220815-1 */
    if (g_u8TriplePWM != FALSE)
    {
        extern volatile ADC_RESULTS_3 l_AdcResult3;                             /* ADC results */
        return (l_AdcResult3.u16AdcVsmF - Get_VsmOffset() );
    }
    else
#elif (_SUPPORT_STALLDET_BZC != FALSE)
    if (l_u8AdcMode == (uint8_t)C_ADC_MODE_ON_BEMF)
    {
        return (l_AdcMotorRunBemf.u16AdcVsmF - Get_VsmOffset() );
    }
    else
#endif
    {
        uint16_t u16RawVSMF = 0U;
        if ((int16_t)l_AdcResult.u16AdcVsmF > Get_VsmOffset())
        {
            u16RawVSMF = (uint16_t)(l_AdcResult.u16AdcVsmF - Get_VsmOffset());
        }
        return (u16RawVSMF);
    }
#endif /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) */
} /* End of Get_RawVmotorF() */

/*!*************************************************************************** *
 * Get_RawTemperature
 * \brief   Get variable l_AdcResult.u16AdcTj
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_AdcResult.u16AdcTj
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_RawTemperature(void)
{
#if (_SUPPORT_STALLDET_BZC != FALSE)
    extern uint8_t l_u8AdcMode;
    extern volatile ADC_MOTORRUN_BEMF l_AdcMotorRunBemf;
#endif /* (_SUPPORT_STALLDET_BZC != FALSE) */

#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) && (_SUPPORT_APP_TYPE != C_APP_SOLENOID)
    return ( (l_AdcResult.u16AdcTj_1 + l_AdcResult.u16AdcTj_2) >> 1);
#else  /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_VARIABLE_PWM != FALSE)    /* MMP220815-1 */
    if (g_u8TriplePWM != FALSE)
    {
        extern volatile ADC_RESULTS_3 l_AdcResult3;                             /* ADC results */
        return (l_AdcResult3.u16AdcTj);
    }
    else
#elif (_SUPPORT_STALLDET_BZC != FALSE)
    if (l_u8AdcMode == (uint8_t)C_ADC_MODE_ON_BEMF)
    {
        return (l_AdcMotorRunBemf.u16AdcTj);
    }
    else
#endif
    {
#if (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX)
        return (l_AdcResult.u16AdcTj);
#else  /* (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX) */
        return (600);
#endif /* (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX) */
    }
#endif /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) */
} /* End of Get_RawTemperature() */

/*!*************************************************************************** *
 * Set_RawTemperature
 * \brief   Set variable l_AdcResult.u16AdcTj
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16Value
 * \return  -
 * *************************************************************************** *
 * \details in-line function to set 'local' variable
 * *************************************************************************** */
static inline void Set_RawTemperature(uint16_t u16Value)
{
#if (_SUPPORT_STALLDET_BZC != FALSE)
    extern uint8_t l_u8AdcMode;
    extern volatile ADC_MOTORRUN_BEMF l_AdcMotorRunBemf;
#endif /* (_SUPPORT_STALLDET_BZC != FALSE) */

#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) && (_SUPPORT_APP_TYPE != C_APP_SOLENOID)
    l_AdcResult.u16AdcTj_1 = u16Value;
    l_AdcResult.u16AdcTj_2 = u16Value;
#else  /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_VARIABLE_PWM != FALSE)    /* MMP220815-1 */
    if (g_u8TriplePWM != FALSE)
    {
        extern volatile ADC_RESULTS_3 l_AdcResult3;                             /* ADC results */
        l_AdcResult3.u16AdcTj = u16Value;
    }
    else
#elif (_SUPPORT_STALLDET_BZC != FALSE)
    if (l_u8AdcMode == (uint8_t)C_ADC_MODE_ON_BEMF)
    {
        l_AdcMotorRunBemf.u16AdcTj = u16Value;
    }
    else
#endif
    {
#if (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX)
        l_AdcResult.u16AdcTj = u16Value;
#else  /* (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX) */
        (void)u16Value;
#endif /* (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX) */
    }
#endif /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) */
} /* End of SetRawTemperature() */

#if (_SUPPORT_ADC_VDDA_VDDD != FALSE)
/*!*************************************************************************** *
 * Get_AdcVdda
 * \brief   Get IC VDDA value (ADC-LSB)
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_AdcResult.u16AdcVdda
 * *************************************************************************** *
 * \details in-line function to get ADC VDDA supply level.
 * *************************************************************************** */
static inline uint16_t Get_AdcVdda(void)
{
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && _SUPPORT_VARIABLE_PWM               /* MMP220815-1 */
    if (g_u8TriplePWM != FALSE)
    {
        extern volatile ADC_RESULTS_3 l_AdcResult3;                             /* ADC results */
        return (l_AdcResult3.u16AdcVdda - C_OADC);
    }
    else
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && _SUPPORT_VARIABLE_PWM */
    {
        return (l_AdcResult.u16AdcVdda - C_OADC);
    }
} /* End of Get_AdcVdda() */

/*!*************************************************************************** *
 * Get_AdcVddd
 * \brief   Get IC VDDD value (ADC-LSB)
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_AdcResult.u16AdcVdda
 * *************************************************************************** *
 * \details in-line function to get ADC VDDD supply level.
 * *************************************************************************** */
static inline uint16_t Get_AdcVddd(void)
{
#if (_SUPPORT_PWM_MODE == TRIPLEPHASE_ALLPWM_MIRROR) || (_SUPPORT_PWM_MODE == BIPOLAR_TRIPHASE_ALLPWM_MIRROR) || \
    (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
    return ( (C_MIN_VDDD + C_MAX_VDDD) / 2U);
#else  /* (_SUPPORT_PWM_MODE == TRIPLEPHASE_ALLPWM_MIRROR) */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && _SUPPORT_VARIABLE_PWM               /* MMP220815-1 */
    if (g_u8TriplePWM != FALSE)
    {
        extern volatile ADC_RESULTS_3 l_AdcResult3;                             /* ADC results */
        return (l_AdcResult3.u16AdcVddd - C_OADC);
    }
    else
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && _SUPPORT_VARIABLE_PWM */
    {
        return (l_AdcResult.u16AdcVddd - C_OADC);
    }
#endif /* (_SUPPORT_PWM_MODE == TRIPLEPHASE_ALLPWM_MIRROR) */
} /* End of Get_AdcVdda() */
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) */

#if (_SUPPORT_ADC_BGD != FALSE)
/*!*************************************************************************** *
 * Get_AdcVaux
 * \brief   Get IC VAUX value (ADC-LSB)
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_AdcResult.u16AdcVaux
 * *************************************************************************** *
 * \details in-line function to get ADC VAUX supply level.
 * (MMP220307-1)
 * *************************************************************************** */
static inline uint16_t Get_AdcVaux(void)
{
    return (l_AdcResult.u16AdcVaux - C_OADC);
} /* End of Get_AdcVaux() */

/*!*************************************************************************** *
 * Get_AdcVbgd
 * \brief   Get IC VBGD value (ADC-LSB)
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_AdcResult.u16AdcVbgd
 * *************************************************************************** *
 * \details in-line function to get ADC VBGD supply level.
 * (MMP220307-1)
 * *************************************************************************** */
static inline uint16_t Get_AdcVbgd(void)
{
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && _SUPPORT_VARIABLE_PWM               /* MMP220815-1 */
    if (g_u8TriplePWM != FALSE)
    {
        extern volatile ADC_RESULTS_3 l_AdcResult3;                             /* ADC results */
        return (l_AdcResult3.u16AdcVbgd - C_OADC);
    }
    else
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && _SUPPORT_VARIABLE_PWM */
    {
        return (l_AdcResult.u16AdcVbgd - C_OADC);
    }
} /* End of Get_AdcVbgd() */
#endif /* (_SUPPORT_ADC_BGD != FALSE) */

#if (_SUPPORT_MOTION_DET == C_MOTION_DET_BEMF)
/*!*************************************************************************** *
 * Get_RawVoltagePhaseU
 * \brief   Get Phase U voltage
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (int16_t) Signed Phase-voltage 'U'
 * *************************************************************************** *
 * \details in-line function to get ADC Phase U voltage level.
 *          l_AdcResult.u16AdcCurrA = Motor Phase 'U' Voltage
 * *************************************************************************** */
static inline int16_t Get_RawVoltagePhaseU(void)
{
    extern uint16_t l_u16HighVoltOffset;
    extern volatile ADC_RESULTS l_AdcResult;                                    /* ADC results Phase Voltage 'U' */
    return ( (int16_t)(l_AdcResult.u16AdcCurrA - l_u16HighVoltOffset));
} /* End of Get_RawVoltagePhaseU() */

/*!*************************************************************************** *
 * Get_RawVoltagePhaseV
 * \brief   Get Phase V voltage
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (int16_t) Signed Phase-voltage 'V'
 * *************************************************************************** *
 * \details in-line function to get ADC Phase V voltage level.
 *          l_AdcResult.u16AdcVboost = Motor Phase 'V' Voltage
 * *************************************************************************** */
static inline int16_t Get_RawVoltagePhaseV(void)
{
    extern uint16_t l_u16HighVoltOffset;
    extern volatile ADC_RESULTS l_AdcResult;                                    /* ADC results Phase Voltage 'V' */
    return ( (int16_t)(l_AdcResult.u16AdcVboost - l_u16HighVoltOffset));
} /* End of Get_RawVoltagePhaseW() */

/*!*************************************************************************** *
 * Get_RawVoltagePhaseW
 * \brief   Get Phase W voltage
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (int16_t) Signed Phase-voltage 'W'
 * *************************************************************************** *
 * \details in-line function to get ADC Phase W voltage level.
 *          l_AdcResult.u16AdcCurrB = Motor Phase 'W' Voltage
 * *************************************************************************** */
static inline int16_t Get_RawVoltagePhaseW(void)
{
    extern uint16_t l_u16HighVoltOffset;
    extern volatile ADC_RESULTS l_AdcResult;                                    /* ADC results Phase Voltage 'W' */
    return ( (int16_t)(l_AdcResult.u16AdcCurrB - l_u16HighVoltOffset));
} /* End of Get_RawVoltagePhaseW() */
#endif /* (_SUPPORT_MOTION_DET == C_MOTION_DET_BEMF) */

#if (_SUPPORT_STALLDET_BZC != FALSE)
/*!*************************************************************************** *
 * Set_ZcTime
 * \brief   Set variable l_u16ZcTime
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16Value: Value for Zero-Cross time
 * \return  -
 * *************************************************************************** *
 * \details in-line function to set 'local' variable
 * *************************************************************************** */
static inline void Set_ZcTime(uint16_t u16Value)
{
    extern uint16_t l_u16ZcTime;
    l_u16ZcTime = u16Value;
} /* End of Set_ZcTime () */
#endif /* (_SUPPORT_STALLDET_BZC != FALSE) */

#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE) && FALSE
/*!*************************************************************************** *
 * Get_ResolverPosX
 * \brief   Get variable l_AdcResult.u16Resolver_X
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_AdcResult.u16Resolver_X
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_ResolverPosX(void)
{
    return (l_AdcResult.u16Resolver_X);
} /* End of Get_ResolverPosX() */

/*!*************************************************************************** *
 * Get_Resolver1PosY
 * \brief   Get variable l_AdcResult.u16Resolver_Y
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_AdcResult.u16Resolver_Y
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_ResolverPosY(void)
{
    return (l_AdcResult.u16Resolver_Y);
} /* End of Get_ResolverPosY() */

#if (_DEBUG_MLX90381 == FALSE) && (_SUPPORT_RESOLVER2_X_IO != PIN_FUNC_IO_NONE) && \
    (_SUPPORT_RESOLVER2_Y_IO != PIN_FUNC_IO_NONE)
/*!*************************************************************************** *
 * Get_Resolver1PosX
 * \brief   Get variable l_AdcResult.u16Resolver2_X
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_AdcResult.u16Resolver2_X
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_Resolver2PosX(void)
{
    return (l_AdcResult.u16Resolver2_X);
} /* End of Get_Resolver2PosX() */

/*!*************************************************************************** *
 * Get_Resolver2PosY
 * \brief   Get variable l_AdcResult.u16Resolver2_Y
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_AdcResult.u16Resolver2_Y
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_Resolver2PosY(void)
{
    return (l_AdcResult.u16Resolver2_Y);
} /* End of Get_Resolver2PosY() */
#endif /* (_DEBUG_MLX90381 == FALSE) && (_SUPPORT_RESOLVER2_X_IO != PIN_FUNC_IO_NONE) && (_SUPPORT_RESOLVER2_Y_IO != PIN_FUNC_IO_NONE) */
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */

#if (_SUPPORT_IO_DUT_SELECT_HVIO != FALSE) && (HVIO_DUT_SELECT == PIN_FUNC_IO_0)
/*!*************************************************************************** *
 * Get_IO0HV
 * \brief   Get variable l_AdcResult.u16AdcIO0HV
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_AdcResult.u16AdcIO0HV
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_IO0HV(void)
{
    return (l_AdcResult.u16AdcIO0HV);
} /* End of Get_IO0HV() */
#endif /* (_SUPPORT_IO_DUT_SELECT_HVIO != FALSE) && (HVIO_DUT_SELECT == PIN_FUNC_IO_0) */

#if (_SUPPORT_NTC != FALSE)
/*!*************************************************************************** *
 * Get_AdcNTC
 * \brief   Get variable l_AdcResult.u16AdcNTC of ADC
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_AdcResult.u16AdcNTC
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_AdcNTC(void)
{
    return (l_AdcResult.u16AdcNTC - C_OADC);
} /* End of Get_AdcNTC() */
#endif /* (_SUPPORT_NTC != FALSE) */

#endif /* ACT_ADC_H_ */

/* EOF */
