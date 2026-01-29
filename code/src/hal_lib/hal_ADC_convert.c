/*!*************************************************************************** *
 * \file        hal_ADC_convert.c
 * \brief       Hardware Abstraction Layer for ADC conversions handling
 *
 * \note        project MLX81160/33x/34x/35x
 *
 * \author      Marcel Braat
 *
 * \date        2024-03-22
 *
 * \version     2.0
 *
 * A list of functions:
 *  - Global Functions:
 *           -# HAL_ADC_Conv_Init()
 *           -# HAL_ADC_Conv_Vsupply()
 *           -# HAL_ADC_Conv_Vmotor()
 *           -# HAL_ADC_Conv_TempJ()
 *           -# HAL_ADC_Conv_Cmotor()
 *  - Internal Functions:
 *
 * \copyright   MELEXIS Microelectronic Integrated Systems
 *
 * Copyright (C) 2024-2024 Melexis N.V.
 * The Software is being delivered 'AS IS' and Melexis, whether explicitly or
 * implicitly, makes no warranty as to its Use or performance.
 * The user accepts the Melexis Firmware License Agreement.
 *
 * Melexis confidential & proprietary
 * *************************************************************************** *
 * Resources:
 *
 * *************************************************************************** */

/*!*************************************************************************** */
/*                           INCLUDES                                          */
/* *************************************************************************** */
#include "AppBuild.h"                                                           /* Application Build */

#include "hal_ADC.h"                                                            /* ADC HAL support */

#include "camculib/private_mathlib.h"                                           /* Private Math Library support */

/*!*************************************************************************** */
/*                           DEFINITIONS                                       */
/* *************************************************************************** */
/* l_u16HV_Corr = l_u16HV * (1 + tempGain); 1 (Base) --> (2^8)
 * mTempDiff = mTemp - OTempCal_EE
 * 81330/32/34:
 * if (mTempDiff > 0)
 *   tempGain = ((GainLo_HVI_EE / 2^11) * mTempDiff)/2^7;
 * else
 *   tempGain = ((GainHi_HVI_EE / 2^11) * mTempDiff)/2^7;
 * 81339/50:
 * if (mTempDiff > 0)
 *   tempGain = ((GainLo_HVI_EE / 2^14) * mTempDiff)/2^8;
 * else
 *   tempGain = ((GainHi_HVI_EE / 2^14) * mTempDiff)/2^8;
 * 81340/44/46:
 * if (mTempDiff > 0)
 *   tempGain = ((GainLo_HVI_EE / 2^11) * mTempDiff)/2^8;
 * else
 *   tempGain = ((GainHi_HVI_EE / 2^11) * mTempDiff)/2^8;
 */
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
#define C_TEMPCORR_BASE             256UL                                       /*!< Temperature correction Base-factor for HV (1 << (16 - EE_Gain-bits=8)) */
#define C_TEMPCORR_HV_DIV   ((1UL << (11U + 7U)) / (C_TEMPCORR_BASE * C_VOLTGAIN_DIV))  /*!< Temperature correction divisor High Voltage */
#define C_TEMPCORR_MC_DIV   ((1UL << (10U + 7U)) / (C_TEMPCORR_BASE * C_GMCURR_DIV))    /*!< Temperature correction divisor Motor Current */
#elif defined (__MLX81339__) || defined (__MLX81350__)
#define C_TEMPCORR_BASE             256UL                                       /*!< Temperature correction Base-factor for HV (1 << (16 - EE_Gain-bits=8)) */
#define C_TEMPCORR_HV_DIV   ((1UL << (14U + 8U)) / (C_TEMPCORR_BASE * C_VOLTGAIN_DIV))  /*!< Temperature correction divisor High Voltage */
#define C_TEMPCORR_MC_DIV   ((1UL << (14U + 8U)) / (C_TEMPCORR_BASE * C_GMCURR_DIV))    /*!< Temperature correction divisor Motor Current */
#elif defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#define C_TEMPCORR_BASE             16UL                                        /*!< Temperature correction Base-factor for HV (1 << (16 - EE_Gain-bits=12)) */
#define C_TEMPCORR_HV_DIV   ((1UL << (11U + 8U)) / (C_TEMPCORR_BASE * C_VOLTGAIN_DIV))  /*!< Temperature correction divisor High Voltage */
#define C_TEMPCORR_MC_DIV   ((1UL << (11U + 8U)) / (C_TEMPCORR_BASE * C_GMCURR_DIV))    /*!< Temperature correction divisor Motor Current */
#endif
#define C_VOLTGAIN_DIV_TC           (C_TEMPCORR_BASE * C_VOLTGAIN_DIV)          /*!< Voltage Gain divisor with temperature correction */
#define C_GMCURR_DIV_TC             (C_TEMPCORR_BASE * C_GMCURR_DIV)            /*!< Motor Current divisor with temperature correction */

#define C_TEMP_LPF_COEF             1024                                        /*!< Junction Temperature LPF Coefficient: n/32768 */

/* Debounce error filter; An error has to be detected twice in a row */
#define C_DEBFLT_ERR_TEMP_SENSOR    0x20U                                       /*!< Bit 5: Temperature sensor defect */
#define C_TEMPERATURE_JUMP          25                                          /*!< Maximum temperature "jump" per measurement-period: 25 degrees Celsius */
#define C_ERR_OTEMP_ERROR           ((uint8_t) 0x80U)                           /*!< Temperature sensor error */

/*! When CSA & OC are enabled, VSMF measurement error occurs (ground-shift due current consumption CSA/OC circuitry (JIRA: MLX81346-187) (MMP220128-1) */
#if defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
/* VSMF Offset can be measured by ADC (TA0) (_SUPPORT_ADC_VSMF_OFF), and is used with CSA/OC enabled */
#define _FIX_VSMF_OFFSET            FALSE                                       /*!< FALSE: VSMF Offset correction fix disabled; TRUE: VSMF Offset correction fix enabled */
#if defined (__MLX81340__)
#define C_VSMF_CSA_OC_CORRECTION    49U                                         /*!< VSMF Offset correction [10mV]: 490 mV */
#elif defined (__MLX81344__)
#define C_VSMF_CSA_OC_CORRECTION    15U                                         /*!< VSMF Offset correction [10mV]: 150 mV */
#elif defined (__MLX81346__)
#define C_VSMF_CSA_OC_CORRECTION    13U                                         /*!< VSMF Offset correction [10mV]: 130 mV */
#define C_VSMF52_CSA_OC_CORRECTION  26U                                         /*!< VSMF Offset correction [10mV]: 260 mV */
#endif
#else  /* defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */
#define _FIX_VSMF_OFFSET            FALSE                                       /*!< FALSE: VSMF Offset correction fix disabled; TRUE: VSMF Offset correction fix enabled */
#define C_VSMF_CSA_OC_CORRECTION    0U                                          /*!< VSMF Offset correction: 0 mV */
#endif /* defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */

extern volatile uint8_t g_e8ErrorOverTemperature;                               /*!< Status-flag over-temperature */

/*!*************************************************************************** */
/*                           GLOBAL & LOCAL VARIABLES                          */
/* *************************************************************************** */
#pragma space dp                                                                /* __TINY_SECTION__ */
/*!< ADC Last conversion */
volatile int16_t l_i16ChipTemperature = (255 - C_TEMPOFF);                      /*!< Chip internal temperature [C] */
volatile uint16_t l_u16MotorVoltage = 1200U;                                    /*!< Motor supply voltage [10mV] */

/*! ADC Gain's and Offsets */
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)  /*MMP39/MMP50*/
int16_t l_i16VsmOffset = C_OADC_VSMF;                                           /*!< Motor Voltage (Filtered) ADC-Offset (MMP200626-1) */
#elif defined (__MLX81160__) ||defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
uint16_t l_u16VsmOffset = C_OADC_VSMF;                                          /*!< Motor Voltage (Filtered) ADC-Offset (MMP200626-1) */
#endif
#pragma space none                                                              /* __TINY_SECTION__ */

#pragma space nodp                                                              /* __NEAR_SECTION__ */
/*!< ADC Last conversion */
volatile uint16_t l_u16SupplyVoltage = 1200U;                                   /*!< Supply Voltage [10mV] (ADC-Ref 2.5V) */
volatile uint16_t l_u16MotorCurrent_mA = 0U;                                    /*!< Motor Current [mA] */

/*! ADC Gain's and Offsets */
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)  /*MMP39/MMP50*/
uint16_t l_u16MCurrGain = C_GADC_MCUR;                                          /*!< Motor Current ADC-Gain */
#elif defined (__MLX81160__)
uint16_t l_u16MCurrGain1 = ((148U * C_ONE_DIV_RSHUNT) / (2048U / C_GMCURR_DIV));  /*!< ADC Current-gain #1; 1.48V * 1000 (mA/A) / 10 (Amplifier) * (1/Rshunt) * C_GMCURR_DIV/2048[LSB-ADC-range] */
uint16_t l_u16MCurrGain2 = ((148U * C_ONE_DIV_RSHUNT) / (2048U / C_GMCURR_DIV));  /*!< ADC Current-gain #2; 1.48V * 1000 (mA/A) / 10 (Amplifier) * (1/Rshunt) * C_GMCURR_DIV/2048[LSB-ADC-range] */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#if (_SUPPORT_CSA_HIGHGAIN != FALSE)
#if (C_ONE_DIV_RSHUNT < (65536 / 74))
uint16_t l_u16MCurrGain = ((74U * C_ONE_DIV_RSHUNT) / (2048U / C_GMCURR_DIV));  /*!< ADC Current-gain; 1.48V * 1000 (mA/A) / 20 (Amplifier) * (1/Rshunt) * C_GMCURR_DIV/2048[LSB-ADC-range] */
#elif (C_ONE_DIV_RSHUNT < (65536 / 74) * 2U)
uint16_t l_u16MCurrGain = (((74U / 2U) * C_ONE_DIV_RSHUNT) / (2048U / C_GMCURR_DIV));  /*!< ADC Current-gain; 1.48V * 1000 (mA/A) / 20 (Amplifier) * (1/Rshunt) */
#elif (C_ONE_DIV_RSHUNT < (65536 / 74) * 4U)
uint16_t l_u16MCurrGain = (((74U / 4U) * C_ONE_DIV_RSHUNT) / (2048U / C_GMCURR_DIV));  /*!< ADC Current-gain; 1.48V * 1000 (mA/A) / 20 (Amplifier) * (1/Rshunt) */
#else  /* C_ONE_DIV_RSHUNT */
#error "Invalid Shunt resistance (too low)"
#endif /* C_ONE_DIV_RSHUNT */
#else  /* (_SUPPORT_CSA_HIGHGAIN != FALSE) */
#if (C_ONE_DIV_RSHUNT < (65536 / 148))
uint16_t l_u16MCurrGain = ((148U * C_ONE_DIV_RSHUNT) / (2048U / C_GMCURR_DIV));  /*!< ADC Current-gain; 1.48V * 1000 (mA/A) / 10 (Amplifier) * (1/Rshunt) * C_GMCURR_DIV/2048[LSB-ADC-range] */
#elif (C_ONE_DIV_RSHUNT < (65536 / 148) * 2U)
uint16_t l_u16MCurrGain = (((148U / 2U) * C_ONE_DIV_RSHUNT) / (2048U / C_GMCURR_DIV));  /*!< ADC Current-gain; 1.48V * 1000 (mA/A) / 10 (Amplifier) * (1/Rshunt) */
#elif (C_ONE_DIV_RSHUNT < (65536 / 148) * 4U)
uint16_t l_u16MCurrGain = (((148U / 4U) * C_ONE_DIV_RSHUNT) / (2048U / C_GMCURR_DIV));  /*!< ADC Current-gain; 1.48V * 1000 (mA/A) / 10 (Amplifier) * (1/Rshunt) */
#else  /* C_ONE_DIV_RSHUNT */
#error "Invalid Shunt resistance (too low)"
#endif /* C_ONE_DIV_RSHUNT */
#endif /* (_SUPPORT_CSA_HIGHGAIN != FALSE) */
#endif /* defined (__MLX81xxx__) */
uint16_t l_u16VsmGain = C_GADC_VSMF;                                            /*!< Motor Voltage (Filtered) ADC-Gain */
uint16_t l_u16HighVoltGain = C_GADC_HV;                                         /*!< IC High Voltage ADC-Gain */
uint16_t l_u16HighVoltOffset = C_OADC_HV;                                       /*!< IC High Voltage ADC-Offset */
uint16_t l_u16LowVoltOffset = C_OADC_LV;                                        /*!< IC Low Voltage ADC-Offset */
#if (_SUPPORT_ADC_VBOOST != FALSE) && (defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__))
uint16_t l_u16VboostGain = C_GADC_VBOOST;                                       /*!< IC Boost Voltage ADC-Gain */
uint16_t l_u16VboostOffset = C_OADC;                                            /*!< IC Boost Voltage ADC-Offset */
#endif /* (_SUPPORT_ADC_VBOOST != FALSE) && (defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)) */
#if defined (__MLX81346__) && (_SUPPORT_APP_48V_ADC != FALSE)
uint16_t l_u16VsGain = C_GADC_HV;                                               /*!< IC Supply Voltage ADC-Gain */
uint16_t l_u16VsOffset = C_OADC_HV;                                             /*!< IC Supply Voltage ADC-Offset */
#endif /* defined (__MLX81346__) && (_SUPPORT_APP_48V_ADC != FALSE) */
uint16_t l_u16TempMidADC = C_ADC_TEMP_MID;                                      /*!< ADC value for mid-temperature */
static uint16_t l_u16TempGainHigh = C_GADC_TEMP;                                /*!< Temperature ADC-Gain (above mid) */
static uint16_t l_u16TempGainLow = C_GADC_TEMP;                                 /*!< Temperature ADC-Gain (below mid) */
static uint16_t l_u16RoomTemp = 35U;                                            /*!< Room Temperature: 35 C (MMP201204-1) */
#pragma space none                                                              /* __NEAR_SECTION__ */

/*!*************************************************************************** *
 * HAL_ADC_Conv_Init
 * \brief   Set ADC Calibration data
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details NV_Calibration area is valid.
 * Note:    This function is not allowed to be called during Non Volatile Memory Write
 *          (due to Non Volatile Memory reads)
 * *************************************************************************** *
 * - Call Hierarchy: AppInit()
 * - Cyclomatic Complexity: 7+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
void HAL_ADC_Conv_Init(void)
{
    /* Voltage & Current */
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
    /* VSMF */
    if (CalibrationParams.u8APP_TRIM18_GainVSMF != 0U)
    {
        l_u16VsmGain = CalibrationParams.u8APP_TRIM18_GainVSMF;
    }
#if defined (__MLX81339__) || defined (__MLX81350__)                            /* TODO[MMP39/MMP50] */
    l_i16VsmOffset = CalibrationParams.i8APP_TRIM20_OffVSMF;
#else  /* defined (__MLX81339__) || defined (__MLX81350__) */
    l_i16VsmOffset = (CalibrationParams.i8APP_TRIM18_OffHV + CalibrationParams.i8APP_TRIM20_OffVSMF);  /* MMP200626-1 */
#endif /* defined (__MLX81339__) || defined (__MLX81350__) */

    /* High Voltage */
    if (CalibrationParams.u8APP_TRIM20_GainHV != 0U)
    {
        l_u16HighVoltGain = CalibrationParams.u8APP_TRIM20_GainHV;
    }
    l_u16HighVoltOffset = CalibrationParams.i8APP_TRIM18_OffHV;

    /* Low Voltage */
    l_u16LowVoltOffset = CalibrationParams.i8APP_TRIM19_OffLV;

    /* Motor Current */
    if (CalibrationParams.u8APP_TRIM22_GainMCur != 0U)
    {
        l_u16MCurrGain = CalibrationParams.u8APP_TRIM22_GainMCur;
    }
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
#if (_SUPPORT_ADC_REF_MC_CALIB != FALSE)
    l_u16MCurrGain = p_MulDivU16_U16byU16byU16(l_u16MCurrGain, 150U, 250U);     /* Convert Motor Current Gain to 1.5Vref-units */
#endif /* (_SUPPORT_ADC_REF_MC_CALIB != FALSE) */
#endif
#elif defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    /* VSMF */
#if defined (__MLX81346__) && (_SUPPORT_APP_48V_ADC != FALSE)
    uint16_t u16Value =
        (((CalibrationParams.u4App_TRIM16_Gain_VSMF52_MSB) << 8) | (CalibrationParams.u8App_TRIM15_Gain_VSMF52_LSB));
#else  /* defined (__MLX81346__) && (_SUPPORT_APP_48V_ADC != FALSE) */
    uint16_t u16Value = CalibrationParams.u12App_TRIM19_GainVSMF;
#endif /* defined (__MLX81346__) && (_SUPPORT_APP_48V_ADC != FALSE) */
    if (u16Value != 0U)
    {
        l_u16VsmGain = u16Value;
    }
#if defined (__MLX81346__) && (_SUPPORT_APP_48V_ADC != FALSE)
    u16Value = CalibrationParams.u12App_TRIM16_O_VSMF52;
#else  /* defined (__MLX81346__) && (_SUPPORT_APP_48V_ADC != FALSE) */
#if defined (__MLX81340__) || defined (__MLX81344__)
    if (CalibrationParams.u8APP_TRIM03_CalibVersion >= C_NV_MLX_VER_4)
    {
        u16Value = (CalibrationParams.u4App_TRIM09_OffVSMF2_MSB << 4) | CalibrationParams.u4App_TRIM01_OffVSMF2_LSB;
    }
    else
#endif
    {
        u16Value = CalibrationParams.u12App_TRIM01_OffVSMF;
    }
#endif /* defined (__MLX81346__) && (_SUPPORT_APP_48V_ADC != FALSE) */
    if (u16Value != 0U)
    {
        l_u16VsmOffset = u16Value;
    }

    /* High-voltage / Phase */
#if defined (__MLX81346__) && (_SUPPORT_APP_48V_ADC != FALSE)
    u16Value = CalibrationParams.u12App_TRIM28_Gain_HVI;
    if (u16Value != 0U)
    {
        l_u16VsGain = u16Value;
    }
    u16Value = (CalibrationParams.u12App_TRIM13_Gain_VPHASE52);
#else  /* defined (__MLX81346__) && (_SUPPORT_APP_48V_ADC != FALSE) */
    u16Value = CalibrationParams.u12App_TRIM28_Gain_HVI;
#endif /* defined (__MLX81346__) && (_SUPPORT_APP_48V_ADC != FALSE) */
    if (u16Value != 0U)
    {
        l_u16HighVoltGain = u16Value;
    }
#if defined (__MLX81346__) && (_SUPPORT_APP_48V_ADC != FALSE)
    u16Value = (CalibrationParams.u8App_TRIM29_O_HVI_MSB << 4) | (CalibrationParams.u4App_TRIM28_O_HVI_LSB);
    if (u16Value != 0U)
    {
        l_u16VsOffset = u16Value;
    }
    u16Value = (CalibrationParams.u8App_TRIM14_O_VPHASE52_MSB << 4) | (CalibrationParams.u4App_TRIM13_O_VPHASE52_LSB);
#else  /* defined (__MLX81346__) && (_SUPPORT_APP_48V_ADC != FALSE) */
    u16Value = (CalibrationParams.u8App_TRIM29_O_HVI_MSB << 4) | (CalibrationParams.u4App_TRIM28_O_HVI_LSB);
#endif /* defined (__MLX81346__) && (_SUPPORT_APP_48V_ADC != FALSE) */
    if (u16Value != 0U)
    {
        l_u16HighVoltOffset = u16Value;
    }
    u16Value = CalibrationParams.u12App_TRIM31_O_LVI;
    if (u16Value != 0U)
    {
        l_u16LowVoltOffset = u16Value;
    }

#if (_SUPPORT_ADC_VBOOST != FALSE) && (defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__))
    /* Boost Voltage */
#if defined (__MLX81346__) && (_SUPPORT_APP_48V_ADC != FALSE)
    u16Value = CalibrationParams.u12App_TRIM37_Gain_VBOOST64;
#else  /* defined (__MLX81346__) && (_SUPPORT_APP_48V_ADC != FALSE) */
    u16Value = ((CalibrationParams.u4App_TRIM26_Gain_VBOOST_MSB) << 8) |
               (CalibrationParams.u8App_TRIM25_Gain_VBOOST_LSB);
#endif /* defined (__MLX81346__) && (_SUPPORT_APP_48V_ADC != FALSE) */
    if (u16Value != 0U)
    {
        l_u16VboostGain = u16Value;
    }
#if defined (__MLX81346__) && (_SUPPORT_APP_48V_ADC != FALSE)
    u16Value = ((CalibrationParams.u8App_TRIM06_O_VBOOST64_MSB) << 4) | CalibrationParams.u4App_TRIM37_O_VBOOST64_LSB;
#else  /* defined (__MLX81346__) && (_SUPPORT_APP_48V_ADC != FALSE) */
    u16Value = CalibrationParams.u12App_TRIM26_O_VBOOST;
#endif /* defined (__MLX81346__) && (_SUPPORT_APP_48V_ADC != FALSE) */
    if (u16Value != 0U)
    {
        l_u16VboostOffset = u16Value;
    }
#endif /* (_SUPPORT_ADC_VBOOST != FALSE) && (defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)) */

#if defined (__MLX81160__)
    /* Current Sense #1 */
    u16Value = (CalibrationParams.u4App_TRIM26_Gain_CURR1_MSB << 8) | (CalibrationParams.u8App_TRIM25_Gain_CURR1_LSB);
    if (u16Value != 0U)
    {
        l_u16MCurrGain1 = p_MulDivU16_U16byU16byU16(u16Value, C_ONE_DIV_RSHUNT, 50U);   /* 50U from Calibration spec */
    }
    /* Current Sense #2 */
    u16Value = CalibrationParams.u12App_TRIM23_Gain_CURR2;
    if (u16Value != 0U)
    {
        l_u16MCurrGain2 = p_MulDivU16_U16byU16byU16(u16Value, C_ONE_DIV_RSHUNT, 50U);   /* 50U from Calibration spec */
    }
#else  /* defined (__MLX81160__) */
       /* Current Sense */
#if (defined (__MLX81340B01__) || defined (__MLX81344B01__)) && _SUPPORT_CSA_HIGHGAIN
    IO_PORT_CURR_SENS |= B_PORT_CURR_SENS_CSA_HIGHGAIN;
    u16Value = CalibrationParams.u12App_TRIM08_Gain_CURR_HG;
#else  /* (defined (__MLX81340B01__) || defined (__MLX81344B01__)) && _SUPPORT_CSA_HIGHGAIN*/
    u16Value = CalibrationParams.u12App_TRIM23_Gain_CURR;
#endif /* (defined (__MLX81340B01__) || defined (__MLX81344B01__)) && _SUPPORT_CSA_HIGHGAIN*/
    if (u16Value != 0U)
    {
        l_u16MCurrGain = p_MulDivU16_U16byU16byU16(u16Value, C_ONE_DIV_RSHUNT, 50U);   /* 50U from Calibration specification */
    }
#endif /* defined (__MLX81160__) */
#endif

    /* Temperature Sensor */
    if (CalibrationParams.u16APP_TRIM02_OTempCal != 0U)
    {
        l_u16TempMidADC = CalibrationParams.u16APP_TRIM02_OTempCal;
    }
    if (CalibrationParams.u8APP_TRIM04_GainTempHigh != 0U)
    {
        l_u16TempGainHigh = CalibrationParams.u8APP_TRIM04_GainTempHigh;
    }
    if (CalibrationParams.u8APP_TRIM04_GainTempLow != 0U)
    {
        l_u16TempGainLow = CalibrationParams.u8APP_TRIM04_GainTempLow;
    }
    if (CalibrationParams.u8APP_TRIM00_TempMid != 0U)
    {
        l_u16RoomTemp = CalibrationParams.u8APP_TRIM00_TempMid;                 /* MMP201204-1 */
    }
} /* End of HAL_ADC_Conv_Init() */

#if (_SUPPORT_ADC_REF_HV_CALIB != FALSE)
/*!*************************************************************************** *
 * HAL_ADC_ConvHvGain
 * \brief   Convert HV-channel gain
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16Mul: Multiply-factor
 * \param   [in] u16Div: Division-factor
 * \return  -
 * *************************************************************************** *
 * \details Correct HV-channels Gain-factor (different ADC Vref)
 * *************************************************************************** *
 * - Call Hierarchy: ADC_ReferenceCalibration()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
void HAL_ADC_ConvHvGain( uint16_t u16Mul, uint16_t u16Div)
{
    l_u16HighVoltGain = p_MulDivU16_U16byU16byU16(l_u16HighVoltGain, u16Mul, u16Div);
    l_u16VsmGain = p_MulDivU16_U16byU16byU16(l_u16VsmGain, u16Mul, u16Div);
} /* End of HAL_ADC_ConvHvGain() */
#endif /* (_SUPPORT_ADC_REF_HV_CALIB != FALSE) */

/*!*************************************************************************** *
 * HAL_ADC_Conv_Vsupply
 * \brief   Get Supply-voltage [10mV], based on ADC sampling of chip supply pin
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16TempJ: IC Junction Temperature [ADC-LSB]
 * \param   [in] u16SupplyVoltage: Supply voltage [ADC-LSB]
 * \return  (uint16_t) Supply voltage [10mV]
 * *************************************************************************** *
 * \details Set local and return IC supply voltage in 10mV units
 * Note:    This function is not allowed to be called during Non Volatile Memory Write
 *          (due to Non Volatile Memory reads) (_SUPPORT_ADC_TEMPCOMP).
 * *************************************************************************** *
 * - Call Hierarchy: ADC_Conv_Vsupply
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
uint16_t HAL_ADC_Conv_Vsupply(uint16_t u16TempJ, uint16_t u16SupplyVoltage)
{
#if (_SUPPORT_ADC_TEMPCOMP != FALSE)                                            /* MMP200626-1 */
    int16_t i16TempDiff = (u16TempJ - l_u16TempMidADC);
#if defined (__MLX81346__) && (_SUPPORT_APP_48V_ADC != FALSE)
    uint16_t u16GainTC = (l_u16VsGain * C_TEMPCORR_BASE);
#else  /* defined (__MLX81346__) && (_SUPPORT_APP_48V_ADC != FALSE) */
    uint16_t u16GainTC = (l_u16HighVoltGain * C_TEMPCORR_BASE);
#endif /* defined (__MLX81346__) && (_SUPPORT_APP_48V_ADC != FALSE) */
    if (i16TempDiff > 0)
    {
        /* Below Mid-Temperature (25/35C) */
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81350__)
        u16GainTC +=
            (int16_t)(p_MulI32_I16byI16(i16TempDiff,
                                        CalibrationParams.i8APP_TRIM25_GainHV_LowT) / (int16_t)C_TEMPCORR_HV_DIV);
#elif defined (__MLX81339__)
        u16GainTC +=
            (int16_t)(p_MulI32_I16byI16(i16TempDiff,
                                        CalibrationParams.i8APP_TRIM25_GainVSMF_LowT) / (int16_t)C_TEMPCORR_HV_DIV);
#else
        u16GainTC +=
            (int16_t)(p_MulI32_I16byI16(i16TempDiff,
                                        CalibrationParams.i8App_TRIM27_GainLo_HVI) / (int16_t)C_TEMPCORR_HV_DIV);
#endif
    }
    else
    {
        /* Above Mid-Temperature (25/35C) */
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81350__)
        u16GainTC +=
            (int16_t)(p_MulI32_I16byI16(i16TempDiff,
                                        CalibrationParams.i8APP_TRIM25_GainHV_HighT) / (int16_t)C_TEMPCORR_HV_DIV);
#elif defined (__MLX81339__)
        u16GainTC +=
            (int16_t)(p_MulI32_I16byI16(i16TempDiff,
                                        CalibrationParams.i8APP_TRIM25_GainVSMF_HighT) / (int16_t)C_TEMPCORR_HV_DIV);
#else
        u16GainTC +=
            (int16_t)(p_MulI32_I16byI16(i16TempDiff,
                                        CalibrationParams.i8App_TRIM27_GainHi_HVI) / (int16_t)C_TEMPCORR_HV_DIV);
#endif
    }
    l_u16SupplyVoltage =
        (uint16_t)((p_MulU32_U16byU16(u16SupplyVoltage, u16GainTC) + (C_VOLTGAIN_DIV_TC / 2U)) / C_VOLTGAIN_DIV_TC);  /* Including rounding */
#else  /* (_SUPPORT_ADC_TEMPCOMP != FALSE) */
    l_u16SupplyVoltage =
        (uint16_t)((p_MulU32_U16byU16(u16SupplyVoltage, l_u16HighVoltGain) + (C_VOLTGAIN_DIV / 2)) / C_VOLTGAIN_DIV);  /* Including rounding */
    (void)u16TempJ;
#endif /* (_SUPPORT_ADC_TEMPCOMP != FALSE) */
    return (l_u16SupplyVoltage);
} /* End of HAL_ADC_Conv_Vsupply() */

/*!*************************************************************************** *
 * HAL_ADC_Conv_Vmotor
 * \brief   Get Motor-voltage [10mV], based on ADC sampling of Motor supply pin
 *           (VsmF)
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16TempJ: IC Junction Temperature [ADC-LSB]
 * \param   [in] u16MotorVoltage: Motor Voltage [ADC-LSB]
 * \return  (uint16_t) Motor voltage [10mV]
 * *************************************************************************** *
 * \details Set local and return IC motor voltage in 10mV units
 * Note:    This function is not allowed to be called during Non Volatile Memory Write
 *          (due to Non Volatile Memory reads) (_SUPPORT_ADC_TEMPCOMP).
 * *************************************************************************** *
 * - Call Hierarchy: ADC_Conv_Vmotor
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
uint16_t HAL_ADC_Conv_Vmotor(uint16_t u16TempJ, uint16_t u16MotorVoltage)
{
#if (_SUPPORT_ADC_TEMPCOMP != FALSE)                                            /* MMP200626-1 */
    int16_t i16TempDiff = (u16TempJ - l_u16TempMidADC);
    uint16_t u16GainTC = (l_u16VsmGain * C_TEMPCORR_BASE);
    if (i16TempDiff > 0)
    {
        /* Below Mid-Temperature (25/35C) */
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81350__)
        u16GainTC +=
            (int16_t)(p_MulI32_I16byI16(i16TempDiff,
                                        CalibrationParams.i8APP_TRIM25_GainHV_LowT) / (int16_t)C_TEMPCORR_HV_DIV);
#elif defined (__MLX81339__)
        u16GainTC +=
            (int16_t)(p_MulI32_I16byI16(i16TempDiff,
                                        CalibrationParams.i8APP_TRIM25_GainVSMF_LowT) / (int16_t)C_TEMPCORR_HV_DIV);
#else
        u16GainTC +=
            (int16_t)(p_MulI32_I16byI16(i16TempDiff,
                                        CalibrationParams.i8App_TRIM18_GainLo_VSMF) / (int16_t)C_TEMPCORR_HV_DIV);
#endif
    }
    else
    {
        /* Above Mid-Temperature (25/35C) */
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81350__)
        u16GainTC +=
            (int16_t)(p_MulI32_I16byI16(i16TempDiff,
                                        CalibrationParams.i8APP_TRIM25_GainHV_HighT) / (int16_t)C_TEMPCORR_HV_DIV);
#elif defined (__MLX81339__)
        u16GainTC +=
            (int16_t)(p_MulI32_I16byI16(i16TempDiff,
                                        CalibrationParams.i8APP_TRIM25_GainVSMF_HighT) / (int16_t)C_TEMPCORR_HV_DIV);
#else
        u16GainTC +=
            (int16_t)(p_MulI32_I16byI16(i16TempDiff,
                                        CalibrationParams.i8App_TRIM18_GainHi_VSMF) / (int16_t)C_TEMPCORR_HV_DIV);
#endif
    }
    l_u16MotorVoltage =
        (uint16_t)((p_MulU32_U16byU16(u16MotorVoltage, u16GainTC) + (C_VOLTGAIN_DIV_TC / 2U)) / C_VOLTGAIN_DIV_TC);  /* Including rounding */
#if (_FIX_VSMF_OFFSET != FALSE)
    if ( (IO_PORT_CURR_SENS & (B_PORT_CURR_SENS_EN_OC | B_PORT_CURR_SENS_EN_CSA)) != 0U)
    {
#if defined (__MLX81346__) && (_SUPPORT_APP_48V_ADC != FALSE)
        l_u16MotorVoltage -= C_VSMF52_CSA_OC_CORRECTION;
#else  /* defined (__MLX81346__) && (_SUPPORT_APP_48V_ADC != FALSE) */
        l_u16MotorVoltage -= C_VSMF_CSA_OC_CORRECTION;
#endif /* defined (__MLX81346__) && (_SUPPORT_APP_48V_ADC != FALSE) */
    }
#endif /* (_FIX_VSMF_OFFSET != FALSE) */
#else  /* (_SUPPORT_ADC_TEMPCOMP != FALSE) */
    l_u16MotorVoltage =
        (uint16_t)((p_MulU32_U16byU16(u16MotorVoltage, l_u16VsmGain) + (C_VOLTGAIN_DIV / 2)) / C_VOLTGAIN_DIV);  /* Including rounding */
    (void)u16TempJ;
#endif /* (_SUPPORT_ADC_TEMPCOMP != FALSE) */
    return (l_u16MotorVoltage);
} /* End of HAL_ADC_Conv_Vmotor() */

/*!*************************************************************************** *
 * HAL_ADC_Conv_TempJ
 * \brief   Get Chip Junction temperature [C]
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16TempJ: IC Junction Temperature [ADC-LSB]
 * \param   [in] u16Init: FALSE: Check new temperature measurement against previous
 *                               measurement, and allow a maximum temperature increase.
 *                        TRUE: Initial measurement (no check)
 * \return  (int16_t) IC junction temperature in degrees Celsius
 * *************************************************************************** *
 * \details Set local and return IC junction temperature in degrees Celsius
 * *************************************************************************** *
 * - Call Hierarchy: ADC_Conv_TempJ
 * - Cyclomatic Complexity: 7+1
 * - Nesting: 4
 * - Function calling: 1 (p_LpfI16_I16byI16())
 * *************************************************************************** */
int16_t HAL_ADC_Conv_TempJ(uint16_t u16TempJ, uint16_t u16Init)
{
    static int32_t i32TemperatureLPF = 0;
    static uint8_t e8ErrorDebounceFilter = FALSE;
    uint16_t u16ChipTemperatureSensor = u16TempJ;

    if ( (u16ChipTemperatureSensor != C_ADC_MIN) && (u16ChipTemperatureSensor != C_ADC_MAX) )
    {
        int16_t i16NewChipTemperature, i16ChipTempDelta;

        if (u16ChipTemperatureSensor < l_u16TempMidADC)
        {
            /* Temperature above "Mid" (35C) (MMP201204-1) */
            /* i16NewChipTemperature = (int16_t) p_MulDivU16_U16byU16byU16( (l_u16TempMidADC - u16ChipTemperatureSensor),
             *                                                           (150 - 35), (l_u16TempMidADC - l_u16TempHighADC)) + CalibrationParams.u8APP_TRIM03_TempMid; */
#if defined (C_GTEMP_SDIV)
            i16NewChipTemperature = (int16_t)(l_u16RoomTemp +
                                              (uint16_t) (p_MulU32_U16byU16( (l_u16TempMidADC - u16ChipTemperatureSensor),
                                                                             l_u16TempGainHigh) >> C_GTEMP_SDIV));
#else
            i16NewChipTemperature = (int16_t)(l_u16RoomTemp +
                                              p_MulDivU16_U16byU16byU16( (l_u16TempMidADC - u16ChipTemperatureSensor),
                                                                         l_u16TempGainHigh, C_GTEMP_DIV));
#endif
        }
        else
        {
            /* Temperature below 35C (MMP201204-1) */
            /* i16NewChipTemperature = CalibrationParams.u8APP_TRIM03_TempMid - (int16_t) p_MulDivU16_U16byU16byU16( (u16ChipTemperatureSensor - l_u16TempMidADC),
             *                                                                        (35 - (-40)), (l_u16TempLowADC - l_u16TempMidADC)); */
#if defined (C_GTEMP_SDIV)
            i16NewChipTemperature = (int16_t)(l_u16RoomTemp -
                                              (uint16_t) (p_MulU32_U16byU16( (u16ChipTemperatureSensor - l_u16TempMidADC),
                                                                             l_u16TempGainLow) >> C_GTEMP_SDIV));
#else
            i16NewChipTemperature = (int16_t)(l_u16RoomTemp -
                                              p_MulDivU16_U16byU16byU16( (u16ChipTemperatureSensor - l_u16TempMidADC),
                                                                         l_u16TempGainLow, C_GTEMP_DIV));
#endif
        }

        if (u16Init == FALSE)
        {
            i16ChipTempDelta = i16NewChipTemperature - l_i16ChipTemperature;    /* Delta-temperature = new-temperature - previous-temperature */
            if (i16ChipTempDelta < 0)
            {
                i16ChipTempDelta = -i16ChipTempDelta;                           /* Absolute temperature change */
            }
            if (i16ChipTempDelta > C_TEMPERATURE_JUMP)                          /* Temperature change small, then accept new temperature */
            {
                /* Abnormal temperature change; Allow only 1 degree change */
                if (i16NewChipTemperature > l_i16ChipTemperature)               /* To great temperature change; Check temperature change "direction" */
                {
                    i16NewChipTemperature = l_i16ChipTemperature + 1;           /* Increase by one degree */
                }
                else
                {
                    i16NewChipTemperature = l_i16ChipTemperature - 1;           /* Decrease by one degree */
                }
            }
        }
        else
        {
            /* First time; Setup Temperature LPF */
            i32TemperatureLPF = ((int32_t)i16NewChipTemperature) << 16;
            l_i16ChipTemperature = i16NewChipTemperature;
        }
        l_i16ChipTemperature = p_LpfI16_I16byI16(&i32TemperatureLPF,
                                                 C_TEMP_LPF_COEF,
                                                 (i16NewChipTemperature - l_i16ChipTemperature));
        e8ErrorDebounceFilter &= (uint8_t) ~C_DEBFLT_ERR_TEMP_SENSOR;
        g_e8ErrorOverTemperature &= ~C_ERR_OTEMP_ERROR;
    }
    else
    {
        if ( (e8ErrorDebounceFilter & (uint8_t)C_DEBFLT_ERR_TEMP_SENSOR) != 0U)
        {
            /* Second time invalid temperature readout */
            g_e8ErrorOverTemperature |= C_ERR_OTEMP_ERROR;
        }
        else
        {
            e8ErrorDebounceFilter |= (uint8_t)C_DEBFLT_ERR_TEMP_SENSOR;         /* First time */
        }
    }
    return (l_i16ChipTemperature);
} /* End of HAL_ADC_Conv_TempJ() */

/*!*************************************************************************** *
 * HAL_ADC_Conv_Cmotor
 * \brief   Get Motor Driver Current [mA]
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16TempJ: IC Junction Temperature [ADC-LSB]
 * \param   [in] u16MotorCurrent: Motor Current [ADC-LSB]
 * \return  (uint16_t) Motor Driver current in [mA]
 * *************************************************************************** *
 * \details Get the Motor Driver current in mA
 * Note:    This function is not allowed to be called during Non Volatile Memory Write
 *          (due to Non Volatile Memory reads) (_SUPPORT_ADC_TEMPCOMP).
 * *************************************************************************** *
 * - Call Hierarchy: ADC_Conv_Cmotor
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 1 (ADC_GetRawMotorDriverCurrent())
 * *************************************************************************** */
uint16_t HAL_ADC_Conv_Cmotor(uint16_t u16TempJ, uint16_t u16MotorCurrent)
{
#if (_SUPPORT_ADC_TEMPCOMP != FALSE)                                            /* MMP200626-1 */
    int16_t i16TempDiff = (u16TempJ - l_u16TempMidADC);
#if defined (__MLX81160__)
    uint16_t u16GainTC = (l_u16MCurrGain1 * C_TEMPCORR_BASE);
#else  /* defined (__MLX81160__) */
    uint16_t u16GainTC = (l_u16MCurrGain * C_TEMPCORR_BASE);
#endif /* defined (__MLX81160__) */
    if (i16TempDiff > 0)
    {
        /* Below Mid-Temperature (25/35C) */
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
        u16GainTC +=
            (int16_t)(p_MulI32_I16byI16(i16TempDiff,
                                        CalibrationParams.i8APP_TRIM24_GainMCur_LowT) / (int16_t)C_TEMPCORR_MC_DIV);
#elif defined (__MLX81160__)
        u16GainTC +=
            (int16_t)(p_MulI32_I16byI16(i16TempDiff,
                                        CalibrationParams.i8App_TRIM24_GainLo_CURR1) / (int16_t)C_TEMPCORR_MC_DIV);
#else /* defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */
        u16GainTC +=
            (int16_t)(p_MulI32_I16byI16(i16TempDiff,
                                        CalibrationParams.i8App_TRIM22_GainLo_CURR) / (int16_t)C_TEMPCORR_MC_DIV);
#endif
    }
    else
    {
        /* Above Mid-Temperature (25/35C) */
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
        u16GainTC +=
            (int16_t)(p_MulI32_I16byI16(i16TempDiff,
                                        CalibrationParams.i8APP_TRIM24_GainMCur_HighT) / (int16_t)C_TEMPCORR_MC_DIV);
#elif defined (__MLX81160__)
        u16GainTC +=
            (int16_t)(p_MulI32_I16byI16(i16TempDiff,
                                        CalibrationParams.i8App_TRIM25_GainHi_CURR1) / (int16_t)C_TEMPCORR_MC_DIV);
#else /* defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */
        u16GainTC +=
            (int16_t)(p_MulI32_I16byI16(i16TempDiff,
                                        CalibrationParams.i8App_TRIM22_GainHi_CURR) / (int16_t)C_TEMPCORR_MC_DIV);
#endif
    }
    l_u16MotorCurrent_mA =
        (uint16_t)((p_MulU32_U16byU16(u16MotorCurrent, u16GainTC) + (C_GMCURR_DIV_TC / 2U)) / C_GMCURR_DIV_TC);  /* Including rounding */
#else  /* (_SUPPORT_ADC_TEMPCOMP != FALSE) */
#if defined (__MLX81160__)
    l_u16MotorCurrent_mA =
        (uint16_t)((p_MulU32_U16byU16(u16MotorCurrent, l_u16MCurrGain1) + (C_GMCURR_DIV / 2U)) / C_GMCURR_DIV);
    (void)u16TempJ;
#else  /* defined (__MLX81160__) */
#if (defined (__MLX81340B01__) || defined (__MLX81344B01__))
    if ( (IO_PORT_CURR_SENS & B_PORT_CURR_SENS_CSA_HIGHGAIN) != 0U)
    {
        l_u16MotorCurrent_mA =
            (uint16_t)((p_MulU32_U16byU16(u16MotorCurrent,
                                          l_u16MCurrGain) + (C_GMCURR_DIV_HG / 2U)) / (int16_t)C_GMCURR_DIV_HG);
    }
    else
#endif /* (defined (__MLX81340B01__) || defined (__MLX81344B01__)) */
    {
        l_u16MotorCurrent_mA =
            (uint16_t)((p_MulU32_U16byU16(u16MotorCurrent,
                                          l_u16MCurrGain) + (C_GMCURR_DIV / 2U)) / (int16_t)C_GMCURR_DIV);
    }
#endif /* defined (__MLX81160__) */
    (void)u16TempJ;
#endif /* (_SUPPORT_ADC_TEMPCOMP != FALSE) */
    return (l_u16MotorCurrent_mA);
} /* End of HAL_ADC_Conv_Cmotor() */
