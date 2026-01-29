/**
 * @file
 * @brief LIN auto addressing library
 * @internal
 *
 * @copyright (C) 2020-2021 Melexis N.V.
 *
 * Melexis N.V. is supplying this code for use with Melexis N.V. processor based microcontrollers only.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY,
 * INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.  MELEXIS N.V. SHALL NOT IN ANY CIRCUMSTANCES,
 * BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 * @endinternal
 *
 * @ingroup fw_lin_aa
 *
 * @details Private function implementations of the LIN auto addressing library
 */

#ifndef FW_LIN_AUTO_ADDRESSING_PRIVATE_H
#define FW_LIN_AUTO_ADDRESSING_PRIVATE_H

#include "io.h"
#include "compiler_abstraction.h"

/** LIN AA measurement result struct */
typedef struct LINShunt_s {
    int16_t commonmode;             /**< common mode measurement */
    int16_t differential;           /**< differential mode measurement (x8) */
} LINShunt_t;                       /**< LIN AA measurement result type */

/** scaling factor for VOUT of LIN AA amplifier (50 LSB/mA) */
#define SAADMUser 50

/** adc frequency used by this module in kHz */
#define ADCFREQUENCYKHZ 4000u

/** current limit for preselection phase -> 1.2mA (50 LSB/mA * 1.2mA -> 60LSB) */
#define LIN_AA_CURRLIMIT_PRE (int16_t)(1.2 * SAADMUser)

/** current limit for selection phase -> 1.2mA (50 LSB/mA * 1.2mA -> 60LSB) */
#define LIN_AA_CURRLIMIT_SEL (int16_t)(1.2 * SAADMUser)

/** multiplication factor applied to differential mode samples (note value fixed by fw_linaa_DoShuntMeasurement) */
#define ADC_DM_MULTIPLIER (1 << 3)

/* flags in LIN_AA_AutoAddressingFlags */
/** LIN_AA_AutoAddressingFlags bit0: 0: slave is not addressed yet; 1: slave is addressed */
#define LIN_AA_SLAVEADDRESSED 0x01u
/** LIN_AA_AutoAddressingFlags bit2: 0: slave in the running for Ishunt3; 1: slave not in the running for this break */
#define LIN_AA_SLAVEWAITING 0x04u
/** LIN_AA_AutoAddressingFlags bit3: 0: slave is not the last in line; 1: slave is the last in line */
#define LIN_AA_LASTSLAVE 0x08u
/** LIN_AA_AutoAddressingFlags bit4: 0: HW Connection is ok; 1: HW Connection is wrong */
#define LIN_AA_INVALID_HW_CONNECT 0x10u
/** LIN_AA_AutoAddressingFlags bit7: 0: LIN AA disabled; 1: LIN AA enabled */
#define LIN_AA_AUTOADDRESSENABLE 0x80u

/** LIN diagnostic message time-out */
#define DIAG_FRAME_TIMEOUT          (1.4 * (34 + 10 * (8 + 1)))

/* LIN AA Bus Shunt Method 0xB5 sub function enum */
typedef enum LinAaBsmSubFunction_e {
    LIN_AA_SUBFUNCTION1 = 1u,       /**< SNPD sub function ID 0x01 "All BSM-nodes enter the unconfigured mode" */
    LIN_AA_SUBFUNCTION2,            /**< SNPD sub function ID 0x02 "Informing all slaves about the next NAD" */
    LIN_AA_SUBFUNCTION3,            /**< SNPD sub function ID 0x03 "Store the assigned NADs in to the NVM of the slaves, if available" */
    LIN_AA_SUBFUNCTION4             /**< SNPD sub function ID 0x04 "Informing all slaves that the procedure is finished" */
} LinAaBsmSubFunction_t;            /**< LIN AA Bus Shunt Method 0xB5 sub function type */

/** LIN AA Bus Shunt Method Slave Node Position Detection step enum */
typedef enum LinAaBsmSnpdStep_e {
    LIN_AA_BSM_SNPD_STEP1 = 1u,     /**< Step 1: Switching off all Pull-Ups and all current sources */
    LIN_AA_BSM_SNPD_STEP2,          /**< Step 2: Start offset measurement (Ishunt1) */
    LIN_AA_BSM_SNPD_STEP3,          /**< Step 3: switching on current source 1  on LIN_IN */
    LIN_AA_BSM_SNPD_STEP4,          /**< Step 4: Start measurement 1 (Ishunt2) */
    LIN_AA_BSM_SNPD_STEP5,          /**< Step 5: all not pre-selected (first action A) nodes are switching off their current source 1
                                     *           all pre-selected (second action B) nodes are switching on their current source 2 on LIN_IN */
    LIN_AA_BSM_SNPD_STEP6,          /**< Step 6: Start measurement 2 (Ishunt3) */
    LIN_AA_BSM_SNPD_STEP7           /**< Step 7: Switching off all current sources and switching on the LIN termination */
} LinAaBsmSnpdStep_t;               /**< LIN AA Bus Shunt Method Slave Node Position Detection Step type */

/** Slave Node Position Detection method : Bus Shunt Method 2
 * @note: only Bus Shunt Method 2 (BSM2) is supported as it is not allowed to use BSM1 on Melexis IC's.
 */
#define SNPD_METHOD_BSM2            0xF1u

#if defined MLX81330 || defined MLX81332
#define ADC_DATA_16_BIT 1
#endif

#ifdef HAS_HW_EC_ADC
/** adc sbase channel configuration for the common mode measurement */
#define CM_MEAS_SAMPLE {{                \
    .adcEos=ADC_NO_SIGN,                 \
    .adcSource=ADC_SRC_TYPE_APPLICATION, \
    .adcChan=ADC_SIG_LINAA_CMO,          \
    .adcType=ADC_TYPE_CYCLIC,            \
    .adcTrig=ADC_CTIMER0_TIM3            \
}}
/** adc sbase channel configuration for the differential mode measurement */
#define DM_MEAS_SAMPLE {{                \
    .adcEos=ADC_NO_SIGN,                 \
    .adcSource=ADC_SRC_TYPE_APPLICATION, \
    .adcChan=ADC_SIG_LINAA_OUT,          \
    .adcType=ADC_TYPE_CYCLIC,            \
    .adcTrig=ADC_CTIMER0_TIM3            \
}}
#else
#ifdef ADC_DATA_16_BIT
/** adc sbase channel configuration for the common mode measurement */
#define CM_MEAS_SAMPLE {{      \
    .adcEosSign=ADC_NO_SIGN,   \
    .adcChan=ADC_SIG_LINVCMO,  \
    .adcRef=ADC_VREF_2_5V,     \
    .adcTrig=ADC_CTIMER0_TIM3, \
    .adcDiv=ADC_DIV_DISABLED   \
}}
/** adc sbase channel configuration for the differential mode measurement */
#define DM_MEAS_SAMPLE {{      \
    .adcEosSign=ADC_NO_SIGN,   \
    .adcChan=ADC_SIG_LINAAMP,  \
    .adcRef=ADC_VREF_2_5V,     \
    .adcTrig=ADC_CTIMER0_TIM3, \
    .adcDiv=ADC_DIV_DISABLED   \
}}
#else
/** adc sbase channel configuration for the common mode measurement */
#define CM_MEAS_SAMPLE {{                      \
    .adcEosSign=ADC_NO_SIGN,                   \
    .adcChan=ADC_SIG_LINVCMO,                  \
    .adcRef=ADC_VREF_2_5V,                     \
    .adcDiv=ADC_DIV_DISABLED,                  \
    .adcSelscSig=ADC_DIFF_DIAG,                \
    .adcSelscRef=ADC_VREF_DIFF_VS,             \
    .adcSelscRefAdd=ADC_VREF_DIFF_HIGH_SIGNAL, \
    .adcTrig=ADC_CTIMER0_TIM3,                 \
    .adcDSel=0u,                               \
    .adcSelscGain=0u,                          \
    .adcScRefDelay=0u,                         \
    .adcScSettleDelay=0u                       \
}}
/** adc sbase channel configuration for the differential mode measurement */
#define DM_MEAS_SAMPLE {{                      \
    .adcEosSign=ADC_NO_SIGN,                   \
    .adcChan=ADC_SIG_LINAAMP,                  \
    .adcRef=ADC_VREF_2_5V,                     \
    .adcDiv=ADC_DIV_DISABLED,                  \
    .adcSelscSig=ADC_DIFF_DIAG,                \
    .adcSelscRef=ADC_VREF_DIFF_VS,             \
    .adcSelscRefAdd=ADC_VREF_DIFF_HIGH_SIGNAL, \
    .adcTrig=ADC_CTIMER0_TIM3,                 \
    .adcDSel=0u,                               \
    .adcSelscGain=0u,                          \
    .adcScRefDelay=0u,                         \
    .adcScSettleDelay=0u                       \
}}
#endif
#endif

/** number of adc samples */
#define LIN_AA_ADC_SAMPLES 18

/** Reset key register to avoid unwanted access to LIN CFG port */
STATIC INLINE void LIN_RESET_XKEY(void)
{
    IO_SET(PORT_LIN_XKEY, LIN_XKEY, 0x0000)
}

/** Set key register to control LIN pull-up termination */
STATIC INLINE void LIN_SET_XKEY(void)
{
    IO_SET(PORT_LIN_XKEY, LIN_XKEY, 0xB2A3)
}

/** Enable LIN pull-up resistor */
STATIC INLINE void LIN_ENABLE_TERMINATION(void)
{
    IO_SET(PORT_LIN_XCFG,  LIN_XCFG,0x0120)
}

/** Disable LIN pull-up resistor */
STATIC INLINE void LIN_DISABLE_TERMINATION(void)
{
    IO_SET(PORT_LIN_XCFG,  LIN_XCFG,0x0320)
}

/** Initialise LIN-AA block for LIN-AA sequence */
STATIC INLINE void LINAA_RESET(void)
{
    IO_SET(PORT_LINAA1,
           LINAA_GAIN, 0u,
           LINAA_DIV, 0u,
           LINAA_RST1, 0u,
           LINAA_RST2, 0u,
           LINAA_EN, 1u,
#if defined(IO_PORT_LINAA1__LINAA_5V_ENABLE)
#if defined(HAS_LIN_AA_EXT_RANGE)
           LINAA_5V_ENABLE, 1u,
#else  /* HAS_LIN_AA_EXT_RANGE */
           LINAA_5V_ENABLE, 0u,
#endif  /* HAS_LIN_AA_EXT_RANGE */
#endif  /* IO_PORT_LINAA1__LINAA_5V_ENABLE */
           LINAA_CDOUTEN, 0u);
    IO_SET(PORT_LINAA2,
           LCD_SEL_LINAA, 0u,
           LCD_ON_LINAA, 0u,
           LCD_DIS_LINAA, 1u);
}

/** Stop/abort LIN-AA block for LIN-AA sequence */
STATIC INLINE void LINAA_STOP(void)
{
    IO_SET(PORT_LINAA1,
           LINAA_GAIN, 0u,
           LINAA_DIV, 0u,
           LINAA_RST1, 0u,
           LINAA_RST2, 0u,
           LINAA_EN, 0u,
#if defined(IO_PORT_LINAA1__LINAA_5V_ENABLE)
#if defined(HAS_LIN_AA_EXT_RANGE)
           LINAA_5V_ENABLE, 1u,
#else  /* HAS_LIN_AA_EXT_RANGE */
           LINAA_5V_ENABLE, 0u,
#endif  /* HAS_LIN_AA_EXT_RANGE */
#endif  /* IO_PORT_LINAA1__LINAA_5V_ENABLE */
           LINAA_CDOUTEN, 0u);
    IO_SET(PORT_LINAA2,
           LCD_SEL_LINAA, 0u,
           LCD_ON_LINAA, 0u,
           LCD_DIS_LINAA, 1u);
}

/** Setup for LIN Bus-shunt current measurement */
STATIC INLINE void LINAA_INIT_MEASUREMENT(void)
{
    IO_SET(PORT_LINAA1,
           LINAA_GAIN, EE_GET(LINAA_GAIN),
#if defined(EE_LINAA_DIV_EXT)
           LINAA_DIV, EE_GET(LINAA_DIV_EXT),
#else  /* EE_LINAA_DIV_EXT */
           LINAA_DIV, EE_GET(LINAA_DIV),
#endif  /* EE_LINAA_DIV_EXT */
           LINAA_RST1, 1u,
           LINAA_RST2, 1u,
           LINAA_EN, 1u,
#if defined(IO_PORT_LINAA1__LINAA_5V_ENABLE)
#if defined(HAS_LIN_AA_EXT_RANGE)
           LINAA_5V_ENABLE, 1u,
#else  /* HAS_LIN_AA_EXT_RANGE */
           LINAA_5V_ENABLE, 0u,
#endif  /* HAS_LIN_AA_EXT_RANGE */
#endif  /* IO_PORT_LINAA1__LINAA_5V_ENABLE */
           LINAA_CDOUTEN, 0u);
}

#if defined(EE_LINAA_LCD_SEL_045)
/** Setup for LIN Bus-shunt preselection current measurement with 0.45mA current source
 * as alternative for pull up resistor 
 */
STATIC INLINE void LINAA_PRESEL_CURRENT_OUTPUT(void)
{
    IO_SET(PORT_LINAA2,
           LCD_SEL_LINAA, EE_GET(LINAA_LCD_SEL_045),
           LCD_ON_LINAA, 1u,
           LCD_DIS_LINAA, 0u);
    IO_SET(TRIM_MISC,
           TRIM_LCD_LINAA, EE_GET(LINAA_TRIM_LCD_045));
    IO_SET(PORT_LINAA1,
           LINAA_CDOUTEN, 1u);
}
#endif  /* EE_LINAA_LCD_SEL_045 */

/** Setup for LIN Bus-shunt selection current measurement with 2.05mA current source */
STATIC INLINE void LINAA_SEL_CURRENT_205_OUTPUT(void)
{
    IO_SET(PORT_LINAA2,
           LCD_SEL_LINAA, EE_GET(LINAA_LCD_SEL_205),
           LCD_ON_LINAA, 1u,
           LCD_DIS_LINAA, 0u);
    IO_SET(TRIM_MISC,
           TRIM_LCD_LINAA, EE_GET(LINAA_TRIM_LCD_205));
    IO_SET(PORT_LINAA1,
           LINAA_CDOUTEN, 1u);
}

#if defined(EE_LINAA_LCD_SEL_240)
/** Setup for LIN Bus-shunt selection current measurement with 2.40mA current source */
STATIC INLINE void LINAA_SEL_CURRENT_240_OUTPUT(void)
{
    IO_SET(PORT_LINAA2,
           LCD_SEL_LINAA, 0x4u | EE_GET(LINAA_LCD_SEL_240),
           LCD_ON_LINAA, 1u,
           LCD_DIS_LINAA, 0u);
    IO_SET(TRIM_MISC,
           TRIM_LCD_LINAA, EE_GET(LINAA_TRIM_LCD_240));
    IO_SET(PORT_LINAA1,
           LINAA_CDOUTEN, 1u);
    
}
#endif  /* EE_LINAA_LCD_SEL_240 */

/** Calculate LIN AA measurements
 *
 * @param[in]  adc_dbase  pointer to the ADC samples for the step to calculate.
 * @param[out]  measurement  pointer to the sample to be updated.
 * @note  this function assumes a 2 common mode and 16 differential mode samples
 *        to be taken by the ADC. Next it will make averages for both numbers and
 *        store them in the requested location.
 *        Differential mode will be stored with factor 8 (ADC_DM_MULTIPLIER) for
 *        accurracy in later calculations.
 */
STATIC INLINE void linaa_CalculateSamples(volatile uint16_t * adc_dbase, LINShunt_t * measurement)
{
#if defined(__COVERITY__) || defined(__POLYSPACE__)
    measurement->commonmode = (int16_t)((adc_dbase[0] + adc_dbase[1]) / 2u);
    uint16_t temp = 0u;
    for (uint8_t ctr = 2u; ctr < 18u; ctr++)
    {
        temp += (int16_t)adc_dbase[ctr];
    }
    measurement->differential = (int16_t)(temp / 2u);
#else
    /* explicit inline assembly as this function has to be executed at fasted speed in LIN AA step ISR */
    asm volatile (
        "mov a, [x++] \n\t"       /* i16Sum =  adc_dbase[0] = LinShunt1 CommonMode */
        "add a, [x++] \n\t"       /* i16Sum += adc_dbase[1] = LinShunt2 CommonMode */
        "lsr a, #1 \n\t"          /* measurement->commonmode = (i16Sum >> 1) */
        "mov y, [x++] \n\t"       /* i16Sum =  adc_dbase[2] = LinShunt1 mode1 */
        "add y, [x++] \n\t"       /* i16Sum += adc_dbase[3] = LinShunt2 mode1 */
        "add y, [x++] \n\t"       /* i16Sum += adc_dbase[4] = LinShunt3 mode1 */
        "add y, [x++] \n\t"       /* i16Sum += adc_dbase[5] = LinShunt4 mode1 */
        "add y, [x++] \n\t"       /* i16Sum += adc_dbase[6] = LinShunt5 mode1 */
        "add y, [x++] \n\t"       /* i16Sum += adc_dbase[7] = LinShunt6 mode1 */
        "add y, [x++] \n\t"       /* i16Sum += adc_dbase[8] = LinShunt7 mode1 */
        "add y, [x++] \n\t"       /* i16Sum += adc_dbase[9] = LinShunt8 mode1 */
        "add y, [x++] \n\t"       /* i16Sum += adc_dbase[10] = LinShunt9 mode1 */
        "add y, [x++] \n\t"       /* i16Sum += adc_dbase[11] = LinShunt10 mode1 */
        "add y, [x++] \n\t"       /* i16Sum += adc_dbase[12] = LinShunt11 mode1 */
        "add y, [x++] \n\t"       /* i16Sum += adc_dbase[13] = LinShunt12 mode1 */
        "add y, [x++] \n\t"       /* i16Sum += adc_dbase[14] = LinShunt13 mode1 */
        "add y, [x++] \n\t"       /* i16Sum += adc_dbase[15] = LinShunt14 mode1 */
        "add y, [x++] \n\t"       /* i16Sum += adc_dbase[16] = LinShunt15 mode1 */
        "add y, [x++] \n\t"       /* i16Sum += adc_dbase[17] = LinShunt16 mode1 */
        "lsr y, #1 \n\t"          /* measurement->differential = (i16Sum >> 1) = DM_meas * ADC_DM_MULTIPLIER */
        : "=a" (measurement->commonmode), "=y" (measurement->differential)
        : "x" (adc_dbase)
    );
#endif  /* __COVERITY__ || __POLYSPACE */
}

#endif  /* FW_LIN_AUTO_ADDRESSING_PRIVATE_H */
