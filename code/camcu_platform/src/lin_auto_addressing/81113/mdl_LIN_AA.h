/*
 * Copyright (C) 2021 Melexis GmbH
 *
 * This file is part of the Mlx81113 module library.
 *
 * File mdl_LIN_AA.h
 *
 * Module prefix: mdl_LIN_AA
 * All module interface functions start with this letters.
 *
 * LIN auto addressing module. Includes the sequence and all functions required to perform
 * the LIN auto addressing algorithm according to the BSM.
 *
 * Revision: 1.3
 * Date:     04.05.2021
 *
 */
/* History:
 *   Revision 1.3
 *     - define added for detection of LINAA twisted connection
 *   Revision 1.2
 *     - reduce amount of parameter for external shunt
 *   Revision 1.1
 *     - defines added for external shunt support
 *     - pre-selection current threshold increased to 1mA
 *     - defines for SW common mode compensation added
 *   Revision 1.0
 *     - Initial release
 **********************************************/

#ifndef MDL_LIN_AA_H_
#define MDL_LIN_AA_H_

#include <mls_api.h>
#include <lib_adc.h>


/* ==========================================================================
 * Public defines
 * ========================================================================== */

#if FPLL == 12000
    #define PROCTIME 6
    #define LINAA_STARTDELAY 1
    #error "Clock Speed not valid for LIN auto-addressing"
#elif FPLL == 14000
    #define PROCTIME 6
    #define LINAA_STARTDELAY 1
    #error "Clock Speed not valid for LIN auto-addressing"
#elif FPLL == 16000
    #define PROCTIME 8
    #define LINAA_STARTDELAY 1
#elif FPLL == 24000
    #define PROCTIME 8
    #define LINAA_STARTDELAY 6
#elif FPLL == 28000
    #define PROCTIME 8
    #define LINAA_STARTDELAY 8
#elif FPLL == 32000
    #define PROCTIME 10
    #define LINAA_STARTDELAY 10
#else
    #error "Clock Speed not valid for LIN auto-addressing"
#endif

/* delay loop */
#define DELAY2(loops)           \
    __asm__ __volatile__ (      \
        "mov  X, %[cnt]\n\t"    \
        "djnz X,."              \
        :                       \
        : [cnt] "ri" (loops)    \
        : "X"                   \
        )

/* usec delay function for MLX81113 */
#define MLX81113_USEC_DELAY(us) DELAY2((FPLL * (uint32_t)(us)) / (PROCTIME*1000u))

/* adc sbase channel configuration for the common mode measurement */
#define CM_MEAS_SAMPLE {{                      \
    .adcEosSign=ADC_NO_SIGN,                   \
    .adcChan=ADC_SIG_LINVCMO,                  \
    .adcRef=ADC_VREF_2_5V,                     \
    .adcSelscSig=ADC_DIFF_DIAG,                \
    .adcSelscRef=ADC_VREF_DIFF_VS,             \
    .adcTrig=ADC_CTIMER0_TIM3,                 \
    .adcDSel=0u,                               \
    .adcSelscGain=0u,                          \
    .adcScRefDelay=0u,                         \
    .adcScSettleDelay=0u                       \
}}
/* adc sbase channel configuration for the differential mode measurement */
#define DM_MEAS_SAMPLE {{                      \
    .adcEosSign=ADC_NO_SIGN,                   \
    .adcChan=ADC_SIG_LINAAMP,                  \
    .adcRef=ADC_VREF_2_5V,                     \
    .adcSelscSig=ADC_DIFF_DIAG,                \
    .adcSelscRef=ADC_VREF_DIFF_VS,             \
    .adcTrig=ADC_CTIMER0_TIM3,                 \
    .adcDSel=0u,                               \
    .adcSelscGain=0u,                          \
    .adcScRefDelay=0u,                         \
    .adcScSettleDelay=0u                       \
}}

/* define end of sequence */
#define LINAA_ADC_EOF 0x0003u

/* type declaration for LINAA ADC channellist */
#define DECLARE_LINAA_ADC_STRUCT(len)          \
typedef struct __attribute__((packed)) {       \
    uint16_t dbase;                            \
    AdcSData_t sdata[len];                     \
    uint16_t adcEndType;                       \
    uint16_t sbase;                            \
} LinaaAdcStruct##len##_t;

/* scaling factor for VOUT of LINAA amplifier */
#define SAADMUser 50      /* 50 LSB / mA */
#define SAADMUserSCALE 12 /* 12 Bit -> 4096 */

#if LINAAEXTSHUNTMOUNT == 1
/* external shunt value * 1000 [mOhm] */
    #define LINAA_R_EXTSHUNT 220
/* internal shunt value * 1000 [mOhm] */
    #define LINAA_R_INTSHUNT_DEFAULT 750
/* define bond wire resistance * 1000 [mOhm] */
    #define LINAA_R_BONDWIRE 120
/* scale resistive divider */
    #define LINAA_SCALE_RESDIV 100
/* preselection current */
    #define LINAA_PRE_CURR 450
/* selection current */
    #define LINAA_SEL_CURR 2400
/* preselection current over internal shunt in SAAMDMUser LSB/mA multiplied by LINAA_SCALE_RESDIV */
    #define LINAA_INTSHUNT_PRE_CURR (int16_t)((int32_t)((int32_t)LINAA_PRE_CURR * (int32_t)SAADMUser * \
                                                        (int32_t)LINAA_SCALE_RESDIV * (int32_t)LINAA_R_BONDWIRE) / \
                                              (int32_t)((int32_t)1000 * (int32_t)LINAA_R_EXTSHUNT))
/* selection current over internal shunt in SAAMDMUser LSB/mA multiplied by LINAA_SCALE_RESDIV */
    #define LINAA_INTSHUNT_SEL_CURR (int16_t)((int32_t)((int32_t)LINAA_SEL_CURR * (int32_t)SAADMUser * \
                                                        (int32_t)LINAA_SCALE_RESDIV * (int32_t)LINAA_R_BONDWIRE) / \
                                              (int32_t)((int32_t)1000 * (int32_t)LINAA_R_EXTSHUNT))
#endif

/* scaling factor for common mode factor of LINAA amplifier */
#define SAADMCM_EE_SCALE 15

/* reset key register to avoid unwanted access to LIN CFG port */
#define LIN_RESET_XKEY() IO_SET(PORT_LIN_XKEY,LIN_XKEY, 0x0000)
/* set key register to control LIN pull-up termination */
#define LIN_SET_XKEY() IO_SET(PORT_LIN_XKEY,LIN_XKEY, 0xB2A3)

/* enable LIN pull-up resistor */
#define LIN_ENABLE_TERMINATION() IO_SET(PORT_LIN_XCFG,  LIN_XCFG,0x0120)
/* disable LIN pull-up resistor */
#define LIN_DISABLE_TERMINATION() IO_SET(PORT_LIN_XCFG,  LIN_XCFG,0x0320)

/* delay offset in dependency on the baud-rate in [us] */
#define LINAA_DELAYOFF 52

/* ADC macros for LINAA */
#define ADC_INT_DISABLE() Itc_Disable(ADC_SAR)     /* macro to disable ADC interrupt */
#define ADCFREQUENCYKHZ_LIN_AA 4000u               /* 4000KHz -> 4MHz ADC frequency used for LINAA */
#define ADCCLOCKDIVIDER_LIN_AA ((FPLL/ADCFREQUENCYKHZ_LIN_AA)-1) /* calculate ADC divider based on system clock */
#define ADCENABLE 1u                               /* ADC enable for ADC HW unit */
#define LINAA_ADC_TRIG_TMR ((FPLL/200u))           /* ADC trigger every 5us */

/* defines for LIN amplifier control and current control */
#define VAL_LINAA_RST1_SET (1u)        /* reset first amplifier - 1 bit */
#define VAL_LINAA_RST1_RESET (0u)      /* finish offset first amplifier - 1 bit */
#define VAL_LINAA_RST2_SET (1u)        /* reset second amplifier - 1 bit */
#define VAL_LINAA_RST2_RESET (0u)      /* finish offset second amplifier - 1 bit */
#define VAL_LINAAEN_ENABLE (1u)        /* enable LINAA unit - 1 bit */
#define VAL_LINAAEN_DISABLE (0u)       /* disable LINAA unit - 1 bit */
#define VAL_LINAA_CDOUTEN_ENABLE (1u)  /* monitor current switch enable */
#define VAL_LINAA_CDOUTEN_DISABLE (0u) /* monitor current switch disable */
#define VAL_LCD_CURR_ENABLE (0u)       /* enable current source */
#define VAL_LCD_CURR_DISABLE (1u)      /* disable current source */
#define VAL_LCD_CURR_On (1u)           /* switch on current source */
#define VAL_LCD_CURR_Off (0u)          /* switch off current source */
#define VAL_VDDA_5V      (1u)          /* extend LIN amplifier range */
#define VAL_VDDA_3V6     (0u)          /* normal operation */

#define VAL_DEFAULT_LCD_SEL_045 (0u)      /* default LCD sel 0.45mA */
#define VAL_DEFAULT_LCD_SEL_205 (3u)      /* default LCD sel 2.05mA */
#define VAL_DEFAULT_LCD_SEL_240 (4u)      /* default LCD sel 2.4mA */
#define VAL_DEFAULT_LCD_TRIM (0u)         /* default LCD trim */
#define VAL_DEFAULT_GAIN_TRIM (3u)        /* default gain settings */
#define VAL_DEFAULT_DIV_TRIM (0u)         /* default div settings */
#define VAL_DEFAULT_GAINAADMCAL (0x0DCBu) /* default scaling */
#define VAL_DEFAULT_SAADMCM (0x0000)      /* default common mode scaling */

#define LINAA_SLOWBAUD_DELAY 25u          /* delay for slow baud comm */

/* selection currents and divider for LIN AA I_DIFF threshold */
#define LIN_AA_CURRLIMIT_PRE        (SAADMUser)     /* current limit for preselection phase -> 1mA (50 LSB/mA * 1mA -> 50LSB) */
#define LIN_AA_CURRLIMIT_SEL        (SAADMUser)     /* current limit for selection phase -> 1mA (50 LSB/mA * 1mA -> 50LSB) */
#define LINAA_SELECTION_CURRENT     7               /*  EEP_CURRDAC_2_05mA_35   => 2mA during selection phase */

/* flags in AutoAddressingFlags */
#define LIN_AA_SLAVEADDRESSED      0x01u            /* 0: slave is not addressed yet                        1: slave is addressed                         */
#define LIN_AA_SLAVEWAITING        0x04u            /* 0: slave in the running for Ishunt3                  1: slave not in the running for this break    */
#define LIN_AA_LASTSLAVE           0x08u            /* 0: slave is not the  last in line                    1: slave is the last in line                  */
#define LIN_AA_AUTOADDRESSENABLE   0x80u            /* 0: LIN AA disabled                                   1: LIN AA enabled                             */

#define LIN_AA_INVALID_HW_CONNECT  0x10u            /* 0: HW Connection is ok                               1: HW Connection is wrong                     */
#define LIN_AA_INVALID_HW_DETECT   0x20u            /* 0: HW Connection is ok                               1: HW Connection could be wrong               */

#define LIN_AA_RESETALLFLAGS       0xFFu            /* clear all flags via reset funciton */

#define LIN_AA_SUBFUNCTION1        0x01u            /* SNPD sub function ID 0x01 "All BSM-nodes enter the unconfigured mode" */
#define LIN_AA_SUBFUNCTION2        0x02u            /* SNPD sub function ID 0x02 "Informing all slaves about the next NAD" */
#define LIN_AA_SUBFUNCTION3        0x03u            /* SNPD sub function ID 0x03 "Store the assigned NADs in to the NVM of the slaves, if available" */
#define LIN_AA_SUBFUNCTION4        0x04u            /* SNPD sub function ID 0x04 "Informing all slaves that the procedure is finished" */

/* LIN AutoAddressing Bus Shunt Method Slave Node Position Detection Steps */
#define LIN_AA_BSM_SNPD_STEP1   0x01u               /* Step 1: Switching off all Pull-Ups and all current sources                                               */
#define LIN_AA_BSM_SNPD_STEP2   0x02u               /* Step 2: Start offset measurement (Ishunt1)                                                               */
#define LIN_AA_BSM_SNPD_STEP3   0x03u               /* Step 3: switching on current source 1  on LIN_IN                                                         */
#define LIN_AA_BSM_SNPD_STEP4   0x04u               /* Step 4: Start measurement 1 (Ishunt2)                                                                    */
#define LIN_AA_BSM_SNPD_STEP5   0x05u               /* Step 5: all not pre-selected (first action A) nodes are switching off their current source 1
                                                     *         all pre-selected (second action B) nodes are switching on their current source 2 on LIN_IN       */
#define LIN_AA_BSM_SNPD_STEP6   0x06u               /* Step 6: Start measurement 2 (Ishunt3)                                                                    */
#define LIN_AA_BSM_SNPD_STEP7   0x07u               /* Step 7: Switching off all current sources and switching on the LIN termination                           */

/* ==========================================================================
 * Public variables
 * ========================================================================== */
extern volatile uint8_t LIN_AA_AutoAddressingFlags;

/* ==========================================================================
 * Public functions
 * ========================================================================== */
extern void mdl_LIN_AA_initAutoAddressing(void);
extern void mdl_LIN_AA_stopAutoAddressing(void);
extern void mdl_LIN_AA_set_AutoAddressingFlags(uint8_t LIN_AA_AutoAddressingFlag);
extern void mdl_LIN_AA_reset_AutoAddressingFlags(uint8_t LIN_AA_AutoAddressingFlag);
extern bool mdl_LIN_AA_proceed_0xB5_diagframe(LINDiagTransfer_t *transfer);

#endif /* MDL_LIN_AA_H_ */
