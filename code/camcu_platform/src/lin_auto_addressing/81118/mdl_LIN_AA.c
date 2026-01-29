/*
 * Copyright (C) 2021 Melexis GmbH
 *
 * This file is part of the Mlx81118 module library.
 *
 * File mdl_LIN_AA.c
 *
 * Module prefix: mdl_LIN_AA
 * All module interface functions start with this letters.
 *
 * LIN auto addressing module. Includes the sequence and all functions required to perform
 * the LIN auto addressing algorithm according to the BSM.
 *
 * Revision: 1.8
 * Date:     04.05.2021
 *
 *
 * ==========================================================================
 * History:
 *   Revision 1.8
 *     - changed "twisted LINAA connection" handling in case of intermediate
 *       LIN messages
 *   Revision 1.7
 *     - update EE_GET macros for 0.45mA and 2.4mA
 *   Revision 1.6
 *     - word aligned variables
 *   Revision 1.5
 *     - reducing calculation complexity for external shunt
 *     - using EEPROM parameter "CALIB_VERSION" to check for SW common mode compensation
 *   Revision 1.4
 *     - join revision 1.3a and 1.3 with EEPROM macro update
 *   Revision 1.3
 *     - updated EEPROM macros for LIN AA according to updated eeprom_parameters.h file
 *   Revision 1.3a (no release version)
 *     - SW common mode compensation added
 *     - ADC channel-number increased for averaging
 *     - supports LIN auto-addressing via external shunt resistor
 *   Revision 1.2
 *     - consider sign for EEPROM parameter EE_APP_GAINAAADMCAL
 *   Revision 1.1
 *     - update EEPROM macros for trimming
 *   Revision 1.0
 *     - Initial release
 *
 *
 * ========================================================================== */

/* ==========================================================================
 * Includes
 * ========================================================================== */
#include <plib.h>
#include <mls_api.h>
#include <mem_checks.h>
#include <eeprom_map.h>
#include <mls_types.h>
#include <atomic.h>
#include <lib_pwm.h>
#include <lib_adc.h>
#include <ctimerlib.h>

#include "mdl_LIN_AA.h"

#define HAS_LIN_AUTOADDRESSING 1

/* ==========================================================================
 * Private type definitions, macros, defines
 * ========================================================================== */
typedef struct {
    int16_t commonmode;
    int16_t differential;
}LINShunt;

/** reset LIN auto addressing registers to default values
 * @param void
 * @return void
 */
#define LIN_AA_STOP()                                                                                \
    do {                                                                                             \
        IO_SET(PORT_LINAA, LINAA_EN, VAL_LINAAEN_DISABLE, LINAA_CDOUTEN, VAL_LINAA_CDOUTEN_DISABLE); \
        IO_SET(PORT_LINAA, LCD_DIS_LINAA, VAL_LCD_CURR_ENABLE, LCD_ON_LINAA, VAL_LCD_CURR_Off);      \
    } while (0);

/** LIN initialize measurement for autoaddressing with offset compensation
 * @param
 * @return void
 *
 */
#define LIN_AA_INIT_MEASUREMENT()          \
    do {                                   \
        MLX81118_USEC_DELAY(40);           \
        IO_SET(PORT_LINAA,                 \
               LINAA_RST1,                 \
               VAL_LINAA_RST1_RESET,       \
               LINAA_RST2,                 \
               VAL_LINAA_RST2_SET,         \
               LINAA_EN,                   \
               VAL_LINAAEN_ENABLE,         \
               LINAA_CDOUTEN,              \
               VAL_LINAA_CDOUTEN_DISABLE); \
        MLX81118_USEC_DELAY(40);           \
        IO_SET(PORT_LINAA,                 \
               LINAA_RST1,                 \
               VAL_LINAA_RST1_RESET,       \
               LINAA_RST2,                 \
               VAL_LINAA_RST2_RESET,       \
               LINAA_EN,                   \
               VAL_LINAAEN_ENABLE,         \
               LINAA_CDOUTEN,              \
               VAL_LINAA_CDOUTEN_DISABLE); \
    } while (0);

/** LIN AA amplifier reset state
 * @param
 * @return void
 *
 */
#define LIN_AA_AMP_RESET()                 \
    do {                                   \
        IO_SET(PORT_LINAA ,                \
               LINAA_RST1,                 \
               VAL_LINAA_RST1_SET,         \
               LINAA_RST2,                 \
               VAL_LINAA_RST2_SET,         \
               LINAA_EN,                   \
               VAL_LINAAEN_ENABLE,         \
               LINAA_CDOUTEN,              \
               VAL_LINAA_CDOUTEN_DISABLE); \
    } while (0);

/** LIN set current at LIN_IN
 * @param
 * @return void
 *
 */
#define LIN_AA_SET_CURRENT_OUTPUT()      \
    do  {                                \
        IO_SET(PORT_LINAA,               \
               LINAA_CDOUTEN,            \
               VAL_LINAA_CDOUTEN_ENABLE, \
               LCD_DIS_LINAA,            \
               VAL_LCD_CURR_ENABLE,      \
               LCD_ON_LINAA,             \
               VAL_LCD_CURR_On);         \
        MLX81118_USEC_DELAY(1); /*1us*/  \
    } while (0);

/* ==========================================================================
 * Declaration internal variables
 * ========================================================================== */

/* LIN auto-addressing ADC conversion data */
static volatile uint16_t LIN_AA_adcValue[20] __attribute__((aligned(2)));
/* detected LIN baud-rate */
static volatile uint16_t LIN_AA_baud __attribute__((aligned(2))) = 0;

/* ADC measurement table for LIN auto-addressing in offset phase */
DECLARE_LINAA_ADC_STRUCT(11);
const LinaaAdcStruct11_t SBASE_LIN_OFF __attribute__((aligned(2))) =
{
    .dbase=(uint16_t)(&LIN_AA_adcValue[0]),
    .sdata=
        {
            CM_MEAS_SAMPLE,
            CM_MEAS_SAMPLE,
            DM_MEAS_SAMPLE,
            DM_MEAS_SAMPLE,
            DM_MEAS_SAMPLE,
            DM_MEAS_SAMPLE,
            DM_MEAS_SAMPLE,
            DM_MEAS_SAMPLE,
            DM_MEAS_SAMPLE,
            DM_MEAS_SAMPLE,
            DM_MEAS_SAMPLE
        },
    .adcEndType = LINAA_ADC_EOF,
    .sbase = (uint16_t)(&SBASE_LIN_OFF)
};

/* ADC measurement table for LIN auto-addressing in (pre-)selection phase */
DECLARE_LINAA_ADC_STRUCT(19);
const LinaaAdcStruct19_t SBASE_LIN_SEL __attribute__((aligned(2))) =
{
    .dbase=(uint16_t)(&LIN_AA_adcValue[0]),
    .sdata=
        {
            CM_MEAS_SAMPLE,
            CM_MEAS_SAMPLE,
            DM_MEAS_SAMPLE,
            DM_MEAS_SAMPLE,
            DM_MEAS_SAMPLE,
            DM_MEAS_SAMPLE,
            DM_MEAS_SAMPLE,
            DM_MEAS_SAMPLE,
            DM_MEAS_SAMPLE,
            DM_MEAS_SAMPLE,
            DM_MEAS_SAMPLE,
            DM_MEAS_SAMPLE,
            DM_MEAS_SAMPLE,
            DM_MEAS_SAMPLE,
            DM_MEAS_SAMPLE,
            DM_MEAS_SAMPLE,
            DM_MEAS_SAMPLE,
            DM_MEAS_SAMPLE,
            DM_MEAS_SAMPLE
        },
    .adcEndType = LINAA_ADC_EOF,
    .sbase = (uint16_t)(&SBASE_LIN_SEL)
};

/* ==========================================================================
 * Declaration private functions
 * ========================================================================== */

void mdl_LIN_AA_init_ports(void);

/* ==========================================================================
 * Declaration public variables
 * ========================================================================== */

volatile uint8_t LIN_AA_AutoAddressingFlags = 0x00u;

/* ==========================================================================
 * Declaration public functions
 * ========================================================================== */

void mdl_LIN_AA_initAutoAddressing(void);
void mdl_LIN_AA_stopAutoAddressing(void);
void mdl_LIN_AA_set_AutoAddressingFlags(uint8_t LIN_AA_AutoAddressingFlag);
void mdl_LIN_AA_reset_AutoAddressingFlags(uint8_t LIN_AA_AutoAddressingFlag);

/* ==========================================================================
 * Implementation public functions
 * ========================================================================== */

/* initialize and run the LIN autoaddressing sequence */

void mdl_LIN_AA_initAutoAddressing(void){
    mdl_LIN_AA_reset_AutoAddressingFlags(LIN_AA_SLAVEADDRESSED | LIN_AA_SLAVEWAITING | \
       LIN_AA_LASTSLAVE | LIN_AA_INVALID_HW_DETECT | LIN_AA_INVALID_HW_CONNECT);

    mdl_LIN_AA_init_ports();

    /* switch VDDA to 5V for CM */
    IO_SET(PORT_MISC_OUT,SWITCH_VDDA_TO_5V,VAL_VDDA_5V);

    /* enable write access to register LIN_XCFG for enable/disable LIN termination*/
    LIN_SET_XKEY();

    /* enable interrupts during LIN break field */
    (void)ml_AutoAddressingConfig(ML_ENABLED);

    /* reset all LIN current and amplifier adjustments */
    LIN_AA_STOP();

    LIN_AA_baud = ml_GetBaudRate(MLX4_FPLL);
}


#if defined (HAS_LIN_AUTOADDRESSING)
/*
 ******************************************************************************
 * mlu_AutoAddressingStep event
 *
 * A step is sent by the LIN Module during the auto addressing
 ******************************************************************************
 */
void l_AutoAddressingStep(uint8_t StepNumber){
    if (StepNumber == LIN_AA_BSM_SNPD_STEP1) {
        /* disable integrated LIN slave termination */
        LIN_DISABLE_TERMINATION();
        mdl_LIN_AA_reset_AutoAddressingFlags(LIN_AA_SLAVEWAITING | LIN_AA_LASTSLAVE \
            | LIN_AA_INVALID_HW_DETECT);
    }

    static LINShunt Ishunt_0 = {0,0};

    LINShunt Ishunt_1;

    /* used for current calculation over Shunt */
    int16_t Idiff = 0;
    int32_t tempGainAADMCal = 0;
    int32_t tempSAADMCM = 0;
    uint16_t adctrigcnt = 0;

    /* inserting CPU dependent delay */
    MLX81118_USEC_DELAY(LINAA_STARTDELAY);

    /* use Idiff for applying baud-rate related delay */
    Idiff = divI16_I32byI16(1000000,(int16_t)LIN_AA_baud);
    Idiff -= LINAA_DELAYOFF;
    if(Idiff > 0) {
        DELAY2(divU16_U32byU16(mulU32_U16byU16(FPLL,(uint16_t)Idiff),PROCTIME * 1000u));
    }

    /* check if slave is passive during the next LIN AA steps */
    if ( (LIN_AA_AutoAddressingFlags & LIN_AA_SLAVEWAITING) != 0u ) {
        if (StepNumber == LIN_AA_BSM_SNPD_STEP7) {
            LIN_AA_STOP();

            /* enable integrated LIN slave termination */
            LIN_ENABLE_TERMINATION();

            /* stop ADC hardware trigger */
            CTimer0_Stop();
            mdl_LIN_AA_reset_AutoAddressingFlags(LIN_AA_SLAVEWAITING);
        }

    }
    else {

        switch(StepNumber) {
            /* -------------------------------------------------------------------------
             * LIN Bus Shunt Method (BSM) Slave Node Position Detection (SNPD) - Step 1:
             * -------------------------------------------------------------------------
             * - Switching off all Pull-Ups and all current sources
             */
            case LIN_AA_BSM_SNPD_STEP1:
                /* reset all LIN current and amplifier adjustments */
                LIN_AA_STOP();

                /* set amplifier into reset state */
                LIN_AA_AMP_RESET();

                /* initialize Complex Timer 0 as ADC hardware trigger */
                /* configure HW trigger for 10 conversions per bittime */
                adctrigcnt = divU16_U32byU16(mulU32_U16byU16(FPLL,100u),LIN_AA_baud);
                CTimer0_AutoloadInit(eTimerCPUClockDivisionBy1, adctrigcnt);
                CTimer0_Stop();

                if (IO_GET(ADC_SAR, STOP)!=1) { /* If ADC is not stopped, we need to stop it first */
                    /* disable ADC interrupt */
                    ADC_INT_DISABLE();

                    /* stop the current ADC action */
                    AdcStop();

                    /* Make sure we will have the ADC HW reaction */
                    MLX81118_USEC_DELAY(1);

                    /* wait until ADC is stopped */
                    while (IO_GET(ADC_SAR, ABORTED )==0u) {
                        /* Wait for the ADC will be aborted after the STOP command */
                    }
                }

                /* enable ADC HW block */
                IO_SET(PORT_ADC_CTRL,ADC_EN,ADCENABLE);

                /* create ADC init structure */
                AdcControl_t adc_ctrl = {{.start=1u,
                                          .stop=0u,
                                          .sosSource=ADC_SOS_FIRST_HARDWARE_TRIGGER,
                                          .socSource=ADC_SOC_HARDWARE_TRIGGER,
                                          .noInterleave=1u,
                                          .saturate=1u,
                                          .intScheme=ADC_INT_EOS,
                                          .asb=ADC_ASB_NEVER,
                                          .adcWidth=ADC_WDT_16Bit}};

                /* initialize ADC - divider 7 -> 32MHz/(7+1) -> 4MHz, sbase and adc ctrl */
                AdcInit(ADCCLOCKDIVIDER_LIN_AA, (void*)&SBASE_LIN_OFF, adc_ctrl);
                /* start ADC conversion */
                AdcStart();

                /* check if slave is already addressed */
                if ( (LIN_AA_AutoAddressingFlags & LIN_AA_SLAVEADDRESSED) != 0u ) {

                    /* slave already addressed -> wait till last auto addressing step and do nothing */
                    mdl_LIN_AA_set_AutoAddressingFlags(LIN_AA_SLAVEWAITING);

                }
#if LINAACSPRESEL == 1
                if((EE_GET(LINAA_GAINDMCAL) == 0) || (EE_GET(LINAA_GAIN) == 0) || (EE_GET(LINAA_LCD_SEL_205) == 0)) {
                    IO_SET(PORT_LC_MISC, LCDIAG_SEL, VAL_DEFAULT_LCD_SEL_045);
                }
                else {
                    IO_SET(TRIM_MISC, LCDIAG_TRIM, EE_GET(LINAA_TRIM_LCD_045));
                    IO_SET(PORT_LC_MISC, LCDIAG_SEL, EE_GET(LINAA_LCD_SEL_045));
                }
#endif
                break;

            /* -------------------------------------------------------------------------
             * LIN Bus Shunt Method (BSM) Slave Node Position Detection (SNPD) - Step 2:
             * -------------------------------------------------------------------------
             * - Start offset measurement (Ishunt1)
             */
            case LIN_AA_BSM_SNPD_STEP2:
                /* initialize offset measurement (Ishunt1) */
                LIN_AA_INIT_MEASUREMENT();
            #if __GNUC__ >= 7
                __attribute__ ((fallthrough));
            #endif

            /* -------------------------------------------------------------------------
             * LIN Bus Shunt Method (BSM) Slave Node Position Detection (SNPD) - Step 4:
             * -------------------------------------------------------------------------
             * - Start measurement 1 (Ishunt2)
             */
            case LIN_AA_BSM_SNPD_STEP4:

            /* -------------------------------------------------------------------------
             * LIN Bus Shunt Method (BSM) Slave Node Position Detection (SNPD) - Step 6:
             * -------------------------------------------------------------------------
             * - Start measurement 2 (Ishunt3)
             */
            case LIN_AA_BSM_SNPD_STEP6:
                /* stop ADC */
                AdcStop();
                /* stop ADC HW trigger */
                CTimer0_Stop();

                if (StepNumber!=LIN_AA_BSM_SNPD_STEP2) {
                    /* signalize ADC to be ready for next conversion */
                    IO_SET(ADC_SAR, SBASE_0, (uint16_t )(&SBASE_LIN_SEL));
                }
                else {
                    /* signalize ADC to be ready for next conversion */
                    IO_SET(ADC_SAR, SBASE_0, (uint16_t )(&SBASE_LIN_OFF));
                }

                /* start ADC */
                AdcStart();

                /* start ADC hardware trigger trigger */
                CTimer0_Start();

                while ( (IO_GET(ADC_SAR,STATE)) != 0 ) {
                    /* In case ADC is active, wait to finish it */
                }

                /* calculate average value from ADC measurement results */
                /* to avoid errors the fifth measurement result will be ignored */
                __asm__ __volatile__ (
                    "mov x, %3 \n\t"          /* load  LIN_AA_adcValue for accumulation */
                    "mov y, [x++] \n\t"       /* i16Sum =  *(int16_t *) &LIN_AA_adcValue[0] = LinShunt1 CommonMode */
                    "add y, [x++] \n\t"       /* i16Sum += *(int16_t *) &LIN_AA_adcValue[1] = LinShunt2 CommonMode */
                    "lsr y, #1 \n\t"          /* (i16Sum >>= 1) */
                    "mov %0, y \n\t"          /* Ishunt_1.commonmode = i16Sum */
                    "mov y, [x++] \n\t"       /* i16Sum =  *(int16_t *) &LIN_AA_adcValue[2] = LinShunt1 mode1 */
                    "mov y, [x++] \n\t"       /* i16Sum =  *(int16_t *) &LIN_AA_adcValue[3] = LinShunt2 mode1 */
                    "add y, [x++] \n\t"       /* i16Sum += *(int16_t *) &LIN_AA_adcValue[4] = LinShunt3 mode1 */
                    "add y, [x++] \n\t"       /* i16Sum += *(int16_t *) &LIN_AA_adcValue[5] = LinShunt4 mode1 */
                    "add y, [x++] \n\t"       /* i16Sum += *(int16_t *) &LIN_AA_adcValue[6] = LinShunt5 mode1 */
                    "add y, [x++] \n\t"       /* i16Sum += *(int16_t *) &LIN_AA_adcValue[7] = LinShunt6 mode1 */
                    "add y, [x++] \n\t"       /* i16Sum += *(int16_t *) &LIN_AA_adcValue[8] = LinShunt7 mode1 */
                    "add y, [x++] \n\t"       /* i16Sum += *(int16_t *) &LIN_AA_adcValue[9] = LinShunt8 mode1 */
                    "add y, [x++] \n\t"       /* i16Sum += *(int16_t *) &LIN_AA_adcValue[10] = LinShunt9 mode1 */
                    "mov a, %2 \n\t"          /* load StepNumber */
                    "cmp al, #2 \n\t"         /* check for step 2 */
                    "jz _endaddlinaa \n\t"    /* jmp to the end if we are in step 2 */
                    "add y, [x++] \n\t"       /* i16Sum =  *(int16_t *) &LIN_AA_adcValue[11] = LinShunt10 mode1 */
                    "add y, [x++] \n\t"       /* i16Sum += *(int16_t *) &LIN_AA_adcValue[12] = LinShunt11 mode1 */
                    "add y, [x++] \n\t"       /* i16Sum += *(int16_t *) &LIN_AA_adcValue[13] = LinShunt12 mode1 */
                    "add y, [x++] \n\t"       /* i16Sum += *(int16_t *) &LIN_AA_adcValue[14] = LinShunt13 mode1 */
                    "add y, [x++] \n\t"       /* i16Sum += *(int16_t *) &LIN_AA_adcValue[15] = LinShunt14 mode1 */
                    "add y, [x++] \n\t"       /* i16Sum += *(int16_t *) &LIN_AA_adcValue[16] = LinShunt15 mode1 */
                    "add y, [x++] \n\t"       /* i16Sum += *(int16_t *) &LIN_AA_adcValue[17] = LinShunt16 mode1 */
                    "add y, [x++] \n\t"       /* i16Sum += *(int16_t *) &LIN_AA_adcValue[18] = LinShunt17 mode1 */
                    "_endaddlinaa: \n\t"      /* shortcut for step 2 */
                    "mov %1, y \n\t"          /* Ishunt_1.differential = i16Sum */
                    : "=m" (Ishunt_1.commonmode), "=m" (Ishunt_1.differential), "=m" (StepNumber)
                    : "X" (&LIN_AA_adcValue)
                    : "A", "X", "Y"
                    );

                if (StepNumber != LIN_AA_BSM_SNPD_STEP2) { /* perform math operations in step 4 and 6 (not step 2) */
                    Ishunt_1.differential >>= 4;  /* divide by 16 for averaging */
                    if((EE_GET(LINAA_GAINDMCAL) == 0) || (EE_GET(LINAA_GAIN) == 0) ||
                       (EE_GET(LINAA_LCD_SEL_205) == 0)) {
                        tempSAADMCM = VAL_DEFAULT_SAADMCM;
                        tempGainAADMCal = VAL_DEFAULT_GAINAADMCAL;
                    }
                    else {
                        tempSAADMCM = (int32_t)((int16_t) EE_GET(LINAA_SDMCM));
                        tempGainAADMCal = (int32_t)((int16_t)EE_GET(LINAA_GAINDMCAL));
                        if(tempGainAADMCal < 0) {
                            /* change sign of gradient to positive */
                            tempGainAADMCal *= -1;
                        }
                    }
                    if ((EE_GET(CALIB_VERSION)) >= 3u) {
                        /* calculate common mode dependency */
                        Idiff =
                            (int16_t)((int32_t)(tempSAADMCM * (int32_t)(Ishunt_0.commonmode - Ishunt_1.commonmode)) >>
                                      SAADMCM_EE_SCALE);
                    }
                    else {
                        Idiff = 0;
                    }
                    /* calculate the current over the LIN shunt I_diff = Ishunt2 - Ishunt1 (offset current) */
                    Idiff =
                        (int16_t)((int32_t)((int32_t)((int32_t)(Ishunt_0.differential - Ishunt_1.differential) -
                                                      (int32_t)Idiff) * \
                                            (int32_t)(tempGainAADMCal)) >> SAADMUserSCALE);
#if LINAAEXTSHUNTMOUNT == 1
                    {
                        int16_t tmp_negcurr = 0;
                        int16_t tmp_ishunt = 0;
                        int16_t tmp_resdiv = 0;

                        /* get value of internal shunt resistor */
                        if (EE_GET(LINAA_INT_SHUNT) == 0) {
                            tmp_ishunt = LINAA_R_INTSHUNT_DEFAULT;
                        }
                        else {
                            /* use default value because there is no trim value present */
                            tmp_ishunt = EE_GET(LINAA_INT_SHUNT);
                        }
                        /* calculate factor due to external shunt resistor */
                        tmp_resdiv =
                            (int16_t)((int32_t)(((2 * LINAA_R_BONDWIRE) + tmp_ishunt) * (int32_t)LINAA_SCALE_RESDIV) / \
                                      (int16_t)(LINAA_R_EXTSHUNT)) + (int16_t)(1 * LINAA_SCALE_RESDIV);
                        if(StepNumber == LIN_AA_BSM_SNPD_STEP4) {
                            tmp_negcurr = LINAA_INTSHUNT_PRE_CURR;
                        }
                        else {
                            /* step 6 */
                            tmp_negcurr = LINAA_INTSHUNT_SEL_CURR;
                        }
                        /* calculate current compensation due to external shunt resistor and bond wires */
                        Idiff =
                            (int16_t)((int32_t)((int32_t)((int32_t)Idiff * (int32_t)tmp_resdiv) +
                                                (int32_t)tmp_negcurr) / \
                                      (int16_t)LINAA_SCALE_RESDIV);
                    }
#endif
                    if (StepNumber == LIN_AA_BSM_SNPD_STEP4) { /* BSM - Step 4 */
                        /* check if the measured current Idiff is below threshold */
                        if (Idiff > ((int16_t) LIN_AA_CURRLIMIT_PRE)) {

                            mdl_LIN_AA_set_AutoAddressingFlags(LIN_AA_SLAVEWAITING);

                        }
                        /* check if the measured current Idiff is below negative threshold */
                        else if(Idiff < ((-1)*((int16_t) LIN_AA_CURRLIMIT_SEL))) {
                            mdl_LIN_AA_set_AutoAddressingFlags(LIN_AA_SLAVEWAITING);
                        }
                    }
                    else { /* BSM - Step 6 */
                           /* check if the measured current Idiff is below threshold */
                        if (Idiff < ((int16_t) LIN_AA_CURRLIMIT_SEL)) {

                            /* check if current value is greater than negative threshold */
                            if(Idiff >= ((-1)*((int16_t) LIN_AA_CURRLIMIT_SEL))) {
                                /* we are the last slave: set LASTSLAVE flag and to take the address in the LIN API*/
                                mdl_LIN_AA_set_AutoAddressingFlags(LIN_AA_LASTSLAVE);
                            }
                            /* check if current value is smaller then negative threshold and greater then (self driven current + Pullup-current) */
                            else if ((Idiff>((-1)*(((int16_t) LIN_AA_CURRLIMIT_SEL)*3)))) {
                                mdl_LIN_AA_set_AutoAddressingFlags(LIN_AA_INVALID_HW_DETECT);
                            }
                        }
                        else {

                            /* we are not the last slave: nothing to do.  Stay in the running for the next break. */

                        }

                    }
                }
                else { /* BSM - Step 2 */
                    Ishunt_0.commonmode = Ishunt_1.commonmode;
                    Ishunt_0.differential = Ishunt_1.differential >> 3;   /* divide by 8 for averaging */
                }
                break;

            /* -------------------------------------------------------------------------
             * LIN Bus Shunt Method (BSM) Slave Node Position Detection (SNPD) - Step 3:
             * -------------------------------------------------------------------------
             * - switching on pull-up resistor on LIN_IN
             */
            case LIN_AA_BSM_SNPD_STEP3:
#if LINAACSPRESEL == 0
                /* enable integrated LIN slave termination */
                LIN_ENABLE_TERMINATION();
#else
                /* set the LIN autoaddressing selection current on LIN_IN */
                LIN_AA_SET_CURRENT_OUTPUT();
#endif
                break;
            /* -------------------------------------------------------------------------
             * LIN Bus Shunt Method (BSM) Slave Node Position Detection (SNPD) - Step 5:
             * -------------------------------------------------------------------------
             * - all not pre-selected (first action A) nodes are switching off their current source 1
             * - all pre-selected (second action B) nodes are switching on their current source 2 on LIN_IN
             */
            case LIN_AA_BSM_SNPD_STEP5:
                /* set the LIN autoaddressing selection current on LIN_IN */
#if LINAACSPRESEL == 0
                LIN_AA_SET_CURRENT_OUTPUT();
#else
                if((EE_GET(LINAA_GAINDMCAL) == 0) || (EE_GET(LINAA_GAIN) == 0) || (EE_GET(LINAA_LCD_SEL_205) == 0)) {
                    IO_SET(PORT_LC_MISC, LCDIAG_SEL, VAL_DEFAULT_LCD_SEL_240);
                }
                else {
                    IO_SET(TRIM_MISC, LCDIAG_TRIM, EE_GET(LINAA_TRIM_LCD_240));
                    IO_SET(PORT_LC_MISC, LCDIAG_SEL, (0x04u | EE_GET(LINAA_LCD_SEL_240)));
                }
#endif
                break;

            /* -------------------------------------------------------------------------
             * LIN Bus Shunt Method (BSM) Slave Node Position Detection (SNPD) - Step 7
             * or Step 0 (error detected):
             * -------------------------------------------------------------------------
             * - Switching off all current sources and switching on the LIN termination
             */
            default:
                LIN_AA_STOP();

                /* enable integrated LIN slave termination */
                LIN_ENABLE_TERMINATION();

                /* stop ADC hardware trigger */
                CTimer0_Stop();

                break;
        }

    }

}
#endif /* HAS_LIN_AUTOADDRESSING */

/* initialize and run the LIN autoaddressing sequence */

void mdl_LIN_AA_stopAutoAddressing(void){

    mdl_LIN_AA_reset_AutoAddressingFlags(LIN_AA_RESETALLFLAGS);

    /* disable interrupts during LIN break field */
    (void)ml_AutoAddressingConfig(ML_DISABLED);

    /* stop ADC HW trigger */
    CTimer0_Stop();

    LIN_AA_STOP();

    /* enable integrated LIN slave termination */
    LIN_ENABLE_TERMINATION();

    /* disable write access to register LIN_XCFG for enable/disable LIN termination*/
    LIN_RESET_XKEY();

    /* switch back to VDDA */
    IO_SET(PORT_MISC_OUT,SWITCH_VDDA_TO_5V,VAL_VDDA_3V6);

    /* trim value for LCx diag current */
    IO_SET(TRIM_MISC, LCDIAG_TRIM, EE_GET(LCDIAG));
}

/** Reset the specified flags in the status byte AutoAddressingFlags.
 * @param LIN_AA_AutoAddressingFlag - auto addressing flags which should be cleared
 */
void mdl_LIN_AA_reset_AutoAddressingFlags(uint8_t LIN_AA_AutoAddressingFlag){

    ENTER_SECTION(ATOMIC_KEEP_MODE);
    LIN_AA_AutoAddressingFlags &= ~LIN_AA_AutoAddressingFlag;
    EXIT_SECTION();
} /* mdl_LIN_AA_reset_AutoAddressingFlags */

/** Set the specified flags in the status byte AutoAddressingFlags.
 * @param LIN_AA_AutoAddressingFlag - auto addressing flags which should be set
 */
void mdl_LIN_AA_set_AutoAddressingFlags(uint8_t LIN_AA_AutoAddressingFlag){
    ENTER_SECTION(ATOMIC_KEEP_MODE);
    LIN_AA_AutoAddressingFlags |= LIN_AA_AutoAddressingFlag;
    EXIT_SECTION();
} /* mdl_LIN_AA_set_AutoAddressingFlags */


/* ==========================================================================
 * Implementation private functions
 * ========================================================================== */

/* initialize ports for LIN auto-addressing */

void mdl_LIN_AA_loaddefault(void){
        /* load default values */
        IO_SET(TRIM_MISC, LCDIAG_TRIM, VAL_DEFAULT_LCD_TRIM);
#if LINAACSPRESEL == 0
        IO_SET(PORT_LC_MISC, LCDIAG_SEL, VAL_DEFAULT_LCD_SEL_205);
#else
        IO_SET(PORT_LC_MISC, LCDIAG_SEL, VAL_DEFAULT_LCD_SEL_045);
#endif
    IO_SET(PORT_LINAA,
           LINAA_GAIN, VAL_DEFAULT_GAIN_TRIM,         /* linaa amplifier gain adjustment */
           LINAA_DIV, VAL_DEFAULT_DIV_TRIM);          /* linaa amplifier common-mode rejection adjustment */
}
void mdl_LIN_AA_loadtrim(void){
#if LINAACSPRESEL == 0
        IO_SET(TRIM_MISC, LCDIAG_TRIM, EE_GET(LINAA_TRIM_LCD_205));
        IO_SET(PORT_LC_MISC, LCDIAG_SEL, EE_GET(LINAA_LCD_SEL_205));
#else
        IO_SET(TRIM_MISC, LCDIAG_TRIM, EE_GET(LINAA_TRIM_LCD_045));
        IO_SET(PORT_LC_MISC, LCDIAG_SEL, EE_GET(LINAA_LCD_SEL_045));
#endif
        IO_SET(PORT_LINAA,
               LINAA_GAIN, EE_GET(LINAA_GAIN),         /* linaa amplifier gain adjustment */
               LINAA_DIV, EE_GET(LINAA_DIV));          /* linaa amplifier common-mode rejection adjustment */
}
void mdl_LIN_AA_init_ports(void){
    if ((nvram_CalcCRC((uint16_t *)EE_APP_TRIM_AREA_START, EE_APP_TRIM_AREA_SIZE>>1) == NVRAM_CORRECT_CRC) && \
        ((EE_GET(LINAA_GAIN) != 0) && (EE_GET(LINAA_LCD_SEL_205) != 0) && (EE_GET(LINAA_GAINDMCAL) != 0))) {
        mdl_LIN_AA_loadtrim();
    }
    else {
        mdl_LIN_AA_loaddefault();
    }
}

/* EOF */
