/**
 * @file
 * @brief LIN auto addressing library
 * @internal
 *
 * @copyright (C) 2019-2021 Melexis N.V.
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
 * @ingroup fw_lin_auto_addressing
 *
 * @details Implementations of the LIN auto addressing library
 */
#include <stdlib.h>
#include "atomic.h"
#include "compiler_abstraction.h"
#include "ctimerlib.h"
#include "eeprom_parameters.h"
#include "itc_helper.h"
#include "io.h"
#include "lib_adc.h"
#include "lin_api_sl.h"
#include "lin_core_sl.h"
#include "adc_refs.h"
#include "lib_miscio.h"
#include "sys_tools.h"
#include "mls_api.h"
#include "mls_support.h"
#include "memory_map.h"

#include "fw_mls_api.h"
#include "fw_lin_auto_addressing.h"
#include "fw_lin_auto_addressing_private.h"

/** adc measurement buffer (adc measurements + crc) */
volatile uint16_t LIN_AA_adcValue[LIN_AA_ADC_SAMPLES + 1];

/** adc measurement table for lin autoaddressing*/
AdcSData_t SBASE_LIN[] =
{
    {.u16=(uint16_t)&LIN_AA_adcValue[0]},
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
#ifdef HAS_HW_EC_ADC
    {{.adcEos=ADC_EOS_SIGN}},
#else
    {{.adcEosSign=ADC_EOS_SIGN}},
#endif
    {.u16=(uint16_t)&SBASE_LIN[0]}
};

/** check that SBASE_LIN array has the expected length */
ASSERT(sizeof(SBASE_LIN)/sizeof(SBASE_LIN[0])==(LIN_AA_ADC_SAMPLES + 3));

#if defined HAS_LIN_AA_DRV_SUP_ENABLE
uint16_t l_u16PreLinAA_MotorDrvState;                   /**< storage of the predriver state from before lin aa */
#endif  /* HAS_LIN_AA_DRV_SUP_ENABLE */
static uint8_t u8linaaTimeout = 0u;                     /**< percentage of the remaining lin aa timeout */
static bool isLinaaTimeoutReload = false;               /**< flag indicates wether the lin aa timeout timer needs
                                                         *   reloading */
static uint16_t u16AdcTimerPeriod;                      /**< adc sampling timer period configuration, will be changed
                                                         *   based on the lin baudrate at the start of aa process */
static uint16_t u16FrameTimeout;                        /**< frame timeout during lin auto addressing */
volatile uint8_t LIN_AA_AutoAddressingFlags = 0x00u;    /**< this modules current status flags */
static bool SlowBaudrateAdjustment = false;             /**< flag indicating that the step timings need slight
                                                         *   adjustment for slow speed mode */

#ifdef LINAA_DEBUG_DATA
static LINShunt_t IshuntOffset;                         /**< offset shunt measurement samples */
static LINShunt_t IshuntPresel;                         /**< pre-selection shunt measurement samples */
static int16_t i16PreSelectCurrent;                     /**< current measured during pre-selection step */
static LINShunt_t IshuntSelect;                         /**< pre-selection shunt measurement samples */
static int16_t i16SelectCurrent;                        /**< current measured during selection step */
#endif


/** Lin auto addressing event: application stop
 *
 * Asks the application permission to start lin auto addressing mode.
 * This is default implementation which always allows.
 *
 * @retval true application is stopped and lin auto addressing mode can be entered
 * @retval false lin auto addressing mode entering isn't possible
 * @note the application should make sure in this routine to secure the system
 *       in such a state that it can cope with unavailable adc and timer0 module.
 */
__attribute__((weak)) bool fw_linaa_ApplicationStop(void);

/** Lin auto addressing event: application start
 *
 * Informs the application that the lin auto addressing has been finished and that
 * it can claim the adc and timer0 module again.
 */
__attribute__((weak)) void fw_linaa_ApplicationStart(void);

/** start the LIN autoaddressing sequence */
STATIC void fw_linaa_StartAutoAddressing(void);

/** Lin event: Auto-Addressing pulse
 *
 * @param[in]  StepNumber  step of the auto addressing pulse.
 */
void l_AutoAddressingStep(uint8_t StepNumber);

/** stop LIN-AA */
STATIC void fw_linaa_StopAutoAddressing(void);

/** execute a single step LIN AA shunt measurement
 *
 * @param  measurement  pointer to where to store the measurement results.
 */
STATIC void fw_linaa_DoShuntMeasurement(LINShunt_t * measurement);

/** LIN handler event: LIN AA SID=0xB5 diagnostic frames processing.
 *
 * Processes LIN AA 0xB5 configuration frames: sub functions 1..4.
 */
STATIC bool fw_linaa_RequestHandler(LINDiagTransfer_t *transfer);


void fw_linaa_Init(void)
{
    u8linaaTimeout = 0u;
    isLinaaTimeoutReload = false;
    (void)ldt_SubscribeMultipleHandler(&ld_RequestMultipleHandler, fw_linaa_RequestHandler);
}

bool fw_linaa_ApplicationStop(void)
{
    /* Always allow to start auto addressing */
    return true;
}

void fw_linaa_ApplicationStart(void)
{
    /* empty default handler */
}

STATIC void fw_linaa_StartAutoAddressing(void)
{
    if (fw_linaa_ApplicationStop()) {
        uint16_t u16LinBaudrate;

        isLinaaTimeoutReload = true;

        u16LinBaudrate = ml_GetBaudRate(MLX4_FPLL);

        if (u16LinBaudrate < 12000u)
        {
            /* enable extra delay at 9600 Baud */
            SlowBaudrateAdjustment = true;
        }
        else
        {
            SlowBaudrateAdjustment = false;
        }

        /* measure over ~2.2 Tbit */
        u16AdcTimerPeriod = divU16_U32byU16((uint32_t)(FPLL * 1000UL * 2.2 / (LIN_AA_ADC_SAMPLES + 1)), u16LinBaudrate);

        /* calculate frame timeout, this is max frame time - 12ticks for deducting LINAA time */
        u16FrameTimeout = divU16_U32byU16((uint32_t)(FPLL * 1000UL / 16 * (DIAG_FRAME_TIMEOUT - 12)), u16LinBaudrate);

        LIN_AA_AutoAddressingFlags = LIN_AA_AUTOADDRESSENABLE;

#if defined (HAS_LIN_AA_DRV_SUP_ENABLE)
        /* LIN-AA need the motor driver 5.25V, and therefor the motor-driver must be enabled */
        l_u16PreLinAA_MotorDrvState = IO_GET(PORT_DRV_OUT, ENABLE_DRVSUP);
        IO_SET(PORT_DRV_OUT, ENABLE_DRVSUP , 1u);
#endif  /* HAS_LIN_AA_DRV_SUP_ENABLE */
#if defined (HAS_LIN_AA_VDDA_5V)
        /* switch VDDA to 5V for CM */
        IO_SET(PORT_MISC_OUT, SWITCH_VDDA_TO_5V, 1u);
        DELAY_US(250u);
#endif  /* HAS_LIN_AA_VDDA_5V */

#if defined (IO_ADC_SAR__START_GET)
        /* disable adc interrupts as not used by this module */
        Itc_Disable(ADC_SAR);
#elif defined (IO_ADC_BLOCK__START_GET)
        /* disable adc interrupts as not used by this module */
        Itc_Disable(ADC_BLOCK);
#else
 #error Unknown ADC module
#endif

        /* stop the current ADC action */
        AdcStopBlocking();      /* TODO replace by better stop */

        /* prior to starting the adc clear all available errors */
        AdcClearAllErrors();
        AdcResume();

#if defined (IO_ADC_SAR__START_GET)
        /* create ADC init structure */
        AdcControl_t adc_ctrl = {{.sosSource=ADC_SOS_FIRST_HARDWARE_TRIGGER,
                                  .socSource=ADC_SOC_HARDWARE_TRIGGER,
                                  .noInterleave=1u,
                                  .saturate=1u,
                                  .intScheme=ADC_INT_NO_INT,
                                  .asb=ADC_ASB_NEVER,
                                  .adcWidth=ADC_WDT_16Bit}};

        /* initialize adc unit */
        AdcInit(((FPLL / ADCFREQUENCYKHZ) - 1u), (void*)&SBASE_LIN[0], adc_ctrl);
#elif defined (IO_ADC_BLOCK__START_GET)
        /* create adc init structure */
        AdcEcControl_t adc_ctrl = {{.sosSource=ADC_EC_SOS_SECOND_HARDWARE_TRIGGER,
                                    .socSource=ADC_EC_SOC_HARDWARE_TRIGGER,
                                    .noInterleave=1u,
                                    .saturate=1u,
                                    .intScheme=ADC_EC_INT_NO_INT,
                                    .asb=ADC_EC_ASB_NEVER,
                                    .adcWidth=ADC_EC_WDT_16Bit,
                                    .mode=ADC_EC_MODE_NORMAL_MODE,
                                    .startPhi=ADC_EC_START_PHI_SAME,
                                    .outMode=ADC_EC_OUT_MODE_REGULAR,
                                    .force=ADC_EC_FORCE_REGULAR,
                                    .noChop=ADC_EC_NO_CHOP_CHOPPING_ACTIVE,
                                    .intRef=ADC_EC_INT_REF_INTERNAL,
                                    .speed=ADC_EC_SPEED_FULL_BIAS,
                                    .refAlwaysOn=ADC_EC_REF_ALWAYS_ON_ADC_STDBY}};
        AdcEcClkPreChControl_t adc_clkdiv = {{.adcClkDiv=((FPLL / ADCFREQUENCYKHZ) - 1u),
                                              .clkToInt=0u,
                                              .cmPrchrgTime=0u,
                                              .cmPrchrg=ADC_EC_NO_PRECHARGE}};

        /* initialize adc unit */
        AdcInit(adc_clkdiv, (void*)&SBASE_LIN[0], adc_ctrl);
#else
 #error Unknown ADC module
#endif

        /* claim timer 0 */
        CTimer0_Stop();
        CTimer0_Int1_Disable();
        CTimer0_Int2_Disable();
        CTimer0_Int3_Disable();

        /* enable write access to register LIN_XCFG for enable/disable LIN termination*/
        LIN_SET_XKEY();

        /* reset all LIN current and amplifier adjustments */
        LINAA_RESET();

        /* enable interrupts during LIN break field */
        (void)ml_AutoAddressingConfig(ML_ENABLED);
    }
}

void l_AutoAddressingStep(uint8_t StepNumber)
{
    static LINShunt_t Ishunt_0 = {0, 0};
    LINShunt_t TempMeas;
    int16_t i16Diff;

    if (StepNumber == LIN_AA_BSM_SNPD_STEP1)
    {
        /* disable integrated LIN slave termination */
        LIN_DISABLE_TERMINATION();
        LIN_AA_AutoAddressingFlags &= ~(LIN_AA_SLAVEWAITING | LIN_AA_LASTSLAVE);

        CTimer0_Int3_Disable();
        CTimer0_Stop();
    }

    /* check if slave is passive during the next LIN AA steps */
    if ((LIN_AA_AutoAddressingFlags & LIN_AA_SLAVEWAITING) != 0u)
    {
        LINAA_STOP();

        if (StepNumber == LIN_AA_BSM_SNPD_STEP7)
        {
            DELAY_US(10u);

            /* enable integrated LIN slave termination */
            LIN_ENABLE_TERMINATION();

            LIN_AA_AutoAddressingFlags &= ~LIN_AA_SLAVEWAITING;
        }
    }
    else
    {
        switch(StepNumber)
        {
            case LIN_AA_BSM_SNPD_STEP1:
                /* step 1: setup for offset measurement */
                if ((LIN_AA_AutoAddressingFlags & LIN_AA_SLAVEADDRESSED) == 0u)
                {
                    /* slave is not yet addressed, setup for measurements */

                    /* initialise offset measurement */
                    LINAA_INIT_MEASUREMENT();
                }
                else
                {
                    /* slave already addressed -> wait till last auto addressing step and do nothing */
                    LIN_AA_AutoAddressingFlags |= LIN_AA_SLAVEWAITING;
                }
                break;

            case LIN_AA_BSM_SNPD_STEP2:
                /* step 2: perform offset measurement */
                if (SlowBaudrateAdjustment)
                {
                    DELAY_US(40u);
                }

                DELAY_US(5u);
                /* deactivate reset first amplifier gain */
                IO_SET(PORT_LINAA1,
                       LINAA_RST1, 0u,
                       LINAA_RST2, 1u,
                       LINAA_EN, 1u,
                       LINAA_CDOUTEN, 0u);
                DELAY_US(10u);
                /* deactivate reset second variable gain amplifier */
                IO_SET(PORT_LINAA1,
                       LINAA_RST1, 0u,
                       LINAA_RST2, 0u,
                       LINAA_EN, 1u,
                       LINAA_CDOUTEN, 0u);

                fw_linaa_DoShuntMeasurement(&Ishunt_0);

#ifdef LINAA_DEBUG_DATA
                IshuntOffset.commonmode = Ishunt_0.commonmode;
                IshuntOffset.differential = Ishunt_0.differential;
#endif

                break;

            case LIN_AA_BSM_SNPD_STEP3:
                /* step 3: setup for pre-selection measurement */
                if (SlowBaudrateAdjustment)
                {
                    DELAY_US(40u);
                }

#if defined(EE_LINAA_LCD_SEL_045)
                if (EE_GET(CALIB_VERSION) >= LIN_AA_PULLUP_BY_CS_VERSION)
                {
                    /* enable current-source of 0.45mA */
                    LINAA_PRESEL_CURRENT_OUTPUT();
                }
                else
#endif  /* EE_LINAA_LCD_SEL_045 */
                {
                    /* enable integrated LIN slave termination */
                    LIN_ENABLE_TERMINATION();
                }
                break;

            case LIN_AA_BSM_SNPD_STEP4:
                /* step 4: perform pre-selection measurement */
                fw_linaa_DoShuntMeasurement(&TempMeas);

#if defined (IO_ADC_SAR__START_GET)
                if ((IO_GET(ADC_SAR, ADC_OVF) == 0u) &&
                    (TempMeas.commonmode != 0) && (TempMeas.commonmode != ((1u << (10 + 3)) - 1u)) &&
                    (TempMeas.differential != 0) && (TempMeas.differential != ((1u << (10 + 3)) - 1u)))
#elif defined (IO_ADC_BLOCK__START_GET)
                if ((IO_GET(ADC_BLOCK, ADC_OVF) == 0u) &&
                    (TempMeas.commonmode != 0) && (TempMeas.commonmode != ((1u << (12 + 3)) - 1u)) &&
                    (TempMeas.differential != 0) && (TempMeas.differential != ((1u << (12 + 3)) - 1u)))
#else
 #error Unknown ADC module
#endif
                {
                    /* calculate the current over the LIN shunt
                     *
                     * cIDM_cm = LINAA_SDMCM_EE * (mVCM_sel - mVCM_offs) / 2^15
                     * cIDM_dmcm = mIDM_sel - mIDM_offs - cIDM_cm
                     * cIDM = LINAA_GainDMCal_EE * cIDM_dmcm / 2^12
                     */
                    i16Diff = (int16_t)(mulI32_I16byI16(TempMeas.commonmode - Ishunt_0.commonmode, EE_GET(LINAA_SDMCM)) / (1L << 15));
                    i16Diff = ((TempMeas.differential - Ishunt_0.differential) / ADC_DM_MULTIPLIER) - i16Diff;
                    i16Diff = (int16_t)(mulI32_I16byI16(i16Diff, EE_GET(LINAA_GAINDMCAL)) / (1L << 12));

#ifdef LINAA_DEBUG_DATA
                    i16PreSelectCurrent = i16Diff;
                    IshuntPresel.commonmode = TempMeas.commonmode;
                    IshuntPresel.differential = TempMeas.differential;
#endif

                    if (i16Diff > LIN_AA_CURRLIMIT_PRE)
                    {
                        /* measured current is above threshold */
                        LIN_AA_AutoAddressingFlags |= LIN_AA_SLAVEWAITING;
                    }
                    else if (i16Diff < ((-1) * LIN_AA_CURRLIMIT_SEL))
                    {
                        /* measured current is below negative threshold, this might be a reverse polarity situation
                         * therefor allow a bigger threshold */
                        LIN_AA_AutoAddressingFlags |= LIN_AA_SLAVEWAITING;
                    }
                    else
                    {
                        /* empty */
                    }
                }
                else
                {
                    /* measurements are overflowing, makes no sense to continue */
                    LIN_AA_AutoAddressingFlags |= LIN_AA_SLAVEWAITING;
                }
                break;

            case LIN_AA_BSM_SNPD_STEP5:
                /* step 5: setup for selection current measurement */
                if (SlowBaudrateAdjustment)
                {
                    DELAY_US(40u);
                }

#if defined(EE_LINAA_LCD_SEL_240)
                if (EE_GET(CALIB_VERSION) >= LIN_AA_PULLUP_BY_CS_VERSION)
                {
                    /* set the LIN autoaddressing selection current on LIN_IN */
                    LINAA_SEL_CURRENT_240_OUTPUT();
                }
                else
#endif  /* EE_LINAA_LCD_SEL_240 */
                {
                    /* enable integrated LIN slave termination */
                    LIN_ENABLE_TERMINATION();
                    /* set the LIN autoaddressing selection current on LIN_IN */
                    LINAA_SEL_CURRENT_205_OUTPUT();
                }
                break;

            case LIN_AA_BSM_SNPD_STEP6:
                /* step 6: perform selection current measurement */
                fw_linaa_DoShuntMeasurement(&TempMeas);

#if defined (IO_ADC_SAR__START_GET)
                if ((IO_GET(ADC_SAR, ADC_OVF) == 0u) &&
                    (TempMeas.commonmode != 0) && (TempMeas.commonmode != ((1u << (10 + 3)) - 1u)) &&
                    (TempMeas.differential != 0) && (TempMeas.differential != ((1u << (10 + 3)) - 1u)))
#elif defined (IO_ADC_BLOCK__START_GET)
                if ((IO_GET(ADC_BLOCK, ADC_OVF) == 0u) &&
                    (TempMeas.commonmode != 0) && (TempMeas.commonmode != ((1u << (12 + 3)) - 1u)) &&
                    (TempMeas.differential != 0) && (TempMeas.differential != ((1u << (12 + 3)) - 1u)))
#else
 #error Unknown ADC module
#endif
                {
                    /* calculate the current over the LIN shunt
                     *
                     * cIDM_cm = LINAA_SDMCM_EE * (mVCM_sel - mVCM_offs) / 2^15
                     * cIDM_dmcm = mIDM_sel - mIDM_offs - cIDM_cm
                     * cIDM = LINAA_GainDMCal_EE * cIDM_dmcm / 2^12
                     */
                    i16Diff = (int16_t)(mulI32_I16byI16(TempMeas.commonmode - Ishunt_0.commonmode, EE_GET(LINAA_SDMCM)) / (1L << 15));
                    i16Diff = ((TempMeas.differential - Ishunt_0.differential) / ADC_DM_MULTIPLIER) - i16Diff;
                    i16Diff = (int16_t)(mulI32_I16byI16(i16Diff, EE_GET(LINAA_GAINDMCAL)) / (1L << 12));

#ifdef LINAA_DEBUG_DATA
                    i16SelectCurrent = i16Diff;
                    IshuntSelect.commonmode = TempMeas.commonmode;
                    IshuntSelect.differential = TempMeas.differential;
#endif

                    /* check if the measured current i16Diff is below threshold */
                    if (i16Diff < LIN_AA_CURRLIMIT_SEL)
                    {
                        /* check if current value is greater than negative threshold */
                        if (i16Diff >= ((-1) * LIN_AA_CURRLIMIT_SEL))
                        {
                            /* we are the last slave: set LASTSLAVE flag and to take the address in the LIN API */
                            LIN_AA_AutoAddressingFlags |= LIN_AA_LASTSLAVE;
                        }
                        /* check if current value is smaller then negative threshold and greater then (self driven current + Pullup-current) */
                        else if (i16Diff > ((-1) * (LIN_AA_CURRLIMIT_SEL * 3)))
                        {
                            LIN_AA_AutoAddressingFlags |= LIN_AA_LASTSLAVE | LIN_AA_INVALID_HW_CONNECT;
                        }
                    }
                    else
                    {
                        /* we are not the last slave: nothing to do. Stay in the running for the next break. */
                    }
                }
                else
                {
                    /* measurements are overflowing, makes no sense to continue */
                    LIN_AA_AutoAddressingFlags |= LIN_AA_SLAVEWAITING;
                }
                break;

            case LIN_AA_BSM_SNPD_STEP7:
                /* step 7: perform offset measurement */
                if (SlowBaudrateAdjustment)
                {
                    DELAY_US(40u);
                }

                /* switch timer to measure frame timeout */
                CTimer0_Stop();

                /* initialize Complex Timer 0 as ADC hardware trigger */
                CTimer0_Int3_Enable();
                CTimer0_AutoloadInit(eTimerCPUClockDivisionBy16, u16FrameTimeout);

                FALLTHROUGH;

            default:
                /* step 7 or step 0: error detected */
                DELAY_US(10u);

                LINAA_STOP();

                /* enable integrated LIN slave termination */
                LIN_ENABLE_TERMINATION();

                LIN_AA_AutoAddressingFlags |= LIN_AA_SLAVEWAITING;
                break;
        }
    }

    if (COLIN_LINstatus.event_overflow != 0)
    {
        LIN_AA_AutoAddressingFlags = (LIN_AA_AutoAddressingFlags & ~LIN_AA_LASTSLAVE) | LIN_AA_SLAVEWAITING;

        /* clear overflow flag */
        (void)ml_GetState(ML_CLR_LIN_CMD_OVERFLOW);
    }
}

STATIC void fw_linaa_StopAutoAddressing(void)
{
    if ((LIN_AA_AutoAddressingFlags & LIN_AA_AUTOADDRESSENABLE) != 0u)
    {
        LIN_AA_AutoAddressingFlags = 0u;

        /* disable interrupts during LIN break field */
        (void)ml_AutoAddressingConfig(ML_DISABLED);

        /* stop ADC HW trigger */
        CTimer0_Int3_Disable();
        CTimer0_Stop();

        LINAA_STOP();

#if defined(IO_PORT_LINAA1__LINAA_5V_ENABLE)
        IO_SET(PORT_LINAA1, LINAA_5V_ENABLE, 0u);
#endif  /* IO_PORT_LINAA1__LINAA_5V_ENABLE */

        /* enable integrated LIN slave termination */
        LIN_ENABLE_TERMINATION();

        /* disable write access to register LIN_XCFG for enable/disable LIN termination*/
        LIN_RESET_XKEY();

#if defined (HAS_LIN_AA_DRV_SUP_ENABLE)
        /* restore driver state */
        IO_SET(PORT_DRV_OUT, ENABLE_DRVSUP , l_u16PreLinAA_MotorDrvState);
#endif  /* HAS_LIN_AA_DRV_SUP_ENABLE */
#if defined (HAS_LIN_AA_VDDA_5V)
        /* switch back to VDDA */
        IO_SET(PORT_MISC_OUT, SWITCH_VDDA_TO_5V, 0u);
        DELAY_US(250u);
#endif  /* HAS_LIN_AA_VDDA_5V */
#if defined (EE_TRIM_LCD_LCX_GET)
        /* trim value for LCx diag current */
        IO_SET(TRIM_MISC, TRIM_LCD_LINAA, EE_GET(TRIM_LCD_LCX));
#endif  /* EE_TRIM_LCD_LCX_GET */

        fw_linaa_ApplicationStart();
    }
    else
    {
        /* lin aa is not enabled so do nothing */
    }
}

STATIC void fw_linaa_DoShuntMeasurement(LINShunt_t * measurement)
{
#if defined (IO_ADC_SAR__START_GET)
    /* (re)setup for ADC conversion */
    IO_SET(ADC_SAR, SBASE_0, (uint16_t)&SBASE_LIN[0]);
#elif defined (IO_ADC_BLOCK__START_GET)
    /* (re)setup for ADC conversion */
    IO_SET(ADC_BLOCK, SBASE_0, (uint16_t)&SBASE_LIN[0]);
#else
 #error Unknown ADC module
#endif

    AdcClearAllErrors();
    AdcResume();

    /* start ADC conversion */
    AdcStart();

    /* initialize Complex Timer 0 as ADC hardware trigger */
    CTimer0_AutoloadInit(eTimerCPUClockDivisionBy1, u16AdcTimerPeriod);

#if defined (IO_ADC_SAR__START_GET)
    while ( IO_GET(ADC_SAR, STOP) == 0U )
    {
        /* Wait till ADC conversions have been finished */
    }
#elif defined (IO_ADC_BLOCK__START_GET)
    while ( IO_GET(ADC_BLOCK, STOP) == 0U )
    {
        /* Wait till ADC conversions have been finished */
    }
#else
 #error Unknown ADC module
#endif

    CTimer0_Int3_Disable();
    CTimer0_Stop();

    /* do calculation */
    linaa_CalculateSamples(LIN_AA_adcValue, measurement);
}

STATIC bool fw_linaa_RequestHandler(LINDiagTransfer_t *transfer)
{
    bool retval = false;

    if ((transfer->request.dataLen == 0x5u) &&
        (transfer->request.reqSId == 0xB5u) &&
        (transfer->request.data[0] == 0xFFu) &&
        (transfer->request.data[1] == 0x7Fu) &&
        (transfer->request.data[3] == SNPD_METHOD_BSM2))
    {
        switch (transfer->request.data[2]) {
            case LIN_AA_SUBFUNCTION1: /* SNPD sub function ID 0x01 "All BSM-nodes enter the unconfigured mode" */
                fw_linaa_StartAutoAddressing();
                break;

            case LIN_AA_SUBFUNCTION2: /* SNPD sub function ID 0x02 "Informing all slaves about the next NAD" */
                if ((LIN_AA_AutoAddressingFlags & LIN_AA_LASTSLAVE) == LIN_AA_LASTSLAVE)
                {
                    ml_ConfiguredNAD = transfer->request.data[4];   /* use the new NAD */

                    ENTER_SECTION(ATOMIC_KEEP_MODE);
                    LIN_AA_AutoAddressingFlags |= LIN_AA_SLAVEADDRESSED;
                    LIN_AA_AutoAddressingFlags &= ~LIN_AA_LASTSLAVE;
                    EXIT_SECTION();
                }
                break;

            case LIN_AA_SUBFUNCTION3: /* SNPD sub function ID 0x03 "Store the assigned NADs in to the NVM of the slaves, if available" */
                s_ifcStatus.mapped.SaveConfig = true;
                break;

            case LIN_AA_SUBFUNCTION4: /* SNPD sub function ID 0x04 "Informing all slaves that the procedure is finished" */
                fw_linaa_StopAutoAddressing();
                break;

            default:
                break;
        }
        /* No response prepared */
#ifdef LINAA_DEBUG_DATA
    } else if ((transfer->request.dataLen == 0x5u) &&
               (transfer->request.reqSId == 0xB2u) &&
               (isSupplierIdValid(&transfer->request.data[1]) == ML_SUCCESS) &&
               (isFunctionIdValid(&transfer->request.data[3]) == ML_SUCCESS) &&
               (transfer->request.data[0] == 0x30u))
    {
        transfer->response.respSId = 0xB2u + 0x40u;
        transfer->response.dataLen = 16u;

        transfer->response.data[0] = (uint8_t)(IshuntOffset.commonmode >> 0);
        transfer->response.data[1] = (uint8_t)(IshuntOffset.commonmode >> 8);
        transfer->response.data[2] = (uint8_t)(IshuntOffset.differential >> 0);
        transfer->response.data[3] = (uint8_t)(IshuntOffset.differential >> 8);

        transfer->response.data[4] = (uint8_t)(IshuntPresel.commonmode >> 0);
        transfer->response.data[5] = (uint8_t)(IshuntPresel.commonmode >> 8);
        transfer->response.data[6] = (uint8_t)(IshuntPresel.differential >> 0);
        transfer->response.data[7] = (uint8_t)(IshuntPresel.differential >> 8);
        transfer->response.data[8] = (uint8_t)(i16PreSelectCurrent >> 0);
        transfer->response.data[9] = (uint8_t)(i16PreSelectCurrent >> 8);

        transfer->response.data[10] = (uint8_t)(IshuntSelect.commonmode >> 0);
        transfer->response.data[11] = (uint8_t)(IshuntSelect.commonmode >> 8);
        transfer->response.data[12] = (uint8_t)(IshuntSelect.differential >> 0);
        transfer->response.data[13] = (uint8_t)(IshuntSelect.differential >> 8);
        transfer->response.data[14] = (uint8_t)(i16SelectCurrent >> 0);
        transfer->response.data[15] = (uint8_t)(i16SelectCurrent >> 8);

        retval = true;
#endif
    }

    return retval;
}

void fw_linaa_Tick(uint8_t percentTO)
{
    if (isLinaaTimeoutReload) {
        u8linaaTimeout = 100u;
        isLinaaTimeoutReload = false;
    }

    if (u8linaaTimeout != 0u) {
        if (u8linaaTimeout > percentTO) {
            u8linaaTimeout -= percentTO;
        } else {
            u8linaaTimeout = 0u;
            fw_linaa_StopAutoAddressing();
        }
    }
}

void fw_linaa_FrameTimoutHandler(void)
{
    /* reset COLIN as there was a frame timeout otherwise we are not in time for LIN AA steps */
    (void)ml_Disconnect();
    (void)ml_Connect();
    CTimer0_Int3_Disable();
    CTimer0_Stop();
}
