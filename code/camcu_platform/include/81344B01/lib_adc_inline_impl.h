/**
 * @file
 * @brief ADC EC support library
 * @internal
 *
 * @copyright (C) 2015-2017 Melexis N.V.
 * git flash d0014c23
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
 * @ingroup adc_ec
 *
 * @details
 * The ADC Extended Counter support library
 */

#ifndef LIB_ADC_EC_INLINE_IMPL_H
#define LIB_ADC_EC_INLINE_IMPL_H

STATIC INLINE void AdcInit(AdcEcClkPreChControl_t adcClkDivCtrl, const void* sBase,
                           AdcEcControl_t ctrl)
{
    IO_SET(ADC_BLOCK, STOP, 1u); /* STOP the ADC from any ADC mode. */
    IO_SET(ADC_BLOCK, ADC_CLK_DIV, adcClkDivCtrl.s.adcClkDiv,
           CLK_TO_INT, adcClkDivCtrl.s.clkToInt,
           CM_PRCHRG_TIME, adcClkDivCtrl.s.cmPrchrgTime,
           CM_PRCHRG, (uint16_t)adcClkDivCtrl.s.cmPrchrg);

    IO_SET(ADC_BLOCK, SBASE_0, (uint16_t)sBase);
    IO_SET(ADC_BLOCK, START, 0u, /* Don't impact on Start-Stop */
           STOP, 0u,
           SOS_SOURCE, (uint16_t)ctrl.s.sosSource,
           SOC_SOURCE, (uint16_t)ctrl.s.socSource,
           NO_INTERLEAVE, ctrl.s.noInterleave,
           SATURATE, ctrl.s.saturate,
           INT_SCHEME, (uint16_t)ctrl.s.intScheme,
           ASB, (uint16_t)ctrl.s.asb,
           ADC_WIDTH, (uint16_t)ctrl.s.adcWidth);
#ifdef HAS_HW_ADC_SET_CONV_TYPE
    IO_SET(ADC_BLOCK,
           TYPE, (uint16_t)ctrl.s.type,
           MODE, (uint16_t)ctrl.s.mode,
           START_PHI, (uint16_t)ctrl.s.startPhi,
           OUTMODE, (uint16_t)ctrl.s.outMode,
           FORCE, (uint16_t)ctrl.s.force,
           NOCHOP, (uint16_t)ctrl.s.noChop,
           INTREF, (uint16_t)ctrl.s.intRef,
           SPEED, (uint16_t)ctrl.s.speed,
           REF_ALWAYS_ON, (uint16_t)ctrl.s.refAlwaysOn);
#else
    IO_SET(ADC_BLOCK,
           COUNT, (uint16_t)ctrl.s.count,
           MODE, (uint16_t)ctrl.s.mode,
           START_PHI, (uint16_t)ctrl.s.startPhi,
           OUTMODE, (uint16_t)ctrl.s.outMode,
           FORCE, (uint16_t)ctrl.s.force,
           NOCHOP, (uint16_t)ctrl.s.noChop,
           INTREF, (uint16_t)ctrl.s.intRef,
           SPEED, (uint16_t)ctrl.s.speed,
           REF_ALWAYS_ON, (uint16_t)ctrl.s.refAlwaysOn);
#endif /* HAS_HW_ADC_SET_CONV_TYPE */
}

STATIC INLINE void AdcStart(void)
{
    IO_HOST(ADC_BLOCK, START) = (uint16_t)1u << IO_OFFSET(ADC_BLOCK, START); /* START the ADC. */
}

STATIC INLINE void AdcStop(void)
{
    IO_HOST(ADC_BLOCK, STOP) = (uint16_t)1u << IO_OFFSET(ADC_BLOCK, STOP); /* STOP the ADC. */
}

STATIC INLINE void AdcPause(void)
{
    IO_HOST(ADC_BLOCK, PAUSE) = (uint16_t)1u << IO_OFFSET(ADC_BLOCK, PAUSE);
}

STATIC INLINE void AdcSwTrigger(void)
{
    IO_SET(ADC_BLOCK, SW_TRIG, 1u);
}

STATIC INLINE void AdcStopBlocking(void)
{
    if (IO_GET(ADC_BLOCK, STOP) != 1u) { /* If ADC is not stopped, we need to stop it first */
        IO_SET(ADC_BLOCK, STOP, 1u);
        DELAY(3u);  /* Make sure we will have the ADC HW reaction */
        while (IO_GET(ADC_BLOCK, ABORTED ) == 0u) {
            /* Wait for the ADC will be aborted after the STOP command */
        }
    }
}

STATIC INLINE void AdcResume(void)
{
    IO_HOST(ADC_BLOCK, RESUME) = (uint16_t)1u << IO_OFFSET(ADC_BLOCK, RESUME);
}

STATIC INLINE AdcEcState_t AdcGetState(void)
{
    AdcEcState_t tmp;
    tmp.data = IO_HOST(ADC_BLOCK, READY);
    return (AdcEcState_t)tmp;
}

STATIC INLINE bool AdcIsBusy(void)
{
    return (IO_GET(ADC_BLOCK, STOP) == 0u);
}

STATIC INLINE void AdcClearAllErrors(void)
{
    IO_SET(ADC_BLOCK,
           ABORTED, 1u,
           FRAME_ERR, 1u,
           MEM_ERR, 1u,
           ADC_ERR, 1u,
           ADC_OVF, 1u);
}

#endif /* LIB_ADC_EC_INLINE_IMPL_H */
