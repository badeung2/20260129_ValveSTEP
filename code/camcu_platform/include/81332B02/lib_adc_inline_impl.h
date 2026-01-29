/**
 * @file
 * @brief ADC support library
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
 * @ingroup adc
 *
 * @details
 * @{
 */

#ifndef LIB_ADC_INLINE_IMPL_H
#define LIB_ADC_INLINE_IMPL_H

STATIC INLINE uint16_t AdcGetDac(void)
{
    return IO_GET(ADC_SAR, ADC_DATA);
}

STATIC INLINE void AdcInit(uint8_t AdcDiv, const void* SBase,
                           AdcControl_t ctrl)
{
    IO_SET(PORT_ADC_CTRL, ADC_EN, 1u);
    IO_SET(ADC_SAR, STOP, 1u); /* STOP the ADC from any ADC mode. */
    IO_SET(ADC_SAR, ADC_CLK_DIV, AdcDiv);
    IO_SET(ADC_SAR, SBASE_0, (uint16_t )SBase);
    IO_SET(ADC_SAR, START, 0u, /* Don't impact on Start-Stop */
           STOP, 0u,
           SOS_SOURCE, (uint16_t)ctrl.s.sosSource,
           SOC_SOURCE, (uint16_t)ctrl.s.socSource,
           NO_INTERLEAVE, ctrl.s.noInterleave,
           SATURATE, ctrl.s.saturate,
           INT_SCHEME, (uint16_t)ctrl.s.intScheme,
           ASB, (uint16_t)ctrl.s.asb,
           ADC_WIDTH, (uint16_t)ctrl.s.adcWidth);
}

STATIC INLINE void AdcStart(void)
{
    IO_HOST(ADC_SAR, START) = (uint16_t)1u << IO_OFFSET(ADC_SAR, START); /* START the ADC. */
}

STATIC INLINE void AdcStop(void)
{
    IO_HOST(ADC_SAR, STOP) = (uint16_t)1u << IO_OFFSET(ADC_SAR, STOP); /* STOP the ADC. */
}

STATIC INLINE void AdcPause(void)
{
    IO_HOST(ADC_SAR, PAUSE) = (uint16_t)1u << IO_OFFSET(ADC_SAR, PAUSE);
}

STATIC INLINE void AdcSwTrigger(void)
{
    IO_SET(ADC_SAR, SW_TRIG, 1u);
}

STATIC INLINE void AdcStopBlocking(void)
{
    if (IO_GET(ADC_SAR, STOP) != 1u) { /* If ADC is not stopped, we need to stop it first */
        IO_SET(ADC_SAR, STOP, 1u);
        DELAY(3u);  /* Make sure we will have the ADC HW reaction */
        while (IO_GET(ADC_SAR, ABORTED ) == 0u) {
            /* Wait for the ADC will be aborted after the STOP command */
        }
    }
}

STATIC INLINE void AdcResume(void)
{
    IO_HOST(ADC_SAR, RESUME) = (uint16_t)1u << IO_OFFSET(ADC_SAR, RESUME);
}

STATIC INLINE AdcState_t AdcGetState(void)
{
    AdcState_t tmp;
    tmp.data = IO_HOST(ADC_SAR, READY);
    return (AdcState_t)tmp;
}

STATIC INLINE bool AdcIsBusy(void)
{
    return (IO_GET(ADC_SAR, STOP) == 0u);
}

STATIC INLINE void AdcClearAllErrors(void)
{
    IO_SET(ADC_SAR,
           ABORTED, 1u,
           FRAME_ERR, 1u,
           MEM_ERR, 1u,
           ADC_ERR, 1u,
           ADC_OVF, 1u);
}

/// @}
#endif /* LIB_ADC_INLINE_IMPL_H */
