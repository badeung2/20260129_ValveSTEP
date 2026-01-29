/**
 * @file
 * @brief Complex timer inline functions definition
 * @internal
 *
 * @copyright (C) 2018 Melexis N.V.
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
 * @ingroup ctimer
 *
 * @brief Complex timer inline functions definition
 * @details
 * @{
 */

#ifndef CTIMERLIB_INLINE_IMPL_H
#define CTIMERLIB_INLINE_IMPL_H

#include <stdint.h>
#include "io.h"

STATIC INLINE void CTimer0_AutoloadInit(CTimer_Clockdivider_t divider, uint16_t cmpb)
{
    IO_SET(CTIMER0, TREGB, cmpb);
    IO_SET(CTIMER0, DIV, (uint16_t)divider, MODE, (uint16_t)eTimerModeTimer);
    IO_SET(CTIMER0, START, 1u);
}


#ifdef IO_CTIMER1__START_GET
STATIC INLINE void CTimer1_AutoloadInit(CTimer_Clockdivider_t divider, uint16_t cmpb)
{
    IO_SET(CTIMER1, TREGB, cmpb);
    IO_SET(CTIMER1, DIV, (uint16_t)divider, MODE, (uint16_t)eTimerModeTimer);
    IO_SET(CTIMER1, START, 1u);
}
#endif /* CTIMER1 */


STATIC INLINE void CTimer0_CompareInit(CTimer_Clockdivider_t divider, uint16_t cmpa,
                                       uint16_t cmpb, CTimer_EnCmp_t enCmp)
{
    IO_SET(CTIMER0, TREGA, cmpa);
    IO_SET(CTIMER0, TREGB, cmpb);
    IO_SET(CTIMER0, DIV, (uint16_t)divider, MODE, (uint16_t)eTimerModeDualCompare, ENCMP, (uint16_t)enCmp);
    IO_SET(CTIMER0, START, 1u);
}


#ifdef IO_CTIMER1__START_GET
STATIC INLINE void CTimer1_CompareInit(CTimer_Clockdivider_t divider, uint16_t cmpa,
                                       uint16_t cmpb, CTimer_EnCmp_t enCmp)
{
    IO_SET(CTIMER1, TREGA, cmpa);
    IO_SET(CTIMER1, TREGB, cmpb);
    IO_SET(CTIMER1, DIV, (uint16_t)divider, MODE, (uint16_t)eTimerModeDualCompare, ENCMP, (uint16_t)enCmp);
    IO_SET(CTIMER1, START, 1u);
}
#endif /* CTIMER1 */


STATIC INLINE void CTimer0_CaptureInit(CTimer_Config_t cfg, uint16_t capa,
                                       uint16_t capb)
{
    IO_SET(CTIMER0, TREGA, capa);
    IO_SET(CTIMER0, TREGB, capb);
    IO_SET(CTIMER0, DIV, (uint16_t)cfg.clockDivider,
           MODE, (uint16_t)eTimerModeDualCapture,
           EDGA, (uint16_t)cfg.edgeSelectorA,
           EDGB, (uint16_t)cfg.edgeSelectorB,
           ENCMP, (uint16_t)cfg.enCmp);
    IO_SET(CTIMER0, START, 1u);
}


#ifdef IO_CTIMER1__START_GET
STATIC INLINE void CTimer1_CaptureInit(CTimer_Config_t cfg, uint16_t capa,
                                       uint16_t capb)
{
    IO_SET(CTIMER1, TREGA, capa);
    IO_SET(CTIMER1, TREGB, capb);
    IO_SET(CTIMER1, DIV, (uint16_t)cfg.clockDivider,
           MODE, (uint16_t)eTimerModeDualCapture,
           EDGA, (uint16_t)cfg.edgeSelectorA,
           EDGB, (uint16_t)cfg.edgeSelectorB,
           ENCMP, (uint16_t)cfg.enCmp);
    IO_SET(CTIMER1, START, 1u);
}
#endif /* CTIMER1 */


STATIC INLINE void CTimer0_CaptureCompareInit(CTimer_Clockdivider_t divider,
                                              uint16_t cmpa, uint16_t capb, CTimer_EdgeSelector_t edgb)
{
    IO_SET(CTIMER0, TREGA, cmpa);
    IO_SET(CTIMER0, TREGB, capb);
    IO_SET(CTIMER0, DIV, (uint16_t)divider,
           MODE, (uint16_t)eTimerModeDualCaptureAndCompare,
           EDGB, (uint16_t)edgb);
    IO_SET(CTIMER0, START, 1u);
}

#ifdef IO_CTIMER1__START_GET
STATIC INLINE void CTimer1_CaptureCompareInit(CTimer_Clockdivider_t divider,
                                              uint16_t cmpa, uint16_t capb, CTimer_EdgeSelector_t edgb)
{
    IO_SET(CTIMER1, TREGA, cmpa);
    IO_SET(CTIMER1, TREGB, capb);
    IO_SET(CTIMER1, DIV, (uint16_t)divider,
           MODE, (uint16_t)eTimerModeDualCaptureAndCompare,
           EDGB, (uint16_t)edgb);
    IO_SET(CTIMER1, START, 1u);
}
#endif /* CTIMER1 */


STATIC INLINE void CTimer0_HighResPwmInit(CTimer_Clockdivider_t divider, uint16_t cmpa,
                                          uint16_t cmpb, CTimer_Polarity_t pol)
{
    IO_SET(CTIMER0, TREGA, cmpa);
    IO_SET(CTIMER0, TREGB, cmpb);
    IO_SET(CTIMER0, DIV, (uint16_t)divider, POL, (uint16_t)pol, MODE, (uint16_t)eTimerModePWM);
    IO_SET(CTIMER0, START, 1u);
}


#ifdef IO_CTIMER1__START_GET
STATIC INLINE void CTimer1_HighResPwmInit(CTimer_Clockdivider_t divider, uint16_t cmpa,
                                          uint16_t cmpb, CTimer_Polarity_t pol)
{
    IO_SET(CTIMER1, TREGA, cmpa);
    IO_SET(CTIMER1, TREGB, cmpb);
    IO_SET(CTIMER1, DIV, (uint16_t)divider, POL, (uint16_t)pol, MODE, (uint16_t)eTimerModePWM);
    IO_SET(CTIMER1, START, 1u);
}
#endif /* CTIMER1 */


STATIC INLINE uint16_t CTimer0_Counter(void)
{
    return IO_GET(CTIMER0, TCNT);
}


#ifdef IO_CTIMER1__START_GET
STATIC INLINE uint16_t CTimer1_Counter(void)
{
    return IO_GET(CTIMER1, TCNT);
}
#endif /* CTIMER1 */


STATIC INLINE void CTimer0_Start(void)
{
    IO_SET(CTIMER0, START, 1u);

}


#ifdef IO_CTIMER1__START_GET
STATIC INLINE void CTimer1_Start(void)
{
    IO_SET(CTIMER1, START, 1u);

}
#endif /* CTIMER1 */


STATIC INLINE void CTimer0_Stop(void)
{
    IO_SET(CTIMER0, STOP, 1u);
}

#ifdef IO_CTIMER1__START_GET
STATIC INLINE void CTimer1_Stop(void)
{
    IO_SET(CTIMER1, STOP, 1u);
}
#endif /* CTIMER1 */


STATIC INLINE uint8_t CTimer0_PwmState(void)
{
    return (uint8_t)IO_GET(CTIMER0, PWMI);
}


#ifdef IO_CTIMER1__START_GET
STATIC INLINE uint8_t CTimer1_PwmState(void)
{
    return (uint8_t)IO_GET(CTIMER1, PWMI);
}
#endif /* CTIMER1 */

#ifdef IO_PORT_TIMER_CFG__TIMER0_CHA_SEL_GET
STATIC INLINE void CTimer0_ChanASel(CTimerCHinputSelect_t sel)
{
    IO_SET(PORT_TIMER_CFG, TIMER0_CHA_SEL, (uint16_t)sel);
}
#endif /* CTimer0 MUX */

#ifdef IO_PORT_TIMER_CFG__TIMER0_CHB_SEL_GET
STATIC INLINE void CTimer0_ChanBSel(CTimerCHinputSelect_t sel)
{
    IO_SET(PORT_TIMER_CFG, TIMER0_CHB_SEL, (uint16_t)sel);
}
#endif /* CTimer0 MUX */

#if defined(IO_PORT_TIMER_CFG__TIMER1_CHA_SEL_GET)
STATIC INLINE void CTimer1_ChanASel(CTimerCHinputSelect_t sel)
{
    IO_SET(PORT_TIMER_CFG, TIMER1_CHA_SEL, (uint16_t)sel);
}
#elif defined(IO_PORT_TIMER_CFG1__TIMER1_CHA_SEL_GET)
STATIC INLINE void CTimer1_ChanASel(CTimerCHinputSelect_t sel)
{
    IO_SET(PORT_TIMER_CFG1, TIMER1_CHA_SEL, (uint16_t)sel);
}
#endif /* CTimer1 MUX */

#if defined(IO_PORT_TIMER_CFG__TIMER1_CHB_SEL_GET)
STATIC INLINE void CTimer1_ChanBSel(CTimerCHinputSelect_t sel)
{
    IO_SET(PORT_TIMER_CFG, TIMER1_CHB_SEL, (uint16_t)sel);
}
#elif defined(IO_PORT_TIMER_CFG1__TIMER1_CHB_SEL_GET)
STATIC INLINE void CTimer1_ChanBSel(CTimerCHinputSelect_t sel)
{
    IO_SET(PORT_TIMER_CFG1, TIMER1_CHB_SEL, (uint16_t)sel);
}
#endif /* CTimer1 MUX */

#ifdef IO_CTIMER1__START_GET
/**
 *  CTIMER1_1, CTIMER1_2, CTIMER1_3
 */
#endif


STATIC INLINE void CTimer0_Int1_Enable(void)
{
    IO_SET(MLX16, CTIMER0_1_PEND, 1u);
    IO_SET(MLX16, CTIMER0_1_ITC, 1u);
}

STATIC INLINE void CTimer0_Int2_Enable(void)
{
    IO_SET(MLX16, CTIMER0_2_PEND, 1u);
    IO_SET(MLX16, CTIMER0_2_ITC, 1u);
}

STATIC INLINE void CTimer0_Int3_Enable(void)
{
    IO_SET(MLX16, CTIMER0_3_PEND, 1u);
    IO_SET(MLX16, CTIMER0_3_ITC, 1u);
}

STATIC INLINE void CTimer0_Int1_Disable(void)
{
    IO_SET(MLX16, CTIMER0_1_ITC, 0u);
}

STATIC INLINE void CTimer0_Int2_Disable(void)
{
    IO_SET(MLX16, CTIMER0_2_ITC, 0u);
}

STATIC INLINE void CTimer0_Int3_Disable(void)
{
    IO_SET(MLX16, CTIMER0_3_ITC, 0u);
}

STATIC INLINE void CTimer0_Int1_Clear(void)
{
    IO_SET(MLX16, CTIMER0_1_PEND, 1u);
}

STATIC INLINE void CTimer0_Int2_Clear(void)
{
    IO_SET(MLX16, CTIMER0_2_PEND, 1u);
}

STATIC INLINE void CTimer0_Int3_Clear(void)
{
    IO_SET(MLX16, CTIMER0_3_PEND, 1u);
}


#ifdef IO_CTIMER1__START_GET

STATIC INLINE void CTimer1_Int1_Enable(void)
{
    IO_SET(MLX16, CTIMER1_1_PEND, 1u);
    IO_SET(MLX16, CTIMER1_1_ITC, 1u);
}

STATIC INLINE void CTimer1_Int2_Enable(void)
{
    IO_SET(MLX16, CTIMER1_2_PEND, 1u);
    IO_SET(MLX16, CTIMER1_2_ITC, 1u);
}

STATIC INLINE void CTimer1_Int3_Enable(void)
{
    IO_SET(MLX16, CTIMER1_3_PEND, 1u);
    IO_SET(MLX16, CTIMER1_3_ITC, 1u);
}

STATIC INLINE void CTimer1_Int1_Disable(void)
{
    IO_SET(MLX16, CTIMER1_1_ITC, 0u);
}

STATIC INLINE void CTimer1_Int2_Disable(void)
{
    IO_SET(MLX16, CTIMER1_2_ITC, 0u);
}

STATIC INLINE void CTimer1_Int3_Disable(void)
{
    IO_SET(MLX16, CTIMER1_3_ITC, 0u);
}

STATIC INLINE void CTimer1_Int1_Clear(void)
{
    IO_SET(MLX16, CTIMER1_1_PEND, 1u);
}

STATIC INLINE void CTimer1_Int2_Clear(void)
{
    IO_SET(MLX16, CTIMER1_2_PEND, 1u);
}

STATIC INLINE void CTimer1_Int3_Clear(void)
{
    IO_SET(MLX16, CTIMER1_3_PEND, 1u);
}
#endif /* CTIMER1 */

/// @}
#endif /* HAL_COMPLEX_TIMER_IMPL_H */
