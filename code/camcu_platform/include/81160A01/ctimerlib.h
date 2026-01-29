/**
 * @file
 * @brief Complex timer support library
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
 * @ingroup ctimer
 * @addtogroup ctimer Complex Timer
 *
 * @brief Complex timer support library
 * @details
 *
 * @{
 */

#ifndef CTIMERLIB_H
#define CTIMERLIB_H

#include <stdint.h>
#include "compiler_abstraction.h"
#include "lib_miscio.h"
#include "io.h"

/** Polarity of the PWM */
typedef enum CTimer_Polarity_e {
    eTimerGatePolarityPositive, /**< PWM Gate positive */
    eTimerGatePolarityNegative  /**< PWM Gate negative */
} CTimer_Polarity_t;

/** Complex timer clock division type */
typedef enum {
    eTimerCPUClockDivisionBy1 = 0U,    /**< Clock is not divided */
    eTimerCPUClockDivisionBy16 = 1U,   /**< Clock divided by 16 */
    eTimerCPUClockDivisionBy256 = 3U,  /**< Clock divided by 256 */
} CTimer_Clockdivider_t;

/** Complex timer capture edge selection type */
typedef enum {
    eTimerNoEdgeSelection = 0U,         /**< No edge is detected */
    eTimerFallingEdgeSelection = 1U,    /**< Falling edge detection */
    eTimerRaisingEdgeSelection = 2U,    /**< Rising edge detection */
    eTimerRaisingAndFallingEdgeSelection = 3U /**< Falling + Rising edges detection */
} CTimer_EdgeSelector_t;

/** Complex timer operation mode type */
typedef enum {
    eTimerModeTimer = 0U,                   /**< Free running timer mode */
    eTimerModeDualCompare = 1U,             /**< Dual compare timer mode */
    eTimerModeDualCapture = 2U,             /**< Dual capture timer mode */
    eTimerModeDualCaptureAndCompare = 3U,   /**< Capture and compare mode mode */
    eTimerModePWM = 6U,                     /**< Timer PWM mode */
} CTimer_OperationMode_t;

/** Complex timer ENCMP config */
typedef enum {
    eTimerEnCmpDisable = 0u,    /**< CTIMER ENCMP setting to "0" */
    eTimerEnCmpEnable = 1u      /**< CTIMER ENCMP setting to "1" */
} CTimer_EnCmp_t;

/** Complex timer configuration structure */
typedef struct {
    CTimer_Clockdivider_t clockDivider;     /**< Clock divider setting */
    CTimer_EdgeSelector_t edgeSelectorA;    /**< Clock EdgeA Selector setting */
    CTimer_EdgeSelector_t edgeSelectorB;    /**< Clock EdgeB Selector setting */
    CTimer_EnCmp_t enCmp;                   /**< 16-bits up counter reset by INT_CAPA or INT_CAPB */
} CTimer_Config_t;

/** CTimer0_AutoloadInit configures Timer0 as autoloading free-running timer and starts it
 * @param[in] divider specifies the divider type for the timer
 * @param[in] cmpb specifies the top value for the timer
 */
STATIC INLINE void CTimer0_AutoloadInit(CTimer_Clockdivider_t divider, uint16_t cmpb);


#ifdef IO_CTIMER1__START_GET
/** CTimer1_AutoloadInit configures Timer1 as autoloading free-running timer and starts it
 * @param[in] divider specifies the divider type for the timer
 * @param[in] cmpb specifies the top value for the timer
 */
STATIC INLINE void CTimer1_AutoloadInit(CTimer_Clockdivider_t divider, uint16_t cmpb);
#endif /* CTIMER1 */


/** CTimer0_CompareInit configures Timer0 in dual compare mode and starts it
 * @param[in] divider specifies the divider type for the timer
 * @param[in] cmpa specifies the top value for the timer to be compared. See the table which Int will be triggered
 * @param[in] cmpb specifies the top value for the timer to be compared. See the table which Int will be triggered
 * @param[in] enCmp specifies whether the 16-bits counter is reset when its value reaches the comparator value CMPB or 0xFFFF
 */
STATIC INLINE void CTimer0_CompareInit(CTimer_Clockdivider_t divider, uint16_t cmpa,
                                       uint16_t cmpb, CTimer_EnCmp_t enCmp);


#ifdef IO_CTIMER1__START_GET
/** CTimer1_CompareInit configures Timer1 in dual compare mode and starts it
 * @param[in] divider specifies the divider type for the timer
 * @param[in] cmpa specifies the top value for the timer to be compared. See the table which Int will be triggered
 * @param[in] cmpb specifies the top value for the timer to be compared. See the table which Int will be triggered
 * @param[in] enCmp specifies whether the 16-bits counter is reset when its value reaches the comparator value CMPB or 0xFFFF
 */
STATIC INLINE void CTimer1_CompareInit(CTimer_Clockdivider_t divider, uint16_t cmpa,
                                       uint16_t cmpb, CTimer_EnCmp_t enCmp);
#endif /* CTIMER1 */


/** CTimer0_CaptureInit configures Timer0 in dual capture mode and starts it
 * @param[in] cfg specifies the set of settings (edges and divider) for the timer
 * @param[in] capa specifies the top value for the timer to be compared with the captured value. See the table which Int will be triggered
 * @param[in] capb specifies the top value for the timer to be compared with the captured value. See the table which Int will be triggered
 */
STATIC INLINE void CTimer0_CaptureInit(CTimer_Config_t cfg, uint16_t capa,
                                       uint16_t capb);


#ifdef IO_CTIMER1__START_GET
/** CTimer1_CaptureInit configures Timer1 in dual capture mode and starts it
 * @param[in] cfg specifies the set of settings (edges and divider) for the timer
 * @param[in] capa specifies the top value for the timer to be compared with the captured value. See the table which Int will be triggered
 * @param[in] capb specifies the top value for the timer to be compared with the captured value. See the table which Int will be triggered
 */
STATIC INLINE void CTimer1_CaptureInit(CTimer_Config_t cfg, uint16_t capa,
                                       uint16_t capb);
#endif /* CTIMER1 */


/** CTimer0_CaptureCompareInit configures Timer0 in capture-and-compare mode and starts it
 * @param[in] divider specifies the divider type for the timer
 * @param[in] cmpa specifies the top value for the timer to be compared with the running value. See the table which Int will be triggered
 * @param[in] capb specifies the top value for the timer to be compared with the captured value. See the table which Int will be triggered
 * @param[in] edgb sets the edge to be captured by the IN_CHB
 * @note function is also enables the timer
 */
STATIC INLINE void CTimer0_CaptureCompareInit(CTimer_Clockdivider_t divider,
                                              uint16_t cmpa, uint16_t capb, CTimer_EdgeSelector_t edgb);


#ifdef IO_CTIMER1__START_GET
/** CTimer1_CaptureCompareInit configures Timer0 in capture-and-compare mode and starts it
 * @param[in] divider specifies the divider type for the timer
 * @param[in] cmpa specifies the top value for the timer to be compared with the running value. See the table which Int will be triggered
 * @param[in] capb specifies the top value for the timer to be compared with the captured value. See the table which Int will be triggered
 * @param[in] edgb sets the edge to be captured by the IN_CHB
 */
STATIC INLINE void CTimer1_CaptureCompareInit(CTimer_Clockdivider_t divider,
                                              uint16_t cmpa, uint16_t capb, CTimer_EdgeSelector_t edgb);
#endif /* CTIMER1 */


/** CTimer0_HighResPwmInit configures Timer0 in the High resolution PWM mode and starts it
 * @param[in] divider specifies the divider type for the timer
 * @param[in] cmpa specifies the first comparator for the timer in PWM mode
 * @param[in] cmpb specifies the second comparator for the timer in PWM mode
 * @param[in] pol sets the startup polarity of the PWM signal
 * @note registers CMPA and CMPB are not buffered
 */
STATIC INLINE void CTimer0_HighResPwmInit(CTimer_Clockdivider_t divider, uint16_t cmpa,
                                          uint16_t cmpb, CTimer_Polarity_t pol);


#ifdef IO_CTIMER1__START_GET
/** CTimer1_HighResPwmInit configures Timer1 in the High resolution PWM mode and starts it
 * @param[in] divider specifies the divider type for the timer
 * @param[in] cmpa specifies the first comparator for the timer in PWM mode
 * @param[in] cmpb specifies the second comparator for the timer in PWM mode
 * @param[in] pol sets the startup polarity of the PWM signal
 * @note registers CMPA and CMPB are not buffered
 */
STATIC INLINE void CTimer1_HighResPwmInit(CTimer_Clockdivider_t divider, uint16_t cmpa,
                                          uint16_t cmpb, CTimer_Polarity_t pol);
#endif /* CTIMER1 */


/** CTimer0_Counter return the current Timer0 counter value
 * @return Timer counter value
 */
STATIC INLINE uint16_t CTimer0_Counter(void);


#ifdef IO_CTIMER1__START_GET
/** CTimer1_Counter return the current Timer1 counter value
 * @return Timer counter value
 */
STATIC INLINE uint16_t CTimer1_Counter(void);
#endif /* CTIMER1 */


/** CTimer0_Start enables the Timer0 */
STATIC INLINE void CTimer0_Start(void);


#ifdef IO_CTIMER1__START_GET
/** CTimer1_Start enables the Timer1 */
STATIC INLINE void CTimer1_Start(void);
#endif /* CTIMER1 */


/** CTimer0_Stop disables the Timer0 */
STATIC INLINE void CTimer0_Stop(void);


#ifdef IO_CTIMER1__START_GET
/** CTimer1_Stop disables the Timer1 */
STATIC INLINE void CTimer1_Stop(void);
#endif /* CTIMER1 */


/** CTimer0_PwmState returns the current PWM state
 * @return The current Timer0 PWM's state
 */
STATIC INLINE uint8_t CTimer0_PwmState(void);


#ifdef IO_CTIMER1__START_GET
/** CTimer1_PwmState returns the current PWM state
 * @return The current Timer1 PWM's state
 */
STATIC INLINE uint8_t CTimer1_PwmState(void);
#endif /* CTIMER1 */

#ifdef IO_PORT_TIMER_CFG__TIMER0_CHA_SEL_GET
/* Channels function selection */
/** CTimer0_ChanASel sets the CTimer0 Channel A Capture Input source
 * @param[in] sel sets the Input source, defined in project's settings
 */
STATIC INLINE void CTimer0_ChanASel(CTimerCHinputSelect_t sel);
#endif /* CTimer0 MUX */

#ifdef IO_PORT_TIMER_CFG__TIMER0_CHB_SEL_GET
/** CTimer0_ChanBSel sets the CTimer0 Channel B Capture Input source
 * @param[in] sel sets the Input source, defined in project's settings
 */
STATIC INLINE void CTimer0_ChanBSel(CTimerCHinputSelect_t sel);
#endif /* CTimer0 MUX */

#if defined(IO_PORT_TIMER_CFG__TIMER1_CHA_SEL_GET) || \
    defined(IO_PORT_TIMER_CFG1__TIMER1_CHA_SEL_GET)
/** CTimer1_ChanASel sets the CTimer1 Channel A Capture Input source
 * @param[in] sel sets the Input source, defined in project's settings
 */
STATIC INLINE void CTimer1_ChanASel(CTimerCHinputSelect_t sel);
#endif /* CTimer1 MUX */

#if defined(IO_PORT_TIMER_CFG__TIMER1_CHB_SEL_GET) || \
    defined(IO_PORT_TIMER_CFG1__TIMER1_CHB_SEL_GET)
/** CTimer1_ChanBSel sets the CTimer1 Channel B Capture Input source
 * @param[in] sel sets the Input source, defined in project's settings
 */
STATIC INLINE void CTimer1_ChanBSel(CTimerCHinputSelect_t sel);
#endif /* CTimer1 MUX */

/**
 *  Interrupts handling
 *  ===================
 *
 *  CTIMER0_1, CTIMER0_2, CTIMER0_3,
 */
#ifdef IO_CTIMER1__START_GET
/**
 *  CTIMER1_1, CTIMER1_2, CTIMER1_3
 */
#endif
/**
 *
 *  Enable, Disable, Clear
 *
 *
 * Depending on the Timer's mode, the interrupts will be triggered by the following events:
 *
 *  Mode:        |  Int1    |      Int2       |  Int3
 * --------------|----------|-----------------|---------
 *  Timer        |   -      |       -         |  TIMER
 *  Dual compare |  CMPA    |       -         |  CMPB
 *  Dual capture |  CAPA    | OVF, OVRA, OVRB |  CAPB
 *  Cap/Comp     |  CMPA    | OVF, OVRB       |  CAPB
 *  PWM          | INT_PWMA |       -         |  PWMB
 *
 */


/**
 * CTimer0 Interrupts
 * ==================
 */

/** CTimer0_Int1_Enable enables the Timer0Int1 interrupt */
STATIC INLINE void CTimer0_Int1_Enable(void);

/** CTimer0_Int2_Enable enables the Timer0Int2 interrupt */
STATIC INLINE void CTimer0_Int2_Enable(void);

/** CTimer0_Int3_Enable enables the Timer0Int3 interrupt */
STATIC INLINE void CTimer0_Int3_Enable(void);

/** CTimer0_Int1_Disable disables the Timer0Int1 interrupt */
STATIC INLINE void CTimer0_Int1_Disable(void);

/** CTimer0_Int2_Disable disables the Timer0Int2 interrupt */
STATIC INLINE void CTimer0_Int2_Disable(void);

/** CTimer0_Int3_Disable disables the Timer0Int3 interrupt */
STATIC INLINE void CTimer0_Int3_Disable(void);

/** CTimer0_Int1_Clear cleans the Timer0Int1 interrupt pending flag */
STATIC INLINE void CTimer0_Int1_Clear(void);

/** CTimer0_Int2_Clear cleans the Timer0Int2 interrupt pending flag */
STATIC INLINE void CTimer0_Int2_Clear(void);

/** CTimer0_Int3_Clear cleans the Timer0Int3 interrupt pending flag */
STATIC INLINE void CTimer0_Int3_Clear(void);



#ifdef IO_CTIMER1__START_GET
/**
 * CTimer1 Interrupts
 * ==================
 */

/** CTimer1_Int1_Enable enables the Timer1Int1 interrupt */
STATIC INLINE void CTimer1_Int1_Enable(void);

/** CTimer1_Int2_Enable enables the Timer1Int2 interrupt */
STATIC INLINE void CTimer1_Int2_Enable(void);

/** CTimer1_Int3_Enable enables the Timer1Int3 interrupt */
STATIC INLINE void CTimer1_Int3_Enable(void);

/** CTimer1_Int1_Disable disables the Timer1Int1 interrupt */
STATIC INLINE void CTimer1_Int1_Disable(void);

/** CTimer1_Int2_Disable disables the Timer1Int2 interrupt */
STATIC INLINE void CTimer1_Int2_Disable(void);

/** CTimer1_Int3_Disable disables the Timer1Int3 interrupt */
STATIC INLINE void CTimer1_Int3_Disable(void);

/** CTimer1_Int1_Clear cleans the Timer1Int1 interrupt pending flag */
STATIC INLINE void CTimer1_Int1_Clear(void);

/** CTimer1_Int2_Clear cleans the Timer1Int2 interrupt pending flag */
STATIC INLINE void CTimer1_Int2_Clear(void);

/** CTimer1_Int3_Clear cleans the Timer1Int3 interrupt pending flag */
STATIC INLINE void CTimer1_Int3_Clear(void);

#endif /* CTIMER1 */

#if !defined(UNITTEST) /* for unit test, mocks will be generated */
#include "ctimerlib_inline_impl.h"
#endif

/// @}
#endif /* CTIMERLIB_H */
