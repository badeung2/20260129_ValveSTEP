/*!************************************************************************** *
 * \file        HAL_ADC_inline.h
 * \brief       Hardware Abstraction Layer for ADC handling (inline)
 *
 * \note        project MLX81160/33x/34x/35x
 *
 * \author      Marcel Braat
 *
 * \date        2024-03-24
 *
 * \version     2.0
 *
 * A list of functions:
 *  - Inline Functions:
 *           -# HAL_ADC_PreCheck()
 *           -# HAL_ADC_Start()
 *           -# HAL_ADC_Stop()
 *           -# HAL_ADC_SoftTrigger()
 *           -# HAL_ADC_ClearErrors()
 *           -# HAL_ADC_DisableIRQ()
 *           -# HAL_ADC_EnableIRQ()
 *           -# HAL_ADC_SetPrio()
 *           -# HAL_ADC_SetPrioAndEnableIRQ()
 *           -# HAL_ADC_Setup()
 *           -# HAL_ADC_PendClear()
 *           -# HAL_ADC_PendWait()
 *
 *
 * MELEXIS Microelectronic Integrated Systems
 *
 * Copyright (C) 2024-2024 Melexis N.V.
 * The Software is being delivered 'AS IS' and Melexis, whether explicitly or
 * implicitly, makes no warranty as to its Use or performance.
 * The user accepts the Melexis Firmware License Agreement.
 *
 * Melexis confidential & proprietary
 *
 * ************************************************************************** */

#ifndef HAL_LIB_ADC_INLINE_H
#define HAL_LIB_ADC_INLINE_H

/*!************************************************************************** */
/*                          INCLUDES                                          */
/* ************************************************************************** */
#include "../AppBuild.h"                                                        /* Application Build */

#include <atomic.h>
#if (_SUPPORT_DIAG_VDDA_UV_SLEEP != FALSE)
#if (LIN_COMM != FALSE)
#include <mls_api.h>                                                            /* Melexis LIN module */
#endif /* (LIN_COMM != FALSE) */
#include <bl_tools.h>                                                           /* Used by MLX16_RESET_SIGNED */
#include <bl_tools_inline_impl.h>
#endif /* (_SUPPORT_DIAG_VDDA_UV_SLEEP != FALSE) */

/*!************************************************************************** */
/*                          DEFINITIONS                                       */
/* ************************************************************************** */
/*! ADC IO-Port Definitions */
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
#define B_MLX16_ITC_PEND2_ADC               B_MLX16_ITC_PEND2_ADC_SAR
#define B_MLX16_ITC_MASK2_ADC               B_MLX16_ITC_MASK2_ADC_SAR

#define M_MLX16_ITC_PRIO2_ADC               M_MLX16_ITC_PRIO2_ADC_SAR
#define C_MLX16_ITC_PRIO2_ADC_PRIO4         C_MLX16_ITC_PRIO2_ADC_SAR_PRIO4
#define C_MLX16_ITC_PRIO2_ADC_PRIO6         C_MLX16_ITC_PRIO2_ADC_SAR_PRIO6

#define IO_ADC_CTRL                         IO_ADC_SAR_CTRL
#define B_ADC_ADC_WIDTH                     B_ADC_SAR_ADC_WIDTH
#define C_ADC_ASB_NEVER                     C_ADC_SAR_ASB_NEVER                 /* Auto Standby: Never */
#define C_ADC_INT_SCHEME_EOC                C_ADC_SAR_INT_SCHEME_EOC            /* Message interrupt: End-of-Conversion */
#define C_ADC_INT_SCHEME_EOF                C_ADC_SAR_INT_SCHEME_EOF            /* Interrupt at each end of frame */
#define C_ADC_INT_SCHEME_EOS                C_ADC_SAR_INT_SCHEME_EOS            /* Interrupt at end of sequence */
#define C_ADC_INT_SCHEME_NOINT              C_ADC_SAR_INT_SCHEME_NOINT          /* Message interrupt: No */
#define B_ADC_SATURATE                      B_ADC_SAR_SATURATE                  /* Saturation: Enabled */
#define B_ADC_NO_INTERLEAVE                 B_ADC_SAR_NO_INTERLEAVE             /* Interleave: No */
#define C_ADC_SOC_SOURCE_HARD_CTRIG         C_ADC_SAR_SOC_SOURCE_HARD_CTRIG     /* Start Of Conversion (SOC) triggered by: Hardware Trigger */
#define C_ADC_SOS_SOURCE_HARD_CTRIG         C_ADC_SAR_SOS_SOURCE_HARD_CTRIG     /* Start Of Conversion (SOS) triggered by: Hardware Trigger */
#define C_ADC_SOS_SOURCE_2ND_HARD_CTRIG     C_ADC_SAR_SOS_SOURCE_2ND_HARD_CTRIG /* Start Of Conversion (SOS) triggered by: 2nd Hardware Trigger */
#define C_ADC_SOS_SOURCE_SOFT_TRIG          C_ADC_SAR_SOS_SOURCE_SOFT_TRIG      /* Start Of Sequence (SOS) triggered: Software Trigger */
#define B_ADC_STOP                          B_ADC_SAR_STOP
#define B_ADC_START                         B_ADC_SAR_START

#define IO_ADC_STATUS                       IO_ADC_SAR_STATUS
#define B_ADC_ABORTED                       B_ADC_SAR_ABORTED
#define B_ADC_FRAME_ERR                     B_ADC_SAR_FRAME_ERR                 /* Clear FRAME_ERROR flag */
#define B_ADC_MEM_ERR                       B_ADC_SAR_MEM_ERR                   /* Clear MEMORY ERROR flag */
#define B_ADC_ADC_ERR                       B_ADC_SAR_ADC_ERR                   /* Clear ADC ERROR flag */
#define B_ADC_ADC_OVF                       B_ADC_SAR_ADC_OVF                   /* Clear ADC Overflow flag */
#define B_ADC_PAUSE                         B_ADC_SAR_PAUSE                     /* Disable all triggers */
#define B_ADC_RESUME                        B_ADC_SAR_RESUME                    /* Enable all Triggers */
#define M_ADC_STATE                         M_ADC_SAR_STATE
#define C_ADC_STATE_IDLE                    C_ADC_SAR_STATE_IDLE
#define C_ADC_STATE_WAIT_FOR_TRIGGER        C_ADC_SAR_STATE_WAIT_FOR_TRIGGER
#define C_ADC_STATE_MEM_TRANSFER            C_ADC_SAR_STATE_MEM_TRANSFER
#define B_ADC_SW_TRIG                       B_ADC_SAR_SW_TRIG
#define IO_ADC_SBASE                        IO_ADC_SAR_SBASE
#elif defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#define B_MLX16_ITC_PEND2_ADC               B_MLX16_ITC_PEND2_ADC_BLOCK
#define B_MLX16_ITC_MASK2_ADC               B_MLX16_ITC_MASK2_ADC_BLOCK

#define M_MLX16_ITC_PRIO2_ADC               M_MLX16_ITC_PRIO2_ADC_BLOCK
#define C_MLX16_ITC_PRIO2_ADC_PRIO4         C_MLX16_ITC_PRIO2_ADC_BLOCK_PRIO4
#define C_MLX16_ITC_PRIO2_ADC_PRIO6         C_MLX16_ITC_PRIO2_ADC_BLOCK_PRIO6

#define IO_ADC_CTRL                         IO_ADC_BLOCK_CTRL
#define B_ADC_ADC_WIDTH                     B_ADC_BLOCK_ADC_WIDTH
#define C_ADC_ASB_NEVER                     C_ADC_BLOCK_ASB_NEVER
#define C_ADC_INT_SCHEME_EOC                C_ADC_BLOCK_INT_SCHEME_EOC
#define C_ADC_INT_SCHEME_EOF                C_ADC_BLOCK_INT_SCHEME_EOF          /* Interrupt at each end of frame */
#define C_ADC_INT_SCHEME_EOS                C_ADC_BLOCK_INT_SCHEME_EOS          /* Interrupt at end of sequence */
#define C_ADC_INT_SCHEME_NOINT              C_ADC_BLOCK_INT_SCHEME_NOINT
#define B_ADC_SATURATE                      B_ADC_BLOCK_SATURATE                /* Saturation: Enabled */
#define B_ADC_NO_INTERLEAVE                 B_ADC_BLOCK_NO_INTERLEAVE           /* Interleave: No */
#define C_ADC_SOC_SOURCE_HARD_CTRIG         C_ADC_BLOCK_SOC_SOURCE_HARD_CTRIG   /* Start Of Conversion (SOC) triggered by: Hardware */
#define C_ADC_SOS_SOURCE_HARD_CTRIG         C_ADC_BLOCK_SOS_SOURCE_HARD_CTRIG   /* Start Of Conversion (SOS) triggered by: Hardware Trigger */
#define C_ADC_SOS_SOURCE_2ND_HARD_CTRIG     C_ADC_BLOCK_SOS_SOURCE_2ND_HARD_CTRIG /* Start Of Conversion (SOS) triggered by: 2nd Hardware Trigger */
#define C_ADC_SOS_SOURCE_SOFT_TRIG          C_ADC_BLOCK_SOS_SOURCE_SOFT_TRIG    /* Start Of Sequence (SOS) triggered: Software */
#define B_ADC_STOP                          B_ADC_BLOCK_STOP
#define B_ADC_START                         B_ADC_BLOCK_START

#define IO_ADC_STATUS                       IO_ADC_BLOCK_STATUS
#define B_ADC_ABORTED                       B_ADC_BLOCK_ABORTED
#define B_ADC_FRAME_ERR                     B_ADC_BLOCK_FRAME_ERR               /* Clear FRAME_ERROR flag */
#define B_ADC_MEM_ERR                       B_ADC_BLOCK_MEM_ERR                 /* Clear MEMORY ERROR flag */
#define B_ADC_ADC_ERR                       B_ADC_BLOCK_ADC_ERR                 /* Clear ADC ERROR flag */
#define B_ADC_ADC_OVF                       B_ADC_BLOCK_ADC_OVF                 /* Clear ADC Overflow flag */
#define B_ADC_PAUSE                         B_ADC_BLOCK_PAUSE                   /* Disable all triggers */
#define B_ADC_RESUME                        B_ADC_BLOCK_RESUME                  /* Enable all Triggers */
#define M_ADC_STATE                         M_ADC_BLOCK_STATE
#define C_ADC_STATE_IDLE                    C_ADC_BLOCK_STATE_IDLE
#define C_ADC_STATE_WAIT_FOR_TRIGGER        C_ADC_BLOCK_STATE_WAIT_FOR_TRIGGER
#define C_ADC_STATE_MEM_TRANSFER            C_ADC_BLOCK_STATE_MEM_TRANSFER
#define B_ADC_SW_TRIG                       B_ADC_BLOCK_SW_TRIG

#define IO_ADC_SBASE                        IO_ADC_BLOCK_SBASE
#endif

#define C_ADC_SUPPLY_CHECK                  20U                                 /*!< Times to check Supply */

/*!*************************************************************************** *
 * HAL_ADC_PreCheck
 * \brief   Pre-check ADC
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Pre-check ADC before usage. ADC rely on VDDD and VDDA supply.
 *          - VDDA must be valid.
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 2
 * - Function calling: 0
 * *************************************************************************** */
static __inline__ void HAL_ADC_PreCheck(void)
{
    uint16_t u16LoopCnt = C_ADC_SUPPLY_CHECK;
    do
    {
        u16LoopCnt--;
        if ( (IO_MLX16_ITC_PEND0_S & B_MLX16_ITC_PEND0_UV_VDDA) != 0U)
        {
#if (_SUPPORT_APP_USER_MODE != FALSE)
            ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
            IO_MLX16_ITC_PEND0_S = B_MLX16_ITC_PEND0_UV_VDDA;
#if (_SUPPORT_APP_USER_MODE != FALSE)
            EXIT_SECTION(); /*lint !e438 !e529 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#if (_SUPPORT_DIAG_VDDA_UV_SLEEP != FALSE)
            if (u16LoopCnt == 0U)
            {
                ENTER_SECTION(ATOMIC_SYSTEM_MODE); /*lint !e534 */
#if (LIN_COMM != FALSE)
                ml_ResetDrv();
#endif /* (LIN_COMM != FALSE) */
                IO_PORT_STOPMD_CTRL_S &= ~B_PORT_STOPMD_CTRL_SEL_STOP_MODE;     /* Deep-sleep (8.5uA) */
                __asm__ ("HALT\n\t" :::);
                MLX16_RESET_COLD();                                             /* Should not come here; In case: Reset */
                EXIT_SECTION(); /*lint !e438 */
            }
#endif /* (_SUPPORT_DIAG_VDDA_UV_SLEEP != FALSE) */
            continue;
        }
    } while(u16LoopCnt != 0U);
} /* End of HAL_ADC_PreCheck() */

/*!*************************************************************************** *
 * HAL_ADC_Start
 * \brief   Start ADC
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Start the ADC
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static __inline__ void HAL_ADC_Start(void)
{
    IO_ADC_CTRL = B_ADC_START;                                                  /* START the ADC */
} /* End of HAL_ADC_Start() */

/*!*************************************************************************** *
 * HAL_ADC_Stop
 * \brief   Stop ADC
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Stop the ADC
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static __inline__ void HAL_ADC_Stop(void)
{
    IO_ADC_CTRL = B_ADC_STOP;                                                   /* STOP the ADC from any ADC mode. */
} /* End of HAL_ADC_Stop() */

/*!*************************************************************************** *
 * HAL_ADC_SoftTrigger
 * \brief   Trigger the ADC by means of a SW-trigger
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Send a SW-trigger to the ADC
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static __inline__ void HAL_ADC_SoftTrigger(void)
{
    IO_ADC_STATUS = B_ADC_SW_TRIG;                                              /* Send SW-trigger to ADC */
} /* End of HAL_ADC_SoftTrigger() */

/*!*************************************************************************** *
 * HAL_ADC_ClearErrors
 * \brief   Clear any ADC error flag(s)
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Clear all error flags of the ADC
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static __inline__ void HAL_ADC_ClearErrors(void)
{
    /* Prior to start, first clear any error flag and enable Triggers */
    IO_ADC_STATUS = B_ADC_PAUSE;                                                /* Disable all Triggers */
    IO_ADC_STATUS = (B_ADC_ABORTED |                                            /* Clear ABORTED flag */
                     B_ADC_FRAME_ERR |                                          /* Clear FRAME_ERROR flag */
                     B_ADC_MEM_ERR |                                            /* Clear MEMORY ERROR flag */
                     B_ADC_ADC_ERR |                                            /* Clear ADC ERROR flag */
                     B_ADC_ADC_OVF);                                            /* Clear ADC Overflow flag */
    IO_ADC_STATUS = B_ADC_RESUME;                                               /* Enable all Triggers */
} /* End of HAL_ADC_ClearErrors() */

/*!*************************************************************************** *
 * HAL_ADC_DisableIRQ
 * \brief   Disable ADC IRQ
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Disable the ADC IRQ
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static __inline__ void HAL_ADC_DisableIRQ(void)
{
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(ATOMIC_SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    IO_MLX16_ITC_MASK2_S &= ~B_MLX16_ITC_MASK2_ADC;                             /* Disable ADC Interrupt */
    IO_MLX16_ITC_PEND2_S = B_MLX16_ITC_PEND2_ADC;
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 !e529 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
} /* End of HAL_ADC_DisableIRQ() */

/*!*************************************************************************** *
 * HAL_ADC_EnableIRQ
 * \brief   Enable ADC IRQ
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Enable the ADC IRQ
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static __inline__ void HAL_ADC_EnableIRQ(void)
{
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(ATOMIC_SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    IO_MLX16_ITC_PEND2_S = B_MLX16_ITC_PEND2_ADC;
    IO_MLX16_ITC_MASK2_S |= B_MLX16_ITC_MASK2_ADC;                              /* Enable ADC Interrupt */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 !e529 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
} /* End of HAL_ADC_EnableIRQ() */

/*!*************************************************************************** *
 * HAL_ADC_SetPrio
 * \brief   Set ADC IRQ Priority
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Set ADC IRQ Priority
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static __inline__ void HAL_ADC_SetPrio(uint16_t u16Priority)
{
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    IO_MLX16_ITC_PRIO2_S = (IO_MLX16_ITC_PRIO2_S & ~M_MLX16_ITC_PRIO2_ADC) | u16Priority; /* ADC IRQ Priority: 3..6 */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 !e529 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
} /* End of HAL_ADC_SetPrio() */

/*!*************************************************************************** *
 * HAL_ADC_SetPrioAndEnableIRQ
 * \brief   Set ADC IRQ Priority and enable ADC IRQ
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Set ADC IRQ Priority and enable the ADC IRQ
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static __inline__ void HAL_ADC_SetPrioAndEnableIRQ(uint16_t u16Priority)
{
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(ATOMIC_SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    IO_MLX16_ITC_PRIO2_S = (IO_MLX16_ITC_PRIO2_S & ~M_MLX16_ITC_PRIO2_ADC) | u16Priority;
    IO_MLX16_ITC_PEND2_S = B_MLX16_ITC_PEND2_ADC;
    IO_MLX16_ITC_MASK2_S |= B_MLX16_ITC_MASK2_ADC;                              /* Enable ADC Interrupt */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 !e529 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
} /* End of HAL_ADC_SetPrioAndEnableIRQ() */

/*!*************************************************************************** *
 * HAL_ADC_Setup
 * \brief   Setup ADC
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Setup ADC by selection the ADC Auto-Sequencer table and configure the
 *          ADC Control register.
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static __inline__ void HAL_ADC_Setup(uint16_t u16Ctrl)
{
#pragma space nodp                                                              /* __NEAR_SECTION__ */
    extern uint16_t l_au16AdcSource[];                                          /*!< Generic ADC HW-Trigger buffer */
#pragma space none                                                              /* __NEAR_SECTION__ */

    IO_ADC_SBASE = (uint16_t)&l_au16AdcSource[0];                               /* Switch ADC input source to LIN Shunt */
    IO_ADC_CTRL = u16Ctrl;
} /* End of HAL_ADC_Setup() */

/*!*************************************************************************** *
 * HAL_ADC_PendClear
 * \brief   Clear ADC PEND-flag
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Clear ADC PEND-flag (polling)
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static __inline__ void HAL_ADC_PendClear(void)
{
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    IO_MLX16_ITC_PEND2_S = B_MLX16_ITC_PEND2_ADC;
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 !e529 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
} /* End of HAL_ADC_PendClear() */

/*!*************************************************************************** *
 * HAL_ADC_PendWait
 * \brief   Wait ADC PEND-flag
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Wait ADC PEND-flag (polling)
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static __inline__ void HAL_ADC_PendWait(void)
{
    while ( (IO_MLX16_ITC_PEND2_S & B_MLX16_ITC_PEND2_ADC) == 0U) {};
} /* End of HAL_ADC_PendWait() */

#endif /* HAL_LIB_ADC_INLINE_H */

/* EOF */
