/*!*************************************************************************** *
 * \file        hal_ADC.c
 * \brief       Hardware Abstraction Layer for ADC handling
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
 *           -# HAL_ADC_StopSafe()
 *           -# HAL_ADC_PowerOff()
 *           -# HAL_ADC_StartSoftTrig()
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

#include <sys_tools.h>

/*!************************************************************************** */
/*                          GLOBAL & LOCAL VARIABLES                          */
/* ************************************************************************** */
#pragma space dp
#pragma space none

#pragma space nodp                                                              /* __NEAR_SECTION__ */
uint16_t l_au16AdcSource[C_ADC_SBASE_LEN];                                      /*!< Generic ADC HW-Trigger buffer */
uint8_t l_u8AdcMode = (uint8_t)C_ADC_MODE_OFF;                                  /*!< ADC Mode */
#pragma space none                                                              /* __NEAR_SECTION__ */

/*!*************************************************************************** *
 * HAL_ADC_StopSafe
 * \brief   Stop ADC (with waiting for pending ADC conversions to be finished)
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Stop ADC and disable the IRQ
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
void HAL_ADC_StopSafe(void)
{
#if (_SUPPORT_WHILE_RETRY_LIMITED != FALSE)
    uint16_t u16Retries = 100U;
    while ( ((IO_ADC_CTRL & B_ADC_STOP) == 0U) && (u16Retries != 0U) )
    {
#if (_SUPPORT_BUGFIX_AMALTHEA_1428 != FALSE)
        while ( (IO_ADC_STATUS & M_ADC_STATE) != C_ADC_STATE_WAIT_FOR_TRIGGER)  /* MMP220713-1 */
        {
            ;
        }
#endif /* (_SUPPORT_BUGFIX_AMALTHEA_1428 != FALSE) */
        HAL_ADC_Stop();                                                         /* STOP the ADC from any ADC mode. */
        u16Retries--;
    }

    while ( ((IO_ADC_STATUS & M_ADC_STATE) != C_ADC_STATE_IDLE) && (u16Retries != 0U) )
    {
        /* Restart ADC ...*/
        while ( ((IO_ADC_CTRL & B_ADC_START) == 0U) && (u16Retries != 0U) )
        {
            HAL_ADC_Start();                                                    /* START the ADC from any ADC mode. */
            u16Retries--;
        }
        NOP();
        /* ... and STOP it again */
        while ( ((IO_ADC_CTRL & B_ADC_STOP) == 0U) && (u16Retries != 0U) )
        {
            HAL_ADC_Stop();                                                     /* STOP the ADC from any ADC mode. */
            u16Retries--;
        }
    }
#if (_SUPPORT_LOG_ERRORS != FALSE)
    if (u16Retries == 0U)
    {
        SetLastError(C_HAL_ERR_TO_ADC_STOP);
    }
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
#else  /* (_SUPPORT_WHILE_RETRY_LIMITED != FALSE) */
#if (_SUPPORT_BUGFIX_AMALTHEA_1428 != FALSE)
    while ( (IO_ADC_STATUS & M_ADC_STATE) != C_ADC_STATE_WAIT_FOR_TRIGGER)      /* MMP220713-1 */
    {
        ;
    }
#endif /* (_SUPPORT_BUGFIX_AMALTHEA_1428 != FALSE) */
    HAL_ADC_Stop();                                                             /* STOP the ADC from any ADC mode. */
    NOP();
    while ( (IO_ADC_STATUS & M_ADC_STATE) != C_ADC_STATE_IDLE)
    {
        /* Restart ADC ...*/
        HAL_ADC_Start();                                                        /* START the ADC from any ADC mode. */
        NOP();
        /* ... and STOP it again */
        HAL_ADC_Stop();                                                         /* STOP the ADC from any ADC mode. */
        NOP();
    }
#endif /* (_SUPPORT_WHILE_RETRY_LIMITED != FALSE) */
    IO_ADC_STATUS = B_ADC_ABORTED;                                              /* Clear Aborted status-flag */
    l_u8AdcMode = C_ADC_MODE_IDLE;
    HAL_ADC_DisableIRQ();                                                       /* Disable ADC Interrupt */
} /* End of HAL_ADC_StopSafe() */

/*!*************************************************************************** *
 * HAL_ADC_PowerOff
 * \brief   Turn off the ADC
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Stop and turn-off ADC
 * *************************************************************************** *
 * - Call Hierarchy: AppProcPowerSave()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 1 (HAL_ADC_StopSafe())
 * *************************************************************************** */
void HAL_ADC_PowerOff(void)
{
    HAL_ADC_StopSafe();
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    IO_PORT_ADC_CTRL_S = 0U;
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 !e529 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#elif defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    IO_ADC_CTRL = 0U;                                                           /* ADC is in stand-by only when not used (START = 0) */
#endif /* defined */
    l_u8AdcMode = (uint8_t)C_ADC_MODE_OFF;
} /* End of HAL_ADC_PowerOff() */

/*!*************************************************************************** *
 * HAL_ADC_PowerOn
 * \brief   Turn on the ADC
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Turn-on ADC
 * *************************************************************************** *
 * - Call Hierarchy: ADC_Init()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
void HAL_ADC_PowerOn(void)
{
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    IO_PORT_ADC_CTRL_S = B_PORT_ADC_CTRL_ADC_EN;
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 !e529 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#endif /* defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__) */
} /* End of HAL_ADC_PowerOn() */

/*!*************************************************************************** *
 * HAL_ADC_StartSoftTrig
 * \brief   Start ADC measurement using Software trigger.
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16NextState: Next expected ADC-state
 * \return  uint16_t u16Result: C_ERR_NONE: SW-TRIG conversion finished
 *                              successful
 *                              C_ERR_ADC: SW-TRIG conversion failed
 * *************************************************************************** *
 * \details ADC ISR priority: 5
 * *************************************************************************** *
 * - Call Hierarchy: ADC_Init(), ADC_MCurrOffCalib(), ADC_MeasureVsupplyAndTemperature()
 * - Cyclomatic Complexity: 4+1
 * - Nesting: 2
 * - Function calling: 0
 * *************************************************************************** */
uint16_t HAL_ADC_StartSoftTrig(uint16_t u16NextState)
{
    uint16_t u16Result = C_HAL_ERR_NONE;

    if ( (IO_ADC_CTRL & B_ADC_START) == 0U)
    {
        /* ADC is not started; */
        IO_ADC_SBASE = (uint16_t)&l_au16AdcSource[0];
        IO_ADC_CTRL = (/*B_ADC_SAR_ADC_WIDTH |*/
            C_ADC_ASB_NEVER |                                                   /* Auto Standby: Never */
#if (_DEBUG_IO_ADC != FALSE)
            C_ADC_INT_SCHEME_EOC |                                              /* Message interrupt: End-of-Conversion */
#else  /* (_DEBUG_IO_ADC != FALSE) */
            C_ADC_INT_SCHEME_NOINT |                                            /* Message interrupt: No */
#endif /* (_DEBUG_IO_ADC != FALSE) */
            B_ADC_SATURATE |                                                    /* Saturation: Enabled */
            B_ADC_NO_INTERLEAVE |                                               /* Interleave: No */
            C_ADC_SOC_SOURCE_HARD_CTRIG |                                       /* Start Of Conversion (SOC) triggered by: Hardware */
            C_ADC_SOS_SOURCE_SOFT_TRIG);                                        /* Start Of Sequence (SOS) triggered: Software */
        HAL_ADC_ClearErrors();                                                  /* Prior to start, first clear any error */
        /* Start it! */
        HAL_ADC_Start();                                                        /* Start ADC */
#if defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
        DELAY_US(C_ADC_SETTLING_TIME);                                          /* Allow some delay for the ADC to load first SBASE channel selection */
        if ( (IO_ADC_STATUS & M_ADC_STATE) == C_ADC_STATE_MEM_TRANSFER)
        {
            HAL_ADC_Stop();                                                     /* Stop ADC (and restart) */
        }
#endif /* defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */
    }
    DELAY_US(C_ADC_SETTLING_TIME);                                              /* Setting time */
    IO_ADC_STATUS = B_ADC_SW_TRIG;                                              /* Send software-trigger now */
#if (_SUPPORT_WHILE_RETRY_LIMITED != FALSE)
    uint16_t u16Retries = 100U;
    while ( (IO_ADC_CTRL & B_ADC_STOP) == 0U)                                   /* MMP181008-2: Last sample correction */
    {
        /* Not stopped */
        if ( (IO_ADC_STATUS & (B_ADC_ABORTED | M_ADC_STATE)) == u16NextState)
        {
            break;
        }
        if ( (IO_ADC_STATUS & B_ADC_ABORTED) != 0U)                             /* ADC Aborted */
        {
            u16Result = C_ERR_ADC;
            break;
        }
        if (--u16Retries == 0U)
        {
            u16Result = C_ERR_ADC;
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_TO_ADC_STRIG);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            break;
        }
    }
#else  /* (_SUPPORT_WHILE_RETRY_LIMITED != FALSE) */
    while ( (IO_ADC_CTRL & B_ADC_STOP) == 0U)                                   /* MMP181008-2: Last sample correction */
    {
        /* Not stopped */
        if ( (IO_ADC_STATUS & (B_ADC_ABORTED | M_ADC_STATE)) == u16NextState)
        {
            break;
        }
        if ( (IO_ADC_STATUS & B_ADC_ABORTED) != 0U)                             /* ADC Aborted */
        {
            u16Result = C_HAL_ERR_ADC;
            break;
        }
    }
#endif /* (_SUPPORT_WHILE_RETRY_LIMITED != FALSE) */
    l_u8AdcMode = (uint8_t)C_ADC_MODE_IDLE;
    return (u16Result);
} /* End of HAL_ADC_StartSoftTrig() */

/* EOF */
