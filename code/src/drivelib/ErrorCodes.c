/*!*************************************************************************** *
 * \file        ErrorCodes.c
 * \brief       MLX813xx Error handling
 *
 * \note        project MLX813xx
 *
 * \author      Marcel Braat
 *
 * \date        2017-08-15
 *
 * \version     2.0
 *
 * A list of functions:
 *  - Global Functions:
 *           -# ErrorLogInit()
 *           -# SetLastError()
 *           -# GetFirstError()
 *           -# PeakFirstError()
 *           -# GetNV_LogError()
 *           -# ClearNV_LogError()
 *
 *
 * \copyright   MELEXIS Microelectronic Integrated Systems
 *
 * Copyright (C) 2017-2022 Melexis N.V.
 * The Software is being delivered 'AS IS' and Melexis, whether explicitly or
 * implicitly, makes no warranty as to its Use or performance.
 * The user accepts the Melexis Firmware License Agreement.
 *
 * Melexis confidential & proprietary
 *
 *                          RAM     Flash
 * _SUPPORT_LOG_ERRORS:     32 B    1132 B
 * *************************************************************************** */

/*!*************************************************************************** */
/*                           INCLUDES                                          */
/* *************************************************************************** */
#include "AppBuild.h"                                                           /* Application Build */

#if (_SUPPORT_LOG_ERRORS != FALSE)

#include "drivelib/AppFunctions.h"                                              /* Application Functions support */
#include "drivelib/ErrorCodes.h"                                                /* Error logging support */
#if (_SUPPORT_NV_LOG_ERROR != FALSE)
#include "drivelib/NV_Functions.h"                                              /* Non Volatile Memory Functions & Layout */
#include "camculib/private_mathlib.h"                                           /* Private Math Library support */
#endif /* (_SUPPORT_NV_LOG_ERROR != FALSE) */

#include <atomic.h>
#include <bl_bist.h>

/*!*************************************************************************** */
/*                           GLOBAL & LOCAL VARIABLES                          */
/* *************************************************************************** */
#pragma space nodp                                                              /* __NEAR_SECTION__ */
#define C_ERR_LOG_SZ    16U                                                     /*!< Error-log storage size */
uint16_t l_au16FiFoErrorLog[C_ERR_LOG_SZ];                                      /*!< Error-log storage (FiFo) */
uint8_t l_u8ErrorLogIdx = 0U;                                                   /*!< Error-log write-index */
#if (_SUPPORT_NV_LOG_ERROR != FALSE)
APP_LOG_t ErrorLogEE __attribute__((aligned(2)));                               /*!< Non Volatile Memory Error Log */
#endif /* (_SUPPORT_NV_LOG_ERROR != FALSE) */
#pragma space none                                                              /* __NEAR_SECTION__ */

/*!*************************************************************************** *
 * ErrorLogInit
 * \brief   Initialise the Error-logging
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Clear Error-FiFo-buffer, in case watchdog reset occurred,
 *            otherwise leave untouched.
 * *************************************************************************** *
 * - Call Hierarchy: main_Init()
 * - Cyclomatic Complexity: 6+1
 * - Nesting: 4
 * - Function calling: 1 (SetLastError())
 * *************************************************************************** */
void ErrorLogInit(void)
{
    if ( ((IO_RST_CTRL_S & (B_RST_CTRL_SOFT_WBOOT | B_RST_CTRL_AWD_WBOOT)) != 0U) &&
         ((bistResetInfo != C_CHIP_STATE_FATAL_CRASH_RECOVERY) &&
          (bistResetInfo != C_CHIP_STATE_FATAL_RECOVER_ENA)) )
    {
        uint16_t i;
        for (i = 0U; i < C_ERR_LOG_SZ; i++)
        {
            l_au16FiFoErrorLog[i] = C_ERR_NONE;
        }
        l_u8ErrorLogIdx = 0U;
    }

#if (_SUPPORT_NV_LOG_ERROR != FALSE)
    /* Copy Non Volatile Memory Error-log data to RAM */
    p_CopyU16( (sizeof(APP_LOG_t) / sizeof(uint16_t)),
               (uint16_t *)(void *)&ErrorLogEE,
               (const uint16_t *)ADDR_NV_ERRORLOG);
#endif /* (_SUPPORT_NV_LOG_ERROR != FALSE) */

    /* Log Watch-dog reset */
    if ( (bistHeader == (BistHeader_t)C_CHIP_HEADER) &&
         (bistResetInfo != (uint16_t)C_CHIP_STATE_CMD_RESET) )                  /* LIN-command chip reset use WD; No need to log */
    {
        uint16_t u16RstCtrl = IO_RST_CTRL_S & (B_RST_CTRL_IWD_WBOOT | B_RST_CTRL_SOFT_WBOOT | B_RST_CTRL_AWD_WBOOT);
        if (u16RstCtrl != 0U)
        {
            /* Warm boot reset */
            SetLastError(C_ERR_WD_RST | C_ERR_EXT | (u16RstCtrl << 8));         /* (Digital) Watchdog reset */
        }
#if (_SUPPORT_RESET_BY_IO != FALSE)
        if ( (IO_RST_CTRL_S & B_RST_CTRL_IO5_WBOOT) != 0U)
        {
            SetLastError(C_ERR_IO_RST);                                         /* I/O Watchdog reset */
        }
#endif /* (_SUPPORT_RESET_BY_IO != FALSE) */
    }
    /* Log wake-up reason (MMP230726-1) */
    {
#if !defined (__MLX81339__)
        uint16_t u16Wakeup = (IO_PORT_MISC_IN & (B_PORT_MISC_IN_LOCAL_WU | B_PORT_MISC_IN_LIN_WU | B_PORT_MISC_IN_INTERNAL_WU)) >> 2;
#else  /* !defined (__MLX81339__) */
        uint16_t u16Wakeup = (IO_PORT_MISC_IN & (B_PORT_MISC_IN_LOCAL_WU | B_PORT_MISC_IN_INTERNAL_WU)) >> 2;
#endif /* !defined (__MLX81339__) */
        if (u16Wakeup != 0U)
        {
            SetLastError(C_INF_WU | C_ERR_EXT | u16Wakeup);                     /* (LIN) Wake-up */
        }
    }

#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    IO_RST_CTRL_S = (B_RST_CTRL_IWD_WBOOT |                                     /* Clear IWD_WBOOT */
                     B_RST_CTRL_DBG_WBOOT |                                     /* Clear DBG_WBOOT */
                     B_RST_CTRL_HVDIG_WBOOT |                                   /* Clear HVDIG_WBOOT */
                     B_RST_CTRL_SOFT_WBOOT |                                    /* Clear SOFT_WBOOT */
                     B_RST_CTRL_AWD_WBOOT);                                     /* Clear AWD_WBOOT */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 !e529 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */

} /* End of ErrorLogInit() */

/*!*************************************************************************** *
 * SetLastError
 * \brief   Store the 'last' error (in RAM and optionally in NVRAM)
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16ErrorCode: Error-code to be logged
 * \return  -
 * *************************************************************************** *
 * \details Save error-code in Error-FiFo-buffer, unless last error is the
 *          same as error posted.
 * *************************************************************************** *
 * - Call Hierarchy: ?
 * - Cyclomatic Complexity: 4+1
 * - Nesting: 2
 * - Function calling: 1 (NV_WriteErrorLog())
 * *************************************************************************** */
void SetLastError(uint16_t u16ErrorCode)
{
#if (_SUPPORT_LOG_ALL_ERRORS == FALSE)
    if ( (l_u8ErrorLogIdx == 0U) || (l_au16FiFoErrorLog[l_u8ErrorLogIdx - 1U] != u16ErrorCode) )
#endif /* (_SUPPORT_LOG_ALL_ERRORS == FALSE) */
    {
        /* Don't log the same error as previously */
        l_au16FiFoErrorLog[l_u8ErrorLogIdx] = u16ErrorCode;
        if (l_u8ErrorLogIdx < (C_ERR_LOG_SZ - 1U) )
        {
            l_u8ErrorLogIdx++;
        }
    }

#if (_SUPPORT_NV_LOG_ERROR != FALSE) /* TODO: This should be done as background task; Not in case of e.g. a diagnostics event */
    /* Update Non Volatile Memory RAM data and write it to Non Volatile Memory */
    if ( (u16ErrorCode & 0xF0U) != 0x40U)                                       /* Don't save warning's */
    {
        uint8_t u8Idx = ErrorLogEE.u8LogIdx;

        if (u8Idx >= (sizeof(ErrorLogEE.au8LogData) / sizeof(ErrorLogEE.au8LogData[0])) )
        {
            u8Idx = 0U;
        }
        ErrorLogEE.au8LogData[u8Idx] = (u16ErrorCode & 0xFFU);
        ErrorLogEE.u8LogIdx = (u8Idx + 1U);
        (void)NV_WriteErrorLog(&ErrorLogEE);
    }
#endif /* (_SUPPORT_NV_LOG_ERROR != FALSE) */
} /* End of SetLastError() */

/*!*************************************************************************** *
 * GetFirstError
 * \brief   Get First error-code
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t First Error-code (16-bits)
 * *************************************************************************** *
 * \details Get First error code and shift error-buffer, and shift error out of
 *          buffer
 * *************************************************************************** *
 * - Call Hierarchy: DfrDiagMlxDebug()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 2
 * - Function calling: 0
 * *************************************************************************** */
uint16_t GetFirstError(void)
{
    uint16_t u16OldestErrorCode = l_au16FiFoErrorLog[0];
    if (l_u8ErrorLogIdx != 0U)
    {
        uint16_t *pu16Dest = &l_au16FiFoErrorLog[0];
        uint16_t *pu16Src = &l_au16FiFoErrorLog[1];
        ENTER_SECTION(ATOMIC_KEEP_MODE); /*lint !e534 */
        {
            uint8_t i = l_u8ErrorLogIdx;
            while (--i != 0U)
            {
                *pu16Dest++ = *pu16Src++;
            }
            *pu16Dest = C_ERR_NONE;
            l_u8ErrorLogIdx--;
        }
        EXIT_SECTION(); /*lint !e438 !e529 */
    }
    return (u16OldestErrorCode);
} /* End of GetFirstError() */

/*!*************************************************************************** *
 * PeakFirstError
 * \brief   Peak First error-code
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t First Error-code (16-bits)
 * *************************************************************************** *
 * \details Get First error code, without buffer update.
 * *************************************************************************** *
 * - Call Hierarchy: DfrDiagMlxDebug()
 * - Cyclomatic Complexity: 02+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
uint16_t PeakFirstError(void)
{
    return (l_au16FiFoErrorLog[0]);
} /* End of PeakFirstError() */

#if (_SUPPORT_NV_LOG_ERROR != FALSE)
/*!*************************************************************************** *
 * GetNV_LogError
 * \brief   Get indexed Non Volatile Memory Error-code
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u8Idx: Index of error stored in Non Volatile Memory
 * \return  uint8_t Error-code
 * *************************************************************************** *
 * \details Return error-code from Non Volatile Memory without Non Volatile Memory update
 * *************************************************************************** *
 * - Call Hierarchy: DfrDiagMlxDebug()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
uint8_t GetNV_LogError(uint8_t u8Idx)
{
    uint8_t u8ErrorCode = 0U;

    if (u8Idx < (sizeof(ErrorLogEE.au8LogData) / sizeof(ErrorLogEE.au8LogData[0])) )
    {
        u8ErrorCode = ErrorLogEE.au8LogData[u8Idx];
    }
    return (u8ErrorCode);
} /* End of GetNV_LogError() */

/*!*************************************************************************** *
 * ClearNV_LogError
 * \brief   Clear Non Volatile Memory Error-log
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Clear Non Volatile Memory Error-log
 * *************************************************************************** *
 * - Call Hierarchy: DfrDiagMlxDebug()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 2 (p_MemSet(), NV_WriteErrorLog())
 * *************************************************************************** */
void ClearNV_LogError(void)
{
    ErrorLogEE.u8LogIdx = 0U;
    p_MemSet( (void *)&ErrorLogEE.au8LogData[0], 0x00U,
              (sizeof(ErrorLogEE.au8LogData) / sizeof(ErrorLogEE.au8LogData[0])));
    (void)NV_WriteErrorLog(&ErrorLogEE);
} /* End of GetNV_LogError() */

#endif /* (_SUPPORT_NV_LOG_ERROR != FALSE) */

#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */

/* EOF */
