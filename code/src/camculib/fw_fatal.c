/*!*************************************************************************** *
 * \file        fw_fatal.c
 * \brief       Fatal-handler for unused and system interrupts
 *
 * \note        project MLX813xx
 *
 * \author      Marcel Braat
 *
 * \date        2017-10-06
 *
 * \version     2.0
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
 * ************************************************************************** */

/*
 *  NOTES:
 *      1.  All functions in this module should be defined with __ROM_MLX_TEXT__
 *          attribute. Thus they will be linked in first half of the Flash.
 *
 *      2.  This module should NOT use _initialized_ global and static variables!
 *          Such variables are linked into .data or .dp.data sections and their
 *          initialisation lists are stored at load address (LMA) in the Flash.
 *          Since there is no control on the position of the load address, the
 *          linker might link it to second half of the Flash and thus
 *          initialisation values will be overwritten by the loader during
 *          programming of the a new application. As a result, variables in .data
 *          sections will not be correctly initialised.
 *          Use uninitialised variables instead (will be linked to .bss section).
 */

#include "AppBuild.h"                                                           /* Application Build */
#include "sys_tools.h"
#include "fw_startup.h"
#include "bl_bist.h"

#include "drivelib/AppFunctions.h"                                              /* Application Functions support */
#if (_SUPPORT_LOG_FATAL_ERRORS != FALSE)
#include "drivelib/ErrorCodes.h"                                                /* Support error logging */
#endif /* (_SUPPORT_LOG_FATAL_ERRORS != FALSE) */

#include <builtin_mlx16.h>

#if (_SUPPORT_CRASH_RECOVERY != FALSE) || (_SUPPORT_LOG_FATAL_ERRORS != FALSE)
extern uint16_t fw_stack;                                                       /*!< Firmware start-of-stack-area */
#endif /* (_SUPPORT_CRASH_RECOVERY != FALSE) || (_SUPPORT_LOG_FATAL_ERRORS != FALSE) */

/*!*************************************************************************** *
 * fw_stack_it
 * \brief   Stack error
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Fall-through to fw_fatal, with YL = 1
 * *************************************************************************** *
 * - Call Hierarchy: All unsupported interrupts
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
void fw_stack_it(void)
{
    __asm__ ("mov yl, #01");
    __asm__ ("jmp _fw_fatal");

} /* End of fw_stack_it() */

/*!*************************************************************************** *
 * fw_mlx16_operr
 * \brief   MLX16 Operation Error
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Fall-through to fw_fatal, with YL = 4
 * *************************************************************************** *
 * - Call Hierarchy: All unsupported interrupts
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
void fw_mlx16_operr(void)
{
    __asm__ ("mov yl, #04");
    __asm__ ("jmp _fw_fatal");

} /* End of fw_mlx16_operr() */

/*!*************************************************************************** *
 * fw_fatal
 * \brief   Handle fatal errors and unsupported interrupts
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details YL = IRQ-vector index
 *          Function can not return in case of system errors!
 * *************************************************************************** *
 * - Call Hierarchy: All unsupported interrupts
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
void fw_fatal(void) __attribute__((noreturn));
/* -------------------------------------------------------------------------
 * Fatal program termination
 */
void fw_fatal(void)
{
#if defined (SYM_DEBUG)
    printtext2("FATAL\n",6);
#endif /* SYM_DEBUG */
    __asm__ (
        "lod YH, #0\n\t"
        "mov _bistError, Y\n\t"
        "cmp Y, #01\n\t"                                                        /* Get address except in case of STACK_IT (MMP191224-1) */
        "je _fw_fatal_10_%=\n\t"
        "lod Y, [S-2]\n\t"                                                      /* Save address of failed instruction */
        "_fw_fatal_10_%=:\n\t"
        "mov _bistErrorInfo, Y"                                                 /* Failure address */
        ::);

#if (_DEBUG_SYSERR != FALSE)
    if ( (bistError & 0x00FFU) < 0x0005U)
    {
        while ( (bistError & 0x00FFU) != 0U)
        {
            DEBUG_TOG_IO_A();
            DEBUG_TOG_IO_A();
            bistError--;
        }
        DEBUG_CLR_IO_A();
    }
#endif /* (_DEBUG_SYSERR != FALSE) */

#if (_SUPPORT_CRASH_RECOVERY != FALSE)                                          /* MMP180730-2 */
    if ( (bistResetInfo == C_CHIP_STATE_FATAL_RECOVER_ENA) && ((bistError & 0x00FFU) < 0x0005U) )
    {
        /* System errors only */
        Sys_SetStack(&fw_stack);                                                /* Re-initialise stack */
#if (_SUPPORT_APP_USER_MODE != FALSE)
        ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        IO_MLX16_ITC_PEND0_S = M_MLX16_ITC_PEND0;
        IO_MLX16_ITC_PEND1_S = M_MLX16_ITC_PEND1;
        IO_MLX16_ITC_PEND2_S = M_MLX16_ITC_PEND2;
        IO_MLX16_ITC_PEND3_S = M_MLX16_ITC_PEND3;
#if defined (__MLX81160__) || defined (__MLX81332B01__) || defined (__MLX81332B02__) || defined (__MLX81334__) || \
    defined (__MLX81339__) || defined (__MLX81340__) || defined (__MLX81344__)
        IO_MLX16_ITC_PEND4_S = M_MLX16_ITC_PEND4;
#endif /* defined (__MLX81160__) || defined (__MLX81332B01__) || defined (__MLX81332B02__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81340__) || defined (__MLX81344__) */
#if defined (__MLX81340__) || defined (__MLX81344__)
        IO_MLX16_ITC_PEND5_S = M_MLX16_ITC_PEND5;
#endif /* defined (__MLX81340__) || defined (__MLX81344__) */
#if (_SUPPORT_APP_USER_MODE != FALSE)
        EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#if (LIN_COMM != FALSE)
        Set_Mlx4ErrorState(C_MLX4_STATE_IMMEDIATE_RST);                         /* Reset MLX4 always */
#endif /* (LIN_COMM != FALSE) */
#if (_SUPPORT_LOG_FATAL_ERRORS != FALSE)
        SetLastError(bistError);
#endif /* (_SUPPORT_LOG_FATAL_ERRORS != FALSE) */
#if defined (__MLX81330A01__)
        SET_PRIORITY(7);
#else
        builtin_mlx16_set_priority(7);                                          /* Protected mode, low priority (7) */
#endif
        bistResetInfo = C_CHIP_STATE_FATAL_CRASH_RECOVERY;
        __asm__ ("JMP _main\n\t" ::); /* jump to the main function */
    }
#endif /* (_SUPPORT_CRASH_RECOVERY != FALSE) */                                 /* MMP180730-2 */

#if defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81332__) || \
    defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
    IO_PORT_DRV_OUT &= ~M_PORT_DRV_OUT_ENABLE_DRV;
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    IO_PORT_DRV2_PROT = B_PORT_DRV2_PROT_DIS_DRV;
#endif

#if (_SUPPORT_LOG_FATAL_ERRORS != FALSE)
    /*SET_STACK(&fw_stack);*/
    Sys_SetStack(&fw_stack);                                                    /* Re-initialise stack */
    SetLastError(bistError);
#endif /* (_SUPPORT_LOG_FATAL_ERRORS != FALSE) */

    for (;;)
    {
        /* loop forever */
#if (_DEBUG_FATAL != FALSE)
        if (bistError > 0)
        {
            bistError--;
            DEBUG_TOG_IO_B();                                                   /* Number of pulses is fatal error-code */
        }
        else
        {
            DEBUG_TOG_IO_A();
        }
#endif /* (_DEBUG_FATAL != FALSE) */
    }
} /* End of fw_fatal() */

/* EOF */
