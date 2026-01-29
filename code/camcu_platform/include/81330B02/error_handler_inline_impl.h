/**
 * @file
 * @brief A library to provide the several type of errors definition during the code execution
 * @internal
 *
 * @copyright (C) 2019 Melexis N.V.
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
 * @ingroup error_handler
 *
 * @details
 * @{
 */

#ifndef ERROR_HANDLER_INLINE_IMPL_H
#define ERROR_HANDLER_INLINE_IMPL_H

#include "bl_bist.h"
#include "fatal.h"
#include "start_checks.h"

STATIC INLINE void ErrH_InitializeBistError(void)
{
    /* Initialize the additional errors bistError and bistErrorInfo (as compared to bistHeader and bistResetInfo)
     * for every cold boot */
    bistError = 0;
    bistErrorInfo = 0;
}

#ifdef HAS_SW_EXPLICIT_FAIL_INFO
/** Sets the BistError to desired value and stores the PC in BistErrorInfo
 * During the reset the Y-register represents the current stack-pointer value used.
 *
 * @param[in] information holds the reason of failure to indicate
 *
 * @note: The function assumes bistErrorInfo follows the bistError field in memory
 */
STATIC INLINE void ErrH_SetInfo (ErrH_Identifier information)
{
    __asm__ __volatile__ (
        "mov [%[bisterr]++], %[info]\n\t" \
        "mov [%[bisterr]++], PC\n\t" \
        "mov Y, S\n\t" \
        :
        : [info] "c" (information), [bisterr] "c" ((uint16_t)&bistError)
        :);
}

STATIC INLINE void ErrH_ResetInformed (ErrH_Identifier information)
{
#ifdef DBG_FPGA
    uint16_t retAddress = (fGetStackPointer() - 2u);
    dumpMem16(retAddress);
    writeln("RESI");
#endif
    ErrH_SetInfo(information);
    Sys_ResetCpu();
}

STATIC INLINE void ErrH_FatalInformed (ErrH_Identifier information)
{
#ifdef DBG_FPGA
    uint16_t retAddress = (fGetStackPointer() - 2u);
    dumpMem16(retAddress);
    writeln("RESI");
#endif
    ErrH_SetInfo(information);
    _fatal();
}

STATIC INLINE void ErrH_NopInformed(ErrH_Identifier information)
{
#ifdef DBG_FPGA
    uint16_t retAddress = (fGetStackPointer() - 2u);
    dumpMem16(retAddress);
    writeln("RESI");
#endif
    ErrH_SetInfo(information);
}

#else

STATIC INLINE void ErrH_ResetInformed(ErrH_Identifier information)
{
    UNUSED(information);
    Sys_ResetCpu();
}

STATIC INLINE void ErrH_FatalInformed(ErrH_Identifier information)
{
    UNUSED(information);
    _fatal();
}

STATIC INLINE void ErrH_NopInformed(ErrH_Identifier information)
{
    UNUSED(information);
}
#endif /* HAS_SW_EXPLICIT_FAIL_INFO */

STATIC INLINE void ErrH_Nop(ErrH_Identifier information)
{
    UNUSED(information);
}

STATIC INLINE void ErrH_Fatal (ErrH_Identifier information)
{
    UNUSED(information);
    _fatal();
}


#ifdef HAS_EE_WR_AREA_CTRL
STATIC INLINE void ErrH_TriggerRomModeInformed(ErrH_Identifier information)
{
#ifdef DBG_FPGA
    uint16_t retAddress = (fGetStackPointer() - 2u);
    dumpMem16(retAddress);
    writeln("RESI");
#endif
    ErrH_SetInfo(information);
    fFlagRunRom = true;
}
#endif /* HAS_EE_WR_AREA_CTRL */

#endif /* ERROR_HANDLER_INLINE_IMPL_H */
/// @}
