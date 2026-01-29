/**
 * @file
 * @brief UART serial interface support library
 * @internal
 *
 * @copyright (C) 2017-2018 Melexis N.V.
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
 * @ingroup lib_uart
 *
 * @brief UART serial interface support library
 * @details
 * @{
 */

#ifndef LIB_UART_INLINE_IMPL_H
#define LIB_UART_INLINE_IMPL_H

#include <stdint.h>
#include "compiler_abstraction.h"
#include "io.h"
#include "lib_wdg.h"
#include "itc_helper.h"

STATIC INLINE uint16_t UART_GetRate(uint32_t freq)
{
    return (uint16_t)(((uint32_t)FPLL * 1000u / 8u) / ((freq * 2u) - 1u));
}

#ifndef HAS_TWO_UARTS

STATIC INLINE void UART_SetBaudrate(uint16_t brrd_reg_value)
{
    IO_SET(UART, BRRD, brrd_reg_value);
}


STATIC INLINE void UART_Init(uint16_t rate, UartCtrl_t cfg)
{
    IO_SET(UART, REE, 0u, TRE, 0u);
    UART_SetBaudrate(rate);
    IO_HOST(UART, LDC) = cfg.data;
}


STATIC INLINE void UART_RriEnable(void)
{
    Itc_Clear(UART_RR);
    Itc_Enable(UART_RR);
}


STATIC INLINE void UART_RriDisable(void)
{
    Itc_Disable(UART_RR);
}


STATIC INLINE void UART_TriEnable(void)
{
    Itc_Clear(UART_TR);
    Itc_Enable(UART_TR);
}


STATIC INLINE void UART_TriDisable(void)
{
    Itc_Disable(UART_TR);
}


STATIC INLINE void UART_RsiEnable(void)
{
    Itc_Clear(UART_RS);
    Itc_Enable(UART_RS);
}


STATIC INLINE void UART_RsiDisable(void)
{
    Itc_Disable(UART_RS);
}


STATIC INLINE void UART_TsiEnable(void)
{
    Itc_Clear(UART_TS);
    Itc_Enable(UART_TS);
}


STATIC INLINE void UART_TsiDisable(void)
{
    Itc_Disable(UART_TS);
}


STATIC INLINE void UART_SbiEnable(void)
{
    Itc_Clear(UART_SB);
    Itc_Enable(UART_SB);
}


STATIC INLINE void UART_SbiDisable(void)
{
    Itc_Disable(UART_SB);
}


STATIC INLINE void UART_TeiEnable(void)
{
    Itc_Clear(UART_TE);
    Itc_Enable(UART_TE);
}


STATIC INLINE void UART_TeiDisable(void)
{
    Itc_Disable(UART_TE);
}


/* Flags support */

STATIC INLINE uint16_t UART_TxBusy(void)
{
    return IO_GET(UART, TRB);
}


STATIC INLINE uint16_t UART_RxFull(void)
{
    return IO_GET(UART, RRF);
}



/* Data sending */


STATIC INLINE void UART_InternalPutFrame(uint16_t length, uint16_t x)
{
    IO_SET(UART, TRD, (x << 1u) | ((uint16_t)1u << (length + 1u)));
}


STATIC INLINE void UART_PutFrame7N1(uint16_t data)
{
    UART_InternalPutFrame(7u, data);
}


STATIC INLINE void UART_PutFrame8N1(uint16_t data)
{
    UART_InternalPutFrame(8u, data);
}


STATIC INLINE void UART_PutFrame8N1M(uint16_t data)
{
    IO_SET(UART, TRD, (data << 7) | 0x40u ); // for 9bit cfg
}


STATIC INLINE void UART_PutFrame9N1(uint16_t data)
{
    UART_InternalPutFrame(9u, data);
}


STATIC INLINE void UART_PutFrame10N1(uint16_t data)
{
    UART_InternalPutFrame(10u, data);
}



STATIC INLINE void UART_PutFrame11N1(uint16_t data)
{
    UART_InternalPutFrame(11u, data);
}



STATIC INLINE void UART_PutFrame12N1(uint16_t data)
{
    UART_InternalPutFrame(12u, data);
}



STATIC INLINE void UART_PutFrame13N1(uint16_t data)
{
    UART_InternalPutFrame(13u, data);
}



STATIC INLINE void UART_PutFrame14N1(uint16_t data)
{
    UART_InternalPutFrame(14u, data);
}



STATIC void UART_WaitTx(void)
{
    while (IO_GET(UART, TRB) != 0u) {
        WDG_conditionalAwdRefresh();
    }
}



STATIC INLINE void UART_PutBreakBlocking(UartMlsCtrl_t break_length)
{
    uint16_t tmp = IO_GET(UART, MLS); /* save the previous config */
    UART_WaitTx();
    IO_SET(UART, MLS, (uint16_t)break_length); /** Creating the 10bit break */
    IO_SET(UART, TRD, (uint16_t)((uint16_t)0x0080u >> (uint16_t) break_length));
    UART_WaitTx(); /** Wait for tranmitter register is ready to make sure we will set the shifter to the previous mode */
    IO_SET(UART, MLS, tmp);
}


STATIC INLINE void UART_PutRaw(uint16_t data)
{
    IO_SET(UART, TRD, data);
}


/* Read functions */

STATIC INLINE uint16_t UART_Get(void)
{
    return IO_GET(UART, RRD);
}

/*********************** Function from lm_uart, Mulan2/3 ******************************************/


STATIC INLINE void UART_TxEnable(void)
{
    IO_SET(UART, TRE, 1u);
}



STATIC INLINE void UART_TxDisable(void)
{
    IO_SET(UART, TRE, 0u);
}



STATIC INLINE void UART_RxEnable(void)
{
    IO_SET(UART, REE, 1u);
}



STATIC INLINE void UART_RxDisable(void)
{
    IO_SET(UART, REE, 0u);
}



STATIC INLINE void UART_AllDisable(void)
{
    IO_SET(UART, TRE, 0u, REE, 0u);
}



STATIC INLINE void UART_LinLdcEnable(void)
{
    IO_SET(UART, LDC, 1u);
}



STATIC INLINE void UART_LinLdcDisable(void)
{
    IO_SET(UART, LDC, 0u);
}



STATIC INLINE void UART_LinLscEnable(void)
{
    IO_SET(UART, LSC, 1u);
}



STATIC INLINE void UART_LinLscDisable(void)
{
    IO_SET(UART, LSC, 0u);
}



STATIC INLINE void UART_LinLbcEnable(void)
{
    IO_SET(UART, LBC, 1u);
}


STATIC INLINE void UART_LinLbcDisable(void)
{
    IO_SET(UART, LBC, 0u);
}


STATIC INLINE void UART_ResetRegister(void)
{
    IO_HOST(UART, TRE) = 0u;
}


STATIC INLINE void UART_TxFormat(UartBscCtrl_t format)
{
    IO_SET(UART, BSC, (uint16_t)format);
}


STATIC INLINE void UART_FrameLength(UartMlsCtrl_t len)
{
    IO_SET(UART, MLS, (uint16_t)len);
}

#else /* HAS_TWO_UARTS */


STATIC INLINE void UART0_SetBaudrate(uint16_t brrd_reg_value)
{
    IO_SET(UART0, BRRD, brrd_reg_value);
}


STATIC INLINE void UART0_Init(uint16_t rate, UartCtrl_t cfg)
{
    IO_SET(UART0, REE, 0u, TRE, 0u);
    UART0_SetBaudrate(rate);
    IO_HOST(UART0, LDC) = cfg.data;
}


STATIC INLINE void UART0_RriEnable(void)
{
    Itc_Clear(UART0_RR);
    Itc_Enable(UART0_RR);
}


STATIC INLINE void UART0_RriDisable(void)
{
    Itc_Disable(UART0_RR);
}


STATIC INLINE void UART0_TriEnable(void)
{
    Itc_Clear(UART0_TR);
    Itc_Enable(UART0_TR);
}


STATIC INLINE void UART0_TriDisable(void)
{
    Itc_Disable(UART0_TR);
}


STATIC INLINE void UART0_RsiEnable(void)
{
    Itc_Clear(UART0_RS);
    Itc_Enable(UART0_RS);
}


STATIC INLINE void UART0_RsiDisable(void)
{
    Itc_Disable(UART0_RS);
}


STATIC INLINE void UART0_TsiEnable(void)
{
    Itc_Clear(UART0_TS);
    Itc_Enable(UART0_TS);
}


STATIC INLINE void UART0_TsiDisable(void)
{
    Itc_Disable(UART0_TS);
}


STATIC INLINE void UART0_SbiEnable(void)
{
    Itc_Clear(UART0_SB);
    Itc_Enable(UART0_SB);
}


STATIC INLINE void UART0_SbiDisable(void)
{
    Itc_Disable(UART0_SB);
}


STATIC INLINE void UART0_TeiEnable(void)
{
    Itc_Clear(UART0_TE);
    Itc_Enable(UART0_TE);
}


STATIC INLINE void UART0_TeiDisable(void)
{
    Itc_Disable(UART0_TE);
}


/* Flags support */

STATIC INLINE uint16_t UART0_TxBusy(void)
{
    return IO_GET(UART0, TRB);
}


STATIC INLINE uint16_t UART0_RxFull(void)
{
    return IO_GET(UART0, RRF);
}



/* Data sending */


STATIC INLINE void INTERNAL_UART0_PUT_FRAME(uint16_t length, uint16_t x)
{
    IO_SET(UART0, TRD, (x << 1u) | ((uint16_t)1u << (length + 1u)));
}


STATIC INLINE void UART0_PutFrame7N1(uint16_t data)
{
    INTERNAL_UART0_PUT_FRAME(7u, data);
}


STATIC INLINE void UART0_PutFrame8N1(uint16_t data)
{
    INTERNAL_UART0_PUT_FRAME(8u, data);
}


STATIC INLINE void UART0_PutFrame8N1M(uint16_t data)
{
    IO_SET(UART0, TRD, (data << 7) | 0x40u ); // for 9bit cfg
}


STATIC INLINE void UART0_PutFrame9N1(uint16_t data)
{
    INTERNAL_UART0_PUT_FRAME(9u, data);
}


STATIC INLINE void UART0_PutFrame10N1(uint16_t data)
{
    INTERNAL_UART0_PUT_FRAME(10u, data);
}



STATIC INLINE void UART0_PutFrame11N1(uint16_t data)
{
    INTERNAL_UART0_PUT_FRAME(11u, data);
}



STATIC INLINE void UART0_PutFrame12N1(uint16_t data)
{
    INTERNAL_UART0_PUT_FRAME(12u, data);
}



STATIC INLINE void UART0_PutFrame13N1(uint16_t data)
{
    INTERNAL_UART0_PUT_FRAME(13u, data);
}



STATIC INLINE void UART0_PutFrame14N1(uint16_t data)
{
    INTERNAL_UART0_PUT_FRAME(14u, data);
}



STATIC void UART0_WaitTx(void)
{
    while (IO_GET(UART0, TRB) != 0u) {
        WDG_conditionalAwdRefresh();
    }
}



STATIC INLINE void UART0_PutBreakBlocking(UartMlsCtrl_t break_length)
{
    uint16_t tmp = IO_GET(UART0, MLS); /* save the previous config */
    UART0_WaitTx();
    IO_SET(UART0, MLS, (uint16_t)break_length); /** Creating the 10bit break */
    IO_SET(UART0, TRD, (uint16_t)((uint16_t)0x0080u >> (uint16_t) break_length));
    UART0_WaitTx(); /** Wait for tranmitter register is ready to make sure we will set the shifter to the previous mode */
    IO_SET(UART0, MLS, tmp);
}


STATIC INLINE void UART0_PutRaw(uint16_t data)
{
    IO_SET(UART0, TRD, data);
}


/* Read functions */

STATIC INLINE uint16_t UART0_Get(void)
{
    return IO_GET(UART0, RRD);
}

/*********************** Function from lm_uart, Mulan2/3 ******************************************/


STATIC INLINE void UART0_TxEnable(void)
{
    IO_SET(UART0, TRE, 1u);
}



STATIC INLINE void UART0_TxDisable(void)
{
    IO_SET(UART0, TRE, 0u);
}



STATIC INLINE void UART0_RxEnable(void)
{
    IO_SET(UART0, REE, 1u);
}



STATIC INLINE void UART0_RxDisable(void)
{
    IO_SET(UART0, REE, 0u);
}



STATIC INLINE void UART0_AllDisable(void)
{
    IO_SET(UART0, TRE, 0u, REE, 0u);
}



STATIC INLINE void UART0_LinLdcEnable(void)
{
    IO_SET(UART0, LDC, 1u);
}



STATIC INLINE void UART0_LinLdcDisable(void)
{
    IO_SET(UART0, LDC, 0u);
}



STATIC INLINE void UART0_LinLscEnable(void)
{
    IO_SET(UART0, LSC, 1u);
}



STATIC INLINE void UART0_LinLscDisable(void)
{
    IO_SET(UART0, LSC, 0u);
}



STATIC INLINE void UART0_LinLbcEnable(void)
{
    IO_SET(UART0, LBC, 1u);
}


STATIC INLINE void UART0_LinLbcDisable(void)
{
    IO_SET(UART0, LBC, 0u);
}


STATIC INLINE void UART0_ResetRegister(void)
{
    IO_HOST(UART0, TRE) = 0u;
}


STATIC INLINE void UART0_TxFormat(UartBscCtrl_t format)
{
    IO_SET(UART0, BSC, (uint16_t)format);
}


STATIC INLINE void UART0_FrameLength(UartMlsCtrl_t len)
{
    IO_SET(UART0, MLS, (uint16_t)len);
}






STATIC INLINE void UART1_SetBaudrate(uint16_t brrd_reg_value)
{
    IO_SET(UART1, BRRD, brrd_reg_value);
}


STATIC INLINE void UART1_Init(uint16_t rate, UartCtrl_t cfg)
{
    IO_SET(UART1, REE, 0u, TRE, 0u);
    UART1_SetBaudrate(rate);
    IO_HOST(UART1, LDC) = cfg.data;
}


STATIC INLINE void UART1_RriEnable(void)
{
    Itc_Clear(UART1_RR);
    Itc_Enable(UART1_RR);
}


STATIC INLINE void UART1_RriDisable(void)
{
    Itc_Disable(UART1_RR);
}


STATIC INLINE void UART1_TriEnable(void)
{
    Itc_Clear(UART1_TR);
    Itc_Enable(UART1_TR);
}


STATIC INLINE void UART1_TriDisable(void)
{
    Itc_Disable(UART1_TR);
}


STATIC INLINE void UART1_RsiEnable(void)
{
    Itc_Clear(UART1_RS);
    Itc_Enable(UART1_RS);
}


STATIC INLINE void UART1_RsiDisable(void)
{
    Itc_Disable(UART1_RS);
}


STATIC INLINE void UART1_TsiEnable(void)
{
    Itc_Clear(UART1_TS);
    Itc_Enable(UART1_TS);
}


STATIC INLINE void UART1_TsiDisable(void)
{
    Itc_Disable(UART1_TS);
}


STATIC INLINE void UART1_SbiEnable(void)
{
    Itc_Clear(UART1_SB);
    Itc_Enable(UART1_SB);
}


STATIC INLINE void UART1_SbiDisable(void)
{
    Itc_Disable(UART1_SB);
}


STATIC INLINE void UART1_TeiEnable(void)
{
    Itc_Clear(UART1_TE);
    Itc_Enable(UART1_TE);
}


STATIC INLINE void UART1_TeiDisable(void)
{
    Itc_Disable(UART1_TE);
}


/* Flags support */

STATIC INLINE uint16_t UART1_TxBusy(void)
{
    return IO_GET(UART1, TRB);
}


STATIC INLINE uint16_t UART1_RxFull(void)
{
    return IO_GET(UART1, RRF);
}



/* Data sending */


STATIC INLINE void INTERNAL_UART1_PUT_FRAME(uint16_t length, uint16_t x)
{
    IO_SET(UART1, TRD, (x << 1u) | ((uint16_t)1u << (length + 1u)));
}


STATIC INLINE void UART1_PutFrame7N1(uint16_t data)
{
    INTERNAL_UART1_PUT_FRAME(7u, data);
}


STATIC INLINE void UART1_PutFrame8N1(uint16_t data)
{
    INTERNAL_UART1_PUT_FRAME(8u, data);
}


STATIC INLINE void UART1_PutFrame8N1M(uint16_t data)
{
    IO_SET(UART1, TRD, (data << 7) | 0x40u ); // for 9bit cfg
}


STATIC INLINE void UART1_PutFrame9N1(uint16_t data)
{
    INTERNAL_UART1_PUT_FRAME(9u, data);
}


STATIC INLINE void UART1_PutFrame10N1(uint16_t data)
{
    INTERNAL_UART1_PUT_FRAME(10u, data);
}



STATIC INLINE void UART1_PutFrame11N1(uint16_t data)
{
    INTERNAL_UART1_PUT_FRAME(11u, data);
}



STATIC INLINE void UART1_PutFrame12N1(uint16_t data)
{
    INTERNAL_UART1_PUT_FRAME(12u, data);
}



STATIC INLINE void UART1_PutFrame13N1(uint16_t data)
{
    INTERNAL_UART1_PUT_FRAME(13u, data);
}



STATIC INLINE void UART1_PutFrame14N1(uint16_t data)
{
    INTERNAL_UART1_PUT_FRAME(14u, data);
}



STATIC void UART1_WaitTx(void)
{
    while (IO_GET(UART1, TRB) != 0u) {
        WDG_conditionalAwdRefresh();
    }
}



STATIC INLINE void UART1_PutBreakBlocking(UartMlsCtrl_t break_length)
{
    uint16_t tmp = IO_GET(UART1, MLS); /* save the previous config */
    UART1_WaitTx();
    IO_SET(UART1, MLS, (uint16_t)break_length); /** Creating the 10bit break */
    IO_SET(UART1, TRD, (uint16_t)((uint16_t)0x0080u >> (uint16_t) break_length));
    UART1_WaitTx(); /** Wait for tranmitter register is ready to make sure we will set the shifter to the previous mode */
    IO_SET(UART1, MLS, tmp);
}


STATIC INLINE void UART1_PutRaw(uint16_t data)
{
    IO_SET(UART1, TRD, data);
}


/* Read functions */

STATIC INLINE uint16_t UART1_Get(void)
{
    return IO_GET(UART1, RRD);
}

/*********************** Function from lm_uart, Mulan2/3 ******************************************/


STATIC INLINE void UART1_TxEnable(void)
{
    IO_SET(UART1, TRE, 1u);
}



STATIC INLINE void UART1_TxDisable(void)
{
    IO_SET(UART1, TRE, 0u);
}



STATIC INLINE void UART1_RxEnable(void)
{
    IO_SET(UART1, REE, 1u);
}



STATIC INLINE void UART1_RxDisable(void)
{
    IO_SET(UART1, REE, 0u);
}



STATIC INLINE void UART1_AllDisable(void)
{
    IO_SET(UART1, TRE, 0u, REE, 0u);
}



STATIC INLINE void UART1_LinLdcEnable(void)
{
    IO_SET(UART1, LDC, 1u);
}



STATIC INLINE void UART1_LinLdcDisable(void)
{
    IO_SET(UART1, LDC, 0u);
}



STATIC INLINE void UART1_LinLscEnable(void)
{
    IO_SET(UART1, LSC, 1u);
}



STATIC INLINE void UART1_LinLscDisable(void)
{
    IO_SET(UART1, LSC, 0u);
}



STATIC INLINE void UART1_LinLbcEnable(void)
{
    IO_SET(UART1, LBC, 1u);
}


STATIC INLINE void UART1_LinLbcDisable(void)
{
    IO_SET(UART1, LBC, 0u);
}


STATIC INLINE void UART1_ResetRegister(void)
{
    IO_HOST(UART1, TRE) = 0u;
}


STATIC INLINE void UART1_TxFormat(UartBscCtrl_t format)
{
    IO_SET(UART1, BSC, (uint16_t)format);
}


STATIC INLINE void UART1_FrameLength(UartMlsCtrl_t len)
{
    IO_SET(UART1, MLS, (uint16_t)len);
}

#endif /* HAS_TWO_UARTS */

#endif /* LIB_UART_INLINE_IMPL_H */
/// @}

