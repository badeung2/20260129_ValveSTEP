/*!************************************************************************** *
 * \file        HAL_UART_inline.h
 * \brief       Hardware Abstraction Layer for UART handling (inline)
 *
 * \note        project MLX81160/33x/34x/35x
 *
 * \author      Marcel Braat
 *
 * \date        2024-05-17
 *
 * \version     2.0
 *
 * A list of functions:
 *  - Inline Functions:
 *           -# HAL_UART0_RX_Setup()
 *           -# HAL_UART0_TX_Setup()
 *           -# HAL_UART0_SetPrioAndEnableIRQ()
 *           -# HAL_UART1_RX_Setup()
 *           -# HAL_UART1_TX_Setup()
 *           -# HAL_UART1_SetPrioAndEnableIRQ()
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

#ifndef HAL_LIB_UART_INLINE_H
#define HAL_LIB_UART_INLINE_H

/*!************************************************************************** */
/*                          INCLUDES                                          */
/* ************************************************************************** */
#include "../AppBuild.h"                                                        /* Application Build */

#if (_SUPPORT_UART != FALSE)

#include <atomic.h>


#if (_SUPPORT_UART_RX_IO == PIN_FUNC_IO_0)
#define C_PORT_COMM_CFG1_UART0_RX_SEL   C_PORT_COMM_CFG1_UART0_RX_SEL_IO0
#define IO_PORT_IO_ENABLE_RX            IO_PORT_IO_ENABLE
#define B_PORT_IO_ENABLE_RX             B_PORT_IO_ENABLE_IO_ENABLE_0
#define IO_PORT_IO_RX_LV                IO_PORT_IO_OUT_EN
#if defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
#define B_PORT_IO_RX_LV_ENABLE          B_PORT_IO_OUT_EN_IO0_LV_ENABLE
#elif defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#define B_PORT_IO_LV_ENABLE             B_PORT_IO_OUT_EN_IO_LV_ENABLE_0
#endif
#elif (_SUPPORT_UART_RX_IO == PIN_FUNC_IO_1)
#define C_PORT_COMM_CFG1_UART0_RX_SEL   C_PORT_COMM_CFG1_UART0_RX_SEL_IO1
#define IO_PORT_IO_ENABLE_RX            IO_PORT_IO_ENABLE
#define B_PORT_IO_ENABLE_RX             B_PORT_IO_ENABLE_IO_ENABLE_1
#if defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#define IO_PORT_IO_RX_LV                IO_PORT_IO_OUT_EN
#define B_PORT_IO_LV_ENABLE             B_PORT_IO_OUT_EN_IO_LV_ENABLE_1
#endif
#elif (_SUPPORT_UART_RX_IO == PIN_FUNC_IO_2)
#define C_PORT_COMM_CFG1_UART0_RX_SEL   C_PORT_COMM_CFG1_UART0_RX_SEL_IO2
#define IO_PORT_IO_ENABLE_RX            IO_PORT_IO_ENABLE
#define B_PORT_IO_ENABLE_RX             B_PORT_IO_ENABLE_IO_ENABLE_2
#if defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#define IO_PORT_IO_RX_LV                IO_PORT_IO_OUT_EN
#define B_PORT_IO_LV_ENABLE             B_PORT_IO_OUT_EN_IO_LV_ENABLE_2
#endif
#elif (_SUPPORT_UART_RX_IO == PIN_FUNC_IO_3)
#define C_PORT_COMM_CFG1_UART0_RX_SEL   C_PORT_COMM_CFG1_UART0_RX_SEL_IO3
#define IO_PORT_IO_ENABLE_RX            IO_PORT_IO_ENABLE
#define B_PORT_IO_ENABLE_RX             B_PORT_IO_ENABLE_IO_ENABLE_3
#if defined (__MLX81344__) || defined (__MLX81346__)
#define IO_PORT_IO_RX_LV                IO_PORT_IO_OUT_EN
#define B_PORT_IO_LV_ENABLE             B_PORT_IO_OUT_EN_IO_LV_ENABLE_3
#endif
#elif defined (PIN_FUNC_IO_4) && (_SUPPORT_UART_RX_IO == PIN_FUNC_IO_4)
#define C_PORT_COMM_CFG1_UART0_RX_SEL   C_PORT_COMM_CFG1_UART0_RX_SEL_IO4
#define IO_PORT_IO_ENABLE_RX            IO_PORT_IO_ENABLE
#define B_PORT_IO_ENABLE_RX             B_PORT_IO_ENABLE_IO_ENABLE_4
#if defined (__MLX81344__) || defined (__MLX81346__)
#define IO_PORT_IO_RX_LV                IO_PORT_IO_OUT_EN
#define B_PORT_IO_LV_ENABLE             B_PORT_IO_OUT_EN_IO_LV_ENABLE_4
#endif
#elif defined (PIN_FUNC_IO_5) && (_SUPPORT_UART_RX_IO == PIN_FUNC_IO_5)
#define C_PORT_COMM_CFG1_UART0_RX_SEL   C_PORT_COMM_CFG1_UART0_RX_SEL_IO5
#define IO_PORT_IO_ENABLE_RX            IO_PORT_IO_ENABLE
#define B_PORT_IO_ENABLE_RX             B_PORT_IO_ENABLE_IO_ENABLE_5
#elif defined (PIN_FUNC_IO_6) && (_SUPPORT_UART_RX_IO == PIN_FUNC_IO_6)
#define C_PORT_COMM_CFG1_UART0_RX_SEL   C_PORT_COMM_CFG1_UART0_RX_SEL_IO6
#define IO_PORT_IO_ENABLE_RX            IO_PORT_IO_ENABLE
#define B_PORT_IO_ENABLE_RX             B_PORT_IO_ENABLE_IO_ENABLE_6
#elif defined (PIN_FUNC_IO_7) && (_SUPPORT_UART_RX_IO == PIN_FUNC_IO_7)
#define C_PORT_COMM_CFG1_UART0_RX_SEL   C_PORT_COMM_CFG1_UART0_RX_SEL_IO7
#define IO_PORT_IO_ENABLE_RX            IO_PORT_IO_ENABLE
#define B_PORT_IO_ENABLE_RX             B_PORT_IO_ENABLE_IO_ENABLE_7
#elif defined (PIN_FUNC_IO_8) && (_SUPPORT_UART_RX_IO == PIN_FUNC_IO_8)
#define C_PORT_COMM_CFG1_UART0_RX_SEL   C_PORT_COMM_CFG1_UART0_RX_SEL_IO8
#define IO_PORT_IO_ENABLE_RX            IO_PORT_IO_ENABLE1
#define B_PORT_IO_ENABLE_RX             B_PORT_IO_ENABLE1_IO_ENABLE_8
#elif defined (PIN_FUNC_IO_9) && (_SUPPORT_UART_RX_IO == PIN_FUNC_IO_9)
#define C_PORT_COMM_CFG1_UART0_RX_SEL   C_PORT_COMM_CFG1_UART0_RX_SEL_IO9
#define IO_PORT_IO_ENABLE_RX            IO_PORT_IO_ENABLE1
#define B_PORT_IO_ENABLE_RX             B_PORT_IO_ENABLE1_IO_ENABLE_9
#elif defined (PIN_FUNC_IO_10) && (_SUPPORT_UART_RX_IO == PIN_FUNC_IO_10)
#define C_PORT_COMM_CFG1_UART0_RX_SEL   C_PORT_COMM_CFG1_UART0_RX_SEL_IO10
#define IO_PORT_IO_ENABLE_RX            IO_PORT_IO_ENABLE1
#define B_PORT_IO_ENABLE_RX             B_PORT_IO_ENABLE1_IO_ENABLE_10
#elif defined (PIN_FUNC_IO_11) && (_SUPPORT_UART_RX_IO == PIN_FUNC_IO_11)
#define C_PORT_COMM_CFG1_UART0_RX_SEL   C_PORT_COMM_CFG1_UART0_RX_SEL_IO11
#define IO_PORT_IO_ENABLE_RX            IO_PORT_IO_ENABLE1
#define B_PORT_IO_ENABLE_RX             B_PORT_IO_ENABLE1_IO_ENABLE_11
#elif (_SUPPORT_UART_RX_IO == PIN_FUNC_LIN)
#error "ERROR: LIN not supported for UART RX IO"
#else
#error "ERROR: Undefined UART RX IO"
#endif

#if (_SUPPORT_UART_TX_IO == PIN_FUNC_IO_0)
#define IO_PORT_IO_TX_CFG               IO_PORT_IO_CFG0
#define M_PORT_IO_TX_CFG_SEL            M_PORT_IO_CFG0_IO0_OUT_SEL
#define C_PORT_IO_TX_CFG_UART           C_PORT_IO_CFG0_IO0_OUT_SEL_UART
#define IO_PORT_IO_ENABLE_TX            IO_PORT_IO_ENABLE
#define B_PORT_IO_ENABLE_TX             B_PORT_IO_ENABLE_IO_ENABLE_0
#define IO_PORT_IO_TX_LV                IO_PORT_IO_OUT_EN
#define IO_PORT_IO_TX_OD                IO_PORT_IO_OUT_EN
#if defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
#define B_PORT_IO_TX_LV_ENABLE          B_PORT_IO_OUT_EN_IO0_LV_ENABLE
#define B_PORT_IO_TX_OD                 B_PORT_IO_OUT_EN_IO0_OD_ENABLE
#elif defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#define B_PORT_IO_TX_LV_ENABLE          B_PORT_IO_OUT_EN_IO_LV_ENABLE_0
#define B_PORT_IO_TX_OD                 B_PORT_IO_OUT_EN_IO_OD_ENABLE_0
#define C_PORT_IO_UART_SEL              C_PORT_IO_UART_SEL_IO0
#endif
#elif (_SUPPORT_UART_TX_IO == PIN_FUNC_IO_1)
#define IO_PORT_IO_TX_CFG               IO_PORT_IO_CFG0
#define M_PORT_IO_TX_CFG_SEL            M_PORT_IO_CFG0_IO1_OUT_SEL
#define C_PORT_IO_TX_CFG_UART           C_PORT_IO_CFG0_IO1_OUT_SEL_UART
#define IO_PORT_IO_ENABLE_TX            IO_PORT_IO_ENABLE
#define B_PORT_IO_ENABLE_TX             B_PORT_IO_ENABLE_IO_ENABLE_1
#if defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#define IO_PORT_IO_TX_OD                IO_PORT_IO_OUT_EN
#define B_PORT_IO_TX_OD                 B_PORT_IO_OUT_EN_IO_OD_ENABLE_1
#define C_PORT_IO_UART_SEL              C_PORT_IO_UART_SEL_IO1
#endif
#elif (_SUPPORT_UART_TX_IO == PIN_FUNC_IO_2)
#define IO_PORT_IO_TX_CFG               IO_PORT_IO_CFG0
#define M_PORT_IO_TX_CFG_SEL            M_PORT_IO_CFG0_IO2_OUT_SEL
#define C_PORT_IO_TX_CFG_UART           C_PORT_IO_CFG0_IO2_OUT_SEL_UART
#define IO_PORT_IO_ENABLE_TX            IO_PORT_IO_ENABLE
#define B_PORT_IO_ENABLE_TX             B_PORT_IO_ENABLE_IO_ENABLE_2
#if defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#define IO_PORT_IO_TX_OD                IO_PORT_IO_OUT_EN
#define B_PORT_IO_TX_OD                 B_PORT_IO_OUT_EN_IO_OD_ENABLE_2
#define C_PORT_IO_UART_SEL              C_PORT_IO_UART_SEL_IO2
#endif
#elif (_SUPPORT_UART_TX_IO == PIN_FUNC_IO_3)
#define IO_PORT_IO_TX_CFG               IO_PORT_IO_CFG0
#define M_PORT_IO_TX_CFG_SEL            M_PORT_IO_CFG0_IO3_OUT_SEL
#define C_PORT_IO_TX_CFG_UART           C_PORT_IO_CFG0_IO3_OUT_SEL_UART
#define IO_PORT_IO_ENABLE_TX            IO_PORT_IO_ENABLE
#define B_PORT_IO_ENABLE_TX             B_PORT_IO_ENABLE_IO_ENABLE_3
#if defined (__MLX81344__) || defined (__MLX81346__)
#define IO_PORT_IO_TX_OD                IO_PORT_IO_OUT_EN
#define B_PORT_IO_TX_OD                 B_PORT_IO_OUT_EN_IO_OD_ENABLE_3
#endif
#if defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#define C_PORT_IO_UART_SEL              C_PORT_IO_UART_SEL_IO3
#endif
#elif defined (PIN_FUNC_IO_4) && (_SUPPORT_UART_TX_IO == PIN_FUNC_IO_4)
#define IO_PORT_IO_TX_CFG               IO_PORT_IO_CFG1
#define M_PORT_IO_TX_CFG_SEL            M_PORT_IO_CFG1_IO4_OUT_SEL
#define C_PORT_IO_TX_CFG_UART           C_PORT_IO_CFG1_IO4_OUT_SEL_UART
#define IO_PORT_IO_ENABLE_TX            IO_PORT_IO_ENABLE
#define B_PORT_IO_ENABLE_TX             B_PORT_IO_ENABLE_IO_ENABLE_4
#if defined (__MLX81344__) || defined (__MLX81346__)
#define IO_PORT_IO_TX_OD                IO_PORT_IO_OUT_EN
#define B_PORT_IO_TX_OD                 B_PORT_IO_OUT_EN_IO_OD_ENABLE_4
#endif
#if defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#define C_PORT_IO_UART_SEL              C_PORT_IO_UART_SEL_IO4
#endif
#elif defined (PIN_FUNC_IO_5) && (_SUPPORT_UART_TX_IO == PIN_FUNC_IO_5)
#define IO_PORT_IO_TX_CFG               IO_PORT_IO_CFG1
#define M_PORT_IO_TX_CFG_SEL            M_PORT_IO_CFG1_IO5_OUT_SEL
#define C_PORT_IO_TX_CFG_UART           C_PORT_IO_CFG1_IO5_OUT_SEL_UART
#define IO_PORT_IO_ENABLE_TX            IO_PORT_IO_ENABLE
#define B_PORT_IO_ENABLE_TX             B_PORT_IO_ENABLE_IO_ENABLE_5
#if defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#define C_PORT_IO_UART_SEL              C_PORT_IO_UART_SEL_IO5
#endif
#elif defined (PIN_FUNC_IO_6) && (_SUPPORT_UART_TX_IO == PIN_FUNC_IO_6)
#define IO_PORT_IO_TX_CFG               IO_PORT_IO_CFG1
#define M_PORT_IO_TX_CFG_SEL            M_PORT_IO_CFG1_IO6_OUT_SEL
#define C_PORT_IO_TX_CFG_UART           C_PORT_IO_CFG1_IO6_OUT_SEL_UART
#define IO_PORT_IO_ENABLE_TX            IO_PORT_IO_ENABLE
#define B_PORT_IO_ENABLE_TX             B_PORT_IO_ENABLE_IO_ENABLE_6
#if defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#define C_PORT_IO_UART_SEL              C_PORT_IO_UART_SEL_IO6
#endif
#elif defined (PIN_FUNC_IO_7) && (_SUPPORT_UART_TX_IO == PIN_FUNC_IO_7)
#define IO_PORT_IO_TX_CFG               IO_PORT_IO_CFG1
#define M_PORT_IO_TX_CFG_SEL            M_PORT_IO_CFG1_IO7_OUT_SEL
#define C_PORT_IO_TX_CFG_UART           C_PORT_IO_CFG1_IO7_OUT_SEL_UART
#define IO_PORT_IO_ENABLE_TX            IO_PORT_IO_ENABLE
#define B_PORT_IO_ENABLE_TX             B_PORT_IO_ENABLE_IO_ENABLE_7
#if defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__)
#define IO_PORT_IO_TX_OD                IO_PORT_IO_OUT_EN
#define B_PORT_IO_TX_OD                 B_PORT_IO_OUT_EN_IO7_OD_ENABLE
#endif
#if defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#define C_PORT_IO_UART_SEL              C_PORT_IO_UART_SEL_IO7
#endif
#elif defined (PIN_FUNC_IO_8) && (_SUPPORT_UART_TX_IO == PIN_FUNC_IO_8)
#define IO_PORT_IO_TX_CFG               IO_PORT_IO_CFG2
#define M_PORT_IO_TX_CFG_SEL            M_PORT_IO_CFG2_IO8_OUT_SEL
#define C_PORT_IO_TX_CFG_UART           C_PORT_IO_CFG2_IO8_OUT_SEL_UART
#define IO_PORT_IO_ENABLE_TX            IO_PORT_IO_ENABLE1
#define B_PORT_IO_ENABLE_TX             B_PORT_IO_ENABLE1_IO_ENABLE_8
#if defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#define C_PORT_IO_UART_SEL              C_PORT_IO_UART_SEL_IO8
#endif
#elif defined (PIN_FUNC_IO_9) && (_SUPPORT_UART_TX_IO == PIN_FUNC_IO_9)
#define IO_PORT_IO_TX_CFG               IO_PORT_IO_CFG2
#define M_PORT_IO_TX_CFG_SEL            M_PORT_IO_CFG2_IO9_OUT_SEL
#define C_PORT_IO_TX_CFG_UART           C_PORT_IO_CFG2_IO9_OUT_SEL_UART
#define IO_PORT_IO_ENABLE_TX            IO_PORT_IO_ENABLE1
#define B_PORT_IO_ENABLE_TX             B_PORT_IO_ENABLE1_IO_ENABLE_9
#if defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#define C_PORT_IO_UART_SEL              C_PORT_IO_UART_SEL_IO9
#endif
#elif defined (PIN_FUNC_IO_10) && (_SUPPORT_UART_TX_IO == PIN_FUNC_IO_10)
#define IO_PORT_IO_TX_CFG               IO_PORT_IO_CFG2
#define M_PORT_IO_TX_CFG_SEL            M_PORT_IO_CFG2_IO10_OUT_SEL
#define C_PORT_IO_TX_CFG_UART           C_PORT_IO_CFG2_IO10_OUT_SEL_UART
#define IO_PORT_IO_ENABLE_TX            IO_PORT_IO_ENABLE1
#define B_PORT_IO_ENABLE_TX             B_PORT_IO_ENABLE1_IO_ENABLE_10
#if defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#define C_PORT_IO_UART_SEL              C_PORT_IO_UART_SEL_IO10
#endif
#elif defined (PIN_FUNC_IO_11) && (_SUPPORT_UART_TX_IO == PIN_FUNC_IO_11)
#define IO_PORT_IO_TX_CFG               IO_PORT_IO_CFG2
#define M_PORT_IO_TX_CFG_SEL            M_PORT_IO_CFG2_IO11_OUT_SEL
#define C_PORT_IO_TX_CFG_UART           C_PORT_IO_CFG2_IO11_OUT_SEL_UART
#define IO_PORT_IO_ENABLE_TX            IO_PORT_IO_ENABLE1
#define B_PORT_IO_ENABLE_TX             B_PORT_IO_ENABLE1_IO_ENABLE_11
#if defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#define C_PORT_IO_UART_SEL              C_PORT_IO_UART_SEL_IO11
#endif
#elif (_SUPPORT_UART_TX_IO == PIN_FUNC_LIN)
#error "ERROR: LIN not supported for UART TX IO"
#else
#error "ERROR: Undefined UART TX IO"
#endif

#if !defined (M_PORT_COMM_CFG1_UART_RX_SEL)
#define M_PORT_COMM_CFG1_UART_RX_SEL    M_PORT_COMM_CFG1_UART0_RX_SEL
#endif

/*!*************************************************************************** *
 * HAL_UART0_RX_Setup
 * \brief   Setup UART0 RX configuration
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Setup UART0 RX configuration by assigning the UART RX I/O-pin,
 *          in input-mode (and if applicable in LV-mode).
 *          This is the first UART interface
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static __inline__ void HAL_UART0_RX_Setup(void)
{
    IO_PORT_COMM_CFG1 = (IO_PORT_COMM_CFG1 & ~M_PORT_COMM_CFG1_UART_RX_SEL) | C_PORT_COMM_CFG1_UART0_RX_SEL;  /* UART_RX from IO[RX] */
    IO_PORT_IO_ENABLE_RX &= ~B_PORT_IO_ENABLE_RX;                               /* IO[RX] as input */
#if defined (IO_PORT_IO_RX_LV) && defined (B_PORT_IO_RX_LV_ENABLE)
    IO_PORT_IO_LV |= B_PORT_IO_LV_ENABLE;                                       /* IO[RX] LV */
#endif /* defined (IO_PORT_IO_LV) && defined (B_PORT_IO_LV_ENABLE) */
} /* End of HAL_UART0_RX_Setup() */

/*!*************************************************************************** *
 * HAL_UART0_TX_Setup
 * \brief   Setup UART0 TX configuration
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Setup UART0 TX configuration by assigning the UART RX I/O-pin,
 *          in output-mode (and if applicable in LV-mode).
 *          This is the first UART interface
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static __inline__ void HAL_UART0_TX_Setup(void)
{
#if (defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) || defined (__MLX81350__)) && \
    (_SUPPORT_UART_TX_IO == PIN_FUNC_LIN)
    /* UART #0 TX on LIN */
#if defined (__MLX81350__)
    IO_PORT_LIN_XTX_CFG = (IO_PORT_LIN_XTX_CFG & ~M_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL) |
                          C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_UART_TX;           /* UART TX via LIN */
#else  /* defined (__MLX81350__) */
    IO_PORT_LIN_XTX_CFG = (IO_PORT_LIN_XTX_CFG & ~M_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL) |
                          C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_UART0_TX;          /* UART0 TX via LIN */
#endif /* defined (__MLX81350__) */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    IO_PORT_LIN_XKEY_S = C_LIN_KEY;
    IO_PORT_LIN_XCFG_S |= (B_PORT_LIN_XCFG_ENA_LIN_REV_PROT |                   /* Disconnects the reverse polarity protection from internal LIN node, is needed to measure LIN level by ADC or to run fast protocol at 5V level (PPM, CXPI) */
                           B_PORT_LIN_XCFG_SEL_RXD_ATDI |                       /* Set fast comparator used in test mode (ATDI) will be switched to the RX input (this allows protocol with higher baudrate, e.g. PPM, FASTLIN or CXPI) */
                           B_PORT_LIN_XCFG_BYPASS |                             /* Bypass the receiver for high-speed mode */
                           B_PORT_LIN_XCFG_HSM |                                /* High-speed mode (slew rate disabled) */
                           B_PORT_LIN_XCFG_SLEEPB |                             /* Disable sleep mode */
                           B_PORT_LIN_XCFG_SEL_COLIN_B |                        /* Select Colin-B */
                           B_PORT_LIN_XCFG_SEL_TX_EXT);                         /* Select TX driver from IO */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#else  /* (_SUPPORT_UART_TX_IO == PIN_FUNC_LIN) */
    IO_PORT_IO_TX_CFG = (IO_PORT_IO_TX_CFG & ~M_PORT_IO_TX_CFG_SEL) | C_PORT_IO_TX_CFG_UART;
#if (defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__))
    IO_PORT_IO_UART_SEL = (IO_PORT_IO_UART_SEL & ~C_PORT_IO_UART_SEL);          /* IO[TX] to UART0 TX */
#endif
#if (_SUPPORT_UART_TX_IO == _SUPPORT_UART_RX_IO)
    IO_PORT_IO_TX_OD |= B_PORT_IO_TX_OD;
#else  /* (_SUPPORT_UART_TX_IO == _SUPPORT_UART_RX_IO) */
#if defined (IO_PORT_IO_TX_LV) && defined (B_PORT_IO_TX_LV_ENABLE)
    IO_PORT_IO_TX_LV |= B_PORT_IO_TX_LV_ENABLE;                                 /* IO[TX] LV */
#endif /* (_SUPPORT_UART_TX_IO == _SUPPORT_UART_RX_IO) */
    IO_PORT_IO_ENABLE_TX |= B_PORT_IO_ENABLE_TX;                                /* IO[TX] as output */
#endif /* (_SUPPORT_UART_TX_IO == _SUPPORT_UART_RX_IO) */
#endif /* (_SUPPORT_UART_TX_IO == PIN_FUNC_LIN) */
} /* End of HAL_UART0_TX_Setup() */

/*!*************************************************************************** *
 * HAL_UART0_SetPrioAndEnableIRQ
 * \brief   Setup UART0 IRQ Priority and enable ADC IRQ
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Setup UART0 IRQ's Priority (PRIO 5) and enable the UART IRQ's
 *          This is the first UART interface
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static __inline__ void HAL_UART0_SetPrioAndEnableIRQ(void)
{
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */

#if (defined (__MLX81332__) || defined (__MLX81334__))
    IO_MLX16_ITC_PRIO4_S = (IO_MLX16_ITC_PRIO4_S & ~(M_MLX16_ITC_PRIO4_UART_SB
                                                     | M_MLX16_ITC_PRIO4_UART_RS
                                                     | M_MLX16_ITC_PRIO4_UART_RR
                                                     | M_MLX16_ITC_PRIO4_UART_TS)) |
                           (C_MLX16_ITC_PRIO4_UART_SB_PRIO5
                            | C_MLX16_ITC_PRIO4_UART_RS_PRIO5
                            | C_MLX16_ITC_PRIO4_UART_RR_PRIO5
                            | C_MLX16_ITC_PRIO4_UART_TS_PRIO5);
    IO_MLX16_ITC_PRIO5_S = (IO_MLX16_ITC_PRIO5_S & ~(M_MLX16_ITC_PRIO5_UART_TR
                                                     | M_MLX16_ITC_PRIO5_UART_TE
#if (_SUPPORT_UART_DMA != FALSE)
                                                     | M_MLX16_ITC_PRIO5_UDFR
                                                     | M_MLX16_ITC_PRIO5_UDFT
#endif /* (_SUPPORT_UART_DMA != FALSE) */
                                                     )) |
                           (C_MLX16_ITC_PRIO5_UART_TR_PRIO5
                            | C_MLX16_ITC_PRIO5_UART_TE_PRIO5
#if (_SUPPORT_UART_DMA != FALSE)
                            | C_MLX16_ITC_PRIO5_UDFR_PRIO5
                            | C_MLX16_ITC_PRIO5_UDFT_PRIO5
#endif /* (_SUPPORT_UART_DMA != FALSE) */
                           );
    IO_MLX16_ITC_PEND3_S = (B_MLX16_ITC_PEND3_UART_SB                           /* UART Stop Bit error */
                            | B_MLX16_ITC_PEND3_UART_RS                         /* UART Receive error */
                            | B_MLX16_ITC_PEND3_UART_RR                         /* UART Receive */
                            | B_MLX16_ITC_PEND3_UART_TS                         /* UART Transmit Error */
                            | B_MLX16_ITC_PEND3_UART_TR                         /* UART Transmit Begin */
                            | B_MLX16_ITC_PEND3_UART_TE                         /* UART Transmit End */
#if (_SUPPORT_UART_DMA != FALSE)
                            | B_MLX16_ITC_PEND3_UDFR                            /* UART DMA Frame Received */
                            | B_MLX16_ITC_PEND3_UDFT                            /* UART DMA Frame Transmitted */
#endif /* (_SUPPORT_UART_DMA != FALSE) */
                            );
#if (_SUPPORT_UART_ISR != FALSE)
    IO_MLX16_ITC_MASK3_S |= (B_MLX16_ITC_MASK3_UART_SB                          /* UART Stop Bit Error */
                             | B_MLX16_ITC_MASK3_UART_RS                        /* UART Receive Error */
#if (_SUPPORT_UART_DMA == FALSE)
                             | B_MLX16_ITC_MASK3_UART_RR                        /* UART Receive */
                             | B_MLX16_ITC_MASK3_UART_TS                        /* UART Transmit End */
                             | B_MLX16_ITC_MASK3_UART_TR                        /* UART Transmit Begin */
#elif (_SUPPORT_UART_IF_APP == C_UART_IF_SCOPE)
                             | B_MLX16_ITC_MASK3_UART_RR                        /* UART Receive */
#endif /* (_SUPPORT_UART_DMA == FALSE) */
                             | B_MLX16_ITC_MASK3_UART_TE                        /* UART Transmit Error */
#if (_SUPPORT_UART_DMA != FALSE)
                             | B_MLX16_ITC_MASK3_UDFR                           /* UART DMA Frame Received */
                             | B_MLX16_ITC_MASK3_UDFT                           /* UART DMA Frame Transmitted */
#endif /* (_SUPPORT_UART_DMA != FALSE) */
                             );
#else  /* (_SUPPORT_UART_ISR != FALSE) */
    IO_MLX16_ITC_MASK3_S &= ~(B_MLX16_ITC_MASK3_UART_SB |                       /* UART Stop Bit Error */
                              B_MLX16_ITC_MASK3_UART_RS |                       /* UART Receive Error */
                              B_MLX16_ITC_MASK3_UART_RR |                       /* UART Receive */
                              B_MLX16_ITC_MASK3_UART_TS |                       /* UART Transmit End */
                              B_MLX16_ITC_MASK3_UART_TR |                       /* UART Transmit Begin */
                              B_MLX16_ITC_MASK3_UART_TE |                       /* UART Transmit Error */
                              B_MLX16_ITC_MASK3_UDFR |                          /* UART DMA Frame Received */
                              B_MLX16_ITC_MASK3_UDFT);                          /* UART DMA Frame Transmitted */
#endif /* (_SUPPORT_UART_ISR != FALSE) */
#elif (defined (__MLX81339__) || defined (__MLX81350__))
#if defined (__MLX81339__)
    IO_MLX16_ITC_PRIO4_S = (IO_MLX16_ITC_PRIO4_S & ~(M_MLX16_ITC_PRIO4_UART0_FRM
                                                     | M_MLX16_ITC_PRIO4_UART0_EVE
                                                     | M_MLX16_ITC_PRIO4_UART0_TXE
                           )) |
                           (C_MLX16_ITC_PRIO4_UART0_FRM_PRIO5
                            | C_MLX16_ITC_PRIO4_UART0_EVE_PRIO5
                            | C_MLX16_ITC_PRIO4_UART0_TXE_PRIO5
                           );
    IO_MLX16_ITC_PRIO5_S = (IO_MLX16_ITC_PRIO5_S & ~(M_MLX16_ITC_PRIO5_UART0_RXF)) |
                           (C_MLX16_ITC_PRIO5_UART0_RXF_PRIO5);
#elif defined (__MLX81350__)
    IO_MLX16_ITC_PRIO4_S = (IO_MLX16_ITC_PRIO4_S & ~(M_MLX16_ITC_PRIO4_UART0_FRM
                                                     | M_MLX16_ITC_PRIO4_UART0_EVE
                           )) |
                           (C_MLX16_ITC_PRIO4_UART0_FRM_PRIO5
                            | C_MLX16_ITC_PRIO4_UART0_EVE_PRIO5
                           );
    IO_MLX16_ITC_PRIO5_S = (IO_MLX16_ITC_PRIO5_S & ~(M_MLX16_ITC_PRIO5_UART0_TXE
                                                     | M_MLX16_ITC_PRIO5_UART0_RXF)) |
                           (C_MLX16_ITC_PRIO5_UART0_TXE_PRIO5
                            | C_MLX16_ITC_PRIO5_UART0_RXF_PRIO5);
#endif
    IO_MLX16_ITC_PEND3_S = (B_MLX16_ITC_PEND3_UART0_FRM |                       /* UART Framing error interrupt */
                            B_MLX16_ITC_PEND3_UART0_EVE |                       /* UART Event interrupt */
                            B_MLX16_ITC_PEND3_UART0_TXE |                       /* TX empty interrupt */
                            B_MLX16_ITC_PEND3_UART0_RXF);                       /* RX full interrupt */
    IO_MLX16_ITC_MASK3_S |= (B_MLX16_ITC_MASK3_UART0_FRM |                      /* UART Framing error interrupt */
                             B_MLX16_ITC_MASK3_UART0_EVE |                      /* UART Event interrupt */
                             B_MLX16_ITC_MASK3_UART0_TXE |                      /* TX empty interrupt */
                             B_MLX16_ITC_MASK3_UART0_RXF);                      /* RX full interrupt */
#elif (defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__))
#if defined (__MLX81160__)
    IO_MLX16_ITC_PRIO4_S = (IO_MLX16_ITC_PRIO4_S & ~(M_MLX16_ITC_PRIO4_UART0_SB |
                                                     M_MLX16_ITC_PRIO4_UART0_RS |
                                                     M_MLX16_ITC_PRIO4_UART0_RR |
                                                     M_MLX16_ITC_PRIO4_UART0_TS |
                                                     M_MLX16_ITC_PRIO4_UART0_TR |
                                                     M_MLX16_ITC_PRIO4_UART0_TE)) |
                           (C_MLX16_ITC_PRIO4_UART0_SB_PRIO5 |
                            C_MLX16_ITC_PRIO4_UART0_RS_PRIO5 |
                            C_MLX16_ITC_PRIO4_UART0_RR_PRIO5 |
                            C_MLX16_ITC_PRIO4_UART0_TS_PRIO5 |
                            C_MLX16_ITC_PRIO4_UART0_TR_PRIO5 |
                            C_MLX16_ITC_PRIO4_UART0_TE_PRIO5);
#if (_SUPPORT_UART_DMA != FALSE)
    IO_MLX16_ITC_PRIO5_S = (IO_MLX16_ITC_PRIO5_S & ~(M_MLX16_ITC_PRIO5_UDFR0 |
                                                     M_MLX16_ITC_PRIO5_UDFT0)) |
                           (C_MLX16_ITC_PRIO5_UDFR0_PRIO5 |
                            C_MLX16_ITC_PRIO5_UDFT0_PRIO5);
#endif /* (_SUPPORT_UART_DMA != FALSE) */
    IO_MLX16_ITC_PEND3_S = (B_MLX16_ITC_PEND3_UART0_SB                          /* UART Stop Bit error */
                            | B_MLX16_ITC_PEND3_UART0_RS                        /* UART Receive error */
                            | B_MLX16_ITC_PEND3_UART0_RR                        /* UART Receive */
                            | B_MLX16_ITC_PEND3_UART0_TS                        /* UART Transmit Error */
                            | B_MLX16_ITC_PEND3_UART0_TR                        /* UART Transmit Begin */
                            | B_MLX16_ITC_PEND3_UART0_TE                        /* UART Transmit End */
#if (_SUPPORT_UART_DMA != FALSE)
                            | B_MLX16_ITC_PEND3_UDFR0                           /* UART DMA Frame Received */
                            | B_MLX16_ITC_PEND3_UDFT0                           /* UART DMA Frame Transmitted */
#endif /* (_SUPPORT_UART_DMA != FALSE) */
                            );
#if (_SUPPORT_UART_0_ISR != FALSE)
    IO_MLX16_ITC_MASK3_S |= (B_MLX16_ITC_MASK3_UART0_SB                         /* UART #0 Stop Bit Error */
                             | B_MLX16_ITC_MASK3_UART0_RS                       /* UART #0 Receive Error */
#if (_SUPPORT_UART_DMA == FALSE)
                             | B_MLX16_ITC_MASK3_UART0_RR                       /* UART #0 Receive */
                             | B_MLX16_ITC_MASK3_UART0_TS                       /* UART #0 Transmit End */
                             | B_MLX16_ITC_MASK3_UART0_TR                       /* UART #0 Transmit Begin */
#endif /* (_SUPPORT_UART_DMA == FALSE) */
                             | B_MLX16_ITC_MASK3_UART0_TE                       /* UART #0 Transmit Error */
#if (_SUPPORT_UART_DMA != FALSE)
                             | B_MLX16_ITC_MASK3_UDFR0                          /* UART #0 DMA Frame Received */
                             | B_MLX16_ITC_MASK3_UDFT0                          /* UART #0 DMA Frame Transmitted */
#endif /* (_SUPPORT_UART_DMA != FALSE) */
                             );
#else  /* (_SUPPORT_UART_0_ISR != FALSE) */
    IO_MLX16_ITC_MASK3_S = 0U;
#endif /* (_SUPPORT_UART_0_ISR != FALSE) */
#else  /* defined (__MLX81160__) */
    IO_MLX16_ITC_PRIO5_S = (IO_MLX16_ITC_PRIO5_S & ~(M_MLX16_ITC_PRIO5_UART0_SB |
                                                     M_MLX16_ITC_PRIO5_UART0_RS |
                                                     M_MLX16_ITC_PRIO5_UART0_RR |
                                                     M_MLX16_ITC_PRIO5_UART0_TS |
                                                     M_MLX16_ITC_PRIO5_UART0_TR)) |
                           (C_MLX16_ITC_PRIO5_UART0_SB_PRIO5 |
                            C_MLX16_ITC_PRIO5_UART0_RS_PRIO5 |
                            C_MLX16_ITC_PRIO5_UART0_RR_PRIO5 |
                            C_MLX16_ITC_PRIO5_UART0_TS_PRIO5 |
                            C_MLX16_ITC_PRIO5_UART0_TR_PRIO5);
    IO_MLX16_ITC_PRIO6_S = (IO_MLX16_ITC_PRIO6_S & ~(M_MLX16_ITC_PRIO6_UART0_TE
#if (_SUPPORT_UART_DMA != FALSE)
                                                     | M_MLX16_ITC_PRIO6_UDFR0
                                                     | M_MLX16_ITC_PRIO6_UDFT0
#endif /* (_SUPPORT_UART_DMA != FALSE) */
                                                     )) |
                           (C_MLX16_ITC_PRIO6_UART0_TE_PRIO5
#if (_SUPPORT_UART_DMA != FALSE)
                            | C_MLX16_ITC_PRIO6_UDFR0_PRIO5
                            | C_MLX16_ITC_PRIO6_UDFT0_PRIO5
#endif /* (_SUPPORT_UART_DMA != FALSE) */
                           );
    IO_MLX16_ITC_PEND4_S = (B_MLX16_ITC_PEND4_UART0_SB                          /* UART Stop Bit error */
                            | B_MLX16_ITC_PEND4_UART0_RS                        /* UART Receive error */
                            | B_MLX16_ITC_PEND4_UART0_RR                        /* UART Receive */
                            | B_MLX16_ITC_PEND4_UART0_TS                        /* UART Transmit Error */
                            | B_MLX16_ITC_PEND4_UART0_TR                        /* UART Transmit Begin */
                            | B_MLX16_ITC_PEND4_UART0_TE                        /* UART Transmit End */
#if (_SUPPORT_UART_DMA != FALSE)
                            | B_MLX16_ITC_PEND4_UDFR0                           /* UART DMA Frame Received */
                            | B_MLX16_ITC_PEND4_UDFT0                           /* UART DMA Frame Transmitted */
#endif /* (_SUPPORT_UART_DMA != FALSE) */
                            );
#if (_SUPPORT_UART_0_ISR != FALSE)
    IO_MLX16_ITC_MASK4_S |= (B_MLX16_ITC_MASK4_UART0_SB                         /* UART #0 Stop Bit Error */
                             | B_MLX16_ITC_MASK4_UART0_RS                       /* UART #0 Receive Error */
#if (_SUPPORT_UART_DMA == FALSE)
                             | B_MLX16_ITC_MASK4_UART0_RR                       /* UART #0 Receive */
                             | B_MLX16_ITC_MASK4_UART0_TS                       /* UART #0 Transmit End */
                             | B_MLX16_ITC_MASK4_UART0_TR                       /* UART #0 Transmit Begin */
#elif (_SUPPORT_UART_IF_APP == C_UART_IF_SCOPE)
                             | B_MLX16_ITC_MASK4_UART0_RR                       /* UART #0 Receive */
#endif /* (_SUPPORT_UART_DMA == FALSE) */
                             | B_MLX16_ITC_MASK4_UART0_TE                       /* UART #0 Transmit Error */
#if (_SUPPORT_UART_DMA != FALSE)
                             | B_MLX16_ITC_MASK4_UDFR0                          /* UART #0 DMA Frame Received */
                             | B_MLX16_ITC_MASK4_UDFT0                          /* UART #0 DMA Frame Transmitted */
#endif /* (_SUPPORT_UART_DMA != FALSE) */
                             );
#else  /* (_SUPPORT_UART_0_ISR != FALSE) */
    IO_MLX16_ITC_MASK4_S &= ~(B_MLX16_ITC_MASK4_UART0_SB                        /* UART #0 Stop Bit Error */
                             | B_MLX16_ITC_MASK4_UART0_RS                       /* UART #0 Receive Error */
                             | B_MLX16_ITC_MASK4_UART0_RR                       /* UART #0 Receive */
                             | B_MLX16_ITC_MASK4_UART0_TS                       /* UART #0 Transmit End */
                             | B_MLX16_ITC_MASK4_UART0_TR                       /* UART #0 Transmit Begin */
                             | B_MLX16_ITC_MASK4_UART0_TE                       /* UART #0 Transmit Error */
                             | B_MLX16_ITC_MASK4_UDFR0                          /* UART #0 DMA Frame Received */
                             | B_MLX16_ITC_MASK4_UDFT0                          /* UART #0 DMA Frame Transmitted */
                             );
#endif /* (_SUPPORT_UART_0_ISR != FALSE) */
#endif /* defined (__MLX81160__) */
#else
#error "ERROR: HAL_UART0_IRQ not supported by this IC."
#endif

#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 !e529 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
} /* End of HAL_UART0_SetPrioAndEnableIRQ() */

#if (defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__))
/*!*************************************************************************** *
 * HAL_UART1_RX_Setup
 * \brief   Setup UART1 RX configuration
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Setup UART1 RX configuration by assigning the UART RX I/O-pin,
 *          in input-mode (and if applicable in LV-mode).
 *          This is the Second UART interface
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static __inline__ void HAL_UART1_RX_Setup(void)
{
#if (_SUPPORT_UART2_RX_IO == PIN_FUNC_IO_0)
    /* UART #1 RX on IO[0] */
    IO_PORT_COMM_CFG1 = (IO_PORT_COMM_CFG1 & ~M_PORT_COMM_CFG1_UART1_RX_SEL) | C_PORT_COMM_CFG1_UART1_RX_SEL_IO0;  /* UART1_RX from IO0 */
    IO_PORT_IO_ENABLE &= ~B_PORT_IO_ENABLE_IO_ENABLE_0;                         /* IO0 as input */
    IO_PORT_IO_OUT_EN |= B_PORT_IO_OUT_EN_IO_LV_ENABLE_0;                       /* IO0 LV */
#elif (_SUPPORT_UART2_RX_IO == PIN_FUNC_IO_1)
    /* UART #1 RX on IO[0] */
    IO_PORT_COMM_CFG1 = (IO_PORT_COMM_CFG1 & ~M_PORT_COMM_CFG1_UART1_RX_SEL) | C_PORT_COMM_CFG1_UART1_RX_SEL_IO1;  /* UART1_RX from IO1 */
    IO_PORT_IO_ENABLE &= ~B_PORT_IO_ENABLE_IO_ENABLE_1;                         /* IO1 as input */
    IO_PORT_IO_OUT_EN |= B_PORT_IO_OUT_EN_IO_LV_ENABLE_1;                       /* IO1 LV */
#elif (_SUPPORT_UART2_RX_IO == PIN_FUNC_IO_2)
    /* UART #1 RX on IO[0] */
    IO_PORT_COMM_CFG1 = (IO_PORT_COMM_CFG1 & ~M_PORT_COMM_CFG1_UART1_RX_SEL) | C_PORT_COMM_CFG1_UART1_RX_SEL_IO2;  /* UART1_RX from IO2 */
    IO_PORT_IO_ENABLE &= ~B_PORT_IO_ENABLE_IO_ENABLE_2;                         /* IO2 as input */
    IO_PORT_IO_OUT_EN |= B_PORT_IO_OUT_EN_IO_LV_ENABLE_2;                       /* IO2 LV */
#elif (_SUPPORT_UART2_RX_IO == PIN_FUNC_IO_3)
    /* UART #1 RX on IO[0] */
    IO_PORT_COMM_CFG1 = (IO_PORT_COMM_CFG1 & ~M_PORT_COMM_CFG1_UART1_RX_SEL) | C_PORT_COMM_CFG1_UART1_RX_SEL_IO3;  /* UART1_RX from IO3 */
    IO_PORT_IO_ENABLE &= ~B_PORT_IO_ENABLE_IO_ENABLE_3;                         /* IO3 as input */
#elif (_SUPPORT_UART2_RX_IO == PIN_FUNC_IO_4)
    /* UART #1 RX on IO[4] */
    IO_PORT_COMM_CFG1 = (IO_PORT_COMM_CFG1 & ~M_PORT_COMM_CFG1_UART1_RX_SEL) | C_PORT_COMM_CFG1_UART1_RX_SEL_IO4;
    IO_PORT_IO_ENABLE &= ~B_PORT_IO_ENABLE_IO_ENABLE_4;
#elif (_SUPPORT_UART_RX_IO == PIN_FUNC_IO_5)
    /* UART #1 RX on IO[5] */
    IO_PORT_COMM_CFG1 = (IO_PORT_COMM_CFG1 & ~M_PORT_COMM_CFG1_UART1_RX_SEL) | C_PORT_COMM_CFG1_UART1_RX_SEL_IO5;
    IO_PORT_IO_ENABLE &= ~B_PORT_IO_ENABLE_IO_ENABLE_5;
#elif (_SUPPORT_UART_RX_IO == PIN_FUNC_IO_6)
    /* UART #1 RX on IO[6] */
    IO_PORT_COMM_CFG1 = (IO_PORT_COMM_CFG1 & ~M_PORT_COMM_CFG1_UART1_RX_SEL) | C_PORT_COMM_CFG1_UART1_RX_SEL_IO6;
    IO_PORT_IO_ENABLE &= ~B_PORT_IO_ENABLE_IO_ENABLE_6;
#elif (_SUPPORT_UART_RX_IO == PIN_FUNC_IO_7)
    /* UART #1 RX on IO[7] */
    IO_PORT_COMM_CFG1 = (IO_PORT_COMM_CFG1 & ~M_PORT_COMM_CFG1_UART1_RX_SEL) | C_PORT_COMM_CFG1_UART1_RX_SEL_IO7;
    IO_PORT_IO_ENABLE &= ~B_PORT_IO_ENABLE_IO_ENABLE_7;
#elif (_SUPPORT_UART_RX_IO == PIN_FUNC_IO_8)
    /* UART #1 RX on IO[8] */
    IO_PORT_COMM_CFG1 = (IO_PORT_COMM_CFG1 & ~M_PORT_COMM_CFG1_UART1_RX_SEL) | C_PORT_COMM_CFG1_UART1_RX_SEL_IO8;
    IO_PORT_IO_ENABLE1 &= ~B_PORT_IO_ENABLE1_IO_ENABLE_8;
#elif (_SUPPORT_UART_RX_IO == PIN_FUNC_IO_9)
    /* UART #1 RX on IO[9] */
    IO_PORT_COMM_CFG1 = (IO_PORT_COMM_CFG1 & ~M_PORT_COMM_CFG1_UART1_RX_SEL) | C_PORT_COMM_CFG1_UART1_RX_SEL_IO9;
    IO_PORT_IO_ENABLE1 &= ~B_PORT_IO_ENABLE1_IO_ENABLE_9;
#elif (_SUPPORT_UART_RX_IO == PIN_FUNC_IO_10)
    /* UART #1 RX on IO[10] */
    IO_PORT_COMM_CFG1 = (IO_PORT_COMM_CFG1 & ~M_PORT_COMM_CFG1_UART1_RX_SEL) | C_PORT_COMM_CFG1_UART1_RX_SEL_IO10;
    IO_PORT_IO_ENABLE1 &= ~B_PORT_IO_ENABLE1_IO_ENABLE_10;
#elif (_SUPPORT_UART_RX_IO == PIN_FUNC_IO_11)
    /* UART #1 RX on IO[11] */
    IO_PORT_COMM_CFG1 = (IO_PORT_COMM_CFG1 & ~M_PORT_COMM_CFG1_UART1_RX_SEL) | C_PORT_COMM_CFG1_UART1_RX_SEL_IO11;
    IO_PORT_IO_ENABLE1 &= ~B_PORT_IO_ENABLE1_IO_ENABLE_11;
#else  /* _SUPPORT_UART_RX_IO */
#error "Undefined UART RX IO"
#endif /* _SUPPORT_UART_RX_IO */
} /* End of HAL_UART1_RX_Setup() */

/*!*************************************************************************** *
 * HAL_UART1_TX_Setup
 * \brief   Setup UART1 TX configuration
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Setup UART0 TX configuration by assigning the UART RX I/O-pin,
 *          in output-mode (and if applicable in LV-mode).
 *          This is the second UART interface
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static __inline__ void HAL_UART1_TX_Setup(void)
{
#if (_SUPPORT_UART2_TX_IO == PIN_FUNC_IO_0)
    /* UART #0 TX on IO[0] */
    IO_PORT_IO_CFG0 = (IO_PORT_IO_CFG0 & ~M_PORT_IO_CFG0_IO0_OUT_SEL) | C_PORT_IO_CFG0_IO0_OUT_SEL_UART;  /* IO0 from UARTx-TX */
    IO_PORT_IO_ENABLE |= B_PORT_IO_ENABLE_IO_ENABLE_0;                          /* IO0 as output */
    IO_PORT_IO_UART_SEL = (IO_PORT_IO_UART_SEL | C_PORT_IO_UART_SEL_IO0);       /* UART1 TX to IO0 */
    IO_PORT_IO_OUT_EN |= B_PORT_IO_OUT_EN_IO_LV_ENABLE_0;                       /* IO0 LV */
#elif (_SUPPORT_UART2_TX_IO == PIN_FUNC_IO_1)
    /* UART #0 TX on IO[1] */
    IO_PORT_IO_CFG0 = (IO_PORT_IO_CFG0 & ~M_PORT_IO_CFG0_IO1_OUT_SEL) | C_PORT_IO_CFG0_IO1_OUT_SEL_UART;  /* IO1 from UARTx-TX */
    IO_PORT_IO_ENABLE |= B_PORT_IO_ENABLE_IO_ENABLE_1;                          /* IO1 as output */
    IO_PORT_IO_UART_SEL = (IO_PORT_IO_UART_SEL | C_PORT_IO_UART_SEL_IO1);       /* UART1 TX to IO1 */
    IO_PORT_IO_OUT_EN |= B_PORT_IO_OUT_EN_IO_LV_ENABLE_1;                       /* IO1 LV */
#elif (_SUPPORT_UART2_TX_IO == PIN_FUNC_IO_2)
    /* UART #0 TX on IO[2] */
    IO_PORT_IO_CFG0 = (IO_PORT_IO_CFG0 & ~M_PORT_IO_CFG0_IO2_OUT_SEL) | C_PORT_IO_CFG0_IO2_OUT_SEL_UART;  /* IO2 from UARTx-TX */
    IO_PORT_IO_ENABLE |= B_PORT_IO_ENABLE_IO_ENABLE_2;                          /* IO2 as output */
    IO_PORT_IO_UART_SEL = (IO_PORT_IO_UART_SEL | C_PORT_IO_UART_SEL_IO2);       /* UART1 TX to IO2 */
    IO_PORT_IO_OUT_EN |= B_PORT_IO_OUT_EN_IO_LV_ENABLE_2;                       /* IO2 LV */
#elif (_SUPPORT_UART2_TX_IO == PIN_FUNC_IO_3)
    /* UART #0 TX on IO[3] */
    IO_PORT_IO_CFG0 = (IO_PORT_IO_CFG0 & ~M_PORT_IO_CFG0_IO3_OUT_SEL) | C_PORT_IO_CFG0_IO3_OUT_SEL_UART;  /* IO3 from UARTx-TX */
    IO_PORT_IO_ENABLE |= B_PORT_IO_ENABLE_IO_ENABLE_3;                          /* IO3 as output */
    IO_PORT_IO_UART_SEL = (IO_PORT_IO_UART_SEL | C_PORT_IO_UART_SEL_IO3);       /* UART1 TX to IO3 */
#elif (_SUPPORT_UART2_TX_IO == PIN_FUNC_IO_4)
    /* UART #1 TX on IO[4] */
    IO_PORT_IO_CFG1 = (IO_PORT_IO_CFG1 & ~M_PORT_IO_CFG1_IO4_OUT_SEL) | C_PORT_IO_CFG1_IO4_OUT_SEL_UART;
    IO_PORT_IO_ENABLE |= B_PORT_IO_ENABLE_IO_ENABLE_4;
    IO_PORT_IO_UART_SEL = (IO_PORT_IO_UART_SEL | C_PORT_IO_UART_SEL_IO4);
#elif (_SUPPORT_UART_TX_IO == PIN_FUNC_IO_5)
    /* UART #1 TX on IO[5] */
    IO_PORT_IO_CFG1 = (IO_PORT_IO_CFG1 & ~M_PORT_IO_CFG1_IO5_OUT_SEL) | C_PORT_IO_CFG1_IO5_OUT_SEL_UART;
    IO_PORT_IO_ENABLE |= B_PORT_IO_ENABLE_IO_ENABLE_5;
    IO_PORT_IO_UART_SEL = (IO_PORT_IO_UART_SEL | C_PORT_IO_UART_SEL_IO5);
#elif (_SUPPORT_UART_TX_IO == PIN_FUNC_IO_6)
    /* UART #1 TX on IO[6] */
    IO_PORT_IO_CFG1 = (IO_PORT_IO_CFG1 & ~M_PORT_IO_CFG1_IO6_OUT_SEL) | C_PORT_IO_CFG1_IO6_OUT_SEL_UART;
    IO_PORT_IO_ENABLE |= B_PORT_IO_ENABLE_IO_ENABLE_6;
    IO_PORT_IO_UART_SEL = (IO_PORT_IO_UART_SEL | C_PORT_IO_UART_SEL_IO6);
#elif (_SUPPORT_UART_TX_IO == PIN_FUNC_IO_7)
    /* UART #1 TX on IO[7] */
    IO_PORT_IO_CFG1 = (IO_PORT_IO_CFG1 & ~M_PORT_IO_CFG1_IO7_OUT_SEL) | C_PORT_IO_CFG1_IO7_OUT_SEL_UART;
    IO_PORT_IO_ENABLE |= B_PORT_IO_ENABLE_IO_ENABLE_7;
    IO_PORT_IO_UART_SEL = (IO_PORT_IO_UART_SEL | C_PORT_IO_UART_SEL_IO7);
#elif (_SUPPORT_UART_TX_IO == PIN_FUNC_IO_8)
    /* UART #1 TX on IO[8] */
    IO_PORT_IO_CFG2 = (IO_PORT_IO_CFG2 & ~M_PORT_IO_CFG2_IO8_OUT_SEL) | C_PORT_IO_CFG2_IO8_OUT_SEL_UART;
    IO_PORT_IO_ENABLE1 |= B_PORT_IO_ENABLE1_IO_ENABLE_8;
    IO_PORT_IO_UART_SEL = (IO_PORT_IO_UART_SEL | C_PORT_IO_UART_SEL_IO8);
#elif (_SUPPORT_UART_TX_IO == PIN_FUNC_IO_9)
    /* UART #1 TX on IO[9] */
    IO_PORT_IO_CFG2 = (IO_PORT_IO_CFG2 & ~M_PORT_IO_CFG2_IO9_OUT_SEL) | C_PORT_IO_CFG2_IO9_OUT_SEL_UART;
    IO_PORT_IO_ENABLE1 |= B_PORT_IO_ENABLE1_IO_ENABLE_9;
    IO_PORT_IO_UART_SEL = (IO_PORT_IO_UART_SEL | C_PORT_IO_UART_SEL_IO9);
#elif (_SUPPORT_UART_TX_IO == PIN_FUNC_IO_10)
    /* UART #1 TX on IO[10] */
    IO_PORT_IO_CFG2 = (IO_PORT_IO_CFG2 & ~M_PORT_IO_CFG2_IO10_OUT_SEL) | C_PORT_IO_CFG2_IO10_OUT_SEL_UART;
    IO_PORT_IO_ENABLE1 |= B_PORT_IO_ENABLE1_IO_ENABLE_10;
    IO_PORT_IO_UART_SEL = (IO_PORT_IO_UART_SEL | C_PORT_IO_UART_SEL_IO10);
#elif (_SUPPORT_UART_TX_IO == PIN_FUNC_IO_11)
    /* UART #1 TX on IO[11] */
    IO_PORT_IO_CFG2 = (IO_PORT_IO_CFG2 & ~M_PORT_IO_CFG2_IO11_OUT_SEL) | C_PORT_IO_CFG2_IO11_OUT_SEL_UART;
    IO_PORT_IO_ENABLE1 |= B_PORT_IO_ENABLE1_IO_ENABLE_11;
    IO_PORT_IO_UART_SEL = (IO_PORT_IO_UART_SEL | C_PORT_IO_UART_SEL_IO11);
#else  /* _SUPPORT_UART_TX_IO */
#error "Undefined UART TX IO"
#endif /* _SUPPORT_UART_TX_IO */
} /* End of HAL_UART1_TX_Setup() */

/*!*************************************************************************** *
 * HAL_UART1_SetPrioAndEnableIRQ
 * \brief   Setup UART1 IRQ Priority and enable ADC IRQ
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Setup UART1 IRQ's Priority (PRIO 5) and enable the UART IRQ's
 *          This is the second UART interface
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static __inline__ void HAL_UART1_SetPrioAndEnableIRQ(void)
{
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */

    IO_MLX16_ITC_PRIO6_S = (IO_MLX16_ITC_PRIO6_S & ~(M_MLX16_ITC_PRIO6_UART1_SB |
                                                     M_MLX16_ITC_PRIO6_UART1_RS |
                                                     M_MLX16_ITC_PRIO6_UART1_RR |
                                                     M_MLX16_ITC_PRIO6_UART1_TS |
                                                     M_MLX16_ITC_PRIO6_UART1_TR)) |
                           (C_MLX16_ITC_PRIO6_UART1_SB_PRIO5 |
                            C_MLX16_ITC_PRIO6_UART1_RS_PRIO5 |
                            C_MLX16_ITC_PRIO6_UART1_RR_PRIO5 |
                            C_MLX16_ITC_PRIO6_UART1_TS_PRIO5 |
                            C_MLX16_ITC_PRIO6_UART1_TR_PRIO5);
    IO_MLX16_ITC_PRIO7_S = (IO_MLX16_ITC_PRIO7_S & ~(M_MLX16_ITC_PRIO7_UART1_TE
#if (_SUPPORT_UART2_DMA != FALSE)
                                                     | M_MLX16_ITC_PRIO7_UDFR1
                                                     | M_MLX16_ITC_PRIO7_UDFT1
#endif /* (_SUPPORT_UART2_DMA != FALSE) */
                                                     )) |
                           (C_MLX16_ITC_PRIO7_UART1_TE_PRIO5
#if (_SUPPORT_UART2_DMA != FALSE)
                            | C_MLX16_ITC_PRIO7_UDFR1_PRIO5
                            | C_MLX16_ITC_PRIO7_UDFT1_PRIO5
#endif /* (_SUPPORT_UART2_DMA != FALSE) */
                           );
    /* Clear UART1 pend-flags (WC) */
    IO_MLX16_ITC_PEND4_S = (B_MLX16_ITC_PEND4_UART1_SB                          /* UART Stop Bit error */
                            | B_MLX16_ITC_PEND4_UART1_RS                        /* UART Receive error */
                            | B_MLX16_ITC_PEND4_UART1_RR                        /* UART Receive */
                            | B_MLX16_ITC_PEND4_UART1_TS                        /* UART Transmit Error */
                            | B_MLX16_ITC_PEND4_UART1_TR                        /* UART Transmit Begin */
                            | B_MLX16_ITC_PEND4_UART1_TE                        /* UART Transmit End */
#if (_SUPPORT_UART2_DMA != FALSE)
                            | B_MLX16_ITC_PEND4_UDFR1                           /* UART DMA Frame Received */
                            | B_MLX16_ITC_PEND4_UDFT1                           /* UART DMA Frame Transmitted */
#endif /* (_SUPPORT_UART2_DMA != FALSE) */
                            );
#if (_SUPPORT_UART2_ISR != FALSE)
    IO_MLX16_ITC_MASK4_S |= (B_MLX16_ITC_MASK4_UART1_SB |                       /* UART #1 Stop Bit Error */
                             B_MLX16_ITC_MASK4_UART1_RS |                       /* UART #1 Receive Error */
#if (_SUPPORT_UART2_DMA == FALSE)
                             B_MLX16_ITC_MASK4_UART1_RR |                       /* UART #1 Receive */
                             B_MLX16_ITC_MASK4_UART1_TS |                       /* UART #1 Transmit End */
                             B_MLX16_ITC_MASK4_UART1_TR |                       /* UART #1 Transmit Begin */
#elif (_SUPPORT_UART2_IF_APP == C_UART_IF_SCOPE)
                             B_MLX16_ITC_MASK4_UART1_RR |                       /* UART #1 Receive */
#endif /* (_SUPPORT_UART2_DMA == FALSE) */
                             B_MLX16_ITC_MASK4_UART1_TE |                       /* UART #1 Transmit Error */
                             B_MLX16_ITC_MASK4_UDFR1 |                          /* UART #1 DMA Frame Received */
                             B_MLX16_ITC_MASK4_UDFT1);                          /* UART #1 DMA Frame Transmitted */
#else  /* (_SUPPORT_UART2_ISR != FALSE) */
    IO_MLX16_ITC_MASK4_S &= ~(B_MLX16_ITC_MASK4_UART1_SB |                      /* UART #1 Stop Bit Error */
                              B_MLX16_ITC_MASK4_UART1_RS |                      /* UART #1 Receive Error */
                              B_MLX16_ITC_MASK4_UART1_RR |                      /* UART #1 Receive */
                              B_MLX16_ITC_MASK4_UART1_TS |                      /* UART #1 Transmit End */
                              B_MLX16_ITC_MASK4_UART1_TR |                      /* UART #1 Transmit Begin */
                              B_MLX16_ITC_MASK4_UART1_TE |                      /* UART #1 Transmit Error */
                              B_MLX16_ITC_MASK4_UDFR1 |                         /* UART #1 DMA Frame Received */
                              B_MLX16_ITC_MASK4_UDFT1);                         /* UART #1 DMA Frame Transmitted */
#endif /* (_SUPPORT_UART2_ISR != FALSE) */

#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
} /* End of HAL_UART1_SetPrioAndEnableIRQ() */
#endif /* (defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)) */

#endif /* (_SUPPORT_UART != FALSE) */

#endif /* HAL_LIB_UART_INLINE_H */

/* EOF */

