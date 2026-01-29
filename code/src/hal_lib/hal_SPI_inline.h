/*!************************************************************************** *
 * \file        HAL_SPI_inline.h
 * \brief       Hardware Abstraction Layer for SPI handling (inline)
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
 *           -# HAL_SPI_IO_Setup()
 *           -# HAL_SPI_SetPrioAndEnableIRQ()
 *           -# HAL_SPI_SS_Active()
 *           -# HAL_SPI_SS_Deactive()
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

#ifndef HAL_LIB_SPI_INLINE_H
#define HAL_LIB_SPI_INLINE_H

/*!************************************************************************** */
/*                          INCLUDES                                          */
/* ************************************************************************** */
#include "../AppBuild.h"                                                        /* Application Build */

#if (_SUPPORT_SPI != FALSE)

#include <atomic.h>

/*!*************************************************************************** */
/*                           DEFINITIONS                                       */
/* *************************************************************************** */
#if defined (__MLX81160__)
#define C_PORT_IO_CFG0_IO0_OUT_SEL_SPI_MOSI     C_PORT_IO_CFG0_IO0_OUT_SEL_SPI_MxSx  /* IO[0]: SPI_MOSI */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_SPI_MISO     C_PORT_IO_CFG0_IO0_OUT_SEL_SPI_MxSx  /* IO[0]: SPI_MISO */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_SPI_MOSI     C_PORT_IO_CFG0_IO1_OUT_SEL_SPI_MxSx  /* IO[1]: SPI_MOSI */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_SPI_MISO     C_PORT_IO_CFG0_IO1_OUT_SEL_SPI_MxSx  /* IO[1]: SPI_MISO */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_SPI_MOSI     C_PORT_IO_CFG0_IO2_OUT_SEL_SPI_MxSx  /* IO[2]: SPI_MOSI */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_SPI_MISO     C_PORT_IO_CFG0_IO2_OUT_SEL_SPI_MxSx  /* IO[2]: SPI_MISO */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_SPI_MOSI     C_PORT_IO_CFG0_IO3_OUT_SEL_SPI_MxSx  /* IO[3]: SPI_MOSI */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_SPI_MISO     C_PORT_IO_CFG0_IO3_OUT_SEL_SPI_MxSx  /* IO[3]: SPI_MISO */
#define C_PORT_IO_CFG1_IO4_OUT_SEL_SPI_MOSI     C_PORT_IO_CFG1_IO4_OUT_SEL_SPI_MxSx  /* IO[4]: SPI_MOSI */
#define C_PORT_IO_CFG1_IO4_OUT_SEL_SPI_MISO     C_PORT_IO_CFG1_IO4_OUT_SEL_SPI_MxSx  /* IO[4]: SPI_MISO */
#define C_PORT_IO_CFG1_IO5_OUT_SEL_SPI_MOSI     C_PORT_IO_CFG1_IO5_OUT_SEL_SPI_MxSx  /* IO[5]: SPI_MOSI */
#define C_PORT_IO_CFG1_IO5_OUT_SEL_SPI_MISO     C_PORT_IO_CFG1_IO5_OUT_SEL_SPI_MxSx  /* IO[5]: SPI_MISO */
#define C_PORT_IO_CFG1_IO6_OUT_SEL_SPI_MOSI     C_PORT_IO_CFG1_IO6_OUT_SEL_SPI_MxSx  /* IO[6]: SPI_MOSI */
#define C_PORT_IO_CFG1_IO6_OUT_SEL_SPI_MISO     C_PORT_IO_CFG1_IO6_OUT_SEL_SPI_MxSx  /* IO[6]: SPI_MISO */
#define C_PORT_IO_CFG1_IO7_OUT_SEL_SPI_MOSI     C_PORT_IO_CFG1_IO7_OUT_SEL_SPI_MxSx  /* IO[7]: SPI_MOSI */
#define C_PORT_IO_CFG1_IO7_OUT_SEL_SPI_MISO     C_PORT_IO_CFG1_IO7_OUT_SEL_SPI_MxSx  /* IO[7]: SPI_MISO */
#define C_PORT_IO_CFG2_IO8_OUT_SEL_SPI_MOSI     C_PORT_IO_CFG2_IO8_OUT_SEL_SPI_MxSx  /* IO[8]: SPI_MOSI */
#define C_PORT_IO_CFG2_IO8_OUT_SEL_SPI_MISO     C_PORT_IO_CFG2_IO8_OUT_SEL_SPI_MxSx  /* IO[8]: SPI_MISO */
#define C_PORT_IO_CFG2_IO9_OUT_SEL_SPI_MOSI     C_PORT_IO_CFG2_IO9_OUT_SEL_SPI_MxSx  /* IO[9]: SPI_MOSI */
#define C_PORT_IO_CFG2_IO9_OUT_SEL_SPI_MISO     C_PORT_IO_CFG2_IO9_OUT_SEL_SPI_MxSx  /* IO[9]: SPI_MISO */
#define C_PORT_IO_CFG2_IO10_OUT_SEL_SPI_MOSI    C_PORT_IO_CFG2_IO10_OUT_SEL_SPI_MxSx  /* IO[10]: SPI_MOSI */
#define C_PORT_IO_CFG2_IO10_OUT_SEL_SPI_MISO    C_PORT_IO_CFG2_IO10_OUT_SEL_SPI_MxSx  /* IO[10]: SPI_MISO */
#define C_PORT_IO_CFG2_IO11_OUT_SEL_SPI_MOSI    C_PORT_IO_CFG2_IO11_OUT_SEL_SPI_MxSx  /* IO[11]: SPI_MOSI */
#define C_PORT_IO_CFG2_IO11_OUT_SEL_SPI_MISO    C_PORT_IO_CFG2_IO11_OUT_SEL_SPI_MxSx  /* IO[11]: SPI_MISO */
#endif /* defined (__MLX81160__) */

/*!*************************************************************************** *
 * HAL_SPI_IO_Setup
 * \brief   Setup SPI to I/O
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Setup the SPI to be connected to the I/O's
 *          1. Setup SPI module to I/O
 *          2. Setup I/O module to SPI
 *          3. Setup used SPI-I/O's in LV-mode
 *          4. Setup used SPI-I/O's in input or output mode, dependent on the
 *             SPI-mode (Master or Slave)
 *             SPI  | MASTER | SLAVE
 *             -----+--------+-------
 *             MOSI | Output | Input
 *             MISO | Input  | Output
 *             SLK  | Output | Input
 *             SS   | Output | Input
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static __inline__ void HAL_SPI_IO_Setup(void)
{
    /* Setup SPI to be connected to IO's */
    IO_PORT_COMM_CFG = (
#if (_SUPPORT_SPI_MOSI_IO == PIN_FUNC_IO_0)
        C_PORT_COMM_CFG_SPI_MOSI_IN_SEL_IO0 |                                   /* SPI MOSI: IO[0] */
#elif (_SUPPORT_SPI_MOSI_IO == PIN_FUNC_IO_1)
        C_PORT_COMM_CFG_SPI_MOSI_IN_SEL_IO1 |                                   /* SPI MOSI: IO[1] */
#elif (_SUPPORT_SPI_MOSI_IO == PIN_FUNC_IO_2)
        C_PORT_COMM_CFG_SPI_MOSI_IN_SEL_IO2 |                                   /* SPI MOSI: IO[2] */
#elif (_SUPPORT_SPI_MOSI_IO == PIN_FUNC_IO_3)
        C_PORT_COMM_CFG_SPI_MOSI_IN_SEL_IO3 |                                   /* SPI MOSI: IO[3] */
#elif defined(PIN_FUNC_IO_4) && (_SUPPORT_SPI_MOSI_IO == PIN_FUNC_IO_4)
        C_PORT_COMM_CFG_SPI_MOSI_IN_SEL_IO4 |                                   /* SPI MOSI: IO[4] */
#elif defined(PIN_FUNC_IO_5) && (_SUPPORT_SPI_MOSI_IO == PIN_FUNC_IO_5)
        C_PORT_COMM_CFG_SPI_MOSI_IN_SEL_IO5 |                                   /* SPI MOSI: IO[5] */
#elif defined(PIN_FUNC_IO_6) && (_SUPPORT_SPI_MOSI_IO == PIN_FUNC_IO_6)
        C_PORT_COMM_CFG_SPI_MOSI_IN_SEL_IO6 |                                   /* SPI MOSI: IO[6] */
#elif defined(PIN_FUNC_IO_7) && (_SUPPORT_SPI_MOSI_IO == PIN_FUNC_IO_7)
        C_PORT_COMM_CFG_SPI_MOSI_IN_SEL_IO7 |                                   /* SPI MOSI: IO[7] */
#elif defined(PIN_FUNC_IO_8) && (_SUPPORT_SPI_MOSI_IO == PIN_FUNC_IO_8)
        C_PORT_COMM_CFG_SPI_MOSI_IN_SEL_IO8 |                                   /* SPI MOSI: IO[8] */
#elif defined(PIN_FUNC_IO_9) && (_SUPPORT_SPI_MOSI_IO == PIN_FUNC_IO_9)
        C_PORT_COMM_CFG_SPI_MOSI_IN_SEL_IO9 |                                   /* SPI MOSI: IO[9] */
#elif defined(PIN_FUNC_IO_10) && (_SUPPORT_SPI_MOSI_IO == PIN_FUNC_IO_10)
        C_PORT_COMM_CFG_SPI_MOSI_IN_SEL_IO10 |                                  /* SPI MOSI: IO[10] */
#elif defined(PIN_FUNC_IO_11) && (_SUPPORT_SPI_MOSI_IO == PIN_FUNC_IO_11)
        C_PORT_COMM_CFG_SPI_MOSI_IN_SEL_IO11 |                                  /* SPI MOSI: IO[11] */
#else
#error "ERROR: SPI MOSI not configured"
#endif /* _SUPPORT_SPI_MOSI_IO */
#if (_SUPPORT_SPI_MISO_IO == PIN_FUNC_IO_0)
        C_PORT_COMM_CFG_SPI_MISO_IN_SEL_IO0 |                                   /* SPI MISO: IO[0] */
#elif (_SUPPORT_SPI_MISO_IO == PIN_FUNC_IO_1)
        C_PORT_COMM_CFG_SPI_MISO_IN_SEL_IO1 |                                   /* SPI MISO: IO[1] */
#elif (_SUPPORT_SPI_MISO_IO == PIN_FUNC_IO_2)
        C_PORT_COMM_CFG_SPI_MISO_IN_SEL_IO2 |                                   /* SPI MISO: IO[2] */
#elif (_SUPPORT_SPI_MISO_IO == PIN_FUNC_IO_3)
        C_PORT_COMM_CFG_SPI_MISO_IN_SEL_IO3 |                                   /* SPI MISO: IO[3] */
#elif defined(PIN_FUNC_IO_4) && (_SUPPORT_SPI_MISO_IO == PIN_FUNC_IO_4)
        C_PORT_COMM_CFG_SPI_MISO_IN_SEL_IO4 |                                   /* SPI MISO: IO[4] */
#elif defined(PIN_FUNC_IO_5) && (_SUPPORT_SPI_MISO_IO == PIN_FUNC_IO_5)
        C_PORT_COMM_CFG_SPI_MISO_IN_SEL_IO5 |                                   /* SPI MISO: IO[5] */
#elif defined(PIN_FUNC_IO_6) && (_SUPPORT_SPI_MISO_IO == PIN_FUNC_IO_6)
        C_PORT_COMM_CFG_SPI_MISO_IN_SEL_IO6 |                                   /* SPI MISO: IO[6] */
#elif defined(PIN_FUNC_IO_7) && (_SUPPORT_SPI_MISO_IO == PIN_FUNC_IO_7)
        C_PORT_COMM_CFG_SPI_MISO_IN_SEL_IO7 |                                   /* SPI MISO: IO[7] */
#elif defined(PIN_FUNC_IO_8) && (_SUPPORT_SPI_MISO_IO == PIN_FUNC_IO_8)
        C_PORT_COMM_CFG_SPI_MISO_IN_SEL_IO8 |                                   /* SPI MISO: IO[8] */
#elif defined(PIN_FUNC_IO_9) && (_SUPPORT_SPI_MISO_IO == PIN_FUNC_IO_9)
        C_PORT_COMM_CFG_SPI_MISO_IN_SEL_IO9 |                                   /* SPI MISO: IO[9] */
#elif defined(PIN_FUNC_IO_10) && (_SUPPORT_SPI_MISO_IO == PIN_FUNC_IO_10)
        C_PORT_COMM_CFG_SPI_MISO_IN_SEL_IO10 |                                  /* SPI MISO: IO[10] */
#elif defined(PIN_FUNC_IO_11) && (_SUPPORT_SPI_MISO_IO == PIN_FUNC_IO_11)
        C_PORT_COMM_CFG_SPI_MISO_IN_SEL_IO11 |                                  /* SPI MISO: IO[11] */
#else
#error "ERROR: SPI MISO not configured"
#endif /* _SUPPORT_SPI_MISO_IO */
#if (_SUPPORT_SPI_CLK == PIN_FUNC_IO_0)
        C_PORT_COMM_CFG_SPI_SCK_IN_SEL_IO0 |                                    /* SPI Clock: IO[0] */
#elif (_SUPPORT_SPI_CLK == PIN_FUNC_IO_1)
        C_PORT_COMM_CFG_SPI_SCK_IN_SEL_IO1 |                                    /* SPI Clock: IO[1] */
#elif (_SUPPORT_SPI_CLK == PIN_FUNC_IO_2)
        C_PORT_COMM_CFG_SPI_SCK_IN_SEL_IO2 |                                    /* SPI Clock: IO[2] */
#elif (_SUPPORT_SPI_CLK == PIN_FUNC_IO_3)
        C_PORT_COMM_CFG_SPI_SCK_IN_SEL_IO3 |                                    /* SPI Clock: IO[3] */
#elif defined(PIN_FUNC_IO_4) && (_SUPPORT_SPI_CLK == PIN_FUNC_IO_4)
        C_PORT_COMM_CFG_SPI_SCK_IN_SEL_IO4 |                                    /* SPI Clock: IO[4] */
#elif defined(PIN_FUNC_IO_5) && (_SUPPORT_SPI_CLK == PIN_FUNC_IO_5)
        C_PORT_COMM_CFG_SPI_SCK_IN_SEL_IO5 |                                    /* SPI Clock: IO[5] */
#elif defined(PIN_FUNC_IO_6) && (_SUPPORT_SPI_CLK == PIN_FUNC_IO_6)
        C_PORT_COMM_CFG_SPI_SCK_IN_SEL_IO6 |                                    /* SPI Clock: IO[6] */
#elif defined(PIN_FUNC_IO_7) && (_SUPPORT_SPI_CLK == PIN_FUNC_IO_7)
        C_PORT_COMM_CFG_SPI_SCK_IN_SEL_IO7 |                                    /* SPI Clock: IO[7] */
#else
#error "ERROR: SPI Clock not configured"
#endif /* _SUPPORT_SPI_CLK */
#if (_SUPPORT_SPI_SS == PIN_FUNC_IO_0)
        C_PORT_COMM_CFG_SPI_SS_IN_SEL_IO0                                       /* SPI Slave Select: IO[0] */
#elif (_SUPPORT_SPI_SS == PIN_FUNC_IO_1)
        C_PORT_COMM_CFG_SPI_SS_IN_SEL_IO1                                       /* SPI Slave Select: IO[1] */
#elif (_SUPPORT_SPI_SS == PIN_FUNC_IO_2)
        C_PORT_COMM_CFG_SPI_SS_IN_SEL_IO2                                       /* SPI Slave Select: IO[2] */
#elif (_SUPPORT_SPI_SS == PIN_FUNC_IO_3)
        C_PORT_COMM_CFG_SPI_SS_IN_SEL_IO3                                       /* SPI Slave Select: IO[3] */
#elif defined(PIN_FUNC_IO_4) && (_SUPPORT_SPI_SS == PIN_FUNC_IO_4)
        C_PORT_COMM_CFG_SPI_SS_IN_SEL_IO4                                       /* SPI Slave Select: IO[4] */
#elif defined(PIN_FUNC_IO_5) && (_SUPPORT_SPI_SS == PIN_FUNC_IO_5)
        C_PORT_COMM_CFG_SPI_SS_IN_SEL_IO5                                       /* SPI Slave Select: IO[5] */
#elif defined(PIN_FUNC_IO_6) && (_SUPPORT_SPI_SS == PIN_FUNC_IO_6)
        C_PORT_COMM_CFG_SPI_SS_IN_SEL_IO6                                       /* SPI Slave Select: IO[6] */
#elif defined(PIN_FUNC_IO_7) && (_SUPPORT_SPI_SS == PIN_FUNC_IO_7)
        C_PORT_COMM_CFG_SPI_SS_IN_SEL_IO7                                       /* SPI Slave Select: IO[7] */
#else
#error "ERROR: SPI Slave Select not configured"
#endif /* _SUPPORT_SPI_CLK */
                       );

    /* Configure the I/O's to be connected to SPI */
    IO_PORT_IO_CFG0 = ( (IO_PORT_IO_CFG0 & ~(0
#if (PIN_FUNC_IO_0 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_0 == _SUPPORT_SPI_MISO_IO) || \
    (PIN_FUNC_IO_0 == _SUPPORT_SPI_CLK) || (PIN_FUNC_IO_0 == _SUPPORT_SPI_SS)
                                             | M_PORT_IO_CFG0_IO0_OUT_SEL
#endif
#if (PIN_FUNC_IO_1 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_1 == _SUPPORT_SPI_MISO_IO) || \
    (PIN_FUNC_IO_1 == _SUPPORT_SPI_CLK) || (PIN_FUNC_IO_1 == _SUPPORT_SPI_SS)
                                             | M_PORT_IO_CFG0_IO1_OUT_SEL
#endif
#if (PIN_FUNC_IO_2 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_2 == _SUPPORT_SPI_MISO_IO) || \
    (PIN_FUNC_IO_2 == _SUPPORT_SPI_CLK) || (PIN_FUNC_IO_2 == _SUPPORT_SPI_SS)
                                             | M_PORT_IO_CFG0_IO2_OUT_SEL
#endif
#if (PIN_FUNC_IO_3 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_3 == _SUPPORT_SPI_MISO_IO) || \
    (PIN_FUNC_IO_3 == _SUPPORT_SPI_CLK) || (PIN_FUNC_IO_3 == _SUPPORT_SPI_SS)
                                             | M_PORT_IO_CFG0_IO3_OUT_SEL
#endif
                                             )) | (0
#if (PIN_FUNC_IO_0 == _SUPPORT_SPI_MOSI_IO)
                                                   | C_PORT_IO_CFG0_IO0_OUT_SEL_SPI_MISO /* IO[0]: SPI_MISO */
#elif (PIN_FUNC_IO_0 == _SUPPORT_SPI_MISO_IO)
                                                   | C_PORT_IO_CFG0_IO0_OUT_SEL_SPI_MISO /* IO[0]: SPI MISO */
#elif (PIN_FUNC_IO_0 == _SUPPORT_SPI_CLK)
                                                   | C_PORT_IO_CFG0_IO0_OUT_SEL_SPI_SCK  /* IO[0]: SPI Clock */
#elif (PIN_FUNC_IO_0 == _SUPPORT_SPI_SS)
#if (_SUPPORT_SPI_SS_SOFT != FALSE)
                                                   | C_PORT_IO_CFG0_IO0_OUT_SEL_SOFT     /* IO[0]: SPI Slave Select by SW */
#else  /* (_SUPPORT_SPI_SS_SOFT != FALSE) */
                                                   | C_PORT_IO_CFG0_IO0_OUT_SEL_SPI_SS   /* IO[0]: SPI Slave Select */
#endif /* (_SUPPORT_SPI_SS_SOFT != FALSE) */
#else  /* PIN_FUNC_IO_0 */
                                                   | 0
#endif /* PIN_FUNC_IO_0 */
#if (PIN_FUNC_IO_1 == _SUPPORT_SPI_MOSI_IO)
                                                   | C_PORT_IO_CFG0_IO1_OUT_SEL_SPI_MOSI /* IO[1]: SPI_MOSI */
#elif (PIN_FUNC_IO_1 == _SUPPORT_SPI_MISO_IO)
                                                   | C_PORT_IO_CFG0_IO1_OUT_SEL_SPI_MISO /* IO[1]: SPI MISO */
#elif (PIN_FUNC_IO_1 == _SUPPORT_SPI_CLK)
                                                   | C_PORT_IO_CFG0_IO1_OUT_SEL_SPI_SCK  /* IO[1]: SPI Clock */
#elif (PIN_FUNC_IO_1 == _SUPPORT_SPI_SS)
#if (_SUPPORT_SPI_SS_SOFT != FALSE)
                                                   | C_PORT_IO_CFG0_IO1_OUT_SEL_SOFT     /* IO[1]: SPI Slave Select by SW */
#else  /* (_SUPPORT_SPI_SS_SOFT != FALSE) */
                                                   | C_PORT_IO_CFG0_IO1_OUT_SEL_SPI_SS   /* IO[1]: SPI Slave Select */
#endif /* (_SUPPORT_SPI_SS_SOFT != FALSE) */
#else  /* PIN_FUNC_IO_1 */
                                                   | 0
#endif /* PIN_FUNC_IO_1 */
#if (PIN_FUNC_IO_2 == _SUPPORT_SPI_MOSI_IO)
                                                   | C_PORT_IO_CFG0_IO2_OUT_SEL_SPI_MOSI /* IO[2]: SPI_MOSI */
#elif (PIN_FUNC_IO_2 == _SUPPORT_SPI_MISO_IO)
                                                   | C_PORT_IO_CFG0_IO2_OUT_SEL_SPI_MISO /* IO[2]: SPI MISO */
#elif (PIN_FUNC_IO_2 == _SUPPORT_SPI_CLK)
                                                   | C_PORT_IO_CFG0_IO2_OUT_SEL_SPI_SCK  /* IO[2]: SPI Clock */
#elif (PIN_FUNC_IO_2 == _SUPPORT_SPI_SS)
#if (_SUPPORT_SPI_SS_SOFT != FALSE)
                                                   | C_PORT_IO_CFG0_IO2_OUT_SEL_SOFT     /* IO[2]: SPI Slave Select by SW */
#else  /* (_SUPPORT_SPI_SS_SOFT != FALSE) */
                                                   | C_PORT_IO_CFG0_IO2_OUT_SEL_SPI_SS   /* IO[2]: SPI Slave Select */
#endif /* (_SUPPORT_SPI_SS_SOFT != FALSE) */
#else  /* PIN_FUNC_IO_2 */
                                                   | 0
#endif /* PIN_FUNC_IO_2 */
#if (PIN_FUNC_IO_3 == _SUPPORT_SPI_MOSI_IO)
                                                   | C_PORT_IO_CFG0_IO3_OUT_SEL_SPI_MOSI /* IO[3]: SPI_MOSI */
#elif (PIN_FUNC_IO_3 == _SUPPORT_SPI_MISO_IO)
                                                   | C_PORT_IO_CFG0_IO3_OUT_SEL_SPI_MISO /* IO[3]: SPI MISO */
#elif (PIN_FUNC_IO_3 == _SUPPORT_SPI_CLK)
                                                   | C_PORT_IO_CFG0_IO3_OUT_SEL_SPI_SCK  /* IO[3]: SPI Clock */
#elif (PIN_FUNC_IO_3 == _SUPPORT_SPI_SS)
#if (_SUPPORT_SPI_SS_SOFT != FALSE)
                                                   | C_PORT_IO_CFG0_IO3_OUT_SEL_SOFT     /* IO[3]: SPI Slave Select by SW */
#else  /* (_SUPPORT_SPI_SS_SOFT != FALSE) */
                                                   | C_PORT_IO_CFG0_IO3_OUT_SEL_SPI_SS   /* IO[3]: SPI Slave Select */
#endif /* (_SUPPORT_SPI_SS_SOFT != FALSE) */
#else  /* PIN_FUNC_IO_3 */
                                                   | 0
#endif /* PIN_FUNC_IO_3 */
                                                   ));
#if !defined (__MLX81330__) && !defined (__MLX81350__)
/* Support IO[7:4] */
    IO_PORT_IO_CFG1 = ( (IO_PORT_IO_CFG1 & ~(0
#if (PIN_FUNC_IO_4 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_4 == _SUPPORT_SPI_MISO_IO) || \
                                             (PIN_FUNC_IO_4 == _SUPPORT_SPI_CLK) || (PIN_FUNC_IO_4 == _SUPPORT_SPI_SS)
                                             | M_PORT_IO_CFG1_IO4_OUT_SEL
#endif
#if (PIN_FUNC_IO_5 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_5 == _SUPPORT_SPI_MISO_IO) || \
                                             (PIN_FUNC_IO_5 == _SUPPORT_SPI_CLK) || (PIN_FUNC_IO_5 == _SUPPORT_SPI_SS)
                                             | M_PORT_IO_CFG1_IO5_OUT_SEL
#endif
#if (PIN_FUNC_IO_6 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_6 == _SUPPORT_SPI_MISO_IO) || \
                                             (PIN_FUNC_IO_6 == _SUPPORT_SPI_CLK) || (PIN_FUNC_IO_6 == _SUPPORT_SPI_SS)
                                             | M_PORT_IO_CFG1_IO6_OUT_SEL
#endif
#if (PIN_FUNC_IO_7 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_7 == _SUPPORT_SPI_MISO_IO) || \
                                             (PIN_FUNC_IO_7 == _SUPPORT_SPI_CLK) || (PIN_FUNC_IO_7 == _SUPPORT_SPI_SS)
                                             | M_PORT_IO_CFG1_IO7_OUT_SEL
#endif
                                             )) | (0
#if (PIN_FUNC_IO_4 == _SUPPORT_SPI_MOSI_IO)
                                                   | C_PORT_IO_CFG1_IO4_OUT_SEL_SPI_MOSI /* IO[4]: SPI_MOSI */
#elif (PIN_FUNC_IO_4 == _SUPPORT_SPI_MISO_IO)
                                                   | C_PORT_IO_CFG1_IO4_OUT_SEL_SPI_MISO /* IO[4]: SPI MISO */
#elif (PIN_FUNC_IO_4 == _SUPPORT_SPI_CLK)
                                                   | C_PORT_IO_CFG1_IO4_OUT_SEL_SPI_SCK  /* IO[4]: SPI Clock */
#elif (PIN_FUNC_IO_4 == _SUPPORT_SPI_SS)
#if (_SUPPORT_SPI_SS_SOFT != FALSE)
                                                   | C_PORT_IO_CFG1_IO4_OUT_SEL_SOFT     /* IO[4]: SPI Slave Select by SW */
#else  /* (_SUPPORT_SPI_SS_SOFT != FALSE) */
                                                   | C_PORT_IO_CFG1_IO4_OUT_SEL_SPI_SS   /* IO[4]: SPI Slave Select */
#endif /* (_SUPPORT_SPI_SS_SOFT != FALSE) */
#else  /* PIN_FUNC_IO_4 */
                                                   | 0
#endif /* PIN_FUNC_IO_4 */
#if (PIN_FUNC_IO_5 == _SUPPORT_SPI_MOSI_IO)
                                                   | C_PORT_IO_CFG1_IO5_OUT_SEL_SPI_MOSI /* IO[5]: SPI_MOSI */
#elif (PIN_FUNC_IO_5 == _SUPPORT_SPI_MISO_IO)
                                                   | C_PORT_IO_CFG1_IO5_OUT_SEL_SPI_MISO /* IO[5]: SPI MISO */
#elif (PIN_FUNC_IO_5 == _SUPPORT_SPI_CLK)
                                                   | C_PORT_IO_CFG1_IO5_OUT_SEL_SPI_SCK  /* IO[5]: SPI Clock */
#elif (PIN_FUNC_IO_5 == _SUPPORT_SPI_SS)
#if (_SUPPORT_SPI_SS_SOFT != FALSE)
                                                   | C_PORT_IO_CFG1_IO5_OUT_SEL_SOFT     /* IO[5]: SPI Slave Select by SW */
#else  /* (_SUPPORT_SPI_SS_SOFT != FALSE) */
                                                   | C_PORT_IO_CFG1_IO5_OUT_SEL_SPI_SS   /* IO[5]: SPI Slave Select */
#endif /* (_SUPPORT_SPI_SS_SOFT != FALSE) */
#else  /* PIN_FUNC_IO_5 */
                                                   | 0
#endif /* PIN_FUNC_IO_5 */

#if (PIN_FUNC_IO_6 == _SUPPORT_SPI_MOSI_IO)
                                                   | C_PORT_IO_CFG1_IO6_OUT_SEL_SPI_MOSI /* IO[6]: SPI_MOSI */
#elif (PIN_FUNC_IO_6 == _SUPPORT_SPI_MISO_IO)
                                                   | C_PORT_IO_CFG1_IO6_OUT_SEL_SPI_MISO /* IO[6]: SPI MISO */
#elif (PIN_FUNC_IO_6 == _SUPPORT_SPI_CLK)
                                                   | C_PORT_IO_CFG1_IO6_OUT_SEL_SPI_SCK  /* IO[6]: SPI Clock */
#elif (PIN_FUNC_IO_6 == _SUPPORT_SPI_SS)
#if (_SUPPORT_SPI_SS_SOFT != FALSE)
                                                   | C_PORT_IO_CFG1_IO6_OUT_SEL_SOFT     /* IO[6]: SPI Slave Select by SW */
#else  /* (_SUPPORT_SPI_SS_SOFT != FALSE) */
                                                   | C_PORT_IO_CFG1_IO6_OUT_SEL_SPI_SS   /* IO[6]: SPI Slave Select */
#endif /* (_SUPPORT_SPI_SS_SOFT != FALSE) */
#else  /* PIN_FUNC_IO_6 */
                                                   | 0
#endif /* PIN_FUNC_IO_6 */
#if (PIN_FUNC_IO_7 == _SUPPORT_SPI_MOSI_IO)
                                                   | C_PORT_IO_CFG1_IO7_OUT_SEL_SPI_MOSI /* IO[7]: SPI_MOSI */
#elif (PIN_FUNC_IO_7 == _SUPPORT_SPI_MISO_IO)
                                                   | C_PORT_IO_CFG1_IO7_OUT_SEL_SPI_MISO /* IO[7]: SPI MISO */
#elif (PIN_FUNC_IO_7 == _SUPPORT_SPI_CLK)
                                                   | C_PORT_IO_CFG1_IO7_OUT_SEL_SPI_SCK  /* IO[7]: SPI Clock */
#elif (PIN_FUNC_IO_7 == _SUPPORT_SPI_SS)
#if (_SUPPORT_SPI_SS_SOFT != FALSE)
                                                   | C_PORT_IO_CFG1_IO7_OUT_SEL_SOFT     /* IO[7]: SPI Slave Select by SW */
#else  /* (_SUPPORT_SPI_SS_SOFT != FALSE) */
                                                   | C_PORT_IO_CFG1_IO7_OUT_SEL_SPI_SS   /* IO[7]: SPI Slave Select */
#endif /* (_SUPPORT_SPI_SS_SOFT != FALSE) */
#else  /* PIN_FUNC_IO_7 */
                                                   | 0
#endif /* PIN_FUNC_IO_7 */
                                                   ));
#if defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
/* Support IO[11:8] */
    IO_PORT_IO_CFG2 = ( (IO_PORT_IO_CFG2 & ~(0
#if (PIN_FUNC_IO_8 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_8 == _SUPPORT_SPI_MISO_IO) || \
                                             (PIN_FUNC_IO_8 == _SUPPORT_SPI_CLK) || (PIN_FUNC_IO_8 == _SUPPORT_SPI_SS)
                                             | M_PORT_IO_CFG2_IO8_OUT_SEL
#endif
#if (PIN_FUNC_IO_9 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_9 == _SUPPORT_SPI_MISO_IO) || \
                                             (PIN_FUNC_IO_9 == _SUPPORT_SPI_CLK) || (PIN_FUNC_IO_9 == _SUPPORT_SPI_SS)
                                             | M_PORT_IO_CFG2_IO9_OUT_SEL
#endif
#if (PIN_FUNC_IO_10 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_10 == _SUPPORT_SPI_MISO_IO) || \
                                             (PIN_FUNC_IO_10 == _SUPPORT_SPI_CLK) || (PIN_FUNC_IO_10 == _SUPPORT_SPI_SS)
                                             | M_PORT_IO_CFG2_IO10_OUT_SEL
#endif
#if (PIN_FUNC_IO_11 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_11 == _SUPPORT_SPI_MISO_IO) || \
                                             (PIN_FUNC_IO_11 == _SUPPORT_SPI_CLK) || (PIN_FUNC_IO_11 == _SUPPORT_SPI_SS)
                                             | M_PORT_IO_CFG2_IO11_OUT_SEL
#endif
                                             )) | (0
#if (PIN_FUNC_IO_8 == _SUPPORT_SPI_MOSI_IO)
                                                   | C_PORT_IO_CFG2_IO8_OUT_SEL_SPI_MOSI /* IO[8]: SPI_MOSI */
#elif (PIN_FUNC_IO_8 == _SUPPORT_SPI_MISO_IO)
                                                   | C_PORT_IO_CFG2_IO8_OUT_SEL_SPI_MISO /* IO[8]: SPI MISO */
#elif (PIN_FUNC_IO_8 == _SUPPORT_SPI_CLK)
                                                   | C_PORT_IO_CFG2_IO8_OUT_SEL_SPI_SCK  /* IO[8]: SPI Clock */
#elif (PIN_FUNC_IO_8 == _SUPPORT_SPI_SS)
#if (_SUPPORT_SPI_SS_SOFT != FALSE)
                                                   | C_PORT_IO_CFG2_IO8_OUT_SEL_SOFT     /* IO[8]: SPI Slave Select by SW */
#else  /* (_SUPPORT_SPI_SS_SOFT != FALSE) */
                                                   | C_PORT_IO_CFG2_IO8_OUT_SEL_SPI_SS   /* IO[8]: SPI Slave Select */
#endif /* (_SUPPORT_SPI_SS_SOFT != FALSE) */
#else  /* PIN_FUNC_IO_8 */
                                                   | 0
#endif /* PIN_FUNC_IO_8 */
#if (PIN_FUNC_IO_9 == _SUPPORT_SPI_MOSI_IO)
                                                   | C_PORT_IO_CFG2_IO9_OUT_SEL_SPI_MOSI /* IO[9]: SPI_MOSI */
#elif (PIN_FUNC_IO_9 == _SUPPORT_SPI_MISO_IO)
                                                   | C_PORT_IO_CFG2_IO9_OUT_SEL_SPI_MISO /* IO[9]: SPI MISO */
#elif (PIN_FUNC_IO_9 == _SUPPORT_SPI_CLK)
                                                   | C_PORT_IO_CFG2_IO9_OUT_SEL_SPI_SCK  /* IO[9]: SPI Clock */
#elif (PIN_FUNC_IO_9 == _SUPPORT_SPI_SS)
#if (_SUPPORT_SPI_SS_SOFT != FALSE)
                                                   | C_PORT_IO_CFG2_IO9_OUT_SEL_SOFT     /* IO[9]: SPI Slave Select by SW */
#else  /* (_SUPPORT_SPI_SS_SOFT != FALSE) */
                                                   | C_PORT_IO_CFG2_IO9_OUT_SEL_SPI_SS   /* IO[9]: SPI Slave Select */
#endif /* (_SUPPORT_SPI_SS_SOFT != FALSE) */
#else  /* PIN_FUNC_IO_9 */
                                                   | 0
#endif /* PIN_FUNC_IO_9 */

#if (PIN_FUNC_IO_10 == _SUPPORT_SPI_MOSI_IO)
                                                   | C_PORT_IO_CFG1_IO10_OUT_SEL_SPI_MOSI /* IO[6]: SPI_MOSI */
#elif (PIN_FUNC_IO_10 == _SUPPORT_SPI_MISO_IO)
                                                   | C_PORT_IO_CFG1_IO10_OUT_SEL_SPI_MISO /* IO[6]: SPI MISO */
#elif (PIN_FUNC_IO_10 == _SUPPORT_SPI_CLK)
                                                   | C_PORT_IO_CFG1_IO10_OUT_SEL_SPI_SCK  /* IO[6]: SPI Clock */
#elif (PIN_FUNC_IO_10 == _SUPPORT_SPI_SS)
#if (_SUPPORT_SPI_SS_SOFT != FALSE)
                                                   | C_PORT_IO_CFG1_IO10_OUT_SEL_SOFT     /* IO[6]: SPI Slave Select by SW */
#else  /* (_SUPPORT_SPI_SS_SOFT != FALSE) */
                                                   | C_PORT_IO_CFG1_IO10_OUT_SEL_SPI_SS   /* IO[6]: SPI Slave Select */
#endif /* (_SUPPORT_SPI_SS_SOFT != FALSE) */
#else  /* PIN_FUNC_IO_10 */
                                                   | 0
#endif /* PIN_FUNC_IO_10 */
#if (PIN_FUNC_IO_11 == _SUPPORT_SPI_MOSI_IO)
                                                   | C_PORT_IO_CFG1_IO11_OUT_SEL_SPI_MOSI /* IO[11]: SPI_MOSI */
#elif (PIN_FUNC_IO_11 == _SUPPORT_SPI_MISO_IO)
                                                   | C_PORT_IO_CFG1_IO11_OUT_SEL_SPI_MISO /* IO[11]: SPI MISO */
#elif (PIN_FUNC_IO_11 == _SUPPORT_SPI_CLK)
                                                   | C_PORT_IO_CFG1_IO11_OUT_SEL_SPI_SCK  /* IO[11]: SPI Clock */
#elif (PIN_FUNC_IO_11 == _SUPPORT_SPI_SS)
#if (_SUPPORT_SPI_SS_SOFT != FALSE)
                                                   | C_PORT_IO_CFG1_IO11_OUT_SEL_SOFT     /* IO[11]: SPI Slave Select by SW */
#else  /* (_SUPPORT_SPI_SS_SOFT != FALSE) */
                                                   | C_PORT_IO_CFG1_IO11_OUT_SEL_SPI_SS   /* IO[11]: SPI Slave Select */
#endif /* (_SUPPORT_SPI_SS_SOFT != FALSE) */
#else  /* PIN_FUNC_IO_11 */
                                                   | 0
#endif /* PIN_FUNC_IO_11 */
                                                   ));
#endif /* defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */
#endif /* !defined (__MLX81330__) && !defined (__MLX81350__) */

    /* Setup I/O for LV */
    /* In case IO[0] used by SPI */
#if (PIN_FUNC_IO_0 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_0 == _SUPPORT_SPI_MISO_IO) || \
    (PIN_FUNC_IO_0 == _SUPPORT_SPI_CLK) || (PIN_FUNC_IO_0 == _SUPPORT_SPI_SS)
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
    IO_PORT_IO_OUT_EN |= B_PORT_IO_OUT_EN_IO0_LV_ENABLE;
#elif defined (__MLX81340A01__) || defined (__MLX81344A01__) || defined (__MLX81346A01__)
    IO_PORT_IO_OUT_EN &= ~B_PORT_IO_OUT_EN_IO_HV_ENABLE_0;
#elif defined (__MLX81160__) || defined (__MLX81340B01__) || defined (__MLX81344B01__) || defined (__MLX81346B01__)
    IO_PORT_IO_OUT_EN |= B_PORT_IO_OUT_EN_IO_LV_ENABLE_0;
#endif /* defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__)|| defined (__MLX81350__) */
#endif /* (PIN_FUNC_IO_0 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_0 == _SUPPORT_SPI_MISO_IO) || (PIN_FUNC_IO_0 == _SUPPORT_SPI_CLK) || (PIN_FUNC_IO_0 == _SUPPORT_SPI_SS) */
#if defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    /* In case IO[1] used by SPI, and support HV/LV */
#if (PIN_FUNC_IO_1 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_1 == _SUPPORT_SPI_MISO_IO) || \
    (PIN_FUNC_IO_1 == _SUPPORT_SPI_CLK) || (PIN_FUNC_IO_1 == _SUPPORT_SPI_SS)
#if defined (__MLX81340A01__) || defined (__MLX81344A01__) || defined (__MLX81346A01__)
    IO_PORT_IO_OUT_EN &= ~B_PORT_IO_OUT_EN_IO_HV_ENABLE_1;
#elif defined (__MLX81160__) || defined (__MLX81340B01__) || defined (__MLX81344B01__) || defined (__MLX81346B01__)
    IO_PORT_IO_OUT_EN |= B_PORT_IO_OUT_EN_IO_LV_ENABLE_1;
#endif /* defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */
#endif /* (PIN_FUNC_IO_1 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_1 == _SUPPORT_SPI_MISO_IO) || (PIN_FUNC_IO_1 == _SUPPORT_SPI_CLK) || (PIN_FUNC_IO_1 == _SUPPORT_SPI_SS) */
    /* In case IO[2] used by SPI, and support HV/LV */
#if (PIN_FUNC_IO_2 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_2 == _SUPPORT_SPI_MISO_IO) || \
    (PIN_FUNC_IO_2 == _SUPPORT_SPI_CLK) || (PIN_FUNC_IO_2 == _SUPPORT_SPI_SS)
#if defined (__MLX81340A01__) || defined (__MLX81344A01__) || defined (__MLX81346A01__)
    IO_PORT_IO_OUT_EN &= ~B_PORT_IO_OUT_EN_IO_HV_ENABLE_2;
#elif defined (__MLX81160__) || defined (__MLX81340B01__) || defined (__MLX81344B01__) || defined (__MLX81346B01__)
    IO_PORT_IO_OUT_EN |= B_PORT_IO_OUT_EN_IO_LV_ENABLE_2;
#endif /* defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */
#endif /* (PIN_FUNC_IO_2 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_2 == _SUPPORT_SPI_MISO_IO) || (PIN_FUNC_IO_2 == _SUPPORT_SPI_CLK) || (PIN_FUNC_IO_2 == _SUPPORT_SPI_SS) */
#if defined (__MLX81344__) || defined (__MLX81346__)
    /* In case IO[3] used by SPI, and support HV/LV */
#if (PIN_FUNC_IO_3 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_3 == _SUPPORT_SPI_MISO_IO) || \
    (PIN_FUNC_IO_3 == _SUPPORT_SPI_CLK) || (PIN_FUNC_IO_3 == _SUPPORT_SPI_SS)
#if defined (__MLX81344A01__) || defined (__MLX81346A01__)
    IO_PORT_IO_OUT_EN &= ~B_PORT_IO_OUT_EN_IO_HV_ENABLE_3;
#elif defined (__MLX81344B01__) || defined (__MLX81346B01__)
    IO_PORT_IO_OUT_EN |= B_PORT_IO_OUT_EN_IO_LV_ENABLE_3;
#endif /* defined (__MLX81344__) || defined (__MLX81346__) */
#endif /* (PIN_FUNC_IO_3 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_3 == _SUPPORT_SPI_MISO_IO) || (PIN_FUNC_IO_3 == _SUPPORT_SPI_CLK) || (PIN_FUNC_IO_3 == _SUPPORT_SPI_SS) */
    /* In case IO[4] used by SPI, and support HV/LV */
#if (PIN_FUNC_IO_4 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_4 == _SUPPORT_SPI_MISO_IO) || \
    (PIN_FUNC_IO_4 == _SUPPORT_SPI_CLK) || (PIN_FUNC_IO_4 == _SUPPORT_SPI_SS)
#if defined (__MLX81344A01__) || defined (__MLX81346A01__)
    IO_PORT_IO_OUT_EN &= ~B_PORT_IO_OUT_EN_IO_HV_ENABLE_4;
#elif defined (__MLX81344B01__) || defined (__MLX81346B01__)
    IO_PORT_IO_OUT_EN |= B_PORT_IO_OUT_EN_IO_LV_ENABLE_4;
#endif /* defined (__MLX81344__) || defined (__MLX81346__) */
#endif /* (PIN_FUNC_IO_4 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_4 == _SUPPORT_SPI_MISO_IO) || (PIN_FUNC_IO_4 == _SUPPORT_SPI_CLK) || (PIN_FUNC_IO_4 == _SUPPORT_SPI_SS) */
#endif /* defined (__MLX81344__) || defined (__MLX81346__) */
#endif /* defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */

#if (_SUPPORT_SPI_MASTER != FALSE)
    /* SPI Master: Make MOSI, CLK & SS output's & MISO an input */
 #if defined (__MLX81330A01__)
    IO_PORT_IO_OUT_EN = ((IO_PORT_IO_OUT_EN & ~(0
  #if (PIN_FUNC_IO_0 == _SUPPORT_SPI_MISO_IO)
                                                | C_PORT_IO_OUT_EN_IO0_EN       /* Disable output IO[0] */
  #elif (PIN_FUNC_IO_1 == _SUPPORT_SPI_MISO_IO)
                                                | C_PORT_IO_OUT_EN_IO1_EN       /* Disable output IO[1] */
  #elif (PIN_FUNC_IO_2 == _SUPPORT_SPI_MISO_IO)
                                                | C_PORT_IO_OUT_EN_IO2_EN       /* Disable output IO[2] */
  #elif (PIN_FUNC_IO_3 == _SUPPORT_SPI_MISO_IO)
                                                | C_PORT_IO_OUT_EN_IO3_EN       /* Disable output IO[3] */
  #endif
                                                )) | (0
  #if (PIN_FUNC_IO_0 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_0 == _SUPPORT_SPI_CLK) || \
      (PIN_FUNC_IO_0 == _SUPPORT_SPI_SS)
                                                      | C_PORT_IO_OUT_EN_IO0_EN /* Enable output IO[0] */
  #endif /* (PIN_FUNC_IO_0 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_0 == _SUPPORT_SPI_MISO_IO) || (PIN_FUNC_IO_0 == _SUPPORT_SPI_CLK) || (PIN_FUNC_IO_0 == _SUPPORT_SPI_SS) */
  #if (PIN_FUNC_IO_1 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_1 == _SUPPORT_SPI_CLK) || \
      (PIN_FUNC_IO_1 == _SUPPORT_SPI_SS)
                                                      | C_PORT_IO_OUT_EN_IO1_EN /* Enable output IO[1] */
  #endif
  #if (PIN_FUNC_IO_2 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_2 == _SUPPORT_SPI_CLK) || \
      (PIN_FUNC_IO_2 == _SUPPORT_SPI_SS)
                                                      | C_PORT_IO_OUT_EN_IO2_EN /* Enable output IO[2] */
  #endif
  #if (PIN_FUNC_IO_3 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_3 == _SUPPORT_SPI_CLK) || \
      (PIN_FUNC_IO_3 == _SUPPORT_SPI_SS)
                                                      | C_PORT_IO_OUT_EN_IO3_EN /* Enable output IO[3] */
  #endif
                                                      ));
 #else  /* defined (__MLX81330A01__) */
    IO_PORT_IO_ENABLE = ((IO_PORT_IO_ENABLE & ~(0
  #if (PIN_FUNC_IO_0 == _SUPPORT_SPI_MISO_IO)
                                                | B_PORT_IO_ENABLE_IO_ENABLE_0  /* Disable output IO[0] */
  #elif (PIN_FUNC_IO_1 == _SUPPORT_SPI_MISO_IO)
                                                | B_PORT_IO_ENABLE_IO_ENABLE_1  /* Disable output IO[1] */
  #elif (PIN_FUNC_IO_2 == _SUPPORT_SPI_MISO_IO)
                                                | B_PORT_IO_ENABLE_IO_ENABLE_2  /* Disable output IO[2] */
  #elif (PIN_FUNC_IO_3 == _SUPPORT_SPI_MISO_IO)
                                                | B_PORT_IO_ENABLE_IO_ENABLE_3  /* Disable output IO[3] */
  #elif (PIN_FUNC_IO_4 == _SUPPORT_SPI_MISO_IO)
                                                | B_PORT_IO_ENABLE_IO_ENABLE_4  /* Disable output IO[4] */
  #elif (PIN_FUNC_IO_5 == _SUPPORT_SPI_MISO_IO)
                                                | B_PORT_IO_ENABLE_IO_ENABLE_5  /* Disable output IO[5] */
  #elif (PIN_FUNC_IO_6 == _SUPPORT_SPI_MISO_IO)
                                                | B_PORT_IO_ENABLE_IO_ENABLE_6  /* Disable output IO[6] */
  #elif (PIN_FUNC_IO_7 == _SUPPORT_SPI_MISO_IO)
                                                | B_PORT_IO_ENABLE_IO_ENABLE_7  /* Disable output IO[7] */
  #endif
                                                )) | (0
  #if (PIN_FUNC_IO_0 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_0 == _SUPPORT_SPI_CLK) || \
      (PIN_FUNC_IO_0 == _SUPPORT_SPI_SS)
                                                      | B_PORT_IO_ENABLE_IO_ENABLE_0 /* Enable output IO[0] */
  #endif /* (PIN_FUNC_IO_0 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_0 == _SUPPORT_SPI_MISO_IO) || (PIN_FUNC_IO_0 == _SUPPORT_SPI_CLK) || (PIN_FUNC_IO_0 == _SUPPORT_SPI_SS) */
  #if (PIN_FUNC_IO_1 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_1 == _SUPPORT_SPI_CLK) || \
      (PIN_FUNC_IO_1 == _SUPPORT_SPI_SS)
                                                      | B_PORT_IO_ENABLE_IO_ENABLE_1 /* Enable output IO[1] */
  #endif
  #if (PIN_FUNC_IO_2 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_2 == _SUPPORT_SPI_CLK) || \
      (PIN_FUNC_IO_2 == _SUPPORT_SPI_SS)
                                                      | B_PORT_IO_ENABLE_IO_ENABLE_2 /* Enable output IO[2] */
  #endif
  #if (PIN_FUNC_IO_3 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_3 == _SUPPORT_SPI_CLK) || \
      (PIN_FUNC_IO_3 == _SUPPORT_SPI_SS)
                                                      | B_PORT_IO_ENABLE_IO_ENABLE_3 /* Enable output IO[3] */
  #endif
  #if !defined (__MLX81330__) && !defined (__MLX81350__) && \
      ((PIN_FUNC_IO_4 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_4 == _SUPPORT_SPI_CLK) || \
       (PIN_FUNC_IO_4 == _SUPPORT_SPI_SS))
                                                      | B_PORT_IO_ENABLE_IO_ENABLE_4 /* Enable output IO[4] */
  #endif
  #if !defined (__MLX81330__) && !defined (__MLX81350__) && \
      ((PIN_FUNC_IO_5 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_5 == _SUPPORT_SPI_CLK) || \
       (PIN_FUNC_IO_5 == _SUPPORT_SPI_SS))
                                                      | B_PORT_IO_ENABLE_IO_ENABLE_5 /* Enable output IO[5] */
  #endif
  #if !defined (__MLX81330__) && !defined (__MLX81350__) && \
      ((PIN_FUNC_IO_6 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_6 == _SUPPORT_SPI_CLK) || \
       (PIN_FUNC_IO_6 == _SUPPORT_SPI_SS))
                                                      | B_PORT_IO_ENABLE_IO_ENABLE_6 /* Enable output IO[6] */
  #endif
  #if !defined (__MLX81330__) && !defined (__MLX81350__) && \
      ((PIN_FUNC_IO_7 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_7 == _SUPPORT_SPI_CLK) || \
       (PIN_FUNC_IO_7 == _SUPPORT_SPI_SS))
                                                      | B_PORT_IO_ENABLE_IO_ENABLE_7 /* Enable output IO[7] */
  #endif
                                                      ));
  #if defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    IO_PORT_IO_ENABLE1 = ((IO_PORT_IO_ENABLE1 & ~(0
   #if (PIN_FUNC_IO_8 == _SUPPORT_SPI_MISO_IO)
                                                  | B_PORT_IO_ENABLE1_IO_ENABLE_8 /* Disable output IO[8] */
   #elif (PIN_FUNC_IO_9 == _SUPPORT_SPI_MISO_IO)
                                                  | B_PORT_IO_ENABLE1_IO_ENABLE_9 /* Disable output IO[9] */
   #elif (PIN_FUNC_IO_10 == _SUPPORT_SPI_MISO_IO)
                                                  | B_PORT_IO_ENABLE1_IO_ENABLE_10 /* Disable output IO[10] */
   #elif (PIN_FUNC_IO_11 == _SUPPORT_SPI_MISO_IO)
                                                  | B_PORT_IO_ENABLE1_IO_ENABLE_11 /* Disable output IO[11] */
   #endif
                                                  )) | (0
   #if (PIN_FUNC_IO_8 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_8 == _SUPPORT_SPI_CLK) || \
       (PIN_FUNC_IO_8 == _SUPPORT_SPI_SS)
                                                        | B_PORT_IO_ENABLE1_IO_ENABLE_8 /* Enable output IO[8] */
   #elif (PIN_FUNC_IO_9 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_9 == _SUPPORT_SPI_CLK) || \
         (PIN_FUNC_IO_9 == _SUPPORT_SPI_SS)
                                                        | B_PORT_IO_ENABLE1_IO_ENABLE_9 /* Enable output IO[8] */
   #elif (PIN_FUNC_IO_10 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_10 == _SUPPORT_SPI_CLK) || \
         (PIN_FUNC_IO_10 == _SUPPORT_SPI_SS)
                                                        | B_PORT_IO_ENABLE1_IO_ENABLE_10 /* Enable output IO[8] */
   #elif (PIN_FUNC_IO_11 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_11 == _SUPPORT_SPI_CLK) || \
         (PIN_FUNC_IO_11 == _SUPPORT_SPI_SS)
                                                        | B_PORT_IO_ENABLE1_IO_ENABLE_11 /* Enable output IO[8] */
   #endif
                                                        ));
  #endif /* defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */
 #endif /* defined (__MLX81330A01__) */
#else  /* (_SUPPORT_SPI_MASTER != FALSE) */
       /* SPI Slave: Only MISO is an OUTPUT, MISO, CLK and SS are INPUT's */
 #if defined (__MLX81330A01__)
    IO_PORT_IO_OUT_EN = ((IO_PORT_IO_OUT_EN & ~(0
  #if (PIN_FUNC_IO_0 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_0 == _SUPPORT_SPI_CLK) || \
      (PIN_FUNC_IO_0 == _SUPPORT_SPI_SS)
                                                | C_PORT_IO_OUT_EN_IO0_EN       /* Disable output IO[0] */
  #endif
  #if (PIN_FUNC_IO_1 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_1 == _SUPPORT_SPI_CLK) || \
      (PIN_FUNC_IO_1 == _SUPPORT_SPI_SS)
                                                | C_PORT_IO_OUT_EN_IO1_EN       /* Disable output IO[1] */
  #endif
  #if (PIN_FUNC_IO_2 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_2 == _SUPPORT_SPI_CLK) || \
      (PIN_FUNC_IO_2 == _SUPPORT_SPI_SS
                                                | C_PORT_IO_OUT_EN_IO2_EN       /* Disable output IO[2] */
  #endif
  #if (PIN_FUNC_IO_3 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_3 == _SUPPORT_SPI_CLK) || \
      (PIN_FUNC_IO_3 == _SUPPORT_SPI_SS)
                                                | C_PORT_IO_OUT_EN_IO3_EN       /* Disable output IO[3] */
  #endif
                                                )) | (0
  #if (PIN_FUNC_IO_0 == _SUPPORT_SPI_MISO_IO)
                                                      | C_PORT_IO_OUT_EN_IO0_EN /* Enable output IO[0] */
  #elif (PIN_FUNC_IO_1 == _SUPPORT_SPI_MISO_IO)
                                                      | C_PORT_IO_OUT_EN_IO1_EN /* Enable output IO[1] */
  #elif (PIN_FUNC_IO_2 == _SUPPORT_SPI_MISO_IO)
                                                      | C_PORT_IO_OUT_EN_IO2_EN /* Enable output IO[2] */
  #elif (PIN_FUNC_IO_3 == _SUPPORT_SPI_MISO_IO)
                                                      | C_PORT_IO_OUT_EN_IO3_EN /* Enable output IO[3] */
  #endif
                                                      ));
 #else  /* defined (__MLX81330A01__) */
    IO_PORT_IO_ENABLE = ((IO_PORT_IO_ENABLE & ~(0
  #if (PIN_FUNC_IO_0 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_0 == _SUPPORT_SPI_CLK) || \
      (PIN_FUNC_IO_0 == _SUPPORT_SPI_SS)
                                                | B_PORT_IO_ENABLE_IO_ENABLE_0  /* Disable output IO[0] */
  #endif
  #if (PIN_FUNC_IO_1 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_1 == _SUPPORT_SPI_CLK) || \
      (PIN_FUNC_IO_1 == _SUPPORT_SPI_SS)
                                                | B_PORT_IO_ENABLE_IO_ENABLE_1  /* Disable output IO[1] */
  #endif
  #if (PIN_FUNC_IO_2 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_2 == _SUPPORT_SPI_CLK) || \
      (PIN_FUNC_IO_2 == _SUPPORT_SPI_SS)
                                                | B_PORT_IO_ENABLE_IO_ENABLE_2  /* Disable output IO[2] */
  #endif
  #if (PIN_FUNC_IO_3 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_3 == _SUPPORT_SPI_CLK) || \
      (PIN_FUNC_IO_3 == _SUPPORT_SPI_SS)
                                                | B_PORT_IO_ENABLE_IO_ENABLE_3  /* Disable output IO[3] */
  #endif
  #if !defined (__MLX81330__) && !defined (__MLX81350__) && \
      ((PIN_FUNC_IO_4 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_4 == _SUPPORT_SPI_CLK) || \
       (PIN_FUNC_IO_4 == _SUPPORT_SPI_SS))
                                                | B_PORT_IO_ENABLE_IO_ENABLE_4  /* Disable output IO[4] */
  #endif
  #if !defined (__MLX81330__) && !defined (__MLX81350__) && \
      ((PIN_FUNC_IO_5 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_5 == _SUPPORT_SPI_CLK) || \
       (PIN_FUNC_IO_5 == _SUPPORT_SPI_SS))
                                                | B_PORT_IO_ENABLE_IO_ENABLE_5  /* Disable output IO[5] */
  #endif
  #if !defined (__MLX81330__) && !defined (__MLX81350__) && \
      ((PIN_FUNC_IO_6 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_6 == _SUPPORT_SPI_CLK) || \
       (PIN_FUNC_IO_6 == _SUPPORT_SPI_SS))
                                                | B_PORT_IO_ENABLE_IO_ENABLE_6  /* Disable output IO[6] */
  #endif
  #if !defined (__MLX81330__) && !defined (__MLX81350__) && \
      ((PIN_FUNC_IO_7 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_7 == _SUPPORT_SPI_CLK) || \
       (PIN_FUNC_IO_7 == _SUPPORT_SPI_SS))
                                                | B_PORT_IO_ENABLE_IO_ENABLE_7  /* Disable output IO[7] */
  #endif
                                                )) | (0
  #if (PIN_FUNC_IO_0 == _SUPPORT_SPI_MISO_IO)
                                                      | B_PORT_IO_ENABLE_IO_ENABLE_0 /* Enable output IO[0] */
  #elif (PIN_FUNC_IO_1 == _SUPPORT_SPI_MISO_IO)
                                                      | B_PORT_IO_ENABLE_IO_ENABLE_1 /* Enable output IO[1] */
  #elif (PIN_FUNC_IO_2 == _SUPPORT_SPI_MISO_IO)
                                                      | B_PORT_IO_ENABLE_IO_ENABLE_2 /* Enable output IO[2] */
  #elif (PIN_FUNC_IO_3 == _SUPPORT_SPI_MISO_IO)
                                                      | B_PORT_IO_ENABLE_IO_ENABLE_3 /* Enable output IO[3] */
  #elif !defined (__MLX81330__) && !defined (__MLX81350__) && (PIN_FUNC_IO_4 == _SUPPORT_SPI_MISO_IO)
                                                      | B_PORT_IO_ENABLE_IO_ENABLE_4 /* Enable output IO[4] */
  #elif !defined (__MLX81330__) && !defined (__MLX81350__) && (PIN_FUNC_IO_5 == _SUPPORT_SPI_MISO_IO)
                                                      | B_PORT_IO_ENABLE_IO_ENABLE_5 /* Enable output IO[5] */
  #elif !defined (__MLX81330__) && !defined (__MLX81350__) && (PIN_FUNC_IO_6 == _SUPPORT_SPI_MISO_IO)
                                                      | B_PORT_IO_ENABLE_IO_ENABLE_6 /* Enable output IO[6] */
  #elif !defined (__MLX81330__) && !defined (__MLX81350__) && (PIN_FUNC_IO_7 == _SUPPORT_SPI_MISO_IO)
                                                      | B_PORT_IO_ENABLE_IO_ENABLE_7 /* Enable output IO[7] */
  #endif
                                                      ));
  #if defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    IO_PORT_IO_ENABLE1 = ((IO_PORT_IO_ENABLE1 & ~(0
   #if (PIN_FUNC_IO_8 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_8 == _SUPPORT_SPI_CLK) || \
       (PIN_FUNC_IO_8 == _SUPPORT_SPI_SS)
                                                  | B_PORT_IO_ENABLE1_IO_ENABLE_8 /* Disable output IO[8] */
   #endif
   #if (PIN_FUNC_IO_9 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_9 == _SUPPORT_SPI_CLK) || \
       (PIN_FUNC_IO_9 == _SUPPORT_SPI_SS)
                                                  | B_PORT_IO_ENABLE1_IO_ENABLE_9 /* Disable output IO[9] */
   #endif
   #if (PIN_FUNC_IO_10 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_10 == _SUPPORT_SPI_CLK) || \
       (PIN_FUNC_IO_10 == _SUPPORT_SPI_SS)
                                                  | B_PORT_IO_ENABLE1_IO_ENABLE_10 /* Disable output IO[10] */
   #endif
   #if (PIN_FUNC_IO_11 == _SUPPORT_SPI_MOSI_IO) || (PIN_FUNC_IO_11 == _SUPPORT_SPI_CLK) || \
       (PIN_FUNC_IO_11 == _SUPPORT_SPI_SS)
                                                  | B_PORT_IO_ENABLE1_IO_ENABLE_11 /* Disable output IO[11] */
   #endif
                                                  )) | (0
   #if (PIN_FUNC_IO_8 == _SUPPORT_SPI_MISO_IO)
                                                        | B_PORT_IO_ENABLE1_IO_ENABLE_8 /* Enable output IO[8] */
   #elif (PIN_FUNC_IO_9 == _SUPPORT_SPI_MISO_IO)
                                                        | B_PORT_IO_ENABLE1_IO_ENABLE_9 /* Enable output IO[9] */
   #elif (PIN_FUNC_IO_10 == _SUPPORT_SPI_MISO_IO)
                                                        | B_PORT_IO_ENABLE1_IO_ENABLE_10 /* Enable output IO[10] */
   #elif (PIN_FUNC_IO_11 == _SUPPORT_SPI_MISO_IO)
                                                        | B_PORT_IO_ENABLE1_IO_ENABLE_11 /* Enable output IO[11] */
   #endif
                                                        ));
  #endif /* defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */
 #endif /* defined (__MLX81330A01__) */
#endif /* (_SUPPORT_SPI_MASTER != FALSE) */

} /* End of HAL_SPI_IO_Setup() */

/*!*************************************************************************** *
 * HAL_SPI_SetPrioAndEnableIRQ
 * \brief   Setup SPI IRQ Priority and enable SPI IRQ
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Setup SPI IRQ's Priority (PRIO 6) and enable the SPI IRQ's
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static __inline__ void HAL_SPI_SetPrioAndEnableIRQ(void)
{
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(ATOMIC_SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    IO_MLX16_ITC_PRIO0_S = (IO_MLX16_ITC_PRIO0_S & ~M_MLX16_ITC_PRIO0_SPI_TE) | C_MLX16_ITC_PRIO0_SPI_TE_PRIO6;
    IO_MLX16_ITC_PRIO1_S = (IO_MLX16_ITC_PRIO1_S & ~M_MLX16_ITC_PRIO1_SPI_RF) | C_MLX16_ITC_PRIO1_SPI_RF_PRIO6;
    IO_MLX16_ITC_PEND1_S = (B_MLX16_ITC_PEND1_SPI_TE | B_MLX16_ITC_PEND1_SPI_RF | B_MLX16_ITC_PEND1_SPI_ER);
#if (_SUPPORT_SPI_ISR != FALSE)
    IO_MLX16_ITC_MASK1_S |= (B_MLX16_ITC_MASK1_SPI_TE | B_MLX16_ITC_MASK1_SPI_RF | B_MLX16_ITC_MASK1_SPI_ER);
#endif /* (_SUPPORT_SPI_ISR != FALSE) */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
} /* End of HAL_SPI_SetPrioAndEnableIRQ() */

/*!*************************************************************************** *
 * HAL_SPI_SS_Active
 * \brief   Activate Soft Slave Select
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Activate Slave Select by setting SS-I/O to Low
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static __inline__ void HAL_SPI_SS_Active(void)
{
#if (PIN_FUNC_IO_0 == _SUPPORT_SPI_SS)
    IO_PORT_IO_OUT_SOFT &= ~C_PORT_IO_OUT_SOFT_IO0_OUT;
#elif (PIN_FUNC_IO_1 == _SUPPORT_SPI_SS)
    IO_PORT_IO_OUT_SOFT &= ~C_PORT_IO_OUT_SOFT_IO1_OUT;
#elif (PIN_FUNC_IO_2 == _SUPPORT_SPI_SS)
    IO_PORT_IO_OUT_SOFT &= ~C_PORT_IO_OUT_SOFT_IO2_OUT;
#elif (PIN_FUNC_IO_3 == _SUPPORT_SPI_SS)
    IO_PORT_IO_OUT_SOFT &= ~C_PORT_IO_OUT_SOFT_IO3_OUT;
#elif (PIN_FUNC_IO_4 == _SUPPORT_SPI_SS)
    IO_PORT_IO_OUT_SOFT &= ~C_PORT_IO_OUT_SOFT_IO4_OUT;
#elif (PIN_FUNC_IO_5 == _SUPPORT_SPI_SS)
    IO_PORT_IO_OUT_SOFT &= ~C_PORT_IO_OUT_SOFT_IO5_OUT;
#elif (PIN_FUNC_IO_6 == _SUPPORT_SPI_SS)
    IO_PORT_IO_OUT_SOFT &= ~C_PORT_IO_OUT_SOFT_IO6_OUT;
#elif (PIN_FUNC_IO_7 == _SUPPORT_SPI_SS)
    IO_PORT_IO_OUT_SOFT &= ~C_PORT_IO_OUT_SOFT_IO7_OUT;
#elif (PIN_FUNC_IO_8 == _SUPPORT_SPI_SS)
    IO_PORT_IO_OUT_SOFT &= ~C_PORT_IO_OUT_SOFT_IO8_OUT;
#elif (PIN_FUNC_IO_9 == _SUPPORT_SPI_SS)
    IO_PORT_IO_OUT_SOFT &= ~C_PORT_IO_OUT_SOFT_IO9_OUT;
#elif (PIN_FUNC_IO_10 == _SUPPORT_SPI_SS)
    IO_PORT_IO_OUT_SOFT &= ~C_PORT_IO_OUT_SOFT_IO10_OUT;
#elif (PIN_FUNC_IO_11 == _SUPPORT_SPI_SS)
    IO_PORT_IO_OUT_SOFT &= ~C_PORT_IO_OUT_SOFT_IO11_OUT;
#else  /* _SUPPORT_SPI_SS */
#error "ERROR: SPI (Soft) Slave Select I/O not support"
#endif /* _SUPPORT_SPI_SS */
} /* End of HAL_SPI_SS_Active() */

/*!*************************************************************************** *
 * HAL_SPI_SS_Deactive
 * \brief   Deactivate Soft Slave Select
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Deactivate Slave Select by setting SS-I/O to High
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
static __inline__ void HAL_SPI_SS_Deactive(void)
{
#if (PIN_FUNC_IO_0 == _SUPPORT_SPI_SS)
    IO_PORT_IO_OUT_SOFT |= C_PORT_IO_OUT_SOFT_IO0_OUT;
#elif (PIN_FUNC_IO_1 == _SUPPORT_SPI_SS)
    IO_PORT_IO_OUT_SOFT |= C_PORT_IO_OUT_SOFT_IO1_OUT;
#elif (PIN_FUNC_IO_2 == _SUPPORT_SPI_SS)
    IO_PORT_IO_OUT_SOFT |= C_PORT_IO_OUT_SOFT_IO2_OUT;
#elif (PIN_FUNC_IO_3 == _SUPPORT_SPI_SS)
    IO_PORT_IO_OUT_SOFT |= C_PORT_IO_OUT_SOFT_IO3_OUT;
#elif (PIN_FUNC_IO_4 == _SUPPORT_SPI_SS)
    IO_PORT_IO_OUT_SOFT |= C_PORT_IO_OUT_SOFT_IO4_OUT;
#elif (PIN_FUNC_IO_5 == _SUPPORT_SPI_SS)
    IO_PORT_IO_OUT_SOFT |= C_PORT_IO_OUT_SOFT_IO5_OUT;
#elif (PIN_FUNC_IO_6 == _SUPPORT_SPI_SS)
    IO_PORT_IO_OUT_SOFT |= C_PORT_IO_OUT_SOFT_IO6_OUT;
#elif (PIN_FUNC_IO_7 == _SUPPORT_SPI_SS)
    IO_PORT_IO_OUT_SOFT |= C_PORT_IO_OUT_SOFT_IO7_OUT;
#elif (PIN_FUNC_IO_8 == _SUPPORT_SPI_SS)
    IO_PORT_IO_OUT_SOFT |= C_PORT_IO_OUT_SOFT_IO8_OUT;
#elif (PIN_FUNC_IO_9 == _SUPPORT_SPI_SS)
    IO_PORT_IO_OUT_SOFT |= C_PORT_IO_OUT_SOFT_IO9_OUT;
#elif (PIN_FUNC_IO_10 == _SUPPORT_SPI_SS)
    IO_PORT_IO_OUT_SOFT |= C_PORT_IO_OUT_SOFT_IO10_OUT;
#elif (PIN_FUNC_IO_11 == _SUPPORT_SPI_SS)
    IO_PORT_IO_OUT_SOFT |= C_PORT_IO_OUT_SOFT_IO11_OUT;
#else  /* _SUPPORT_SPI_SS */
#error "ERROR: SPI (Soft) Slave Select I/O not support"
#endif /* _SUPPORT_SPI_SS */
} /* End of HAL_SPI_SS_Deactive() */

#endif /* (_SUPPORT_SPI != FALSE) */

#endif /* HAL_LIB_SPI_INLINE_H */

/* EOF */

