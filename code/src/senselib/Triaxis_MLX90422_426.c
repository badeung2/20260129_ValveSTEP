/*!*************************************************************************** *
 * \file        Triaxis_MLX90422_426.c
 * \brief       MLX90422 or MLX90426 Triaxis (SENT) support
 *
 * \note        project MLX813xx
 *
 * \author      Marcel Braat
 *
 * \date        2023-04-05 (MMP230405-1)
 *
 * \version     2.0
 *
 * A list of functions:
 *  - Global Functions:
 *           -# Triaxis_Init()
 *           -# Triaxis_Start()
 *           -# Triaxis_Stop()
 *           -# Triaxis_Data()
 *           -# GetTriaxisAngle()
 *           -# GetShaftAngle()
 *  - Internal Functions:
 *           -# ISR_CTIMER1_1()
 *           -# Sent_LegacyCRC()
 *           -# Sent_CRC()
 *
 * \copyright   MELEXIS Microelectronic Integrated Systems
 *
 * Copyright (C) 2023-2023 Melexis N.V.
 * The Software is being delivered 'AS IS' and Melexis, whether explicitly or
 * implicitly, makes no warranty as to its Use or performance.
 * The user accepts the Melexis Firmware License Agreement.
 *
 * Melexis confidential & proprietary
 *
 * *************************************************************************** */

/*!*************************************************************************** */
/*                           INCLUDES                                          */
/* *************************************************************************** */
#include "AppBuild.h"                                                           /* Application Build */

#if (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE)

#include "senselib/Triaxis_MLX90422_426.h"                                      /* SENT Triaxis MLX90422 and MLX90426 support */

#if (LIN_COMM != FALSE)
#include "commlib/LIN_Communication.h"                                          /* LIN support */
#include "drivelib/ErrorCodes.h"                                                /* Error-logging support */
#endif /* (LIN_COMM != FALSE) */
#include "drivelib/GlobalVars.h"                                                /* Global Variables support */
#include "drivelib/NV_UserPage.h"                                               /* Non Volatile Memory User Application Page Layout */
#include "camculib/private_mathlib.h"                                           /* Private Math Library support */

#include <atomic.h>
#include <sent_hal.h>                                                           /* Sent protocol support for HAL */
#include <string.h>

/*!*************************************************************************** */
/*                           DEFINITIONS                                       */
/* *************************************************************************** */
#define C_LIN_KEY                   0xB2A3U                                     /*!< LIN-key to access LIN-AA */

#if (_SUPPORT_TRIAXIS_MLX90422 != FALSE)
void Triaxis90422_ReceiveCallback(SentMessage_t* message);
#elif (_SUPPORT_TRIAXIS_MLX90426 != FALSE)
void Triaxis90426_ReceiveCallback(SentMessage_t* message);
#endif
void Triaxis_ErrorCallback(SentDiagnostic_t decodingResult);

/*!*************************************************************************** */
/*                           GLOBAL & LOCAL VARIABLES                          */
/* *************************************************************************** */
#pragma space dp                                                                /* __TINY_SECTION__ */
#pragma space none                                                              /* __TINY_SECTION__ */

#pragma space nodp                                                              /* __NEAR_SECTION__ */
static volatile uint8_t l_e8SentMessageReceived = (uint8_t)C_SENT_NONE;         /*!< SENT Message receive-state */
static volatile uint8_t l_au8SentMessage[C_SENT_MSG_LEN];                       /*!< SENT Message buffer */
uint16_t g_u16ShaftAngle = 0U;                                                  /*!< Shaft Angle; Triaxis angle corrected with Sense-magnet offset */
static uint16_t l_u16TriaxisAngleOffset = 0U;                                   /*!< Triaxis Angle Offset (between sense-magnet and shaft) */
static uint16_t l_u16TriaxisAngleRaw = 0U;                                      /*!< Triaxis Angle Raw uncorrected (sense-magnet) */
uint16_t g_u16TriaxisErrorCode = 0U;                                            /*!< Triaxis Error Code */
static uint8_t l_u8SenseMagnetPolePairs = 1U;                                   /*!< Sense magnet pole-pairs */
static uint8_t l_u8TriaxisDir = TRUE;                                           /*!< Triaxis direction opposite from actuator */
uint8_t g_u8TriaxisStatus = 0U;                                                 /*!< Triaxis Status */
static uint8_t l_u8RollingCounter = 0U;                                         /*!< Rolling Counter */

#if (_SUPPORT_SENT_HW_RESOURCE == C_SENT_PPM)
static PpmBuffer_t ppmBuffer;
const SentConfiguration_t SentConfigPPM = {
    .clocksPerTick = SENT_INIT_CLOCKS_PER_TICK(3u, FPLL / 1000u),               /* Tick-time: 3us */
    .nrOfDataNibbles = 6u,                                                      /* Number of data nibbles: 6 */
    .ppmIsrPrio = 4u,                                                           /* PPM IRQ priority: 4 */
    .mode = SPC_MODE_NONE,                                                      /* Mode: without SPC */
    .crcType = RECOMMENDED,
#if (_SUPPORT_SENSOR_SENT_IO == PIN_FUNC_LIN)
    .ppmInSel = ppm_IN_LIN_XRX,
#elif (_SUPPORT_SENSOR_SENT_IO == PIN_FUNC_IO_0)
    .ppmInSel = ppm_IN_IO_IN0,
#elif (_SUPPORT_SENSOR_SENT_IO == PIN_FUNC_IO_1)
    .ppmInSel = ppm_IN_IO_IN1,
#elif (_SUPPORT_SENSOR_SENT_IO == PIN_FUNC_IO_2)
    .ppmInSel = ppm_IN_IO_IN2,
#elif (_SUPPORT_SENSOR_SENT_IO == PIN_FUNC_IO_3)
    .ppmInSel = ppm_IN_IO_IN3,
#elif (_SUPPORT_SENSOR_SENT_IO == PIN_FUNC_IO_4)
    .ppmInSel = ppm_IN_IO_IN4,
#elif (_SUPPORT_SENSOR_SENT_IO == PIN_FUNC_IO_5)
    .ppmInSel = ppm_IN_IO_IN5,
#elif (_SUPPORT_SENSOR_SENT_IO == PIN_FUNC_IO_6)
    .ppmInSel = ppm_IN_IO_IN6,
#elif (_SUPPORT_SENSOR_SENT_IO == PIN_FUNC_IO_7)
    .ppmInSel = ppm_IN_IO_IN7,
#endif
    .ppmBufferPtr = &ppmBuffer,
#if (_SUPPORT_TRIAXIS_MLX90422 != FALSE)
    .receiveCallback = Triaxis90422_ReceiveCallback,
#elif (_SUPPORT_TRIAXIS_MLX90426 != FALSE)
    .receiveCallback = Triaxis90426_ReceiveCallback,
#endif
    .transmitCallback = SENT_NO_CALLBACK,
    .receivedTriggerPulseCallback = SENT_NO_CALLBACK,
    .transmittedTriggerPulseCallback = SENT_NO_CALLBACK,
    .errorCallback = Triaxis_ErrorCallback
};
#endif /* (_SUPPORT_SENT_HW_RESOURCE == C_SENT_PPM) */
#pragma space none                                                              /* __NEAR_SECTION__ */

#if (_SUPPORT_SENT_HW_RESOURCE == C_SENT_CTIMER1)
static const uint8_t CRC4Table[] =
{
    0,  1,  2,  3,  4,  5,  6,  7,                                              /* Index: 0x00 - 0x07 */
    8,  9, 10, 11, 12, 13, 14, 15,                                              /* Index: 0x08 - 0x0F */
    13, 12, 15, 14,  9,  8, 11, 10,                                             /* Index: 0x10 - 0x17 */
    5,  4,  7,  6,  1,  0,  3,  2,                                              /* Index: 0x18 - 0x1F */
    7,  6,  5,  4,  3,  2,  1,  0,                                              /* Index: 0x20 - 0x27 */
    15, 14, 13, 12, 11, 10,  9,  8,                                             /* Index: 0x28 - 0x2F */
    10, 11,  8,  9, 14, 15, 12, 13,                                             /* Index: 0x30 - 0x37 */
    2,  3,  0,  1,  6,  7,  4,  5,                                              /* Index: 0x38 - 0x3F */
    14, 15, 12, 13, 10, 11,  8,  9,                                             /* Index: 0x40 - 0x47 */
    6,  7,  4,  5,  2,  3,  0,  1,                                              /* Index: 0x48 - 0x4F */
    3,  2,  1,  0,  7,  6,  5,  4,                                              /* Index: 0x50 - 0x57 */
    11, 10,  9,  8, 15, 14, 13, 12,                                             /* Index: 0x58 - 0x5F */
    9,  8, 11, 10, 13, 12, 15, 14,                                              /* Index: 0x60 - 0x67 */
    1,  0,  3,  2,  5,  4,  7,  6,                                              /* Index: 0x68 - 0x6F */
    4,  5,  6,  7,  0,  1,  2,  3,                                              /* Index: 0x70 - 0x77 */
    12, 13, 14, 15,  8,  9, 10, 11,                                             /* Index: 0x78 - 0x7F */
    1,  0,  3,  2,  5,  4,  7,  6,                                              /* Index: 0x80 - 0x87 */
    9,  8, 11, 10, 13, 12, 15, 14,                                              /* Index: 0x88 - 0x8F */
    12, 13, 14, 15,  8,  9, 10, 11,                                             /* Index: 0x90 - 0x97 */
    4,  5,  6,  7,  0,  1,  2,  3,                                              /* Index: 0x98 - 0x9F */
    6,  7,  4,  5,  2,  3,  0,  1,                                              /* Index: 0xA0 - 0xA7 */
    14, 15, 12, 13, 10, 11,  8,  9,                                             /* Index: 0xA8 - 0xAF */
    11, 10,  9,  8, 15, 14, 13, 12,                                             /* Index: 0xB0 - 0xB7 */
    3,  2,  1,  0,  7,  6,  5,  4,                                              /* Index: 0xB8 - 0xBF */
    15, 14, 13, 12, 11, 10,  9,  8,                                             /* Index: 0xC0 - 0xC7 */
    7,  6,  5,  4,  3,  2,  1,  0,                                              /* Index: 0xC8 - 0xCF */
    2,  3,  0,  1,  6,  7,  4,  5,                                              /* Index: 0xD0 - 0xD7 */
    10, 11,  8,  9, 14, 15, 12, 13,                                             /* Index: 0xD8 - 0xDF */
    8,  9, 10, 11, 12, 13, 14, 15,                                              /* Index: 0xE0 - 0xE7 */
    0,  1,  2,  3,  4,  5,  6,  7,                                              /* Index: 0xE8 - 0xEF */
    5,  4,  7,  6,  1,  0,  3,  2,                                              /* Index: 0xF0 - 0xF7 */
    13, 12, 15, 14,  9,  8, 11, 10                                              /* Index: 0xF8 - 0xFF */
};
#endif /* (_SUPPORT_SENT_HW_RESOURCE == C_SENT_CTIMER1) */

#if (C_TRIAXIS_SENT_SLOW_CHANNEL != FALSE)
/*! Slow channel CRC6 helper table */
static const uint8_t crc6Table[] = {
    0, 25, 50, 43, 61, 36, 15, 22, 35, 58, 17,  8, 30,  7, 44, 53,
    31,  6, 45, 52, 34, 59, 16,  9, 60, 37, 14, 23,  1, 24, 51, 42,
    62, 39, 12, 21,  3, 26, 49, 40, 29,  4, 47, 54, 32, 57, 18, 11,
    33, 56, 19, 10, 28,  5, 46, 55,  2, 27, 48, 41, 63, 38, 13, 20
};

/*! Supported Enhanced Slow-channel information */
static const uint8_t au8EnhancedSlowChannelID[] =
{
    0x01U,                                                                      /*  0: Diagnostic error code; ESM status code */
    0x03U,                                                                      /*  1: Channel 1 / 2 Sensor type; SENT_SENSOR_TYPE[11:0] */
    0x05U,                                                                      /*  2: Manufacturer code; SENT_MAN_CODE[11:0] */
    0x06U,                                                                      /*  3: SENT standard revision; {8'd0, SENT_REV[3:0]} */
    0x07U,                                                                      /*  4: Fast channel 1: X1; SC_X1[15:4] */
    0x08U,                                                                      /*  5: Fast channel 1: X2; SC_X2[15:4] */
    0x09U,                                                                      /*  6: Fast channel 1: Y1; H.6: SC_Y1[13:2], H.7: SC_Y1[15:4], else SC_Y1[11:0] */
    0x0AU,                                                                      /*  7: Fast channel 1: Y2; H.6: SC_Y2[13:2], H.7: SC_Y2[15:4], else SC_Y2[11:0] */
    0x23U,                                                                      /*  8: (Internal) temperature; Current temperature */
    0x29U,                                                                      /*  9: Sensor ID #1; SENT_SENSOR_ID1[11:0] */
    0x2AU,                                                                      /* 10: Sensor ID #2; SENT_SENSOR_ID2[11:0] */
    0x2BU,                                                                      /* 11: Sensor ID #3; SENT_SENSOR_ID3[11:0] */
    0x2CU,                                                                      /* 12: Sensor ID #4; SENT_SENSOR_ID4[11:0] */
    0x80U,                                                                      /* 13: Magnetic field Norm; Field Strength corrected from RAM */
    0x90U,                                                                      /* 14: OEM Code #1; SENT_OEM_CODE1[11:0] */
    0x91U,                                                                      /* 15: OEM Code #2; SENT_OEM_CODE2[11:0] */
    0x92U,                                                                      /* 16: OEM Code #3; SENT_OEM_CODE3[11:0] */
    0x93U                                                                       /* 17: OEM Code #4; SENT_OEM_CODE4[11:0] */
};
#define C_SZ_ENHANCED_SLOW_CHANNELS     sizeof(au8EnhancedSlowChannelID)        /*!< Number of Enhanced Slow channels */
uint16_t au16EnhancedSlowChannelInfo[C_SZ_ENHANCED_SLOW_CHANNELS] = { 0 };      /*!< Enhanced Slow Channel Information array */
#if (LIN_COMM != FALSE)
uint16_t l_u16SlowChannelIdx = 0U;                                              /*!< Slow Channel table-index */
#endif /* (LIN_COMM != FALSE) */
#endif /* (C_TRIAXIS_SENT_SLOW_CHANNEL == 1U) */

/*!************************************************************************** *
 * Triaxis_Init
 * \brief   Initialise the triaxis Sent frame interface
 * \author  mmp
 * ************************************************************************** *
 * \param   -
 * \return  (uint16_t) u16Result: ERR_TRIAXIS_OK
 * ************************************************************************** *
 * \details
 * Initialise Timer-capture interface and Triaxis MLX90422/90426.
 * The Triaxis MLX90422/426 is connected between 5V and ground.
 *
 * --+  +------------------+  +------+  +------------+
 *   |  |                  |  |      |  |            |
 *   +--+                  +--+      +--+            +--
 *   <-------- 56T --------X-- 12T --X----- 27T ----->
 *     Synchronisation       Shortest  Longest nibble
 *   0b0000: 12T   0b0100: 16T   0b1000: 20T   0b1100: 24T
 *   0b0001: 13T   0b0101: 17T   0b1001: 21T   0b1101: 25T
 *   0b0010: 14T   0b0110: 18T   0b1010: 22T   0b1110: 26T
 *   0b0011: 15T   0b0111: 19T   0b1011: 23T   0b1111: 27T
 *   T = 3.0us (Normal SENT), or T = 6.0us (Slow SENT)
 *   Falling-edge to falling-edge.
 *
 * --+  +---------------+  +--+  +----+  +--+  +----+  +--+  +--+  +--+  +--+
 *   |  |               |  |  |  |    |  |  |  |    |  |  |  |  |  |  |  |  |
 *   +--+               +--+  +--+    +--+  +--+    +--+  +--+  +--+  +--+  +-
 *   < Synchronisation >< #1 ><- #2 ->< #3 ><- #4 ->< #5 >< #6 >< #7 >< #8 >
 *   #1: Status
 *       bit 0: Channel 1 indicator
 *       bit 1: Channel 2 indicator (0 = Rolling-counter + Inverted Ch1 MSN)
 *                                  (1 = Channel 2)
 *       bit 2: Enhanced Serial Message
 *       bit 3: Enhanced Serial Message
 *   #2: Channel 1 - Most Significant Nibble (12-bit angle)
 *   #3: Channel 1 - Middle Nibble
 *   #4: Channel 1 - Least Significant Nibble
 *   #5: Channel 2 - Least Significant Nibble (0: Rolling-counter MSN)
 *                                           (1: Inverted Channel 1 LSN)
 *   #6: Channel 2 - Middle Nibble (0: rolling-counter LSN)
 *                                 (1: Inverted Channel 1 MN)
 *   #7: Channel 2 - Most Significant Nibble (x: Inverted Channel 1 MSN)
 *   #8: CRC
 *   Note: Data 1 is MSN to LSN, Data 2 is LSN to MSN.
 * ************************************************************************** *
 * - Call Hierarchy: main_init
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * ************************************************************************** */
uint16_t Triaxis_Init(void)
{
    l_u16TriaxisAngleOffset = (int16_t)NV_TRIAXIS_ANGLE_OFF;
    l_u8SenseMagnetPolePairs = NV_SENSE_POLE_PAIRS;
    l_u8TriaxisDir = NV_TRIAXIS_DIRECTION;

#if (_SUPPORT_SENT_HW_RESOURCE == C_SENT_CTIMER1)
    IO_CTIMER1_CTRL = B_CTIMER1_STOP;
    IO_CTIMER1_CTRL = C_CTIMER1_DIV_CPU |                                       /* Timer-divider 1:1 */
                      C_CTIMER1_MODE_DUAL_CAPT |                                /* Timer mode: Dual capture */
                      C_CTIMER1_EDGA_FALLING |                                  /* Channel A edge: Falling */
                      C_CTIMER1_EDGB_RAISING;                                   /* Channel B edge: Rising */

#if (_SUPPORT_SENSOR_SENT_IO == PIN_FUNC_IO_1)
    /* Configure IO[1] as SENT-interface */
    IO_PORT_TIMER_CFG1 = (IO_PORT_TIMER_CFG1 & ~(M_PORT_TIMER_CFG1_TIMER1_CHA_SEL |
                                                 M_PORT_TIMER_CFG1_TIMER1_CHB_SEL)) |
                         (C_PORT_TIMER_CFG1_TIMER1_CHA_SEL_IO1 |
                          C_PORT_TIMER_CFG1_TIMER1_CHB_SEL_IO1);
#elif (_SUPPORT_SENSOR_SENT_IO == PIN_FUNC_IO_2)
    /* Configure IO[2] as SENT-interface */
    IO_PORT_TIMER_CFG1 = (IO_PORT_TIMER_CFG1 & ~(M_PORT_TIMER_CFG_TIMER1_CHA_SEL |
                                                 M_PORT_TIMER_CFG_TIMER1_CHB_SEL)) |
                         (C_PORT_TIMER_CFG1_TIMER1_CHA_SEL_IO2 |
                          C_PORT_TIMER_CFG1_TIMER1_CHB_SEL_IO2);
#elif (_SUPPORT_SENSOR_SENT_IO == PIN_FUNC_IO_3)
    /* Configure IO[3] as SENT-interface */
    IO_PORT_TIMER_CFG1 = (IO_PORT_TIMER_CFG1 & ~(M_PORT_TIMER_CFG1_TIMER1_CHA_SEL |
                                                 M_PORT_TIMER_CFG1_TIMER1_CHB_SEL)) |
                         (C_PORT_TIMER_CFG1_TIMER1_CHA_SEL_IO3 |
                          C_PORT_TIMER_CFG1_TIMER1_CHB_SEL_IO3);
#else
#error "ERROR: SENT-interface IO not supported"
#endif

    /* Configure CTimer1 Interrupt Service */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    IO_MLX16_ITC_PRIO0_S = (IO_MLX16_ITC_PRIO0_S & ~M_MLX16_ITC_PRIO0_CTIMER1_1) |
                           C_MLX16_ITC_PRIO0_CTIMER1_1_PRIO4;
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#elif (_SUPPORT_SENT_HW_RESOURCE == C_SENT_PPM)
#if (_SUPPORT_SENSOR_SENT_IO == PIN_FUNC_LIN)
    IO_PORT_LIN_XTX_CFG = (IO_PORT_LIN_XTX_CFG & ~M_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL) |
                          C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_PPM_OUT;
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
#elif (_SUPPORT_SENSOR_SENT_IO == PIN_FUNC_IO_0)
    IO_PORT_IO_ENABLE = (IO_PORT_IO_ENABLE & ~(B_PORT_IO_ENABLE_IO_ENABLE_0 |   /* Disable IO0 output (Select input) */
                                               B_PORT_IO_ENABLE_IO_DISREC_0));  /* Enable IO0 weak pull-up */
#if (_SUPPORT_SENSOR_VDDA_5V != FALSE)
    IO_PORT_IO_OUT_EN |= B_PORT_IO_OUT_EN_IO0_LV_ENABLE;                        /* Supplied via VDDA @ 5V */
#else  /* (_SUPPORT_SENSOR_VDDA_5V != FALSE) */
    IO_PORT_IO_OUT_EN &= ~B_PORT_IO_OUT_EN_IO0_LV_ENABLE;                       /* Supplied via External @ 5V */
#endif /* (_SUPPORT_SENSOR_VDDA_5V != FALSE) */
#elif (_SUPPORT_SENSOR_SENT_IO == PIN_FUNC_IO_1)
    IO_PORT_IO_ENABLE = (IO_PORT_IO_ENABLE & ~B_PORT_IO_ENABLE_IO_ENABLE_1) |   /* Disable IO1 output (Select input) */
                        B_PORT_IO_ENABLE_IO_DISREC_1;                           /* Disable IO1 weak pull-up */
#elif (_SUPPORT_SENSOR_SENT_IO == PIN_FUNC_IO_2)
    IO_PORT_IO_ENABLE = (IO_PORT_IO_ENABLE & ~B_PORT_IO_ENABLE_IO_ENABLE_2) |   /* Disable IO2 output (Select input) */
                        B_PORT_IO_ENABLE_IO_DISREC_2;                           /* Disable IO2 weak pull-up */
#elif (_SUPPORT_SENSOR_SENT_IO == PIN_FUNC_IO_3)
    IO_PORT_IO_ENABLE = (IO_PORT_IO_ENABLE & ~B_PORT_IO_ENABLE_IO_ENABLE_3);    /* Disable IO3 output (Select input) */
#elif (_SUPPORT_SENSOR_SENT_IO == PIN_FUNC_IO_4)
    IO_PORT_IO_ENABLE = (IO_PORT_IO_ENABLE & ~B_PORT_IO_ENABLE_IO_ENABLE_4) |   /* Disable IO4 output (Select input) */
                        B_PORT_IO_ENABLE_IO_DISREC_4;                           /* Disable IO4 weak pull-up */
#elif (_SUPPORT_SENSOR_SENT_IO == PIN_FUNC_IO_5)
    IO_PORT_IO_ENABLE = (IO_PORT_IO_ENABLE & ~B_PORT_IO_ENABLE_IO_ENABLE_5) |   /* Disable IO5 output (Select input) */
                        B_PORT_IO_ENABLE_IO_DISREC_5;                           /* Disable IO5 weak pull-up */
#elif (_SUPPORT_SENSOR_SENT_IO == PIN_FUNC_IO_6)
    IO_PORT_IO_ENABLE = (IO_PORT_IO_ENABLE & ~B_PORT_IO_ENABLE_IO_ENABLE_6) |   /* Disable IO6 output (Select input) */
                        B_PORT_IO_ENABLE_IO_DISREC_6;                           /* Disable IO6 weak pull-up */
#elif (_SUPPORT_SENSOR_SENT_IO == PIN_FUNC_IO_7)
    IO_PORT_IO_ENABLE = (IO_PORT_IO_ENABLE & ~B_PORT_IO_ENABLE_IO_ENABLE_7) |   /* Disable IO7 output (Select input) */
                        B_PORT_IO_ENABLE_IO_DISREC_7;                           /* Disable IO7 weak pull-up */
#endif
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    sentReceiveInit(&SentConfigPPM);
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#endif
    return (ERR_TRIAXIS_OK);
} /* End of Triaxis_Init() */

/*!************************************************************************** *
 * Triaxis_Start
 * \brief   Start receiving Triaxis Sent frames.
 * \author  mmp
 * ************************************************************************** *
 * \param   -
 * \return  -
 * ************************************************************************** *
 * \details
 * ************************************************************************** *
 * - Call Hierarchy: main_init
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * ************************************************************************** */
void Triaxis_Start(void)
{
#if (_SUPPORT_SENT_HW_RESOURCE == C_SENT_CTIMER1)
    (void)IO_CTIMER1_TREGA;                                                     /* Dummy Read - Otherwise OVRA INT */
    (void)IO_CTIMER1_TREGB;                                                     /* Dummy Read - Otherwise OVRB INT */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    IO_MLX16_ITC_PEND1_S = B_MLX16_ITC_PEND1_CTIMER1_1;
    IO_MLX16_ITC_MASK1_S |= B_MLX16_ITC_MASK1_CTIMER1_1;
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    IO_CTIMER1_CTRL = B_CTIMER1_START;
    (void)IO_CTIMER1_TREGA;                                                     /* Dummy Read - Otherwise OVRA INT */
    (void)IO_CTIMER1_TREGB;                                                     /* Dummy Read - Otherwise OVRB INT */
#elif (_SUPPORT_SENT_HW_RESOURCE == C_SENT_PPM)
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    sentReceiveStart();
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#endif
} /* End of Triaxis_Start() */

/*!************************************************************************** *
 * Function : Triaxis_Stop
 * Purpose  : Stop receiving Triaxis Sent frames.
 * Author   : mmp
 * ************************************************************************** *
 * \param   -
 * \return  -
 * ************************************************************************** *
 * \details
 * ************************************************************************** *
 * - Call Hierarchy: main_init
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * ************************************************************************** */
void Triaxis_Stop(void)
{
#if (_SUPPORT_SENT_HW_RESOURCE == C_SENT_CTIMER1)
    IO_CTIMER1_CTRL = B_CTIMER1_STOP;
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    IO_MLX16_ITC_MASK1_S &= ~B_MLX16_ITC_MASK1_CTIMER1_1;
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#elif (_SUPPORT_SENT_HW_RESOURCE == C_SENT_PPM)
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    sentStop();
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#endif
} /* End of Triaxis_Stop() */

#if (_SUPPORT_SENT_HW_RESOURCE == C_SENT_CTIMER1)
/*!************************************************************************** *
 * ISR_CTIMER1_1
 * \brief   Timer Capture Interrupt Handler
 * \author  mmp
 * ************************************************************************** *
 * \param   -
 * \return  -
 * ************************************************************************** *
 * \details Sync-pulse: 10.3 [us]
 *          Data nibble: 10.0 [us]
 *          CRC nibble: 10.0 [us]
 *          Overflow ISR: 3.15 [us]
 *          CPU-load: (10.3 + 7*10.0 + 10.0) [us]/ 1 [ms] = 9.0%
 *          IRQ-Priority: 4
 * ************************************************************************** *
 * - Call Hierarchy: IRQ
 * - Cyclomatic Complexity: 6+1
 * - Nesting: 5
 * - Function calling: 0
 * ************************************************************************** */
__attribute__((interrupt)) void ISR_CTIMER1_1(void)
{
#if (_DEBUG_CPU_LOAD != FALSE)
    DEBUG_SET_IO_B();                                                           /* IRQ-Priority 4 */
#endif /* (_DEBUG_CPU_LOAD != FALSE) */
#if (_DEBUG_TRIAXIS != FALSE)
    DEBUG_SET_IO_A();
#endif /* (_DEBUG_TRIAXIS != FALSE) */
    static uint16_t u16LastSentFallingEdge = 0U;
    static uint16_t u16TickPeriod = (C_TICK * (PLL_FREQ / 1000000UL));
    static uint16_t u16MinNibblePeriod = (C_TICK * (PLL_FREQ / 1000000UL)) * C_MIN_DATA_NIBBLE_TICKS;
    static uint16_t u16MaxNibblePeriod = (C_TICK * (PLL_FREQ / 1000000UL)) * C_MAX_DATA_NIBBLE_TICKS;
    static uint8_t *pu8SentNibble = 0U;

    /* Falling edge */
    uint16_t u16TmrAReg = IO_CTIMER1_TREGA;
    /* uint16_t u16TmrBReg = CTIMER_REGB; */ /* TODO[MMP] use to check valid pulse ? */
    uint16_t u16SentNibblePeriod = u16TmrAReg - u16LastSentFallingEdge;
    u16LastSentFallingEdge = u16TmrAReg;

    if (l_e8SentMessageReceived != (uint8_t)C_SENT_MSG)
    {
        if ( (u16SentNibblePeriod > C_MIN_SYNC_PERIOD_CNT) &&
             (u16SentNibblePeriod < C_MAX_SYNC_PERIOD_CNT) )
        {
            /* Valid Sync period */
            u16SentNibblePeriod += (C_TYP_SYNC_PERIOD_TICKS / 2U);
            u16TickPeriod = p_DivU16_U32byU16( (uint32_t)u16SentNibblePeriod, C_TYP_SYNC_PERIOD_TICKS);
            u16MinNibblePeriod = p_MulU16lo_U16byU16lsr1(u16TickPeriod, ((C_MIN_DATA_NIBBLE_TICKS * 2U) - 1U));
            u16MaxNibblePeriod = p_MulU16lo_U16byU16lsr1(u16TickPeriod, ((C_MAX_DATA_NIBBLE_TICKS * 2U) + 1U));
            pu8SentNibble = (uint8_t *)&l_au8SentMessage[0U];
            l_e8SentMessageReceived = (uint8_t)C_SENT_SYNC;
        }
        else if ( (l_e8SentMessageReceived != (uint8_t)C_SENT_NONE) &&
                  (u16SentNibblePeriod >= u16MinNibblePeriod) &&
                  (u16SentNibblePeriod <= u16MaxNibblePeriod) )
        {
            /* Valid Nibble period; Translate to nibble-data */
            u16SentNibblePeriod -= u16MinNibblePeriod;
            *pu8SentNibble = (uint8_t)p_DivU16_U32byU16( (uint32_t)u16SentNibblePeriod, u16TickPeriod);
            if (pu8SentNibble >= &l_au8SentMessage[C_SENT_MSG_LEN - 1U])
            {
                l_e8SentMessageReceived = (uint8_t)C_SENT_MSG;
            }
            else
            {
                pu8SentNibble++;
            }
        }
        else
        {
            /* Invalid nibble period */
            l_e8SentMessageReceived = (uint8_t)C_SENT_NONE;
        }
    }
#if (_DEBUG_TRIAXIS != FALSE)
    DEBUG_CLR_IO_A();
#endif /* (_DEBUG_TRIAXIS != FALSE) */
#if (_DEBUG_CPU_LOAD != FALSE)
    DEBUG_CLR_IO_B();                                                           /* IRQ-Priority 4 */
#endif /* (_DEBUG_CPU_LOAD != FALSE) */
} /* End of ISR_CTIMER1_1() */
#elif (_SUPPORT_SENT_HW_RESOURCE == C_SENT_PPM)

/*!************************************************************************** *
 * ISR_PPM_ERR
 * \brief   PPM Error interrupt handler
 * \author  mmp
 * ************************************************************************** *
 * \param   -
 * \return  -
 * ************************************************************************** *
 * \details The PPM Error Interrupt handle is used to restart SENT receiving.
 *          The Error interrupt is given after a time-out (PPM header).
 *          Performance @ 32MHz: 33.7-36.3 us
 * ************************************************************************** *
 * - Call Hierarchy: IRQ
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 1 (sentReceiveInterruptHandler())
 * ************************************************************************** */
__attribute__((interrupt)) void ISR_PPM_ERR(void)
{
#if (_DEBUG_TRIAXIS != FALSE)
    DEBUG_SET_IO_B();
#endif /* (_DEBUG_TRIAXIS != FALSE) */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    sentReceiveInterruptHandler(&SentConfigPPM);
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#if (_DEBUG_TRIAXIS != FALSE)
    DEBUG_CLR_IO_B();
#endif /* (_DEBUG_TRIAXIS != FALSE) */
} /* End of PPM_ERR_INT() */

#if (_SUPPORT_TRIAXIS_MLX90422 != FALSE)
/*!************************************************************************** *
 * Triaxis90422_ReceiveCallback
 * \brief   Sent Receive Call-back function for Triaxis 90422
 * \author  mmp
 * ************************************************************************** *
 * \param   message: Pointer to sent-message of nibbles
 * \return  -
 * ************************************************************************** *
 * \details
 * ************************************************************************** *
 * - Call Hierarchy: Call-back
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 2
 * - Function calling: 0
 * ************************************************************************** */
void Triaxis90422_ReceiveCallback(SentMessage_t* message)
#elif (_SUPPORT_TRIAXIS_MLX90426 != FALSE)
/*!************************************************************************** *
 * Triaxis90426_ReceiveCallback
 * \brief   Sent Receive Call-back function for Triaxis 90426
 * \author  mmp
 * ************************************************************************** *
 * \param   message: Pointer to sent-message of nibbles
 * \return  -
 * ************************************************************************** *
 * \details
 * ************************************************************************** *
 * - Call Hierarchy: Call-back
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 2
 * - Function calling: 0
 * ************************************************************************** */
void Triaxis90426_ReceiveCallback(SentMessage_t* message)
#endif
{
    /* Sensor shall be configured to transmit single secure sensor A.3
     * message[0] = status nibble
     * message[1] = Ch1[11:8]
     * message[2] = Ch1[7:4]
     * message[3] = Ch1[3:0]
     * message[4] = COUNT[7:4]
     * message[5] = COUNT[3:0]
     * message[6] = ~Ch1[11:8]
     * message[7] = CRC
     */
    memcpy( (void *)&l_au8SentMessage[0], (const void *)message, 8U);
    l_e8SentMessageReceived = (uint8_t)C_SENT_MSG;
} /* End of Triaxis9042X_ReceiveCallback() */

void Triaxis_ErrorCallback(SentDiagnostic_t decodingResult)
{
    if (decodingResult == SENT_MESSAGE_CRC_ERROR)
    {
        l_e8SentMessageReceived = (uint8_t)C_SENT_ERROR_CRC;
    }
    else /* SENT_MESSAGE_ERROR */
    {
        l_e8SentMessageReceived = (uint8_t)C_SENT_ERROR_OTHER;
    }
} /* End of Triaxis_ErrorCallback() */
#endif

#if (_SUPPORT_SENT_HW_RESOURCE == C_SENT_CTIMER1)
/*!************************************************************************** *
 * Sent_LegacyCRC
 * \brief   Calculate the crc for a sent message
 * \author  mmp
 * ************************************************************************** *
 * \param   const uint8_t * pMessage: Pointer to the array holding the
 *                                    message's nibbles
 *          uint8_t u8NrOfNibbles: Total number of nibbles in this message
 * \return  (uint8_t) Calculated CRC value
 * ************************************************************************** *
 * \details This function will calculate the CRC for a sent message using
 *          the Legacy method.
 * ************************************************************************** *
 * - Call Hierarchy: -
 * - Cyclomatic Complexity: 6+1
 * - Nesting: 4
 * - Function calling: 0
 * ************************************************************************** */
static uint8_t Sent_LegacyCRC(const uint8_t *pu8Message, uint8_t u8NrOfNibbles)
{
    uint8_t *pu8MsgEnd = (uint8_t *)(pu8Message + u8NrOfNibbles);
    uint8_t u8Crc = 0x05U;

    /* CRC is calculated excluding status and CRC nibble */
    while (pu8Message < pu8MsgEnd)
    {
        u8Crc = (u8Crc * 16U) + *pu8Message++;
        u8Crc = CRC4Table[u8Crc];
    }

    return (u8Crc);
} /* End of Sent_LegacyCRC() */

#if (_SUPPORT_SENT_LEGACY_CRC == FALSE)
/*!************************************************************************** *
 * Sent_CRC
 * \brief   Calculate the crc for a sent message
 * \author  mmp
 * ************************************************************************** *
 * \param   const uint8_t * pMessage: Pointer to the array holding the
 *                                    message's nibbles
 *          uint8_t u8NrOfNibbles: Total number of nibbles in this message
 * \return  (uint8_t) Calculated CRC value
 * ************************************************************************** *
 * \details This function will calculate the CRC for a sent message using
 *          the recommended method.
 * ************************************************************************** *
 * - Call Hierarchy: Triaxis_Data
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 0
 * ************************************************************************** */
static uint8_t Sent_CRC(const uint8_t *pu8Message, uint8_t u8NrOfNibbles)
{
    uint8_t u8Crc = Sent_LegacyCRC(pu8Message, u8NrOfNibbles);

    /* checksum with an extra "0" value */
    u8Crc = CRC4Table[u8Crc * 16U];

    return (u8Crc);
} /* End of Sent_CRC() */
#endif /* (_SUPPORT_SENT_LEGACY_CRC == FALSE) */
#endif /* (_SUPPORT_SENT_HW_RESOURCE == C_SENT_CTIMER1) */

#if (C_TRIAXIS_SENT_SLOW_CHANNEL != FALSE)
/*!************************************************************************** *
 * Triaxis_GetSlowChannelInfo
 * \brief   Retrieve Slow-channel information
 * \author  mmp
 * ************************************************************************** *
 * \param   [in] u16SerialID : Slow channel ID
 * \return  (uint16_t) Slow channel information (12-bits)
 * ************************************************************************** *
 * \details This function return the collected Enhanced Slow Channel information
 * ************************************************************************** *
 * - Call Hierarchy: -
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 2
 * - Function calling: 0
 * ************************************************************************** */
uint16_t Triaxis_GetSlowChannelInfo(uint16_t u16SerialID)
{
    uint16_t u16Idx;
    uint16_t u16Result = 0U;

    for (u16Idx = 0U;  u16Idx < C_SZ_ENHANCED_SLOW_CHANNELS; u16Idx++)
    {
        if (au8EnhancedSlowChannelID[u16Idx] == (uint8_t) u16SerialID)
        {
            u16Result = au16EnhancedSlowChannelInfo[u16Idx];
            break;
        }
    }
    return (u16Result);
} /* End of Triaxis_GetSlowChannelInfo) */

/*!************************************************************************** *
 * Triaxis_SlowChannelReceiveCallback
 * \brief   Collect Inductive Position Sensor Slow channel information
 * \author  mmp
 * ************************************************************************** *
 * \param   [in] u16SerialID : Slow channel ID
 * \param   [in] u16SerialData : Slow channel information (12-bits)
 * \return  -
 * ************************************************************************** *
 * \details This function will collect all the Enhanced Slow Channel information
 * ************************************************************************** *
 * - Call Hierarchy: Sent_DecodeEnhancedSlowChannel
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 2
 * - Function calling: 0
 * ************************************************************************** */
void Triaxis_SlowChannelReceiveCallback(uint16_t u16SerialID, uint16_t u16SerialData)
{
    uint16_t u16Idx;

    for (u16Idx = 0U;  u16Idx < C_SZ_ENHANCED_SLOW_CHANNELS; u16Idx++)
    {
        if (au8EnhancedSlowChannelID[u16Idx] == (uint8_t) u16SerialID)
        {
            au16EnhancedSlowChannelInfo[u16Idx] = u16SerialData;
        }
    }
} /* End of Triaxis_SlowChannelReceiveCallback) */

/*!************************************************************************** *
 * Sent_DecodeEnhancedSlowChannel
 * \brief   Decode Enhanced Slow Channel information
 * \author  nio
 * ************************************************************************** *
 * \param   [in] bit2: SENT Status-bit2
 * \param   [in] bit3: SENT Status-bit3
 * \return  -
 * ************************************************************************** *
 * \details This function will decode the Enhanced Slow Channel information of SENT
 * ************************************************************************** *
 * - Call Hierarchy: InductivePosSensor_Data
 * - Cyclomatic Complexity: 5+1
 * - Nesting: 4
 * - Function calling: 0
 * ************************************************************************** */
static inline void Sent_DecodeEnhancedSlowChannel(uint8_t bit2, uint8_t bit3)
{
    static uint16_t serialDataCrc;
    static uint16_t serialDataBit2;
    static uint16_t serialDataBit3;
    static uint32_t serialMessage;
    static uint8_t serialDataCtr = 0u;

    serialMessage <<= 2;
    serialMessage |= (bit2 << 1) | bit3;
    serialDataBit2 <<= 1;
    serialDataBit2 |= bit2;
    serialDataBit3 <<= 1;
    serialDataBit3 |= bit3;
    serialDataCtr++;

    if (((serialDataCtr < 7u) && (bit3 == 0u)) ||
        (((serialDataCtr == 7u) || (serialDataCtr == 13u) || (serialDataCtr == 18u)) && (bit3 != 0u)))
    {
        /* something went wrong with the synchronisation */
        serialDataCtr = 0u;
    }
    else if (serialDataCtr == 6u)
    {
        /* first 6 bits are correctly received, copy the crc field */
        serialDataCrc = serialDataBit2 & 0x3Fu;
    }
    else if (serialDataCtr == 18u)
    {
        /* complete serial data message received */
        uint16_t serialId;
        uint16_t serialData;

        serialData = serialDataBit2 & 0xFFFu;
        if ((serialDataBit3 & 0x400u) != 0u)
        {
            /* 16-bit data and 4-bit message ID (configuration bit = 1) */
            serialId = (serialDataBit3 >> 6) & 0xFu;
            serialData |= ((serialDataBit3 >> 1) & 0x0Fu) << 12;
        }
        else
        {
            /* 12-bit data and 8-bit message ID (configuration bit = 0) */
            serialId = ((serialDataBit3 >> 2) & 0xF0u) | ((serialDataBit3 >> 1) & 0x0Fu);
        }

        uint16_t checksum64 = 0x15;    /* initialise checksum */
        checksum64 = ((uint16_t)(serialMessage >> 18) & 0x3Fu) ^ (uint16_t)crc6Table[checksum64];
        checksum64 = ((uint16_t)(serialMessage >> 12) & 0x3Fu) ^ (uint16_t)crc6Table[checksum64];
        checksum64 = ((uint16_t)(serialMessage >> 6) & 0x3Fu) ^ (uint16_t)crc6Table[checksum64];
        checksum64 = ((uint16_t)(serialMessage >> 0) & 0x3Fu) ^ (uint16_t)crc6Table[checksum64];
        /* checksum with an extra "0" value (message is augmented by six zeros) */
        checksum64 = 0u ^ (uint16_t)crc6Table[checksum64];

        if (checksum64 == serialDataCrc)
        {
            Triaxis_SlowChannelReceiveCallback(serialId, serialData);
        }

        serialDataCtr = 0u;
    }
} /* End of Sent_DecodeEnhancedSlowChannel() */
#endif /* (C_TRIAXIS_SENT_SLOW_CHANNEL != FALSE) */

/*!************************************************************************** *
 * Triaxis_Data
 * \brief   Get Triaxis Data (angle or error-code)
 * \author  mmp
 * ************************************************************************** *
 * \param   -
 * \return  (uint16_t) Error-code
 *          ERR_TRIAXIS_OK: No error (Angle in l_u16TriaxisAngleRaw)
 *          ERR_TRIAXIS_CRC: Sent CRC error
 *          ERR_TRIAXIS_DIAGNOSTIC_INFO: Triaxis Error code received
 *          ERR_TRIAXIS_INVREPLY: Sent-Frame incorrect (nibble 6)
 *          ERR_TRIAXIS_NODATA: No data
 *          ERR_TRIAXIS_UNKNOWN: Unknown
 * ************************************************************************** *
 * \details CPU-load: 23.45 [us]/ 1 [ms] = 2.35%
 * ************************************************************************** *
 * - Call Hierarchy: main
 * - Cyclomatic Complexity: 6+1
 * - Nesting: 4
 * - Function calling: 0
 * ************************************************************************** */
uint16_t Triaxis_Data(void)
{
    uint16_t u16Result;

    if (l_e8SentMessageReceived == (uint8_t)C_SENT_MSG)
    {
#if (_DEBUG_TRIAXIS != FALSE)
        DEBUG_SET_IO_B();
#endif /* (_DEBUG_TRIAXIS != FALSE) */

#if (_SUPPORT_SENT_HW_RESOURCE == C_SENT_CTIMER1)
        /* Check CRC */
#if (_SUPPORT_SENT_LEGACY_CRC != FALSE)
        /* Skip Status-byte [0] and CRC */
        if (Sent_LegacyCRC( (const uint8_t *)&l_au8SentMessage[1],
                            (C_SENT_MSG_LEN - 2U)) != l_au8SentMessage[C_SENT_MSG_LEN - 1U])
#else  /* (_SUPPORT_SENT_LEGACY_CRC != FALSE) */
        /* Skip Status-byte [0] and CRC */
        if (Sent_CRC( (const uint8_t *)&l_au8SentMessage[1],
                      (C_SENT_MSG_LEN - 2U)) != l_au8SentMessage[C_SENT_MSG_LEN - 1U])
#endif /* (_SUPPORT_SENT_LEGACY_CRC != FALSE) */
        {
            u16Result = ERR_TRIAXIS_CRC;
        }
        else
#endif /* (_SUPPORT_SENT_HW_RESOURCE == C_SENT_CTIMER1) */
#if (C_TRIAXIS_SENT_FRAME_FORMAT == 1U)
        {
            /* A.1. Throttle position */
            uint16_t u16Data1 = (((uint16_t)l_au8SentMessage[1]) << 8) |        /* Most significant Nibble */
                                (((uint16_t)l_au8SentMessage[2]) << 4) |
                                ((uint16_t)l_au8SentMessage[3]);                /* Least significant Nibble */
            uint16_t u16Data2 = (((uint16_t)l_au8SentMessage[6]) << 8) |        /* Most significant Nibble */
                                (((uint16_t)l_au8SentMessage[5]) << 4) |
                                ((uint16_t)l_au8SentMessage[4]);                /* Least significant Nibble */
            g_u8TriaxisStatus = l_au8SentMessage[0];
            if ( (g_u8TriaxisStatus & 0x03U) == 0x00U)
            {
                /* Valid data for Ch1 and Ch2 */
                if ( (u16Data1 ^ u16Data2) == 0x0FFFU)                          /* Check data matches */
                {
                    l_u16TriaxisAngleRaw = (u16Data1 << 4);                     /* Convert 12-bit angle to 16-bit */
                    if (l_u8TriaxisDir != FALSE)
                    {
                        /* Triaxis has opposite rotational direction as actuator */
                        g_u16ShaftAngle = l_u16TriaxisAngleOffset - l_u16TriaxisAngleRaw;
                    }
                    else
                    {
                        /* Triaxis has same rotational direction as actuator */
                        g_u16ShaftAngle = l_u16TriaxisAngleRaw - l_u16TriaxisAngleOffset;
                    }
                    u16Result = ERR_TRIAXIS_OK;
                }
                else
                {
                    /* Wrong inverse data */
                    u16Result = ERR_TRIAXIS_INVREPLY;
                }
            }
            else
            {
                /* Status indicate error */
                if ( (g_u8TriaxisStatus & 0x01U) != 0x00U)
                {
                    /* Channel 1 has error information */
                    g_u16TriaxisErrorCode = u16Data1;                           /* 12-bit error-code */
                    u16Result = ERR_TRIAXIS_DIAGNOSTIC_INFO;
                }
                else if ( (g_u8TriaxisStatus & 0x02U) != 0x00U)
                {
                    g_u16TriaxisErrorCode = u16Data2;                           /* 12-bit error-code */
                    u16Result = ERR_TRIAXIS_DIAGNOSTIC_INFO;
                }
            }

#if (C_TRIAXIS_SENT_SLOW_CHANNEL != FALSE)
            /* Slow Channel decoding */
            {
                uint8_t u8Bit2 = ((l_au8SentMessage[0] & 0x04U) != 0x00U) ? 0x01U : 0x00U;
                uint8_t u8Bit3 = ((l_au8SentMessage[0] & 0x08U) != 0x00U) ? 0x01U : 0x00U;
                Sent_DecodeEnhancedSlowChannel(u8Bit2, u8Bit3);
            }
#endif /* (C_TRIAXIS_SENT_SLOW_CHANNEL != FALSE) */
        }
#elif (C_TRIAXIS_SENT_FRAME_FORMAT == 3U)
        {
            /* A.3. Single secure sensor (default) */
            uint16_t u16Data1 = (((uint16_t)l_au8SentMessage[1]) << 8) |        /* Most significant Nibble */
                                (((uint16_t)l_au8SentMessage[2]) << 4) |
                                ((uint16_t)l_au8SentMessage[3]);                /* Least significant Nibble */
            uint8_t u8RollingCounter = (l_au8SentMessage[4] << 4) | l_au8SentMessage[5];
            g_u8TriaxisStatus = l_au8SentMessage[0];
            if ( (g_u8TriaxisStatus & 0x01U) == 0x00U)
            {
                /* Channel 1 indicator "Info" */
                if ( (l_au8SentMessage[1] ^ l_au8SentMessage[6]) == 0x0FU)      /* Check Nibble 6 is inverted Nibble 1 */
                {
                    l_u16TriaxisAngleRaw = (u16Data1 << 4);                     /* Convert 12-bit angle to 16-bit */
                    if (l_u8TriaxisDir != FALSE)
                    {
                        /* Triaxis has opposite rotational direction as actuator */
                        g_u16ShaftAngle = l_u16TriaxisAngleOffset - l_u16TriaxisAngleRaw;
                    }
                    else
                    {
                        /* Triaxis has same rotational direction as actuator */
                        g_u16ShaftAngle = l_u16TriaxisAngleRaw - l_u16TriaxisAngleOffset;
                    }
                    u16Result = ERR_TRIAXIS_OK;
#if (_DEBUG_TRIAXIS != FALSE)
                    DEBUG_CLR_IO_A();
#endif /* (_DEBUG_TRIAXIS != FALSE) */
#if FALSE           /* TODO[MMP]: Remove; For Test Purpose Only */
                    extern uint16_t g_u16ActualPosition;
                    g_u16ActualPosition = g_u16ShaftAngle;
#endif
                }
                else
                {
                    /* Wrong inverse data */
                    u16Result = ERR_TRIAXIS_INVREPLY;
#if (_DEBUG_TRIAXIS != FALSE)
                    DEBUG_SET_IO_A();
#endif /* (_DEBUG_TRIAXIS != FALSE) */
                }
            }
            else
            {
                /* Channel 1 has error information */
                g_u16TriaxisErrorCode = u16Data1;                               /* 12-bit error-code */
                u16Result = ERR_TRIAXIS_DIAGNOSTIC_INFO;
#if (_DEBUG_TRIAXIS != FALSE)
                DEBUG_CLR_IO_A();
#endif /* (_DEBUG_TRIAXIS != FALSE) */
            }

            /* Rolling Counter */
            if ( (uint8_t)(l_u8RollingCounter + 1U) != u8RollingCounter)
            {
                /* RollingCounter out of sequence */
#if (_DEBUG_TRIAXIS != FALSE)
                DEBUG_SET_IO_A();
#endif /* (_DEBUG_TRIAXIS != FALSE) */
            }
            else
            {
                /* RollingCounter in of sequence */
#if (_DEBUG_TRIAXIS != FALSE)
                DEBUG_CLR_IO_A();
#endif /* (_DEBUG_TRIAXIS != FALSE) */
            }
            l_u8RollingCounter = u8RollingCounter;

#if (C_TRIAXIS_SENT_SLOW_CHANNEL != FALSE)
            /* Slow Channel decoding */
            {
                uint8_t u8Bit2 = ((l_au8SentMessage[0] & 0x04U) != 0x00U) ? 0x01U : 0x00U;
                uint8_t u8Bit3 = ((l_au8SentMessage[0] & 0x08U) != 0x00U) ? 0x01U : 0x00U;
                Sent_DecodeEnhancedSlowChannel(u8Bit2, u8Bit3);
            }
#endif /* (C_TRIAXIS_SENT_SLOW_CHANNEL != FALSE) */
        }
#endif /* C_TRIAXIS_SENT_FRAME_FORMAT */
        l_e8SentMessageReceived = (uint8_t)C_SENT_NONE;

#if (_DEBUG_TRIAXIS != FALSE)
        DEBUG_CLR_IO_B();
#endif /* (_DEBUG_TRIAXIS != FALSE) */
    }
#if (_SUPPORT_SENT_HW_RESOURCE == C_SENT_PPM)
    else if (l_e8SentMessageReceived == (uint8_t)C_SENT_ERROR_CRC)
    {
        u16Result = ERR_TRIAXIS_CRC;
#if (_DEBUG_TRIAXIS != FALSE)
        DEBUG_SET_IO_A();
#endif /* (_DEBUG_TRIAXIS != FALSE) */
        l_e8SentMessageReceived = (uint8_t)C_SENT_NONE;
    }
    else if (l_e8SentMessageReceived == (uint8_t)C_SENT_ERROR_OTHER)
    {
        u16Result = ERR_TRIAXIS_UNKNOWN;
#if (_DEBUG_TRIAXIS != FALSE)
        DEBUG_SET_IO_A();
#endif /* (_DEBUG_TRIAXIS != FALSE) */
        l_e8SentMessageReceived = (uint8_t)C_SENT_NONE;
    }
#endif /* (_SUPPORT_SENT_HW_RESOURCE == C_SENT_PPM) */
    else
    {
        u16Result = ERR_TRIAXIS_NODATA;
#if (_DEBUG_TRIAXIS != FALSE)
        /* DEBUG_SET_IO_A(); */
#endif /* (_DEBUG_TRIAXIS != FALSE) */
    }

    return (u16Result);
} /* End of Triaxis_Data() */

/*!************************************************************************** *
 * GetTriaxisAngle()
 * \brief   Get Triaxis Angle
 * \author  mmp
 * ************************************************************************** *
 * \param   -
 * \return  (uint16_t) Angle 0...FFFF (0...360 degrees)
 * ************************************************************************** *
 * \details
 * ************************************************************************** *
 * - Call Hierarchy: ?
 * - Cyclomatic Complexity: 6+1
 * - Nesting: 4
 * - Function calling: 0
 * ************************************************************************** */
uint16_t GetTriaxisAngle(void)
{
    return (l_u16TriaxisAngleRaw);
} /* End of GetTriaxisAngle() */

/*!************************************************************************** *
 * GetShaftAngle()
 * \brief   Get Shaft Angle
 * \author  mmp
 * ************************************************************************** *
 * \param   -
 * \return  (uint16_t) Angle 0...FFFF (0...360 degrees)
 * ************************************************************************** *
 * \details Corrected Triaxis-angle with Sense-magnet offset-angle
 * ************************************************************************** *
 * - Call Hierarchy: ?
 * - Cyclomatic Complexity: 6+1
 * - Nesting: 4
 * - Function calling: 0
 * ************************************************************************** */
uint16_t GetShaftAngle(void)
{
    return (g_u16ShaftAngle);
} /* End of GetShaftAngle() */

#if (LIN_COMM != FALSE)
/*!*************************************************************************** *
 * HandleTriaxisStatus
 * \brief   Reply LIN response
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: -
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 2
 * - Function calling: 2 (ml_DataReady(), SetLastError())
 * *************************************************************************** */
void HandleTriaxisStatus(void)
{
#if defined (__MLX81330A01__) || defined (__MLX81332A01__)
    uint16_t *pu16LinResponse = (uint16_t *)(void *)LinFrameDataBuffer;
#else
    uint16_t *pu16LinResponse = (uint16_t *)((void *)ML_DATA_LIN_FRAME_DATA_BUFFER);
#endif
    pu16LinResponse[0] = l_u16TriaxisAngleRaw;
    pu16LinResponse[1] = l_u16TriaxisAngleOffset;
#if (C_TRIAXIS_SENT_SLOW_CHANNEL != FALSE)
    pu16LinResponse[2] = (uint16_t) au8EnhancedSlowChannelID[l_u16SlowChannelIdx];
    pu16LinResponse[3] = Triaxis_GetSlowChannelInfo(l_u16SlowChannelIdx);
#else  /* (C_TRIAXIS_SENT_SLOW_CHANNEL != FALSE) */
    pu16LinResponse[2] = 0x0000U;
    pu16LinResponse[3] = 0x0000U;
#endif /* (C_TRIAXIS_SENT_SLOW_CHANNEL != FALSE) */

    if (COLIN_LINstatus.buffer_used == 0U)                                      /* MMP240408-1 */
    {
        if (ml_DataReady(ML_END_OF_TX_ENABLED) != ML_SUCCESS)                   /* Request end-of-transmission Event */
        {
            g_u8ErrorCommunication = TRUE;
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_LIN_API);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
        }
    }
#if (_SUPPORT_LOG_ERRORS != FALSE)
    else
    {
        g_u8ErrorCommunication = TRUE;
        SetLastError(C_ERR_LIN_BUF_NOT_FREE);
    }
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
#if (C_TRIAXIS_SENT_SLOW_CHANNEL != FALSE)
    if (++l_u16SlowChannelIdx >= C_SZ_ENHANCED_SLOW_CHANNELS)
    {
        l_u16SlowChannelIdx = 0U;
    }
#endif /* (C_TRIAXIS_SENT_SLOW_CHANNEL != FALSE) */
} /* End of HandleTriaxisStatus() */
#endif /* (LIN_COMM != FALSE) */

#endif /* (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) */

/* EOF */
