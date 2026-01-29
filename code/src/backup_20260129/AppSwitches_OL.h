/*!*************************************************************************** *
 * \file        AppSwitches.h
 * \brief       MLX8133x Application Switches file (Used by C and S files)
 *
 * \note        project MLX8133x
 *
 * \author      Marcel Braat
 *
 * \date        2017-08-14
 *
 * \version     2.0
 *
 *
 *
 * \copyright   MELEXIS Microelectronic Integrated Systems
 *
 * Copyright (C) 2017-2024 Melexis N.V.
 * The Software is being delivered 'AS IS' and Melexis, whether explicitly or
 * implicitly, makes no warranty as to its Use or performance.
 * The user accepts the Melexis Firmware License Agreement.
 *
 * Melexis confidential & proprietary
 *
 * Default I/O usage:
 *        CAN-I/F    SENT/SPC  DEBUG-I/F  HALL-LATCH  HVIO-DUT  LVIO-SEL    I2C-SLAVE  I2C_MASTER
 * IO[0]: SPI-nSS    -         -          -           HVIO-SEL  -           I2C-SDA    -
 * IO[1]: SPI-CLK    -         TI         HL #1       -         LVIO_SEL_A  I2C-SCK    -
 * IO[2]: SPI-MOSI   -         TO         HL #2       -         LVIO_SEL_B  -          I2C-SDA
 * IO[3]: SPI-MISO   SENT/SPC  -          HL #3       -         LVIO_SEL_C  -          I2C_SCK
 *
 * *************************************************************************** *
 * Note: Only use "#define" statements, no "typedefs"
 * *************************************************************************** */

#ifndef _APP_SWITCHES_H_

#define _APP_SWITCHES_H_

/*!*************************************************************************** */
/*                           INCLUDES                                          */
/* *************************************************************************** */
#include "ActDefines.h"                                                         /* Actuator Defines */
#include "AppDefines.h"                                                         /* HAL Defines */

#define _APP_BURN_IN                        FALSE /*TRUE*/                      /*!< FALSE: Normal application; TRUE: Burn-in application */
#define _APP_SUPPLY_RANGE                   C_APP_SUPPLY_RANGE_12V              /*!< Application supply range: 12V, 24V or 48V */

#define MOTOR_TYPE                          MT_ITW_EVENT                        /*!< Motor Type selection */
#define _SUPPORT_FULLSTEP                   FALSE /*TRUE*/                      /*!< FALSE: Micro-stepping; TRUE: Full-step only */
#define _SUPPORT_HALFSTEP                   FALSE /*TRUE*/                      /*!< FALSE: Micro-stepping; TRUE: Half-step only */
#if (_SUPPORT_FULLSTEP != FALSE)
#define C_MICROSTEP_PER_FULLSTEP            1U                                  /*!< 4/6 full-steps per electric rotation; 1 full-step = 360/4 or 6 = 90 or 60 degrees */
#elif (_SUPPORT_HALFSTEP != FALSE)
#define C_MICROSTEP_PER_FULLSTEP            2U                                  /*!< 8/12 half-steps per electric rotation; 1 half-step = 360/8 or 12 = 45 or 30 degrees */
#else  /* (_SUPPORT_FULLSTEP != FALSE) */
#define C_MICROSTEP_PER_FULLSTEP            /*4U*/ 8U /*16U 32U*/               /*!< 24/48/96/192 micro-steps per electric rotation; 1 micro-step = 360/n = 7.5/3.75/1.875 degrees */
#endif /* (_SUPPORT_FULLSTEP != FALSE) */
#include "ActParams.h"                                                          /* Motor Parameters */

/*!*************************************************************************** */
/*                                 SWITCHES                                    */
/* *************************************************************************** */
#define _SUPPORT_CPUSPEED4                  /*FALSE*/ TRUE                      /*!< FALSE: CPU Speed #4 not support; CPU Speed #4 is supported: 32MHz, 2WS */
#define _SUPPORT_CPUSPEED5                  FALSE /*TRUE*/                      /*!< FALSE: CPU Speed #5 not support; CPU Speed #5 is supported: 40MHz: 3WS */
#define _SUPPORT_CPUSPEED_29M5              FALSE /*TRUE*/                      /*!< FALSE: Use Makefile.configure.mk CPU-speed; TRUE: Interpolate CPU-Frequency between 28MHz and 32MHz */
#define _SUPPORT_CPUSPEED_40M               FALSE /*TRUE*/                      /*!< FALSE: Use selected RCO32-frequency; TRUE: Force 40MHz RCO32 Frequency */
#if (_SUPPORT_CPUSPEED_29M5 != FALSE)
#undef FPLL
#define FPLL                                29500U                              /*!< PLL frequency: 29.5 MHz */
#elif (_SUPPORT_CPUSPEED_40M != FALSE)
#undef FPLL
#define FPLL                                40000U                              /*!< PLL frequency: 40 MHz; NOTE: DELAYs ARE STILL BASED ON FPLL */
#endif
#define PLL_FREQ                            (1000UL * FPLL)                     /*!< PLL frequency: 24/28/32 MHz or others */
#define _SUPPORT_COPRO                      FALSE                               /*!< FALSE: No coPRO present */
#define _SUPPORT_RAM_FUNC                   FALSE /*TRUE*/                      /*!< FALSE: Code execution from Flash; TRUE: Code execution from RAM */
#define _SUPPORT_OPTIMIZE_FOR_SPEED         FALSE /*TRUE*/                      /*!< FALSE: Optimised for size; TRUE: Optimised for speed */
#define _SUPPORT_ROM_TABLE                  FALSE /*TRUE*/                      /*!< FALSE: ROM Table not support. Use Flash-table */

#define _SUPPORT_BUGFIX_DCC_1293            /*FALSE*/ TRUE                      /*!< FALSE: No Fix required; TRUE: Fix required (movsb/movsw) */
#define _SUPPORT_BUGFIX_AMALTHEA_1428       FALSE /*TRUE*/                      /*!< FALSE: No Fix required (ADC table in RAM); TRUE: Fix required (ADC table in Flash/ROM) (MMP220713-1) */

/* MLX16FX CPU Benchmark, based on CoreMark */
#define _APP_BENCHMARK_COREMARK             FALSE /*TRUE*/                      /*!< FALSE: No CoreMark benchmark; TRUE: CoreMark Benchmark */
/* DMA test:
 * ADC @ 8 MHz (MLX8133x) or @ 16MHz (MLX81160/MLX8134x)
 * SPI @ 400kBaud or 2MBaud
 * UART @ 500kBaud or 2MBaud (@ 32MHz) */
#define _APP_DMA_STRESS_TEST                FALSE /*TRUE*/                      /*!< FALSE: No DMA Stress test; TRUE: DMA Stress test: ADC, SPI & UART; */
/* Zwickau test:
 * LIN-Message-ID 0x0C & 0x0D
 *          MLX81160    MLX8133x    MLX8134x
 * IO[0]    HV_OUT      SPI_SS
 * IO[1]    NC          SPI_CLK
 * IO[2]    HV_IN       SPI_MISO
 * IO[3]    LV PWM-OUT  SPI_MOSI
 * IO[4]    SPI_SS      NC (32)
 * IO[5]    SPI_CLK     NC (32)
 * IO[6]    SPI_MISO    NC (32)
 * IO[7]    SPI_MOSI    NC (32)
 * IO[8]    CSA #2      NA
 * IO[9]    CSA #2      NA
 * IO[10]   CSA #1      NA
 * IO[11]   CSA #1      NA
 */
#define _APP_ZWICKAU                        FALSE /*TRUE*/                      /*!< FALSE: Normal application; TRUE: Zwickau application (Enable SPI, Disable CPU HALT) */

/* *** Section #1: Communication *** */
#define ANA_COMM                            FALSE /*TRUE*/                      /*!< Analogue-interface: FALSE: Disabled; TRUE: Enabled */
#define CAN_COMM                            FALSE /*TRUE*/                      /*!< CAN-interface: FALSE: Disabled; TRUE: Enabled */
#define GPIO_COMM                           FALSE /*TRUE*/                      /*!< GPIO-interface: FALSE: Disabled; TRUE: Enabled */
#define I2C_COMM                            FALSE /*TRUE*/                      /*!< I2C-interface: FALSE: Disabled; TRUE: Enabled */
#define LIN_COMM                            /*FALSE*/ TRUE                      /*!< LIN-interface: FALSE: Disabled; TRUE: Enabled */
#define PWM_COMM                            FALSE /*TRUE*/                      /*!< PWM-interface: FALSE: Disabled; TRUE: Enabled */
#define SPI_COMM                            FALSE /*TRUE*/                      /*!< SPI-interface: FALSE: Disabled; TRUE: Enabled */
#if !defined (__MLX81330__)
#define UART_COMM                           FALSE /*TRUE*/                      /*!< UART-interface: FALSE: Disabled; TRUE: Enabled */
#else  /* !defined (__MLX81330__) */
#define UART_COMM                           FALSE                               /*!< UART-interface: FALSE: Disabled; TRUE: Enabled */
#endif /* !defined (__MLX81330__) */

/* I2C Protocol (MMP211104-2) */
#if (I2C_COMM == FALSE)
/* I2C_MASTER_PROT_NONE, I2C_MASTER_PROT_GENERIC, I2C_MASTER_PROT_MEMORY or I2C_MASTER_PROT_TRIAXIS_90381 */
#if defined (__MLX81332_w90381__) || defined (__MLX81340_w90381__)
#define I2C_MASTER_PROT                     I2C_MASTER_PROT_TRIAXIS_90381       /*!< I2C Master protocol choice */
#else  /* __MLX81332_w90381__ */
#define I2C_MASTER_PROT                     I2C_MASTER_PROT_NONE                /*!< I2C Master protocol choice */
#endif /* __MLX81332_w90381__ */
/* I2C_SLAVE_PROT_NONE, I2C_SLAVE_PROT_GENERIC */
#define I2C_SLAVE_PROT                      I2C_SLAVE_PROT_NONE                 /*!< I2C Slave protocol choice */
#else  /* (I2C_COMM == FALSE) */
/* I2C_MASTER_PROT_NONE, I2C_MASTER_PROT_GENERIC, I2C_MASTER_PROT_MEMORY or I2C_MASTER_PROT_TRIAXIS_90381 */
#define I2C_MASTER_PROT                     I2C_MASTER_PROT_NONE                /*!< I2C Master protocol choice */
/* I2C_SLAVE_PROT_NONE, I2C_SLAVE_PROT_GENERIC, I2C_SLAVE_PROT_POSITIONING, I2C_SLAVE_PROT_CONTINUOUS, I2C_SLAVE_PROT_MLX_ACT */
#define I2C_SLAVE_PROT                      I2C_SLAVE_PROT_MLX_ACT              /*!< I2C Slave protocol choice */
#endif /* (I2C_COMM == FALSE) */

/* LIN Protocol */
#if (LIN_COMM != FALSE)
#define LINPROT    /*LIN2X_AGS LIN2X_AIRVENT12 LIN2X_FAN01*/ LIN2X_HVAC49 /*LIN2X_HVAC52 LIN2X_RELAY LIN22_SIMPLE_PCT LIN2X_SOLENOID LIN2X_VALVE*/  /*!< Select LIN application protocol */
#define _SUPPORT_AUTO_BAUDRATE              /*FALSE*/ TRUE                      /*!< FALSE: Fixed baudrate; TRUE: Auto-detection of baudrate */
#define _SUPPORT_BUSTIMEOUT                 /*FALSE*/ TRUE                      /*!< FALSE: Do not move to emergency position after bus-timeout; TRUE: Move to emergency position after bus-timeout */
#define _SUPPORT_BUSTIMEOUT_SLEEP           FALSE /*TRUE*/                      /*!< FALSE: After emergency run no SLEEP; TRUE: After emergency run enter SLEEP mode (MMP170722-2) */
#define _SUPPORT_LIN_BUS_ACTIVITY_CHECK     /*FALSE*/ TRUE                      /*!< FALSE: No MLX4/LIN-Bus activity check; TRUE: LIN-Bus activity check */
#define _SUPPORT_LIN_SLEEP                  /*FALSE*/ TRUE                      /*!< FALSE: No LIN-sleep support; TRUE LIN-sleep support */
#define _SUPPORT_LIN_UV                     FALSE /*TRUE*/                      /*!< FALSE: No LIN UV check; TRUE: LIN UV check (reset Bus-time-out) */
#define _SUPPORT_LINNETWORK_LOADER          FALSE /*TRUE*/                      /*!< FALSE: No support for LIN-loader; TRUE: Support of LIN-Loader */

/* LIN Enhanced */
#if (LIN_COMM != FALSE)
#if (LINPROT == LIN2X_HVAC52)
#define _SUPPORT_HVAC_GROUP_ADDRESS         /*FALSE*/ TRUE                      /*!< Support of Group-address */
#define _SUPPORT_LINAA_ESHUNT               /*FALSE*/ TRUE                      /*!< Support External LIN-AA shunt */
#define _SUPPORT_LINAA_VS_COMPENSATION      /*FALSE*/ TRUE                      /*!< Support VS-supply compensation (MMP240214-1) */
#else  /* (LINPROT == LIN2X_HVAC52) */
#define _SUPPORT_HVAC_GROUP_ADDRESS         FALSE /*TRUE*/                      /*!< Support of Group-address */
#define _SUPPORT_LINAA_ESHUNT               FALSE /*TRUE*/                      /*!< Support External LIN-AA shunt */
#define _SUPPORT_LINAA_VS_COMPENSATION      FALSE /*TRUE*/                      /*!< Support VS-supply compensation (MMP240214-1) */
#endif /* (LINPROT == LIN2X_HVAC52) */
#endif /* (LIN_COMM != FALSE) */

/* LIN-AA stuff (MLX81160, MLX81344/46 doesn't support LIN-AA) */
#if defined (__MLX81160__) || defined (__MLX81344__) || defined (__MLX81346__)
#define _SUPPORT_LIN_AA                     FALSE /*TRUE*/                      /*!< FALSE: No LIN-AA Based on BSM; TRUE: LIN-AA based on BSM */
#elif ((LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52) || (LINPROT == LIN2X_AIRVENT12)) && \
    (PROJECT_ID == 0x0503) || (PROJECT_ID == 0x050B) ||                            /* MLX81330A  SO8|QFN */ \
    (PROJECT_ID == 0x050F) || (PROJECT_ID == 0x0510) || (PROJECT_ID == 0x0511) ||  /* MLX81330B1 SO8-001|QFN|SO8-101 */ \
    (PROJECT_ID == 0x0514) || (PROJECT_ID == 0x0515) || (PROJECT_ID == 0x0516) ||  /* MLX81330B2 SO8-002|QFN|SO8-102 */ \
    (PROJECT_ID == 0x0701) || (PROJECT_ID == 0x0705) ||                            /* MLX81332A QFN|SO8-001 */ \
    (PROJECT_ID == 0x070A) || (PROJECT_ID == 0x070B) || (PROJECT_ID == 0x070C) ||  /* MLX81332B1 QFN|SO8-001|SO8-101 */ \
    (PROJECT_ID == 0x070F) || (PROJECT_ID == 0x0710) || (PROJECT_ID == 0x0711) ||  /* MLX81332B2 QFN|SO8-002|SO8-102 */ \
    (PROJECT_ID == 0x0901) ||                                                      /* MLX81334AA QFN */ \
    (PROJECT_ID == 0x0A01) || (PROJECT_ID == 0x0A03) || (PROJECT_ID == 0x0A05) ||  /* MLX81340A QFN32|MLX81340B QFN32 */ \
    (PROJECT_ID == 0x2601) || (PROJECT_ID == 0x2602) || (PROJECT_ID == 0x2604)     /* MXL81350AA SO8-001|SO8-100|QFN */
#define _SUPPORT_LIN_AA                     /*FALSE*/ TRUE                      /*!< FALSE: No LIN-AA Based on BSM; TRUE: LIN-AA based on BSM (IC support LIN-AA) */
#else
/* LIN protocol doesn't support LIN-AA */
#define _SUPPORT_LIN_AA                     FALSE /*TRUE*/                      /*!< FALSE: No LIN-AA Based on BSM supported; TRUE: Not supported! */
#endif
#if (_SUPPORT_LIN_AA != FALSE)
#define LIN_AA_BSM_SNPD_R1p0                /*FALSE*/ TRUE                      /*!< FALSE: Cooling-mode; TRUE: LIN Bus Shunt Method Slave Node Position Detection */
#define LIN_AA_CURRSRC_ONLY                 /*FALSE*/ TRUE                      /*!< FALSE: Use LIN Pull-up resistance as pre-select; TRUE: Use Current-Source iAA045 as pre-select */
#define LIN_AA_NV                           /*FALSE*/ TRUE                      /*!< FALSE: Use fixed values; TRUE: Use Non Volatile Memory values */
#define LIN_AA_EXT_VOLTAGE                  FALSE /*TRUE*/                      /*!< FALSE: Normal range; TRUE: Extended LIN voltage range (only MLX81340AB) */
#define LIN_AA_INFO                         FALSE /*TRUE*/                      /*!< Collect Ish1, Ish2 and Ish3 data for 10 measurements */
#define LIN_AA_LINFRAME_TIMEOUT             /*FALSE*/ TRUE                      /*!< FALSE: No LIN-AA LIN-Frame timeout detection; TRUE: LIN-AA LIN-Frame timeout detection */
#define LIN_AA_RESOURCE                     LIN_AA_RESOURCE_CTIMER1             /*!< LIN-AA ADC HW-Trigger Resource selection */
#define LIN_AA_SCREENTEST                   FALSE /*TRUE*/                      /*!< Additional screening data CM1..3 & DM1..3 */
#define LIN_AA_TEST_DUT                     FALSE /*TRUE*/                      /*!< FALSE: Normal DUT; TRUE: Test DUT (Skip storage NAD) */
#define LIN_AA_TIMEOUT                      /*FALSE*/ TRUE                      /*!< FALSE: No LIN-AA Timeout; TRUE: LIN-AA Timeout of 40 seconds */
#if (PROJECT_ID == 0x0503) || (PROJECT_ID == 0x050B) ||                         /* MLX81330A  SO8|QFN */ \
    (PROJECT_ID == 0x0514) || (PROJECT_ID == 0x0515) || (PROJECT_ID == 0x0516)  /* MLX81330B2 SO8-002|QFN|SO8-102 */
#define LIN_AA_VDDA_5V                      /*FALSE*/ TRUE                      /*!< FALSE: LIN-AA amplifier at 3.3V; TRUE: LIN-AA amplifier at 5V */
#else /* MLX81330B2 SO8|QFN */
#define LIN_AA_VDDA_5V                      FALSE /*TRUE*/                      /*!< FALSE: LIN-AA amplifier at 3.3V; TRUE: LIN-AA amplifier at 5V */
#endif
#endif /* (_SUPPORT_LIN_AA != FALSE) */

/* LIN Diagnostics services (0xB0-0xB7) */
#define _SUPPORT_NV_WRITE_IMMEDIATE         FALSE /*TRUE*/                      /*!< FALSE: Some (see code) LIN Diagnostics Data Dump Service doesn't write to NVRAM; TRUE: All LIN Diagnostics Data Dump Service immediately write to NVRAM (MMP170119-2) */
#if (LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52) || (LINPROT == LIN2X_AIRVENT12)
#define _SUPPORT_DIAG_B0                    /*FALSE*/ TRUE                      /*!< FALSE: Re-assign NAD 0xB0 not supported; TRUE: LIN Diagnostics 0xB0 supported (LH5.2: 9.8.3) */
#define _SUPPORT_DIAG_B0_NV_STORE           /*FALSE*/ TRUE                      /*!< FALSE: Don't store NAD at 'B0' (use 'B6'); TRUE: Store NAD in NVRAM at 'B0' (LH5.2: 9.8.3) */
#define _SUPPORT_DIAG_B1                    /*FALSE*/ TRUE                      /*!< FALSE: Assign Message ID to Frame ID 0xB1 not supported; TRUE: LIN Diagnostics 0xB1 supported (LH5.2: 9.8.1) */
#define _SUPPORT_DIAG_B1_NV_STORE           /*FALSE*/ TRUE                      /*!< FALSE: Don't store PID at 'B1' (use 'B6'); TRUE: Store PID in NVRAM at 'B1' (LH5.2: 9.8.1) */
#define _SUPPORT_DIAG_B2_SERIALNR           FALSE /*TRUE*/                      /*!< FALSE: Don't support Serial-Number; TRUE: Support Serial-Number (0x01) */
#define _SUPPORT_DIAG_B2_MSG_ID             /*FALSE*/ TRUE                      /*!< FALSE: Don't support 0x10-0x1F; TRUE: Support 0x10-0x1F */
#define _SUPPORT_READ_BY_ID_CUSTOMERID      FALSE /*TRUE*/                      /*!< LIN Diagnostics 0xB2-0x3D */
#define _SUPPORT_READ_BY_ID_PROD_DATE       FALSE /*TRUE*/                      /*!< LIN Diagnostics 0xB2-0x3E */
#define _SUPPORT_DIAG_B3                    FALSE /*TRUE*/                      /*!< FALSE: Conditional change NAD 0xB3 not supported; TRUE: LIN Diagnostics 0xB3 supported */
#define _SUPPORT_DIAG_B3_NV_STORE           FALSE /*TRUE*/                      /*!< FALSE: Don't store NAD at 'B3' (use 'B6'); TRUE: Store NAD in NVRAM at 'B3' */
#define _SUPPORT_DIAG_B4                    /*FALSE*/ TRUE                      /*!< FALSE: Assign Variant-ID is not supported; TRUE: Assign Variant-ID is supported */
#if (LINPROT == LIN2X_HVAC52)
#define _SUPPORT_DIAG_B6                    /*FALSE*/ TRUE                      /*!< FALSE: Assign Group address not supported; TRUE: Assign Group address supported */
#else  /* (LINPROT == LIN2X_HVAC52) */
#define _SUPPORT_DIAG_B6                    FALSE /*TRUE*/                      /*!< FALSE: Assign Group address not supported; TRUE: Assign Group address supported */
#endif /* (LINPROT == LIN2X_HVAC52) */
#define _SUPPORT_DIAG_B7                    /*FALSE*/ TRUE                      /*!< FALSE: LIN-Diagnostics 0xB7 not supported; TRUE: LIN Diagnostics 0xB7 supported */
#define _SUPPORT_DIAG_B7_NV_STORE           /*FALSE*/ TRUE                      /*!< FALSE: Don't store PID's at 'B7 (use 'B6')'; TRUE: Store PID's in NVRAM at 'B7' (LH5.2: 9.8.2) */
#define _SUPPORT_ISO17987                   FALSE /*TRUE*/                      /*!< FALSE: No ISO17987 support; TRUE: ISO17987 support */
#else  /* (LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52) || (LINPROT == LIN2X_AIRVENT12) */
#define _SUPPORT_DIAG_B0                    /*FALSE*/ TRUE                      /*!< FALSE: Re-assign NAD 0xB0 not supported; TRUE: LIN Diagnostics 0xB0 supported */
#define _SUPPORT_DIAG_B0_NV_STORE           /*FALSE*/ TRUE                      /*!< FALSE: Don't store NAD at 'B0' (use 'B6'); TRUE: Store NAD in NVRAM at 'B0' */
#define _SUPPORT_DIAG_B1                    FALSE /*TRUE*/                      /*!< FALSE: Assign Message ID to Frame ID 0xB1 not supported; TRUE: LIN Diagnostics 0xB1 supported */
#define _SUPPORT_DIAG_B1_NV_STORE           FALSE /*TRUE*/                      /*!< FALSE: Don't store PID at 'B1' (use 'B6'); TRUE: Store PID in NVRAM at 'B1' */
#define _SUPPORT_DIAG_B2_SERIALNR           FALSE /*TRUE*/                      /*!< FALSE: Don't support Serial-Number; TRUE: Support Serial-Number (0x01) */
#define _SUPPORT_DIAG_B2_MSG_ID             /*FALSE*/ TRUE                      /*!< FALSE: Don't support 0x10-0x1F; TRUE: Support 0x10-0x1F */
#define _SUPPORT_READ_BY_ID_CUSTOMERID      FALSE /*TRUE*/                      /*!< LIN Diagnostics 0xB2-0x3D */
#define _SUPPORT_READ_BY_ID_PROD_DATE       FALSE /*TRUE*/                      /*!< LIN Diagnostics 0xB2-0x3E */
#define _SUPPORT_DIAG_B3                    FALSE /*TRUE*/                      /*!< FALSE: Conditional change NAD 0xB3 not supported; TRUE: LIN Diagnostics 0xB3 supported */
#define _SUPPORT_DIAG_B3_NV_STORE           FALSE /*TRUE*/                      /*!< FALSE: Don't store NAD at 'B3' (use 'B6'); TRUE: Store NAD in NVRAM at 'B3' */
#define _SUPPORT_DIAG_B4                    FALSE /*TRUE*/                      /*!< FALSE: Data-dump 0xB4 is not supported; TRUE: Data-dump 0xB4 is supported */
#define _SUPPORT_DIAG_B6                    /*FALSE*/ TRUE                      /*!< FALSE: LIN-Diagnostics 0xB6 not supported; TRUE: LIN Diagnostics 0xB6 supported */
#define _SUPPORT_DIAG_B7                    /*FALSE*/ TRUE                      /*!< FALSE: LIN-Diagnostics 0xB7 not supported; TRUE: LIN Diagnostics 0xB7 supported */
#define _SUPPORT_DIAG_B7_NV_STORE           FALSE /*TRUE*/                      /*!< FALSE: Don't store PID's at 'B7 (use 'B6')'; TRUE: Store PID's in NVRAM at 'B7' */
#define _SUPPORT_ISO17987                   /*FALSE*/ TRUE                      /*!< FALSE: No ISO17987 support; TRUE: ISO17987 support */
#endif /* (LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52) || (LINPROT == LIN2X_AIRVENT12) */
#define _SUPPORT_UDS                        FALSE /*TRUE*/                      /*!< FALSE: UDS is not supported; TRUE: UDS is supported */
#define _SUPPORT_UDS_DK                     _SUPPORT_UDS_DK1                    /*!< LIN UDS Method selection */
#define _SUPPORT_UDS_READ_BY_ID_SUBSYS      /*FALSE*/ TRUE                      /*!< FALSE: UDS Read by ID not supported; TRUE: UDS Read by ID supported */
#define _SUPPORT_UDS_SESSION_CTRL           FALSE /*TRUE*/                      /*!< FALSE: UDS Session Control is not support; TRUE: UDS Session Control is supported */
#define _SUPPORT_UDS_RESET                  FALSE /*TRUE*/                      /*!< FALSE: UDS Reset is not support; TRUE: UDS Reset is supported */
#define _SUPPORT_UDS_WRITE_BY_ID            /*FALSE*/ TRUE                      /*!< FALSE: UDS Write-by-ID is not support; TRUE: UDS Write-by-ID is supported */
#define _SUPPORT_UDS_TESTER_PRESENT         FALSE /*TRUE*/                      /*!< FALSE: UDS Tester is not support; TRUE: UDS Tester is supported */
#define _SUPPORT_MLX_DEBUG_MODE             /*FALSE*/ TRUE                      /*!< Melexis Debug-mode (Flash testing) */
#define _SUPPORT_MLX_DEBUG_OPTIONS          FALSE /*TRUE*/                      /*!< FALSE: No Melexis LIN Diagnostics frames; TRUE: Support Melexis LIN Diagnostics frames */
#define _SUPPORT_NAD_RANGE_CHECK            FALSE /*TRUE*/                      /*!< FALSE: no NAD range check, all NAD's allowed; NAD range check (MMP230306-1) */
#else  /* (LIN_COMM != FALSE) */
#define LINPROT                             NOLIN                               /*!< No LIN support */
#endif /* (LIN_COMM != FALSE) */

#if (UART_COMM != FALSE)
/* UART Communication Protocol */
#define _SUPPORT_UART_IF_APP                C_UART_IF_NONE                      /*!< (First) UART-Interface selection */
#define _SUPPORT_UART2_IF_APP               C_UART_IF_NONE                      /*!< Second UART-Interface selection */
#define _SUPPORT_UART_SCOPE_MODE    /*TINY_SCOPE DUAL_SCOPE*/ QUAD_SCOPE /*CH12_SCOPE*/  /*!< Scope mode: TINY (Single 16-bit channel), DUAL (Dual 16-bits channels), QUAD (Quad 16-bits channels), CH12 (12x 8-bit channels) */
#endif /* (UART_COMM != FALSE) */

/* Application */
#define _SUPPORT_ACOUSTIC_NOISE_REDUCTION   FALSE /*TRUE*/                      /*!< FALSE: No special features to reduce acoustic Noise; TRUE: Extra features to reduce acoustic Noise */
#define _SUPPORT_ACT_SPEED_BY_LIN           FALSE /*TRUE*/                      /*!< FALSE: Actuator speed by MotroDriver; TRUE: Actuator speed by LIN Command */
#define _SUPPORT_ADC_IO_CHECK               FALSE /*TRUE*/                      /*!< FALSE: Don't check for stuck I/O's; TRUE: Check for stuck I/O's */
#define _SUPPORT_ADC_PWM_DELAY              /*FALSE*/ TRUE                      /*!< FALSE: Up to 6 HW-Trigger ADC-channel-support; TRUE: Up to 5 HW-Trigger ADC-channel-support PWM-synchronised */
#define _SUPPORT_ADC_REF_HV_CALIB           FALSE /*TRUE*/                      /*!< FALSE: Use ADC Reference 2.5V for HV-channels; TRUE: Use ADC Reference 1.5V for HV-channels */
#define _SUPPORT_ADC_REF_MC_CALIB           FALSE /*TRUE*/                      /*!< FALSE: Use ADC Reference 2.5V for MCUR-channel (zero-current @ ~500 LSB); TRUE: Use ADC Reference 1.5V for MCUR-channel (zero-current @ ~800 LSB) (MMP220523-1) */
#define _SUPPORT_ADC_SOS_TRIGGER    ADC_SOS_TRIGGER_HW /*ADC_SOS_TRIGGER_SW*/   /*!< ADC SOS Trigger for Motor Run */
#if (defined (__MLX81340B01__) || defined (__MLX81344B01__) || defined (__MLX81346B01__))
#define _SUPPORT_ADC_VSMF_OFF               /*FALSE*/ TRUE                      /*!< FALSE: No VSMF-Offset measurement (MLX8133x); TRUE: VSMF-Offset measurement (MLX8134x) (MMP220607-1) */
#else  /* (defined (__MLX81340B01__) || defined (__MLX81344B01__) || defined (__MLX81346B01__)) */
#define _SUPPORT_ADC_VSMF_OFF               FALSE /*TRUE*/                      /*!< FALSE: No VSMF-Offset measurement (MLX8133x); TRUE: VSMF-Offset measurement (MLX8134x) */
#endif /* (defined (__MLX81340B01__) || defined (__MLX81344B01__) || defined (__MLX81346B01__)) */
#define _SUPPORT_ADC_TEMPCOMP               /*FALSE*/ TRUE                      /*!< FALSE: ADC without temperature compensation; TRUE: ADC with temperature compensation (MMP200626-1); This requires EEPROM V3 layout or higher */
#define _SUPPORT_AMBIENT_TEMP               /*FALSE*/ TRUE                      /*!< FALSE: Use chip temperature for compensation; TRUE: Use estimated ambient temperature for compensation */
#if (_APP_SUPPLY_RANGE != C_APP_SUPPLY_RANGE_48V)
#define _SUPPORT_APP_48V_ADC                FALSE /*TRUE*/                      /*!< FALSE: 12/24V ADC-range (DIV 26); TRUE: 48V ADC-range (DIV 52) (MMP211102-1) */
#else  /* (_APP_SUPPLY_RANGE != C_APP_SUPPLY_RANGE_48V) */
#define _SUPPORT_APP_48V_ADC                /*FALSE*/ TRUE                      /*!< FALSE: 12/24V ADC-range (DIV 26); TRUE: 48V ADC-range (DIV 52) (MMP211102-1) */
#endif /* (_APP_SUPPLY_RANGE != C_APP_SUPPLY_RANGE_48V) */
#define _SUPPORT_APP_SAVE                   /*FALSE*/ TRUE                      /*!< FALSE: Don't save APP info; TRUE: Save APP info */
#define _SUPPORT_APP_USER_MODE              /*FALSE*/ TRUE                      /*!< FALSE: Application in System-mode; TRUE: Application in User mode (MMP200630-1) */
#define _SUPPORT_AUTONOMOUS_DEMO            FALSE /*TRUE*/                      /*!< FALSE: Operate by LIN; TRUE: Autonomous demo mode */
#define _SUPPORT_BG_FLASH_TEST              FALSE /*TRUE*/                      /*!< FALSE: No Flash background test; TRUE: Flash background test */
#define _SUPPORT_BG_RAM_TEST                FALSE /*TRUE*/                      /*!< FALSE: No RAM background test; TRUE: RAM background test */
#define _SUPPORT_BG_ROM_TEST                FALSE /*TRUE*/                      /*!< FALSE: No ROM background test; TRUE: ROM background test */
#define _SUPPORT_BG_MEM_TEST                FALSE /*TRUE*/                      /*!< FALSE: No Memory background test; TRUE: Memory background test */
#if (FLASH_BIST_START == 0x5800) || (FLASH_BIST_START == 0x8000)
#define _SUPPORT_BOOTLOADER                 BOOTLOADER_NONE                     /*!< Choice of Boot-loader supported */
#define BOOTLOADER_SIZE                     0U                                  /*!< Boot-loader size in [kB]; Must be module 2kB */
#else  /* (FLASH_BIST_START == 0x5800) || (FLASH_BIST_START == 0x8000) */
#define _SUPPORT_BOOTLOADER                 BOOTLOADER_LIN                      /*!< Choice of Boot-loader supported */
#if (FLASH_BIST_START > 0x8000)
#define BOOTLOADER_SIZE                     (FLASH_BIST_START - 0x8000) / 2048    /*!< Boot-loader size in [kB]; Must be module 2kB */
#else  /* (FLASH_BIST_START > 0x8000) */
#define BOOTLOADER_SIZE                     (FLASH_BIST_START - 0x5800) / 2048    /*!< Boot-loader size in [kB]; Must be module 2kB */
#endif /* (FLASH_BIST_START > 0x8000) */
#endif /* (FLASH_BIST_START == 0x5800) || (FLASH_BIST_START == 0x8000) */
#if (_SUPPORT_BOOTLOADER == BOOTLOADER_LIN)
#define _SUPPORT_BOOTLOADER_LIN             /*FALSE*/ TRUE                      /*!< FALSE: No LIN-Bootloader support; TRUE: LIN-Bootloader support */
#else  /* (_SUPPORT_BOOTLOADER == BOOTLOADER_LIN) */
#define _SUPPORT_BOOTLOADER_LIN             FALSE /*TRUE*/                      /*!< FALSE: No LIN-Bootloader support; TRUE: LIN-Bootloader support */
#endif /* (_SUPPORT_BOOTLOADER == BOOTLOADER_LIN) */
#define _SUPPORT_BOOTLOADER_PPM             /*FALSE*/ TRUE                      /*!< FALSE: No PPM-Bootloader support; TRUE: PPM-Bootloader support */
#define _SUPPORT_BRAKING                    FALSE /*TRUE*/                      /*!< FALSE: No braking support; TRUE: Support motor braking */
#define _SUPPORT_CP_SSCM                    /*FALSE*/ TRUE                      /*!< FALSE: Disable Charge-pump Spread-Spectrum; TRUE: Enable Charge Pump Spread-Spectrum */
#define _SUPPORT_CPFREQ                     82U                                 /*!< Define Charge-pump frequency between 60 and 82MHz */
#define _SUPPORT_CPFREQ_TEMPCOMP            /*FALSE*/ TRUE                      /*!< FALSE: No Charge-pump clock compensation; TRUE: Charge-pump clock compensation for temperature (MMP200728-1) */
#if (_APP_ZWICKAU != FALSE)
#define _SUPPORT_CPU_HALT                   FALSE /*TRUE*/                      /*!< FALSE: MLX16 doesn't HALT; TRUE: MLX16 enters HALT during holding mode (power-safe) */
#define _SUPPORT_CPU_HALT_ADC_OFF           FALSE /*TRUE*/                      /*!< FALSE: ADC remains ON; TRUE: ADC goes OFF */
#define _SUPPORT_MLX_CHIP_STATUS            /*FALSE*/ TRUE                      /*!< FALSE: No Melexis Chip status; TRUE: Melexis Chip status (EMC/BCI/...) */
#define _SUPPORT_2ND_MLX_CHIP_STATUS        FALSE /*TRUE*/                      /*!< FALSE: 1st MLX Chip Status (3A/3B); TRUE: 2nd MLX Chip Status (38/39) */
#else  /* (_APP_ZWICKAU != FALSE) */
#if (ANA_COMM != FALSE) || (PWM_COMM != FALSE)
#define _SUPPORT_CPU_HALT                   FALSE /*TRUE*/                      /*!< FALSE: MLX16 doesn't HALT; TRUE: MLX16 enters HALT during holding mode (power-safe) */
#else  /* (ANA_COMM != FALSE) || (PWM_COMM != FALSE) */
#define _SUPPORT_CPU_HALT                   /*FALSE*/ TRUE                      /*!< FALSE: MLX16 doesn't HALT; TRUE: MLX16 enters HALT during holding mode (power-safe) */
#endif /* (ANA_COMM != FALSE) || (PWM_COMM != FALSE) */
#define _SUPPORT_CPU_HALT_ADC_OFF           /*FALSE*/ TRUE                      /*!< FALSE: ADC remains ON; TRUE: ADC goes OFF */
#define _SUPPORT_MLX_CHIP_STATUS            FALSE /*TRUE*/                      /*!< FALSE: No Melexis Chip status; TRUE: Melexis Chip status (EMC/BCI/...) */
#define _SUPPORT_2ND_MLX_CHIP_STATUS        FALSE /*TRUE*/                      /*!< FALSE: 1st MLX Chip Status (3A/3B); TRUE: 2nd MLX Chip Status (38/39) */
#endif /* (_APP_ZWICKAU != FALSE) */
#if (LINPROT == LIN22_SIMPLE_PCT) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_POSITIONING)
#define _SUPPORT_CALIBRATION                TRUE                                /*!< FALSE: No calibration at power-up supported; TRUE: Calibration at power-up if configured in NVRAM (Does require _SUPPORT_POS_INIT) */
#define _SUPPORT_HALF_AUTO_CALIB            FALSE                               /*!< FALSE: Only full-calibration; TRUE: Half-calibration support (MMP190807-2) */
#elif (LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52)
#define _SUPPORT_CALIBRATION                FALSE                               /*!< FALSE: No calibration at power-up supported; TRUE: Calibration at power-up if configured in NVRAM (Does require _SUPPORT_POS_INIT) */
#define _SUPPORT_HALF_AUTO_CALIB            FALSE                               /*!< FALSE: Only full-calibration; TRUE: Half-calibration support (MMP190807-2) */
#endif /* (LINPROT) */
#define _SUPPORT_DYN_MCUR_EXP               FALSE /*TRUE*/                      /*!< FALSE: MCur Exponent based on last Command; TRUE: MCur Exponent dynamically calculated */
#define _SUPPORT_SPEED_AUTO                 /*FALSE*/ TRUE                      /*!< FALSE: No auto-speed based on Voltage & Temperature; TRUE: Auto-speed based on Voltage & Temperature */
#define _SUPPORT_CRASH_RECOVERY             FALSE /*TRUE*/                      /*!< FALSE: No crash recovery; TRUE: Fast crash recovery */
#define _SUPPORT_CRITICAL_PERIPHERAL_CHECK  /*FALSE*/ TRUE                      /*!< FALSE: No peripheral check; TRUE: peripheral check supported */
#if (defined (__MLX81340B01__) || defined (__MLX81344B01__) || defined (__MLX81346B01__))
#define _SUPPORT_CSA_HIGHGAIN               FALSE /*TRUE*/                      /*!< FALSE: Standard Gain of 10x; TRUE: High Gain of 20x */
#else  /* (defined (__MLX81340B01__) || defined (__MLX81344B01__) || defined (__MLX81346B01__)) */
#define _SUPPORT_CSA_HIGHGAIN               FALSE                               /*!< FALSE: Support only standard Gain (10x) */
#endif /* (defined (__MLX81340B01__) || defined (__MLX81344B01__) || defined (__MLX81346B01__)) */
#define _SUPPORT_DED_RETRY                  FALSE /*TRUE*/                      /*!< FALSE: DED Retries are OFF; TRUE: DED Retries are ON (4) */
#define _SUPPORT_DEGRADED_MODE              /*FALSE*/ TRUE                      /*!< FALSE: No degraded mode support; TRUE: Degraded mode support */
#define _SUPPORT_DIRECTION_IN               FALSE /*TRUE*/                      /*!< FALSE: No direction-pin support; TRUE: Direction-pin support */
#define DIRECTION_PIN_IO                    PIN_FUNC_IO_4                       /*!< Direction-in pin IO[5:4], Phase T */
#define _SUPPORT_DUAL_APP_BLOCK             /*FALSE*/ TRUE                      /*!< FALSE: Use single block; TRUE: Use multiple (2) blocks */
#define _SUPPORT_DUAL_RELAY                 FALSE /*TRUE*/                      /*!< FALSE: Single Relay; TRUE: Dual: Relay */
#define _DEBUG_FLASH_WRITE_CYCLES           FALSE /*TRUE*/                      /*!< FALSE: APP info; TRUE: Flash Write cycles */
#define _SUPPORT_DWD                        FALSE /*TRUE*/                      /*!< FALSE: Digital Watchdog disabled; TRUE: Digital Watchdog enabled */
#define _SUPPORT_NV_TYPE            /*C_NV_NONE*/ C_NV_EEPROM /*C_NV_FLASH*/    /*!< Type of Non-volatile Storage selection */
#define _SUPPORT_NV_BACKUP                  FALSE /*TRUE*/                      /*!< FALSE: No Non Volatile Memory Backup; TRUE: Non Volatile Memory backup */
#define _SUPPORT_NV_MOTOR_PARAMS            /*FALSE*/ TRUE                      /*!< FALSE: Motor parameters as constant; TRUE: Motor parameters from Non Volatile Memory */
#define _SUPPORT_NV_NON_BLOCK_WRITE         FALSE /*TRUE*/                      /*!< FALSE: Non Volatile Memory Write block application; TRUE: Non Volatile Memory Write w/o block application; Application use system RAM copy of Non Volatile Memory (TODO: To be implemented) */
#define _SUPPORT_NV_EMERGENCY_STORE         FALSE /*TRUE*/                      /*!< FALSE: No Non Volatile Memory Emergency Store; TRUE: Non Volatile Memory Emergency Store below UV-level */
#define _SUPPORT_NV_HEX                     FALSE /*TRUE*/                      /*!< FALSE: No Non Volatile Memory Hex; TRUE: Support Non Volatile Memory Hex-file */
#define _SUPPORT_EMERGENCY_RUN              /*FALSE*/ TRUE                      /*!< FALSE: No Emergency/Safety run supported (Enter SLEEP); TRUE: Emergency/Safety run supported (Position defined in Non Volatile Memory) (MMP220331-1) */
#define _SUPPORT_FAST_ATAN2                 /*FALSE*/ TRUE                      /*!< FALSE: Use math-library atan2; TRUE: Use private atan2 (MMP190117-1) */
#define _SUPPORT_FAST_ATAN2_LOOKUP          /*FALSE*/ TRUE                      /*!< FALSE: 16-bit ATAN2; TRUE: 11-bit ATAN2 */
#define _SUPPORT_FAST_STOP                  FALSE /*TRUE*/                      /*!< FALSE: Natural motor stop; TRUE: Active motor stop (MMP231121-1) */
#define _SUPPORT_FLASH_PRODUCTION_DATA      FALSE /*TRUE*/                      /*!< FALSE: No Flash Production Data area/support; TRUE: Flash Production Data area/support (BIST_PAGE_COUNT) (MMP211006-2) */
#define _SUPPORT_MECHANICAL_ERROR           FALSE /*TRUE*/                      /*!< FALSE: No Mechanical error detection; TRUE: Mechanical error detection */
#define _SUPPORT_POR_MOTORDRIVER            FALSE /*TRUE*/                      /*!< FALSE: Motor Driver at POR is disabled; Motor Driver at POR is enabled */
#if defined (_SUPPORT_APP_TYPE) && (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
#define _SUPPORT_POS_INIT                   FALSE /*TRUE*/                      /*!< FALSE: No "Set initial position" support; TRUE: "Set initial position" supported */
#else  /* defined (_SUPPORT_APP_TYPE) && (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT) */
#define _SUPPORT_POS_INIT                   /*FALSE*/ TRUE                      /*!< FALSE: No "Set initial position" support; TRUE: "Set initial position" supported */
#endif /* defined (_SUPPORT_APP_TYPE) && (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT) */
#define _SUPPORT_RESET_BY_IO                FALSE /*TRUE*/                      /*!< FALSE: No chip reset by IO; TRUE: Reset by IO (MLX81160, IO5) */
#define _SUPPORT_RINNEN                     FALSE /*TRUE*/                      /*!< FALSE: No Rinnen-test improvements; TRUE: Rinnen-test improvements */
#if defined (__MLX81340B01__) || defined (__MLX81344B01__) || defined (__MLX81346B01__)
#define _SUPPORT_ROM_FAST_MATH              /*FALSE*/ TRUE                      /*!< FALSE: Use Flash fast-math ATAN2; TRUE: Use ROM fast-math ATAN2 (Makefile.srcs.mk: include "PLTF_LIBS += fast_math") */
#else
#define _SUPPORT_ROM_FAST_MATH              FALSE /*TRUE*/                      /*!< FALSE: Use Flash fast-math ATAN2; TRUE: Use ROM fast-math ATAN2 */
#endif
#define _SUPPORT_SINCOS_TABLE_SZ            /*192*/ 256 /*1024*/                /*!< Sine/Cosine look-up table of 192 or 256 entries */
#define _SUPPORT_RSTAT                      FALSE /*TRUE*/                      /*!< FALSE: RAM initialisation by start-up; TRUE: RAM initialisation depend on RSTAT */
#define _SUPPORT_SAVE_POS_AT_SLEEP          FALSE /*TRUE*/                      /*!< FALSE: No storage of last position at SLEEP; TRUE: Save position entering SLEEP */
#define _SUPPORT_SLEEP_EMRUN                FALSE /*TRUE*/                      /*!< FALSE: On sleep request enter SLEEP mode; TRUE: On sleep request first move to Safety position, than enter SLEEP mode (MMP170722-1) */
#define _SUPPORT_SLEEP_RAM_RETENTION        FALSE /*TRUE*/                      /*!< FALSE: Deep-sleep w/o RAM retention; TRUE: Deep-sleep with RAM retention */
#define _SUPPORT_SPECIAL_COMM_FIELD         FALSE /*TRUE*/                      /*!< FALSE: Standard LIN communication; TRUE: Special LIN communication with extra (reserved) fields */
#define _SUPPORT_SPECIAL_PCTPOS             FALSE /*TRUE*/                      /*!< FALSE: No special Percentage position support; TRUE: Special percentage position support */
#define _SUPPORT_SPEED_CHANGE               /*FALSE*/ TRUE                      /*!< FALSE: No speed change allowed during run; TRUE: Speed change allowed */
#if (UART_COMM != FALSE) || (PWM_COMM != FALSE) || (_SUPPORT_ACOUSTIC_NOISE_REDUCTION != FALSE)
                                                                                /* Important Note: Enabling SSCM improves EMC;
                                                                                 * Negative side-effetcs: A-synchronical communication doesn't work (UART, PWM),
                                                                                 *                        Acoustic related to SSCM modulation frequency and PWM-frequency */
#define _SUPPORT_SSCM                       FALSE /*TRUE*/                      /*!< FALSE: Disable CPU & Driver Spread-Spectrum; TRUE: Enable CPU & Driver Spread-Spectrum */
#else  /* (UART_COMM != FALSE) || (PWM_COMM != FALSE) */
#define _SUPPORT_SSCM                       /*FALSE*/ TRUE                      /*!< FALSE: Disable CPU & Driver Spread-Spectrum; TRUE: Enable CPU & Driver Spread-Spectrum */
#endif /* (UART_COMM != FALSE) || (PWM_COMM != FALSE) */
#define _SUPPORT_START_TO_RUN               FALSE /*TRUE*/                      /*!< FALSE: No Start-to-Run support; TRUE: Start-to-Run support */
#define _SUPPORT_STIMER_MODE                C_STIMER_CLKSRC_1MHz /*C_STIMER_CLKSRC_CPU*/  /*!< STimer Clock Source selection */
#define _SUPPORT_TACHO_OUT                  FALSE /*TRUE*/                      /*!< FALSE: No Tacho support; TRUE: Tacho-support on IO[0] */
#define _SUPPORT_TACHO_OUT_OD               FALSE /*TRUE*/                      /*!< FALSE: Push-Pull output; TRUE: Open-Drain output (external pull-up) */
#define TACHO_OUT_IO                        PIN_FUNC_IO_0                       /*!< Tacho-output I/O bit [3:0] */
#define _SUPPORT_TNCTOC                     FALSE /*TRUE*/                      /*!< FALSE: TNCTOC not supported; TNCTOC supported */
#define _SUPPORT_VDDA_5V                    FALSE /*TRUE*/                      /*!< FALSE: Application VDDA is 3.3V; TRUE: Application VDDA is 5.0V (NOTE: Debug/Test-interface doesn't work at 5V) */
#define _SUPPORT_WAKEUP_BY_IO               FALSE /*TRUE*/                      /*!< FALSE: Wake-up by IO disabled; TRUE: Wake-up by IO enabled (MMP211105-2) */
#define _SUPPORT_WHILE_RETRY_LIMITED        FALSE /*TRUE*/                      /*!< FALSE: While-forever; TRUE: While-retry-count */
#define _DEBUG_IO_7_0_ISR                   FALSE /*TRUE*/                      /*!< Test IO[7:0] */

/* Chip Diagnostics */
#define _SUPPORT_DIAG_DRV_PROT              /*FALSE*/ TRUE                      /*!< FALSE: Separate Diagnostics IRQ-Events; TRUE: Single Diagnostic IRQ Event */
#define _SUPPORT_DIAG_IRQCTLR_LEVEL         /*FALSE*/ TRUE                      /*!< FALSE: Edge; TRUE: Level */
#define _SUPPORT_DIAG_OC                    /*FALSE*/ TRUE                      /*!< FALSE: Ignore OC; TRUE: Handle OC IRQ */
#define _SUPPORT_DIAG_OT                    /*FALSE*/ TRUE                      /*!< FALSE: Ignore OT; TRUE: Handle OT IRQ */
#define _SUPPORT_DIAG_RINNEN_ROBUSTNESS     FALSE /*TRUE*/                      /*!< FALSE: No Diagnostics Rinnen Robustness (single fault); TRUE: Include Diagnostics Rinnen Robustness (double fault in 1 Tick) (MMP200628-1) */
#define _SUPPORT_DIAG_VDDA                  /*FALSE*/ TRUE                      /*!< FALSE: Ignore UV VDDA; TRUE: Enable Driver protection against UV VDDA */
#define _SUPPORT_DIAG_VDDA_UV_RESET         FALSE /*TRUE*/                      /*!< FALSE: Don't reset; TRUE: Reset application and IC (MMP231117-1) */
#define _SUPPORT_DIAG_VDDA_UV_SLEEP         FALSE /*TRUE*/                      /*!< FALSE: Don't enter Deep-sleep; TRUE: Enter DEEP-sleep a.s.a.p. (MMP211005-1) */
#define _SUPPORT_DIAG_VDDA_UV_WAKE_UP_TIMER FALSE /*TRUE*/                      /*!< FALSE: Don't enter Deep-sleep; TRUE: Enter DEEP-sleep with Wake-up timer enabled */
#define _SUPPORT_DIAG_VDDAF                 /*FALSE*/ TRUE                      /*!< FALSE: Ignore UV VDDAF; TRUE: Enable Driver protection against UV VDDAF */
#define _SUPPORT_DIAG_VDS                   FALSE /*TRUE*/                      /*!< FALSE: Ignore VDS; TRUE: Handle VDS Event */
#define _SUPPORT_DRV_PROT_TRISTATE          FALSE /*TRUE*/                      /*!< FALSE: Switch to LS/HS; TRUE: Switch to Tri-state (MMP190909-1) */
#define _SUPPORT_DRV_PROT_VDS               /*FALSE*/ TRUE                      /*!< FALSE: Ignore VDS; TRUE: Enable Driver protection against OV High & Low-side VDS (MLX81160 doesn't support VDS) */
#define _SUPPORT_DRV_PROT_VDS_PM_TRI        FALSE /*TRUE*/                      /*!< FALSE: VDS Protection HS (VDS_LS) / LS (VDS_HS); TRUE: VDS Protection Tri-state */
#define _SUPPORT_DRV_PROT_OC                FALSE /*TRUE*/                      /*!< FALSE: Ignore OC; TRUE: Enable Driver protection against Over-Current */
#define _SUPPORT_DRV_PROT_UV_VDDAF          FALSE /*TRUE*/                      /*!< FALSE: Ignore UV VDDAF; TRUE: Enable Driver protection against UV VDDAF */
#define _SUPPORT_DRV_PROT_UV_VDDA           FALSE /*TRUE*/                      /*!< FALSE: Ignore UV VDDA; TRUE: Enable Driver protection against UV VDDA */
#define _SUPPORT_DRV_PROT_UV_VS             FALSE /*TRUE*/                      /*!< FALSE: Ignore UV VS; TRUE: Enable Driver protection against UV VS */
#define _SUPPORT_DRV_PROT_OV_VS             /*FALSE*/ TRUE                      /*!< FALSE: Ignore OV VS; TRUE: Enable Driver protection against OV VS */
#define _SUPPORT_DRV_PROT_OT                FALSE /*TRUE*/                      /*!< FALSE: Ignore OT; TRUE: Enable Driver protection against OT */
#define _SUPPORT_OT_PERMANENT_ERROR         FALSE /*TRUE*/                      /*!< FALSE: Multiple Over-Temperature doesn't result in permanent error; TRUE: It will */
#define _SUPPORT_ADC_BGD                    FALSE /*TRUE*/                      /*!< FALSE: No periodic BGD/ADC check; TRUE Periodic BDG/ADC check */
#define _SUPPORT_ADC_VDDA_VDDD              /*FALSE*/ TRUE                      /*!< FALSE: VDDA/VDDD check only at POR; TRUE: VDDA/VDDD check continuously */
#if defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#define _SUPPORT_ADC_VBOOST                 /*FALSE*/ TRUE                      /*!< FALSE: VBOOST not checked; TRUE: VBOOST check */
#else  /* defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */
#define _SUPPORT_ADC_VBOOST                 FALSE /*TRUE*/                      /*!< FALSE: VBOOST not checked; TRUE: VBOOST check */
#endif /* defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */
#define _SUPPORT_VDDA_VDDD_LPF              /*FALSE*/ TRUE                      /*!< FALSE: No filtering; TRUE: Add LPF filter (MMP210208-1) */
#define C_VDDA_LPF_COEF                     1024                                /*!< VDDA Low-Pass Filter coefficient: 1024/65536, each 500us */
#define C_VDDD_LPF_COEF                     1024                                /*!< VDDD Low-Pass Filter coefficient: 1024/65536, each 500us */

/* Error logging */
#define _SUPPORT_LOG_ERRORS                 /*FALSE*/ TRUE                      /*!< FALSE: No error logging; TRUE: Error logging RAM/LIN-frame support */
#define _SUPPORT_DETAIL_ERROR_LOG           /*FALSE*/ TRUE                      /*!< FALSE: Only main-error-code; TRUE: Sub-error codes */
#define _SUPPORT_LOG_LIN_ERRORS             FALSE /*TRUE*/                      /*!< FALSE: No error logging; TRUE: Error logging RAM/LIN-frame support */
#define _SUPPORT_LOG_ALL_ERRORS             FALSE /*TRUE*/                      /*!< FALSE: Log only new/unique error-codes; TRUE: Log all error-codes (include double) (MMP160914-2) */
#define _SUPPORT_LOG_FATAL_ERRORS           FALSE /*TRUE*/                      /*!< FALSE: No Fatal logging; TRUE: Log fatal in Non Volatile Memory-log (12ms delay) */
#define _SUPPORT_NV_LOG_ERROR               FALSE /*TRUE*/                      /*!< FALSE: Only RAM-log; TRUE: Non Volatile Memory-log */

/* Motor */
#define _SUPPORT_ACCELERATION               C_ACCELERATION_CONSTANT             /*!< Choice of Acceleration and deceleration: Constant or Cosine (MMP230307-1) */
#define _SUPPORT_ANTICOGGING                FALSE /*TRUE*/                      /*!< FALSE: No Anti-cogging solution; TRUE: Anti-cogging support (MMP220519-1) */
#define _SUPPORT_AUTO_SPEED_FOC             /*FALSE*/ TRUE                      /*!< FALSE: FOC from LowSpeed; TRUE: FOC from MaxSpeed */
#define _SUPPORT_BEMF_LEADANGLE_COMPENSATE  FALSE /*TRUE*/                      /*!< FALSE: Without lead-angle compensation; TRUE: With lead-angle compensation (MMP230516-2) */
#define _SUPPORT_BEMF_SINUSOIDAL            /*FALSE*/ TRUE                      /*!< FALSE: None sinusoidal BEMF, use SQRT; TRUE: Sinusoidal BEMF, use Sine/Cosine */
#define _SUPPORT_BEMF_STARPOINT             FALSE /*TRUE*/                      /*!< FALSE: Don't sense star-point; TRUE: Sense star-point via HV-IO[0] (MMP230516-1) */
#define _SUPPORT_BOOST_MODE                 BOOST_MODE_CLIPPING                 /*!< Boost mode selection */
#define _SUPPORT_CDI                        FALSE /*TRUE*/                      /*!< FALSE: Without CDI; TRUE: With CDI (MMP210802-1) */
#define _SUPPORT_CDI_PID                    /*FALSE*/ TRUE                      /*!< FALSE: Use LPF for commutation period; TRUE: Use PID for commutation period */
#define _SUPPORT_CLOSED_LOOP_STARTUP        FALSE /*TRUE*/                      /*!< FALSE: Open-loop start-up; TRUE: Closed-loop start-up */
#define _SUPPORT_COIL_TEMP_COMP             FALSE /*TRUE*/                      /*!< FALSE: No Coil impedance compensation; TRUE: Coil impedance temperature compensation */
#if (C_TOT_COILS_R <= 250)
#define _SUPPORT_COIL_UNIT_10mR             TRUE                                /*!< FALSE: Coil unit is [R]; TRUE: Coil unit is [10mR] */
#define _SUPPORT_COIL_UNIT_100mR            FALSE                               /*!< FALSE: Coil unit is [R]; TRUE: Coil unit is [100mR] */
#elif (C_TOT_COILS_R <= 2500)
#define _SUPPORT_COIL_UNIT_10mR             FALSE                               /*!< FALSE: Coil unit is [R]; TRUE: Coil unit is [10mR] */
#define _SUPPORT_COIL_UNIT_100mR            TRUE                                /*!< FALSE: Coil unit is [R]; TRUE: Coil unit is [100mR] */
#elif (C_TOT_COILS_R <= 25000)
#define _SUPPORT_COIL_UNIT_10mR             FALSE                               /*!< FALSE: Coil unit is [R]; TRUE: Coil unit is [10mR] */
#define _SUPPORT_COIL_UNIT_100mR            FALSE                               /*!< FALSE: Coil unit is [R]; TRUE: Coil unit is [100mR] */
#else
#error "ERROR: Undefined coil units"
#endif
#define _SUPPORT_CURRENT_TEMP_COMPENSATION  FALSE /*TRUE*/                      /*!< FALSE: Motor current temperature compensation OFF; TRUE: Motor current temperature compensated */
#define _SUPPORT_DRVSUP_ALWAYS_ENA          /*FALSE*/ TRUE                      /*!< FALSE: Switch off VDDAF when driver is switch off; TRUE: Keep VDDAF enabled, when driver is switched off (MMP210618-1) */
#define _SUPPORT_FOC_MODE                   FOC_MODE_NONE                       /*!< FOC mode Selection (with Rotor-sensor) */
/*#define _SUPPORT_FOC_MODE                 FOC_MODE_IB*/                       /*!< FOC mode Selection (with Rotor-sensor) */
/*#define _SUPPORT_FOC_MODE                 FOC_MODE_IB_IPK*/                   /*!< FOC mode Selection (with Rotor-sensor) */
#define _SUPPORT_FOC_ID_IQ_MODE             FOC_ID_IQ_LUT /*FOC_ID_IQ_iCLARKE*/  /*!< FOC ID/IQ mode selection (MMP220722-1) */
#define _SUPPORT_FOC_STALL_A_THRSHLD        FALSE /*TRUE*/                      /*!< FALSE: 0% threshold; TRUE: Stall "A" Threshold */
#if (LINPROT == LIN2X_AGS) || (LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52) || (LINPROT == LIN2X_AIRVENT12) || (LINPROT == LIN22_SIMPLE_PCT) || \
    (I2C_SLAVE_PROT == I2C_SLAVE_PROT_POSITIONING) || (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT)
#define _SUPPORT_MOTOR_DIRECTION            C_MOTOR_DIRECTION_CMD               /*!< Motor-direction selection (MMP220826-1)(MMP230710-1) */
#define _SUPPORT_MOTOR_POSITION             C_MOTOR_POS_ROTOR                   /*!< Motor-operation rotor position based */
#elif (LINPROT == LIN2X_FAN01) || (LINPROT == LIN2X_RELAY) || (LINPROT == LIN2X_SOLENOID) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_CONTINUOUS) || \
    (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
#define _SUPPORT_MOTOR_DIRECTION            C_MOTOR_DIRECTION_CMD               /*!< Motor-direction selection (MMP220826-1)(MMP230710-1) */
#define _SUPPORT_MOTOR_POSITION             C_MOTOR_POS_NONE                    /*!< Motor-operation continuous */
#else
#error "ERROR: Motor Position type not defined."
#endif
#define _SUPPORT_MOTOR_SELFTEST             FALSE /*TRUE*/                      /*!< FALSE: No motor driver check at POR; TRUE: Motor driver check at POR */
#define _SUPPORT_MOTION_DET                 C_MOTION_DET_NONE                   /*!< Motion-detector selection (MMP220825-1) */
#if (_SUPPORT_APP_TYPE != C_APP_SOLENOID)
#define _SUPPORT_MULTI_VECTOR_WAVEFORM      FALSE /*TRUE*/                      /*!< FALSE: Single waveform; TRUE: Multiple waveform (supply voltage depended) */
#endif /* (_SUPPORT_APP_TYPE != C_APP_SOLENOID) */
#if defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81339__) || defined (__MLX81350__)
#define _SUPPORT_OSD                        FALSE                               /*!< FALSE: OSD is not used (Device doesn't support it) */
#else  /* defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81339__) || defined (__MLX81350__) */
#define _SUPPORT_OSD                        FALSE /*TRUE*/                      /*!< FALSE: OSD is not used; TRUE: Off-state Driver check is used */
#endif /* defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81339__) || defined (__MLX81350__) */
#define _SUPPORT_PID_D_COEF                 /*FALSE*/ TRUE                      /*!< FALSE: PI Controller, without D; TRUE: PID Controller */
#define _SUPPORT_PID_INLINE                 FALSE /*TRUE*/                      /*!< FALSE: Use call/return; TRUE: in-line function (MMP210608-1) */
#define _SUPPORT_PID_SPEED_CURRENT          /*FALSE*/ TRUE                      /*!< FALSE: PID Error based on Speed; TRUE: PID Error based on Speed * Current */
#define _SUPPORT_PID_SPEED_LIMIT            /*FALSE*/ TRUE                      /*!< FALSE: No speed limit; TRUE: Speed limit */
#if defined (C_SPEED_3) && (1UL * C_SPEED_3 * C_MOTOR_POLE_PAIRS) > 65500UL
#define _SUPPORT_PID_U32                    /*FALSE*/ TRUE                      /*!< FALSE: Max electric-speed 64k eRPM; TRUE: Max speed 128k/256k eRPM (See below) */
#else  /* (C_SPEED_3 * C_MOTOR_POLE_PAIRS) > 65500UL */
#define _SUPPORT_PID_U32                    FALSE /*TRUE*/                      /*!< FALSE: Max electric-speed 64k eRPM; TRUE: Max speed 128k/256k eRPM (See below) */
#endif /* (C_SPEED_3 * C_MOTOR_POLE_PAIRS) > 65500UL */
#define C_FOC_PID_MULT                      2U /*4U 8U*/                        /*!< eRPM multiplier; 2: max 128k eRPM, 4: max. 256k eRPM */
#if (C_MOTOR_COILS == 3U)
#define _SUPPORT_PWM_MIRROR                 /*FALSE*/ TRUE                      /*!< FALSE: Use PWM Independent mode; TRUE: Use PWM Mirror mode */
/* Choice: TRIPLEPHASE_ALLPWM_MIRROR -or- TRIPLEPHASE_TWOPWM_MIRROR_GND -or- TRIPLEPHASE_TWOPWM_INDEPENDENT_GND -or- TRIPLEPHASE_FULL_STEP_BEMF -or- TRIPLEPHASE_FULL_STEP */
#if (_SUPPORT_FULLSTEP != FALSE)
#define _SUPPORT_PWM_MODE                   TRIPLEPHASE_FULL_STEP_BEMF          /*!< Choice of 3-Coil Motor PWM mode (Full-step BEMF) */
#else   /* (_SUPPORT_FULLSTEP != FALSE) */
#define _SUPPORT_PWM_MODE                   TRIPLEPHASE_TWOPWM_INDEPENDENT_GND  /*!< Choice of 3-Coil Motor PWM mode (Sinusoidal) */
#endif /* (_SUPPORT_FULLSTEP != FALSE) */
#elif (C_MOTOR_COILS == 2U) && (C_MOTOR_PHASES == 3U)
/* Choice: BIPOLAR_TRIPHASE_ALLPWM_MIRROR -or- BIPOLAR_TRIPHASE_TWOPWM_INDEPENDENT_GND */
#define _SUPPORT_PWM_MODE                   BIPOLAR_TRIPHASE_ALLPWM_MIRROR      /*!< Choice of 2-Coil Motor/3-phases PWM mode */
#elif (C_MOTOR_COILS == 2U) && (C_MOTOR_PHASES == 4U)
/* Choice: BIPOLAR_PWM_SINGLE_INDEPENDENT_GND -or- BIPOLAR_FULL_STEP_BEMF */
#if (_SUPPORT_FULLSTEP != FALSE)
#define _SUPPORT_PWM_MODE                   BIPOLAR_FULL_STEP_BEMF              /*!< Choice of 2-Coil Motor/4-phases PWM mode (Dual H-bridge) */
#elif (_SUPPORT_HALFSTEP != FALSE)
#define _SUPPORT_PWM_MODE                   BIPOLAR_HALF_STEP_BEMF              /*!< Choice of 2-Coil Motor/4-phases PWM mode (Dual H-bridge) */
#else  /* (_SUPPORT_FULLSTEP != FALSE) */
#define _SUPPORT_PWM_MODE                   BIPOLAR_PWM_SINGLE_INDEPENDENT_GND  /*!< Choice of 2-Coil Motor/4-phases PWM mode (Dual H-bridge) */
#endif /* (_SUPPORT_FULLSTEP != FALSE) */
#elif (C_MOTOR_COILS == 1U)
/* Choice: SINGLE_COIL_PWM -or- SINGLE_COIL_PWM_BIPOLAR */
#define _SUPPORT_PWM_MODE                   SINGLE_COIL_PWM                     /*!< Choice of 1-Coil Motor PWM mode (H-Bridge) */
#else
#error "ERROR: Check _SUPPORT_PWM_MODE"
#endif
#define _SUPPORT_PWM_POL_CORR               /*FALSE*/ TRUE                      /*!< FALSE: PWM Polarity incorrect (720 degrees PWM scheme); TRUE: PWM Polarity correct (360-degrees PWM scheme) */
#define _SUPPORT_PWM_SPREAD_SPECTRUM        FALSE /*TRUE*/                      /*!< FALSE: No Motor-PWM-Frequency Spread-spectrum (Note: Max PWM Duty-Cycle is 98%); TRUE: Motor-PWM Frequency spread-spectrum (Note: Max PWM Duty-Cycle is 95%) */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) || (_SUPPORT_FULLSTEP != FALSE) || (_SUPPORT_HALFSTEP != FALSE) || (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
#define _SUPPORT_PWM_SYNC                   /*FALSE*/ TRUE                      /*!< FALSE: Use PWM_MASTER1_END ISR to update PWM_LT; TRUE: Immediately update PWM_LT registers (FOC) */
#else  /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
#define _SUPPORT_PWM_SYNC                   FALSE /*TRUE*/                      /*!< FALSE: Use PWM_MASTER1_END ISR to update PWM_LT; TRUE: Immediately update PWM_LT registers (FOC) */
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
#define PWM_LIMIT                           PWM_LIMIT_BY_PID                    /*!< Motor PWM Limit selection */
#define _SUPPORT_SOFT_START                 FALSE /*TRUE*/                      /*!< FALSE: Quick start; TRUE: Soft-start */
#define _SUPPORT_SPEED_CTRL                 /*FALSE*/ TRUE                      /*!< FALSE: No speed-control; TRUE: PID-control based on Speed */
#define _SUPPORT_SRP                        FALSE /*TRUE*/                      /*!< FALSE: No SRP; TRUE: Use Static Rotor Positioning */
#define _SUPPORT_VARIABLE_PWM               FALSE /*TRUE*/                      /*!< FALSE: 2-PWM; TRUE: 2- or 3-PWM */
#define C_VAR_PWM_MODE_OFF                  0U                                  /*!< Variable PWM mode OFF */
#define C_VAR_PWM_MODE_ON_DUTY_CYCLE        1U                                  /*!< Variable PWM mode based on PWM Duty Cycle */
#define C_VAR_PWM_MODE_ON_STARTUP_MODE      2U                                  /*!< Variable PWM mode based on start-up mode */
#define _SUPPORT_VAR_PWM_MODE               C_VAR_PWM_MODE_ON_STARTUP_MODE      /*!< Variable PWM Mode selection */
#define _SUPPORT_VOLTAGE_CTRL               FALSE /*TRUE*/                      /*!< FALSE: No voltage-control; TRUE: Voltage control */
#if (_SUPPORT_PWM_MODE == TRIPLEPHASE_ALLPWM_MIRROR) || (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_MIRROR_GND) || \
    (_SUPPORT_PWM_MODE == BIPOLAR_TRIPHASE_ALLPWM_MIRROR)
/* Mirror-mode; Choice: VOLTAGE_SHAPE_SINE -or- VOLTAGE_SHAPE_SVM */
#define _SUPPORT_VOLTAGE_SHAPE              VOLTAGE_SHAPE_SINE                  /*!< Voltage shape choice: Sine or Space Vector Modulation (+15%) */
#elif (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_INDEPENDENT_GND) || \
    (_SUPPORT_PWM_MODE == BIPOLAR_PWM_SINGLE_INDEPENDENT_GND) || \
    (_SUPPORT_PWM_MODE == BIPOLAR_TRIPHASE_TWOPWM_INDEPENDENT_GND)
/* Independent-mode; Choice: VOLTAGE_SHAPE_DSVM_GND (SVM-L) -or- VOLTAGE_SHAPE_DSVM_ALT (SVM-LH) */
#define _SUPPORT_VOLTAGE_SHAPE              VOLTAGE_SHAPE_DSVM_GND              /*!< Voltage shape choice: Discontinue Space Vector Modulation (GND or Alternating between GND and SUP) */
#elif (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR)
#define _SUPPORT_VOLTAGE_SHAPE              VOLTAGE_SHAPE_SINE                  /*!< Voltage shape choice: Sine (alike) */
#elif (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) || (_SUPPORT_PWM_MODE == TRIPLEPHASE_FULL_STEP_BEMF) || (_SUPPORT_PWM_MODE == BIPOLAR_FULL_STEP_BEMF) || (_SUPPORT_PWM_MODE == BIPOLAR_HALF_STEP_BEMF) || (_SUPPORT_PWM_MODE == TRIPLEPHASE_FULL_STEP)
#define _SUPPORT_VOLTAGE_SHAPE              VOLTAGE_SHAPE_NONE                  /*!< Voltage shape choice: None */
#else
#error "ERROR: Check _SUPPORT_VOLTAGE_SHAPE"
#endif
#define _SUPPORT_WINDMILL                   FALSE /*TRUE*/                      /*!< FALSE: No wind-mill check; TRUE: Check for wind-mill (FAN's) */
#define _SUPPORT_WINDMILL_FOC               FALSE /*TRUE*/                      /*!< FALSE: BEMF Voltage tracking (CTimer1); TRUE: BEMF Voltage FOC (ADC-IRQ) */

/* Sensors */
/* Resolver/Triaxis */
#define _SUPPORT_TRIAXIS_MLX90363           FALSE /*TRUE*/                      /*!< FALSE: No MLX90363 Triaxis support; TRUE: MLX90363 Triaxis support (SPI) */
#define _SUPPORT_TRIAXIS_MLX90363_XYZ       FALSE /*TRUE*/                      /*!< FALSE: No XYZ support, use Alpha; TRUE: XYZ-support */
#define _SUPPORT_TRIAXIS_MLX90367           FALSE /*TRUE*/                      /*!< FALSE: No MLX90367 Triaxis support; TRUE: MLX90367 Triaxis support (SENT) */
#define _SUPPORT_TRIAXIS_MLX90372           FALSE /*TRUE*/                      /*!< FALSE: No MLX90372 Triaxis support; TRUE: MLX90372 Triaxis support (SENT) */
#define _SUPPORT_TRIAXIS_MLX90377           FALSE /*TRUE*/                      /*!< FALSE: No MLX90377 Triaxis support; TRUE: MLX90377 Triaxis support (SENT/SPC) (MMP220610-1) */
#if defined (__MLX81332_w90381__) || defined (__MLX81340_w90381__) || ((_SUPPORT_FOC_MODE & FOC_OBSERVER_MASK) == FOC_OBSERVER_SENSORED)
#define _SUPPORT_TRIAXIS_MLX9038x           /*FALSE*/ TRUE                      /*!< FALSE: No MLX9038x Triaxis (Micro-)Resolver support; TRUE: MLX90381 Triaxis (Micro-)Resolver support, IO[1:0] (_SUPPORT_CALIBRATION) */
#else  /* ((_SUPPORT_FOC_MODE & FOC_OBSERVER_MASK) == FOC_OBSERVER_SENSORED) */
#define _SUPPORT_TRIAXIS_MLX9038x           FALSE /*TRUE*/                      /*!< FALSE: No MLX9038x Triaxis (Micro-)Resolver support; TRUE: MLX90381 Triaxis (Micro-)Resolver support, IO[1:0] (_SUPPORT_CALIBRATION) */
#endif /* ((_SUPPORT_FOC_MODE & FOC_OBSERVER_MASK) == FOC_OBSERVER_SENSORED) */
#define _SUPPORT_TRIAXIS_MLX90395           FALSE /*TRUE*/                      /*!< FALSE: No MLX90395 Triaxis support; TRUE: MLX90395 Triaxis support (SPI or I2C) */
#define _SUPPORT_TRIAXIS_MLX90421           FALSE /*TRUE*/                      /*!< FALSE: No MLX90421 Triaxis support; TRUE: MLX90421 Triaxis support (PWM) */
#define _SUPPORT_TRIAXIS_MLX90422           /*FALSE*/ TRUE                      /*!< FALSE: No MLX90422 Triaxis support; TRUE: MLX90422 Triaxis support (SENT) */
#define _SUPPORT_TRIAXIS_MLX90425           FALSE /*TRUE*/                      /*!< FALSE: No MLX90425 Triaxis support; TRUE: MLX90425 Triaxis support (PWM) */
#define _SUPPORT_TRIAXIS_MLX90426           FALSE /*TRUE*/                      /*!< FALSE: No MLX90426 Triaxis support; TRUE: MLX90426 Triaxis support (SENT) */
#define _SUPPORT_HUMIDITY_HDC302x           FALSE /*TRUE*/                      /*!< FALSE: No TI HDC302x Humidity Sensor Support; TRUE: TI HDC302x Humidity Sensor Support (I2C) */
#define _SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513  FALSE /*TRUE*/                  /*!< FALSE: No MLX90513 Inductive Position Sensor support; TRUE: MLX90513 Inductive Position Sensor support (SENT/SPC/PWM) (MMP220920-1) */
#define _SUPPORT_PRESSURE_MLX90829          FALSE /*TRUE*/                      /*!< FALSE: No MLX90829 Pressure Sensor support; TRUE: MLX90829 Pressure Sensor support (SENT) */
/* Sensor I/O */
#if defined (__MLX81332_w90381__)
#define _SUPPORT_RESOLVER_X_IO              PIN_FUNC_IO_2                       /*!< Resolver X on IO[2] (fixed by design) */
#define _SUPPORT_RESOLVER_Y_IO              PIN_FUNC_IO_3                       /*!< Resolver Y on IO[3] (fixed by design) */
#define _SUPPORT_RESOLVER_TEST_IO           PIN_FUNC_IO_NONE                    /*!< Resolver TEST is N.C. */
#elif defined (__MLX81340_w90381__)
#define _SUPPORT_RESOLVER_X_IO              PIN_FUNC_IO_10                      /*!< Resolver X on IO[10] (fixed by design) */
#define _SUPPORT_RESOLVER_Y_IO              PIN_FUNC_IO_11                      /*!< Resolver Y on IO[11] (fixed by design) */
#define _SUPPORT_RESOLVER_TEST_IO           PIN_FUNC_IO_5                       /*!< Resolver TEST on IO[5] (fixed by design) */
#else  /* __MLX813xx_w90381__ */
#define _SUPPORT_RESOLVER_X_IO              PIN_FUNC_IO_2                       /*!< Resolver X on IO[7:0] */
#define _SUPPORT_RESOLVER_Y_IO              PIN_FUNC_IO_3                       /*!< Resolver Y on IO[7:0] */
#define _SUPPORT_RESOLVER_TEST_IO           PIN_FUNC_IO_NONE                    /*!< Resolver TEST is GND */
#endif /* __MLX81332_w90381__ */
#define _SUPPORT_RESOLVER2_X_IO             PIN_FUNC_IO_NONE                    /*!< Resolver2 X on IO[7:0] */
#define _SUPPORT_RESOLVER2_Y_IO             PIN_FUNC_IO_NONE                    /*!< Resolver2 Y on IO[7:0] */
#define _SUPPORT_SENSOR_PWM_IO              PIN_FUNC_IO_0                       /*!< PWM RX on IO[7:0] */
#define _SUPPORT_SENSOR_SENT_IO             PIN_FUNC_IO_0                       /*!< SENT RX on IO[7:0] */
#define _SUPPORT_SENT_LEGACY_CRC            FALSE /*TRUE*/                      /*!< FALSE: SENT Recommended CRC; TRUE: SENT Legacy CRC */
/* Sensor Voltage */
#if (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
    (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE)
#define _SUPPORT_SENSOR_VDDA_5V             /*FALSE*/ TRUE                      /*!< FALSE: Application VDDA is 3.3V; TRUE: Application VDDA is 5.0V (NOTE: Debug/Test-interface doesn't work at 5V) */
#else  /* (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
        * (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) */
#define _SUPPORT_SENSOR_VDDA_5V             FALSE /*TRUE*/                      /*!< FALSE: Application VDDA is 3.3V; TRUE: Application VDDA is 5.0V (NOTE: Debug/Test-interface doesn't work at 5V) */
#endif /* (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
        * (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) */
/* Sensor others */
#define _SUPPORT_TRIAXIS_DETAILS            FALSE /*TRUE*/                      /*!< FALSE: No Version/Oscillator-count details; TRUE: Versions & Oscillator-count details */
#define _SUPPORT_TRIAXIS_MISMATCH           FALSE /*TRUE*/                      /*!< FALSE: No mismatch read-out; TRUE: Mismatch readout via LIN Diagnostics */
#define _SUPPORT_TRIAXIS_STANDBY            FALSE /*TRUE*/                      /*!< FALSE: Periodic Triaxis status/position; TRUE: Triaxis in standby after initial position (MMP161201-1) */

/* (Dual) hall-Latch */
#define _SUPPORT_DUAL_HALL_LATCH_MLX92251   FALSE /*TRUE*/                      /*!< FALSE: No MLX92251 Dual Hall Latch support; TRUE: MLX92251 support */
#define _SUPPORT_DUAL_HALL_LATCH_MLX92255   FALSE /*TRUE*/                      /*!< FALSE: No MLX92255 Dual Hall Latch support; TRUE: MLX92255 support */
#define _SUPPORT_HALL_LATCH                 FALSE /*TRUE*/                      /*!< FALSE: No Hall latch; TRUE Hall latch ISR */
#define _SUPPORT_HALL_LATCH_DIAG            FALSE /*TRUE*/                      /*!< FALSE: Use Hall-latch (Normal operation); TRUE: Use micro-stepping to detect HL-switching */
#define _SUPPORT_HALL_LATCH_MLX9227x        FALSE /*TRUE*/                      /*!< FALSE: No MLX9227x Hall Latch support; TRUE: MLX9227x Hall Latch support */
#define _SUPPORT_HALL_SWITCH_MLX9229x       FALSE /*TRUE*/                      /*!< FALSE: No MLX9229x Hall Switch support; TRUE: MLX9229x Hall Switch support */
#if (_SUPPORT_HALL_LATCH != FALSE)
#define _SUPPORT_NR_OF_HL                   3U                                  /*!< Number of Hall Latches */
#define _SUPPORT_HLA_IO                     PIN_FUNC_IO_1                       /*!< Hall-Latch A on IO[5:1] */
#if (_SUPPORT_NR_OF_HL >= 2)
#define _SUPPORT_HLB_IO                     PIN_FUNC_IO_2                       /*!< Hall-Latch B on IO[6:2]; Support only adjacent IO's */
#if (_SUPPORT_NR_OF_HL >= 3)
#define _SUPPORT_HLC_IO                     PIN_FUNC_IO_3                       /*!< Hall-Latch C on IO[7:3]; Support only adjacent IO's */
#endif /* (_SUPPORT_NR_OF_HL >= 2) */
#endif /* (_SUPPORT_NR_OF_HL >= 3) */
#define _SUPPORT_HALL_POSLOST               FALSE /*TRUE*/                      /*!< FALSE: No Position Lost counter; TRUE: Position Lost counter */
#if (_SUPPORT_FULLSTEP != FALSE)
#define _SUPPORT_MICRO_STEP_COMMUTATION     FALSE /*TRUE*/                      /*!< FALSE: Full-step commutation; TRUE: Micro-step commutation */
#else  /* (_SUPPORT_FULLSTEP != FALSE) */
#define _SUPPORT_MICRO_STEP_COMMUTATION     /*FALSE*/ TRUE                      /*!< FALSE: Full-step commutation; TRUE: Micro-step commutation */
#endif /* (_SUPPORT_FULLSTEP != FALSE) */
#define _SUPPORT_HALL_LATCH_LEADANGLE_COMP  /*FALSE*/ TRUE                      /*!< FALSE: Without lead-angle compensation; TRUE: With lead-angle compensation (MMP230516-2) */
#define _SUPPORT_HALL_LATCH_POLARITY        /*FALSE*/ TRUE                      /*!< FALSE: Falling edge is index 0; TRUE: Raising edge is index 0 */
#define _SUPPORT_HALL_LATCH_POS             FALSE /*TRUE*/                      /*!< FALSE: Use Hall-latch (Normal operation); TRUE: Keep HL Position */
#define _SUPPORT_HALL_LATCH_SMOOTHING       FALSE /*TRUE*/                      /*!< FALSE: Force micro-step index by hall-latch uStep-index table; TRUE: Smoothing micro-step timing */
#define _SUPPORT_HALL_LATCH_SWITCH_MODE     C_3HL_SWITCH_MODE_ALT2_EDGE         /*!< Hall-Ltach Switching mode selection */
#endif /* (_SUPPORT_HALL_LATCH != FALSE) */

/* Sense Magnet */
#if !defined (C_SENSE_MAGNET_POLE_PAIRS)
#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE) ||                                     /* SPI Triaxis */ \
    (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) ||  /* SENT Triaxis */ \
    (_SUPPORT_TRIAXIS_MLX90377 != FALSE) ||                                     /* SENT/SPC Triaxis */ \
    (_SUPPORT_TRIAXIS_MLX9038x != FALSE) ||                                     /* Resolver */ \
    (_SUPPORT_TRIAXIS_MLX90395 != FALSE) ||                                     /* SPI Triaxis */ \
    (_SUPPORT_TRIAXIS_MLX90421 != FALSE) ||                                     /* PWM Triaxis */ \
    (_SUPPORT_TRIAXIS_MLX90422 != FALSE) ||                                     /* SENT Triaxis */ \
    (_SUPPORT_TRIAXIS_MLX90425 != FALSE) ||                                     /* PWM Triaxis */ \
    (_SUPPORT_TRIAXIS_MLX90426 != FALSE) ||                                     /* SENT Triaxis */ \
    (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) /* Dual Hall Latch */
#define C_SENSE_MAGNET_POLE_PAIRS   1U                                          /*!< Sense Magnet number of pole-pairs */
#endif
#endif

/* NTC */
#define _SUPPORT_NTC                        FALSE /*TRUE*/                      /*!< FALSE: No NTC support; TRUE: NTC support */
#define _SUPPORT_NTC_IO                     PIN_FUNC_IO_3                       /*!< NTC on IO[7:0] */

/* Potentiometer Position */
#define _SUPPORT_POTI                       FALSE /*TRUE*/                      /*!< FALSE: No Potentiometer support; TRUE: Potentiometer support (outer-shaft) (IO[3]) */
#define C_POTI_MODE_NONE                    0U                                  /*!< Potentiometer mode: Unknown purpose */
#define C_POTI_MODE_OPEN_LOOP               1U                                  /*!< Potentiometer mode: Open Loop (e.g. Host ECU via LIN Status Frame) */
#define C_POTI_MODE_CLOSED_LOOP             2U                                  /*!< Potentiometer mode: Closed Loop (e.g. motor outer-shaft control) */
#define _SUPPORT_POTI_MODE                  C_POTI_MODE_OPEN_LOOP               /*!< Potentiometer mode selection */
#define _SUPPORT_POTI_IO                    PIN_FUNC_IO_3                       /*!< Potentiometer IO[7:0] */
#define C_POTI_CCW                          FALSE /*TRUE*/                      /*!< FALSE: Potentiometer-shaft same direction as motor-shaft; TRUE: Potentiometer-shaft opposite direction as motor-shaft */
#define C_POS_TRAVEL                        1024U                               /*!< Position feedback (ADC) w/o conversion */

/* Current-spike Position (Sensorless) */
#define _SUPPORT_CURRSPIKE_POSITION         FALSE /*TRUE*/                      /*!< FALSE: No current spike support; TRUE: Current spike support (including positioning) */
#if (_SUPPORT_CURRSPIKE_POSITION != FALSE)
#define C_SPIKE_TYPE_POS_NEG                0U                                  /*!< Spike Type positive/negative */
#define C_SPIKE_TYPE_NEG                    1U                                  /*!< Spike type negative only */
#define _SUPPORT_SPIKE_TYPE                 C_SPIKE_TYPE_NEG                    /*!< Spike type */
#define C_EDGE_SPIKE_SKIP                   3U                                  /*!< Number of ISR skip before next edge sense */
#endif /* (_SUPPORT_CURRSPIKE_POSITION != FALSE) */

/* Stall Detectors */
#define _SUPPORT_REWIND                     FALSE /*TRUE*/                      /*!< Stall Rewind */
#define _SUPPORT_STALL_REVERSE              /*FALSE*/ TRUE                      /*!< Reverse movement at stall */
#define _SUPPORT_STALLDET_A                 /*FALSE*/ TRUE                      /*!< FALSE: Disable over-current stall; TRUE: Enable over-current stall (MMP161201-2) */
#define C_STALL_THRSHLD_DYN                 1U                                  /*!< Stall detector threshold is Dynamic */
#define C_STALL_THRSHLD_STC                 2U                                  /*!< Stall detector threshold is Static */
#define _SUPPORT_STALLDET_A_MODE            /*C_STALL_THRSHLD_DYN*/ C_STALL_THRSHLD_STC  /*!< Stall detector threshold mode: Dynamic or Static */
#if ((_SUPPORT_PWM_MODE == TRIPLEPHASE_FULL_STEP_BEMF) || (_SUPPORT_PWM_MODE == BIPOLAR_FULL_STEP_BEMF) || (_SUPPORT_PWM_MODE == BIPOLAR_HALF_STEP_BEMF)) && (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
#define _SUPPORT_STALLDET_BRI               /*FALSE*/ TRUE                      /*!< Stall detector based on BEMF Rectifier Integrator */
#define _SUPPORT_STALLDET_BZC               FALSE /*TRUE*/                      /*!< Stall detector based on BEMF Zero Cross */
#elif (_SUPPORT_PWM_MODE == TRIPLEPHASE_FULL_STEP_BEMF) && (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_NONE)
#define _SUPPORT_STALLDET_BRI               FALSE /*TRUE*/                      /*!< Stall detector based on BEMF Rectifier Integrator */
#define _SUPPORT_STALLDET_BZC               /*FALSE*/ TRUE                      /*!< Stall detector based on BEMF Zero Cross */
#else  /* (_SUPPORT_PWM_MODE == TRIPLEPHASE_FULL_STEP_BEMF) */
#define _SUPPORT_STALLDET_BRI               FALSE /*TRUE*/                      /*!< Stall detector based on BEMF Rectifier Integrator */
#define _SUPPORT_STALLDET_BZC               FALSE /*TRUE*/                      /*!< Stall detector based on BEMF Zero Cross */
#endif /* (_SUPPORT_PWM_MODE == TRIPLEPHASE_FULL_STEP_BEMF) */
#if (_SUPPORT_HALL_LATCH != FALSE)
#define _SUPPORT_STALLDET_H                 /*FALSE*/ TRUE                      /*!< Stall detector based on Hall-Latch events */
#else  /* (_SUPPORT_HALL_LATCH != FALSE) */
#define _SUPPORT_STALLDET_H                 FALSE /*TRUE*/                      /*!< Stall detector based on Hall-Latch events */
#endif /* (_SUPPORT_HALL_LATCH != FALSE) */
#if (_SUPPORT_PWM_MODE != TRIPLEPHASE_FULL_STEP_BEMF) && (_SUPPORT_PWM_MODE != TRIPLEPHASE_FULL_STEP) && (_SUPPORT_TRIAXIS_MLX9038x == FALSE) && (C_MOTOR_COILS != 1U)
#define _SUPPORT_STALLDET_O                 FALSE /*TRUE*/                      /*!< Stall detector based on current oscillations */
#else  /* (_SUPPORT_PWM_MODE != TRIPLEPHASE_FULL_STEP_BEMF) && (_SUPPORT_TRIAXIS_MLX9038x == FALSE) && (C_MOTOR_COILS != 1U) */
#define _SUPPORT_STALLDET_O                 FALSE /*TRUE*/                      /*!< Stall detector based on current oscillations */
#endif /* (_SUPPORT_PWM_MODE != TRIPLEPHASE_FULL_STEP_BEMF) && (_SUPPORT_TRIAXIS_MLX9038x == FALSE) && (C_MOTOR_COILS != 1U) */
#define _SUPPORT_STALLDET_O_SPEED1          FALSE /*TRUE*/                      /*!< Stall detector based on current oscillations (Below SPEED2 only) */
#define _SUPPORT_STALLDET_O_SPEED2          FALSE /*TRUE*/                      /*!< Stall detector based on current oscillations (Below SPEED3 only) */
#define _SUPPORT_STALLDET_O_SPEED3          FALSE /*TRUE*/                      /*!< Stall detector based on current oscillations (Below SPEED4 only) */
#define _SUPPORT_STALLDET_P                 FALSE /*TRUE*/                      /*!< Stall detector based on position sensor */
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) != FOC_MODE_NONE)
#define _SUPPORT_STALLDET_S                 /*FALSE*/ TRUE                      /*!< Stall detector based on under-speed */
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB) || ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK)
#define _SUPPORT_STALLDET_LA                /*FALSE*/ TRUE                      /*!< Stall detector based on IV-angle */
#else  /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB) || ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK) */
#define _SUPPORT_STALLDET_LA                FALSE /*TRUE*/                      /*!< Stall detector based on IV-angle */
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB) || ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK) */
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ)
#define _SUPPORT_STALLDET_FLUX              /*FALSE*/ TRUE                      /*!< Stall detector based on motor flux */
#else  /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) */
#define _SUPPORT_STALLDET_FLUX              FALSE /*TRUE*/                      /*!< Stall detector based on motor flux */
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) */
#else  /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) != FOC_MODE_NONE) */
#define _SUPPORT_STALLDET_S                 FALSE /*TRUE*/                      /*!< Stall detector based on under-speed */
#define _SUPPORT_STALLDET_LA                /*FALSE*/ TRUE                      /*!< Stall detector based on IV-angle */
#define _SUPPORT_STALLDET_FLUX              FALSE /*TRUE*/                      /*!< Stall detector based on motor flux */
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) != FOC_MODE_NONE) */
#define _SUPPORT_IGNORE_ENDSTOP_STALL       /*FALSE*/ TRUE                      /*!< FALSE: Report all stalls; TRUE: Ignore end-stop stalls */
#if (PWM_COMM != FALSE)
#define _SUPPORT_STALL_AUTO_CLEAR           /*FALSE*/ TRUE                      /*!< FALSE: Stall not cleared by delay-time (e.g by message); TRUE: Stall cleared after delay-time (MMP230614-1) */
#else  /* (PWM_COMM != FALSE) */
#define _SUPPORT_STALL_AUTO_CLEAR           FALSE /*TRUE*/                      /*!< FALSE: Stall not cleared by delay-time (e.g by message); TRUE: Stall cleared after delay-time (MMP230614-1) */
#endif /* (PWM_COMM != FALSE) */
#define _SUPPORT_STALLDET_LPF               /*FALSE*/ TRUE                      /*!< FALSE: Raw data (unfiltered); TRUE: Low Pass Filtered */
#define _SUPPORT_STALLDET_MOVAVG_FLTR       FALSE /*TRUE*/                      /*!< FALSE: Raw data (unfiltered); TRUE: Filtered (by Moving-average) data */

/* MLX81xxx EVB Section */
#define _SUPPORT_EVB                        MLX8133xEVB                         /*!< EVB version selection */
#define _SUPPORT_EVB_APP                    MLX813xxEVB_APP                     /*!< EVB Application selection */

#if defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
/* External shunt (MLX81160 & MLX8134x) */
#if (_SUPPORT_EVB == MLX8134xEVB_100W)
#define _SUPPORT_RSHUNT                     C_RSHUNT_R020                       /*!< Choice of motor current shunt */
#elif (_SUPPORT_EVB == MLX8134xEVB_400W)
#define _SUPPORT_RSHUNT                     C_RSHUNT_R003                       /*!< Choice of motor current shunt */
#else
#define _SUPPORT_RSHUNT                     C_RSHUNT_R100                       /*!< Choice of motor current shunt */
#endif /* (_SUPPORT_EVB == MLX8134xEVB_100W) */
#if (_SUPPORT_RSHUNT == C_RSHUNT_R500)
#define C_ONE_DIV_RSHUNT                    2U                                  /*!< Rshunt = 500mR -> 1/0.500 =  2 */
#elif (_SUPPORT_RSHUNT == C_RSHUNT_R330)
#define C_ONE_DIV_RSHUNT                    3U                                  /*!< Rshunt = 330mR -> 1/0.330 =  3 */
#elif (_SUPPORT_RSHUNT == C_RSHUNT_R250)
#define C_ONE_DIV_RSHUNT                    4U                                  /*!< Rshunt = 250mR -> 1/0.250 =  4 */
#elif (_SUPPORT_RSHUNT == C_RSHUNT_R200)
#define C_ONE_DIV_RSHUNT                    5U                                  /*!< Rshunt = 200mR -> 1/0.200 =  5 */
#elif (_SUPPORT_RSHUNT == C_RSHUNT_R100)
#define C_ONE_DIV_RSHUNT                    10U                                 /*!< Rshunt = 100mR -> 1/0.100 =  10 */
#elif (_SUPPORT_RSHUNT == C_RSHUNT_R050)
#define C_ONE_DIV_RSHUNT                    20U                                 /*!< Rshunt =  50mR -> 1/0.050 =  20 */
#elif (_SUPPORT_RSHUNT == C_RSHUNT_R033)
#define C_ONE_DIV_RSHUNT                    30U                                 /*!< Rshunt =  33mR -> 1/0.033 =  30 */
#elif (_SUPPORT_RSHUNT == C_RSHUNT_R025)
#define C_ONE_DIV_RSHUNT                    40U                                 /*!< Rshunt =  25mR -> 1/0.025 =  40 */
#elif (_SUPPORT_RSHUNT == C_RSHUNT_R022)
#define C_ONE_DIV_RSHUNT                    46U                                 /*!< Rshunt =  22mR -> 1/0.022 =  45.45 */
#elif (_SUPPORT_RSHUNT == C_RSHUNT_R020)
#define C_ONE_DIV_RSHUNT                    50U                                 /*!< Rshunt =  20mR -> 1/0.020 =  50 */
#elif (_SUPPORT_RSHUNT == C_RSHUNT_R010)
#define C_ONE_DIV_RSHUNT                    100U                                /*!< Rshunt =  10mR -> 1/0.010 = 100 */
#elif (_SUPPORT_RSHUNT == C_RSHUNT_R005)
#define C_ONE_DIV_RSHUNT                    200U                                /*!< Rshunt =   5mR -> 1/0.005 = 200 */
#elif (_SUPPORT_RSHUNT == C_RSHUNT_R004)
#define C_ONE_DIV_RSHUNT                    250U                                /*!< Rshunt =   4mR -> 1/0.004 = 250 */
#elif (_SUPPORT_RSHUNT == C_RSHUNT_R003)
#define C_ONE_DIV_RSHUNT                    333U                                /*!< Rshunt =   3mR -> 1/0.003 = 333 */
#elif (_SUPPORT_RSHUNT == C_RSHUNT_R002)
#define C_ONE_DIV_RSHUNT                    500U                                /*!< Rshunt =   2mR -> 1/0.002 = 500 */
#elif (_SUPPORT_RSHUNT == C_RSHUNT_R001)
#define C_ONE_DIV_RSHUNT                    1000U                               /*!< Rshunt =   1mR -> 1/0.001 = 1000 */
#else  /* _SUPPORT_RSHUNT */
#error "Unimplemented Rshunt"
#endif /* _SUPPORT_RSHUNT */

#if defined(C_PID_RUNNING_CURR_LEVEL) && ( (1UL * C_PID_RUNNING_CURR_LEVEL * _SUPPORT_RSHUNT) > 148000UL)
#error "ERROR: Motor running current or Shunt too large!"
#elif defined(C_PID_RUNNING_CURR_MAX) && ( (1UL * C_PID_RUNNING_CURR_MAX * _SUPPORT_RSHUNT) > 148000UL)
#error "ERROR: Motor running current or Shunt too large!"
#endif
#endif /* defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */

/* Interfaces */
/* ANA */
#if (ANA_COMM != FALSE)
#define C_ANA_IO                            PIN_FUNC_IO_4                       /*!< Analogue input IO[7:0] or LIN */
#endif /* (ANA_COMM != FALSE) */

/* CAN */
#if (CAN_COMM != FALSE)
#define _SUPPORT_CAN_MCP2515                FALSE /*TRUE*/                      /*!< FALSE: No MCP2515 CAN Control; TRUE: MCP2515 CAN Controller, CAN V2.0B up to 1Mb/s */
#define _SUPPORT_CAN_TCAN4550               FALSE /*TRUE*/                      /*!< FALSE: No TI TCAN4550 Control; TRUE: TI TCAN4550 Controller, CAN FD up to 5Mb/s  */
#if (_SUPPORT_CAN_MCP2515 != FALSE)
#define MCP2515_OSC                         8000000U /*16000000U*/              /*!< MCP2515 Clock:  8 MHz (NiRen PCB) or 16 MHz (Microchip PCB) */
#define TQ                                  8 /*10 16 20*/                      /*!< Number of Time Quanta per CAN-bit: 8, 10, 16, 20 */
#elif (_SUPPORT_CAN_TCAN4550 != FALSE)
#define TCAN4550_OSC                        /*20000000U*/ 40000000U             /*!< TCAN4550 Clock: 20 MHz or 40 MHz (default) */
#define TCAN4550_AUTO_RETRANSMIT            FALSE /*TRUE*/                      /*!< FALSE: Disable Auto retransmit; TRUE: Auto retransmit if message not transmitted successfully */
#define TCAN4550_FD                         FALSE /*TRUE*/                      /*!< FALSE: Standard CAN; TRUE: Flexible Data-rate CAN */
#define TCAN4550_WD                         FALSE /*TRUE*/                      /*!< FALSE: No TCAN4550 Watchdog usage; TRUE: TCAN4550 Watchdog enabled (600ms) */
#define TQ                                  /*8*/ 10 /*16 20 25*/               /*!< Number of Time Quanta per CAN-bit: 8, 10, 16, 20 or 25 */
#endif
#define _SUPPORT_CAN_IRQ_IO                 PIN_FUNC_IO_NONE                    /*!< CAN IRQ on IO[7:0] (DI) */
#define _SUPPORT_CAN_STDBY_IO               PIN_FUNC_IO_NONE                    /*!< CAN Controller STDBY on IO[7:0] (DO) */
#define _SUPPORT_CAN_RST_IO                 PIN_FUNC_IO_NONE                    /*!< CAN RST on IO[7:0] (DO) */
#endif /* (CAN_COMM != FALSE) */

/* I2C (MMP211104-2) */
#if (I2C_MASTER_PROT != I2C_MASTER_PROT_NONE) || (I2C_SLAVE_PROT != I2C_SLAVE_PROT_NONE)
#define _SUPPORT_I2C                        /*FALSE*/ TRUE                      /*!< FALSE: No I2C support; TRUE: I2C support */
#else  /* (I2C_MASTER_PROT != I2C_MASTER_PROT_NONE) || (I2C_SLAVE_PROT != I2C_SLAVE_PROT_NONE) */
#define _SUPPORT_I2C                        FALSE /*TRUE*/                      /*!< FALSE: No I2C support; TRUE: I2C support */
#endif /* (I2C_MASTER_PROT != I2C_MASTER_PROT_NONE) || (I2C_SLAVE_PROT != I2C_SLAVE_PROT_NONE) */
#if (_SUPPORT_I2C != FALSE) && (I2C_MASTER_PROT != I2C_MASTER_PROT_NONE)
#define _SUPPORT_I2C_MASTER                 /*FALSE*/ TRUE                      /*!< FALSE: No I2C Master; TRUE: I2C Master (SW/PWM) */
#else  /* (_SUPPORT_I2C != FALSE) && (I2C_MASTER_PROT != I2C_MASTER_PROT_NONE) */
#define _SUPPORT_I2C_MASTER                 FALSE /*TRUE*/                      /*!< FALSE: No I2C Master; TRUE: I2C Master (SW/PWM) */
#endif /* (_SUPPORT_I2C != FALSE) && (I2C_MASTER_PROT != I2C_MASTER_PROT_NONE) */
#if (_SUPPORT_I2C != FALSE) && (I2C_SLAVE_PROT != I2C_SLAVE_PROT_NONE)
#define _SUPPORT_I2C_SLAVE                  /*FALSE*/ TRUE                      /*!< FALSE: No I2C Slave; TRUE: I2C Slave (HW/DMA) */
#else  /* (_SUPPORT_I2C != FALSE) && (I2C_SLAVE_PROT != I2C_SLAVE_PROT_NONE) */
#define _SUPPORT_I2C_SLAVE                  FALSE /*TRUE*/                      /*!< FALSE: No I2C Slave; TRUE: I2C Slave (HW/DMA) */
#endif /* (_SUPPORT_I2C != FALSE) && (I2C_SLAVE_PROT != I2C_SLAVE_PROT_NONE) */
#if (_SUPPORT_I2C != FALSE)
#define _SUPPORT_I2C_ERR_ISR                FALSE /*TRUE*/                      /*!< FALSE: I2C Error disabled; TRUE: I2C Error enabled */
#define _SUPPORT_I2C_MSTR_ISR               FALSE /*TRUE*/                      /*!< FALSE: I2C Master IRQ disabled; TRUE: I2C Master IRQ enabled */
#define _SUPPORT_I2C_RESET_ISR              FALSE /*TRUE*/                      /*!< FALSE: I2C doesn't support RESET; TRUE: I2C supports RESET */
#define _SUPPORT_I2C_RW_ISR                 FALSE /*TRUE*/                      /*!< FALSE: I2C Read/Write ISR disabled; TRUE: I2C Read/Write ISR enabled (MLX81334/40/44/46) */
#define _SUPPORT_I2C_MSBitFirst             /*FALSE*/ TRUE                      /*!< FALSE: LSBit first; TRUE: MSbit first */
#define _SUPPORT_I2C_BYTE_SWAP              /*FALSE*/ TRUE                      /*!< FALSE: Byte-order meet I2C-bus transfer; TRUE: Byte-order swapped from I2C-bus transfer */
#else  /* (_SUPPORT_I2C != FALSE) */
#define _SUPPORT_I2C_ERR                    FALSE                               /*!< I2C-interface IRQ support */
#define _SUPPORT_I2C_ISR                    FALSE                               /*!< I2C-interface IRQ support */
#endif /* (_SUPPORT_I2C != FALSE) */
#if (_SUPPORT_I2C_MASTER != FALSE)
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
#define C_I2C_MASTER_SCK_IO                 _SUPPORT_RESOLVER_X_IO              /*!< I2C Master SCK IO[7:1] */
#define C_I2C_MASTER_SDA_IO                 _SUPPORT_RESOLVER_Y_IO              /*!< I2C Master SDA IO[7|0] */
#else  /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
#define C_I2C_MASTER_SCK_IO                 PIN_FUNC_IO_3                       /*!< I2C Master SCK IO[7:1] */
#define C_I2C_MASTER_SDA_IO                 PIN_FUNC_IO_7                       /*!< I2C Master SDA IO[7|0] */
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
#define _SUPPORT_I2C_MASTER_HW_RESOURCE     C_I2C_MASTER_CTIMER1                /*!< I2C Master SCK Resource choice */
#else  /* _SUPPORT_I2C_MASTER */
#define _SUPPORT_I2C_MASTER_HW_RESOURCE     C_I2C_MASTER_NONE                   /*!< I2C Master SCK Resource choice */
#endif /* _SUPPORT_I2C_MASTER */
#if (_SUPPORT_I2C_SLAVE != FALSE)
#if (CHIP_PACKAGE == SO8)
#if defined (__MLX81339__)
#define C_I2C_SLAVE_SCK_IO                  PIN_FUNC_COMM                       /*!< I2C Slave SCK IO[7:1] or COMM (SOIC8) */
#else  /* defined (__MLX81339__) */
#define C_I2C_SLAVE_SCK_IO                  PIN_FUNC_LIN                        /*!< I2C Slave SCK IO[7:1] or LIN (SOIC8) */
#endif /* defined (__MLX81339__) */
#else  /* (CHIP_PACKAGE == SO8) */
#define C_I2C_SLAVE_SCK_IO                  PIN_FUNC_IO_1                       /*!< I2C Slave SCK IO[7:1] or LIN (QFN24) */
#endif /* (CHIP_PACKAGE == SO8) */
#define C_I2C_SLAVE_SDA_IO                  PIN_FUNC_IO_0                       /*!< I2C Slave SDA IO[0] */
#endif /* (_SUPPORT_I2C_SLAVE != FALSE) */

/* IO */
#define _SUPPORT_IO_DUT_SELECT_HVIO         FALSE /*TRUE*/                      /*!< FALSE: No HV-IO DUT Selection; TRUE: HV-IO DUT Selection */
#define _SUPPORT_IO_DUT_SELECT_PWR          FALSE /*TRUE*/                      /*!< FALSE: ; TRUE: DUT Configuration Selection by Power-Line connection (2-3) */
#define _SUPPORT_IO_CMD_SELECT_LVIO         FALSE /*TRUE*/                      /*!< FALSE: No Command/Position selection by IO; TRUE: DUT Command/position by Selection of Low Voltage IO */
#if (_SUPPORT_IO_DUT_SELECT_HVIO != FALSE)
#define HVIO_DUT_SELECT                     PIN_FUNC_IO_0                       /*!< HVIO Selection by HVIO[0] (MLX8133x), HVIO[2:0] (ML8134x) */
#endif /* (_SUPPORT_IO_DUT_SELECT_HVIO != FALSE) */
#if (_SUPPORT_IO_DUT_SELECT_PWR != FALSE) || (_SUPPORT_IO_CMD_SELECT_LVIO != FALSE)
#define _SUPPORT_NR_OF_IO_SELECT            3U                                  /*!< Number of LVIO Selections */
#define IO_DUT_SELECT_A                     PIN_FUNC_IO_1                       /*!< LVIO Selection by LVIO[3:0] */
#if (_SUPPORT_NR_OF_IO_SELECT >= 2U)
#define IO_DUT_SELECT_B                     PIN_FUNC_IO_2                       /*!< LVIO Selection by LVIO[3:1]; Support only adjacent IO's */
#if (_SUPPORT_NR_OF_IO_SELECT >= 3U)
#define IO_DUT_SELECT_C                     PIN_FUNC_IO_3                       /*!< LVIO Selection by LVIO[3:2]; Support only adjacent IO's */
#endif /* (_SUPPORT_NR_OF_IO_SELECT >= 3U) */
#endif /* (_SUPPORT_NR_OF_IO_SELECT >= 2U) */
#endif /* (_SUPPORT_IO_DUT_SELECT_PWR != FALSE) || (_SUPPORT_IO_CMD_SELECT_LVIO != FALSE) */

/* PWM */
#if (PWM_COMM != FALSE)
#define C_PWM_COMM_IN                       PIN_FUNC_LIN                        /*!< PWM-Communication Input: LIN */
#define C_PWM_COMM_OUT                      PIN_FUNC_IO_NONE                    /*!< PWM-Communication Output: None */
#define _SUPPORT_PWM_COMM_ISR               C_PWM_COMM_CTIMER1 /*C_PWM_COMM_IO_ISR*/  /*!< PWM Communication ISR */
#define _SUPPORT_PWMCOMM_STALL              /*FALSE*/ TRUE                      /*!< FALSE: New PWM-in DC clears STALL; TRUE: PWM-in not accepted during STALL (other way of clearing, e.g. delay) (MMP230614-3) */
#endif /* (PWM_COMM != FALSE) */
#if (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90425 != FALSE)
#define _SUPPORT_PWM_HW_RESOURCE            C_PWM_HW_RES_CTIMER1                /*!< PWM Hardware resource */
#else  /* (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90425 != FALSE) */
#define _SUPPORT_PWM_HW_RESOURCE            C_PWM_HW_RES_NONE                   /*!< PWM Hardware resource */
#endif /* (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90425 != FALSE) */

/* SENT */
#if (_SUPPORT_SENSOR_SENT_IO == PIN_FUNC_IO_NONE)
#define _SUPPORT_SENT_HW_RESOURCE           C_SENT_NONE                         /*!< No SENT Hardware resource required */
#else  /* (_SUPPORT_SENSOR_SENT_IO == PIN_FUNC_IO_NONE) */
#define _SUPPORT_SENT_HW_RESOURCE           /*C_SENT_CTIMER1*/ C_SENT_PPM       /*!< SENT Hardware resource */
#endif /* (_SUPPORT_SENSOR_SENT_IO == PIN_FUNC_IO_NONE) */

/* SPI */
#if (_APP_DMA_STRESS_TEST != FALSE) || (_APP_ZWICKAU != FALSE) ||               /* Test-mode */ \
    ((CAN_COMM != FALSE) && ((_SUPPORT_CAN_MCP2515 != FALSE) || (_SUPPORT_CAN_TCAN4550 != FALSE))) ||  /* CAN via SPI */ \
    (SPI_COMM != FALSE) ||                                                      /* SPI Communication */ \
    (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE)  /* SPI Sensor */
#define _SUPPORT_SPI                        /*FALSE*/ TRUE                      /*!< 4-Wire SPI-interface support (eg Zwickau) */
#else  /* (_APP_DMA_STRESS_TEST != FALSE) || (_APP_ZWICKAU != FALSE) */
#define _SUPPORT_SPI                        FALSE /*TRUE*/                      /*!< 4-Wire SPI-interface support */
#endif /* (_APP_DMA_STRESS_TEST != FALSE) || (_APP_ZWICKAU != FALSE) */
#define _SUPPORT_3WIRE_SPI                  FALSE /*TRUE*/                      /*!< 3-Wire SPI-interface support */
#if (_SUPPORT_SPI != FALSE) || (_SUPPORT_3WIRE_SPI != FALSE)
#if (PLL_FREQ == 32000000UL)
#define C_SPI_BAUDRATE  /*100000U*/ 200000U /*400000U 1000000U 2000000U 4000000U 8000000U*/  /*!< SPI Baudrate: 4MBaud (100k-8MBaud) */
#else  /* (PLL_FREQ == 32000000UL) */
#define C_SPI_BAUDRATE  /*100000U*/ 200000U /*400000U 1000000U 2000000U 4000000U*/  /*!< SPI Baudrate: 4MBaud (100k-4MBaud) */
#endif /* (PLL_FREQ == 32000000UL) */
#if (CAN_COMM && (_SUPPORT_CAN_MCP2515 || _SUPPORT_CAN_TCAN4550))
#define _SUPPORT_SPI_CPHA                   FALSE                               /*!< SPI Clock Phase: FALSE: Get data on the leading edge, set data on the trailing edge; TRUE: Set data on the leading edge, get data on the trailing edge */
#else  /* (CAN_COMM && (_SUPPORT_CAN_MCP2515 || _SUPPORT_CAN_TCAN4550)) */
#define _SUPPORT_SPI_CPHA                   TRUE                                /*!< SPI Clock Phase: FALSE: Get data on the leading edge, set data on the trailing edge; TRUE: Set data on the leading edge, get data on the trailing edge */
#endif /* (CAN_COMM && (_SUPPORT_CAN_MCP2515 || _SUPPORT_CAN_TCAN4550)) */
#if (_APP_DMA_STRESS_TEST != FALSE) || (SPI_COMM != FALSE)
#define _SUPPORT_SPI_DMA                    TRUE                                /*!< FALSE: DR-register based; TRUE: DMA buffer based (Full-duplex only) */
#else  /* (_APP_DMA_STRESS_TEST != FALSE) */
#define _SUPPORT_SPI_DMA                    FALSE /*TRUE*/                      /*!< FALSE: DR-register based; TRUE: DMA buffer based (Full-duplex only) */
#endif /* (_APP_DMA_STRESS_TEST != FALSE) */
#if ((CAN_COMM != FALSE) && ((_SUPPORT_CAN_MCP2515 != FALSE) || (_SUPPORT_CAN_TCAN4550 != FALSE)))
#define _SUPPORT_SPI_DUPLEX         C_SPI_HALF_DUPLEX /*C_SPI_FULL_DUPLEX*/     /*!< SPI Duplex mode: HALF/FULL */
#else  /* ((CAN_COMM != FALSE) && ((_SUPPORT_CAN_MCP2515 != FALSE) || (_SUPPORT_CAN_TCAN4550 != FALSE))) */
#define _SUPPORT_SPI_DUPLEX         /*C_SPI_HALF_DUPLEX*/ C_SPI_FULL_DUPLEX     /*!< SPI Duplex mode: HALF/FULL */
#endif /* ((CAN_COMM != FALSE) && ((_SUPPORT_CAN_MCP2515 != FALSE) || (_SUPPORT_CAN_TCAN4550 != FALSE))) */
#define _SUPPORT_SPI_MASTER                 /*FALSE*/ TRUE                      /*!< FALSE: SPI-Slave; TRUE: SPI-Master */
#define _SUPPORT_SPI_WORD_SIZE              SPI_WORD_8BITS /*SPI_WORD_16BITS */ /*!< SPI Word length: 8/16-bits */
#define _SUPPORT_SPI_MOSI_IO                PIN_FUNC_IO_3                       /*!< SPI MOSI on IO[7:0] - MLX90363-Pin6 (brown) */
#define _SUPPORT_SPI_MISO_IO                PIN_FUNC_IO_4                       /*!< SPI MISO on IO[7:0] - MLX90363-Pin2 (red) */
#define _SUPPORT_SPI_CLK                    PIN_FUNC_IO_2                       /*!< SPI SCLK on IO[7:0] - MLX90363-Pin4 (orange) */
#define _SUPPORT_SPI_SS                     PIN_FUNC_IO_1                       /*!< SPI  -SS on IO[7:0] - MLX90363-Pin5 (yellow) */
#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || (CAN_COMM != FALSE)
#define _SUPPORT_SPI_SS_SOFT                /*FALSE*/ TRUE                      /*!< FALSE: SPI Slave Select by SPI; TRUE: SPI Slave Select by SW */
#else  /* (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) */
#define _SUPPORT_SPI_SS_SOFT                FALSE /*TRUE*/                      /*!< FALSE: SPI Slave Select by SPI; TRUE: SPI Slave Select by SW */
#endif /* (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) */
#if (_APP_ZWICKAU != FALSE) || (SPI_COMM != FALSE)
#define _SUPPORT_SPI_ISR                    /*FALSE*/ TRUE                      /*!< SPI-interface IRQ support */
#else  /* (_APP_ZWICKAU != FALSE) */
#define _SUPPORT_SPI_ISR                    FALSE /*TRUE*/                      /*!< SPI-interface IRQ support */
#endif /* (_APP_ZWICKAU != FALSE) */
#define SPI_2WIRE_IO                        SPI_2WIRE_IO0_1                     /*!< Select 2-Wire SPI I/O's */
#else  /* (_SUPPORT_SPI != FALSE) || (_SUPPORT_3WIRE_SPI != FALSE) */
#define _SUPPORT_SPI_ISR                    FALSE                               /*!< SPI-interface IRQ support */
#endif /* (_SUPPORT_SPI != FALSE) || (_SUPPORT_3WIRE_SPI != FALSE) */

#if !defined (__MLX81330__)
/* UART */
#if (_APP_DMA_STRESS_TEST != FALSE) || (UART_COMM != FALSE)
#define _SUPPORT_UART                       /*FALSE*/ TRUE                      /*!< FALSE: No UART support; TRUE: UART support */
#else  /* (_APP_DMA_STRESS_TEST != FALSE) */
#define _SUPPORT_UART                       FALSE /*TRUE*/                      /*!< FALSE: No UART support; TRUE: UART support */
#endif /* (_APP_DMA_STRESS_TEST != FALSE) */
#if (_SUPPORT_UART != FALSE)
#define _SUPPORT_UART_RX_IO                 PIN_FUNC_IO_4                       /*!< UART RX on IO[7:0] */
#define _SUPPORT_UART_TX_IO                 PIN_FUNC_IO_5                       /*!< UART TX on IO[7:0] */
#define _SUPPORT_UART_LSbitFirst            /*FALSE*/ TRUE                      /*!< UART RX/TX Bit order: FALSE: MSbit first; TRUE: LSbit first */
#define _SUPPORT_UART_LSByteFirst           /*FALSE*/ TRUE                      /*!< UART RX/TX Byte order: FALSE: MSByte first; TRUE: LSByte first */
#if (_SUPPORT_UART_IF_APP == C_UART_IF_BLUETOOTH) || (_SUPPORT_UART_IF_APP == C_UART_IF_TERMINAL) || (_SUPPORT_UART_IF_APP == C_UART_IF_MLX_ACT)
#define _SUPPORT_UART_DMA                   /*FALSE*/ TRUE                      /*!< FALSE: UART RX/TX by Receive/Transmit IO; TRUE: UART RX/TX by DMA */
#define _SUPPORT_UART_DMA_VAR_LEN           /*FALSE*/ TRUE                      /*!< FALSE: Fixed length, e.g. 16; TRUE: Variable length (RX=1, TX=n) (MMP240313-1) */
#else  /* (_SUPPORT_UART_IF_APP == C_UART_IF_BLUETOOTH) */
#define _SUPPORT_UART_DMA                   /*FALSE*/ TRUE                      /*!< FALSE: UART RX/TX by Receive/Transmit IO; TRUE: UART RX/TX by DMA */
#define _SUPPORT_UART_DMA_VAR_LEN           FALSE /*TRUE*/                      /*!< FALSE: Fixed length, e.g. 16; TRUE: Variable length (RX=1, TX=n) (MMP240313-1) */
#endif /* (_SUPPORT_UART_IF_APP == C_UART_IF_BLUETOOTH) */
#define _SUPPORT_UART_ISR                   /*FALSE*/ TRUE                      /*!< FALSE: UART not used or doesn't use IRQ; TRUE: UART use IRQ's */
/* UART baudrate dependent on application and CPU-clock */
#if (_SUPPORT_UART_IF_APP == C_UART_IF_SCOPE)
#if (PLL_FREQ == 32000000UL)
#define C_UART_BAUDRATE  /*9600U 19200U 38400U 57600U 76800U 115200U 230400U 500000U*/ 1000000U /*2000000U*/  /*!< UART Baudrate: 1MBaud */
#elif (PLL_FREQ == 29500000UL)
#define C_UART_BAUDRATE  /*9600U 19200U 38400U 57600U 76800U 115200U 230400U 460800U*/ 921600U  /*!< UART Baudrate: 921.6 kBaud */
#else  /* PLL_FREQ */
#define C_UART_BAUDRATE  /*9600U 19200U 38400U 57600U 76800U*/ 115200U          /*!< UART Baudrate: 115200 Baud */
#endif /* PLL_FREQ */
#elif (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR)
#if (PLL_FREQ == 32000000UL)
/* Highest Baudrate: ~1MBaud preferred */
#define C_UART_BAUDRATE  /*9600U 19200U 38400U 57600U 76800U 115200U 230400U 500000U*/ 1000000U /*2000000U*/  /*!< UART Baudrate: 1MBaud */
#elif (PLL_FREQ == 29500000UL)
#define C_UART_BAUDRATE  /*9600U 19200U 38400U 57600U 76800U 115200U 230400U 460800U*/ 921600U  /*!< UART Baudrate: 921.6 kBaud */
#else  /* PLL_FREQ */
#define C_UART_BAUDRATE  /*9600U 19200U 38400U 57600U 76800U*/ 115200U          /*!< UART Baudrate: 115200 Baud */
#endif /* PLL_FREQ */
#elif (_SUPPORT_UART_IF_APP == C_UART_IF_BLUETOOTH)
#define C_UART_BAUDRATE  9600U /*19200U 38400U 57600U 76800U 115200U*/          /*!< UART Baudrate: 9600 Baud (BT module HC-08/42 configuration/(default) work mode) */
#elif (_SUPPORT_UART_IF_APP == C_UART_IF_TERMINAL)
#define C_UART_BAUDRATE  /*9600U 19200U 38400U 57600U 76800U*/ 115200U          /*!< UART Baudrate: 115200 Baud */
#else  /* (__SUPPORT_UART_IF_APP) */
/* Standard Baudrate: 115200 Baud */
#define C_UART_BAUDRATE  /*9600U 19200U 38400U 57600U 76800U*/ 115200U          /*!< UART Baudrate: 115200 Baud */
#endif /* (_SUPPORT_UART_IF_APP) */
#else  /* _SUPPORT_UART */
#define _SUPPORT_UART_ISR                   FALSE /*TRUE*/                      /*!< FALSE: UART not used or doesn't use IRQ; TRUE: UART use IRQ's */
#endif /* _SUPPORT_UART */
#else  /* !defined (__MLX81330__) */
#define _SUPPORT_UART                       FALSE /*TRUE*/                      /*!< FALSE: No UART present */
#endif /* !defined (__MLX81330__) */

/* ADC Clock */
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
#if (PLL_FREQ == 40000000UL)
#if (_APP_DMA_STRESS_TEST != FALSE)
#define ADC_FREQ                            8000000UL                           /*!< ADC frequency:  8 MHz */
#else  /* (_APP_DMA_STRESS_TEST != FALSE) */
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
#define ADC_FREQ                            5714285UL                           /*!< ADC frequency:  5.7 MHz */
#else  /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
#define ADC_FREQ                            4000000UL                           /*!< ADC frequency:  4 MHz */
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
#endif /* (_APP_DMA_STRESS_TEST != FALSE) */
#elif (PLL_FREQ == 32000000UL)
#if (_APP_DMA_STRESS_TEST != FALSE)
#define ADC_FREQ                            8000000UL                           /*!< ADC frequency:  8 MHz */
#else  /* (_APP_DMA_STRESS_TEST != FALSE) */
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
#define ADC_FREQ                            5333333UL                           /*!< ADC frequency:  5.3 MHz */
#else
#define ADC_FREQ                            4000000UL                           /*!< ADC frequency:  4 MHz */
#endif
#endif /* (_APP_DMA_STRESS_TEST != FALSE) */
#elif (PLL_FREQ == 29500000UL)
#if (_APP_DMA_STRESS_TEST != FALSE)
#define ADC_FREQ                            7375000UL                           /*!< ADC frequency:  7.375 MHz */
#else  /* (_APP_DMA_STRESS_TEST != FALSE) */
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
#define ADC_FREQ                            5900000UL                           /*!< ADC frequency:  5.9 MHz */
#else  /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
#define ADC_FREQ                            4214286UL                           /*!< ADC frequency:  4.2 MHz */
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
#endif /* (_APP_DMA_STRESS_TEST != FALSE) */
#elif (PLL_FREQ == 28000000UL)
#if (_APP_DMA_STRESS_TEST != FALSE)
#define ADC_FREQ                            7000000UL                           /*!< ADC frequency:  7 MHz */
#else  /* (_APP_DMA_STRESS_TEST != FALSE) */
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
#define ADC_FREQ                            5600000UL                           /*!< ADC frequency:  5.6 MHz */
#else  /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
#define ADC_FREQ                            4000000UL                           /*!< ADC frequency:  4 MHz */
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
#endif /* (_APP_DMA_STRESS_TEST != FALSE) */
#elif (PLL_FREQ == 24000000UL)
#if (_APP_DMA_STRESS_TEST != FALSE)
#define ADC_FREQ                            8000000UL                           /*!< ADC frequency:  8 MHz */
#else  /* (_APP_DMA_STRESS_TEST != FALSE) */
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
#define ADC_FREQ                            6000000UL                           /*!< ADC frequency:  6 MHz */
#else  /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
#define ADC_FREQ                            4000000UL                           /*!< ADC frequency:  4 MHz */
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
#endif /* (_APP_DMA_STRESS_TEST != FALSE) */
#else
#define ADC_FREQ                            4000000UL                           /*!< ADC frequency:  4 MHz */
#endif /* PLL_FREQ */
#elif defined (__MLX81160__) || defined (__MLX81339__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) || defined (__MLX81350__)
#if (PLL_FREQ == 48000000UL)
#define ADC_FREQ                            (PLL_FREQ / 3)                      /*!< ADC frequency: 16 MHz (min. 15MHz, max. 20 MHz) */
/* #define ADC_FREQ                         (PLL_FREQ / 6) */                   /*!< ADC frequency: 8 MHz (min. 15MHz, max. 20 MHz) */
/* #define ADC_FREQ                         (PLL_FREQ / 12) */                  /*!< ADC frequency: 4 MHz (min. 15MHz, max. 20 MHz) */
#elif (PLL_FREQ == 40000000UL)
#define ADC_FREQ                            (PLL_FREQ / 2)                      /*!< ADC frequency: 20 MHz (min. 15MHz, max. 20 MHz) */
/* #define ADC_FREQ                         (PLL_FREQ / 5) */                   /*!< ADC frequency: 8 MHz (min. 15MHz, max. 20 MHz) */
/* #define ADC_FREQ                         (PLL_FREQ / 10) */                  /*!< ADC frequency: 4 MHz (min. 15MHz, max. 20 MHz) */
#elif (PLL_FREQ == 36000000UL)
#define ADC_FREQ                            (PLL_FREQ / 2)                      /*!< ADC frequency: 20 MHz (min. 15MHz, max. 20 MHz) */
/* #define ADC_FREQ                         (PLL_FREQ / 5) */                   /*!< ADC frequency: 7.2 MHz (min. 15MHz, max. 20 MHz) */
/* #define ADC_FREQ                         (PLL_FREQ / 9) */                   /*!< ADC frequency: 4 MHz (min. 15MHz, max. 20 MHz) */
#elif (PLL_FREQ == 32000000UL)
#define ADC_FREQ                            (PLL_FREQ / 2)                      /*!< ADC frequency: 16 MHz (min. 15MHz, max. 20 MHz) */
/* #define ADC_FREQ                         (PLL_FREQ / 4) */                   /*!< ADC frequency: 8 MHz (min. 15MHz, max. 20 MHz) */
/* #define ADC_FREQ                         (PLL_FREQ / 8) */                   /*!< ADC frequency: 4 MHz (min. 15MHz, max. 20 MHz) */
#elif (PLL_FREQ == 29500000UL)
#define ADC_FREQ                            (PLL_FREQ / 2)                      /*!< ADC frequency: 14.75 MHz (min. 15MHz, max. 20 MHz) */
/* #define ADC_FREQ                         (PLL_FREQ / 4) */                   /*!< ADC frequency: 7.375 MHz (min. 15MHz, max. 20 MHz) */
/* #define ADC_FREQ                         (PLL_FREQ / 8) */                   /*!< ADC frequency: 3.6875 MHz (min. 15MHz, max. 20 MHz) */
#elif (PLL_FREQ == 28000000UL)
#define ADC_FREQ                            (PLL_FREQ / 2)                      /*!< ADC frequency: 14 MHz (min. 15MHz, max. 20 MHz) */
/* #define ADC_FREQ                         (PLL_FREQ / 4) */                   /*!< ADC frequency: 7 MHz (min. 15MHz, max. 20 MHz) */
/* #define ADC_FREQ                         (PLL_FREQ / 7) */                   /*!< ADC frequency: 4 MHz (min. 15MHz, max. 20 MHz) */
#elif (PLL_FREQ == 24000000UL)
#define ADC_FREQ                            (PLL_FREQ / 2)                      /*!< ADC frequency: 12 MHz (min. 15MHz, max. 20 MHz) */
/* #define ADC_FREQ                         (PLL_FREQ / 3) */                   /*!< ADC frequency: 8 MHz (min. 15MHz, max. 20 MHz) */
/* #define ADC_FREQ                         (PLL_FREQ / 6) */                   /*!< ADC frequency: 4 MHz (min. 15MHz, max. 20 MHz) */
#elif (PLL_FREQ <= 20000000UL)
#define ADC_FREQ                            (PLL_FREQ / 1)                      /*!< ADC frequency: 20 MHz (min. 15MHz, max. 20 MHz) */
/* #define ADC_FREQ                         (PLL_FREQ / 2) */                   /*!< ADC frequency: 10 MHz (min. 15MHz, max. 20 MHz) */
/* #define ADC_FREQ                         (PLL_FREQ / 5) */                   /*!< ADC frequency: 4 MHz (min. 15MHz, max. 20 MHz) */
#endif
#endif

/* Checks */
#if (_SUPPORT_TRIAXIS_MLX90377 != FALSE) && (_SUPPORT_SENSOR_SENT_IO != PIN_FUNC_IO_NONE) && (_SUPPORT_SSCM != FALSE)
#error "ERROR: SPC/SENT with 1.5us Tick-time doesn't work well with SSCM enabled"
#endif
/* Variable PWM (0-50 or 0-33-67) only supported with SVM-L */
#if (_SUPPORT_VARIABLE_PWM != FALSE) && (_SUPPORT_VOLTAGE_SHAPE != VOLTAGE_SHAPE_DSVM_GND)
#error "ERROR: Variable PWM (0-33-67) only supported using SVM-L"
#endif
/* Inverse Clarke (Cartesian) only supported with SVM-L */
#if (_SUPPORT_FOC_ID_IQ_MODE == FOC_ID_IQ_iCLARKE) && (_SUPPORT_VOLTAGE_SHAPE != VOLTAGE_SHAPE_DSVM_GND)
#error "ERROR: Inverse Clarke (Cartesian) only supported with SVM-L"
#endif

#endif /* _APP_SWITCHES_H_ */

/* EOF */
