/*!*************************************************************************** *
 * \file        AppBuild.h
 * \brief       MLX8133x Application Build file
 *
 * \note        project MLX813xx
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
 * Copyright (C) 2017-2022 Melexis N.V.
 * The Software is being delivered 'AS IS' and Melexis, whether explicitly or
 * implicitly, makes no warranty as to its Use or performance.
 * The user accepts the Melexis Firmware License Agreement.
 *
 * Melexis confidential & proprietary
 *
 *
 * *************************************************************************** */

#ifndef _APP_BUILD_H_

#define _APP_BUILD_H_

/*#define __MLX81332_w90381__ 1*/                                               /*!< Special IC version 81332 with integrated 90381 Resolver */
/*#define __MLX81340_w90381__ 1*/                                               /*!< Special IC version 81340 with integrated 90381 Resolver */

/*!*************************************************************************** */
/*                           INCLUDES                                          */
/* *************************************************************************** */
#include <stdint.h>
#include <syslib.h>

#if defined (__MLX81160__)
#include "../camculib/ioports_MLX81160.h"                                       /* MLX81160 I/O-ports definitions */
#include "../camculib/mem_map_MLX81160.h"                                       /* MLX81160 Memory-map */
#elif defined (__MLX81330__)
#include "../camculib/ioports_MLX81330.h"                                       /* MLX81330 I/O-ports definitions */
#include "../camculib/mem_map_MLX81330.h"                                       /* MLX81330 Memory-map */
#elif defined (__MLX81332__)
#include "../camculib/ioports_MLX81332.h"                                       /* MLX81332 I/O-ports definitions */
#include "../camculib/mem_map_MLX81332.h"                                       /* MLX81332 Memory-map */
#elif defined (__MLX81334__)
#include "../camculib/ioports_MLX81334.h"                                       /* MLX81334 I/O-ports definitions */
#include "../camculib/mem_map_MLX81334.h"                                       /* MLX81334 Memory-map */
#elif defined (__MLX81339__)
#include "camculib/ioports_MLX81339.h"                                          /* MLX81339 I/O-ports definitions */
#include "camculib/mem_map_MLX81339.h"                                          /* MLX81339 Memory-map */
#elif defined (__MLX81340__)
#include "../camculib/ioports_MLX81340.h"                                       /* MLX81340 I/O-ports definitions */
#include "../camculib/mem_map_MLX81340.h"                                       /* MLX81340 Memory-map */
#elif defined (__MLX81344__)
#include "../camculib/ioports_MLX81344.h"                                       /* MLX81344 I/O-ports definitions */
#include "../camculib/mem_map_MLX81344.h"                                       /* MLX81344 Memory-map */
#elif defined (__MLX81346__)
#include "../camculib/ioports_MLX81346.h"                                       /* MLX81346 I/O-ports definitions */
#include "../camculib/mem_map_MLX81346.h"                                       /* MLX81346 Memory-map */
#elif defined (__MLX81350__)
#include "../camculib/ioports_MLX81350.h"                                       /* MLX81350 I/O-ports definitions */
#include "../camculib/mem_map_MLX81350.h"                                       /* MLX81350 Memory-map */
#else
#error "ERROR: ioports-file not found"
#endif

#include "AppDefines.h"                                                         /* Build Defines for C and S files */
#include "AppSwitches.h"                                                        /* Application Switches for C and S files */

/*!*************************************************************************** */
/*                           DEFINITIONS                                       */
/* *************************************************************************** */
/* Debug: Application Code */
#define _CHIP_PACKAGE                       /*SO8*/ QFN24 /*QFN32 TQFP48*/      /*!< IC package selection */
#if (_APP_ZWICKAU != FALSE) || (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || \
    (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) || (CAN_COMM != FALSE) || (UART_COMM != FALSE)
#define _DEBUG_IO                           FALSE /*TRUE*/                      /*!< Debug by I/O[3:1] */
#else  /* (_SUPPORT_TRIAXIS_MLX90363 != FALSE) */
#define _DEBUG_IO                           FALSE /*TRUE*/                      /*!< Debug by I/O[3:1] */
#endif /* (_SUPPORT_TRIAXIS_MLX90363 != FALSE) */
#define _DEBUG_ACTUATOR_MODE                FALSE /*TRUE*/                      /*!< IO[A]; FALSE: Nothing; TRUE: Ramp-up/down DC-actuator */
#define _DEBUG_ALIVE                        FALSE /*TRUE*/                      /*!< IO[A]; FALSE: Nothing; TRUE: Toggle each main-loop */
#define _DEBUG_BEMF_SENSE                   FALSE /*TRUE*/                      /*!< IO[D] */
#define _DEBUG_CAN                          FALSE /*TRUE*/                      /*!< FALSE: Nothing; TRUE: CAN Debug info */
#define _DEBUG_CAN_TCAN4550                 FALSE /*TRUE*/                      /*!< FALSE: Nothing; TRUE: Extra SPI communication */
#define _DEBUG_COMMUT_ISR                   FALSE /*TRUE*/                      /*!< IO[B]; FALSE: Nothing; TRUE: Set IO[B] high during commutation ISR */
#define _DEBUG_CPU_CLOCK                    FALSE /*TRUE*/                      /*!< CPU-clock DIV 4 on IO[C] */
#define _DEBUG_CPU_HALT                     FALSE /*TRUE*/                      /*!< IO[A]; FALSE: Nothing; TRUE: Set IO[A] high during CPU HALT */
#define _DEBUG_CPU_LOAD                     FALSE /*TRUE*/                      /*!< IO[A:D]: FALSE: Nothing; TRUE: Set IO[A:D] during IRQ [3:6] to estimate CPU-load, IO[E] during priority [1] of AWD */
#define _DEBUG_DEBUGGER_SUPPORT             FALSE /*TRUE*/                      /*!< Debugger support */
#define _DEBUG_DIAG                         FALSE /*TRUE*/                      /*!< IO[B]; FALSE: Nothing; TRUE: Toggle during ISR_DIAG */
#define _DEBUG_DIAG_OC                      FALSE /*TRUE*/                      /*!< IO[C]; FALSE: Nothing; TRUE: Toggle during HandleDiagnosticsOC() */
#define _DEBUG_DIAG_OT                      FALSE /*TRUE*/                      /*!< IO[A]; FALSE: Nothing; TRUE: Toggle during ISR_DIAG_OT */
#define _DEBUG_DIAG_OV_UV                   FALSE /*TRUE*/                      /*!< IO[A:B]; FALSE: Nothing; TRUE: Toggle during ISR_OV_UV [A] OV/UV On, [B] Each 500us check */
#define _DEBUG_DIAG_PROT                    FALSE /*TRUE*/                      /*!< IO[A] */
#define _DEBUG_DIAG_UV_VDDA                 FALSE /*TRUE*/                      /*!< IO[A] */
#define _DEBUG_DIAG_VDS                     FALSE /*TRUE*/                      /*!< IO[B] */
#define _DEBUG_DIAG_VDDD_UV                 FALSE /*TRUE*/                      /*!< IO[A]; FALSE: Nothing; TRUE: Set at main_init() */
#define _DEBUG_DIAG_VDDAF_UV                FALSE /*TRUE*/                      /*!< IO[E]; FALSE: Nothing; TRUE: Set at main_init() */
#define _DEBUG_EMC_BCI_TEST                 FALSE /*TRUE*/                      /*!< IO[?]; EMC/BCI test support */
#define _DEBUG_FATAL                        FALSE /*TRUE*/                      /*!< IO[A]; Fatal handle */
#define _DEBUG_FLASH_BIST                   FALSE /*TRUE*/                      /*!< IO[A]; FALSE: Nothing; TRUE: Active during Flash-BIST */
#define _DEBUG_FLPD_ERASE                   FALSE /*TRUE*/                      /*!< IO[A]: Active pulse during erase */
#define _DEBUG_FLPD_WRITE                   FALSE /*TRUE*/                      /*!< IO[B]: Active pulse during write */
#define _DEBUG_FOC_IB_IQ                    FALSE /*TRUE*/                      /*!< IO[4] FOC IB Iq on IO as Duty cycle */
#define _DEBUG_FOC_PERF                     FALSE /*TRUE*/                      /*!< IO[C:D] FOC Performance */
#define _DEBUG_HALL_LATCH_ISR               FALSE /*TRUE*/                      /*!< IO[A]: Hall-Latch ISR period */
#define _DEBUG_I2C                          FALSE /*TRUE*/                      /*!< IO[C:B] I2C IRQ's */
#define _DEBUG_I2C_MASTER                   FALSE /*TRUE*/                      /*!< IO[D] I2C Master */
#define _DEBUG_INDUCTIVE_POS_SENSOR         FALSE /*TRUE*/                      /*!< IO[A:B] Inductive Position Sensor debugging */
#define _DEBUG_IO_ADC                       FALSE /*TRUE*/                      /*!< IO[A]; FALSE: Nothing; TRUE: Toggle IO[A] for each ADC conversion */
#define _DEBUG_IO_CPU_PERF                  FALSE /*TRUE*/                      /*!< IO[B:A]; FALSE: Nothing; TRUE: Toggle IO[1:0] 4-times */
#define _DEBUG_LIN_BUS_TO                   FALSE /*TRUE*/                      /*!< IO[A]; FALSE: Nothing; TRUE: Set IO[A] after LIN Bus-time-out has been detected */
#define _DEBUG_LIN_ISR                      FALSE /*TRUE*/                      /*!< IO[A]; FALSE: Nothing; TRUE: Set IO[A] high during LIN ISR */
#define _DEBUG_LINAA                        FALSE /*TRUE*/                      /*!< IO[B:A]; FALSE: Nothing; TRUE: Set IO[A] during Step, Set IO[B] during wait */
#define _DEBUG_MOTION                       FALSE /*TRUE*/                      /*!< IO[B:A]; FALSE: Nothing; TRUE: Set IO[A] during motion detection */
#define _DEBUG_MOTOR_START                  FALSE /*TRUE*/                      /*!< IO[E]; FALSE: Nothing; TRUE Set IO[E] during Motor-Start */
#define _DEBUG_MOTOR_STOP                   FALSE /*TRUE*/                      /*!< IO[B]; FALSE: Nothing; TRUE: Set IO[B] during Motor-Stop */
#define _DEBUG_NV_EMERGENCY_STORE           FALSE /*TRUE*/                      /*!< IO[A]; FALSE: Nothing; TRUE: Active during Non Volatile Memory Emergency Write */
#define _DEBUG_NV_WRITE                     FALSE /*TRUE*/                      /*!< IO[A]; FALSE: Nothing; TRUE: Set IO[A] high during Non Volatile Memory Write */
#define _DEBUG_NV_WRITE_BACKGROUND          FALSE /*TRUE*/                      /*!< IO[A]; FALSE: nothing; TRUE: Toggle IO[A] as long as Non Volatile Memory background write is busy */
#define _DEBUG_OSD                          FALSE /*TRUE*/                      /*!< IO[A] OSD */
#define _DEBUG_PID_CONTROL                  FALSE /*TRUE*/                      /*!< IO[A]; FALSE: Nothing; TRUE: Active during Non Volatile Memory Emergency Write */
#define _DEBUG_PRESSURE                     FALSE /*TRUE*/                      /*!< IO[A:B] Pressure sensor debugging */
#define _DEBUG_PWM_COMM                     FALSE /*TRUE*/                      /*!< IO[B]; FALSE: Nothing; TRUE: Active during PWM Communication */
#define _DEBUG_PWM_POL                      FALSE /*TRUE*/                      /*!< IO[D]; FALSE: Nothing; TRUE: IO[A]: Set/Clear POL-bit */
#define _DEBUG_PWM_TO_DRV_DELAY             FALSE /*TRUE*/                      /*!< IO[4]; PWM on IO[4] */
#define _DEBUG_PWM_VARIABLE_PWM             FALSE /*TRUE*/                      /*!< IO[A]; FALSE: Nothing; TRUE: IO[A] LOW = 2-coil/phase, HIGH = 3-coil-phase */
#define _DEBUG_RELAY_SOLENOID               FALSE /*TRUE*/                      /*!< IO[B:A] */
#define _DEBUG_RESOLVER                     FALSE /*TRUE*/                      /*!< Monitor Resolver angle */
#define _DEBUG_RSTAT                        FALSE /*TRUE*/                      /*!< IO[B:A]; FALSE: No RAM Retention check; TUE: RAM Retention check (Requires: _DEBUG_MOTOR_CURRENT_FLT) */
#define _DEBUG_S2R                          FALSE /*TRUE*/                      /*!< IO[D:C]; FALSE: Nothing; TRUE: IO[A]: Sub-Function, IO[B]: ADC */
#define _DEBUG_SELFTEST                     FALSE /*TRUE*/                      /*!< IO[B]; FALSE: Nothing; TRUE: During Motor SelfTest */
#define _DEBUG_SPI                          FALSE /*TRUE*/                      /*!< IO[4:3] */
#define _DEBUG_SPIKE_DETECTED               FALSE /*TRUE*/                      /*!< IO[D/E] high during spike detected */
#define _DEBUG_SRP                          FALSE /*TRUE*/                      /*!< IO[A] Static Rotor Position debugging */
#define _DEBUG_STALL_A                      FALSE /*TRUE*/                      /*!< IO[A] */
#define _DEBUG_STALL_O                      FALSE /*TRUE*/                      /*!< IO[C] */
#define _DEBUG_STIMER_ISR                   FALSE /*TRUE*/                      /*!< IO[A]; FALSE: Nothing; TRUE: Set IO[C] high during ISR */
#define _DEBUG_SUPPLY_ROBUSTNESS            FALSE /*TRUE*/                      /*!< IO[A:B]; FALSE: Nothing; TRUE: IO[A:B] ADC_Init()/ISR_UV_VS */
#define _DEBUG_SYSERR                       FALSE /*TRUE*/                      /*!< IO[A] */
#define _DEBUG_TRIAXIS                      FALSE /*TRUE*/                      /*!< IO[A:B] Triaxis debugging */
#define _DEBUG_UART                         FALSE /*TRUE*/                      /*!< IO[A] UART */
#define _DEBUG_UART_ACTUATOR                FALSE /*TRUE*/                      /*!< IO[B:A] UART Actuator */
#define _DEBUG_UART_QUAD_CAR                FALSE /*TRUE*/                      /*!< IO[B:A] UART Quadruple-actuator Car */
#define _DEBUG_UART_SCOPE                   FALSE /*TRUE*/                      /*!< IO[B:A] UART Scope */
#define _DEBUG_UDS_WRITE                    FALSE /*TRUE*/                      /*!< IO[B]; FALSE: Nothing; TRUE: Set IO[B] high during UDS Write */
#define _DEBUG_WDACK                        FALSE /*TRUE*/                      /*!< IO[A]; FALSE: Nothing; TRUE: During WD Acknowledge */
#define _DEBUG_WIND_MILL                    FALSE /*TRUE*/                      /*!< IO[B:A] Wind-milling debugging*/

/* Debug: IC Test */
#define _SUPPORT_CHIP_TEST                  FALSE /*TRUE*/                      /*!< FALSE: No IC test functions; TRUE: IC test functions */
/* ADC Test */
#define _DEBUG_ADC                          FALSE /*TRUE*/                      /*!< FALSE: Nothing; TRUE: Sample channel 16x and return via LIN PID Diagnostics */
/* Select one of the ADC test below */
#define _DEBUG_ADC_HV_IO                    FALSE /*TRUE*/                      /*!< ADC High Voltage I/O test */
#define _DEBUG_ADC_LV_IOx                   FALSE /*TRUE*/                      /*!< ADC Low Voltage I/O test */
#define _DEBUG_ADC_VDDA_VDDD                FALSE /*TRUE*/                      /*!< ADC VDDA and VDDD test */
#define _DEBUG_ADC_VDDAF                    FALSE /*TRUE*/                      /*!< ADC VDDAF test */
#define _DEBUG_ADC_VAUX                     FALSE /*TRUE*/                      /*!< ADC VAUX test */
#define _DEBUG_ADC_VBGD                     FALSE /*TRUE*/                      /*!< ADC VBGD test */
#define _DEBUG_ADC_VPHx                     FALSE /*TRUE*/                      /*!< ADC Phase-voltage test */
#define _DEBUG_ADC_VS                       FALSE /*TRUE*/                      /*!< ADC Chip supply VS test */
#define _DEBUG_ADC_VSM                      FALSE /*TRUE*/                      /*!< ADC Motor driver supply VSM test */
#define _DEBUG_ADC_VSMF                     FALSE /*TRUE*/                      /*!< ADC Motor driver supply VSM Filtered test */
#define _DEBUG_ADC_LIN                      FALSE /*TRUE*/                      /*!< ADC LIN (IN) test */
#define _DEBUG_ADC_LINAA_CM                 FALSE /*TRUE*/                      /*!< ADC LIN-AA Common-Mode LIN voltage test */
#define _DEBUG_ADC_LINAA_DM                 FALSE /*TRUE*/                      /*!< ADC LIN-AA Differential-Mode LIN current test */
#define _DEBUG_ADC_MCUR                     FALSE /*TRUE*/                      /*!< ADC Motor driver Current test */
#define _DEBUG_ADC_MCURF                    FALSE /*TRUE*/                      /*!< ADC Motor driver Current test */
#define _DEBUG_ADC_SAMPLEnHOLD              FALSE /*TRUE*/                      /*!< ADC Sample & Hold test */
/* Watch dog test */
#define _DEBUG_AWD                          FALSE /*TRUE*/                      /*!< IO[B:A]; FALSE: Nothing: TRUE: Test AWD (period/reset) */
#define _DEBUG_IWD                          FALSE /*TRUE*/                      /*!< IO[B:A]; FALSE: Nothing: TRUE: Test IWD (period/reset) */
/* CPU Performance test */
#define _DEBUG_CPU_CLOCK                    FALSE /*TRUE*/                      /*!< CPU-clock DIV 4 on IO[A] */
#define _DEBUG_IO_CPU_PERF                  FALSE /*TRUE*/                      /*!< IO[B:A]; FALSE: Nothing; TRUE: Toggle IO[1:0] 4-times */
#define _DEBUG_DELAY                        FALSE /*TRUE*/                      /*!< DELAY_US and MSEC_DELAY() test */
#define _DEBUG_RESET                        FALSE /*TRUE*/                      /*!< Reset test */
#define _DEBUG_STIMER                       FALSE /*TRUE*/                      /*!< Simple Timer (STimer) test */
/* IO test */
#define _DEBUG_IO_3_0                       FALSE /*TRUE*/                      /*!< Test IO[3:0] (MLX81330 only) */
#define _DEBUG_IO_7_0                       FALSE /*TRUE*/                      /*!< Test IO[7:0] */
#define _DEBUG_IO_8_11                      FALSE /*TRUE*/                      /*!< Test IO[11:8] */
/* _DEBUG_IO should be set to TRUE; Other (above) _DEBUG must be FALSE */
#define _DEBUG_IO_ISR                       FALSE /*TRUE*/                      /*!< FALSE: Polling/Pend; TRUE: IRQ (Check fw_Vectors.S for _DEBUG_IO_11_0_ISR) */
#define _DEBUG_IO_EDGE                      FALSE /*TRUE*/                      /*!< FALSE: Level; FALSE: Edge */
#define _DEBUG_IO_0                         FALSE /*TRUE*/                      /*!< Out: IO[0] */
#define _DEBUG_IO_1                         FALSE /*TRUE*/                      /*!< Out: IO[1] */
#define _DEBUG_IO_2                         FALSE /*TRUE*/                      /*!< Out: IO[2] */
#define _DEBUG_IO_3                         FALSE /*TRUE*/                      /*!< Out: IO[3] */
#define _DEBUG_IO_4                         FALSE /*TRUE*/                      /*!< Out: IO[4] */
#define _DEBUG_IO_5                         FALSE /*TRUE*/                      /*!< Out: IO[5] */
#define _DEBUG_IO_6                         FALSE /*TRUE*/                      /*!< Out: IO[6] */
#define _DEBUG_IO_7                         FALSE /*TRUE*/                      /*!< Out: IO[7] */
/* PWM test */
#define _DEBUG_PWM_TO_IO                    FALSE /*TRUE*/                      /*!< IO[D:A] */
#define _DEBUG_PWM_MASTER1_TO_PHASE         FALSE /*TRUE*/                      /*!< PWM_Master to Phase */
/* Non Volatile Memory test */
#define _DEBUG_NV                           FALSE /* Enable _APP_ZWICKAU */     /*!< IO[D:A]; Uses SPI to communicate counter/error */

#define _DEBUG_MOTOR_CURRENT_FLT            FALSE /*TRUE*/                      /*!< Motor current filter debug-buffers */
#define _DEBUG_MCUR_CYCLIC                  FALSE /*TRUE*/                      /*!< FALSE: Single buffer fill; TRUE: Cyclic buffer fill (round-robin) */
#define _DEBUG_MCUR_SUB_SAMPLED             FALSE /*TRUE*/                      /*!< FALSE: Each sample; TRUE: One out of xxx samples */
#define C_SUB_SAMPLE_MASK                   0x0003U                             /*!< Sub-sample mask */
#define _DEBUG_MCUR_FOC                     FALSE /*TRUE*/                      /*!< FALSE: No FOC debugging; TRUE: FOC Motor Current debugging */
#define _DEBUG_MCUR_FOC_PID                 FALSE /*TRUE*/                      /*!< FALSE: No PID sensing; TRUE: PID Sensing */
#define _DEBUG_MCUR_SENSE                   FALSE /*TRUE*/                      /*!< FALSE: No MCur sensing; TRUE: Motor current Sensing */
#define _DEBUG_MCUR_STALL                   FALSE /*TRUE*/                      /*!< FALSE: No MotorCurrent stall debugging; TRUE: Motor current stall debugging */
#define _DEBUG_MCUR_WINDMILL                FALSE /*TRUE*/                      /*!< FALSE: No BEMF sensing; TRUE: BEMF Sensing */
#define _DEBUG_MLX90377                     FALSE /*TRUE*/                      /*!< TRUE: Calculate angler-speed/Stall "P" */
#define _DEBUG_MLX90381                     FALSE /*TRUE*/                      /*!< TRUE: Calculate angler-speed */
#define _DEBUG_IV_STALL                     FALSE /*TRUE*/                      /*!< IV-Stall detector (MMP230824-1) */
#define _DEBUG_LA_STALL                     FALSE /*TRUE*/                      /*!< IV-Stall detector (MMP240725-1) */
#define _DEBUG_HALL_LATCH                   FALSE /*TRUE*/                      /*!< TRUE: Hall-Latch info */
#define _DEBUG_BEMF_SENSE_STALL             FALSE /*TRUE*/                      /*!< TRUE: Motor BEMF Sensing */

#if (_DEBUG_IO != FALSE)
static inline void DEBUG_IO_INIT(void)
{
#if (_DEBUG_CPU_CLOCK != FALSE)
    IO_CTIMER1_TREGB = 4U;
    IO_CTIMER1_TREGA = 2U;
    IO_CTIMER1_CTRL = (C_CTIMER1_DIV_CPU | C_CTIMER1_MODE_PWM | B_CTIMER1_ENCMP); /* PWM mode */
    IO_CTIMER1_CTRL = B_CTIMER1_START;
    IO_PORT_IO_CFG0 = (C_PORT_IO_CFG0_IO0_OUT_SEL_CTIMER1                       /*!< Configure IO[0] */
#if (_CHIP_PACKAGE == QFN24) || (_CHIP_PACKAGE == QFN32)
                       | C_PORT_IO_CFG0_IO1_OUT_SEL_SOFT                        /*!< ... and IO[1] */
                       | C_PORT_IO_CFG0_IO2_OUT_SEL_SOFT                        /*!< ... and IO[2] */
                       | C_PORT_IO_CFG0_IO3_OUT_SEL_SOFT                        /*!< ... and IO[3] */
#endif /* (_CHIP_PACKAGE == QFN24) || (_CHIP_PACKAGE == QFN32) */
                       );
#if !defined (__MLX81330__) && !defined (__MLX81350__)
    IO_PORT_IO_CFG1 = (C_PORT_IO_CFG1_IO4_OUT_SEL_SOFT |                        /*!< ... and IO[4] */
                       C_PORT_IO_CFG1_IO5_OUT_SEL_SOFT);                        /*!< ... and IO[5] */
#endif /* !defined (__MLX81330__) && !defined (__MLX81350__) */
#else  /* (_DEBUG_CPU_CLOCK != FALSE) */
#if (_CHIP_PACKAGE == QFN24) || (_CHIP_PACKAGE == QFN32)
    IO_PORT_IO_CFG0 = (0
#if ((_SUPPORT_IO_DUT_SELECT_HVIO == FALSE) || (HVIO_DUT_SELECT != PIN_FUNC_IO_0)) && \
    ((_SUPPORT_UART == FALSE) || ((_SUPPORT_UART_RX_IO != PIN_FUNC_IO_0) && (_SUPPORT_UART_TX_IO != PIN_FUNC_IO_0))) && \
    ((_SUPPORT_SPI == FALSE) || ((_SUPPORT_SPI_MOSI_IO != PIN_FUNC_IO_0) && (_SUPPORT_SPI_MISO_IO != PIN_FUNC_IO_0) && (_SUPPORT_SPI_CLK != PIN_FUNC_IO_0) && (_SUPPORT_SPI_SS != PIN_FUNC_IO_0))) && \
    ((PWM_COMM == FALSE) || ((C_PWM_COMM_IN != PIN_FUNC_IO_0) && (C_PWM_COMM_OUT != PIN_FUNC_IO_0))) && \
    ((_SUPPORT_I2C == FALSE) || ((((_SUPPORT_I2C_MASTER != FALSE) && (C_I2C_MASTER_SCK_IO != PIN_FUNC_IO_0) && (C_I2C_MASTER_SDA_IO != PIN_FUNC_IO_0)) || \
                                  ((_SUPPORT_I2C_SLAVE != FALSE) && (C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_0) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_0))))) && \
    ((CAN_COMM == FALSE) || ((_SUPPORT_CAN_IRQ_IO != PIN_FUNC_IO_0) && (_SUPPORT_CAN_STDBY_IO != PIN_FUNC_IO_0) && (_SUPPORT_CAN_RST_IO != PIN_FUNC_IO_0))) && \
    ((_SUPPORT_HALL_LATCH == FALSE) || ((_SUPPORT_HLA_IO != PIN_FUNC_IO_0) && (_SUPPORT_HLB_IO != PIN_FUNC_IO_0) && (_SUPPORT_HLC_IO != PIN_FUNC_IO_0))) && \
    (((_SUPPORT_TRIAXIS_MLX90377 == FALSE) && (_SUPPORT_TRIAXIS_MLX90422 == FALSE)) || (_SUPPORT_SENSOR_SENT_IO != PIN_FUNC_IO_0)) && \
    ((_SUPPORT_TRIAXIS_MLX9038x == FALSE) || ((_SUPPORT_RESOLVER_X_IO != PIN_FUNC_IO_0) && (_SUPPORT_RESOLVER_Y_IO != PIN_FUNC_IO_0)))
                       | C_PORT_IO_CFG0_IO0_OUT_SEL_SOFT                        /* IO[0] */
#endif
#if ((_SUPPORT_UART == FALSE) || ((_SUPPORT_UART_RX_IO != PIN_FUNC_IO_1) && (_SUPPORT_UART_TX_IO != PIN_FUNC_IO_1))) && \
    ((_SUPPORT_SPI == FALSE) || ((_SUPPORT_SPI_MOSI_IO != PIN_FUNC_IO_1) && (_SUPPORT_SPI_MISO_IO != PIN_FUNC_IO_1) && (_SUPPORT_SPI_CLK != PIN_FUNC_IO_1) && (_SUPPORT_SPI_SS != PIN_FUNC_IO_1))) && \
    ((PWM_COMM == FALSE) || ((C_PWM_COMM_IN != PIN_FUNC_IO_1) && (C_PWM_COMM_OUT != PIN_FUNC_IO_1))) && \
    ((_SUPPORT_I2C == FALSE) || ((((_SUPPORT_I2C_MASTER != FALSE) && (C_I2C_MASTER_SCK_IO != PIN_FUNC_IO_1) && (C_I2C_MASTER_SDA_IO != PIN_FUNC_IO_1)) || \
                                  ((_SUPPORT_I2C_SLAVE != FALSE) && (C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_1) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_1))))) && \
    ((CAN_COMM == FALSE) || ((_SUPPORT_CAN_IRQ_IO != PIN_FUNC_IO_1) && (_SUPPORT_CAN_STDBY_IO != PIN_FUNC_IO_1) && (_SUPPORT_CAN_RST_IO != PIN_FUNC_IO_1))) && \
    ((_SUPPORT_HALL_LATCH == FALSE) || ((_SUPPORT_HLA_IO != PIN_FUNC_IO_1) && ((_SUPPORT_NR_OF_HL < 2) || (_SUPPORT_HLB_IO != PIN_FUNC_IO_1)) && ((_SUPPORT_NR_OF_HL < 3) || (_SUPPORT_HLC_IO != PIN_FUNC_IO_1)))) && \
    ((_SUPPORT_TRIAXIS_MLX90377 == FALSE) || (_SUPPORT_SENSOR_SENT_IO != PIN_FUNC_IO_1)) && \
    ((_SUPPORT_TRIAXIS_MLX9038x == FALSE) || ((_SUPPORT_RESOLVER_X_IO != PIN_FUNC_IO_1) && (_SUPPORT_RESOLVER_Y_IO != PIN_FUNC_IO_1)))
                       | C_PORT_IO_CFG0_IO1_OUT_SEL_SOFT                        /* IO[1] */
#endif
#if ((_SUPPORT_UART == FALSE) || ((_SUPPORT_UART_RX_IO != PIN_FUNC_IO_2) && (_SUPPORT_UART_TX_IO != PIN_FUNC_IO_2))) && \
    ((_SUPPORT_SPI == FALSE)  || ((_SUPPORT_SPI_MOSI_IO != PIN_FUNC_IO_2) && (_SUPPORT_SPI_MISO_IO != PIN_FUNC_IO_2) && (_SUPPORT_SPI_CLK != PIN_FUNC_IO_2) && (_SUPPORT_SPI_SS != PIN_FUNC_IO_2))) && \
    ((PWM_COMM == FALSE) || ((C_PWM_COMM_IN != PIN_FUNC_IO_2) && (C_PWM_COMM_OUT != PIN_FUNC_IO_2))) && \
    ((_SUPPORT_I2C == FALSE) || ((((_SUPPORT_I2C_MASTER != FALSE) && (C_I2C_MASTER_SCK_IO != PIN_FUNC_IO_2) && (C_I2C_MASTER_SDA_IO != PIN_FUNC_IO_2)) || \
                                  ((_SUPPORT_I2C_SLAVE != FALSE) && (C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_2) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_2))))) && \
    ((CAN_COMM == FALSE) || ((_SUPPORT_CAN_IRQ_IO != PIN_FUNC_IO_2) && (_SUPPORT_CAN_STDBY_IO != PIN_FUNC_IO_2) && (_SUPPORT_CAN_RST_IO != PIN_FUNC_IO_2))) && \
    ((_SUPPORT_HALL_LATCH == FALSE) || ((_SUPPORT_HLA_IO != PIN_FUNC_IO_2) && ((_SUPPORT_NR_OF_HL < 2) || (_SUPPORT_HLB_IO != PIN_FUNC_IO_2)) && ((_SUPPORT_NR_OF_HL < 3) || (_SUPPORT_HLC_IO != PIN_FUNC_IO_2)))) && \
    ((_SUPPORT_TRIAXIS_MLX90377 == FALSE) || (_SUPPORT_SENSOR_SENT_IO != PIN_FUNC_IO_2)) && \
    ((_SUPPORT_TRIAXIS_MLX9038x == FALSE) || ((_SUPPORT_RESOLVER_X_IO != PIN_FUNC_IO_2) && (_SUPPORT_RESOLVER_Y_IO != PIN_FUNC_IO_2)))
                       | C_PORT_IO_CFG0_IO2_OUT_SEL_SOFT                        /* IO[2] */
#endif
#if ((_SUPPORT_UART == FALSE) || ((_SUPPORT_UART_RX_IO != PIN_FUNC_IO_3) && (_SUPPORT_UART_TX_IO != PIN_FUNC_IO_3))) && \
    ((_SUPPORT_SPI == FALSE)  || ((_SUPPORT_SPI_MOSI_IO != PIN_FUNC_IO_3) && (_SUPPORT_SPI_MISO_IO != PIN_FUNC_IO_3) && (_SUPPORT_SPI_CLK != PIN_FUNC_IO_3) && (_SUPPORT_SPI_SS != PIN_FUNC_IO_3))) && \
    ((PWM_COMM == FALSE) || ((C_PWM_COMM_IN != PIN_FUNC_IO_3) && (C_PWM_COMM_OUT != PIN_FUNC_IO_3))) && \
    ((_SUPPORT_I2C == FALSE) || ((((_SUPPORT_I2C_MASTER != FALSE) && (C_I2C_MASTER_SCK_IO != PIN_FUNC_IO_3) && (C_I2C_MASTER_SDA_IO != PIN_FUNC_IO_3)) || \
                                  ((_SUPPORT_I2C_SLAVE != FALSE) && (C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_3) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_3))))) && \
    ((CAN_COMM == FALSE) || ((_SUPPORT_CAN_IRQ_IO != PIN_FUNC_IO_3) && (_SUPPORT_CAN_STDBY_IO != PIN_FUNC_IO_3) && (_SUPPORT_CAN_RST_IO != PIN_FUNC_IO_3))) && \
    ((_SUPPORT_HALL_LATCH == FALSE) || ((_SUPPORT_HLA_IO != PIN_FUNC_IO_3) && ((_SUPPORT_NR_OF_HL < 2) || (_SUPPORT_HLB_IO != PIN_FUNC_IO_3)) && ((_SUPPORT_NR_OF_HL < 3) || (_SUPPORT_HLC_IO != PIN_FUNC_IO_3)))) && \
    ((_SUPPORT_TRIAXIS_MLX90377 == FALSE) || (_SUPPORT_SENSOR_SENT_IO != PIN_FUNC_IO_3)) && \
    ((_SUPPORT_TRIAXIS_MLX9038x == FALSE) || ((_SUPPORT_RESOLVER_X_IO != PIN_FUNC_IO_3) && (_SUPPORT_RESOLVER_Y_IO != PIN_FUNC_IO_3)))
                       | C_PORT_IO_CFG0_IO3_OUT_SEL_SOFT                        /* IO[3] */
#endif
                      );
#if !defined (__MLX81330__) && !defined (__MLX81350__)
    IO_PORT_IO_CFG1 = (0
#if ((_SUPPORT_UART == FALSE) || ((_SUPPORT_UART_RX_IO != PIN_FUNC_IO_4) && (_SUPPORT_UART_TX_IO != PIN_FUNC_IO_4))) && \
    ((_SUPPORT_SPI == FALSE) || ((_SUPPORT_SPI_MOSI_IO != PIN_FUNC_IO_4) && (_SUPPORT_SPI_MISO_IO != PIN_FUNC_IO_4) && (_SUPPORT_SPI_CLK != PIN_FUNC_IO_4) && (_SUPPORT_SPI_SS != PIN_FUNC_IO_4))) && \
    ((PWM_COMM == FALSE) || ((C_PWM_COMM_IN != PIN_FUNC_IO_4) && (C_PWM_COMM_OUT != PIN_FUNC_IO_4))) && \
    ((_SUPPORT_I2C == FALSE) || ((((_SUPPORT_I2C_MASTER != FALSE) && (C_I2C_MASTER_SCK_IO != PIN_FUNC_IO_4) && (C_I2C_MASTER_SDA_IO != PIN_FUNC_IO_4)) || \
                                  ((_SUPPORT_I2C_SLAVE != FALSE) && (C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_4) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_4))))) && \
    ((CAN_COMM == FALSE) || ((_SUPPORT_CAN_IRQ_IO != PIN_FUNC_IO_4) && (_SUPPORT_CAN_STDBY_IO != PIN_FUNC_IO_4) && (_SUPPORT_CAN_RST_IO != PIN_FUNC_IO_4))) && \
    ((_SUPPORT_HALL_LATCH == FALSE) || ((_SUPPORT_HLA_IO != PIN_FUNC_IO_4) && ((_SUPPORT_NR_OF_HL < 2) || (_SUPPORT_HLB_IO != PIN_FUNC_IO_4)) && ((_SUPPORT_NR_OF_HL < 3) || (_SUPPORT_HLC_IO != PIN_FUNC_IO_4)))) && \
    ((_SUPPORT_TRIAXIS_MLX90377 == FALSE) || (_SUPPORT_SENSOR_SENT_IO != PIN_FUNC_IO_4)) && \
    ((_SUPPORT_TRIAXIS_MLX9038x == FALSE) || ((_SUPPORT_RESOLVER_X_IO != PIN_FUNC_IO_4) && (_SUPPORT_RESOLVER_Y_IO != PIN_FUNC_IO_4)))
                       | C_PORT_IO_CFG1_IO4_OUT_SEL_SOFT                        /*!< ... and IO[4] */
#endif
#if ((_SUPPORT_UART == FALSE) || ((_SUPPORT_UART_RX_IO != PIN_FUNC_IO_5) && (_SUPPORT_UART_TX_IO != PIN_FUNC_IO_5))) && \
    ((_SUPPORT_SPI == FALSE) || ((_SUPPORT_SPI_MOSI_IO != PIN_FUNC_IO_5) && (_SUPPORT_SPI_MISO_IO != PIN_FUNC_IO_5) && (_SUPPORT_SPI_CLK != PIN_FUNC_IO_5) && (_SUPPORT_SPI_SS != PIN_FUNC_IO_5))) && \
    ((PWM_COMM == FALSE) || ((C_PWM_COMM_IN != PIN_FUNC_IO_5) && (C_PWM_COMM_OUT != PIN_FUNC_IO_5))) && \
    ((_SUPPORT_I2C == FALSE) || ((((_SUPPORT_I2C_MASTER != FALSE) && (C_I2C_MASTER_SCK_IO != PIN_FUNC_IO_5) && (C_I2C_MASTER_SDA_IO != PIN_FUNC_IO_5)) || \
                                  ((_SUPPORT_I2C_SLAVE != FALSE) && (C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_5) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_5))))) && \
    ((CAN_COMM == FALSE) || ((_SUPPORT_CAN_IRQ_IO != PIN_FUNC_IO_5) && (_SUPPORT_CAN_STDBY_IO != PIN_FUNC_IO_5) && (_SUPPORT_CAN_RST_IO != PIN_FUNC_IO_5))) && \
    ((_SUPPORT_HALL_LATCH == FALSE) || ((_SUPPORT_HLA_IO != PIN_FUNC_IO_5) && ((_SUPPORT_NR_OF_HL < 2) || (_SUPPORT_HLB_IO != PIN_FUNC_IO_5)) && ((_SUPPORT_NR_OF_HL < 3) || (_SUPPORT_HLC_IO != PIN_FUNC_IO_5)))) && \
    ((_SUPPORT_TRIAXIS_MLX90377 == FALSE) || (_SUPPORT_SENSOR_SENT_IO != PIN_FUNC_IO_5)) && \
    ((_SUPPORT_TRIAXIS_MLX9038x == FALSE) || ((_SUPPORT_RESOLVER_X_IO != PIN_FUNC_IO_5) && (_SUPPORT_RESOLVER_Y_IO != PIN_FUNC_IO_5)))
                       | C_PORT_IO_CFG1_IO5_OUT_SEL_SOFT                        /*!< ... and IO[5] */
#endif
                      );
#endif /* !defined (__MLX81330__) && !defined (__MLX81350__) */
#else  /* (_CHIP_PACKAGE == QFN24) || (_CHIP_PACKAGE == QFN32) */
    IO_PORT_IO_CFG0 = C_PORT_IO_CFG0_IO0_OUT_SEL_SOFT;                          /*!< Configure IO[0] */
#endif /* (_CHIP_PACKAGE == QFN24) || (_CHIP_PACKAGE == QFN32) */
#endif /* (_DEBUG_CPU_CLOCK != FALSE) */

#if ((_SUPPORT_IO_DUT_SELECT_HVIO == FALSE) || (HVIO_DUT_SELECT != PIN_FUNC_IO_0)) && \
    ((_SUPPORT_UART == FALSE) || ((_SUPPORT_UART_RX_IO != PIN_FUNC_IO_0) && (_SUPPORT_UART_TX_IO != PIN_FUNC_IO_0))) && \
    ((_SUPPORT_SPI == FALSE) || ((_SUPPORT_SPI_MOSI_IO != PIN_FUNC_IO_0) && (_SUPPORT_SPI_MISO_IO != PIN_FUNC_IO_0) && (_SUPPORT_SPI_CLK != PIN_FUNC_IO_0) && (_SUPPORT_SPI_SS != PIN_FUNC_IO_0))) && \
    ((PWM_COMM == FALSE) || ((C_PWM_COMM_IN != PIN_FUNC_IO_0) && (C_PWM_COMM_OUT != PIN_FUNC_IO_0))) && \
    ((_SUPPORT_I2C == FALSE) || ((((_SUPPORT_I2C_MASTER != FALSE) && (C_I2C_MASTER_SCK_IO != PIN_FUNC_IO_0) && (C_I2C_MASTER_SDA_IO != PIN_FUNC_IO_0)) || \
                                  ((_SUPPORT_I2C_SLAVE != FALSE) && (C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_0) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_0))))) && \
    ((CAN_COMM == FALSE) || ((_SUPPORT_CAN_IRQ_IO != PIN_FUNC_IO_0) && (_SUPPORT_CAN_STDBY_IO != PIN_FUNC_IO_0) && (_SUPPORT_CAN_RST_IO != PIN_FUNC_IO_0))) && \
    ((_SUPPORT_HALL_LATCH == FALSE) || ((_SUPPORT_HLA_IO != PIN_FUNC_IO_0) && (_SUPPORT_HLB_IO != PIN_FUNC_IO_0) && (_SUPPORT_HLC_IO != PIN_FUNC_IO_0))) && \
    (((_SUPPORT_TRIAXIS_MLX90377 == FALSE) && (_SUPPORT_TRIAXIS_MLX90422 == FALSE)) || (_SUPPORT_SENSOR_SENT_IO != PIN_FUNC_IO_0)) && \
    ((_SUPPORT_TRIAXIS_MLX9038x == FALSE) || ((_SUPPORT_RESOLVER_X_IO != PIN_FUNC_IO_0) && (_SUPPORT_RESOLVER_Y_IO != PIN_FUNC_IO_0)))
    IO_PORT_IO_OUT_EN |= B_PORT_IO_OUT_EN_IO0_LV_ENABLE;                        /*!< Enable LV of IO[0] when used as Output */
#endif /* (_SUPPORT_IO_DUT_SELECT_HVIO == FALSE) || (HVIO_DUT_SELECT != PIN_FUNC_IO_0) */

#if (_CHIP_PACKAGE == QFN24) || (_CHIP_PACKAGE == QFN32)
    IO_PORT_IO_ENABLE = (0
#if ((_SUPPORT_IO_DUT_SELECT_HVIO == FALSE) || (HVIO_DUT_SELECT != PIN_FUNC_IO_0)) && \
    ((_SUPPORT_UART == FALSE) || ((_SUPPORT_UART_RX_IO != PIN_FUNC_IO_0) && (_SUPPORT_UART_TX_IO != PIN_FUNC_IO_0))) && \
    ((_SUPPORT_SPI == FALSE) || ((_SUPPORT_SPI_MOSI_IO != PIN_FUNC_IO_0) && (_SUPPORT_SPI_MISO_IO != PIN_FUNC_IO_0) && (_SUPPORT_SPI_CLK != PIN_FUNC_IO_0) && (_SUPPORT_SPI_SS != PIN_FUNC_IO_0))) && \
    ((PWM_COMM == FALSE) || ((C_PWM_COMM_IN != PIN_FUNC_IO_0) && (C_PWM_COMM_OUT != PIN_FUNC_IO_0))) && \
    ((_SUPPORT_I2C == FALSE) || ((((_SUPPORT_I2C_MASTER != FALSE) && (C_I2C_MASTER_SCK_IO != PIN_FUNC_IO_0) && (C_I2C_MASTER_SDA_IO != PIN_FUNC_IO_0)) || \
                                  ((_SUPPORT_I2C_SLAVE != FALSE) && (C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_0) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_0))))) && \
    ((CAN_COMM == FALSE) || ((_SUPPORT_CAN_IRQ_IO != PIN_FUNC_IO_0) && (_SUPPORT_CAN_STDBY_IO != PIN_FUNC_IO_0) && (_SUPPORT_CAN_RST_IO != PIN_FUNC_IO_0))) && \
    ((_SUPPORT_HALL_LATCH == FALSE) || ((_SUPPORT_HLA_IO != PIN_FUNC_IO_0) && (_SUPPORT_HLB_IO != PIN_FUNC_IO_0) && (_SUPPORT_HLC_IO != PIN_FUNC_IO_0))) && \
    (((_SUPPORT_TRIAXIS_MLX90377 == FALSE) && (_SUPPORT_TRIAXIS_MLX90422 == FALSE)) || (_SUPPORT_SENSOR_SENT_IO != PIN_FUNC_IO_0)) && \
    ((_SUPPORT_TRIAXIS_MLX9038x == FALSE) || ((_SUPPORT_RESOLVER_X_IO != PIN_FUNC_IO_0) && (_SUPPORT_RESOLVER_Y_IO != PIN_FUNC_IO_0)))
                       | B_PORT_IO_ENABLE_IO_ENABLE_0                           /* IO[0] */
#endif
#if ((_SUPPORT_UART == FALSE) || ((_SUPPORT_UART_RX_IO != PIN_FUNC_IO_1) && (_SUPPORT_UART_TX_IO != PIN_FUNC_IO_1))) && \
    ((_SUPPORT_SPI == FALSE) || ((_SUPPORT_SPI_MOSI_IO != PIN_FUNC_IO_1) && (_SUPPORT_SPI_MISO_IO != PIN_FUNC_IO_1) && (_SUPPORT_SPI_CLK != PIN_FUNC_IO_1) && (_SUPPORT_SPI_SS != PIN_FUNC_IO_1))) && \
    ((PWM_COMM == FALSE) || ((C_PWM_COMM_IN != PIN_FUNC_IO_1) && (C_PWM_COMM_OUT != PIN_FUNC_IO_1))) && \
    ((_SUPPORT_I2C == FALSE) || ((((_SUPPORT_I2C_MASTER != FALSE) && (C_I2C_MASTER_SCK_IO != PIN_FUNC_IO_1) && (C_I2C_MASTER_SDA_IO != PIN_FUNC_IO_1)) || \
                                  ((_SUPPORT_I2C_SLAVE != FALSE) && (C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_1) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_1))))) && \
    ((CAN_COMM == FALSE) || ((_SUPPORT_CAN_IRQ_IO != PIN_FUNC_IO_1) && (_SUPPORT_CAN_STDBY_IO != PIN_FUNC_IO_1) && (_SUPPORT_CAN_RST_IO != PIN_FUNC_IO_1))) && \
    ((_SUPPORT_HALL_LATCH == FALSE) || ((_SUPPORT_HLA_IO != PIN_FUNC_IO_1) && ((_SUPPORT_NR_OF_HL < 2) || (_SUPPORT_HLB_IO != PIN_FUNC_IO_1)) && ((_SUPPORT_NR_OF_HL < 3) || (_SUPPORT_HLC_IO != PIN_FUNC_IO_1)))) && \
    ((_SUPPORT_TRIAXIS_MLX90377 == FALSE) || (_SUPPORT_SENSOR_SENT_IO != PIN_FUNC_IO_1)) && \
    ((_SUPPORT_TRIAXIS_MLX9038x == FALSE) || ((_SUPPORT_RESOLVER_X_IO != PIN_FUNC_IO_1) && (_SUPPORT_RESOLVER_Y_IO != PIN_FUNC_IO_1)))
                       | B_PORT_IO_ENABLE_IO_ENABLE_1                           /* IO[1] */
#endif
#if ((_SUPPORT_UART == FALSE) || ((_SUPPORT_UART_RX_IO != PIN_FUNC_IO_2) && (_SUPPORT_UART_TX_IO != PIN_FUNC_IO_2))) && \
    ((_SUPPORT_SPI == FALSE)  || ((_SUPPORT_SPI_MOSI_IO != PIN_FUNC_IO_2) && (_SUPPORT_SPI_MISO_IO != PIN_FUNC_IO_2) && (_SUPPORT_SPI_CLK != PIN_FUNC_IO_2) && (_SUPPORT_SPI_SS != PIN_FUNC_IO_2))) && \
    ((PWM_COMM == FALSE) || ((C_PWM_COMM_IN != PIN_FUNC_IO_2) && (C_PWM_COMM_OUT != PIN_FUNC_IO_2))) && \
    ((_SUPPORT_I2C == FALSE) || ((((_SUPPORT_I2C_MASTER != FALSE) && (C_I2C_MASTER_SCK_IO != PIN_FUNC_IO_2) && (C_I2C_MASTER_SDA_IO != PIN_FUNC_IO_2)) || \
                                  ((_SUPPORT_I2C_SLAVE != FALSE) && (C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_2) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_2))))) && \
    ((CAN_COMM == FALSE) || ((_SUPPORT_CAN_IRQ_IO != PIN_FUNC_IO_2) && (_SUPPORT_CAN_STDBY_IO != PIN_FUNC_IO_2) && (_SUPPORT_CAN_RST_IO != PIN_FUNC_IO_2))) && \
    ((_SUPPORT_HALL_LATCH == FALSE) || ((_SUPPORT_HLA_IO != PIN_FUNC_IO_2) && ((_SUPPORT_NR_OF_HL < 2) || (_SUPPORT_HLB_IO != PIN_FUNC_IO_2)) && ((_SUPPORT_NR_OF_HL < 3) || (_SUPPORT_HLC_IO != PIN_FUNC_IO_2)))) && \
    ((_SUPPORT_TRIAXIS_MLX90377 == FALSE) || (_SUPPORT_SENSOR_SENT_IO != PIN_FUNC_IO_2)) && \
    ((_SUPPORT_TRIAXIS_MLX9038x == FALSE) || ((_SUPPORT_RESOLVER_X_IO != PIN_FUNC_IO_2) && (_SUPPORT_RESOLVER_Y_IO != PIN_FUNC_IO_2)))
                       | B_PORT_IO_ENABLE_IO_ENABLE_2                           /* IO[2] */
#endif
#if ((_SUPPORT_UART == FALSE) || ((_SUPPORT_UART_RX_IO != PIN_FUNC_IO_3) && (_SUPPORT_UART_TX_IO != PIN_FUNC_IO_3))) && \
    ((_SUPPORT_SPI == FALSE)  || ((_SUPPORT_SPI_MOSI_IO != PIN_FUNC_IO_3) && (_SUPPORT_SPI_MISO_IO != PIN_FUNC_IO_3) && (_SUPPORT_SPI_CLK != PIN_FUNC_IO_3) && (_SUPPORT_SPI_SS != PIN_FUNC_IO_3))) && \
    ((PWM_COMM == FALSE) || ((C_PWM_COMM_IN != PIN_FUNC_IO_3) && (C_PWM_COMM_OUT != PIN_FUNC_IO_3))) && \
    ((_SUPPORT_I2C == FALSE) || ((((_SUPPORT_I2C_MASTER != FALSE) && (C_I2C_MASTER_SCK_IO != PIN_FUNC_IO_3) && (C_I2C_MASTER_SDA_IO != PIN_FUNC_IO_3)) || \
                                  ((_SUPPORT_I2C_SLAVE != FALSE) && (C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_3) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_3))))) && \
    ((CAN_COMM == FALSE) || ((_SUPPORT_CAN_IRQ_IO != PIN_FUNC_IO_3) && (_SUPPORT_CAN_STDBY_IO != PIN_FUNC_IO_3) && (_SUPPORT_CAN_RST_IO != PIN_FUNC_IO_3))) && \
    ((_SUPPORT_HALL_LATCH == FALSE) || ((_SUPPORT_HLA_IO != PIN_FUNC_IO_3) && ((_SUPPORT_NR_OF_HL < 2) || (_SUPPORT_HLB_IO != PIN_FUNC_IO_3)) && ((_SUPPORT_NR_OF_HL < 3) || (_SUPPORT_HLC_IO != PIN_FUNC_IO_3)))) && \
    ((_SUPPORT_TRIAXIS_MLX90377 == FALSE) || (_SUPPORT_SENSOR_SENT_IO != PIN_FUNC_IO_3)) && \
    ((_SUPPORT_TRIAXIS_MLX9038x == FALSE) || ((_SUPPORT_RESOLVER_X_IO != PIN_FUNC_IO_3) && (_SUPPORT_RESOLVER_Y_IO != PIN_FUNC_IO_3)))
                       | B_PORT_IO_ENABLE_IO_ENABLE_3                           /* IO[3] */
#endif
#if !defined (__MLX81330__) && !defined (__MLX81350__)
#if ((_SUPPORT_UART == FALSE) || ((_SUPPORT_UART_RX_IO != PIN_FUNC_IO_4) && (_SUPPORT_UART_TX_IO != PIN_FUNC_IO_4))) && \
    ((_SUPPORT_SPI == FALSE) || ((_SUPPORT_SPI_MOSI_IO != PIN_FUNC_IO_4) && (_SUPPORT_SPI_MISO_IO != PIN_FUNC_IO_4) && (_SUPPORT_SPI_CLK != PIN_FUNC_IO_4) && (_SUPPORT_SPI_SS != PIN_FUNC_IO_4))) && \
    ((PWM_COMM == FALSE) || ((C_PWM_COMM_IN != PIN_FUNC_IO_4) && (C_PWM_COMM_OUT != PIN_FUNC_IO_4))) && \
    ((_SUPPORT_I2C == FALSE) || ((((_SUPPORT_I2C_MASTER != FALSE) && (C_I2C_MASTER_SCK_IO != PIN_FUNC_IO_4) && (C_I2C_MASTER_SDA_IO != PIN_FUNC_IO_4)) || \
                                  ((_SUPPORT_I2C_SLAVE != FALSE) && (C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_4) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_4))))) && \
    ((CAN_COMM == FALSE) || ((_SUPPORT_CAN_IRQ_IO != PIN_FUNC_IO_4) && (_SUPPORT_CAN_STDBY_IO != PIN_FUNC_IO_4) && (_SUPPORT_CAN_RST_IO != PIN_FUNC_IO_4))) && \
	((_SUPPORT_HALL_LATCH == FALSE) || ((_SUPPORT_HLA_IO != PIN_FUNC_IO_4) && ((_SUPPORT_NR_OF_HL < 2) || (_SUPPORT_HLB_IO != PIN_FUNC_IO_4)) && ((_SUPPORT_NR_OF_HL < 3) || (_SUPPORT_HLC_IO != PIN_FUNC_IO_4)))) && \
    ((_SUPPORT_TRIAXIS_MLX90377 == FALSE) || (_SUPPORT_SENSOR_SENT_IO != PIN_FUNC_IO_4)) && \
    ((_SUPPORT_TRIAXIS_MLX9038x == FALSE) || ((_SUPPORT_RESOLVER_X_IO != PIN_FUNC_IO_4) && (_SUPPORT_RESOLVER_Y_IO != PIN_FUNC_IO_4)))
                       | B_PORT_IO_ENABLE_IO_ENABLE_4                           /* IO[4] */
#endif
#if ((_SUPPORT_UART == FALSE) || ((_SUPPORT_UART_RX_IO != PIN_FUNC_IO_5) && (_SUPPORT_UART_TX_IO != PIN_FUNC_IO_5))) && \
    ((_SUPPORT_SPI == FALSE) || ((_SUPPORT_SPI_MOSI_IO != PIN_FUNC_IO_5) && (_SUPPORT_SPI_MISO_IO != PIN_FUNC_IO_5) && (_SUPPORT_SPI_CLK != PIN_FUNC_IO_5) && (_SUPPORT_SPI_SS != PIN_FUNC_IO_5))) && \
    ((PWM_COMM == FALSE) || ((C_PWM_COMM_IN != PIN_FUNC_IO_5) && (C_PWM_COMM_OUT != PIN_FUNC_IO_5))) && \
    ((_SUPPORT_I2C == FALSE) || ((((_SUPPORT_I2C_MASTER != FALSE) && (C_I2C_MASTER_SCK_IO != PIN_FUNC_IO_5) && (C_I2C_MASTER_SDA_IO != PIN_FUNC_IO_5)) || \
                                  ((_SUPPORT_I2C_SLAVE != FALSE) && (C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_5) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_5))))) && \
    ((CAN_COMM == FALSE) || ((_SUPPORT_CAN_IRQ_IO != PIN_FUNC_IO_5) && (_SUPPORT_CAN_STDBY_IO != PIN_FUNC_IO_5) && (_SUPPORT_CAN_RST_IO != PIN_FUNC_IO_5))) && \
	((_SUPPORT_HALL_LATCH == FALSE) || ((_SUPPORT_HLA_IO != PIN_FUNC_IO_5) && ((_SUPPORT_NR_OF_HL < 2) || (_SUPPORT_HLB_IO != PIN_FUNC_IO_5)) && ((_SUPPORT_NR_OF_HL < 3) || (_SUPPORT_HLC_IO != PIN_FUNC_IO_5)))) && \
    ((_SUPPORT_TRIAXIS_MLX90377 == FALSE) || (_SUPPORT_SENSOR_SENT_IO != PIN_FUNC_IO_5)) && \
    ((_SUPPORT_TRIAXIS_MLX9038x == FALSE) || ((_SUPPORT_RESOLVER_X_IO != PIN_FUNC_IO_5) && (_SUPPORT_RESOLVER_Y_IO != PIN_FUNC_IO_5)))
                       | B_PORT_IO_ENABLE_IO_ENABLE_5                           /* IO[5] */
#endif
#endif /* !defined (__MLX81330__) && !defined (__MLX81350__) */
                      );
#endif /* (_CHIP_PACKAGE == QFN24) || (_CHIP_PACKAGE == C_CHIP_QFN32) */
} /* End of DEBUG_IO_INIT() */

#if (_CHIP_PACKAGE == QFN24) || (_CHIP_PACKAGE == QFN32)
#if ((_SUPPORT_IO_DUT_SELECT_HVIO == FALSE) || (HVIO_DUT_SELECT != PIN_FUNC_IO_0)) && \
    ((_SUPPORT_UART == FALSE) || ((_SUPPORT_UART_RX_IO != PIN_FUNC_IO_0) && (_SUPPORT_UART_TX_IO != PIN_FUNC_IO_0))) && \
    ((_SUPPORT_SPI == FALSE) || ((_SUPPORT_SPI_MOSI_IO != PIN_FUNC_IO_0) && (_SUPPORT_SPI_MISO_IO != PIN_FUNC_IO_0) && (_SUPPORT_SPI_CLK != PIN_FUNC_IO_0) && (_SUPPORT_SPI_SS != PIN_FUNC_IO_0))) && \
    ((PWM_COMM == FALSE) || ((C_PWM_COMM_IN != PIN_FUNC_IO_0) && (C_PWM_COMM_OUT != PIN_FUNC_IO_0))) && \
    ((_SUPPORT_I2C == FALSE) || ((((_SUPPORT_I2C_MASTER != FALSE) && (C_I2C_MASTER_SCK_IO != PIN_FUNC_IO_0) && (C_I2C_MASTER_SDA_IO != PIN_FUNC_IO_0)) || \
                                  ((_SUPPORT_I2C_SLAVE != FALSE) && (C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_0) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_0))))) && \
    ((CAN_COMM == FALSE) || ((_SUPPORT_CAN_IRQ_IO != PIN_FUNC_IO_0) && (_SUPPORT_CAN_STDBY_IO != PIN_FUNC_IO_0) && (_SUPPORT_CAN_RST_IO != PIN_FUNC_IO_0))) && \
    ((_SUPPORT_HALL_LATCH == FALSE) || ((_SUPPORT_HLA_IO != PIN_FUNC_IO_0) && (_SUPPORT_HLB_IO != PIN_FUNC_IO_0) && (_SUPPORT_HLC_IO != PIN_FUNC_IO_0))) && \
    (((_SUPPORT_TRIAXIS_MLX90377 == FALSE) && (_SUPPORT_TRIAXIS_MLX90422 == FALSE)) || (_SUPPORT_SENSOR_SENT_IO != PIN_FUNC_IO_0)) && \
    ((_SUPPORT_TRIAXIS_MLX9038x == FALSE) || ((_SUPPORT_RESOLVER_X_IO != PIN_FUNC_IO_0) && (_SUPPORT_RESOLVER_Y_IO != PIN_FUNC_IO_0)))
#define DEBUG_CLR_IO_E()    {IO_PORT_IO_OUT_SOFT &= ~C_PORT_IO_OUT_SOFT_IO0_OUT; }  /*!< Clear  IO[0] */
#define DEBUG_SET_IO_E()    {IO_PORT_IO_OUT_SOFT |= C_PORT_IO_OUT_SOFT_IO0_OUT; }   /*!< Set    IO[0] */
#define DEBUG_TOG_IO_E()    {IO_PORT_IO_OUT_SOFT ^= C_PORT_IO_OUT_SOFT_IO0_OUT; }   /*!< Toggle IO[0] */
#endif
#if ((_SUPPORT_UART == FALSE) || ((_SUPPORT_UART_RX_IO != PIN_FUNC_IO_1) && (_SUPPORT_UART_TX_IO != PIN_FUNC_IO_1))) && \
    ((_SUPPORT_SPI == FALSE) || ((_SUPPORT_SPI_MOSI_IO != PIN_FUNC_IO_1) && (_SUPPORT_SPI_MISO_IO != PIN_FUNC_IO_1) && (_SUPPORT_SPI_CLK != PIN_FUNC_IO_1) && (_SUPPORT_SPI_SS != PIN_FUNC_IO_1))) && \
    ((PWM_COMM == FALSE) || ((C_PWM_COMM_IN != PIN_FUNC_IO_1) && (C_PWM_COMM_OUT != PIN_FUNC_IO_1))) && \
    ((_SUPPORT_I2C == FALSE) || ((((_SUPPORT_I2C_MASTER != FALSE) && (C_I2C_MASTER_SCK_IO != PIN_FUNC_IO_1) && (C_I2C_MASTER_SDA_IO != PIN_FUNC_IO_1)) || \
                                  ((_SUPPORT_I2C_SLAVE != FALSE) && (C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_1) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_1))))) && \
    ((CAN_COMM == FALSE) || ((_SUPPORT_CAN_IRQ_IO != PIN_FUNC_IO_1) && (_SUPPORT_CAN_STDBY_IO != PIN_FUNC_IO_1) && (_SUPPORT_CAN_RST_IO != PIN_FUNC_IO_1))) && \
    ((_SUPPORT_HALL_LATCH == FALSE) || ((_SUPPORT_HLA_IO != PIN_FUNC_IO_1) && ((_SUPPORT_NR_OF_HL < 2) || (_SUPPORT_HLB_IO != PIN_FUNC_IO_1)) && ((_SUPPORT_NR_OF_HL < 3) || (_SUPPORT_HLC_IO != PIN_FUNC_IO_1)))) && \
    ((_SUPPORT_TRIAXIS_MLX90377 == FALSE) || (_SUPPORT_SENSOR_SENT_IO != PIN_FUNC_IO_1)) && \
    ((_SUPPORT_TRIAXIS_MLX9038x == FALSE) || ((_SUPPORT_RESOLVER_X_IO != PIN_FUNC_IO_1) && (_SUPPORT_RESOLVER_Y_IO != PIN_FUNC_IO_1)))
#define DEBUG_CLR_IO_A()    {IO_PORT_IO_OUT_SOFT &= ~C_PORT_IO_OUT_SOFT_IO1_OUT; }  /*!< Clear  IO[1] */
#define DEBUG_SET_IO_A()    {IO_PORT_IO_OUT_SOFT |= C_PORT_IO_OUT_SOFT_IO1_OUT; }   /*!< Set    IO[1] */
#define DEBUG_TOG_IO_A()    {IO_PORT_IO_OUT_SOFT ^= C_PORT_IO_OUT_SOFT_IO1_OUT; }   /*!< Toggle IO[1] */
#endif
#if ((_SUPPORT_UART == FALSE) || ((_SUPPORT_UART_RX_IO != PIN_FUNC_IO_2) && (_SUPPORT_UART_TX_IO != PIN_FUNC_IO_2))) && \
    ((_SUPPORT_SPI == FALSE)  || ((_SUPPORT_SPI_MOSI_IO != PIN_FUNC_IO_2) && (_SUPPORT_SPI_MISO_IO != PIN_FUNC_IO_2) && (_SUPPORT_SPI_CLK != PIN_FUNC_IO_2) && (_SUPPORT_SPI_SS != PIN_FUNC_IO_2))) && \
    ((PWM_COMM == FALSE) || ((C_PWM_COMM_IN != PIN_FUNC_IO_2) && (C_PWM_COMM_OUT != PIN_FUNC_IO_2))) && \
    ((_SUPPORT_I2C == FALSE) || ((((_SUPPORT_I2C_MASTER != FALSE) && (C_I2C_MASTER_SCK_IO != PIN_FUNC_IO_2) && (C_I2C_MASTER_SDA_IO != PIN_FUNC_IO_2)) || \
                                  ((_SUPPORT_I2C_SLAVE != FALSE) && (C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_2) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_2))))) && \
    ((CAN_COMM == FALSE) || ((_SUPPORT_CAN_IRQ_IO != PIN_FUNC_IO_2) && (_SUPPORT_CAN_STDBY_IO != PIN_FUNC_IO_2) && (_SUPPORT_CAN_RST_IO != PIN_FUNC_IO_2))) && \
    ((_SUPPORT_HALL_LATCH == FALSE) || ((_SUPPORT_HLA_IO != PIN_FUNC_IO_2) && ((_SUPPORT_NR_OF_HL < 2) || (_SUPPORT_HLB_IO != PIN_FUNC_IO_2)) && ((_SUPPORT_NR_OF_HL < 3) || (_SUPPORT_HLC_IO != PIN_FUNC_IO_2)))) && \
    ((_SUPPORT_TRIAXIS_MLX90377 == FALSE) || (_SUPPORT_SENSOR_SENT_IO != PIN_FUNC_IO_2)) && \
    ((_SUPPORT_TRIAXIS_MLX9038x == FALSE) || ((_SUPPORT_RESOLVER_X_IO != PIN_FUNC_IO_2) && (_SUPPORT_RESOLVER_Y_IO != PIN_FUNC_IO_2)))
#define DEBUG_CLR_IO_B()    {IO_PORT_IO_OUT_SOFT &= ~C_PORT_IO_OUT_SOFT_IO2_OUT; }  /*!< Clear  IO[2] */
#define DEBUG_SET_IO_B()    {IO_PORT_IO_OUT_SOFT |= C_PORT_IO_OUT_SOFT_IO2_OUT; }   /*!< Set    IO[2] */
#define DEBUG_TOG_IO_B()    {IO_PORT_IO_OUT_SOFT ^= C_PORT_IO_OUT_SOFT_IO2_OUT; }   /*!< Toggle IO[2] */
#endif
#if ((_SUPPORT_UART == FALSE) || ((_SUPPORT_UART_RX_IO != PIN_FUNC_IO_3) && (_SUPPORT_UART_TX_IO != PIN_FUNC_IO_3))) && \
    ((_SUPPORT_SPI == FALSE)  || ((_SUPPORT_SPI_MOSI_IO != PIN_FUNC_IO_3) && (_SUPPORT_SPI_MISO_IO != PIN_FUNC_IO_3) && (_SUPPORT_SPI_CLK != PIN_FUNC_IO_3) && (_SUPPORT_SPI_SS != PIN_FUNC_IO_3))) && \
    ((PWM_COMM == FALSE) || ((C_PWM_COMM_IN != PIN_FUNC_IO_3) && (C_PWM_COMM_OUT != PIN_FUNC_IO_3))) && \
    ((_SUPPORT_I2C == FALSE) || ((((_SUPPORT_I2C_MASTER != FALSE) && (C_I2C_MASTER_SCK_IO != PIN_FUNC_IO_3) && (C_I2C_MASTER_SDA_IO != PIN_FUNC_IO_3)) || \
                                  ((_SUPPORT_I2C_SLAVE != FALSE) && (C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_3) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_3))))) && \
    ((CAN_COMM == FALSE) || ((_SUPPORT_CAN_IRQ_IO != PIN_FUNC_IO_3) && (_SUPPORT_CAN_STDBY_IO != PIN_FUNC_IO_3) && (_SUPPORT_CAN_RST_IO != PIN_FUNC_IO_3))) && \
    ((_SUPPORT_HALL_LATCH == FALSE) || ((_SUPPORT_HLA_IO != PIN_FUNC_IO_3) && ((_SUPPORT_NR_OF_HL < 2) || (_SUPPORT_HLB_IO != PIN_FUNC_IO_3)) && ((_SUPPORT_NR_OF_HL < 3) || (_SUPPORT_HLC_IO != PIN_FUNC_IO_3)))) && \
    ((_SUPPORT_TRIAXIS_MLX90377 == FALSE) || (_SUPPORT_SENSOR_SENT_IO != PIN_FUNC_IO_3)) && \
    ((_SUPPORT_TRIAXIS_MLX9038x == FALSE) || ((_SUPPORT_RESOLVER_X_IO != PIN_FUNC_IO_3) && (_SUPPORT_RESOLVER_Y_IO != PIN_FUNC_IO_3)))
#define DEBUG_CLR_IO_C()    {IO_PORT_IO_OUT_SOFT &= ~C_PORT_IO_OUT_SOFT_IO3_OUT; }  /*!< Clear  IO[3] */
#define DEBUG_SET_IO_C()    {IO_PORT_IO_OUT_SOFT |= C_PORT_IO_OUT_SOFT_IO3_OUT; }   /*!< Set    IO[3] */
#define DEBUG_TOG_IO_C()    {IO_PORT_IO_OUT_SOFT ^= C_PORT_IO_OUT_SOFT_IO3_OUT; }   /*!< Toggle IO[3] */
#endif
#if !defined (__MLX81330__) && !defined (__MLX81350__)
#if ((_SUPPORT_UART == FALSE) || ((_SUPPORT_UART_RX_IO != PIN_FUNC_IO_4) && (_SUPPORT_UART_TX_IO != PIN_FUNC_IO_4))) && \
    ((_SUPPORT_SPI == FALSE) || ((_SUPPORT_SPI_MOSI_IO != PIN_FUNC_IO_4) && (_SUPPORT_SPI_MISO_IO != PIN_FUNC_IO_4) && (_SUPPORT_SPI_CLK != PIN_FUNC_IO_4) && (_SUPPORT_SPI_SS != PIN_FUNC_IO_4))) && \
    ((PWM_COMM == FALSE) || ((C_PWM_COMM_IN != PIN_FUNC_IO_4) && (C_PWM_COMM_OUT != PIN_FUNC_IO_4))) && \
    ((_SUPPORT_I2C == FALSE) || ((((_SUPPORT_I2C_MASTER != FALSE) && (C_I2C_MASTER_SCK_IO != PIN_FUNC_IO_4) && (C_I2C_MASTER_SDA_IO != PIN_FUNC_IO_4)) || \
                                  ((_SUPPORT_I2C_SLAVE != FALSE) && (C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_4) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_4))))) && \
    ((CAN_COMM == FALSE) || ((_SUPPORT_CAN_IRQ_IO != PIN_FUNC_IO_4) && (_SUPPORT_CAN_STDBY_IO != PIN_FUNC_IO_4) && (_SUPPORT_CAN_RST_IO != PIN_FUNC_IO_4))) && \
    ((_SUPPORT_HALL_LATCH == FALSE) || ((_SUPPORT_HLA_IO != PIN_FUNC_IO_4) && ((_SUPPORT_NR_OF_HL < 2) || (_SUPPORT_HLB_IO != PIN_FUNC_IO_4)) && ((_SUPPORT_NR_OF_HL < 3) || (_SUPPORT_HLC_IO != PIN_FUNC_IO_4)))) && \
    ((_SUPPORT_TRIAXIS_MLX90377 == FALSE) || (_SUPPORT_SENSOR_SENT_IO != PIN_FUNC_IO_4)) && \
    ((_SUPPORT_TRIAXIS_MLX9038x == FALSE) || ((_SUPPORT_RESOLVER_X_IO != PIN_FUNC_IO_4) && (_SUPPORT_RESOLVER_Y_IO != PIN_FUNC_IO_4)))
#define DEBUG_CLR_IO_D()    {IO_PORT_IO_OUT_SOFT &= ~C_PORT_IO_OUT_SOFT_IO4_OUT; }  /*!< Clear  IO[4] */
#define DEBUG_SET_IO_D()    {IO_PORT_IO_OUT_SOFT |= C_PORT_IO_OUT_SOFT_IO4_OUT; }   /*!< Set    IO[4] */
#define DEBUG_TOG_IO_D()    {IO_PORT_IO_OUT_SOFT ^= C_PORT_IO_OUT_SOFT_IO4_OUT; }   /*!< Toggle IO[4] */
#endif
#if ((_SUPPORT_UART == FALSE) || ((_SUPPORT_UART_RX_IO != PIN_FUNC_IO_5) && (_SUPPORT_UART_TX_IO != PIN_FUNC_IO_5))) && \
    ((_SUPPORT_SPI == FALSE) || ((_SUPPORT_SPI_MOSI_IO != PIN_FUNC_IO_5) && (_SUPPORT_SPI_MISO_IO != PIN_FUNC_IO_5) && (_SUPPORT_SPI_CLK != PIN_FUNC_IO_5) && (_SUPPORT_SPI_SS != PIN_FUNC_IO_5))) && \
    ((PWM_COMM == FALSE) || ((C_PWM_COMM_IN != PIN_FUNC_IO_5) && (C_PWM_COMM_OUT != PIN_FUNC_IO_5))) && \
    ((_SUPPORT_I2C == FALSE) || ((((_SUPPORT_I2C_MASTER != FALSE) && (C_I2C_MASTER_SCK_IO != PIN_FUNC_IO_5) && (C_I2C_MASTER_SDA_IO != PIN_FUNC_IO_5)) || \
                                  ((_SUPPORT_I2C_SLAVE != FALSE) && (C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_5) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_5))))) && \
    ((CAN_COMM == FALSE) || ((_SUPPORT_CAN_IRQ_IO != PIN_FUNC_IO_5) && (_SUPPORT_CAN_STDBY_IO != PIN_FUNC_IO_5) && (_SUPPORT_CAN_RST_IO != PIN_FUNC_IO_5))) && \
    ((_SUPPORT_HALL_LATCH == FALSE) || ((_SUPPORT_HLA_IO != PIN_FUNC_IO_5) && ((_SUPPORT_NR_OF_HL < 2) || (_SUPPORT_HLB_IO != PIN_FUNC_IO_5)) && ((_SUPPORT_NR_OF_HL < 3) || (_SUPPORT_HLC_IO != PIN_FUNC_IO_5)))) && \
    ((_SUPPORT_TRIAXIS_MLX90377 == FALSE) || (_SUPPORT_SENSOR_SENT_IO != PIN_FUNC_IO_5)) && \
    ((_SUPPORT_TRIAXIS_MLX9038x == FALSE) || ((_SUPPORT_RESOLVER_X_IO != PIN_FUNC_IO_5) && (_SUPPORT_RESOLVER_Y_IO != PIN_FUNC_IO_5)))
#define DEBUG_CLR_IO_E()    {IO_PORT_IO_OUT_SOFT &= ~C_PORT_IO_OUT_SOFT_IO5_OUT; }  /*!< Clear  IO[5] */
#define DEBUG_SET_IO_E()    {IO_PORT_IO_OUT_SOFT |= C_PORT_IO_OUT_SOFT_IO5_OUT; }   /*!< Set    IO[5] */
#define DEBUG_TOG_IO_E()    {IO_PORT_IO_OUT_SOFT ^= C_PORT_IO_OUT_SOFT_IO5_OUT; }   /*!< Toggle IO[5] */
#endif
#endif /* !defined (__MLX81330__) && !defined (__MLX81350__) */
#else  /* (_CHIP_PACKAGE == QFN24) || (_CHIP_PACKAGE == QFN32) */
#define DEBUG_CLR_IO_A()    {IO_PORT_IO_OUT_SOFT &= ~C_PORT_IO_OUT_SOFT_IO0_OUT; }  /*!< Clear  IO[0] */
#define DEBUG_SET_IO_A()    {IO_PORT_IO_OUT_SOFT |= C_PORT_IO_OUT_SOFT_IO0_OUT; }   /*!< Set    IO[0] */
#define DEBUG_TOG_IO_A()    {IO_PORT_IO_OUT_SOFT ^= C_PORT_IO_OUT_SOFT_IO0_OUT; }   /*!< Toggle IO[0] */
#endif /* (_CHIP_PACKAGE == QFN24) || (_CHIP_PACKAGE == QFN32) */
#endif /* (_DEBUG_IO != FALSE) */

#endif /* _APP_BUILD_H_ */

/* EOF */

