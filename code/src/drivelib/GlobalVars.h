/*!*************************************************************************** *
 * \file        GlobalVars.h
 * \brief       MLX8133x Global variables application
 *
 * \note        project MLX8133x
 *
 * \author      Marcel Braat
 *
 * \date        2017-08-15
 *
 * \version     2.0
 *
 * A list of functions:
 *  - Global Functions:
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
 * *************************************************************************** */

#ifndef DRIVE_LIB_GLOBAL_VARS_H
#define DRIVE_LIB_GLOBAL_VARS_H

/*!*************************************************************************** */
/*                           INCLUDES                                          */
/* *************************************************************************** */
#include "AppBuild.h"                                                           /* Application Build */

/*!*************************************************************************** */
/*                           DEFINITIONS                                       */
/* *************************************************************************** */
#define PWM_PRESCALER_M         1U                                              /*!< Define the PWM timer clock frequency */
#define PWM_PRESCALER_N         0U                                              /*!< as F = Fpll / ( Mx2^N ) */
#define PWM_PRESCALER           (((PWM_PRESCALER_M - 1U) << 4) + PWM_PRESCALER_N)   /*!< Prescaler value */
#if (PLL_FREQ <= (16000 * 1000UL))
#define PWM_TIMER_CLK           ((2UL * PLL_FREQ) / ((PWM_PRESCALER_M * 1U) << PWM_PRESCALER_N))  /*!< Counter frequency */
#else  /* (PLL_FREQ < (16000 * 1000UL)) */
#define PWM_TIMER_CLK           (PLL_FREQ / ((PWM_PRESCALER_M * 1U) << PWM_PRESCALER_N))  /*!< Counter frequency */
#endif /* (PLL_FREQ < (16000 * 1000UL)) */
#define PWM_REG_PERIOD          (PWM_TIMER_CLK / PWM_FREQ)                      /*!< Value of the period register; Fpwm = Fcnt/(PWM_REG_PERIOD+1) ==> 24KHz */
#define PWM_SCALE_OFFSET        ((uint16_t)((PWM_REG_PERIOD + 1U) / 2U))        /*!< Offset to have the PWM wave always in the positive area */
#define PWM_OFFSET_25           (uint16_t)((PWM_REG_PERIOD + 2U) / 4U)          /*!< 25% of the PWM-period */
#define PWM_OFFSET_75           (uint16_t)(PWM_REG_PERIOD - PWM_OFFSET_25)      /*!< 75% of the PWM-period */
#define TIMER_PRESCALER         16U                                             /*!< Timer divider is 1, 16 or 256; Minimum speed: >= 134rpm @ 28MHz/2PP, 89rpm @ 28MHz/3PP, 67rpm @ 28MHz/4PP, 54rpm @ 28MHz/5PP */
#define TIMER_CLOCK             ((uint32_t)(PLL_FREQ / TIMER_PRESCALER))        /*!< CTimer clock */
#if (TIMER_PRESCALER == 1)
#define C_TMRx_CTRL_MODE0       (C_CTIMER0_DIV_CPU_DIV_1 | C_CTIMER0_MODE_TIMER | B_CTIMER0_ENCMP)  /*!< Timer mode 0, Divider 1 */
#elif (TIMER_PRESCALER == 16)
#define C_TMRx_CTRL_MODE0       (C_CTIMER0_DIV_CPU_DIV_16 | C_CTIMER0_MODE_TIMER | B_CTIMER0_ENCMP)  /*!< Timer mode 0, Divider 16 */
#elif (TIMER_PRESCALER == 256)
#define C_TMRx_CTRL_MODE0       (C_CTIMER0_DIV_CPU_DIV_256 | C_CTIMER0_MODE_TIMER | B_CTIMER0_ENCMP)  /*!< Timer mode 0, Divider 256 */
#else  /* (TIMER_PRESCALER) */
#error "ERROR: Unsupported TIMER_PRESCALER"
#endif /* (TIMER_PRESCALER) */
#define C_MIN_PWM_PERIOD        (PWM_REG_PERIOD / TIMER_PRESCALER)              /*!< Minimum timer period */

#if (_SUPPORT_APP_TYPE != C_APP_SOLENOID) && (_SUPPORT_APP_TYPE != C_APP_RELAY)
#if ((((PLL_FREQ / TIMER_PRESCALER) * 60UL)/(C_MOTOR_PHASES * 2UL * C_MICROSTEP_PER_FULLSTEP * C_MOTOR_POLE_PAIRS * C_SPEED_MIN)) > 65535UL)
#error "ERROR: 16-bit CTimer commutation-period overflow"
#endif
#endif /* (_SUPPORT_APP_TYPE != C_APP_SOLENOID) && (_SUPPORT_APP_TYPE != C_APP_RELAY) */

#if (LINPROT == LIN13_HVACTB)
/*! Motor Mode */
typedef enum __attribute__((packed))
{
    C_MOTOR_MODE_NONE = 0,                                                      /*!< 0: Mode unknown */
    C_MOTOR_MODE_NORMAL,                                                        /*!< 1: Normal mode */
    C_MOTOR_MODE_RECOVER                                                        /*!< 2: Recover mode */
} MOTOR_MODE;
#endif /* (LINPROT == LIN13_HVACTB) */

/*! Motor Request types */
typedef enum __attribute__((packed))
{
    C_MOTOR_REQUEST_NONE = 0,                                                   /*!< 0: No request */
#if (_SUPPORT_APP_TYPE == C_APP_RELAY)
    C_RELAYS_REQUEST_OFF,                                                       /*!< 1: Request to RELAY OFF */
    C_RELAY1_REQUEST_ON,                                                        /*!< 2: Request to RELAY #1 ON */
    C_RELAY2_REQUEST_ON,                                                        /*!< 3: Request to RELAY #2 ON */
    C_RELAYS_REQUEST_ON,                                                        /*!< 4: Request to RELAY #1 & #2 ON */
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
    C_SOLENOID_REQUEST_DEACTIVATE,                                              /*!< 1: Request to de-activate Solenoid */
    C_SOLENOID_REQUEST_ACTIVATE,                                                /*!< 2: Request to activate Solenoid */
#else  /* (_SUPPORT_APP_TYPE) */
    C_MOTOR_REQUEST_STOP,                                                       /*!< 1: Request to STOP motor */
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) && (_SUPPORT_POS_INIT != FALSE)
    C_MOTOR_REQUEST_INIT,                                                       /*!< 2: Request to initialise motor */
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) && (_SUPPORT_POS_INIT != FALSE) */
    C_MOTOR_REQUEST_START,                                                      /*!< 3: Request to START/SET motor */
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) && (_SUPPORT_POS_INIT != FALSE)
    C_MOTOR_REQUEST_START_wINIT,                                                /*!< 4: Request to START with INIT motor */
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) && (_SUPPORT_POS_INIT != FALSE) */
#if (_SUPPORT_CALIBRATION != FALSE)
    C_MOTOR_REQUEST_CALIBRATION,                                                /*!< 5: Request to start calibration */
#endif /* (_SUPPORT_CALIBRATION != FALSE) */
#endif /* (_SUPPORT_APP_TYPE) */
#if (LIN_COMM != FALSE) && (_SUPPORT_LIN_SLEEP != FALSE)
    C_MOTOR_REQUEST_SLEEP,                                                      /*!< 6: Request to sleep */
#if (LINPROT == LIN13_HVACTB)
    C_MOTOR_REQUEST_SLEEP_AFTER_STOP,                                           /*!< 4: Request to sleep after motor-STOP (MMP220331-1) */
#endif /* (LINPROT == LIN13_HVACTB) */
#endif /* (LIN_COMM != FALSE) && (_SUPPORT_LIN_SLEEP != FALSE) */
#if (LIN_COMM != FALSE) && (_SUPPORT_BUSTIMEOUT != FALSE) && (_SUPPORT_EMERGENCY_RUN != FALSE)
    C_MOTOR_REQUEST_EMRUN,                                                      /*!< 7: Request to Emergency Run */
#endif /* (LIN_COMM != FALSE) && (_SUPPORT_BUSTIMEOUT != FALSE) && (_SUPPORT_EMERGENCY_RUN != FALSE) */
#if (_SUPPORT_SPEED_CHANGE != FALSE)
    C_MOTOR_REQUEST_SPEED_CHANGE,                                               /*!< 8: Request to change speed */
#endif /* (_SUPPORT_SPEED_CHANGE != FALSE) */
    C_MOTOR_REQUEST_RESTART,                                                    /*!< 9: Request to restart motor (stop + start) */
    C_MOTOR_REQUEST_RESET,                                                      /*!< 10: Request to reset */
    C_MOTOR_REQUEST_CALIB_FACTORY,                                              /*!< 11: Request to calibrate (factory mode) */
#if (_SUPPORT_MOTOR_SELFTEST != FALSE)
    C_MOTOR_REQUEST_SELFTEST                                                    /*!< 12: Self-Test */
#endif /* (_SUPPORT_MOTOR_SELFTEST != FALSE) */
} MOTOR_REQUEST;

#if (_SUPPORT_CALIBRATION != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
/*! Calibration State */
typedef enum __attribute__((packed))
{
    C_CALIB_NONE = 0,                                                           /*!< 0: No calibration */
    C_CALIB_START,                                                              /*!< 1: Start calibration */
    C_CALIB_SETUP_HI_ENDPOS,                                                    /*!< 2: Setup movement towards High Endstop */
    C_CALIB_CHECK_HI_ENDPOS,                                                    /*!< 3: High Endstop reached */
    C_CALIB_PAUSE_HI_ENDSTOP,                                                   /*!< 4: High Endstop Pause */
    C_CALIB_SETUP_LO_ENDPOS,                                                    /*!< 5: Setup movement towards Low Endstop */
    C_CALIB_CHECK_LO_ENDPOS,                                                    /*!< 6: Low Endstop reached */
#if (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL == 1U)
    C_CALIB_SETUP_MIDPOS,                                                       /*!< 7: Setup movement towards middle position */
    C_CALIB_CHECK_MIDPOS,                                                       /*!< 8: Check movement middle position */
#endif /* (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL == 1U) */
    C_CALIB_DONE = 10,                                                          /*!< 10: Calibration successfully done */
    C_CALIB_FAILED,                                                             /*!< 11: Calibration failed */
    C_CALIB_FAILED_NO_ENDSTOP,                                                  /*!< 12: Calibration failed, no end-stop */
    C_CALIB_FAILED_TOO_LONG,                                                    /*!< 13: Calibration failed, too long */
    C_CALIB_FAILED_TOO_SHORT,                                                   /*!< 14: Calibration failed, too short */
    C_CALIB_INIT,                                                               /*!< 15: Calibration towards safety-position (MMP170622-1) */
#if (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL == 1U)
    C_CALIB_FAILED_NO_MIDSTOP,                                                  /*!< 16: Calibration failed, no mid-stop */
#endif /* (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL == 1U) */
    C_CALIB_CHECK_1ST_ENDPOS = C_CALIB_CHECK_HI_ENDPOS,                         /*!< 1st End-stop */
    C_CALIB_CHECK_2ND_ENDPOS = C_CALIB_CHECK_LO_ENDPOS                          /*!< 2nd End-stop */
} CALIB_MODE;

/*! Calibration Auto-Mode */
typedef enum __attribute__((packed))
{
    C_CALIB_LOW_POS = 0,                                                        /*!< 0: Half-automatic, towards low position */
    C_CALIB_HIGH_POS,                                                           /*!< 1: Half-automatic, towards high position */
    C_CALIB_FULL_AUTOMATIC,                                                     /*!< 2: Full-automatic */
    C_CALIB_UNKNOWN,                                                            /*!< 3: Unknown */
    C_CALIB_HALF_WAY = 0x40,                                                    /*!< Calibration half-way done */
    C_CALIB_FINAL = 0x80                                                        /*!< Final-step for Half-automatic */
} CALIB_AUTO_MODE;
#endif /* (_SUPPORT_CALIBRATION != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */

#if ((LIN_COMM != FALSE) && ((CAN_COMM != FALSE) || (I2C_COMM != FALSE) || (PWM_COMM != FALSE) || (SPI_COMM != FALSE) || \
    ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR)))) || \
    ((I2C_COMM != FALSE) && (PWM_COMM != FALSE))
/*! Communication Mode */
typedef enum __attribute__((packed))
{
    C_COMM_DISCONNECTED = 0,                                                    /*!< 0: Communication disconnected */
    C_COMM_CONNECTED,                                                           /*!< 1: Communication connected */
    C_COMM_ACTIVE                                                               /*!< 2: Communication active */
} COMM_MODE;
#endif /* ((LIN_COMM != FALSE) && ((CAN_COMM != FALSE) || (PWM_COMM != FALSE) || (SPI_COMM != FALSE))) || ((I2C_COMM != FALSE) && (PWM_COMM != FALSE)) */

/*! Stall detection mode */
#define C_STALLDET_NONE 0x00U                                                   /*!< No stall detector enabled */
#if (_SUPPORT_STALLDET_A != FALSE)
#define C_STALLDET_A 0x01U                                                      /*!< Stall detector "A" enabled (Current) */
#else  /* (_SUPPORT_STALLDET_A != FALSE) */
#define C_STALLDET_A 0x00U                                                      /*!< Stall detector "A" disabled */
#endif /* (_SUPPORT_STALLDET_A != FALSE) */
#if (_SUPPORT_STALLDET_O != FALSE)
#define C_STALLDET_O 0x02U                                                      /*!< Stall detector "O" enabled (Oscillations) */
#else  /* (_SUPPORT_STALLDET_O != FALSE) */
#define C_STALLDET_O 0x00U                                                      /*!< Stall detector "O" disabled */
#endif /* (_SUPPORT_STALLDET_O != FALSE) */
#if (_SUPPORT_STALLDET_P != FALSE)
#define C_STALLDET_P 0x04U                                                      /*!< Stall detector "P" enabled (Position) */
#else  /* (_SUPPORT_STALLDET_P != FALSE) */
#define C_STALLDET_P 0x00U                                                      /*!< Stall detector "P" disabled */
#endif /* _SUPPORT_STALLDET_P */
#if (_SUPPORT_STALLDET_LA != FALSE)
#define C_STALLDET_LA 0x08U                                                     /*!< Stall detector "LA" enabled */
#else  /* (_SUPPORT_STALLDET_LA != FALSE) */
#define C_STALLDET_LA 0x00U                                                     /*!< Stall detector "LA" disabled */
#endif /* (_SUPPORT_STALLDET_LA != FALSE) */
#if (_SUPPORT_STALLDET_FLUX != FALSE)
#define C_STALLDET_FLUX 0x08U                                                   /*!< Stall detector "FLUX" enabled */
#else  /* (_SUPPORT_STALLDET_FLUX != FALSE) */
#define C_STALLDET_FLUX 0x00U                                                   /*!< Stall detector "FLUX" disabled */
#endif /* (_SUPPORT_STALLDET_FLUX != FALSE) */
#if (_SUPPORT_STALLDET_S != FALSE)
#define C_STALLDET_S 0x10U                                                      /*!< Stall detector "S" enabled (Speed) */
#else  /* (_SUPPORT_STALLDET_S != FALSE) */
#define C_STALLDET_S 0x00U                                                      /*!< Stall detector "S" disabled */
#endif /* (_SUPPORT_STALLDET_S != FALSE) */
#if (_SUPPORT_STALLDET_BZC != FALSE) || (_SUPPORT_STALLDET_BRI != FALSE)
#define C_STALLDET_B 0x20U                                                      /*!< Stall detector "B" enabled (BEMF Zero-cross or BEMF Rectified Integrator) */
#else  /* (_SUPPORT_STALLDET_BZC != FALSE) || (_SUPPORT_STALLDET_BRI != FALSE) */
#define C_STALLDET_B 0x00U                                                      /*!< Stall detector "B" disabled */
#endif /* (_SUPPORT_STALLDET_BZC != FALSE) || (_SUPPORT_STALLDET_BRI != FALSE) */
#if (_SUPPORT_CDI != FALSE)
#define C_STALLDET_CDI 0x20U                                                    /*!< Stall detector "CDI" enabled (Current Direction Indicator) */
#else  /* (_SUPPORT_CDI != FALSE) */
#define C_STALLDET_CDI 0x00U                                                    /*!< Stall detector "CDI" disabled */
#endif /* (_SUPPORT_CDI != FALSE) */
#if (_SUPPORT_STALLDET_H != FALSE)
#define C_STALLDET_H 0x20U                                                      /*!< Stall detector "H" enabled (Hall-Latch) */
#else  /* _SUPPORT_STALLDET_H != FALSE) */
#define C_STALLDET_H 0x00U                                                      /*!< Stall detector "H" disabled */
#endif /* _SUPPORT_STALLDET_H != FALSE) */
#define C_STALLDET_ALL (C_STALLDET_A | C_STALLDET_O | C_STALLDET_P | C_STALLDET_LA | \
                        C_STALLDET_FLUX | C_STALLDET_S | C_STALLDET_B | C_STALLDET_H)  /*!< All stall detectors enabled */
#define C_STALLDET_CALIB 0x40U                                                  /*!< Stall detector calibration enabled */

/* Motor speeds (g_u8MotorCtrlSpeed/g_u8MotorStatusSpeed) */
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
#if (LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52) || (LINPROT == LIN2X_AGS)
#define C_MOTOR_SPEED_STOP          0U                                          /*!< Actuator: Stop (Status) */
#define C_MOTOR_SPEED_1             1U                                          /*!< Actuator: 2.25 RPM */
#define C_MOTOR_SPEED_2             2U                                          /*!< Actuator: 3 RPM */
#define C_MOTOR_SPEED_3             3U                                          /*!< Actuator: 4 RPM */
#define C_MOTOR_SPEED_4             4U                                          /*!< Actuator: 5 RPM */
#define C_MOTOR_SPEED_AUTO          5U                                          /*!< Actuator: Auto-speed (Control) */
#elif (LINPROT == LIN2X_AIRVENT12)
#define C_MOTOR_SPEED_STOP          0U                                          /*!< Actuator: Stop (Status) */
#define C_MOTOR_SPEED_1             1U                                          /*!< Actuator: 6 RPM */
#define C_MOTOR_SPEED_2             2U                                          /*!< Actuator: 9 RPM */
#define C_MOTOR_SPEED_3             3U                                          /*!< Actuator: 13 RPM */
#define C_MOTOR_TORQUE_MODE         6U                                          /*!< Actuator: Torque-mode (1-7 RPM) */
#elif (LINPROT == LIN22_SIMPLE_PCT)
#define C_MOTOR_SPEED_STOP          0U                                          /*!< Actuator: Stop (Status) */
#define C_MOTOR_SPEED_AUTO          0U                                          /*!< Actuator: Auto-speed (Control) */
#define C_MOTOR_SPEED_1             1U                                          /*!< Actuator:  2cm/s */
#define C_MOTOR_SPEED_2             2U                                          /*!< Actuator: 10cm/s */
#define C_MOTOR_SPEED_3             3U                                          /*!< Actuator: 16cm/s */
#elif (I2C_SLAVE_PROT == I2C_SLAVE_PROT_POSITIONING)
#define C_MOTOR_SPEED_STOP          0U                                          /*!< Actuator: Stop (Status) */
#define C_MOTOR_SPEED_AUTO          0U                                          /*!< Actuator: Auto-speed (Control) */
#define C_MOTOR_SPEED_1             1U                                          /*!< Actuator: Speed-Mode #1 */
#define C_MOTOR_SPEED_2             2U                                          /*!< Actuator Speed-mode 2 */
#define C_MOTOR_SPEED_3             3U                                          /*!< Actuator Speed-mode 3 */
#elif (LINPROT == LIN13_HVACTB)
#define C_MOTOR_SPEED_STOP          0U                                          /*!< Actuator: Stop (Status) */
#define C_MOTOR_SPEED_1             1U                                          /*!< Actuator: 2.25 RPM */
#define C_MOTOR_SPEED_2             2U                                          /*!< Actuator: 3 RPM */
#define C_MOTOR_SPEED_3             3U                                          /*!< Actuator: 4 RPM */
#define C_MOTOR_SPEED_4             4U                                          /*!< Actuator: 5 RPM */
#else
#define C_MOTOR_SPEED_STOP          0U                                          /*!< Actuator: Stop (Status) */
#define C_MOTOR_SPEED_AUTO          0U                                          /*!< Actuator: Auto-speed (Control) */
#define C_MOTOR_SPEED_1             1U                                          /*!< Actuator: Speed-Mode #1 */
#define C_MOTOR_SPEED_2             2U                                          /*!< Actuator Speed-mode 2 */
#define C_MOTOR_SPEED_3             3U                                          /*!< Actuator Speed-mode 3 */
#endif /* (LINPROT == LIN22_SIMPLE_PCT) */
#define C_CALIB_MOTOR_SPEED         C_MOTOR_SPEED_1                             /*!< Calibration speed-mode */
#define C_DEFAULT_MOTOR_SPEED       C_MOTOR_SPEED_1                             /*!< Default speed-mode */
#define C_BUTTON_MOTOR_SPEED        C_MOTOR_SPEED_2                             /*!< Button-operation speed-mode */
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */

#if (LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52) || (LINPROT == LIN2X_AIRVENT12) || (LINPROT == LIN2X_FAN01) || (LINPROT == LIN2X_AGS) || \
    (I2C_SLAVE_PROT == I2C_SLAVE_PROT_POSITIONING) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_CONTINUOUS) || \
    (CAN_COMM != FALSE)
/* Motor Control mode (g_e8MotorCtrlMode) */
#define C_MOTOR_CTRL_STOP           0x00U                                       /*!< Motor control STOP */
#define C_MOTOR_CTRL_NORMAL         0x01U                                       /*!< Motor control NORMAL */
#if (LINPROT == LIN2X_HVAC52)                                                   /* MMP201217-2 */
#define C_MOTOR_CTRL_NORMAL_GAD     0x02U                                       /*!< Motor control NORMAL (GAD-mode) */
#endif /* (LINPROT == LIN2X_HVAC52) */
#define C_MOTOR_CTRL_CALIBRATION    0x03U                                       /*!< Motor Control calibration */
#endif /* (LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52) || (LINPROT == LIN2X_AIRVENT12) || (LINPROT == LIN2X_FAN01) || (LINPROT == LIN2X_AGS) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_POSITIONING) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_CONTINUOUS) */

/*! Motor Status mode (g_e8MotorStatus) */
typedef enum __attribute__((packed))
{
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
    C_MOTOR_STATUS_STOP = ((uint8_t) 0x00U),                                    /*!< bit 3:0 = 0b0000: Stop mode (w/o holding) */
    C_MOTOR_STATUS_HOLD = ((uint8_t) 0x01U),                                    /*!< bit 3:0 = 0b0001: Holding mode (stopped) */
#if (_SUPPORT_FAST_STOP != FALSE)
    C_MOTOR_STATUS_FAST_STOP = ((uint8_t) 0x02U),                               /*!< bit 3:0 = 0b0010: Actuator Fast-stop (Safe-stop) */
#endif /* (_SUPPORT_FAST_STOP != FALSE) */
    C_MOTOR_STATUS_WINDMILLING = ((uint8_t) 0x04U),                             /*!< bit 3:0 = 0b0100: Windmilling */
    C_MOTOR_STATUS_SRP = ((uint8_t) 0x05U),                                     /*!< bit 3:0 = 0b0101: Static Rotor Positioning */
#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR)
    C_MOTOR_STATUS_ALIGNMENT = ((uint8_t) 0x06U),                               /*!< bit 3:0 = 0b0110: Alignment */
#endif /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) */
    C_MOTOR_STATUS_RUNNING = ((uint8_t) 0x08U),                                 /*!< bit 3:0 = 0b1000: Running (including accelerating, decelerating) */
#if (_SUPPORT_SOFT_START != FALSE)
    C_MOTOR_STATUS_SOFT_START = ((uint8_t) 0x09U),                              /*!< bit 3:0 = 0b1001: Soft Start-up */
#endif /* (_SUPPORT_SOFT_START != FALSE) */
    C_MOTOR_STATUS_STOPPING = ((uint8_t) 0x0AU),                                /*!< bit 3:0 = 0b1010: Stopping (but still running, decelerating) */
#if (_SUPPORT_BRAKING != FALSE)
    C_MOTOR_STATUS_BRAKING = ((uint8_t) 0x0CU),                                 /*!< bit 3:0 = 0b1100: Actuator is braking */
#endif /* (_SUPPORT_BRAKING != FALSE) */
    C_MOTOR_STATUS_STOP_MASK = ((uint8_t) 0x0CU),                               /*!< AND-MASK; 0b1100: If zero: Stopped */
    C_MOTOR_STATUS_MASK = ((uint8_t) 0x0FU),                                    /*!< AND-MASK of state */
#elif (_SUPPORT_APP_TYPE == C_APP_RELAY)
    /* Relay Status:
     * - Off-state: Relay is OFF
     * - Activation State: Relay is activated (OFF to ON)
     * - On-state: Relay is ON
     * - Holding State: Relay remains ON, with reduced holding current
     * - De-activation State: Relay is de-actiavted (ON to OFF)
     */
    C_RELAY_STATUS_MASK = ((uint8_t) (7U << 0)),                                /*!< Relay Status AND-MASK */
    C_RELAY_STATUS_OFF = ((uint8_t) (0U << 0)),                                 /*!< bit 2:0 = 0b000: Relay is OFF */
    C_RELAY_STATUS_RESISTANCE = ((uint8_t) (1U << 0)),                          /*!< bit 2:0 = 0b001: Relay Resistance measurement */
    C_RELAY_STATUS_INDUCTANCE = ((uint8_t) (2U << 0)),                          /*!< bit 2:0 = 0b010: Relay Inductance measurement */
    C_RELAY_STATUS_ACTIVATED = ((uint8_t) (4U << 0)),                           /*!< bit 2:0 = 0b100: Relay goes ON */
    C_RELAY_STATUS_ON = ((uint8_t) (5U << 0)),                                  /*!< bit 2:0 = 0b101: Relay is ON */
    C_RELAY_STATUS_HOLDING = ((uint8_t) (6U << 0)),                             /*!< bit 2:0 = 0b110: Relay in Holding */
    C_RELAY_STATUS_DEACTIVATED = ((uint8_t) (7U << 0)),                         /*!< bit 2:0 = 0b111: Relay goes OFF */
    C_RELAY_STATUS_ON_MASK = ((uint8_t) (4U << 0)),                             /*!< bit 2:0 = 0b1xx: Relay Status-ON AND-MASK */
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
    C_SOLENOID_STATUS_DEACTIVATED = ((uint8_t) 0x00U),                          /*!< bit 3:0 = 0b0000: Stop mode (w/o holding) */
    C_SOLENOID_STATUS_HOLDING = ((uint8_t) 0x01U),                              /*!< bit 3:0 = 0b0001: Holding mode (stopped) */
    C_SOLENOID_STATUS_ACTIVATED = ((uint8_t) 0x02U),                            /*!< bit 3:0 = 0b0010: Active */
    C_SOLENOID_STATUS_MASK = ((uint8_t) 0x0FU),                                 /*!< AND-MASK of state */
#endif /* (_SUPPORT_APP_TYPE) */
    C_MOTOR_STATUS_INIT = ((uint8_t) 0x10U),                                    /*!< bit 4 = 0b1: Actuator initialisation (Only internal state) */
    C_MOTOR_STATUS_SELFTEST = ((uint8_t) 0x20U),                                /*!< bit 5 = 0b1: Actuator Self-test (Only internal state) */
    C_MOTOR_STATUS_APPL_STOP = ((uint8_t) 0x40U),                               /*!< bit 6 = 0b1: Application stopped */
} MOTOR_STATUS;

/*! Over-temperature error (g_e8ErrorOverTemperature) */
typedef enum __attribute__((packed))
{
    C_ERR_OTEMP_NO = ((uint8_t) 0x00U),                                         /*!< Temperature OK */
    C_ERR_OTEMP_WARN = ((uint8_t) 0x01U),                                       /*!< Temperature warning level */
    C_ERR_OTEMP_SHUTDOWN = ((uint8_t) 0x02U),                                   /*!< IC Junction over temperature reached >165C */
    C_ERR_OTEMP_ERROR = ((uint8_t) 0x80U)                                       /*!< Temperature sensor error */
} TEMPSENSOR_STATUS;
#define C_TEMPERATURE_HYS           3                                           /*!< Temperature hysteric: 3 degrees Celsius */
#define C_TEMPERATURE_JUMP          25                                          /*!< Maximum temperature "jump" per measurement-period: 25 degrees Celsius */
#define C_CHIP_OVERTEMP_LEVEL       160                                         /*!< Chip Over-temperature level is 160C */

/* Electronics error (g_e8ErrorElectric) */
#define C_ERR_NONE                  0x00U                                       /*!< No electric error */
#define C_ERR_PERMANENT             0x80U                                       /*!< Bit 7: Permanent error (reset after POR) */
#define C_ERR_SEMI_PERMANENT        0x40U                                       /*!< Bit 6: Semipermanent error (cleared by application) */
#define C_ERR_SUPPLY                0x20U                                       /*!< Bit 5: Generic supply error (Permanent) */
#define C_ERR_SUP_VDDA              C_ERR_SUPPLY                                /*!< Analogue supply error (VDDA) */
#define C_ERR_SUP_VDDD              (C_ERR_PERMANENT | C_ERR_SUPPLY)            /*!< Digital supply error (VDDD) */
#define C_ERR_SUP_VBGD              C_ERR_SUPPLY                                /*!< Bandgap Digital supply error (VBGD) )(MMP220307-1) */
#define C_ERR_SUP_VIO               C_ERR_SUPPLY                                /*!< Vio supply error */
#define C_ERR_SUP_VDDAF             C_ERR_SUPPLY                                /*!< VDDAF supply error (MLX8133x) */
#define C_ERR_SUP_VBOOST            C_ERR_SUPPLY                                /*!< VBOOST supply error (MLX8134x) */
#define C_ERR_MEMORY                0x10U                                       /*!< Bit 4: Generic Memory error (Permanent) */
#define C_ERR_MEM_NVM_CALIB         (C_ERR_PERMANENT | C_ERR_MEMORY)            /*!< Non Volatile Memory Calibration Area error */
#define C_ERR_MEM_FLASH             (C_ERR_PERMANENT | C_ERR_MEMORY)            /*!< Flash CRC24 error */
#define C_ERR_MEM_COLIN_ROM         (C_ERR_PERMANENT | C_ERR_MEMORY)            /*!< COLIN ROM CRC24 error */
#define C_ERR_MEM_SYS_ROM           (C_ERR_PERMANENT | C_ERR_MEMORY)            /*!< System ROM CRC24 error */
#define C_ERR_MEM_RAM               (C_ERR_PERMANENT | C_ERR_MEMORY)            /*!< RAM Transparent BIST error */
#define C_ERR_SENSOR                (C_ERR_PERMANENT | 0x08U)                   /*!< Bit 3: Sensor error (Permanent) */
#define C_ERR_RPT_OVER_TEMPERATURE  0x04U                                       /*!< bit 2: Repeated Over-temperature */
#define C_ERR_MOTOR_ZERO_CURRENT    0x02U                                       /*!< bit 1: Zero-current, Open-coil */
#define C_ERR_MOTOR_OVER_CURRENT    0x01U                                       /*!< bit 0: Over-current, Vds-error, Phase-to-phase, Phase-to-Supply, Phase-to-Ground */
#define C_ERR_MOTOR                 (C_ERR_MOTOR_ZERO_CURRENT | C_ERR_MOTOR_OVER_CURRENT) /*!< Motor related error */
#define C_ERR_VDS                   C_ERR_MOTOR_OVER_CURRENT                    /*!< VDS-error */
#define C_ERR_MOTOR_TEST_VPH        (C_ERR_PERMANENT | C_ERR_MOTOR_OVER_CURRENT) /*!< Motor Phase-voltage error */

/* Voltage error (g_e8ErrorVoltage) */
#define C_ERR_VOLTAGE_IN_RANGE      0U                                          /*!< Motor Voltage in (user) range */
#define C_ERR_VOLTAGE_UNDER         1U                                          /*!< Motor Voltage below user range */
#define C_ERR_VOLTAGE_OVER          2U                                          /*!< Motor Voltage above user range */
#define C_VOLTAGE_HYS               ((uint16_t)50U)                             /*!< Voltage hysteric:  0.5V (  50 x 10mV) */

/* Emergency/Safety run */
#define C_SAFETY_RUN_NO             0U                                          /*!< No Safety/Emergency-run */
#define C_SAFETY_RUN_ACTIVE         1U                                          /*!< Active Safety/Emergency-run */
#define C_SAFETY_RUN_DONE           2U                                          /*!< Safety/Emergency-run done */

/* Motor rotational direction (g_e8MotorDirectionCCW) */
#define C_MOTOR_DIR_OPENING         0U                                          /*!< Positioning: Moving from low-to-high Position */
#define C_MOTOR_DIR_CW              0U                                          /*!< Motor direction: Clock Wise */
#define C_MOTOR_DIR_CLOSING         1U                                          /*!< Positioning: Moving from high-to-low Position */
#define C_MOTOR_DIR_CCW             1U                                          /*!< Motor direction: Counter Clock Wise */
#define C_MOTOR_DIR_UNKNOWN         2U                                          /*!< Motor direction: Unknown */

#if (_SUPPORT_TNCTOC != FALSE)
/* g_e8TNCTOC */
#define C_TNCTOC_NONE               0x00U                                       /*!< No TNCTOC indication */
#define C_TNCTOC_ADC_DONE           0x01U                                       /*!< Bit 0: ADC measurement of Temperature & Voltage */
#define C_TNCTOC_LIN                0x02U                                       /*!< Bit 1: LIN Communication detected */
#define C_TNCTOC_MOTOR              0x04U                                       /*!< Bit 2: Motor operation */
#endif /* (_SUPPORT_TNCTOC != FALSE) */

/* Debounce error filter; An error has to be detected twice in a row */
#define C_DEBFLT_ERR_NONE           0x00U                                       /*!< No error */
#define C_DEBFLT_ERR_PHASE_SHORT    0x01U                                       /*!< Bit 0: Phase short to ground error */
#define C_DEBFLT_ERR_OVT            0x02U                                       /*!< Bit 1: Over-Temperature error */
#define C_DEBFLT_ERR_UV             0x04U                                       /*!< Bit 2: Under-Voltage error */
#define C_DEBFLT_ERR_OV             0x08U                                       /*!< Bit 3: Over-Voltage error */
#define C_DEBFLT_ERR_TEMP_PROFILE   0x10U                                       /*!< Bit 4: Chip Temperature profile error */
#define C_DEBFLT_ERR_TEMP_SENSOR    0x20U                                       /*!< Bit 5: Temperature sensor defect */
#define C_DEBFLT_ERR_HALL_LATCH     0x40U                                       /*!< Bit 6: Hall-Latch sequence error */
#define C_DEBFLT_ERR_GEARBOX        0x80U                                       /*!< Bit 7: Gearbox error */

/* DIV 16: Max period: 37ms @ 28MHz, 32.5ms @ 32MHz */
#define C_SLEEP_TIMER_CONFIG    (C_CTIMER0_DIV_CPU_DIV_256 | C_CTIMER0_MODE_TIMER | B_CTIMER0_ENCMP) /*!< Sleep timer configuration */
#define C_SLEEP_TIMER_PERIOD    ((uint16_t)((PLL_FREQ / 256U) * 0.030))           /*!< Sleep period of 30 ms (limited by AWD) */

#define C_DELAY_mPWM  (uint16_t)(((1000000UL / C_DELAY_CONST) * FPLL) / PWM_FREQ)  /*!< PWM-period delay */

#define C_REWIND_DIRECTION_CCW      0x01U                                       /*!< bit 0: Last rotational direction; Must be bit 0 (g_e8MotorDirectionCCW) */
#define C_REWIND_STALL_DETECT       0x02U                                       /*!< bit 1: Stall have been detected; Must be bit 1 (DC-motors) */
#define C_REWIND_DIRECTION_ANY      0x04U                                       /*!< bit 2: Any-direction (POR) */
#define C_REWIND_ACTIVE             0x08U                                       /*!< bit 3: Rewind active */
#define C_REWIND_FULLAUTO           0x10U                                       /*!< bit 4: Full Automatic actuator mode */
#define C_REWIND_REWIND             0x20U                                       /*!< bit 5: Rewinding of the Rewind is active */

/* MLX4/LIN State */
#define C_LIN_SLEEP                 0U                                          /*!< MLX4/LIN Bus-timeout or Sleep-request */
#define C_LIN_AWAKE                 1U                                          /*!< LIN-bus active */

/*! Application check error codes */
#define C_ERR_APP_OK                0U                                          /*!< Application module check OK */
#define C_ERR_APP_RST               1U                                          /*!< Application module check failed; Chip reset required */

#define C_TEMP_STABIL_TIMEOUT       C_PI_TICKS_250MS                            /*!< Temperature interval of 250ms */
#define C_TEMP_STABIL_INT_FILTER_COEF   8U                                      /*!< Temperature stability integrator filter coefficient: 1/(2^n) */
#define C_TEMP_STABIL_THRESHOLD     20U                                         /*!< Temperature threshold (output integrator filter) 30C * (1 - ((2^n-1)/2^n)^(4*60)) = 19 */

#if (_SUPPORT_SPEED_CTRL != FALSE) && (_SUPPORT_VOLTAGE_CTRL != FALSE)
#define C_SPEED_CTRL                0U                                          /*!< Motor Control Type: SPEED */
#define C_VOLTAGE_CTRL              1U                                          /*!< Motor Control Type: VOLTAGE */
#endif /* (_SUPPORT_SPEED_CTRL != FALSE) && (_SUPPORT_VOLTAGE_CTRL != FALSE) */

#if (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_ROTOR)
#define C_ROTOR_TOLERANCE           0                                           /*!< Rotor tolerance: 0 steps */
#elif (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_SHAFT)
#define C_SHAFT_TOLERANCE_ANGLE     (int16_t) ((65536UL * 0.5)/360)             /*!< Shaft-angle tolerance: 0.5 degrees */
#endif /* (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_SHAFT) */


/*!*************************************************************************** */
/*                           GLOBAL VARIABLES                                  */
/* *************************************************************************** */
#pragma space dp                                                                /* __TINY_SECTION__ */
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
extern uint16_t g_u16MinSpeedRPM;                                               /*!< Minimum Speed [RPM] */
extern uint16_t g_u16StartupSpeedRPM;                                           /*!< Initial Start-up Speed [RPM] */
extern uint16_t g_u16LowSpeedRPM;                                               /*!< Low Speed [RPM] (NV_ACT_SPEED1) */
extern uint16_t g_u16MaxSpeedRPM;                                               /*!< Maximum Speed [RPM] (NV_ACT_SPEED3 or NV_ACT_SPEED4) */
#if (LINPROT == LIN2X_AIRVENT12)
extern uint16_t g_u16TorqueSpeedRPM;                                            /*!< Torque-mode speed [RPM] (NV_ACT_SPEED4) */
#endif /* (LINPROT == LIN2X_AIRVENT12) */
extern uint16_t g_u16MotorPolePairs;                                            /*!< Number of Motor rotor pole-pairs (MMP221101-1) */
extern uint8_t g_u8StallTypeComm;                                               /*!< Stall type occurred (communication) */
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
extern uint8_t g_u8MotorStatusSpeed;                                            /*!< (Status) Actual motor-speed */
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
#if (LINPROT == LIN2X_AAB) || (LINPROT == LIN2X_KVA)
extern uint8_t g_u8ActPosValid;                                                 /*!< Initial Position Set */
#endif /* (LINPROT == LIN2X_AAB) || (LINPROT == LIN2X_KVA) */
#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT) */
extern uint8_t g_e8MotorRequest;                                                /*!< Control Motor Request */
extern volatile uint8_t g_e8ErrorElectric;                                      /*!< Status-flags electric error */
extern volatile uint8_t g_e8ErrorVoltage;                                       /*!< Status-flags voltage */
extern volatile uint8_t g_u8ChipResetOcc;                                       /*!< Status-flag indicate chip-reset occurred (POR) */
extern volatile uint8_t g_u8StallOcc;                                           /*!< Status-flag indicate stall occurred */
extern volatile uint8_t g_e8EmergencyRunOcc;                                    /*!< Status-flag indicate Emergency-run occurred */
extern volatile uint8_t g_e8ErrorOverTemperature;                               /*!< Status-flag over-temperature */
#if (_SUPPORT_DEGRADED_MODE != FALSE)
extern uint8_t g_e8DegradedMotorRequest;                                        /*!< Degraded Motor Request */
#endif /* (_SUPPORT_DEGRADED_MODE != FALSE) */
#if (_SUPPORT_CALIBRATION != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
extern uint8_t g_e8CalibrationStep;                                             /*!< Calibration step */
extern uint8_t g_e8CalibPostMotorRequest;                                       /*!< Post calibration Motor Request */
#if (_SUPPORT_HALF_AUTO_CALIB != FALSE)
extern uint8_t g_u8HalfAutomaticCalibration;                                    /*!< Calibration mode */
#endif /* (_SUPPORT_HALF_AUTO_CALIB != FALSE) */
#endif /* (_SUPPORT_CALIBRATION != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
#pragma space none                                                              /* __TINY_SECTION__ */

#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
#if (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_ROTOR)
extern uint16_t g_u16ActualPosition __attribute__ ((dp, section(".dp.noinit")));  /*!< (Control/Status) Actual motor-rotor position */
extern uint16_t g_u16TargetPosition __attribute__ ((dp, section(".dp.noinit")));  /*!< (Control) Target motor-rotor position (invalid) */
#elif (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_SHAFT) && \
      ((_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
       (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
       (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE))
extern uint16_t g_u16ActualShaftAngle __attribute__ ((dp, section(".dp.noinit")));  /*!< (Control/Status) Actual (outer) shaft angle */
extern uint16_t g_u16TargetShaftAngle __attribute__ ((dp, section(".dp.noinit")));  /*!< (Control) Target (outer) shaft angle */
#endif /* (_SUPPORT_MOTOR_POSITION) */
#if (_SUPPORT_CALIBRATION != FALSE)
extern uint16_t g_u16RealTravel __attribute__ ((dp, section(".dp.noinit")));    /*!< Number of steps between two end-stops */
#endif /* (_SUPPORT_CALIBRATION !=  FALSE) */
#if (_SUPPORT_VOLTAGE_CTRL != FALSE)
extern uint16_t g_u16ActualMotorVoltage __attribute__ ((dp, section(".dp.noinit")));  /*!< (Control) Actual motor-voltage (mode) */
extern uint16_t g_u16TargetMotorVoltage __attribute__ ((dp, section(".dp.noinit")));  /*!< (Control) Target motor-voltage (mode) */
#endif /* (_SUPPORT_VOLTAGE_CTRL != FALSE) */
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
extern uint16_t g_u16ActualMotorSpeedRPM __attribute__ ((dp, section(".dp.noinit")));  /*!< Actual motor-speed [RPM] */
extern uint16_t g_u16TargetMotorSpeedRPM __attribute__ ((dp, section(".dp.noinit")));  /*!< Target motor-speed [RPM] */
#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT) */
extern volatile uint8_t g_e8MotorStatus __attribute__ ((dp, section(".dp.noinit")));  /*!< Status-flags motor mode */
#if (_SUPPORT_DEGRADED_MODE != FALSE)
extern volatile uint8_t g_e8DegradeStatus __attribute__ ((dp, section(".dp.noinit")));  /*!< Status-flags degraded mode */
#endif /* (_SUPPORT_DEGRADED_MODE != FALSE) */
extern uint8_t g_e8StallDetectorEna __attribute__ ((dp, section(".dp.noinit")));  /*!< Control-flag Stall-detector enabled */
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
extern uint8_t g_u8MotorHoldingCurrEna __attribute__ ((dp, section(".dp.noinit"))); /*!< Control-flag motor Holding-current enabled */
#if (_SUPPORT_ACT_SPEED_BY_LIN == FALSE) || (CAN_COMM != FALSE) || (SPI_COMM != FALSE)
extern uint8_t g_u8MotorCtrlSpeed __attribute__ ((dp, section(".dp.noinit")));  /*!< (Control) Selected motor-speed */
#endif /* (_SUPPORT_ACT_SPEED_BY_LIN == FALSE) || (CAN_COMM != FALSE) || (SPI_COMM != FALSE) */
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) || (CAN_COMM != FALSE) */
extern uint8_t g_e8MotorDirectionCCW __attribute__ ((dp, section(".dp.noinit")));  /*!< Control/Status-flag motor rotational direction Counter Clock-wise */
#if (LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52) || (LINPROT == LIN2X_AIRVENT12) || (LINPROT == LIN2X_FAN01) || (LINPROT == LIN2X_AGS) || \
    (I2C_SLAVE_PROT == I2C_SLAVE_PROT_POSITIONING) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_CONTINUOUS) || \
    (CAN_COMM != FALSE)
extern uint8_t g_e8MotorCtrlMode __attribute__ ((dp, section(".dp.noinit")));   /*!< Control-flags motor mode (from Master) */
#endif /* (LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52) || (LINPROT == LIN2X_AIRVENT12) || (LINPROT == LIN2X_FAN01) || (LINPROT == LIN2X_AGS) || \
        * (I2C_SLAVE_PROT == I2C_SLAVE_PROT_POSITIONING) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_CONTINUOUS) || (CAN_COMM != FALSE) */
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) && (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM)
extern volatile uint32_t l_u32ActualPosition __attribute__ ((dp, section(".dp.noinit")));  /*!< (Motor-driver) Actual motor-rotor position */
extern uint32_t g_u32TargetPosition __attribute__ ((dp, section(".dp.noinit")));  /*!< (Motor-driver) Target motor-rotor position */
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) && (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM) */
#elif (_SUPPORT_APP_TYPE == C_APP_RELAY)
#if (_SUPPORT_DUAL_RELAY == FALSE)
extern volatile uint8_t g_e8RelayStatus __attribute__ ((dp, section(".dp.noinit")));  /*!< Status-flags relay mode */
#else  /* (_SUPPORT_DUAL_RELAY == FALSE) */
extern volatile uint8_t g_e8RelayStatusA __attribute__ ((dp, section(".dp.noinit")));  /*!< Status-flags relay A mode */
extern volatile uint8_t g_e8RelayStatusB __attribute__ ((dp, section(".dp.noinit")));  /*!< Status-flags relay B mode */
#endif /* (_SUPPORT_DUAL_RELAY == FALSE) */
extern volatile uint8_t g_e8DegradeStatus __attribute__ ((dp, section(".dp.noinit")));  /*!< Status-flags degraded mode */
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
extern volatile uint8_t g_e8SolenoidStatus __attribute__ ((dp, section(".dp.noinit")));  /*!< Status-flags solenoid mode */
extern uint8_t g_e8StallDetectorEna __attribute__ ((dp, section(".dp.noinit")));  /*!< Control-flag Stall-detector enabled */
#if (_SUPPORT_DEGRADED_MODE != FALSE)
extern volatile uint8_t g_e8DegradeStatus __attribute__ ((dp, section(".dp.noinit")));  /*!< Status-flags degraded mode */
#endif /* (_SUPPORT_DEGRADED_MODE != FALSE) */
#endif /* (_SUPPORT_APP_TYPE) */

#pragma space nodp                                                              /* __NEAR_SECTION__ */

extern uint16_t g_u16MotorStartDelay;                                           /*!< Motor start delay (500us) */

#if ((GPIO_COMM != FALSE) && ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_TERMINAL)))
extern uint8_t g_u8GpioConnected;                                               /*!< GPIO Communication state */
#endif /* ((GPIO_COMM != FALSE) && ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_TERMINAL))) */

extern uint8_t g_u8OverTemperatureCount;                                        /*!< Number of over-temperature events */
extern uint8_t g_e8ErrorVoltageComm;                                            /*!< Status-flags voltage (Communication) */
extern uint8_t g_u8NewMotorDirectionCCW;
#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || (_SUPPORT_TRIAXIS_MLX90427 != FALSE)
extern uint8_t g_u8PorMovement;                                                 /*!< Power-on movement */
#endif /* (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || (_SUPPORT_TRIAXIS_MLX90427 != FALSE) */
#if (_SUPPORT_MECHANICAL_ERROR != FALSE)
extern uint8_t g_u8MechError;                                                   /*!< No mechanical error */
#endif /* (_SUPPORT_MECHANICAL_ERROR != FALSE) */
#if (LIN_COMM != FALSE) && (_SUPPORT_NV_WRITE_IMMEDIATE == FALSE)
extern uint8_t g_u8RamTest;                                                     /*!< RAM background test (MMP170119-1) */
#endif /* (LIN_COMM != FALSE) && (_SUPPORT_NV_WRITE_IMMEDIATE == FALSE) */
#if (_SUPPORT_TNCTOC != FALSE)
extern volatile uint8_t g_e8TNCTOC;                                             /*!< TNCTOC done flags */
#endif /* (_SUPPORT_TNCTOC != FALSE) */
#if (_SUPPORT_MOTOR_SELFTEST != FALSE)
extern uint8_t g_u8ForceMotorDriverSelfTest;                                    /*!< Force Motor Driver Self-test flag; MMP180917-1 */
#endif /* (_SUPPORT_MOTOR_SELFTEST != FALSE) */

#if (_SUPPORT_SPEED_CTRL != FALSE) && (_SUPPORT_VOLTAGE_CTRL != FALSE)
extern uint8_t g_e8ControlType;                                                 /*!< Motor Control Type: SPEED or VOLTAGE */
#endif /* (_SUPPORT_SPEED_CTRL != FALSE) && (_SUPPORT_VOLTAGE_CTRL != FALSE) */

#if (_SUPPORT_REWIND != FALSE) || (_SUPPORT_STALL_REVERSE != FALSE)
extern volatile uint8_t g_u8RewindFlags;                                        /*!< Rewind flags */
#endif /* (_SUPPORT_REWIND != FALSE) || (_SUPPORT_STALL_REVERSE != FALSE) */

#if (_SUPPORT_SPECIAL_COMM_FIELD != FALSE)
extern uint8_t g_u8Special[6];                                                  /*!< Special communication field info */
#endif /* (_SUPPORT_SPECIAL_COMM_FIELD != FALSE) */

#if (LINPROT == LIN13_HVACTB)
extern uint8_t g_u8MotorOperationMode;                                          /*!< Motor operation mode (default: Stepper) */
#endif /* (LINPROT == LIN13_HVACTB) */

#if (CAN_COMM != FALSE) && (LIN_COMM != FALSE)
extern uint8_t g_u8CanConnected;                                                /*!< CAN Communication state */
#endif /* (CAN_COMM != FALSE) && (LIN_COMM != FALSE) */
#if (I2C_COMM != FALSE) && \
    (((LIN_COMM != FALSE) && (I2C_SLAVE_PROT != I2C_SLAVE_PROT_NONE) && (C_I2C_SLAVE_SCK_IO == PIN_FUNC_LIN)) || \
     (PWM_COMM != FALSE))
extern uint8_t g_u8I2cConnected;                                                /*!< I2C Communication state */
#endif /* (I2C_COMM != FALSE) && (((LIN_COMM != FALSE) && (I2C_SLAVE_PROT != I2C_SLAVE_PROT_NONE) && (C_I2C_SLAVE_SCK_IO == PIN_FUNC_LIN)) || (PWM_COMM != FALSE)) */
#if (LIN_COMM != FALSE) && ((CAN_COMM != FALSE) || \
     ((I2C_COMM != FALSE) && (I2C_SLAVE_PROT != I2C_SLAVE_PROT_NONE) && (C_I2C_SLAVE_SCK_IO == PIN_FUNC_LIN)) || \
     (PWM_COMM != FALSE) || (SPI_COMM != FALSE) || \
     ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR)))
extern uint8_t g_u8LinConnected;                                                /*!< LIN Communication state */
#endif /* (LIN_COMM != FALSE) && ((CAN_COMM != FALSE) || (PWM_COMM != FALSE) || (SPI_COMM != FALSE) || ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR))) */
#if (PWM_COMM != FALSE) && ((LIN_COMM != FALSE) || (I2C_COMM != FALSE))
extern uint8_t g_u8PwmConnected;                                                /*!< PWM Communication state */
#endif /* (PWM_COMM != FALSE) && ((LIN_COMM != FALSE) || (I2C_COMM != FALSE)) */
#if (SPI_COMM != FALSE) && (LIN_COMM != FALSE)
extern uint8_t g_u8SpiConnected;                                                /*!< SPI Communication state */
#endif /* (SPI_COMM != FALSE) && (LIN_COMM != FALSE) */
#if (LIN_COMM != FALSE)
extern uint16_t g_u16MLX4_RAM_Dynamic_CRC1;                                     /*!< MLX4-RAM Dynamic Frame-IDs CRC (Part 1) */
extern uint16_t g_u16MLX4_RAM_Dynamic_CRC2;                                     /*!< MLX4-RAM Dynamic Frame-IDs CRC (Part 2) */
#endif /* (LIN_COMM != FALSE) */
#if (_SUPPORT_STALL_AUTO_CLEAR != FALSE)
extern uint16_t g_u16StallPeriod;                                               /*!< Stall automatic clear period */
#endif /* (_SUPPORT_STALL_AUTO_CLEAR != FALSE) */

#if (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE)
extern uint16_t g_u16ActSpeed2_NV;                                              /*!< Actuator speed #2 [RPM] */
extern uint16_t g_u16ActSpeed3_NV;                                              /*!< Actuator speed #3 [RPM] */
extern uint16_t g_u16MotorConst_NV;                                             /*!< Motor (BEMF) constant */
extern uint16_t g_u16StallDetectorDelay_NV;                                     /*!< Stall Detector Delay */
extern uint16_t g_u16MaxCorrectionRatio_NV;                                     /*!< Maximum Correction Ratio */
extern uint16_t g_u16StartUpCorrectionRatio_NV;                                 /*!< Start-up Correction Ratio */
extern uint16_t g_u16HoldingCurrentLevel_NV;                                    /*!< Holding Current Level */
extern uint16_t g_u16StartUpCurrentLevel_NV;                                    /*!< Start-up Current level */
extern uint16_t g_u16TargetLoadAngle_NV;                                        /*!< Target Load-Angle */
extern uint16_t g_u16MotorCoilResistanceTotal_NV;                               /*!< Motor Coil Resistance (Total) */
extern uint8_t g_u8EmergencyRunPosEna_NV;                                       /*!< Emergency Run Position Enable flag */
extern uint8_t g_u8EmergencyRunPos_NV;                                          /*!< Emergency Run Position */
extern uint8_t g_u8MotorDirectionCCW_NV;                                        /*!< Rotational Direction Motor */
extern uint8_t g_u8BusTimeOutSleep_NV;                                          /*!< Bus Time-out Sleep flag */
#endif /* (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE) */
#pragma space none                                                              /* __NEAR_SECTION__ */

#endif /* DRIVE_LIB_GLOBAL_VARS_H */

/* EOF */

