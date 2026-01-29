/*!*************************************************************************** *
 * \file        ActParams.h
 * \brief       MLX8133x Actuator Parameters
 *
 * \note        project MLX8133x
 *
 * \author      Marcel Braat
 *
 * \date        2017-08-15
 *
 * \version     2.0
 *
 * A list of actuators:
 *           -# MT_ITW_EVENT
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
 * *************************************************************************** */

#ifndef _ACT_PARAMS_H_
#define _ACT_PARAMS_H_

/*!*************************************************************************** */
/*                           INCLUDES                                          */
/* *************************************************************************** */
#include "ActDefines.h"                                                         /* Actuator Defines */

/*!*************************************************************************** */
/*                           DEFINITIONS                                       */
/* *************************************************************************** */
#define C_IC_UV_LEVEL               3U                                          /*!< 0 = 4V, 1 = 5V, 2 = 6V, 3 = 7V, 4 = 8V, 5 = 9V +/- 0.5V */
#define C_IC_OV_LEVEL               0U                                          /*!< 0 = 22V, 1 = 24V, 2 = 40V +/- 2.0V */
#define C_HALL_LATCHES              0U                                          /*!< No Hall-Latches */
#define C_BUSTIMEOUT_SLEEP          0U                                          /*!< Bus-timeout to Sleep: Disabled */
#define C_AUTO_RECALIBRATE          0U                                          /*!< Disable Auto re-calibration */
#define C_LIN_UV                    2U                                          /*!< LIN UV: 7.0V (6.0V + n * 0.V) */
#define C_CURRTHRSHLD_TEMP_HYS      2                                           /*!< Hysteric around points */

#if (C_MICROSTEP_PER_FULLSTEP == 1)
#define C_MOTOR_MICROSTEPS          0U                                          /*!< Number of micro-steps per full step: 1 (1 << 0) steps per phase-step */
#elif (C_MICROSTEP_PER_FULLSTEP == 2)
#define C_MOTOR_MICROSTEPS          1U                                          /*!< Number of micro-steps per full step: 2 (1 << 1) steps per phase-step */
#elif (C_MICROSTEP_PER_FULLSTEP == 4)
#define C_MOTOR_MICROSTEPS          2U                                          /*!< Number of micro-steps per full step: 4 (1 << 2) steps per phase-step */
#elif (C_MICROSTEP_PER_FULLSTEP == 8U)
#define C_MOTOR_MICROSTEPS          3U                                          /*!<  8 (1 << 3) steps per phase-step */
#elif (C_MICROSTEP_PER_FULLSTEP == 16U)
#define C_MOTOR_MICROSTEPS          4U                                          /*!< 16 (1 << 4) steps per phase-step */
#elif (C_MICROSTEP_PER_FULLSTEP == 32U)
#define C_MOTOR_MICROSTEPS          5U                                          /*!< 32 (1 << 5) steps per phase-step */
#endif /* (C_MICROSTEP_PER_FULLSTEP == 16U) */

#if (MOTOR_TYPE == MT_ITW_EVENT)
/* **************************************************************************** *
 * ITW e-Vent Flap Stepper Motor
 *
 * Inductance: 40mH, Resistor: 50R
 *
 * Motor PWM Frequency is based on:
 *  maximum speed of 1430 RPM (internal), 5 pole-pairs and 32 micro-steps.
 *  mPWM-Freq = 1430/60 (RPS) * 5 (PP) * 32 (uSteps) = 3813.3Hz.
 *  mPWM-Freq = 5 * 3813.3Hz = 19067Hz
 *  mPWM-Freq = 6 * 3813.3Hz = 22880Hz
 * **************************************************************************** */
#define CONFIGURATION_ID   (uint16_t)((('U' - '@') << 10) | (('H' - '@') << 5) | ('L' - '@')) /*!< UniROM Basic 'L' (ITW e-Vent, 50R) */
#define _SUPPORT_APP_TYPE           C_APP_POSITIONING_ACT                       /*!< Application Device is Position operation (Flap) */
#define C_MOTOR_FAMILY              MF_STEPPER                                  /*!< Stepper motor family */
#define _SUPPORT_BIPOLAR_MODE       BIPOLAR_MODE_UW_VT				//BIPOLAR_MODE_UW_VT, BIPOLAR_MODE_UT_VW,
#define PWM_FREQ                    19067U                                      /*!< Motor PWM signal frequency in Hz */

#define C_MOTOR_COILS               2U                                          /*!< Number of Motor Coils */
#define C_MOTOR_PHASES              4U                                          /*!< Number of phases: 4 (Bi-Polar) */
#define C_MOTOR_POLE_PAIRS          5U                                          /*!< Number of Pole-pairs */
#define C_MOTOR_STEP_PER_PHASE      2U                                          /*!< Number of full-steps per phase */
#define C_MOTOR_GEAR_BOX_RATIO      143U                                        /*!< Gear-box ratio: 142.9 */
#define C_SHAFT_STEPS_PER_ROTATION  6400U                                       /*!< One mechanical rotation of the shaft is divided into full-steps */
#define C_TACHO_MODE                0U                                          /*!< 0: No tacho, 1: 60-degrees commutation, 2: 180-degrees commutation, 3: 180-degrees mechanical-rotation */

#define C_MOVAVG_SSZ                4U                                          /*!< Two commutation periods of 8 micro-steps */
#define C_COIL_R                    5000U                                       /*!< Coil Resistance: 50R */
#define C_COIL_L                    40000U                                      /*!< Coil Inductance [uH] */
#define C_DETECTOR_DELAY            128U                                        /*!< Number of micro-steps before running stall-detection becomes active */
#define C_STALL_A_DET               1U                                          /*!< Stall "A": Enabled */
#define C_STALL_O_DET               0U                                          /*!< Stall "O": Enabled */
#define C_STALL_S_DET               0U                                          /*!< Stall "S": Disabled */
#define C_STALL_A_THRESHOLD         ((uint8_t)(((20.0 * 128) + 50) / 100))      /*!< Stall "A" Threshold:  20.0% */
#define C_STALL_A_WIDTH             3U                                          /*!< Stall "A" Width: 3 micro-steps */
#define C_STALL_O_THRESHOLD         ((uint8_t)((0 * 128) / 100))                /*!< Stall "O" Threshold: 0% */
#define C_STALL_O_WIDTH             (1U - 1U)                                   /*!< Stall "O" Width: 1 micro-steps */
#define C_STALL_S_THRESHOLD         ((uint8_t)((0 * 128U) / 100U))              /*!< Stall "S" Threshold: 0% of SPEED_0 */
#define C_STALL_S_WIDTH             0U                                          /*!< Stall "S" Width: 1 updates */
#define C_STALL_SPEED_DEPENDED      0U                                          /*!< 0 = Fixed stall-threshold; 1 = Speed depended threshold */
#define C_RESTALL_POR               0U                                          /*!< Re-stall after POR: disabled */
#define C_MOTOR_CONST_10MV_PER_RPS  17U                                         /*!< Motor-constant: 17 [10mV/RPS-m] */
#define C_MOTOR_CONST_MV_PER_RPS    170U                                        /*!< Motor-constant: 170 [mV/RPS-m] (MMP240726-2) */
#define C_TOT_COILS_R               C_COIL_R                                    /*!< Phase-to-phase Coil Resistance, bi-polar [10mR] */
#define C_TOT_COILS_L               C_COIL_L                                    /*!< Phase-to-phase Coil Inductance, bi-polar [uH] */
#define C_PID_HOLDING_CURR_LEVEL    25U                                         /*!< Motor holding current threshold [mA] */
#define C_PID_RUNNING_CURR_LEVEL    100U                                        /*!< Motor running current threshold [mA] */
#define C_PID_BOOST_CURR_LEVEL      0U                                          /*!< Motor running current threshold in torque-boost mode [mA] */
#define C_NV_CURR_DIV               0U                                          /*!< Motor current divider: 1 << n */
#define C_REWIND_STEPS              8U                                          /*!< Rewinding steps with/out holding current */

#define C_PID_RUNNING_CURR_MIN      10U                                         /*!< Minimum operating motor current [mA] */
#define C_PID_RUNNING_CURR_MAX      100U                                        /*!< Maximum operating motor current [mA] */
#define C_PID_STARTING_CURR_MAX     100U                                        /*!< Start-up motor current [mA] */
#define C_SPEED_ALIGNMENT           60U                                         /*!< Start-up alignment speed [RPM] */

/* ***
 * Speed
 * ***/
#define C_SPEED_MIN                  143U                                       /*!< Start-up speed RPM */
#define C_SPEED_0                    286U                                       /*!< Motor RPM's (w/o gearbox), 2 RPM */
#define C_SPEED_1                    572U                                       /*!< Motor RPM's (w/o gearbox), 4 RPM */
#define C_SPEED_2                   1000U                                       /*!< Motor RPM's (w/o gearbox), 7 RPM */
#define C_SPEED_3                   1430U                                       /*!< Motor RPM's (w/o gearbox), 10 RPM */
#define C_SPEED_TORQUE_BOOST        0U                                          /*!< N.A. */
#define C_ACCELERATION_CONST        10000U                                      /*!< Reach 2400RPM in 128 updates */
#define C_ACCELERATION_STEPS        C_MOTOR_MICROSTEPS                          /*!< Acceleration points */
#define C_DECELERATION_STEPS        C_MOTOR_MICROSTEPS                          /*!< Deceleration points */

#define C_AUTOSPEED_TEMP_1          -40                                         /*!< Auto-speed temperature #1 */
#define C_AUTOSPEED_TEMP_2          -15                                         /*!< Auto-speed temperature #2 */
#define C_AUTOSPEED_TEMP_3          +60                                         /*!< Auto-speed temperature #3 */
#define C_AUTOSPEED_TEMPZONE_1      0U                                          /*!< Auto-speed temperature-zone #1 interpolation: constant */
#define C_AUTOSPEED_TEMPZONE_2      1U                                          /*!< Auto-speed temperature-zone #2 interpolation: linear */
#define C_AUTOSPEED_TEMPZONE_3      3U                                          /*!< Auto-speed temperature-zone #3 interpolation: constant */
#define C_AUTOSPEED_TEMPZONE_4      1U                                          /*!< Auto-speed temperature-zone #4 interpolation: linear */
#define C_AUTOSPEED_VOLT_1          ((uint16_t)(10.5 * 8))                      /*!< Auto-speed Voltage #1 */
#define C_AUTOSPEED_VOLT_2          ((uint16_t)(11.75 * 8))                     /*!< Auto-speed Voltage #2 */
#define C_AUTOSPEED_VOLT_3          0xFFU                                       /*!< Auto-speed Voltage #3 */
#define C_AUTOSPEED_VOLTZONE_1      0U                                          /*!< Auto-speed voltage-zone #1 interpolation: constant */
#define C_AUTOSPEED_VOLTZONE_2      1U                                          /*!< Auto-speed voltage-zone #2 interpolation: linear */
#define C_AUTOSPEED_VOLTZONE_3      3U                                          /*!< Auto-speed voltage-zone #3 interpolation: linear */
#define C_AUTOSPEED_VOLTZONE_4      0U                                          /*!< Auto-speed voltage-zone #4 interpolation: linear */

/* IV compensation: P = 192(/256), I = 2(/256), D = 0(/256) */
#define C_TARGET_LA                 0U                                          /*!< 452 degrees (speed-dependent); 45/360 * 2^12  = 3uSteps @ 48uSteps/rotation; Needed to support negative torque of 60 cN.m */
#define C_PID_LA_COEF_P             192U                                        /*!< Load-angle PID COEF_P = 160/256 = 0.75 */
#define C_PID_LA_COEF_I             2U                                          /*!< Load-angle PID COEF_I = 5/256 = 0.0078 */
#define C_PID_LA_COEF_D             0U                                          /*!< Load-angle PID COEF_D = 0/256 = 0.0 */
#define C_LA_OPEN_TO_CLOSE_AMPL     6U                                          /*!< Load-angle Open-to-close amplitude reduction: 6/8 */
#define C_STALL_LA_SPEED_OFFSET     C_SPEED_MIN                                 /*!< Stall "LA" Speed offset [RPM] */
#define C_STALL_SPEED_THRESHOLD     24U                                         /*!< n/32 */

/*
 *  4-line/5-zone Current threshold vs. temperature curve.
 *
 *              |Zone1|   Zone2   |   Zone3   |   Zone4   | Zone5
 *  RATIO_1 ----+-----*
 *  (1.35)      |       \
 *  RATIO_4 ----+         \                              *-------
 *  (1.20)      |           \                          /
 *              |             \                     /
 *              |               \                /
 *  RATIO_2/3 --+                 *-----------*
 *  (1.00)      |
 *              +-----+-----------+-----------+-----------+---------+-------
 *                    |           |           |           |         |
 *                  TEMP1       TEMP2       TEMP3       TEMP4   C_APPL_OT
 *                  (-34)       ( +8)       (+38)       (+62)     (+80)
 */
#define C_CURRTHRSHLD_TEMP_1        -34                                         /*!< Current Set-level compensation temperature #1 */
#define C_CURRTHRSHLD_TEMP_2         +8                                         /*!< Current Set-level compensation temperature #2 */
#define C_CURRTHRSHLD_TEMP_3        +38                                         /*!< Current Set-level compensation temperature #3 */
#define C_CURRTHRSHLD_TEMP_4        +62                                         /*!< Current Set-level compensation temperature #4 */
#define C_CURRTHRSHLD_TEMP_HYS      2                                           /*!< Hysteric around points */
#define C_CURRTHRSHLD_RATIO_1       ((uint16_t)(1.02 * 128))                    /*!< Current Set-level compensation ratio #1: +2 */
#define C_CURRTHRSHLD_RATIO_2       ((uint16_t)(1.00 * 128))                    /*!< Current Set-level compensation ratio #2: 0% */
#define C_CURRTHRSHLD_RATIO_3       ((uint16_t)(1.00 * 128))                    /*!< Current Set-level compensation ratio #3: 0% */
#define C_CURRTHRSHLD_RATIO_4       ((uint16_t)(1.12 * 128))                    /*!< Current Set-level compensation ratio #4: +12% */
#define C_CURRTHRSHLD_AREA_1        1U                                          /*!< As _1 */
#define C_CURRTHRSHLD_AREA_2        3U                                          /*!< Interpolate between _1 and _2 */
#define C_CURRTHRSHLD_AREA_3        1U                                          /*!< As _2 */
#define C_CURRTHRSHLD_AREA_4        3U                                          /*!< Interpolate between _3 and _4 */
#define C_CURRTHRSHLD_AREA_5        1U                                          /*!< As _4 */

/* ***
 * Application diagnostic levels
 * ***/
#define C_APP_OT_LEVEL              105                                         /*!< Application Over-Temperature level (ambient) */
#define C_APP_UV_LEVEL              ((uint8_t)(8.0 * 8))                        /*!< Application Under-Voltage level */
#define C_APP_OV_LEVEL              ((uint8_t)(17.5 * 8))                       /*!< Application Over-Voltage level */

/* ***
 * PID Control
 * ***/
#define C_PID_RUNNINGCTRL_PERIOD    1U                                          /*!< Every 10ms (= 10 x 1ms); Range: 1..255 ms */
#define C_PID_RUNNINGCTRL_PERIOD_UNIT C_PID_PERIOD_UNIT_TIME                    /*!< PID Period base: C_PID_PERIOD_UNIT_TIME: Time or C_PID_PERIOD_UNIT_SPEED: Speed */
#define C_PID_HOLDINGCTRL_PERIOD    125U                                        /*!< Every 250ms (= (125<<2) x 500us); Range: 2..500 ms */
#define C_PID_COEF_P                32U                                         /*!< COEF_P = 32/64 = 0.5 */
#define C_PID_COEF_I                32U                                         /*!< COEF_I = 32/64 = 0.5 */
#define C_PID_COEF_D                0U                                          /*!< COEF_D = 0/64 = 0.000 */
#define C_PID_THRSHLDCTRL_PERIOD    16U                                         /*!< Every 1s (= (16<<6) x 1ms); Range: 64ms..16s */
#define C_LOADDUMP_VOLT             100U                                        /*!< 1.00 V change (approximate 7.5-8%) */
#define C_VSUP_REF                  ((uint8_t)(13.5 * 8))                       /*!< Reference voltage for PWM Duty-cycle control */
#define C_MIN_HOLDCORR_RATIO        ((uint8_t)(2.5 * 256 / 100))                /*!< HOLDING: 2.5% = 6/256 */
#define C_MIN_CORR_RATIO            ((uint8_t)(8 * 256 / 100))                  /*!< DELTA: 8% = 64/256 */
#define C_MAX_CORR_RATIO            ((uint8_t)((98.4 * 256 / 100) - 1))         /*!< 98.4% = 252/256 */
#define C_PID_RAMP_DOWN             0U                                          /*!< Disabled ( 50 * 4) RPM/PID-period */
#define C_PID_RAMP_UP               0U                                          /*!< Disabled (150 * 4) RPM/PID period */

#define C_LEAD_ANGLE                0U                                          /*!< 0 degrees (= N * 1.875 degrees) */
#define C_STARTUP_CORR_RATIO        0U                                          /*!< Startup Motor PWM Duty-Cycle */

/* R = 50R, L = 50mH
 * LA Threshold: Atan2(R, XL), with XL = 2*pi*(C_SPEED_0 = 286/60)*PP*L; XL =  7.49 --> ATAN2 = 8.52[deg] (1551)
 * LA Threshold: Atan2(R, XL), with XL = 2*pi*(C_SPEED_1 = 572/60)*PP*L; XL =  15.0 --> ATAN2 = 16.7[deg]
 * LA Threshold: Atan2(R, XL), with XL = 2*pi*(C_SPEED_2 = 1000/60)*PP*L; XL =  26.2 --> ATAN2 = 27.6[deg]
 * LA Threshold: Atan2(R, XL), with XL = 2*pi*(C_SPEED_3 = 1430/60)*PP*L; XL =  37.4 --> ATAN2 = 36.8[deg]
 */
/*#define C_STALL_LA_THRESHOLD_SPEED_0  3U*/                                    /*!< Stall "LA" negative angle threshold (3 degrees units) (SPEED_0) */
/*#define C_STALL_LA_THRESHOLD_SPEED_1  5U*/                                    /*!< Stall "LA" negative angle threshold (3 degrees units) (SPEED_1) */
/*#define C_STALL_LA_THRESHOLD_SPEED_2  9U*/                                    /*!< Stall "LA" negative angle threshold (3 degrees units) (SPEED_2) */
/*#define C_STALL_LA_THRESHOLD_SPEED_3 12U*/                                    /*!< Stall "LA" negative angle threshold (3 degrees units) (SPEED_3) */
#define C_STALL_LA_THRESHOLD_FOC    3U                                          /*!< Stall "LA" negative angle threshold (3 degrees units) (SPEED_0) - Lowest Speed */
#define C_STALL_LA_THRESHOLD_OL     12U                                         /*!< Stall "LA" negative angle threshold (3 degrees units) (SPEED_3) - Highest Speed */
#define C_STALL_LA_WIDTH_FOC        8U                                          /*!< Stall "LA" Width: 8 PWM-periods */
#define C_STALL_LA_WIDTH_OL         3U                                          /*!< Stall "LA" Width: 3 OL-periods */
#endif /* (MOTOR_TYPE == MT_ITW_EVENT) */

#if !defined (C_DEFAULT_TRAVEL)
#define C_DEFAULT_TRAVEL     (uint16_t)(((120UL * (C_SHAFT_STEPS_PER_ROTATION / 20U)) + 9U) / 18U) /*!< 120 degrees */
#endif
#define C_TRAVEL_OFFSET      (uint16_t)(((15UL * (C_SHAFT_STEPS_PER_ROTATION / 20U)) + 9U) / 18U) /*!<  15-degrees */
#define C_ROUNDING_OFFSET    (uint16_t)(((1UL * (C_SHAFT_STEPS_PER_ROTATION / 20U)) + 9U) / 18U) /*!<   1-degrees */
#define C_TRAVEL_TOLERANCE_LO (uint8_t)(((10UL * (C_SHAFT_STEPS_PER_ROTATION / 20U)) + 9U) / 18U) /*!<  10-degrees */
#define C_TRAVEL_TOLERANCE_UP (uint8_t)(((10UL * (C_SHAFT_STEPS_PER_ROTATION / 20U)) + 9U) / 18U) /*!<  10-degrees */
#if !defined (C_ENDSTOP_TIME)
#define C_ENDSTOP_TIME              100U                                        /*!< End-stop delay: 100ms */
#endif

#endif /* MOTOR_PARAMS_H */

/* EOF */
