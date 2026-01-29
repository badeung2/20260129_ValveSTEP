/*!************************************************************************** *
 * \file        MotorDriver.h
 * \brief       MLX8133x Motor Driver handling
 *
 * \note        project MLX8133x
 *
 * \author      Marcel Braat
 *
 * \date        2017-08-15
 *
 * \version     2.0
 *
 *
 * MELEXIS Microelectronic Integrated Systems
 *
 * Copyright (C) 2017-2022 Melexis N.V.
 * The Software is being delivered 'AS IS' and Melexis, whether explicitly or
 * implicitly, makes no warranty as to its Use or performance.
 * The user accepts the Melexis Firmware License Agreement.
 *
 * Melexis confidential & proprietary
 *
 * ************************************************************************** */

#ifndef DRIVE_LIB_MOTOR_DRIVER_H
#define DRIVE_LIB_MOTOR_DRIVER_H

/*!************************************************************************** */
/*                          INCLUDES                                          */
/* ************************************************************************** */
#include "../AppBuild.h"                                                        /* Application Build */

#include "../ActADC.h"                                                          /* Application ADC support */

#include "../camculib/private_mathlib.h"                                        /* Private Math Library support */
#include "../hal_lib/hal_PWM.h"                                                 /* PWM HAL support */

#include "fm.h"                                                                 /* Fast Math */

#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)

/*!************************************************************************** */
/*                          DEFINITIONS                                       */
/* ************************************************************************** */
#if defined (__MLX81160__)
#if (_SUPPORT_EVB == MLX81160EVB_3P_3N)
#define DRVCFG_DIS_3P_3N()  {IO_PORT_DRV_OUT = \
                                 (IO_PORT_DRV_OUT & \
                                  ~(B_PORT_DRV_OUT_ENABLE_DRV0 | \
                                    B_PORT_DRV_OUT_ENABLE_DRV1 | \
                                    B_PORT_DRV_OUT_ENABLE_DRV2 | \
                                    B_PORT_DRV_OUT_ENABLE_DRV3 | \
                                    B_PORT_DRV_OUT_ENABLE_DRV4 | \
                                    B_PORT_DRV_OUT_ENABLE_DRV5)); }             /*!< Disable motor-driver */
#define DRVCFG_ENA_3P_3N()  {IO_PORT_DRV_OUT = \
                                 (IO_PORT_DRV_OUT | \
                                  (B_PORT_DRV_OUT_ENABLE_DRV0 | \
                                   B_PORT_DRV_OUT_ENABLE_DRV1 | \
                                   B_PORT_DRV_OUT_ENABLE_DRV2 | \
                                   B_PORT_DRV_OUT_ENABLE_DRV3 | \
                                   B_PORT_DRV_OUT_ENABLE_DRV4 | \
                                   B_PORT_DRV_OUT_ENABLE_DRV5)); }              /*!< Enable motor-driver */
#elif (_SUPPORT_DUAL_BLDC != FALSE)
/* Enable the driver and the PWM phase R, S, T, U, V and W */
#define DRVCFG_DIS_RST_UVW() {IO_PORT_DRV_OUT = \
                                  (IO_PORT_DRV_OUT & \
                                   ~(B_PORT_DRV_OUT_ENABLE_DRV0 | \
                                     B_PORT_DRV_OUT_ENABLE_DRV1 | \
                                     B_PORT_DRV_OUT_ENABLE_DRV2 | \
                                     B_PORT_DRV_OUT_ENABLE_DRV3 | \
                                     B_PORT_DRV_OUT_ENABLE_DRV4 | \
                                     B_PORT_DRV_OUT_ENABLE_DRV5)); }            /*!< Disable motor-driver */
#define DRVCFG_ENA_RST_UVW() {IO_PORT_DRV_OUT = \
                                  (IO_PORT_DRV_OUT | \
                                   (B_PORT_DRV_OUT_ENABLE_DRV0 | \
                                    B_PORT_DRV_OUT_ENABLE_DRV1 | \
                                    B_PORT_DRV_OUT_ENABLE_DRV2 | \
                                    B_PORT_DRV_OUT_ENABLE_DRV3 | \
                                    B_PORT_DRV_OUT_ENABLE_DRV4 | \
                                    B_PORT_DRV_OUT_ENABLE_DRV5)); }             /*!< Enable motor-driver */
/* Enable the driver and the PWM phase R, S and T */
#define DRVCFG_DIS_RST()    {IO_PORT_DRV_OUT = \
                                 (IO_PORT_DRV_OUT & \
                                  ~(B_PORT_DRV_OUT_ENABLE_DRV0 | \
                                    B_PORT_DRV_OUT_ENABLE_DRV1 | \
                                    B_PORT_DRV_OUT_ENABLE_DRV2)); }             /*!< Disable motor-driver */
#define DRVCFG_ENA_RST()    {IO_PORT_DRV_OUT = \
                                 (IO_PORT_DRV_OUT | \
                                  (B_PORT_DRV_OUT_ENABLE_DRV0 | \
                                   B_PORT_DRV_OUT_ENABLE_DRV1 | \
                                   B_PORT_DRV_OUT_ENABLE_DRV2)); }              /*!< Enable motor-driver */
/* Enable the driver and the PWM phase U, V and W */
#define DRVCFG_DIS_UVW()    {IO_PORT_DRV_OUT = \
                                 (IO_PORT_DRV_OUT & \
                                  ~(B_PORT_DRV_OUT_ENABLE_DRV3 | \
                                    B_PORT_DRV_OUT_ENABLE_DRV4 | \
                                    B_PORT_DRV_OUT_ENABLE_DRV5)); }             /*!< Disable motor-driver */
#define DRVCFG_ENA_UVW()    {IO_PORT_DRV_OUT = \
                                 (IO_PORT_DRV_OUT | \
                                  (B_PORT_DRV_OUT_ENABLE_DRV3 | \
                                   B_PORT_DRV_OUT_ENABLE_DRV4 | \
                                   B_PORT_DRV_OUT_ENABLE_DRV5)); }              /*!< Enable motor-driver */
#elif _SUPPORT_1ST_BLDC
/* Enable the driver and the PWM phase R, S and T */
#define DRVCFG_DIS_RST()    {IO_PORT_DRV_OUT = \
                                 (IO_PORT_DRV_OUT & \
                                  ~(B_PORT_DRV_OUT_ENABLE_DRV0 | \
                                    B_PORT_DRV_OUT_ENABLE_DRV1 | \
                                    B_PORT_DRV_OUT_ENABLE_DRV2)); }             /*!< Disable motor-driver */
#define DRVCFG_ENA_RST()    {IO_PORT_DRV_OUT = \
                                 (IO_PORT_DRV_OUT | \
                                  (B_PORT_DRV_OUT_ENABLE_DRV0 | \
                                   B_PORT_DRV_OUT_ENABLE_DRV1 | \
                                   B_PORT_DRV_OUT_ENABLE_DRV2)); }              /*!< Enable motor-driver */
#else
/* Enable the driver and the PWM phase U, V and W */
#define DRVCFG_DIS_UVW()    {IO_PORT_DRV_OUT = \
                                 (IO_PORT_DRV_OUT & \
                                  ~(B_PORT_DRV_OUT_ENABLE_DRV3 | \
                                    B_PORT_DRV_OUT_ENABLE_DRV4 | \
                                    B_PORT_DRV_OUT_ENABLE_DRV5)); }             /*!< Disable motor-driver */
#define DRVCFG_ENA_UVW()    {IO_PORT_DRV_OUT = \
                                 (IO_PORT_DRV_OUT | \
                                  (B_PORT_DRV_OUT_ENABLE_DRV3 | \
                                   B_PORT_DRV_OUT_ENABLE_DRV4 | \
                                   B_PORT_DRV_OUT_ENABLE_DRV5)); }              /*!< Enable motor-driver */
#endif
#if (_SUPPORT_EVB != MLX81160EVB_3P_3N)
/* Enable the driver and the PWM phase R, S, T and U */
#define DRVCFG_DIS_RSTU()   {IO_PORT_DRV_OUT = \
                                 (IO_PORT_DRV_OUT & \
                                  ~(B_PORT_DRV_OUT_ENABLE_DRV0 | \
                                    B_PORT_DRV_OUT_ENABLE_DRV1 | \
                                    B_PORT_DRV_OUT_ENABLE_DRV2 | \
                                    B_PORT_DRV_OUT_ENABLE_DRV3)); }             /*!< Disable motor-driver */
#define DRVCFG_ENA_RSTU()   {IO_PORT_DRV_OUT = \
                                 (IO_PORT_DRV_OUT | \
                                  (B_PORT_DRV_OUT_ENABLE_DRV0 | \
                                   B_PORT_DRV_OUT_ENABLE_DRV1 | \
                                   B_PORT_DRV_OUT_ENABLE_DRV2 | \
                                   B_PORT_DRV_OUT_ENABLE_DRV3)); }              /*!< Enable motor-driver */
/* Enable the driver and the PWM phase T, U, V and W */
#define DRVCFG_DIS_TUVW()   {IO_PORT_DRV_OUT = \
                                 (IO_PORT_DRV_OUT & \
                                  ~(B_PORT_DRV_OUT_ENABLE_DRV2 | \
                                    B_PORT_DRV_OUT_ENABLE_DRV3 | \
                                    B_PORT_DRV_OUT_ENABLE_DRV4 | \
                                    B_PORT_DRV_OUT_ENABLE_DRV5)); }             /*!< Disable motor-driver */
#define DRVCFG_ENA_TUVW()   {IO_PORT_DRV_OUT = \
                                 (IO_PORT_DRV_OUT | \
                                  (B_PORT_DRV_OUT_ENABLE_DRV2 | \
                                   B_PORT_DRV_OUT_ENABLE_DRV3 | \
                                   B_PORT_DRV_OUT_ENABLE_DRV4 | \
                                   B_PORT_DRV_OUT_ENABLE_DRV5)); }              /*!< Enable motor-driver */
/* Enable the driver and the PWM phase R, S, T, U, V and W */
#define DRVCFG_DIS_RSTUVW() {IO_PORT_DRV_OUT = \
                                 (IO_PORT_DRV_OUT & \
                                  ~(B_PORT_DRV_OUT_ENABLE_DRV0 | \
                                    B_PORT_DRV_OUT_ENABLE_DRV1 | \
                                    B_PORT_DRV_OUT_ENABLE_DRV2 | \
                                    B_PORT_DRV_OUT_ENABLE_DRV3 | \
                                    B_PORT_DRV_OUT_ENABLE_DRV4 | \
                                    B_PORT_DRV_OUT_ENABLE_DRV5)); }             /*!< Disable motor-driver */
#define DRVCFG_ENA_RSTUVW() {IO_PORT_DRV_OUT = \
                                 (IO_PORT_DRV_OUT | \
                                  (B_PORT_DRV_OUT_ENABLE_DRV0 | \
                                   B_PORT_DRV_OUT_ENABLE_DRV1 | \
                                   B_PORT_DRV_OUT_ENABLE_DRV2 | \
                                   B_PORT_DRV_OUT_ENABLE_DRV3 | \
                                   B_PORT_DRV_OUT_ENABLE_DRV4 | \
                                   B_PORT_DRV_OUT_ENABLE_DRV5)); }              /*!< Enable motor-driver */
#endif /* (_SUPPORT_EVB != MLX81160EVB_3P_3N) */

/* 3-Phase: R, S and T */
#if (C_MOTOR_PHASES == 3U)
#if (_SUPPORT_EVB == MLX81160EVB_3P_3N)
/* RST = P-FET, UVW = N-FET */
#define DRVCFG_PWM_3P_3N()  {IO_PORT_DRV1_CTRL = (C_PORT_DRV1_CTRL_DRV3_MASTER2 | \
                                                  C_PORT_DRV1_CTRL_DRV2_SLAVE2 | \
                                                  C_PORT_DRV1_CTRL_DRV1_SLAVE1 | \
                                                  C_PORT_DRV1_CTRL_DRV0_MASTER1); \
                             IO_PORT_DRV2_CTRL = (C_PORT_DRV2_CTRL_DRV5_SLAVE4 | \
                                                  C_PORT_DRV2_CTRL_DRV4_SLAVE3); }  /*!< PWM motor-driver */
#define DRVCFG_GND_3P_3N()  {IO_PORT_DRV1_CTRL = (C_PORT_DRV1_CTRL_DRV3_H | \
                                                  C_PORT_DRV1_CTRL_DRV2_H | \
                                                  C_PORT_DRV1_CTRL_DRV1_H | \
                                                  C_PORT_DRV1_CTRL_DRV0_H); \
                             IO_PORT_DRV2_CTRL = (C_PORT_DRV2_CTRL_DRV5_H | \
                                                  C_PORT_DRV2_CTRL_DRV4_H); }   /*!< Grounded motor-driver */
#define DRVCFG_SUP_3P_3N()  {IO_PORT_DRV1_CTRL = (C_PORT_DRV1_CTRL_DRV3_L | \
                                                  C_PORT_DRV1_CTRL_DRV2_L | \
                                                  C_PORT_DRV1_CTRL_DRV1_L | \
                                                  C_PORT_DRV1_CTRL_DRV0_L); \
                             IO_PORT_DRV2_CTRL = (C_PORT_DRV2_CTRL_DRV5_L | \
                                                  C_PORT_DRV2_CTRL_DRV4_L); }   /*!< Supplied motor-driver */
#define DRVCFG_TRI_3P_3N()  {IO_PORT_DRV1_CTRL = (C_PORT_DRV1_CTRL_DRV3_L | \
                                                  C_PORT_DRV1_CTRL_DRV2_H | \
                                                  C_PORT_DRV1_CTRL_DRV1_H | \
                                                  C_PORT_DRV1_CTRL_DRV0_H); \
                             IO_PORT_DRV2_CTRL = (C_PORT_DRV2_CTRL_DRV5_L | \
                                                  C_PORT_DRV2_CTRL_DRV4_L); }   /*!< Tri-stated motor-driver */
#define DRVCFG_CNFG_3P_3N(x)    {IO_PORT_DRV1_CTRL = (x); \
                                 IO_PORT_DRV2_CTRL = (x >> 16); }               /*!< Configure 3-phase Motor Driver */
#elif (_SUPPORT_DUAL_BLDC != FALSE)
#define DRVCFG_PWM_RST_UVW() {IO_PORT_DRV1_CTRL = (C_PORT_DRV1_CTRL_DRV3_MASTER1 | \
                                                   C_PORT_DRV1_CTRL_DRV2_SLAVE2 | \
                                                   C_PORT_DRV1_CTRL_DRV1_SLAVE1 | \
                                                   C_PORT_DRV1_CTRL_DRV0_MASTER1); \
                              IO_PORT_DRV2_CTRL = (C_PORT_DRV2_CTRL_DRV5_SLAVE2 | \
                                                   C_PORT_DRV2_CTRL_DRV4_SLAVE1); }  /*!< PWM motor-driver */
#define DRVCFG_GND_RST_UVW() {IO_PORT_DRV1_CTRL = (C_PORT_DRV1_CTRL_DRV3_L | \
                                                   C_PORT_DRV1_CTRL_DRV2_L | \
                                                   C_PORT_DRV1_CTRL_DRV1_L | \
                                                   C_PORT_DRV1_CTRL_DRV0_L); \
                              IO_PORT_DRV2_CTRL = (C_PORT_DRV2_CTRL_DRV5_L | \
                                                   C_PORT_DRV2_CTRL_DRV4_L); }  /*!< Grounded motor-driver */
#define DRVCFG_SUP_RST_UVW() {IO_PORT_DRV1_CTRL = (C_PORT_DRV1_CTRL_DRV3_H | \
                                                   C_PORT_DRV1_CTRL_DRV2_H | \
                                                   C_PORT_DRV1_CTRL_DRV1_H | \
                                                   C_PORT_DRV1_CTRL_DRV0_H); \
                              IO_PORT_DRV2_CTRL = (C_PORT_DRV2_CTRL_DRV5_H | \
                                                   C_PORT_DRV2_CTRL_DRV4_H); }  /*!< Supplied motor-driver */
#elif _SUPPORT_1ST_BLDC
#define DRVCFG_PWM_RST()    {IO_PORT_DRV1_CTRL = (C_PORT_DRV1_CTRL_DRV3_TRISTATE | \
                                                  C_PORT_DRV1_CTRL_DRV2_SLAVE2 | \
                                                  C_PORT_DRV1_CTRL_DRV1_SLAVE1 | \
                                                  C_PORT_DRV1_CTRL_DRV0_MASTER1); \
                             IO_PORT_DRV2_CTRL = (C_PORT_DRV2_CTRL_DRV5_TRISTATE | \
                                                  C_PORT_DRV2_CTRL_DRV4_TRISTATE); }  /*!< PWM motor-driver */
#define DRVCFG_GND_RST()    {IO_PORT_DRV1_CTRL = (C_PORT_DRV1_CTRL_DRV3_TRISTATE | \
                                                  C_PORT_DRV1_CTRL_DRV2_L | \
                                                  C_PORT_DRV1_CTRL_DRV1_L | \
                                                  C_PORT_DRV1_CTRL_DRV0_L); \
                             IO_PORT_DRV2_CTRL = (C_PORT_DRV2_CTRL_DRV5_TRISTATE | \
                                                  C_PORT_DRV2_CTRL_DRV4_TRISTATE); }   /*!< Grounded motor-driver */
#define DRVCFG_SUP_RST()    {IO_PORT_DRV1_CTRL = (C_PORT_DRV1_CTRL_DRV3_TRISTATE | \
                                                  C_PORT_DRV1_CTRL_DRV2_H | \
                                                  C_PORT_DRV1_CTRL_DRV1_H | \
                                                  C_PORT_DRV1_CTRL_DRV0_H); \
                             IO_PORT_DRV2_CTRL = (C_PORT_DRV2_CTRL_DRV5_TRISTATE | \
                                                  C_PORT_DRV2_CTRL_DRV4_TRISTATE); }   /*!< Supplied motor-driver */
#define DRVCFG_TRI_RST()    {IO_PORT_DRV1_CTRL = (C_PORT_DRV1_CTRL_DRV3_TRISTATE | \
                                                  C_PORT_DRV1_CTRL_DRV2_TRISTATE | \
                                                  C_PORT_DRV1_CTRL_DRV1_TRISTATE | \
                                                  C_PORT_DRV1_CTRL_DRV0_TRISTATE); \
                             IO_PORT_DRV2_CTRL = (C_PORT_DRV2_CTRL_DRV5_TRISTATE | \
                                                  C_PORT_DRV2_CTRL_DRV4_TRISTATE); }  /*!< Tri-stated motor-driver */
#define DRVCFG_CNFG_RST(x)  {IO_PORT_DRV1_CTRL = (C_PORT_DRV1_CTRL_DRV3_TRISTATE | \
                                                  (x)); \
                             IO_PORT_DRV2_CTRL = (C_PORT_DRV2_CTRL_DRV5_TRISTATE | \
                                                  C_PORT_DRV2_CTRL_DRV4_TRISTATE); }  /*!< Configure 3-phase Motor Driver */
#else
/* 3-Phase: U, V and W */
#define DRVCFG_PWM_UVW()    {IO_PORT_DRV1_CTRL = (C_PORT_DRV1_CTRL_DRV3_MASTER1 | \
                                                  C_PORT_DRV1_CTRL_DRV2_TRISTATE | \
                                                  C_PORT_DRV1_CTRL_DRV1_TRISTATE | \
                                                  C_PORT_DRV1_CTRL_DRV0_TRISTATE); \
                             IO_PORT_DRV2_CTRL = (C_PORT_DRV2_CTRL_DRV5_SLAVE2 | \
                                                  C_PORT_DRV2_CTRL_DRV4_SLAVE1); }  /*!< PWM motor-driver */
#define DRVCFG_GND_UVW()    {IO_PORT_DRV1_CTRL = (C_PORT_DRV1_CTRL_DRV3_L | \
                                                  C_PORT_DRV1_CTRL_DRV2_TRISTATE | \
                                                  C_PORT_DRV1_CTRL_DRV1_TRISTATE | \
                                                  C_PORT_DRV1_CTRL_DRV0_TRISTATE); \
                             IO_PORT_DRV2_CTRL = (C_PORT_DRV2_CTRL_DRV5_L | \
                                                  C_PORT_DRV2_CTRL_DRV4_L); }   /*!< Grounded motor-driver */
#define DRVCFG_SUP_UVW()    {IO_PORT_DRV1_CTRL = (C_PORT_DRV1_CTRL_DRV3_H | \
                                                  C_PORT_DRV1_CTRL_DRV2_TRISTATE | \
                                                  C_PORT_DRV1_CTRL_DRV1_TRISTATE | \
                                                  C_PORT_DRV1_CTRL_DRV0_TRISTATE); \
                             IO_PORT_DRV2_CTRL = (C_PORT_DRV2_CTRL_DRV5_H | \
                                                  C_PORT_DRV2_CTRL_DRV4_H); }   /*!< Supplied motor-driver */
#define DRVCFG_TRI_UVW()    {IO_PORT_DRV1_CTRL = (C_PORT_DRV1_CTRL_DRV3_TRISTATE | \
                                                  C_PORT_DRV1_CTRL_DRV2_TRISTATE | \
                                                  C_PORT_DRV1_CTRL_DRV1_TRISTATE | \
                                                  C_PORT_DRV1_CTRL_DRV0_TRISTATE); \
                             IO_PORT_DRV2_CTRL = (C_PORT_DRV2_CTRL_DRV5_TRISTATE | \
                                                  C_PORT_DRV2_CTRL_DRV4_TRISTATE); }  /*!< Tri-stated motor-driver */
#define DRVCFG_CNFG_UVW(x)  {IO_PORT_DRV1_CTRL = ((x << 12) | \
                                                  C_PORT_DRV1_CTRL_DRV2_TRISTATE | \
                                                  C_PORT_DRV1_CTRL_DRV1_TRISTATE | \
                                                  C_PORT_DRV1_CTRL_DRV0_TRISTATE); \
                             IO_PORT_DRV2_CTRL = (x >> 4); }                    /*!< Configure 3-phase Motor Driver */
#endif
#elif (C_MOTOR_PHASES == 4U)
/* 4-Phase: R, S, T and U */
#if (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_RS_TU)
#define DRVCFG_PWM_RSTU()   {IO_PORT_DRV1_CTRL = (C_PORT_DRV1_CTRL_DRV3_SLAVE3 | \
                                                  C_PORT_DRV1_CTRL_DRV2_MASTER2 | \
                                                  C_PORT_DRV1_CTRL_DRV1_SLAVE1 | \
                                                  C_PORT_DRV1_CTRL_DRV0_MASTER1); \
                             IO_PORT_DRV2_CTRL = (C_PORT_DRV2_CTRL_DRV5_TRISTATE | \
                                                  C_PORT_DRV2_CTRL_DRV4_TRISTATE); }  /*!< PWM motor-driver */
#elif (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_RU_ST)
#define DRVCFG_PWM_RSTU()   {IO_PORT_DRV1_CTRL = (C_PORT_DRV1_CTRL_DRV2_SLAVE3 | \
                                                  C_PORT_DRV1_CTRL_DRV1_MASTER2 | \
                                                  C_PORT_DRV1_CTRL_DRV3_SLAVE1 | \
                                                  C_PORT_DRV1_CTRL_DRV0_MASTER1); \
                             IO_PORT_DRV2_CTRL = (C_PORT_DRV2_CTRL_DRV5_TRISTATE | \
                                                  C_PORT_DRV2_CTRL_DRV4_TRISTATE); }  /*!< PWM motor-driver */
#elif (_SUPPORT_BIPOLAR_MODE == BIPOLAR_MODE_RT_SU)
#define DRVCFG_PWM_RSTU()   {IO_PORT_DRV1_CTRL = (C_PORT_DRV1_CTRL_DRV3_SLAVE3 | \
                                                  C_PORT_DRV1_CTRL_DRV1_MASTER2 | \
                                                  C_PORT_DRV1_CTRL_DRV2_SLAVE1 | \
                                                  C_PORT_DRV1_CTRL_DRV0_MASTER1); \
                             IO_PORT_DRV2_CTRL = (C_PORT_DRV2_CTRL_DRV5_TRISTATE | \
                                                  C_PORT_DRV2_CTRL_DRV4_TRISTATE); }  /*!< PWM motor-driver */
#endif
#define DRVCFG_GND_RSTU()   {IO_PORT_DRV1_CTRL = (C_PORT_DRV1_CTRL_DRV3_L | \
                                                  C_PORT_DRV1_CTRL_DRV2_L | \
                                                  C_PORT_DRV1_CTRL_DRV1_L | \
                                                  C_PORT_DRV1_CTRL_DRV0_L); \
                             IO_PORT_DRV2_CTRL = (C_PORT_DRV2_CTRL_DRV5_TRISTATE | \
                                                  C_PORT_DRV2_CTRL_DRV4_TRISTATE); }  /*!< Grounded motor-driver */
#define DRVCFG_SUP_RSTU()   {IO_PORT_DRV1_CTRL = (C_PORT_DRV1_CTRL_DRV3_H | \
                                                  C_PORT_DRV1_CTRL_DRV2_H | \
                                                  C_PORT_DRV1_CTRL_DRV1_H | \
                                                  C_PORT_DRV1_CTRL_DRV0_H); \
                             IO_PORT_DRV2_CTRL = (C_PORT_DRV2_CTRL_DRV5_TRISTATE | \
                                                  C_PORT_DRV2_CTRL_DRV4_TRISTATE); }  /*!< Supplied motor-driver */
#define DRVCFG_CNFG_RSTU(x) {IO_PORT_DRV1_CTRL = (x); \
                             IO_PORT_DRV2_CTRL = (C_PORT_DRV2_CTRL_DRV5_TRISTATE | \
                                                  C_PORT_DRV2_CTRL_DRV4_TRISTATE); }  /*!< Configure 4-phase Motor Driver */
#elif (C_MOTOR_PHASES == 6U)
/* 6-Phase: R, S, T, U, V and W */
#define DRVCFG_PWM_RSTUVW() {IO_PORT_DRV1_CTRL = (C_PORT_DRV1_CTRL_DRV3_MASTER2 | \
                                                  C_PORT_DRV1_CTRL_DRV2_SLAVE2 | \
                                                  C_PORT_DRV1_CTRL_DRV1_SLAVE1 | \
                                                  C_PORT_DRV1_CTRL_DRV0_MASTER1); \
                             IO_PORT_DRV2_CTRL = (C_PORT_DRV2_CTRL_DRV5_SLAVE4 | \
                                                  C_PORT_DRV2_CTRL_DRV4_SLAVE3); }  /*!< PWM motor-driver */
#define DRVCFG_GND_RSTUVW() {IO_PORT_DRV1_CTRL = (C_PORT_DRV1_CTRL_DRV3_L | \
                                                  C_PORT_DRV1_CTRL_DRV2_L | \
                                                  C_PORT_DRV1_CTRL_DRV1_L | \
                                                  C_PORT_DRV1_CTRL_DRV0_L); \
                             IO_PORT_DRV2_CTRL = (C_PORT_DRV2_CTRL_DRV5_L | \
                                                  C_PORT_DRV2_CTRL_DRV4_L); }   /*!< Grounded motor-driver */
#define DRVCFG_SUP_RSTUVW() {IO_PORT_DRV1_CTRL = (C_PORT_DRV1_CTRL_DRV3_H | \
                                                  C_PORT_DRV1_CTRL_DRV2_H | \
                                                  C_PORT_DRV1_CTRL_DRV1_H | \
                                                  C_PORT_DRV1_CTRL_DRV0_H); \
                             IO_PORT_DRV2_CTRL = (C_PORT_DRV2_CTRL_DRV5_H | \
                                                  C_PORT_DRV2_CTRL_DRV4_H); }   /*!< Supplied motor-driver */
#endif /* (C_MOTOR_PHASES == 6U) */
#elif defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
#if (C_MOTOR_PHASES == 3)
/* Enable the driver and the PWM phase U, V and W */
#define DRVCFG_DIS_UVW()    {IO_PORT_DRV_OUT = \
                                 (IO_PORT_DRV_OUT & \
                                  ~(B_PORT_DRV_OUT_ENABLE_DRV2 | \
                                    B_PORT_DRV_OUT_ENABLE_DRV1 | \
                                    B_PORT_DRV_OUT_ENABLE_DRV0)); }             /*!< Disable motor-driver */
#define DRVCFG_ENA_UVW()    {IO_PORT_DRV_OUT = \
                                 (IO_PORT_DRV_OUT | \
                                  (B_PORT_DRV_OUT_ENABLE_DRV2 | \
                                   B_PORT_DRV_OUT_ENABLE_DRV1 | \
                                   B_PORT_DRV_OUT_ENABLE_DRV0)); }              /*!< Enable motor-driver */
#endif /* (C_MOTOR_PHASES == 3) */
/* Enable the driver and the PWM phase T, U and V */
/* #define DRVCFG_DIS_UVW()    {IO_PORT_DRV_OUT = (IO_PORT_DRV_OUT & ~(B_PORT_DRV_OUT_ENABLE_DRV3 | B_PORT_DRV_OUT_ENABLE_DRV2 | B_PORT_DRV_OUT_ENABLE_DRV1 | B_PORT_DRV_OUT_ENABLE_DRV0)); } */  /*!< Disable motor-driver */
/* #define DRVCFG_ENA_UVW()    {IO_PORT_DRV_OUT = (IO_PORT_DRV_OUT | (B_PORT_DRV_OUT_ENABLE_DRV3 | B_PORT_DRV_OUT_ENABLE_DRV2 | B_PORT_DRV_OUT_ENABLE_DRV1 | B_PORT_DRV_OUT_ENABLE_DRV0)); } */  /*!< Enable motor-driver */
/* Enable the driver and the PWM phase T, U, V and W */
#define DRVCFG_DIS_TUVW()   {IO_PORT_DRV_OUT = \
                                 (IO_PORT_DRV_OUT & \
                                  ~(B_PORT_DRV_OUT_ENABLE_DRV3 | \
                                    B_PORT_DRV_OUT_ENABLE_DRV2 | \
                                    B_PORT_DRV_OUT_ENABLE_DRV1 | \
                                    B_PORT_DRV_OUT_ENABLE_DRV0)); }             /*!< Disable motor-driver */
#define DRVCFG_ENA_TUVW()   {IO_PORT_DRV_OUT = \
                                 (IO_PORT_DRV_OUT | \
                                  (B_PORT_DRV_OUT_ENABLE_DRV3 | \
                                   B_PORT_DRV_OUT_ENABLE_DRV2 | \
                                   B_PORT_DRV_OUT_ENABLE_DRV1 | \
                                   B_PORT_DRV_OUT_ENABLE_DRV0)); }              /*!< Enable motor-driver */

/* 3-Phase: U, V and W or 4-Phase: U, V, W and T */
#if (C_MOTOR_PHASES == 3)
#define DRVCFG_PWM_UVW()    {IO_PORT_DRV_CTRL = \
                                 (C_PORT_DRV_CTRL_DRV3_TRISTATE | \
                                  C_PORT_DRV_CTRL_DRV2_SLAVE2 | \
                                  C_PORT_DRV_CTRL_DRV1_SLAVE1 | \
                                  C_PORT_DRV_CTRL_DRV0_MASTER1); }              /*!< PWM motor-driver */
#define DRVCFG_GND_UVW()    {IO_PORT_DRV_CTRL = \
                                 (C_PORT_DRV_CTRL_DRV3_TRISTATE | \
                                  C_PORT_DRV_CTRL_DRV2_L | \
                                  C_PORT_DRV_CTRL_DRV1_L | \
                                  C_PORT_DRV_CTRL_DRV0_L); }                    /*!< Grounded motor-driver */
#define DRVCFG_SUP_UVW()    {IO_PORT_DRV_CTRL = \
                                 (C_PORT_DRV_CTRL_DRV3_TRISTATE | \
                                  C_PORT_DRV_CTRL_DRV2_H | \
                                  C_PORT_DRV_CTRL_DRV1_H | \
                                  C_PORT_DRV_CTRL_DRV0_H); }                    /*!< Supplied motor-driver */
#define DRVCFG_TRI_UVW()    {IO_PORT_DRV_CTRL = \
                                 (C_PORT_DRV_CTRL_DRV3_TRISTATE | \
                                  C_PORT_DRV_CTRL_DRV2_TRISTATE | \
                                  C_PORT_DRV_CTRL_DRV1_TRISTATE | \
                                  C_PORT_DRV_CTRL_DRV0_TRISTATE); }             /*!< Tri-stated motor-driver */
#define DRVCFG_CNFG_UVW(x)  {IO_PORT_DRV_CTRL = (C_PORT_DRV_CTRL_DRV3_TRISTATE | x); }  /*!< Configure 3-phase Motor Driver */
#endif /* (C_MOTOR_PHASES == 3) */
/* 3-Phase: U, V and T */
/* #define DRVCFG_PWM_UVW()    {IO_PORT_DRV_CTRL = (C_PORT_DRV_CTRL_DRV3_SLAVE2 | C_PORT_DRV_CTRL_DRV2_L | C_PORT_DRV_CTRL_DRV1_SLAVE1 | C_PORT_DRV_CTRL_DRV0_MASTER1); } */  /*!< PWM motor-driver */
/* #define DRVCFG_GND_UVW()    {IO_PORT_DRV_CTRL = (C_PORT_DRV_CTRL_DRV3_L | C_PORT_DRV_CTRL_DRV2_TRISTATE | C_PORT_DRV_CTRL_DRV1_L | C_PORT_DRV_CTRL_DRV0_L); } */    /*!< Grounded motor-driver */
/* #define DRVCFG_SUP_UVW()    {IO_PORT_DRV_CTRL = (C_PORT_DRV_CTRL_DRV3_H | C_PORT_DRV_CTRL_DRV2_TRISTATE | C_PORT_DRV_CTRL_DRV1_H | C_PORT_DRV_CTRL_DRV0_H); } */    /*!< Supplied motor-driver */
/* 4-Phase: U, V, W and T */
#define DRVCFG_PWM_TUVW()   {IO_PORT_DRV_CTRL = \
                                 (C_PORT_DRV_CTRL_DRV3_SLAVE3 | \
                                  C_PORT_DRV_CTRL_DRV2_SLAVE2 | \
                                  C_PORT_DRV_CTRL_DRV1_SLAVE1 | \
                                  C_PORT_DRV_CTRL_DRV0_MASTER1); }              /*!< PWM motor-driver */
#define DRVCFG_GND_TUVW()   {IO_PORT_DRV_CTRL = \
                                 (C_PORT_DRV_CTRL_DRV3_L | \
                                  C_PORT_DRV_CTRL_DRV2_L | \
                                  C_PORT_DRV_CTRL_DRV1_L | \
                                  C_PORT_DRV_CTRL_DRV0_L); }                    /*!< Grounded motor-driver */
#define DRVCFG_SUP_TUVW()   {IO_PORT_DRV_CTRL = \
                                 (C_PORT_DRV_CTRL_DRV3_H | \
                                  C_PORT_DRV_CTRL_DRV2_H | \
                                  C_PORT_DRV_CTRL_DRV1_H | \
                                  C_PORT_DRV_CTRL_DRV0_H); }                    /*!< Supplied motor-driver */
#define DRVCFG_CNFG_TUVW(x) {IO_PORT_DRV_CTRL = x;}                             /*!< Configure motor-driver */
#define DRVCFG_TRI_TUVW()   {IO_PORT_DRV_CTRL = \
                                 (C_PORT_DRV_CTRL_DRV3_TRISTATE | \
                                  C_PORT_DRV_CTRL_DRV2_TRISTATE | \
                                  C_PORT_DRV_CTRL_DRV1_TRISTATE | \
                                  C_PORT_DRV_CTRL_DRV0_TRISTATE); }             /*!< Configure tri-state 4-phase Motor Driver */
#define DRVCFG_PWM_UV_WT()  {IO_PORT_DRV_CTRL = \
                                 (C_PORT_DRV_CTRL_DRV3_SLAVE1 | \
                                  C_PORT_DRV_CTRL_DRV2_SLAVE1 | \
                                  C_PORT_DRV_CTRL_DRV1_MASTER1 | \
                                  C_PORT_DRV_CTRL_DRV0_MASTER1); }              /*!< Configure PWM DC Motor Driver */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#define DRVCFG_DIS()        {IO_PORT_DRV2_PROT = IO_PORT_DRV2_PROT | \
                                                 (B_PORT_DRV2_PROT_EN_DRV | \
                                                  B_PORT_DRV2_PROT_DIS_DRV); }  /*!< Disable 3-phase Motor Driver */
#define DRVCFG_ENA()        {IO_PORT_DRV2_PROT = (IO_PORT_DRV2_PROT & ~B_PORT_DRV2_PROT_DIS_DRV) | \
                                                 B_PORT_DRV2_PROT_EN_DRV; }     /*!< Enable 3-phase Motor Driver */

/* 3-Phase: U, V and W */
#define DRVCFG_PWM_UVW()    {IO_PORT_DRV_CTRL = \
                                 (C_PORT_DRV_CTRL_DRV2_SLAVE2 | \
                                  C_PORT_DRV_CTRL_DRV1_SLAVE1 | \
                                  C_PORT_DRV_CTRL_DRV0_MASTER1); }              /*!< PWM motor-driver */
#define DRVCFG_GND_UVW()    {IO_PORT_DRV_CTRL = \
                                 (C_PORT_DRV_CTRL_DRV2_L | \
                                  C_PORT_DRV_CTRL_DRV1_L | \
                                  C_PORT_DRV_CTRL_DRV0_L); }                    /*!< Grounded motor-driver */
#define DRVCFG_SUP_UVW()    {IO_PORT_DRV_CTRL = \
                                 (C_PORT_DRV_CTRL_DRV2_H | \
                                  C_PORT_DRV_CTRL_DRV1_H | \
                                  C_PORT_DRV_CTRL_DRV0_H); }                    /*!< Supplied motor-driver */
#define DRVCFG_CNFG_UVW(x)  {IO_PORT_DRV_CTRL = (x); }                          /*!< Configure 3-phase Motor Driver */
#define DRVCFG_TRI_UVW()    {IO_PORT_DRV_CTRL = \
                                 (C_PORT_DRV_CTRL_DRV2_TRISTATE | \
                                  C_PORT_DRV_CTRL_DRV1_TRISTATE | \
                                  C_PORT_DRV_CTRL_DRV0_TRISTATE); }             /*!< Configure tri-state 4-phase Motor Driver */

/* 2-Phase: U and V */
#define DRVCFG_PWM_UV()    {IO_PORT_DRV_CTRL = \
                                (C_PORT_DRV_CTRL_DRV2_TRISTATE | \
                                 C_PORT_DRV_CTRL_DRV1_SLAVE1 | \
                                 C_PORT_DRV_CTRL_DRV0_MASTER1); }               /*!< PWM motor-driver */
#define DRVCFG_GND_UV()    {IO_PORT_DRV_CTRL = \
                                (C_PORT_DRV_CTRL_DRV2_TRISTATE | \
                                 C_PORT_DRV_CTRL_DRV1_L | \
                                 C_PORT_DRV_CTRL_DRV0_L); }                     /*!< Grounded motor-driver */
#define DRVCFG_SUP_UV()    {IO_PORT_DRV_CTRL = \
                                (C_PORT_DRV_CTRL_DRV2_TRISTATE | \
                                 C_PORT_DRV_CTRL_DRV1_H | \
                                 C_PORT_DRV_CTRL_DRV0_H); }                     /*!< Supplied motor-driver */
#endif

#define C_DELAY_5US     (uint16_t)((5UL * FPLL) / C_DELAY_CONST)                /*!<   5us delay */
#define C_DELAY_10US    (uint16_t)((10UL * FPLL) / C_DELAY_CONST)               /*!<  10us delay */

/* MLX81330/32 Motor-Driver FETs (P+N-FET) */
#if defined (__MLX81160__)
#if (_SUPPORT_EVB == MLX81160EVB_3P_3N)
#define C_FETS_RTOT                 10U                                         /*!< External motor driver half-bridge resistance [10mR] (MMP240515-1) */
#define C_EXT_FET_DEADTIME      ((uint16_t)(0.25 * PWM_TIMER_CLK / 1000000UL))  /*!< External FET Dead-time: 250ns */
#else  /* (_SUPPORT_EVB == MLX81160EVB_3P_3N) */
#define C_FETS_RTOT                 1400U                                       /*!< Internal motor driver half-bridge resistance [10mR] (MMP240515-1) */
#endif /* (_SUPPORT_EVB == MLX81160EVB_3P_3N) */
#elif defined (__MLX81330__)
#define C_FETS_RTOT                 300U                                        /*!< Internal motor driver half-bridge resistance [10mR] (MMP240515-1) */
#elif defined (__MLX81332__) || defined (__MLX81334__)
#define C_FETS_RTOT                 80U                                         /*!< FET RDS-ON total [10mR] (MMP240515-1) */
#elif defined (__MLX81339__)
#define C_FETS_RTOT                 30U                                         /*!< FET RDS-ON total [10mR] (MMP240515-1) */ /*MMP39*/
#elif defined (__MLX81340__) ||  defined (__MLX81344__) || defined (__MLX81346__)
#define C_FETS_RTOT                 10U                                         /*!< FET RDS-ON total [10mR] (MMP240515-1) */
#elif defined (__MLX81350__)
#define C_FETS_RTOT                 300U                                        /*!< Internal motor driver half-bridge resistance [10mR] (MMP240515-1) */ /*MMP50*/
#endif

#if (C_MOTOR_PHASES == 1)
#define C_NR_OF_FULLSTEPS           2U                                          /*!< Number of full-steps per electric rotation (SCF) */
#elif (C_MOTOR_PHASES == 3)
#define C_NR_OF_FULLSTEPS           6U                                          /*!< Number of full-steps per electric rotation */
#define C_NR_OF_HALFSTEPS           12U                                         /*!< Number of full-steps per electric rotation */
#else
#define C_NR_OF_FULLSTEPS           4U                                          /*!< Number of full-steps per electric rotation */
#define C_NR_OF_HALFSTEPS           8U                                          /*!< Number of half-steps per electric rotation */
#endif

#define C_ZERO_POS_OFFSET           10000U                                      /*!< Actuator Position Offset */

#define UNKNOWN_STEP                0xFFFFU                                     /*!< Unknown step Index */

/* MMP191216-1: Don't use driver-slew-rate '7'; max./fastest slew-rate is '6'; Use 0.25us units, or make sure after rounding the minimum delay is guaranteed */
#if defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
#if defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81350__)
#define C_PWM_DCORR_8   ((uint16_t)(((2.25 * PWM_TIMER_CLK) + 500000UL) / 1000000UL))  /*!< PWM to Phase-out delay-correction: 2.25us (Slew-rate setting: 8, slowest) */
#define C_PWM_MIN_DC_8  ((uint16_t)(((2.50 * PWM_TIMER_CLK) + 500000UL) / 1000000UL))  /*!< PWM Minimum pulse: 2.50 us */
#define C_PWM_DCORR_0   ((uint16_t)(((1.25 * PWM_TIMER_CLK) + 500000UL) / 1000000UL))  /*!< PWM to Phase-out delay-correction: 1.25us (Slew-rate setting: 0, medium) */
#define C_PWM_MIN_DC_0  ((uint16_t)(((1.50 * PWM_TIMER_CLK) + 500000UL) / 1000000UL))  /*!< PWM Minimum pulse: 1.50 us */
#define C_PWM_DCORR_6   ((uint16_t)(((0.80 * PWM_TIMER_CLK) + 500000UL) / 1000000UL))  /*!< PWM to Phase-out delay-correction: 0.78us (Slew-rate setting: 6, fastest) */
#define C_PWM_MIN_DC_6  ((uint16_t)(((1.00 * PWM_TIMER_CLK) + 500000UL) / 1000000UL))  /*!< PWM Minimum pulse: 1.00 us */
#elif defined (__MLX81332__) || defined (__MLX81334__)
#define C_PWM_DCORR_8   ((uint16_t)(((2.29 * PWM_TIMER_CLK) + 500000UL) / 1000000UL))  /*!< PWM to Phase-out delay-correction: 2.29us (Slew-rate setting: 8, slowest) */
#define C_PWM_MIN_DC_8  ((uint16_t)(((2.65 * PWM_TIMER_CLK) + 500000UL) / 1000000UL))  /*!< PWM Minimum pulse: 2.65 us */
#define C_PWM_DCORR_0   ((uint16_t)(((1.13 * PWM_TIMER_CLK) + 500000UL) / 1000000UL))  /*!< PWM to Phase-out delay-correction: 1.13us (Slew-rate setting: 0, medium) */
#define C_PWM_MIN_DC_0  ((uint16_t)(((1.38 * PWM_TIMER_CLK) + 500000UL) / 1000000UL))  /*!< PWM Minimum pulse: 1.38 us */
#define C_PWM_DCORR_6   ((uint16_t)(((0.08 * PWM_TIMER_CLK) + 500000UL) / 1000000UL))  /*!< PWM to Phase-out delay-correction: 0.078us (Slew-rate setting: 6, fastest) */
#define C_PWM_MIN_DC_6  ((uint16_t)(((0.80 * PWM_TIMER_CLK) + 500000UL) / 1000000UL))  /*!< PWM Minimum pulse: 1.00 us */
#elif defined (__MLX81339__)  /*MMP39: Check */
#define C_PWM_DCORR_8   ((uint16_t)(((2.29 * PWM_TIMER_CLK) + 500000UL) / 1000000UL))  /*!< PWM to Phase-out delay-correction: 2.29us (Slew-rate setting: 8, slowest) */
#define C_PWM_MIN_DC_8  ((uint16_t)(((2.65 * PWM_TIMER_CLK) + 500000UL) / 1000000UL))  /*!< PWM Minimum pulse: 2.65 us */
#define C_PWM_DCORR_0   ((uint16_t)(((1.13 * PWM_TIMER_CLK) + 500000UL) / 1000000UL))  /*!< PWM to Phase-out delay-correction: 1.13us (Slew-rate setting: 0, medium) */
#define C_PWM_MIN_DC_0  ((uint16_t)(((1.38 * PWM_TIMER_CLK) + 500000UL) / 1000000UL))  /*!< PWM Minimum pulse: 1.38 us */
#define C_PWM_DCORR_6   ((uint16_t)(((0.08 * PWM_TIMER_CLK) + 500000UL) / 1000000UL))  /*!< PWM to Phase-out delay-correction: 0.078us (Slew-rate setting: 6, fastest) */
#define C_PWM_MIN_DC_6  ((uint16_t)(((0.80 * PWM_TIMER_CLK) + 500000UL) / 1000000UL))  /*!< PWM Minimum pulse: 1.00 us */
#endif
#if (_APP_ZWICKAU != FALSE)
#define C_DRV_SLWRT                 8U                                          /*!< Driver Slew-rate setting: Slowest */
#define C_CP_SLWRT_DRV              8U                                          /*!< CP Slew-rate driver setting: Slowest (MMP221027-1) */
#define C_PWM_DCORR                 C_PWM_DCORR_8                               /*!< PWM-to-Driver delay correction */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
#define C_PWM_MIN_DC                (C_PWM_MIN_DC_8 >> 1)                       /*!< Minimum PWM pulse width */
#else  /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
#define C_PWM_MIN_DC                C_PWM_MIN_DC_8                              /*!< Minimum PWM pulse width */
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
#else  /* (_APP_ZWICKAU != FALSE) */
#define C_DRV_SLWRT                 6U                                          /*!< Driver Slew-rate setting: Fast */
#define C_CP_SLWRT_DRV              0U                                          /*!< CP Slew-rate driver setting: Normal (Default) (MMP221027-1) */
#define C_PWM_DCORR                 C_PWM_DCORR_6                               /*!< PWM-to-Driver delay correction */
#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) || \
    ((_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) && (C_NR_OF_DC_MOTORS == 1))
#define C_PWM_MIN_DC                0U                                          /*!< Minimum PWM pulse width (up to 100%) */
#elif (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
#define C_PWM_MIN_DC                (C_PWM_MIN_DC_6 >> 1)                       /*!< Minimum PWM pulse width */
#else  /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
#define C_PWM_MIN_DC                C_PWM_MIN_DC_6                              /*!< Minimum PWM pulse width */
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
#endif /* (_APP_ZWICKAU != FALSE) */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#if (_SUPPORT_EVB == MLX8134xEVB_100W) || (_SUPPORT_EVB == MLX81346EVB_200W) || (_SUPPORT_EVB == MLX81346EVB_ITFAN)
#define C_PWM_DCORR     ((uint16_t)((0.25 * PWM_TIMER_CLK) / 1000000UL))        /*!< PWM to Phase-out delay-correction: 0.25us  */
#define C_PWM_MIN_DC    ((uint16_t)((0.375 * PWM_TIMER_CLK) / 1000000UL))       /*!< PWM Minimum pulse: 0.375 us */
#elif (_SUPPORT_EVB == MLX8134xEVB_400W) || (_SUPPORT_EVB == MLX81346EVB_600W)
#define C_PWM_DCORR     ((uint16_t)((0.25 * PWM_TIMER_CLK) / 1000000UL))        /*!< PWM to Phase-out delay-correction: 0.9us  */
#define C_PWM_MIN_DC    ((uint16_t)((0.60 * PWM_TIMER_CLK) / 1000000UL))        /*!< PWM Minimum pulse: 1.20 us */
#endif /* _SUPPORT_EVB */
#endif /* __MLX813xx__ */

#define   Q15(A)                (int16_t)((A) * 32768)                          /*!< Define for Q15 */
#define C_INV_SQRT3             Q15(0.57735026918962576450914878050196)         /*!< 1/SQRT(3) */

#define C_PI_TICKS_STABILISE        (32U * PI_TICKS_PER_MILLISECOND)            /*!< Stabilisation-time; Max. 255 */

/* Motor running average filter length */
#define C_MOVAVG_SZ                 (1U << C_MOVAVG_SSZ)                        /*!< Two Full-steps or 8 micro-steps; ((C_MOTOR_STEP_PER_PHASE * C_MOTOR_PHASES) * C_MOTOR_POLE_PAIRS) */
#if (_DEBUG_MOTOR_CURRENT_FLT != FALSE)
#if (C_MOTOR_PHASES == 3)
/* Important Note: If _DEBUG_MOTOR_CURRENT_FLT supported, buffer-size must be n*C_UART_DEBUG_PACKET_SZ
 * The following C_UART_DEBUG_PACKET_SZ are supported: 2, 3, 4, 6, 8, 12, 16, 24, 32, 48
 */
#define C_DEBUG_BUF_SZ              (16U * 48U)                                 /*!< 16 electric rotations of 48 uSteps */
#else  /* (C_MOTOR_PHASES == 3) */
#define C_DEBUG_BUF_SZ              (24U * 32U)                                 /*!< 24 electric rotations of 32 uSteps */
#endif /* (C_MOTOR_PHASES == 3) */
#endif /* (_DEBUG_MOTOR_CURRENT_FLT != FALSE) */

/*! Motor Start-up mode */
typedef enum __attribute__((packed))
{
    E_MSM_STOP = ((uint8_t) 0x00U),                                             /*!< Stop mode */
    E_MSM_ACCELERATE = ((uint8_t) 0x01U),                                       /*!< Accelerate speed mode */
    E_MSM_DECELERATE = ((uint8_t) 0x02U),                                       /*!< Decelerate speed mode */
    E_MSM_CONSTANT = ((uint8_t) 0x03U),                                         /*!< Constant speed mode */
    E_MSM_SPEED_MODE_MASK = ((uint8_t) 0x03U),                                  /*!< Speed-mode mask */
    E_MSM_ALIGN = ((uint8_t) 0x04U),                                            /*!< Alignment */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
    E_MSM_2PWM = ((uint8_t) 0x05U),                                             /*!< Two-PWM mode */
    E_MSM_3PWM = ((uint8_t) 0x06U),                                             /*!< Three-PWM mode */
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
#if (_SUPPORT_BRAKING != FALSE)
    E_MSM_BRAKING = ((uint8_t) 0x07U),                                          /*!< Braking mode */
#endif /* (_SUPPORT_BRAKING != FALSE) */
#if (_SUPPORT_STALLDET_BZC != FALSE)
    E_MSM_BEMF = ((uint8_t) 0x08U),                                             /*!< BEMF mode */
#if (_SUPPORT_FOC_MODE == FOC_MODE_NONE)
    E_MSM_MODE_MASK = ((uint8_t) 0x08U),                                        /*!< Mode-mask */
#endif /* (_SUPPORT_FOC_MODE == FOC_MODE_NONE) */
#endif /* (_SUPPORT_STALLDET_BZC != FALSE) */
    E_MSM_STEPPER = ((uint8_t) 0x00U),                                          /*!< Stepper mode */
#if (_SUPPORT_CDI != FALSE)
    E_MSM_CDI = ((uint8_t) 0x40U),                                              /*!< CDI mode */
#endif /* (_SUPPORT_CDI != FALSE) */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
    E_MSM_FOC = ((uint8_t) 0x80U),                                              /*!< FOC-mode */
    E_MSM_MODE_MASK = ((uint8_t) 0x80U),                                        /*!< Mode-mask */
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
    E_MSM_STEPPER_A = ((uint8_t)E_MSM_STEPPER | (uint8_t)E_MSM_ACCELERATE),     /*!< Stepper-Acceleration */
    E_MSM_STEPPER_D = ((uint8_t)E_MSM_STEPPER | (uint8_t)E_MSM_DECELERATE),     /*!< Stepper-Deceleration */
    E_MSM_STEPPER_C = ((uint8_t)E_MSM_STEPPER | (uint8_t)E_MSM_CONSTANT),       /*!< Stepper-Constant */
    E_MSM_STEPPER_ALIGN = ((uint8_t)E_MSM_STEPPER | (uint8_t)E_MSM_ALIGN),      /*!< Stepper-Alignment */
#if (_SUPPORT_STALLDET_BZC != FALSE)
    E_MSM_BEMF_A = ((uint8_t)E_MSM_BEMF | (uint8_t)E_MSM_ACCELERATE),           /* BEMF-Acceleration */
    E_MSM_BEMF_D = ((uint8_t)E_MSM_BEMF | (uint8_t)E_MSM_DECELERATE),           /* BEMF-Deceleration */
    E_MSM_BEMF_C = ((uint8_t)E_MSM_BEMF | (uint8_t)E_MSM_CONSTANT),             /* BEMF-Constant */
#endif /* (_SUPPORT_STALLDET_BZC != FALSE) */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
    E_MSM_FOC_CL_STARTUP = ((uint8_t)E_MSM_FOC | (uint8_t)E_MSM_ACCELERATE),    /*!< FOC Closed-Loop Startup */
    E_MSM_FOC_2PWM = ((uint8_t)E_MSM_FOC | (uint8_t)E_MSM_2PWM),                /*!< FOC 2 PWM mode */
    E_MSM_FOC_3PWM = ((uint8_t)E_MSM_FOC | (uint8_t)E_MSM_3PWM),                /*!< FOC 3 PWM mode */
#if (_SUPPORT_BRAKING != FALSE)
    E_MSM_FOC_BRAKING = ((uint8_t)E_MSM_FOC | (uint8_t)E_MSM_BRAKING),          /*!< FOC Braking */
#endif /* (_SUPPORT_BRAKING != FALSE) */
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
} E_MOTOR_STARTUP_MODE_t;

/*! Alignment steps */
typedef enum __attribute__((packed))
{
    E_ALIGN_STEP_SHORT = 0U,                                                    /*!< Short alignment */
    E_ALIGN_STEP_1,                                                             /*!< Alignment pre-step */
    E_ALIGN_STEP_2A,                                                            /*!< Alignment at next Full-step, with increased alignment current */
    E_ALIGN_STEP_2B,                                                            /*!< Decreasing alignment current */
    E_ALIGN_STEP_2C,                                                            /*!< Stable alignment current */
    E_ALIGN_STEP_DONE                                                           /*!< Alignment is done */
} E_ALIGNMENT_STEPS_t;

#define C_ALIGN_SOFT_PERIOD         C_MICROSTEP_PER_FULLSTEP                    /*!< Soft alignment period (SRP successful) */
#if (_SUPPORT_SINCOS_TABLE_SZ == 256)
#define C_ALIGN_STEP_1_PERIOD       (1U * 64U)                                  /*!< Alignment Period Step 1 (First Position) */
#define C_ALIGN_STEP_2A_PERIOD      (1U * 64U)                                  /*!< Alignment Period Step 2a (Second Position) */
#define C_ALIGN_STEP_2B_PERIOD      (2U * 64U)                                  /*!< Alignment Period Step 2b (Current decrease) */
#define C_ALIGN_STEP_2C_PERIOD      (4U * 64U)                                  /*!< Alignment Period Step 2c (Holding current) */
#elif (_SUPPORT_SINCOS_TABLE_SZ == 192)
#define C_ALIGN_STEP_1_PERIOD       (1U * 48U)                                  /*!< Alignment Period Step 1 (First Position) */
#define C_ALIGN_STEP_2A_PERIOD      (1U * 48U)                                  /*!< Alignment Period Step 2a (Second Position) */
#define C_ALIGN_STEP_2B_PERIOD      (2U * 48U)                                  /*!< Alignment Period Step 2b (Current decrease) */
#define C_ALIGN_STEP_2C_PERIOD      (4U * 48U)                                  /*!< Alignment Period Step 2c (Holding current) */
#else
#define C_ALIGN_STEP_1_PERIOD       (1U * 64U)                                  /*!< Alignment Period Step 1 (First Position) */
#define C_ALIGN_STEP_2A_PERIOD      (1U * 64U)                                  /*!< Alignment Period Step 2a (Second Position) */
#define C_ALIGN_STEP_2B_PERIOD      (2U * 64U)                                  /*!< Alignment Period Step 2b (Current decrease) */
#define C_ALIGN_STEP_2C_PERIOD      (4U * 64U)                                  /*!< Alignment Period Step 2c (Holding current) */
#endif

/*! Motor Stop Mode */
typedef enum __attribute__((packed))
{
    C_STOP_RAMPDOWN = 0U,                                                       /*!< Stop actuator with ramp-down speed */
    C_STOP_IMMEDIATE,                                                           /*!< Stop actuator immediately, without start-up delay */
    C_STOP_EMERGENCY,                                                           /*!< Stop actuator immediately, with start-up delay */
#if (_SUPPORT_STALL_REVERSE != FALSE)
    C_STOP_REVERSE,                                                             /*!< Stop actuator and reverse some step to release gear-stress */
#endif /* (_SUPPORT_STALL_REVERSE != FALSE) */
#if (_SUPPORT_FAST_STOP != FALSE)
    C_STOP_FAST_STOP,                                                           /*!< Stop actuator Fast (Safety), active Braking */
#endif /* (_SUPPORT_FAST_STOP != FALSE) */
    C_STOP_WO_HOLDING                                                           /*!< Stop actuator w/o holding current */
} MOTOR_STOP_MODE;

#if (_SUPPORT_STALLDET_BZC != FALSE)
/* [BEMF] Zero-crossing detector state */
#define ZC_RESET    0                                                           /*!< Zero-cross detector: Reset */
#define ZC_INIT     (1U << 0)                                                   /*!< Zero-cross detector: Initialised */
#define ZC_START    (1U << 1)                                                   /*!< Zero-cross detector: Started */
#define ZC_FOUND    (1U << 2)                                                   /*!< Zero-cross detector: Found */
#define ZC_ERROR    (1U << 3)                                                   /*!< Zero-cross detector: Error */
#endif /* (_SUPPORT_STALLDET_BZC != FALSE) */

#if (_SUPPORT_TACHO_OUT != FALSE)
/*! Motor tacho mode */
typedef enum __attribute__((packed))
{
    C_TACHO_NONE = 0U,                                                          /**< No Tacho output */
    C_TACHO_60DEG_ELECTRIC,                                                     /**< Tacho output toggles every 60 degrees of an electric rotation */
    C_TACHO_180DEG_ELECTRIC,                                                    /**< Tacho output toggles every 180 degrees of an electric rotation */
    C_TACHO_180DEG_MECHANICAL                                                   /**< Tacho output toggles every 180 degrees of a mechanical rotation */
} TACHO_MODES;
#endif /* (_SUPPORT_TACHO_OUT != FALSE) */

#if (_SUPPORT_MOTOR_SELFTEST != FALSE)
/*! ADC structure for Motor-Driver Self-Test */
typedef struct _ADC_SELFTEST
{
    uint16_t MotorDriverVoltage;                                                /**<  50%: Unfiltered Motor Driver Voltage */
    uint16_t PhaseVoltage;                                                      /**< 100%: Unfiltered Motor Driver Current */
    uint16_t PhaseU_Voltage50;                                                  /**<  50%: Phase-U High-voltage */
    uint16_t PhaseU_Voltage100;                                                 /**< 100%: Phase-U Low-voltage */
    uint16_t PhaseV_Voltage50;                                                  /**<  50%: Phase-V High-voltage */
    uint16_t PhaseV_Voltage100;                                                 /**< 100%: Phase-V Low-voltage */
    uint16_t PhaseW_Voltage50;                                                  /**<  50%: Phase-W High-voltage */
    uint16_t PhaseW_Voltage100;                                                 /**< 100%: Phase-W Low-voltage */
    uint16_t MotorDriverCurrent50;                                              /**<  50%: Unfiltered Motor Driver Current */
    uint16_t MotorDriverCurrent100;                                             /**< 100%: Unfiltered Motor Driver Current */
    uint16_t u16CRC;                                                            /**< CRC Place holder */
} ADC_SELFTEST;

/*! ADC structure for Motor-Driver Pre-Test */
typedef struct _ADC_PRE_TEST
{
    uint16_t MotorDriverVoltageA;                                               /**<  16%: Motor Driver Voltage (Dummy) */
    uint16_t PhaseU_Voltage;                                                    /**<  33%: Phase-U voltage */
    uint16_t PhaseV_Voltage;                                                    /**<  50%: Phase-V voltage */
    uint16_t PhaseW_Voltage;                                                    /**<  67%: Phase-W voltage */
    uint16_t MotorDriverVoltageB;                                               /**<  100%: Motor Driver Voltage */
    uint16_t u16CRC;                                                            /**< CRC Place holder */
} ADC_PRE_TEST;

/*! ADC structure for Motor-Driver Pin-Test */
typedef struct _ADC_PIN_TEST
{
    uint16_t Temperature;                                                       /**<  16%: Temperature (Dummy) */
    uint16_t MotorDriverVoltage;                                                /**<  33%: Motor Driver Voltage */
    uint16_t PhaseVoltage;                                                      /**<  50%+D: Phase voltage */
    uint16_t u16CRC;                                                            /**< CRC Place holder */
} ADC_PIN_TEST;
#endif /* (_SUPPORT_MOTOR_SELFTEST != FALSE) */

#define C_DELTA_POS_MOTOR_STOP      1U                                          /*!< Number of full-steps from target-position to stop actuator (MMP161031-1) */

#ifndef C_STATUS_SPEED_STOP                                                     /* Fix to set global speed status when no LIN communication is used */
#define C_STATUS_SPEED_STOP         0U                                          /*!< Actual speed: Stop */
#endif /* C_STATUS_SPEED_STOP */

#if (_SUPPORT_PWM_SYNC == FALSE)
/*! Motor PWM (LT) Structure for none-synchronised PWM update to PWM Period End */
typedef struct _MOTOR_PWM_t
{
#if (_SUPPORT_VOLTAGE_SHAPE != VOLTAGE_SHAPE_DSVM_ALT)
#if (_SUPPORT_PWM_MODE == BIPOLAR_TRIPHASE_TWOPWM_INDEPENDENT_GND) || \
    ((_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_INDEPENDENT_GND) && (_SUPPORT_FOC_MODE == FOC_MODE_NONE))
    uint16_t u16PwmCtrl;                                                        /*!< PWM Control */
#endif
#else  /* (_SUPPORT_VOLTAGE_SHAPE != VOLTAGE_SHAPE_DSVM_ALT) */
    uint16_t u16PwmCtrlM1;                                                      /*!< PWM Control Master #1 */
    uint16_t u16PwmCtrlS1;                                                      /*!< PWM Control Slave #1 */
    uint16_t u16PwmCtrlS2;                                                      /*!< PWM Control Slave #2 */
#endif /* (_SUPPORT_VOLTAGE_SHAPE != VOLTAGE_SHAPE_DSVM_ALT) */
    uint16_t u16PwmMaster1;                                                     /*!< Master PWM LT value */
    uint16_t u16PwmSlave1;                                                      /*!< Slave PWM #1 LT value */
#if (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM_BIPOLAR)
    uint16_t u16PwmSlave2;                                                      /*!< Slave PWM #2 LT value */
#if (C_MOTOR_PHASES > 3)
    uint16_t u16PwmSlave3;                                                      /*!< Slave PWM #3 LT value */
    /* uint16_t u16PwmMaster2; */
#endif /* (C_MOTOR_PHASES > 3) */
#endif /* (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM_BIPOLAR) */
} MOTOR_PWM_t;
#endif /* (_SUPPORT_PWM_SYNC == FALSE) */

static inline int16_t* p_SinDirectLutInlined_2_5k(uint16_t theta)
{
    int16_t *pSinLUT;

    __asm__ __volatile__ (
        "lsr %w1, #2\n\t"
        "lsr %w1, #2\n\t"
        "lsr %w1, #2\n\t"
        "lsl %w1\n\t"
        "add %w1, #(_fm_SinCosTable_2_5k+2)\n\t"
        : "=x" (pSinLUT)
        : "x" (theta)
        );
    return ( pSinLUT );
}

/*!*************************************************************************** */
/*                           GLOBAL VARIABLES                                  */
/* *************************************************************************** */
#pragma space dp                                                                /* __TINY_SECTION__ */
extern uint16_t l_u16MotorMicroStepsPerElecRotation;                            /*!< Number of (micro-) steps per electric rotation */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
extern uint16_t l_u16CommutTimerPeriod;                                         /*!< Commutation timer period */
#if (C_MOTOR_PHASES != 1)
extern uint16_t l_u16mR_AT;                                                     /*!< Coil resistance at ambient temperature */
extern uint16_t l_u16mZ;                                                        /*!< Coil Inductance */
extern uint16_t l_u16Ipk;                                                       /*!< Ipk value */
#if (_SUPPORT_PID_U32 == FALSE)
extern uint16_t g_u16ActualMotorSpeedRPMe;                                      /*!< Actual motor-speed (electric-rotations) */
#else  /* (_SUPPORT_PID_U32 == FALSE) */
extern uint32_t g_u32ActualMotorSpeedRPMe;                                      /*!< Actual motor-speed (electric-rotations) */
#endif /* (_SUPPORT_PID_U32 == FALSE) */
#endif /* (C_MOTOR_PHASES != 1) */
#if (_SUPPORT_VARIABLE_PWM != FALSE) && (_SUPPORT_VAR_PWM_MODE != C_VAR_PWM_MODE_OFF)
extern uint8_t g_u8TriplePWM;                                                   /*!< FALSE: 2-coil/phase current; TRUE: 3-coil/phase current (MMP220815-1) */
#endif /* (_SUPPORT_VARIABLE_PWM != FALSE) && (_SUPPORT_VAR_PWM_MODE != C_VAR_PWM_MODE_OFF) */
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
#if (_SUPPORT_STALLDET_BZC != FALSE)
extern volatile uint8_t g_e8ZcDetectorState;                                    /*!< [BEMF] Zero-crossing detector state */
#endif /* (_SUPPORT_STALLDET_BZC != FALSE) */
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ)
extern uint16_t l_u16VoltageAngle;                                              /*!< Vector Angle (0..2pi :: 0..0xFFFF) */
extern volatile int16_t l_i16Valpha;                                            /*!< Valpha (Inverse Park Transformation) */
extern volatile int16_t l_i16Vbeta;                                             /*!< Vbeta (Inverse Park Transformation) */
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) */
#pragma space none                                                              /* __TINY_SECTION__ */

#if (_SUPPORT_CURRSPIKE_POSITION != FALSE)
extern uint16_t g_u16TargetSpikePos __attribute__ ((dp, section(".dp.noinit")));  /*!< [DC] (Motor-driver) Target motor-rotor position [Left] */
#elif (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE)
extern uint16_t g_u16TargetHallLatchPos __attribute__ ((section(".dp.noinit")));  /*!< [DC] (Motor-driver) Target motor-rotor position [left] */
#endif
#if (_SUPPORT_POTI != FALSE) && (_SUPPORT_POTI_MODE != C_POTI_MODE_NONE)
extern volatile uint16_t g_u16ActualPotiPos __attribute__ ((section(".dp.noinit")));  /*!< (Motor-driver) Actual motor-rotor position */
#if (_SUPPORT_POTI_MODE == C_POTI_MODE_CLOSED_LOOP)
extern uint16_t l_u16TargetPotiPos __attribute__ ((section(".dp.noinit")));     /*!< (Motor-driver) Target motor-rotor position */
#endif /* (_SUPPORT_POTI_MODE == C_POTI_MODE_CLOSED_LOOP) && (_SUPPORT_POTI_MODE != C_POTI_MODE_NONE) */
#endif /* (_SUPPORT_POTI != FALSE) */

#pragma space nodp                                                              /* __NEAR_SECTION__ */
#if (_SUPPORT_PWM_SYNC == FALSE)
extern MOTOR_PWM_t MotorPwm;                                                    /*!< PWM LT Values */
#endif /* (_SUPPORT_PWM_SYNC == FALSE) */
extern uint16_t g_u16NrOfMicroStepsPerFullStep;                                 /*!< Number of Micro-steps per full-step */
#if (C_MOTOR_PHASES != 1) || (_SUPPORT_HALL_LATCH == FALSE)
extern uint16_t g_u16ForcedSpeedRPM;                                            /*!< Forced (Feed Forward) Speed [RPM] */
#endif /* (C_MOTOR_PHASES != 1) || (_SUPPORT_HALL_LATCH == FALSE) */
#if (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL > 1)
extern volatile uint8_t g_u8ZcHallFound;                                        /*!< Zero-crossing Hall-Latch Found flag */
#if (_SUPPORT_MICRO_STEP_COMMUTATION != FALSE)
extern uint8_t g_u8HallMicroSteps;                                              /*!< Number of micro-steps between hall-latch edges */
#endif /* (_SUPPORT_MICRO_STEP_COMMUTATION != FALSE) */
#endif /*(_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL > 1) */
#if (_SUPPORT_HALL_LATCH_DIAG != FALSE)
extern uint8_t l_au8MotorMicroStep[C_MOTOR_PHASES * C_MOTOR_STEP_PER_PHASE * C_MICROSTEP_PER_FULLSTEP];
#endif /* (_SUPPORT_HALL_LATCH_DIAG != FALSE) */
#if (_DEBUG_MOTOR_CURRENT_FLT != FALSE)
extern uint8_t g_au8DebugBuf[];                                                 /*!< Raw (unfiltered) motor current measurement */
extern uint16_t g_u16DebugBufWrIdx;                                             /*!< Motor current measurement index */
#if (_DEBUG_MCUR_SUB_SAMPLED != FALSE)
extern uint16_t g_u16SubSamplingIdx;                                            /*!< Modulo index */
#endif /* (_DEBUG_MCUR_SUB_SAMPLED != FALSE) */
#if (UART_COMM != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_SCOPE) && (_SUPPORT_UART_SCOPE_MODE == CH12_SCOPE)
extern uint16_t g_u16DebufBufUartTxIdx;                                         /*!< Motor current Transmission index */
#endif /* (UART_COMM != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_SCOPE) && (_SUPPORT_UART_SCOPE_MODE == CH12_SCOPE) */
#endif /* (_DEBUG_MOTOR_CURRENT_FLT != FALSE) */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
extern uint16_t g_u16ActualCommutTimerPeriod;
#if (C_MOTOR_PHASES != 1) || (_SUPPORT_HALL_LATCH == FALSE)
extern uint16_t l_u16AccelerationConst;                                         /*!< Acceleration (and deceleration) constant */
#endif /* (C_MOTOR_PHASES != 1) || (_SUPPORT_HALL_LATCH == FALSE) */
extern uint32_t l_u32MotorCurrentLPF;                                           /*!< Motor current LPF internal variable */
#if ((_SUPPORT_CLOSED_LOOP_STARTUP == FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE)) && (C_MOTOR_PHASES != 1)
extern uint16_t l_u16SpeedUpdateAcc;                                            /*!< Speed acceleration update (micro-steps) */
extern uint16_t l_u16SpeedUpdateDec;                                            /*!< Speed deceleration update (micro-steps) */
#endif /* ((_SUPPORT_CLOSED_LOOP_STARTUP == FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE)) && (C_MOTOR_PHASES != 1) */
#if (PWM_LIMIT == PWM_LIMIT_BY_PWM)
extern uint16_t l_u16MaxPwmRatio;                                               /*!< Maximum Motor-PWM ratio (MMP181114-2) */
#if (_SUPPORT_BOOST_MODE == BOOST_MODE_WAVEFORM)
extern uint16_t l_u16MaxPwmCorrRatio;                                           /*!< Max PWM Correction Ratio */
extern uint16_t l_u16MaxPwmCorrRatioBoost;                                      /*!< Max PWM Correction Ratio Boost */
#endif /* (_SUPPORT_BOOST_MODE == BOOST_MODE_WAVEFORM) */
#endif /* (PWM_LIMIT == PWM_LIMIT_BY_PWM) */
#if (_SUPPORT_CLOSED_LOOP_STARTUP != FALSE)
extern uint16_t l_u16StartrupThrshldSpeedRPMe;                                  /*!< FOC Close-loop Start-up max. speed threshold */
#endif /* (_SUPPORT_CLOSED_LOOP_STARTUP != FALSE) */
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
#if (_SUPPORT_TACHO_OUT != FALSE)
extern uint16_t l_u16TachoThreshold;                                            /*!< Tacho threshold (disabled) */
extern uint16_t l_u16TachoCount;                                                /*!< Tacho count */
#endif /* (_SUPPORT_TACHO_OUT != FALSE) */
#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR)
extern uint16_t g_u16ActualCommutTimerPeriod;                                   /*!< Actual commutation period (actual speed) */
#endif /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) */
#if (_SUPPORT_CURRSPIKE_POSITION != FALSE)
extern uint16_t g_u16SpikePeriodSum;                                            /*!< [DC] Max 65535 x 2 x 1/PWM-Frequency = 7.28 s */
extern uint16_t g_u16SpikeCount;                                                /*!< [DC] Current Spike Count */
#endif /* (_SUPPORT_CURRSPIKE_POSITION != FALSE) */
extern uint16_t l_u16CDI;                                                       /*!< CDI State */
#pragma space none                                                              /* __NEAR_SECTION__ */

/*!*************************************************************************** */
/*                           GLOBAL FUNCTIONS                                  */
/* *************************************************************************** */
extern void MotorDriverConfig(uint16_t u16State);
extern void MotorDriverInit(uint16_t u16FullInit);
extern void MotorDriverPermanentError(uint8_t u8ElectricErrorCode);
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
#if (_SUPPORT_POS_INIT != FALSE)
extern void MotorDriverPosInit(uint16_t u16InitPosition);
#endif /* (_SUPPORT_POS_INIT != FALSE) */
#if (C_MOTOR_PHASES != 1)
extern void ConvMicroSteps2ShaftSteps(void);
extern uint32_t ConvShaftSteps2MicroSteps(uint16_t u16Position);
#endif /* (C_MOTOR_PHASES != 1) */
#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) && (_SUPPORT_HALL_LATCH || (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || \
    (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE))
extern void ConvHallLatchSteps2ShaftSteps(void);
extern uint16_t ConvShaftSteps2HallLatchSteps(uint16_t u16Position);
#endif /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) && (_SUPPORT_HALL_LATCH || (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE)) */
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
#if (C_MOTOR_PHASES == 3)
extern void MotorDriver_3Phase(uint16_t u16MicroStepIdx);
#elif (C_MOTOR_PHASES != 1)
extern void MotorDriver_4Phase(uint16_t u16MicroStepIdx);
#endif /* C_MOTOR_PHASES */
extern void MotorDriverCurrentMeasureInit(void);
extern void MotorDriverHoldCurrentMeasure(void);
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) || (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR)
extern void MotorDriver_InitialPwmDutyCycle(uint16_t u16CurrentLevel, uint16_t u16MotorSpeed);
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) || (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) */
#if (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM) && ((_SUPPORT_PWM_MODE != SINGLE_COIL_PWM_BIPOLAR) || _SUPPORT_HALL_LATCH) && \
    (C_MICROSTEP_PER_FULLSTEP != 0)
extern uint16_t MotorDriverUpdateMicroStepIndex(uint16_t u16MicroStepIdx);
#endif /* (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM) && ((_SUPPORT_PWM_MODE != SINGLE_COIL_PWM_BIPOLAR) || _SUPPORT_HALL_LATCH) && (C_MICROSTEP_PER_FULLSTEP != 0) */
#if  (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR)
extern uint16_t MotorDriverCurrentMeasure(uint16_t u16MicroStepIdx);
#elif (_SUPPORT_FOC_MODE != FOC_MODE_NONE) || (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM)
extern void MotorDriverCurrentMeasure(uint16_t u16MicroStepIdx);
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) || (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) */
#if (_SUPPORT_CDI != FALSE)
extern void MotorDriverCurrentMeasureCDI(void);
#endif /* (_SUPPORT_CDI != FALSE) */
extern void MotorDriverSpeed(uint16_t u16MotorTargetSpeed);
extern void MotorDriverStart(uint16_t u16MotorTargetSpeed);                     /*!< Start Motor Driver */
extern void MotorDriverStop(uint16_t u16Immediate);
extern void MotorDriverPeriodicTimer(uint16_t u16Period);
#if (_SUPPORT_TACHO_OUT != FALSE)
extern void MotorDriverTachoInit(void);
#endif /* (_SUPPORT_TACHO_OUT !=  FALSE) */
#if (_SUPPORT_CDI != FALSE)
extern void CheckActivationCDI(void);
extern uint16_t MotorDriverUpdateFullStepIndex(uint16_t u16FullStepIdx);
#endif /* (_SUPPORT_CDI != FALSE) */
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE) && (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
extern uint16_t MotorDriverResolverAngleToMicroStepIndex(uint16_t u16ResolverAngle);
#endif /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) && (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || \
    /* (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || */ (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
    (_SUPPORT_TRIAXIS_MLX90427 != FALSE)
extern void ConvTriaxisPos2ShaftSteps(void);
extern uint16_t ConvShaftSteps2TriaxisPos(uint16_t u16Position);
#endif /* (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX9038x != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90425 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
        * (_SUPPORT_TRIAXIS_MLX90427 != FALSE) */
#if (_SUPPORT_POTI != FALSE) && (_SUPPORT_POTI_MODE == C_POTI_MODE_CLOSED_LOOP)
extern void ConvPotiPos2ShaftSteps(void);
extern uint16_t ConvShaftSteps2PotiPos(uint16_t u16Position);
#endif /* (_SUPPORT_POTI != FALSE) && (_SUPPORT_POTI_MODE == C_POTI_MODE_CLOSED_LOOP) */

/*! MotorDriverSelfTest.c */
#if (_SUPPORT_MOTOR_SELFTEST != FALSE)
extern void MotorDriverSelfTest(void);                                          /*!< Perform driver Self-test */
#endif /* (_SUPPORT_MOTOR_SELFTEST != FALSE) */
#if (_SUPPORT_OSD != FALSE) && (!defined (__MLX81160__) && !defined (__MLX81330__) && !defined (__MLX81339__) && !defined (__MLX81350__))
extern void MotorDriverTest_OSD(void);
#endif /* (_SUPPORT_OSD != FALSE) && (!defined (__MLX81160__) && !defined (__MLX81330__) && !defined (__MLX81339__) && !defined (__MLX81350__)) */

#pragma space dp
extern volatile uint16_t l_u16CorrectionRatio;                                  /*!< Motor correction ratio, depend on temperature and voltage */
extern volatile uint16_t l_u16StallDetectorDelay;                               /*!< Stall detector delay (micro-steps) */
extern uint16_t l_u16MotorCurrentMovAvgxN;                                      /*!< Moving average current (4..16 samples) */
extern uint16_t l_u16MotorCurrentLPF;                                           /*!< Low-pass filter (IIR-1) motor-current x 64 */
extern volatile uint16_t l_u16MicroStepIdx;                                     /*!< (Micro)step index */
extern uint16_t l_u16MotorDriverDisconDelay;                                    /*!< Motor Driver Disconnect Delay: From LS/TRI-state to OFF */
extern volatile E_MOTOR_STARTUP_MODE_t l_e8MotorStartupMode;                    /*!< Motor driver state */
#if (_SUPPORT_STALLDET_BZC != FALSE)
extern volatile uint8_t g_e8ZcDetectorState;                                    /*!< [BEMF] Zero-crossing detector state */
#endif /* (_SUPPORT_STALLDET_BZC != FALSE) */
#pragma space none

/*!*************************************************************************** *
 * Get_CorrectionRatio
 * \brief   Get variable l_u16CorrectionRatio
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_u16CorrectionRatio
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_CorrectionRatio(void)
{
/*    extern volatile uint16_t l_u16CorrectionRatio; __attribute__ ((dp)); */     /* Motor correction ratio, depend on temperature and voltage */
    return (l_u16CorrectionRatio);
} /* End of Get_CorrectionRatio() */

/*!*************************************************************************** *
 * Set_CorrectionRatio
 * \brief   Set variable l_u16CorrectionRatio
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16Value
 * \return  -
 * *************************************************************************** *
 * \details in-line function to set 'local' variable
 * *************************************************************************** */
static inline void Set_CorrectionRatio(uint16_t u16Value)
{
/*    extern volatile uint16_t l_u16CorrectionRatio; __attribute__ ((dp)); */     /* Motor correction ratio, depend on temperature and voltage */
    l_u16CorrectionRatio = u16Value;
} /* End of Set_CorrectionRatio() */

/*!*************************************************************************** *
 * Get_MicroStepIdx
 * \brief   Get variable l_u16MicroStepIdx
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_u16MicroStepIdx
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_MicroStepIdx(void)
{
/*    extern uint16_t l_u16MicroStepIdx; __attribute__ ((dp)); */
    return (l_u16MicroStepIdx);
} /* End of Get_MicroStepIdxs() */

/*!*************************************************************************** *
 * Get_MotorDriverDisconDelay
 * \brief   Get variable l_u16MotorDriverDisconDelay
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_u16MotorDriverDisconDelay
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_MotorDriverDisconDelay(void)
{
/*    extern uint16_t l_u16MotorDriverDisconDelay; __attribute__ ((dp)); */
    return (l_u16MotorDriverDisconDelay);
} /* End of Get_MotorDriverDisconDelay() */

/*!*************************************************************************** *
 * Get_StartupDelay / Get_StallDetectorDelay
 * \brief   Get variable l_u16StallDetectorDelay
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_u16StallDetectorDelay
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_StartupDelay(void)
{
/*    extern uint16_t l_u16StallDetectorDelay; __attribute__ ((dp)); */
    return (l_u16StallDetectorDelay);
} /* End of Get_StartupDelay() */

/*!*************************************************************************** *
 * Get_MotorStartupMode
 * \brief   Get variable l_e8MotorStartupMode
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_e8MotorStartupMode
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline E_MOTOR_STARTUP_MODE_t  Get_MotorStartupMode(void)
{
/*    extern volatile E_MOTOR_STARTUP_MODE_t l_e8MotorStartupMode; __attribute__ ((dp)); */
    return (l_e8MotorStartupMode);
} /* End of Get_MotorStartupMode() */

#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
/*!*************************************************************************** *
 * Get_MotorHoldingCurrState
 * \brief   Get variable l_u8MotorHoldingCurrState
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_u8MotorHoldingCurrState
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint8_t Get_MotorHoldingCurrState(void)
{
    extern uint8_t l_u8MotorHoldingCurrState;
    return (l_u8MotorHoldingCurrState);
} /* End of Get_MotorHoldingCurrState() */
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */

/*!*************************************************************************** *
 * Get_MotorCurrentMovAvgxN
 * \brief   Get variable l_u16MotorCurrentMovAvgxN
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_u16MotorCurrentMovAvgxN
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_MotorCurrentMovAvgxN(void)
{
/*    extern uint16_t l_u16MotorCurrentMovAvgxN __attribute__ ((dp)); */
    return (l_u16MotorCurrentMovAvgxN);
} /* End of Get_MotorCurrentMovAvgxN() */

/*!*************************************************************************** *
 * Get_MotorCurrentMovAvgxN_mA
 * \brief   Get Motor Current Moving-Average in mA
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t Moving Average motor current in mA
 * *************************************************************************** *
 * \details in-line function
 * *************************************************************************** */
static inline uint16_t Get_MotorCurrentMovAvgxN_mA(void)
{
#if (C_MOVAVG_SZ == (65536UL / C_GMCURR_DIV))
    uint16_t u16Value = p_MulU16hi_U16byU16(Get_MotorCurrentMovAvgxN(),         /* Moving-average motor-current [ADC-LSB] */
                                            Get_MCurrGain());
#elif defined (C_GMCURR_SDIV) && defined (C_MOVAVG_SSZ)
    uint16_t u16Value =
        (uint16_t) (p_MulU32_U16byU16(l_u16MotorCurrentMovAvgxN,                /* Moving-average motor-current [ADC-LSB] */
                                      Get_MCurrGain()) >> (C_GMCURR_SDIV + C_MOVAVG_SSZ));
#else
    uint16_t u16Value =
        p_MulDivU16_U16byU16byU16(Get_MotorCurrentMovAvgxN(),                   /* Moving-average motor-current [ADC-LSB] */
                                  Get_MCurrGain(),
                                  (C_GMCURR_DIV * C_MOVAVG_SZ));
#endif
    return (u16Value);                                                          /* Moving-average motor-current [mA] */
} /* End of Get_MotorCurrentMovAvgxN_mA() */

/*!*************************************************************************** *
 * Set_MotorCurrentMovAvgxN
 * \brief   Set variable l_u16MotorCurrentMovAvgxN
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16Value
 * \return  -
 * *************************************************************************** *
 * \details in-line function to set 'local' variable
 * *************************************************************************** */
static inline void Set_MotorCurrentMovAvgxN(uint16_t u16Value)
{
/*    extern uint16_t l_u16MotorCurrentMovAvgxN __attribute__ ((dp)); */
    l_u16MotorCurrentMovAvgxN = u16Value;
} /* End of Set_MotorCurrentMovAvgxN() */

/*!*************************************************************************** *
 * Get_MotorCurrentLPF
 * \brief   Get variable l_u16MotorCurrentLPF
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_u16MotorCurrentLPF
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_MotorCurrentLPF(void)
{
/*    extern uint16_t l_u16MotorCurrentLPF __attribute__ ((dp)); */
    return (l_u16MotorCurrentLPF);
} /* End of Get_MotorCurrentLPF() */

/*!*************************************************************************** *
 * Get_ShaftRatiox512
 * \brief   Get variable l_u16ShaftRatiox512
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_u16ShaftRatiox512
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_ShaftRatiox512(void)
{
    extern uint16_t l_u16ShaftRatiox512;
    return (l_u16ShaftRatiox512);
} /* End of Get_ShaftRatio() */

/*!*************************************************************************** *
 * Set_CommutTimerPeriod
 * \brief   Set variable l_u16CommutTimerPeriod
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16Value
 * \return  -
 * *************************************************************************** *
 * \details in-line function to set 'local' variable
 * *************************************************************************** */
static inline void Set_CommutTimerPeriod(uint16_t u16Value)
{
    extern uint16_t l_u16CommutTimerPeriod;                                     /* Commutation time */
    l_u16CommutTimerPeriod = u16Value;
} /* End of Set_CommutTimerPeriod() */

#if (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL > 1U) && (_SUPPORT_HALL_LATCH_DIAG == FALSE)
/*!*************************************************************************** *
 * Get_NrOfCommut
 * \brief   Get variable l_u16NrOfCommut
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_u16NrOfCommut
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_NrOfCommut(void)
{
    extern uint16_t l_u16NrOfCommut;
    return (l_u16NrOfCommut);
} /* End of Get_NrOfCommut() */
#endif /* (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL > 1U) && (_SUPPORT_HALL_LATCH_DIAG == FALSE) */

/*!*************************************************************************** *
 * Get_MotorMicroStepsPerMechRotation
 * \brief   Get variable l_u16MotorMicroStepsPerMechRotation
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_u16MotorMicroStepsPerMechRotation
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_MotorMicroStepsPerMechRotation(void)
{
    extern uint16_t l_u16MotorMicroStepsPerMechRotation;
    return (l_u16MotorMicroStepsPerMechRotation);
} /* End of Get_MotorMicroStepsPerMechRotation() */

/*!*************************************************************************** *
 * Get_MotorMicroStepsPerElecRotation
 * \brief   Get variable l_u16MotorMicroStepsPerElecRotation
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_u16MotorMicroStepsPerElecRotation
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_MotorMicroStepsPerElecRotation(void)
{
    extern uint16_t l_u16MotorMicroStepsPerElecRotation;
    return (l_u16MotorMicroStepsPerElecRotation);
} /* End of Get_MotorMicroStepsPerElecRotation() */

/*!*************************************************************************** *
 * Get_CommutTimerPeriod
 * \brief   Get variable l_u16CommutTimerPeriod
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_u16CommutTimerPeriod
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_CommutTimerPeriod(void)
{
    extern uint16_t l_u16CommutTimerPeriod;
    return (l_u16CommutTimerPeriod);
} /* End of Get_CommutTimerPeriod() */

#if (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM)
/*!*************************************************************************** *
 * Get_MicroStepPeriodOneRPM
 * \brief   Get variable l_u32MicroStepPeriodOneRPM
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint32_t l_u32MicroStepPeriodOneRPM
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint32_t Get_MicroStepPeriodOneRPM(void)
{
    extern uint32_t l_u32MicroStepPeriodOneRPM;
    return (l_u32MicroStepPeriodOneRPM);
} /* End of Get_MicroStepPeriodOneRPM() */
#endif /* (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM) */

#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE)                                       /* FOC */
/*!*************************************************************************** *
 * Get_ActLoadAngle
 * \brief   Get variable l_i16ActLoadAngle
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  int16_t l_i16ActLoadAngle
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline int16_t Get_ActLoadAngle(void)
{
    extern int16_t l_i16ActLoadAngle;                                           /* Actual Load-angle */
    return (l_i16ActLoadAngle);
} /* End of Get_ActLoadAngle() */

/*!*************************************************************************** *
 * Get_ActLoadAngleLPF
 * \brief   Get variable l_i16ActLoadAngleLPF
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  int16_t l_i16ActLoadAngle
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 *          MMP220720-1
 * *************************************************************************** */
static inline int16_t Get_ActLoadAngleLPF(void)
{
    extern int16_t l_i16ActLoadAngleLPF;                                        /* Actual Load-angle (LPF) */
    return (l_i16ActLoadAngleLPF);
} /* End of Get_ActLoadAngle() */
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */

#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) && (LINPROT == LIN13_HVACTB)
/*!*************************************************************************** *
 * Set_RampUpPeriod
 * \brief   Set variable l_u16RampUpPeriod
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16Value
 * \return  -
 * *************************************************************************** *
 * \details in-line function to set 'local' variable
 * *************************************************************************** */
static inline void Set_RampUpPeriod(uint16_t u16Value)
{
    extern uint16_t l_u16RampUpPeriod;
    l_u16RampUpPeriod = u16Value;
} /* End of Set_RampUpPeriod() */

/*!*************************************************************************** *
 * Set_RampDownPeriod
 * \brief   Set variable l_u16RampDownPeriod
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16Value
 * \return  -
 * *************************************************************************** *
 * \details in-line function to set 'local' variable
 * *************************************************************************** */
static inline void Set_RampDownPeriod(uint16_t u16Value)
{
    extern uint16_t l_u16RampDownPeriod;
    l_u16RampDownPeriod = u16Value;
} /* End of Set_RampDownPeriod() */
#endif /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) && (LINPROT == LIN13_HVACTB) */

#if (_SUPPORT_POTI != FALSE) && (_SUPPORT_POTI_MODE == C_POTI_MODE_CLOSED_LOOP)
/*!*************************************************************************** *
 * Set_TargetPotiPos
 * \brief   Set variable l_u16TargetPotiPos
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16Value
 * \return  -
 * *************************************************************************** *
 * \details in-line function to set 'local' variable
 * *************************************************************************** */
static inline void Set_TargetPotiPos(uint16_t u16Value)
{
    l_u16TargetPotiPos = u16Value;
} /* End of Set_TargetPotiPos() */
#endif /* (_SUPPORT_POTI != FALSE) && (_SUPPORT_POTI_MODE == C_POTI_MODE_CLOSED_LOOP) */

#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT) */

#endif /* DRIVE_LIB_MOTOR_DRIVER_H */

/* EOF */
