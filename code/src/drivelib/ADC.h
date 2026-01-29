/*!************************************************************************** *
 * \file        ADC.h
 * \brief       MLX8133x ADC handling
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
 *           -# Get_CurrentZeroOffset()
 *           -# Get_MCurrGain()
 *           -# Get_HighVoltGain()
 *           -# Get_MotorVoltGainF()
 *           -# Get_ChipTemperature()
 *           -# Get_SupplyVoltage()
 *           -# Get_MotorVoltage()
 *           -# Get_MotorCurrent_mA()
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
 * ************************************************************************** */

#ifndef DRIVE_LIB_ADC_H_
#define DRIVE_LIB_ADC_H_

/*!*************************************************************************** */
/*                           INCLUDES                                          */
/* *************************************************************************** */
#include "../AppBuild.h"                                                        /* Application Build */

#include "../hal_lib/hal_ADC.h"                                                 /* HAL ADC support */

/*!*************************************************************************** */
/*                           DEFINITIONS                                       */
/* *************************************************************************** */
#define C_ADC_DNL           1U                                                  /*!< ADC DNL +/- 1 LSB */
#define C_ADC_INL           3U                                                  /*!< ADC INL +/- 3 LSB */
#define C_ADC_ERROR         (C_ADC_DNL + C_ADC_INL)                             /*!< ADC (total) ERROR (MMP240412-2) */
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
#define C_MAX_NOISE_VS       10U                                                /*!< 0.50V --> 0.50V/21 = 0.0238V/2.5Vref * 1023 = 9.74 */
#if (C_IC_UV_LEVEL == 0)
#define C_MIN_VS            ( 88U - C_ADC_ERROR)                                /*!< 4.50V --> 4.50V/21 = 0.214V/2.5Vref * 1023 = 87.69 */
#define C_MIN_VS_wNOISE     (127U - C_ADC_ERROR)                                /*!< 6.50V --> 6.50V/21 = 0.310V/2.5Vref * 1023 = 126.66 */
#elif (C_IC_UV_LEVEL == 1)
#define C_MIN_VS            (107U - C_ADC_ERROR)                                /*!< 5.50V --> 5.50V/21 = 0.262V/2.5Vref * 1023 = 107.17 */
#define C_MIN_VS_wNOISE     (127U - C_ADC_ERROR)                                /*!< 6.50V --> 6.50V/21 = 0.310V/2.5Vref * 1023 = 126.66 */
#elif (C_IC_UV_LEVEL == 2)
#define C_MIN_VS            (127U - C_ADC_ERROR)                                /*!< 6.50V --> 6.50V/21 = 0.310V/2.5Vref * 1023 = 126.66 */
#define C_MIN_VS_wNOISE     (127U - C_ADC_ERROR)                                /*!< 6.50V --> 6.50V/21 = 0.310V/2.5Vref * 1023 = 126.66 */
#elif (C_IC_UV_LEVEL == 3)
#define C_MIN_VS            (146U - C_ADC_ERROR)                                /*!< 7.50V --> 7.50V/21 = 0.357V/2.5Vref * 1023 = 146.14 */
#define C_MIN_VS_wNOISE     (146U - C_ADC_ERROR)                                /*!< 7.50V --> 7.50V/21 = 0.357V/2.5Vref * 1023 = 146.14 */
#elif (C_IC_UV_LEVEL == 4)
#define C_MIN_VS            (166U - C_ADC_ERROR)                                /*!< 8.50V --> 8.50V/21 = 0.405V/2.5Vref * 1023 = 165.63 */
#define C_MIN_VS_wNOISE     (166U - C_ADC_ERROR)                                /*!< 8.50V --> 8.50V/21 = 0.405V/2.5Vref * 1023 = 165.63 */
#else
#define C_MIN_VS            (185U - C_ADC_ERROR)                                /*!< 9.50V --> 9.50V/21 = 0.452V/2.5Vref * 1023 = 185.11 */
#define C_MIN_VS_wNOISE     (185U - C_ADC_ERROR)                                /*!< 9.50V --> 9.50V/21 = 0.452V/2.5Vref * 1023 = 185.11 */
#endif
#define C_MIN_VS_EEWRT      (146U - C_ADC_ERROR)                                /*!< 7.50V --> 7.50V/21 = 0.357V/2.5Vref * 1023 = 146.14 */
#define C_MIN_VDDA          (643U - C_ADC_ERROR)                                /*!< 3.15V --> 3.14500V/2 = 1.575V/2.5Vref * 1023 = 643.47 */
#define C_MAX_VDDA          (707U + C_ADC_ERROR)                                /*!< 3.45V --> 3.45499V/2 = 1.725V/2.5Vref * 1023 = 706.89 */
#define C_ABS_MAX_VDDA      (738U + C_ADC_ERROR)                                /*!< Abs. Max Rating: 3.6V --> 3.60499V/2 = 1.8V/2.5Vref * 1023 = 737.58 */
#define C_MIN_VDDD          (734U - C_ADC_ERROR)                                /*!< 1.80V --> 1.79500V/2.5Vref * 1023 = 734.514 (MMP191205-1) */
#define C_MAX_VDDD          (800U + C_ADC_ERROR)                                /*!< 1.95V --> 1.95499V/2.5Vref * 1023 = 799.98 (MMP191205-1) */
#define C_ABS_MAX_VDDD      (798U + C_ADC_ERROR)                                /*!< Abs. Max Rating: 1.95V --> 1.95V/2.5Vref * 1023 = 797.94 */
/*#define C_MIN_VBGD        (468U - C_ADC_ERROR)*/                              /*!< 1.15V --> 1.14500V/2.5Vref * 1023 = 468.534 (MMP220307-1) */
/*#define C_MAX_VBGD        (501U + C_ADC_ERROR)*/                              /*!< 1.22V --> 1.22499V/2.5Vref * 1023 = 501.266 (MMP220307-1) */
#define C_MIN_VBGD          (457U - C_ADC_ERROR)                                /*!< 1.15V --> 1.11700V/2.5Vref * 1023 = 457.076 (MMP220307-1); ADC accuracy: +/- 33mV (DS: 45mV/1.36) */
#define C_MAX_VBGD          (513U + C_ADC_ERROR)                                /*!< 1.22V --> 1.25300V/2.5Vref * 1023 = 512.728 (MMP220307-1); ADC accuracy: +/- 33mV (DS: 45mV/1.36) */
#define C_MAX_IO_LOW        (120U + C_ADC_ERROR)                                /*!< 0.4V --> 0.4V/1.36 --> 0.294V/2.5Vref * 1023 = 120.35 */
#define C_MIN_IO_HIGH       (722U - C_ADC_ERROR)                                /*!< 2.4V --> 2.4V/1.36 --> 1.765/2.5Vref * 1023 = 722.12 */
#elif defined (__MLX81339__) || defined (__MLX81350__)
#if (_SUPPORT_ADC_HIRES == FALSE)
#define C_MAX_NOISE_VS       17U                                                /*!< 0.50V --> 0.50V/18 = 0.0294V/1.65Vref * 1023 = 17.22 */
#if (C_IC_UV_LEVEL == 0)
#define C_MIN_VS            (155U - C_ADC_ERROR)                                /*!< 4.50V --> 4.50V/18 = 0.250V/1.65Vref * 1023 = 155.00 */
#define C_MIN_VS_wNOISE     (224U - C_ADC_ERROR)                                /*!< 6.50V --> 6.50V/18 = 0.361V/1.65Vref * 1023 = 223.88 */
#elif (C_IC_UV_LEVEL == 1)
#define C_MIN_VS            (189U - C_ADC_ERROR)                                /*!< 5.50V --> 5.50V/18 = 0.306V/1.65Vref * 1023 = 189.44 */
#define C_MIN_VS_wNOISE     (224U - C_ADC_ERROR)                                /*!< 6.50V --> 6.50V/18 = 0.361V/1.65Vref * 1023 = 223.88 */
#elif (C_IC_UV_LEVEL == 2)
#define C_MIN_VS            (224U - C_ADC_ERROR)                                /*!< 6.50V --> 6.50V/18 = 0.361V/1.65Vref * 1023 = 223.88 */
#define C_MIN_VS_wNOISE     (224U - C_ADC_ERROR)                                /*!< 6.50V --> 6.50V/18 = 0.361V/1.65Vref * 1023 = 223.88 */
#elif (C_IC_UV_LEVEL == 3)
#define C_MIN_VS            (258U - C_ADC_ERROR)                                /*!< 7.50V --> 7.50V/18 = 0.441V/1.65Vref * 1023 = 258.33 */
#define C_MIN_VS_wNOISE     (258U - C_ADC_ERROR)                                /*!< 7.50V --> 7.50V/18 = 0.441V/1.65Vref * 1023 = 258.33 */
#elif (C_IC_UV_LEVEL == 4)
#define C_MIN_VS            (293U - C_ADC_ERROR)                                /*!< 8.50V --> 8.50V/18 = 0.472V/1.65Vref * 1023 = 292.78 */
#define C_MIN_VS_wNOISE     (293U - C_ADC_ERROR)                                /*!< 8.50V --> 8.50V/18 = 0.472V/1.65Vref * 1023 = 292.78 */
#else
#define C_MIN_VS            (327U - C_ADC_ERROR)                                /*!< 9.50V --> 9.50V/18 = 0.528V/1.65Vref * 1023 = 327.22 */
#define C_MIN_VS_wNOISE     (327U - C_ADC_ERROR)                                /*!< 9.50V --> 9.50V/18 = 0.528V/1.65Vref * 1023 = 327.22 */
#endif
#define C_MIN_VS_EEWRT      (258U - C_ADC_ERROR)                                /*!< 7.50V --> 7.50V/18 = 0.441V/1.65Vref * 1023 = 258.33 */
#define C_MIN_VDDA          (488U - C_ADC_ERROR)                                /*!< 3.15V --> 3.14500V/4 = 0.78625V/1.65Vref * 1023 = 488.25 */
#define C_MAX_VDDA          (535U + C_ADC_ERROR)                                /*!< 3.45V --> 3.45499V/4 = 0.864V/1.65Vref * 1023 = 534.75 */
#define C_ABS_MAX_VDDA      (559U + C_ADC_ERROR)                                /*!< Abs. Max Rating: 3.6V --> 3.60499V/4 = 0.901V/1.65Vref * 1023 = 558.775 */
#define C_MIN_VDDD          (465U - C_ADC_ERROR)                                /*!< 1.50V --> 1.79500V/2 = 0.8975/1.65Vref * 1023 = 558.00 */ /*MMP19: VDDD=?*/
#define C_MAX_VDDD          (605U + C_ADC_ERROR)                                /*!< 1.95V --> 1.95499V/2 = 0.977/1.65Vref * 1023 = 604.50 */ /*MMP19: VDDD=?*/
#define C_ABS_MAX_VDDD      (605U + C_ADC_ERROR)                                /*!< Abs. Max Rating: 1.95V --> 1.95V/2 = 0.975V/1.65Vref * 1023 = 604.50 */ /*MMP19: VDDD=?*/
#else  /* (_SUPPORT_ADC_HIRES == FALSE) */
#define C_MAX_NOISE_VS       69U                                                /*!< 0.50V --> 0.50V/18 = 0.0278V/1.65Vref * 4095 = 68.94 */
#if (C_IC_UV_LEVEL == 0)
#define C_MIN_VS            (620U - C_ADC_ERROR)                                /*!< 4.50V --> 4.50V/18 = 0.250V/1.65Vref * 4095 = 620.45 */
#define C_MIN_VS_wNOISE     (896U - C_ADC_ERROR)                                /*!< 6.50V --> 6.50V/18 = 0.361V/1.65Vref * 4095 = 896.21 */
#elif (C_IC_UV_LEVEL == 1)
#define C_MIN_VS            (758U - C_ADC_ERROR)                                /*!< 5.50V --> 5.50V/18 = 0.306V/1.65Vref * 4095 = 758.33 */
#define C_MIN_VS_wNOISE     (896U - C_ADC_ERROR)                                /*!< 6.50V --> 6.50V/18 = 0.361V/1.65Vref * 4095 = 896.21 */
#elif (C_IC_UV_LEVEL == 2)
#define C_MIN_VS            (949U - C_ADC_ERROR)                                /*!< 6.50V --> 6.50V/18 = 0.361V/1.65Vref * 4095 = 896.21 */
#define C_MIN_VS_wNOISE     (949U - C_ADC_ERROR)                                /*!< 6.50V --> 6.50V/18 = 0.361V/1.65Vref * 4095 = 896.21 */
#elif (C_IC_UV_LEVEL == 3)
#define C_MIN_VS            (1034U - C_ADC_ERROR)                               /*!< 7.50V --> 7.50V/18 = 0.417V/1.65Vref * 4095 = 1034.09 */
#define C_MIN_VS_wNOISE     (1034U - C_ADC_ERROR)                               /*!< 7.50V --> 7.50V/18 = 0.417V/1.65Vref * 4095 = 1034.09 */
#elif (C_IC_UV_LEVEL == 4)
#define C_MIN_VS            (1172U - C_ADC_ERROR)                               /*!< 8.50V --> 8.50V/18 = 0.472V/1.65Vref * 4095 = 1171.97 */
#define C_MIN_VS_wNOISE     (1172U - C_ADC_ERROR)                               /*!< 8.50V --> 8.50V/18 = 0.472V/1.65Vref * 4095 = 1171.97 */
#else
#define C_MIN_VS            (1310U - C_ADC_ERROR)                               /*!< 9.50V --> 9.50V/18 = 0.528V/1.65Vref * 4095 = 1309.85 */
#define C_MIN_VS_wNOISE     (1310U - C_ADC_ERROR)                               /*!< 9.50V --> 9.50V/18 = 0.528V/1.65Vref * 4095 = 1309.85 */
#endif
#define C_MIN_VS_EEWRT      (1034U - C_ADC_ERROR)                               /*!< 7.50V --> 7.50V/18 = 0.417V/1.65Vref * 4095 = 1034.09 */
#define C_MIN_VDDA          (1951U - C_ADC_ERROR)                               /*!< 3.15V --> 3.14500V/4 = 0.78625V/1.65Vref * 4095 = 1951.33 */
#define C_MAX_VDDA          (2144U + C_ADC_ERROR)                               /*!< 3.45V --> 3.45499V/4 = 0.864V/1.65Vref * 4095 = 2143.66 */
#define C_ABS_MAX_VDDA      (2237U + C_ADC_ERROR)                               /*!< Abs. Max Rating: 3.6V --> 3.60499V/4 = 0.901V/1.65Vref * 4095 = 2236.73 */
#define C_MIN_VDDD          (2227U - C_ADC_ERROR)                               /*!< 1.50V --> 1.79500V/2 = 0.8975/1.65Vref * 4095 = 2227.43 */ /*MMP19: VDDD=?*/
#define C_MAX_VDDD          (2426U + C_ADC_ERROR)                               /*!< 1.95V --> 1.95499V/2 = 0.977/1.65Vref * 4095 = 2425.97 */ /*MMP19: VDDD=?*/
#define C_ABS_MAX_VDDD      (2426U + C_ADC_ERROR)                               /*!< Abs. Max Rating: 1.95V --> 1.95V/2 = 0.975V/1.65Vref * 4095 = 2425.97 */ /*MMP19: VDDD=?*/
#endif /* (_SUPPORT_ADC_HIRES == FALSE) */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#define C_MAX_NOISE_VS        27U                                               /*!< 0.50V --> 0.50V/26 = 0.0192V/1.48Vref * 2048 = 26.6 */
#define C_MIN_VS            ( 231U - C_ADC_ERROR)                               /*!< 4.00V --> 4.00V/26 = 0.167V/1.48Vref * 2048 = 212.89 */
#define C_MIN_VS_wNOISE     ( 346U - C_ADC_ERROR)                               /*!< 6.50V --> 6.50V/26 = 0.250V/1.48Vref * 2048 = 345.95 */
#define C_MIN_VS_EEWRT      ( 399U - C_ADC_ERROR)                               /*!< 7.50V --> 7.50V/26 = 0.288V/1.48Vref * 2048 = 399.17 */
#define C_MIN_VS_VBOOST     ( 532U - C_ADC_ERROR)                               /*!< 10.0V --> 10.0V/26 = 0.385V/1.48Vref * 2048 = 532.22 */
#define C_MIN_VDDA          (1453U - C_ADC_ERROR)                               /*!< 3.15V --> 3.15V/3 = 1.050V/1.48Vref * 2048 = 1452.97 */
#define C_MAX_VDDA          (1591U + C_ADC_ERROR)                               /*!< 3.45V --> 3.45V/3 = 1.150V/1.48Vref * 2048 = 1591.35 */
#define C_ABS_MAX_VDDA      (1661U + C_ADC_ERROR)                               /*!< Abs. Max Rating: 3.6V --> 3.6V/3 = 1.2V/1.48Vref * 2048 = 1660.54 */
#define C_MIN_VDDD          (1159U - C_ADC_ERROR)                               /*!< 1.675V --> 1.675V/2 = 0.8375/1.48Vref * 2048 = 1158.92 */
#define C_MAX_VDDD          (1349U + C_ADC_ERROR)                               /*!< 1.95V --> 1.95V/2 = 0.975/1.48Vref * 2048 = 1349.19 */
#define C_ABS_MAX_VDDD      (1349U + C_ADC_ERROR)                               /*!< Abs. Max Rating: 1.95V --> 1.95V/2 = 0.975/1.48Vref * 2048 = 1349.19 */
/*#define C_MIN_VBGD        (1584U - C_ADC_ERROR)*/                             /*!< 1.15V --> 1.14500V/1.48Vref * 2048 = 1584.432 (MMP220307-1) */
/*#define C_MAX_VBGD        (1695U + C_ADC_ERROR)*/                             /*!< 1.22V --> 1.22499V/1.48Vref * 2048 = 1695.121 (MMP220307-1) */
#define C_MIN_VBGD          (1566U - C_ADC_ERROR)                               /*!< 1.15V --> 1.13200V/1.48Vref * 2048 = 1566.443 (MMP220307-1); ADC accuracy: +/- 18mV (DS: 45mV/2.5) */
#define C_MAX_VBGD          (1713U + C_ADC_ERROR)                               /*!< 1.22V --> 1.23800V/1.48Vref * 2048 = 1713.124 (MMP220307-1); ADC accuracy: +/- 18mV (DS: 45mV/2.5) */
#define C_MAX_IO_LOW        ( 221U + C_ADC_ERROR)                               /*!< 0.4V --> 0.4V/2.4 --> 0.167V/1.48Vref * 2048 = 221.41 */
#define C_MIN_IO_HIGH       (1328U - C_ADC_ERROR)                               /*!< 2.4V --> 2.4V/2.4 --> 1.000/1.48Vref * 2048 = 1328.43 */
#elif defined (__MLX81160__)
#define C_MAX_NOISE_VS        27U                                               /*!< 0.50V --> 0.50V/26 = 0.0192V/1.48Vref * 2048 = 26.6 */
#define C_MIN_VS            ( 231U - C_ADC_ERROR)                               /*!< 4.00V --> 4.00V/26 = 0.167V/1.48Vref * 2048 = 212.89 */
#define C_MIN_VS_wNOISE     ( 346U - C_ADC_ERROR)                               /*!< 6.50V --> 6.50V/26 = 0.250V/1.48Vref * 2048 = 345.95 */
#define C_MIN_VS_EEWRT      ( 399U - C_ADC_ERROR)                               /*!< 7.50V --> 7.50V/26 = 0.288V/1.48Vref * 2048 = 399.17 */
#define C_MIN_VDDA          (1453U - C_ADC_ERROR)                               /*!< 3.15V --> 3.15V/3 = 1.050V/1.48Vref * 2048 = 1452.97 */
#define C_MAX_VDDA          (1591U + C_ADC_ERROR)                               /*!< 3.45V --> 3.45V/3 = 1.150V/1.48Vref * 2048 = 1591.35 */
#define C_ABS_MAX_VDDA      (1661U + C_ADC_ERROR)                               /*!< Abs. Max Rating: 3.6V --> 3.6V/3 = 1.2V/1.48Vref * 2048 = 1660.54 */
#define C_MIN_VDDD          (1159U - C_ADC_ERROR)                               /*!< 1.675V --> 1.675V/2 = 0.8375/1.48Vref * 2048 = 1158.92 */
#define C_MAX_VDDD          (1349U + C_ADC_ERROR)                               /*!< 1.95V --> 1.95V/2 = 0.975/1.48Vref * 2048 = 1349.19 */
#define C_ABS_MAX_VDDD      (1349U + C_ADC_ERROR)                               /*!< Abs. Max Rating: 1.95V --> 1.95V/2 = 0.975/1.48Vref * 2048 = 1349.19 */
/*#define C_MIN_VBGD        (1584U - C_ADC_ERROR)*/                             /*!< 1.15V --> 1.14500V/1.48Vref * 2048 = 1584.432 (MMP220307-1) */
/*#define C_MAX_VBGD        (1695U + C_ADC_ERROR)*/                             /*!< 1.22V --> 1.22499V/1.48Vref * 2048 = 1695.121 (MMP220307-1) */
#define C_MIN_VBGD          (1566U - C_ADC_ERROR)                               /*!< 1.15V --> 1.13200V/1.48Vref * 2048 = 1566.443 (MMP220307-1); ADC accuracy: +/- 18mV (DS: 45mV/2.5) */
#define C_MAX_VBGD          (1713U + C_ADC_ERROR)                               /*!< 1.22V --> 1.23800V/1.48Vref * 2048 = 1713.124 (MMP220307-1); ADC accuracy: +/- 18mV (DS: 45mV/2.5) */
#define C_MAX_IO_LOW        ( 221U + C_ADC_ERROR)                               /*!< 0.4V --> 0.4V/2.4 --> 0.167V/1.48Vref * 2048 = 221.41 */
#define C_MIN_IO_HIGH       (1328U - C_ADC_ERROR)                               /*!< 2.4V --> 2.4V/2.4 --> 1.000/1.48Vref * 2048 = 1328.43 */
#endif

#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
/* ADC S&H Delay of 1.5us */
#if (ADC_FREQ > 10666667UL)                                                     /* ADC frequency: >10.67 MHz */
#error "ERROR: ADC-Clock too high"
#elif (ADC_FREQ > 9333333UL)                                                    /* ADC frequency: 9.34 ..10.67 MHz */
#define C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK C_ADC_HW_TRIGGER_EOCpls16xADC_CLOCK
#elif (ADC_FREQ > 8000000UL)                                                    /* ADC frequency: 8.01 .. 9.33 MHz */
#define C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK C_ADC_HW_TRIGGER_EOCpls14xADC_CLOCK
#elif (ADC_FREQ > 6666667UL)                                                    /* ADC frequency: 6.67 .. 8.00 MHz */
#define C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK C_ADC_HW_TRIGGER_EOCpls12xADC_CLOCK
#elif (ADC_FREQ > 5333333UL)                                                    /* ADC frequency: 5.34 .. 6.66 MHz */
#define C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK C_ADC_HW_TRIGGER_EOCpls10xADC_CLOCK
#elif (ADC_FREQ > 4000000UL)                                                    /* ADC frequency: 4.01 .. 5.33 MHz */
#define C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK C_ADC_HW_TRIGGER_EOCpls8xADC_CLOCK
#elif (ADC_FREQ > 2666667UL)                                                    /* ADC frequency: 2.67 .. 4.00 MHz */
#define C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK C_ADC_HW_TRIGGER_EOCpls6xADC_CLOCK
#else  /* ADC_FREQ */
#error "ERROR: C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK is not defined"
#endif /* ADC_FREQ */
#elif defined (__MLX81339__) || defined (__MLX81350__)  /*MMP39/MMP50*/
/* ADC S&H Delay of 1.0us */
#define C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK C_ADC_HW_TRIGGER_EOCpls16xADC_CLOCK
#elif defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
/* ADC S&H Delay of 0.6us */
#if (ADC_FREQ > 20000000UL)                                                     /* ADC frequency: >20.00 MHz */
#error "ERROR: ADC-Clock too high"
#elif (ADC_FREQ > 18333333UL)                                                   /* ADC frequency: 18.34 .. 20.00 MHz */
#define C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK C_ADC_HW_TRIGGER_EOCpls13xADC_CLOCK
#elif (ADC_FREQ > 15000000UL)                                                   /* ADC frequency: 15.01 .. 18.33 MHz */
#define C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK C_ADC_HW_TRIGGER_EOCpls11xADC_CLOCK
#elif (ADC_FREQ > 11666667UL)                                                   /* ADC frequency: 11.67 .. 15.00 MHz */
#define C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK C_ADC_HW_TRIGGER_EOCpls9xADC_CLOCK
#elif (ADC_FREQ >  8333333UL)                                                   /* ADC frequency:  8.34 .. 11.66 MHz */
#define C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK C_ADC_HW_TRIGGER_EOCpls7xADC_CLOCK
#elif (ADC_FREQ >= 5000000UL)                                                   /* ADC frequency:  5.00 ..  8.34 MHz */
#define C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK C_ADC_HW_TRIGGER_EOCpls5xADC_CLOCK
#else  /* ADC_FREQ */
#error "ERROR: C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK is not defined"
#endif /* ADC_FREQ */
#endif

/*! ADC Initial Results */
typedef struct _ADC_INIT_RESULTS
{
    uint16_t u16AdcVs;                                                          /**< Chip supply voltage (21:1) */
    uint16_t u16AdcVsmF;                                                        /**< Motor supply voltage (21:1) */
    uint16_t u16AdcVdda;                                                        /**< Chip analogue supply system (2:1) */
    uint16_t u16AdcVddd;                                                        /**< Chip digital supply system (1:1) */
    uint16_t u16AdcVaux;                                                        /**< VAUX voltage (4:1) */
    uint16_t u16AdcZeroCurrent_Dummy;                                           /**< Motor current offset - Dummy Sample (MMP240412-1) */
    uint16_t u16AdcZeroCurrent_1;                                               /**< Motor current offset - Sample #1 */
    uint16_t u16AdcZeroCurrent_2;                                               /**< Motor current offset - Sample #2 */
#if defined (__MLX81160__)
    uint16_t u16AdcZeroCurrent2_Dummy;                                          /**< Motor current offset - Dummy Sample (MMP240412-1) */
    uint16_t u16AdcZeroCurrent_3;                                               /**< Motor current offset - Sample #3 */
    uint16_t u16AdcZeroCurrent_4;                                               /**< Motor current offset - Sample #4 */
    uint16_t u16AdcGnda_1;                                                      /**< Analogue Ground level - Sample #1 */
    uint16_t u16AdcGnda_2;                                                      /**< Analogue Ground level - Sample #2 */
#elif defined (__MLX81344__) || defined (__MLX81346__)
    uint16_t u16AdcZeroVoltage_1;                                               /**< Voltage offset - Sample #1 */
    uint16_t u16AdcZeroVoltage_2;                                               /**< Voltage offset - Sample #2 */
#endif /* defined (__MLX81344__) || defined (__MLX81346__) */
#if (_SUPPORT_ADC_VSMF_OFF != FALSE)
    uint16_t u16AdcZeroVSMF_1;                                                  /**< VSMF offset - Sample #1 */
    uint16_t u16AdcZeroVSMF_2;                                                  /**< VSMF offset - Sample #2 */
#endif /* (_SUPPORT_ADC_VSMF_OFF != FALSE) */
    uint16_t u16CRC;                                                            /**< CRC of data */
} ADC_INIT_RESULTS;

#if (_SUPPORT_ADC_IO_CHECK != FALSE)
typedef struct _ADC_IO_RESULT
{
    uint16_t u16AdcIO;                                                          /**< IO voltage (1.36:1) */
    uint16_t u16CRC;                                                            /**< CRC of data */
} ADC_IO_RESULT;
#endif /* (_SUPPORT_ADC_IO_CHECK != FALSE) */

/*! ADC Motor Current Results */
typedef struct _ADC_MCURR_RESULTS
{
    uint16_t u16AdcZeroCurrent_Dummy;                                           /**< Motor current offset - Dummy Sample (MMP240412-1) */
    uint16_t u16AdcZeroCurrent_1;                                               /**< Motor current offset - Sample #1 */
    uint16_t u16AdcZeroCurrent_2;                                               /**< Motor current offset - Sample #2 */
    uint16_t u16AdcZeroCurrent_3;                                               /**< Motor current offset - Sample #3 */
    uint16_t u16AdcZeroCurrent_4;                                               /**< Motor current offset - Sample #4 */
    uint16_t u16CRC;                                                            /**< CRC of data */
} ADC_MCURR_RESULTS;

#if (_SUPPORT_OSD != FALSE) && (!defined (__MLX81160__) && !defined (__MLX81330__) && !defined (__MLX81339__) && !defined (__MLX81350__))
/*! ADC Off State Diagnostics Results */
typedef struct _ADC_OSD_RESULTS
{
    uint16_t u16AdcVsmF;                                                        /**< Driver supply voltage (8133x: 21:1; 8134x: 26:1) */
    uint16_t u16AdcVphU;                                                        /**< Phase U-voltage */
    uint16_t u16AdcVphV;                                                        /**< Phase V-voltage */
    uint16_t u16AdcVphW;                                                        /**< Phase W-voltage */
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
    uint16_t u16AdcVphT;                                                        /**< Phase T-voltage */
#endif /* defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__) */
    uint16_t u16CRC;                                                            /**< CRC of data */
} ADC_OSD_RESULTS;
#endif /* (_SUPPORT_OSD != FALSE) && (!defined (__MLX81160__) && !defined (__MLX81330__) && !defined (__MLX81339__) && !defined (__MLX81350__)) */

#if (defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)) && _SUPPORT_ADC_REF_HV_CALIB  /* MMP190523-2 */
/*! ADC Reference Calibration Results */
#define C_ADC_VS_REF1V5     (C_ADC_CH0 | (uint16_t)C_ADC_REF_1_50_V)            /*!<  0           1.5V    Vs-unfiltered (divided by 21) */
#define C_ADC_BGD_REF1V5    (C_ADC_CH4 | (uint16_t)C_ADC_REF_1_50_V)            /*!<  4           1.5V    Band gap (Digital) */
#define C_ADC_VPHU_REF1V5   (C_ADC_CH12 | (uint16_t)C_ADC_REF_1_50_V)           /*!< 12           1.5V    Phase U voltage (divided by 21) */
#define C_ADC_VPHV_REF1V5   (C_ADC_CH13 | (uint16_t)C_ADC_REF_1_50_V)           /*!< 13           1.5V    Phase V voltage (divided by 21) */
#define C_ADC_VPHW_REF1V5   (C_ADC_CH14 | (uint16_t)C_ADC_REF_1_50_V)           /*!< 14           1.5V    Phase W voltage (divided by 21) */
#define C_ADC_VPHT_REF1V5   (C_ADC_CH15 | (uint16_t)C_ADC_REF_1_50_V)           /*!< 15           1.5V    Phase T voltage (divided by 21) */
#define C_ADC_VSM_REF1V5    (C_ADC_CH17 | (uint16_t)C_ADC_REF_1_50_V)           /*!< 17           1.5V    Vsm-unfiltered (divided by 21) */
#define C_ADC_IO0HV_REF1V5  (C_ADC_CH18 | (uint16_t)C_ADC_REF_1_50_V)           /*!< 18           1.5V    IO[0].HV test-purpose */
#define C_ADC_VSMF_REF1V5   (C_ADC_CH20 | (uint16_t)C_ADC_REF_1_50_V)           /*!< 20           1.5V    Vsm-filtered (divided by 21) */

/*! ADC VS Calibration structure */
typedef struct
{
    uint16_t u16ADC_VS_Vref2V5_1;                                               /**< Chip supply voltage (21:1) with ADC Reference 2.5V, #1 */
    uint16_t u16ADC_VS_Vref1V5_1;                                               /**< Chip supply voltage (21:1) with ADC Reference 1.5V, #1 */
    uint16_t u16ADC_VS_Vref2V5_2;                                               /**< Chip supply voltage (21:1) with ADC Reference 2.5V, #2 */
    uint16_t u16ADC_VS_Vref1V5_2;                                               /**< Chip supply voltage (21:1) with ADC Reference 1.5V, #2 */
    uint16_t u16ADC_VS_Vref2V5_3;                                               /**< Chip supply voltage (21:1) with ADC Reference 2.5V, #3 */
    uint16_t u16ADC_VS_Vref1V5_3;                                               /**< Chip supply voltage (21:1) with ADC Reference 1.5V, #3 */
    uint16_t u16ADC_VS_Vref2V5_4;                                               /**< Chip supply voltage (21:1) with ADC Reference 2.5V, #4 */
    uint16_t u16ADC_VS_Vref1V5_4;                                               /**< Chip supply voltage (21:1) with ADC Reference 1.5V, #4 */
    uint16_t u16CRC;                                                            /**< ADC Data CRC */
} ADC_VS_TEST;
#endif /* (defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)) && _SUPPORT_ADC_REF */

/* ************************************************************************** */
/*                          GLOBAL VARIABLES                                  */
/* ************************************************************************** */
#pragma space dp                                                                /* __TINY_SECTION__ */
extern uint16_t l_u16CurrentZeroOffset;                                         /*!< Zero-current ADC-offset */
#pragma space none                                                              /* __TINY_SECTION__ */

#pragma space nodp                                                              /* __NEAR_SECTION__ */
#pragma space none                                                              /* __NEAR_SECTION__ */

/* ************************************************************************** */
/*                          GLOBAL FUNCTIONS                                  */
/* ************************************************************************** */
extern void ADC_Init(void);                                                     /* Initialise ADC */
extern void ADC_MCurrOffCalib(void);                                            /* Motor Current Offset calibration */
extern uint16_t ADC_Conv_Vsupply(void);                                         /* Get Supply-voltage [10mV] */
extern uint16_t ADC_Conv_Vmotor(void);                                          /* Get Motor-voltage [10mV] */
extern int16_t ADC_Conv_TempJ(uint16_t u16Init);                                /* Get Chip Junction temperature [C] */
extern uint16_t ADC_Conv_Cmotor(void);                                          /* Get Motor Driver Current [mA] */
#if (_SUPPORT_OSD != FALSE) && (!defined (__MLX81160__) && !defined (__MLX81330__) && !defined (__MLX81339__) && !defined (__MLX81350__))
extern void ADC_OSD_Start(void);
#endif /* (_SUPPORT_OSD != FALSE) && (!defined (__MLX81160__) && !defined (__MLX81330__) && !defined (__MLX81339__) && !defined (__MLX81350__)) */

#if (LIN_COMM != FALSE) && (_SUPPORT_MLX_DEBUG_MODE != FALSE)
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE)
extern void LinDiag_VddaVddd(void);
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) */
#if (_SUPPORT_ADC_BGD != FALSE)
extern void LinDiag_VauxVbgd(void);
#endif /* (_SUPPORT_ADC_BGD != FALSE) */
#if (_SUPPORT_ADC_VBOOST != FALSE)
extern void LinDiag_Vboost(void);
#endif /* (_SUPPORT_ADC_VBOOST != FALSE) */
#endif /* (LIN_COMM != FALSE) && (_SUPPORT_MLX_DEBUG_MODE != FALSE) */

/*!*************************************************************************** *
 * Get_CurrentZeroOffset
 * \brief   Get variable l_u16CurrentZeroOffset
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_u16CurrentZeroOffset
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_CurrentZeroOffset(void)
{
    return (l_u16CurrentZeroOffset);
} /* End of Get_CurrentZeroOffset() */

#endif /* DRIVE_LIB_ADC_H_ */
