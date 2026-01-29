/*!*************************************************************************** *
 * \file        ADC.c
 * \brief       MLX8133x/4x ADC handling
 *
 * \note        project MLX8133x/4x
 *
 * \author      Marcel Braat
 *
 * \date        2017-08-14
 *
 * \version     2.0
 *
 * A list of functions:
 *  - Global Functions:
 *           -# ADC_Calib()
 *           -# ADC_Init()
 *           -# ADC_MCurrOffCalib()
 *           -# ADC_Conv_Vsupply()
 *           -# ADC_Conv_Vmotor()
 *           -# ADC_Conv_TempJ()
 *           -# ADC_Conv_Cmotor()
 *           -# ADC_OSD_Start()
 *           -# LinDiag_VddaVddd()
 *  - Internal Functions:
 *           -# ADC_IO_Check()
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
 * ADC Sample & Hold Frequency RC: ~4MHz --> 5Cy = 1.25us
 *                                           7Cy = 1.75us
 * ADC Conversion time: 12 Cy @ ADC_Frequency (Max. samples/PWM-period):
 *                                   PWM-Period:     50us   40us
 *  @ 4.0 MHz           = 3.0 us (Total: 4.75us)     10x     8x
 *  @ 4.8 MHz (24MHz/5) = 2.5 us (Total: 4.25us)     11x     9x
 *  @ 5.33MHz (32MHz/6) = 2.25us (Total: 4.0us)      12x    10x
 *  @ 5.6 MHz (28MHz/5) = 2.15us (Total: 3.9us)      12x    10x
 *  @ 6.4 MHz (32MHz/5) = 1.875us (Total: 3.625us)   13x    11x
 *  @ 8.0 MHz (32MHz/4) = 1.5 us (Total: 3.25us)     15x    12x
 *
 * *************************************************************************** */

/*!*************************************************************************** *
 *                              I N C L U D E S                                *
 * *************************************************************************** */
#include "AppBuild.h"                                                           /* Application Build */

/* Actuator Platform includes */
#include "drivelib/ErrorCodes.h"                                                /* Error-logging support */
#include "drivelib/GlobalVars.h"                                                /* Global Variables support */
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
#include "drivelib/MotorDriver.h"                                               /* Motor Driver support */
#elif (_SUPPORT_APP_TYPE == C_APP_RELAY)
#include "drivelib/RelayDriver.h"                                               /* Relay driver support */
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
#include "drivelib/SolenoidDriver.h"                                            /* Solenoid Driver support */
#endif /* (_SUPPORT_APP_TYPE) */
#include "camculib/private_mathlib.h"                                           /* Private Math Library support */

/* Application includes */
#include "../ActADC.h"                                                          /* Application ADC support */

/* Communication includes */
#if (LIN_COMM != FALSE)
#include "commlib/LIN_Communication.h"                                          /* LIN Communication support */
#endif /* (LIN_COMM != FALSE) */

/* CAMCU Platform includes */
#include <bist_inline_impl.h>
#include <sys_tools.h>

/*!*************************************************************************** */
/*                           DEFINITIONS                                       */
/* *************************************************************************** */
#define ADC_TRIGGER_OSD_SW      0                                               /*!< Trigger OSD SW ID */
#define ADC_TRIGGER_OSD_CTIMER1 1                                               /*!< Trigger OSD CTIMER1 ID */
#define ADC_TRIGGER_OSD         ADC_TRIGGER_OSD_CTIMER1                         /*!< Trigger OSD Selection */

/*!*************************************************************************** */
/*                           GLOBAL & LOCAL VARIABLES                          */
/* *************************************************************************** */
#pragma space dp                                                                /* __TINY_SECTION__ */
uint16_t l_u16CurrentZeroOffset = C_OADC_MCUR;                                  /*!< Zero-current ADC-offset */
#pragma space none                                                              /* __TINY_SECTION__ */

#pragma space nodp                                                              /* __NEAR_SECTION__ */
/* ADC Calibration parameters */
#if defined (__MLX81160__)
uint16_t l_u16CurrentZeroOffset2 = C_OADC_MCUR;                                 /*!< Zero-current ADC-offset (2nd channel) */
#endif /* defined (__MLX81160__) */
#if (_SUPPORT_ADC_REF_HV_CALIB != FALSE)
static uint8_t l_u8AdcRefCalib = FALSE;                                         /*!< ADC Reference calibration */
#endif /* (_SUPPORT_ADC_REF_HV_CALIB != FALSE) */

#if (_SUPPORT_OSD != FALSE) && (!defined (__MLX81160__) && !defined (__MLX81330__) && !defined (__MLX81339__) && !defined (__MLX81350__))
ADC_OSD_RESULTS AdcOsdResult;                                                   /*!< Motor-driver inactive */
#endif /* (_SUPPORT_OSD != FALSE) && (!defined (__MLX81160__) && !defined (__MLX81330__) && !defined (__MLX81339__) && !defined (__MLX81350__)) */

#if (LIN_COMM != FALSE) && _SUPPORT_MLX_DEBUG_MODE
#if (_APP_DMA_STRESS_TEST != FALSE)
uint16_t l_u16VddaADC;                                                          /*!< VDDA Voltage [ADC-LSB] */
uint16_t l_u16VdddADC;                                                          /*!< VDDD Voltage [ADC-LSB] */
#else  /* (_APP_DMA_STRESS_TEST != FALSE) */
static uint16_t l_u16VddaADC;                                                   /*!< VDDA Voltage [ADC-LSB] */
static uint16_t l_u16VdddADC;                                                   /*!< VDDD Voltage [ADC-LSB] */
#endif /* (_APP_DMA_STRESS_TEST != FALSE) */
#if (_SUPPORT_ADC_VBOOST != FALSE)
static uint16_t l_u16VboostADC;                                                 /*!< Boost Voltage [ADC-LSB] */
#endif /* (_SUPPORT_ADC_VBOOST != FALSE) */
#endif /* (LIN_COMM != FALSE) && _SUPPORT_MLX_DEBUG_MODE */
#pragma space none                                                              /* __NEAR_SECTION__ */

#if (_SUPPORT_ADC_IO_CHECK != FALSE)
/*!*************************************************************************** *
 * ADC_IO_Check
 * \brief   Check I/O channels
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: ADC_Init()
 * - Cyclomatic Complexity: 5+1
 * - Nesting: 4
 * - Function calling: 2 (HAL_ADC_StartSoftTrig(), SetLastError())
 * *************************************************************************** */
static void ADC_IO_Check(void)
{
#if defined (__MLX81330__) || defined (__MLX81350__)
    static const ADC_SDATA_t SBASE_VIO[4] =
    {
        C_ADC_IO0_LV_EOC,
        C_ADC_IO1_LV_EOC,
        C_ADC_IO2_LV_EOC,
        C_ADC_IO3_LV_EOC
    };
    static const uint16_t IO_CONFIG[4][2] =
    {
        {~M_PORT_IO_CFG0_IO0_OUT_SEL, C_PORT_IO_CFG0_IO0_OUT_SEL_SOFT},
        {~M_PORT_IO_CFG0_IO1_OUT_SEL, C_PORT_IO_CFG0_IO1_OUT_SEL_SOFT},
        {~M_PORT_IO_CFG0_IO2_OUT_SEL, C_PORT_IO_CFG0_IO2_OUT_SEL_SOFT},
        {~M_PORT_IO_CFG0_IO3_OUT_SEL, C_PORT_IO_CFG0_IO3_OUT_SEL_SOFT}
    };
#else  /* defined (__MLX81330__) || defined (__MLX81350__) */
    static const ADC_SDATA_t SBASE_VIO[8] =
    {
#if (_SUPPORT_I2C == FALSE) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_NONE) || ((C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_0) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_0))
        C_ADC_IO0_LV_EOC,
#endif /* (_SUPPORT_I2C == FALSE) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_NONE) || ((C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_0) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_0)) */
#if (_SUPPORT_I2C == FALSE) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_NONE) || ((C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_1) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_1))
        C_ADC_IO1_LV_EOC,
#endif /* (_SUPPORT_I2C == FALSE) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_NONE) || ((C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_1) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_1)) */
#if (_SUPPORT_I2C == FALSE) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_NONE) || ((C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_2) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_2))
        C_ADC_IO2_LV_EOC,
#endif /* (_SUPPORT_I2C == FALSE) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_NONE) || ((C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_2) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_2)) */
#if (_SUPPORT_I2C == FALSE) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_NONE) || ((C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_3) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_3))
        C_ADC_IO3_LV_EOC,
#endif /* (_SUPPORT_I2C == FALSE) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_NONE) || ((C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_3) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_3)) */
#if (_SUPPORT_I2C == FALSE) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_NONE) || ((C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_4) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_4))
        C_ADC_IO4_LV_EOC,
#endif /* (_SUPPORT_I2C == FALSE) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_NONE) || ((C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_4) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_4)) */
#if (_SUPPORT_I2C == FALSE) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_NONE) || ((C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_5) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_5))
        C_ADC_IO5_LV_EOC,
#endif /* (_SUPPORT_I2C == FALSE) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_NONE) || ((C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_5) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_5)) */
#if (_SUPPORT_I2C == FALSE) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_NONE) || ((C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_6) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_6))
        C_ADC_IO6_LV_EOC,
#endif /* (_SUPPORT_I2C == FALSE) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_NONE) || ((C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_6) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_6)) */
#if (_SUPPORT_I2C == FALSE) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_NONE) || ((C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_7) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_7))
        C_ADC_IO7_LV_EOC
#endif /* (_SUPPORT_I2C == FALSE) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_NONE) || ((C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_7) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_7)) */
    };
    static const uint16_t IO_CONFIG[8][2] =
    {
#if (_SUPPORT_I2C == FALSE) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_NONE) || ((C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_0) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_0))
        {~M_PORT_IO_CFG0_IO0_OUT_SEL, C_PORT_IO_CFG0_IO0_OUT_SEL_SOFT},
#endif /* (_SUPPORT_I2C == FALSE) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_NONE) || ((C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_0) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_0)) */
#if (_SUPPORT_I2C == FALSE) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_NONE) || ((C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_1) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_1))
        {~M_PORT_IO_CFG0_IO1_OUT_SEL, C_PORT_IO_CFG0_IO1_OUT_SEL_SOFT},
#endif /* (_SUPPORT_I2C == FALSE) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_NONE) || ((C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_1) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_1)) */
#if (_SUPPORT_I2C == FALSE) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_NONE) || ((C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_2) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_2))
        {~M_PORT_IO_CFG0_IO2_OUT_SEL, C_PORT_IO_CFG0_IO2_OUT_SEL_SOFT},
#endif /* (_SUPPORT_I2C == FALSE) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_NONE) || ((C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_2) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_2)) */
#if (_SUPPORT_I2C == FALSE) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_NONE) || ((C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_3) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_3))
        {~M_PORT_IO_CFG0_IO3_OUT_SEL, C_PORT_IO_CFG0_IO3_OUT_SEL_SOFT},
#endif /* (_SUPPORT_I2C == FALSE) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_NONE) || ((C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_3) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_3)) */
#if (_SUPPORT_I2C == FALSE) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_NONE) || ((C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_4) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_4))
        {~M_PORT_IO_CFG1_IO4_OUT_SEL, C_PORT_IO_CFG1_IO4_OUT_SEL_SOFT},
#endif /* (_SUPPORT_I2C == FALSE) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_NONE) || ((C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_4) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_4)) */
#if (_SUPPORT_I2C == FALSE) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_NONE) || ((C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_5) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_5))
        {~M_PORT_IO_CFG1_IO5_OUT_SEL, C_PORT_IO_CFG1_IO5_OUT_SEL_SOFT},
#endif /* (_SUPPORT_I2C == FALSE) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_NONE) || ((C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_5) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_5)) */
#if (_SUPPORT_I2C == FALSE) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_NONE) || ((C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_6) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_6))
        {~M_PORT_IO_CFG1_IO6_OUT_SEL, C_PORT_IO_CFG1_IO6_OUT_SEL_SOFT},
#endif /* (_SUPPORT_I2C == FALSE) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_NONE) || ((C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_6) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_6)) */
#if (_SUPPORT_I2C == FALSE) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_NONE) || ((C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_7) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_7))
        {~M_PORT_IO_CFG1_IO7_OUT_SEL, C_PORT_IO_CFG1_IO7_OUT_SEL_SOFT}
#endif /* (_SUPPORT_I2C == FALSE) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_NONE) || ((C_I2C_SLAVE_SCK_IO != PIN_FUNC_IO_7) && (C_I2C_SLAVE_SDA_IO != PIN_FUNC_IO_7)) */
    };
#endif /* defined (__MLX81330__) || defined (__MLX81350__) */

    /* Check all I/O being not stuck to supply or ground, when defined as OUTPUT */
    if ( (IO_PORT_IO_ENABLE & M_PORT_IO_ENABLE_IO_ENABLE) != 0U)
    {
        /* At least one or more I/O's are defined as output */
        ADC_IO_RESULT AdcIoResult;
        uint16_t u16Idx;

        l_au16AdcSource[0] = (uint16_t)&AdcIoResult.u16AdcIO;
        l_au16AdcSource[2] = C_ADC_EOS;

        /* Full IO-range: IO[7:0] */
        for (u16Idx = 0U; u16Idx < (sizeof(SBASE_VIO) / sizeof(uint16_t)); u16Idx++)
        {
            uint16_t u16Mask = (B_PORT_IO_ENABLE_IO_ENABLE_0 << u16Idx);
            uint16_t u16OrgOutState = IO_PORT_IO_OUT_SOFT;                      /* Save I/O-State */
            uint16_t u16OrgConfigIO;
#if defined (__MLX81330__) || defined (__MLX81350__)
            uint16_t *pu16ConfigIO_Port = (uint16_t *)&IO_PORT_IO_CFG0;         /* I/O-pin configuration */
#else  /* !defined (__MLX81330__) || defined (__MLX81350__) */
            uint16_t *pu16ConfigIO_Port;
            if (u16Idx < 4)
            {
                pu16ConfigIO_Port = (uint16_t *)&IO_PORT_IO_CFG0;               /* I/O-pin configuration */
            }
            else
            {
                pu16ConfigIO_Port = (uint16_t *)&IO_PORT_IO_CFG1;               /* I/O-pin configuration */
            }
#endif /* !defined (__MLX81330__) || defined (__MLX81350__) */
            u16OrgConfigIO = *pu16ConfigIO_Port;

            if ( (IO_PORT_IO_ENABLE & u16Mask) != 0U)
            {
                /* Check high level; May cause VDDA overload */

                l_au16AdcSource[1] = SBASE_VIO[u16Idx].u16;
                *pu16ConfigIO_Port = (u16OrgConfigIO & IO_CONFIG[u16Idx][0]) | IO_CONFIG[u16Idx][1];  /* Set I/O-configuration into SW-mode */
                IO_PORT_IO_OUT_SOFT = u16OrgOutState | u16Mask;                 /* Set I/O High */
                (void)HAL_ADC_StartSoftTrig(C_ADC_STATE_IDLE);
                IO_PORT_IO_OUT_SOFT = u16OrgOutState;                           /* Restore I/O-state */
                *pu16ConfigIO_Port = u16OrgConfigIO;                            /* Restore I/O-configuration */
                if (AdcIoResult.u16AdcIO < C_MIN_IO_HIGH)
                {
                    g_e8ErrorElectric = (uint8_t)C_ERR_SUP_VIO;
                    IO_PORT_IO_ENABLE &= ~u16Mask;                              /* Disable I/O Output-mode */
#if (_SUPPORT_LOG_ERRORS != FALSE)
                    SetLastError( (C_ERR_VIO_0 + u16Idx) | (C_ERR_EXT | 0x0100U));
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                    continue;
                }

                /* Check low level */
                *pu16ConfigIO_Port = (u16OrgConfigIO & IO_CONFIG[u16Idx][0]) | IO_CONFIG[u16Idx][1];  /* Set I/O-configuration into SW-mode */
                IO_PORT_IO_OUT_SOFT = u16OrgOutState & ~u16Mask;
                (void)HAL_ADC_StartSoftTrig(C_ADC_STATE_IDLE);
                IO_PORT_IO_OUT_SOFT = u16OrgOutState;                           /* Restore I/O-state */
                *pu16ConfigIO_Port = u16OrgConfigIO;                            /* Restore I/O-configuration */
                if ( (AdcIoResult.u16AdcIO > C_MAX_IO_LOW) )
                {
                    g_e8ErrorElectric = (uint8_t)C_ERR_SUP_VIO;
                    IO_PORT_IO_ENABLE &= ~u16Mask;                              /* Disable I/O Output-mode */
#if (_SUPPORT_LOG_ERRORS != FALSE)
                    SetLastError( (C_ERR_VIO_0 + u16Idx) | (C_ERR_EXT | 0x0200U));
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
                }
            }
        }
    }
} /* End of ADC_IO_Check() */
#endif /* (_SUPPORT_ADC_IO_CHECK != FALSE) */

#if (_SUPPORT_ADC_REF_HV_CALIB != FALSE)
/*!*************************************************************************** *
 * ADC_ReferenceCalibration
 * \brief   ADC reference calibration.
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Adapting the Non Volatile Memory ADC Calibration data from 2.5V to 1.5V ADC
 *          reference voltage.
 * *************************************************************************** *
 * - Call Hierarchy: ADC_Init()
 * - Cyclomatic Complexity: 3+1
 * - Nesting: 3
 * - Function calling: 1 (HAL_ADC_StartSoftTrig())
 * *************************************************************************** */
void ADC_ReferenceCalibration(void)
{
    if (l_u8AdcRefCalib == FALSE)
    {
        /* Re-tune the VS, VMS(F) Gains for 1.5V ADC Reference Voltage */
        ADC_SDATA_t u16BGD_2V5 = C_ADC_VBGD_EOC;
        ADC_SDATA_t u16BGD_1V5 = C_ADC_VBGD_EOC_1V5;
        ADC_VS_TEST sADC_VS_Test;
        uint16_t u16ADC_Vref2V5;
        uint16_t u16ADC_Vref1V5;
        int16_t i16VsmOffset = Get_VsmOffset();

        l_au16AdcSource[0] = (uint16_t)&sADC_VS_Test;
        l_au16AdcSource[1] = u16BGD_2V5.u16;
        l_au16AdcSource[2] = u16BGD_1V5.u16;
        l_au16AdcSource[3] = u16BGD_2V5.u16;
        l_au16AdcSource[4] = u16BGD_1V5.u16;
        l_au16AdcSource[5] = u16BGD_2V5.u16;
        l_au16AdcSource[6] = u16BGD_1V5.u16;
        l_au16AdcSource[7] = u16BGD_2V5.u16;
        l_au16AdcSource[8] = u16BGD_1V5.u16;
        l_au16AdcSource[9] = C_ADC_EOS;
        (void)HAL_ADC_StartSoftTrig(C_ADC_STATE_IDLE);

        u16ADC_Vref2V5 =
            (sADC_VS_Test.u16ADC_VS_Vref2V5_1 + sADC_VS_Test.u16ADC_VS_Vref2V5_2 + sADC_VS_Test.u16ADC_VS_Vref2V5_3 +
             sADC_VS_Test.u16ADC_VS_Vref2V5_4);
        u16ADC_Vref1V5 =
            (sADC_VS_Test.u16ADC_VS_Vref1V5_1 + sADC_VS_Test.u16ADC_VS_Vref1V5_2 + sADC_VS_Test.u16ADC_VS_Vref1V5_3 +
             sADC_VS_Test.u16ADC_VS_Vref1V5_4);
        HAL_ADC_ConvHvGain(u16ADC_Vref2V5, u16ADC_Vref1V5);
        if (i16VsmOffset != 0)
        {
            if (i16VsmOffset > 0)
            {
                Set_VsmOffset(p_DivI16_I32byI16(p_MulI32_I16byI16(i16VsmOffset, u16ADC_Vref1V5) + (u16ADC_Vref2V5 / 2),
                              u16ADC_Vref2V5));
            }
            else
            {
                Set_VsmOffset(p_DivI16_I32byI16(p_MulI32_I16byI16(i16VsmOffset, u16ADC_Vref1V5) - (u16ADC_Vref2V5 / 2),
                              u16ADC_Vref2V5));
            }
        }
        l_u8AdcRefCalib = TRUE;
    }
} /* End of ADC_ReferenceCalibration() */
#endif /* (_SUPPORT_ADC_REF_HV_CALIB != FALSE) */

/*!*************************************************************************** *
 * ADC_Init
 * \brief   Measure Zero-current offset
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Before performing the ADC_Init, the I/O's intended to be used as
 *          output, should be configured as output (_SUPPORT_ADC_IO_CHECK).
 *          Prior to the ADC initialisation, VDDA and VDDAF (MLX81160 & MLX8133x)
 *          may not show UV
 * Note:    This function is not allowed to be called during Non Volatile Memory Write
 *          (due to Non Volatile Memory reads) (__MLX81160__).
 * *************************************************************************** *
 * - Call Hierarchy: AppInit(), MotorDriverStart()
 * - Cyclomatic Complexity: 9+1
 * - Nesting: 5
 * - Function calling: 3 (HAL_ADC_StopSafe(), HAL_ADC_StartSoftTrig(), SetLastError())
 *           Optional: 2 (ADC_IO_Check(), ADC_ReferenceCalibration())
 * *************************************************************************** */
void ADC_Init(void)
{
    static const ADC_SDATA_t SBASE_INIT[] =
    {
#if defined (__MLX81160__)
        C_ADC_VS_EOC,
        C_ADC_VSMF_EOC,
        C_ADC_VDDA_EOC,
        C_ADC_VDDD_EOC,
        C_ADC_VAUX_EOC,
        C_ADC_MCUR1_EOC,                                                        /* MMP240412-1: Improve ADC Offset measurement by skipping 1st measurement (S&H) */
        C_ADC_MCUR1_EOC,
        C_ADC_MCUR1_EOC,
        C_ADC_MCUR2_EOC,
        C_ADC_MCUR2_EOC,                                                        /* MMP240412-1: Improve ADC Offset measurement by skipping 1st measurement (S&H) */
        C_ADC_MCUR2_EOC,
        C_ADC_AGND_EOC,
        C_ADC_AGND_EOC,
#elif defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
        /* Chip supply voltage (21:1) */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_VS_HV,
#if !defined (__MLX81339__) && !defined (__MLX81350__)
                .u3AdcVref = C_ADC_VREF_2_50_V,                                 /* Force to use 2.5V reference */
#else  /* !defined (__MLX81339__) && !defined (__MLX81350__) */
                .u3AdcReserved = 0U,
#endif /* !defined (__MLX81339__) && !defined (__MLX81350__) */
                .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,
                .u1AdcReserved = 0U
            }
        },
        /* Driver supply voltage (21:1) */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_VSMF_HV,
#if !defined (__MLX81339__) && !defined (__MLX81350__)
                .u3AdcVref = C_ADC_VREF_2_50_V,                                 /* Force to use 2.5V reference */
#else  /* !defined (__MLX81339__) && !defined (__MLX81350__) */
                .u3AdcReserved = 0U,
#endif /* !defined (__MLX81339__) && !defined (__MLX81350__) */
                .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK,
                .u1AdcReserved = 0U
            }
        },
        C_ADC_VDDA_EOC,
        C_ADC_VDDD_EOC,
        C_ADC_VAUX_EOC,
        C_ADC_MCUR_EOC,                                                         /* MMP240412-1: Improve ADC Offset measurement by skipping 1st measurement (S&H) */
        C_ADC_MCUR_EOC,
        C_ADC_MCUR_EOC,
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
        C_ADC_VS_EOC,
        C_ADC_VSMF_EOC,
        C_ADC_VDDA_EOC,
        C_ADC_VDDD_EOC,
        C_ADC_VAUX_EOC,
        C_ADC_MCUR_EOC,                                                         /* MMP240412-1: Improve ADC Offset measurement by skipping 1st measurement (S&H) */
        C_ADC_MCUR_EOC,
        C_ADC_MCUR_EOC,
#if defined (__MLX81344__) || defined (__MLX81346__)
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u1AdcSin = C_ADC_SIN_SDATA,
                .u6AdcChannel = C_ADC_RES6,
                .u1AdcType = C_ADC_TYPE_CYCLIC,
                .u6AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK
            }
        },
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u1AdcSin = C_ADC_SIN_SDATA,
                .u6AdcChannel = C_ADC_RES7,
                .u1AdcType = C_ADC_TYPE_CYCLIC,
                .u6AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK
            }
        },
#endif /* defined (__MLX81344__) || defined (__MLX81346__) */
#if (_SUPPORT_ADC_VSMF_OFF != FALSE)
        C_ADC_AGND_EOC,
        C_ADC_AGND_EOC,
#endif /* (_SUPPORT_ADC_VSMF_OFF != FALSE) */
#endif
        {.u16 = C_ADC_EOS}                                                      /* End-of-Sequence */
    };

    uint16_t u16LoopCnt;
    uint16_t u16AdcValue;
    ADC_INIT_RESULTS AdcInitResult;                                             /*!< ADC Initial measurement results */

    if (l_u8AdcMode > C_ADC_MODE_IDLE)
    {
        HAL_ADC_StopSafe();
    }

    /* Enable Current Sense amplifier (and OC detector, with threshold) */
#if defined (__MLX81160__)
    IO_PORT_IO_ENABLE1 |= M_PORT_IO_ENABLE1_IO_DISREC;                          /* Disable weak pull-up */
    IO_PORT_CURR_SENS1 = B_PORT_CURR_SENS1_CSA1_EN_OC | B_PORT_CURR_SENS1_EN_CSA1 | NV_CURR_SENS_OC_DAC;  /* Enable Over-current comparator1 & Current Sense amplifier */
    IO_PORT_CURR_SENS2 = B_PORT_CURR_SENS2_CSA2_EN_OC | B_PORT_CURR_SENS2_EN_CSA2 | NV_CURR_SENS_OC_DAC;  /* Enable Over-current comparator2 & Current Sense amplifier */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#if defined (__MLX81340A01__) || defined (__MLX81344A01__)
    IO_PORT_CURR_SENS = B_PORT_CURR_SENS_EN_CSA | NV_CURR_SENS_OC_DAC;          /* Enable Over-current comparator & Current Sense amplifier */
#else  /* defined (__MLX81340B01__) || defined (__MLX81344B01__) || defined (__MLX81346__) */
    IO_PORT_CURR_SENS = B_PORT_CURR_SENS_EN_OC | B_PORT_CURR_SENS_EN_CSA | NV_CURR_SENS_OC_DAC;  /* Enable Over-current comparator & Current Sense amplifier (MMP210901-1) */
#endif /* defined (__MLX81340B01__) || defined (__MLX81344B01__) || defined (__MLX81346__) */
#endif /* defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */

    {
        uint16_t *pu16Src = &l_au16AdcSource[0];
        uint16_t *pu16SBase = (uint16_t *)&SBASE_INIT[0];
        *pu16Src++ = (uint16_t)&AdcInitResult.u16AdcVs;
        do
        {
            *pu16Src++ = *pu16SBase++;
        } while (pu16SBase < (uint16_t *)&SBASE_INIT[sizeof(SBASE_INIT) / sizeof(uint16_t)]);
    }

    HAL_ADC_PowerOn();
#if defined (__MLX81339__) || defined (__MLX81350__)
#if (_SUPPORT_ADC_HIRES != FALSE)
    IO_PORT_ADC_CTRL2 |= B_PORT_ADC_CTRL2_ADC_HIGHRES;                          /* Set ADC in 12-bit mode */
#else  /* (_SUPPORT_ADC_HIRES != FALSE) */
    IO_PORT_ADC_CTRL2 &= ~B_PORT_ADC_CTRL2_ADC_HIGHRES;                         /* Set ADC in 10-bit mode */
#endif /* (_SUPPORT_ADC_HIRES != FALSE) */
#endif /* defined (__MLX81339__) || defined (__MLX81350__) */
#if defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
    if ( (IO_PORT_DRV_OUT & B_PORT_DRV_OUT_ENABLE_DRVSUP) == 0U)                /* MMP201019-1: Make sure DRVSUP is enabled before ENABLE_CSA */
    {
        IO_PORT_DRV_OUT |= B_PORT_DRV_OUT_ENABLE_DRVSUP;
        DELAY_US(10U);
#if (_SUPPORT_APP_USER_MODE != FALSE)
        ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        IO_MLX16_ITC_PEND0_S = B_MLX16_ITC_PEND0_UV_VDDAF;
        IO_MLX16_ITC_MASK0_S |= B_MLX16_ITC_MASK0_UV_VDDAF;                     /* Enable VDDAF UV */
#if (_SUPPORT_APP_USER_MODE != FALSE)
        EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    }
#if defined (__MLX81160__)
    IO_PORT_CURR_SENS1 |= B_PORT_CURR_SENS1_CSA1_ZERO;                          /* Shorten current sense input 1 */
    IO_PORT_CURR_SENS2 |= B_PORT_CURR_SENS2_CSA2_ZERO;                          /* Shorten current sense input 2 */
#else  /* defined (__MLX81160__) */
    IO_PORT_DRV_OUT |= B_PORT_DRV_OUT_ENABLE_CSA;                               /* Enable Current Sense amplifier */
#endif /* defined (__MLX81160__) */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    IO_PORT_CURR_SENS |= B_PORT_CURR_SENS_CSA_ZERO;                             /* Shorten current sense input */
#endif
#if defined (__MLX81346__) && _SUPPORT_APP_48V_ADC
    IO_PORT_MISC2_OUT |= (B_PORT_MISC2_OUT_VSM_LOWGAIN | B_PORT_MISC2_OUT_VSM_FILT_ON);  /* Enable 52-divider and Enable VSM Filter */
#else  /* defined (__MLX81346__) && _SUPPORT_APP_48V_ADC */
#if defined (__MLX81350__)
    IO_PORT_MISC2_OUT |= (B_PORT_MISC2_OUT_ENABLE_VSMF_BUFFER | B_PORT_MISC2_OUT_VSM_FILT_ON);  /* Enable VSM Filter */
#else  /* defined (__MLX81350__) */
    IO_PORT_MISC2_OUT |= B_PORT_MISC2_OUT_VSM_FILT_ON;                          /* Enable VSM Filter */
#endif /* defined (__MLX81350__) */
#endif /* defined (__MLX81346__) && _SUPPORT_APP_48V_ADC */
    DELAY_US(1250U);

    /* ADC Initialise */
    HAL_ADC_Stop();                                                             /* STOP the ADC from any ADC mode. */
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
    /* Should be tested. If it need NOPS for MLX16-FX or not */
/* MMP: Must this be done?    IO_ADC_DATA = 0U; */
    IO_ADC_SAR_CLK_DIV = ((PLL_FREQ / ADC_FREQ) - 1U);                          /* ADC @ 4MHz */
#elif defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    /* Should be tested. If it need NOPS for MLX16-FX or not */
/* MMP: Must this be done?    IO_ADC_DATA = 0U; */
    IO_ADC_BLOCK_CLK = (IO_ADC_BLOCK_CLK & M_ADC_BLOCK_ADC_CLK_DIV) | ((PLL_FREQ / ADC_FREQ) - 1U);  /* ADC @ 16MHz */
    IO_ADC_BLOCK_X = B_ADC_BLOCK_INTREF;                                        /* Select internal reference voltage (internalVref = 1.5V) */
#endif

    /* MMP190920-2: In case of VS UV, repeat VS measurement until above UV-level, otherwise IC will reset */
    u16LoopCnt = C_ADC_SUPPLY_CHECK;
#if (_DEBUG_SUPPLY_ROBUSTNESS != FALSE)
    DEBUG_SET_IO_A();
#endif /* (_DEBUG_SUPPLY_ROBUSTNESS != FALSE) */
    do
    {
        (void)HAL_ADC_StartSoftTrig(C_ADC_STATE_IDLE);
        /* Check VS and VSMF (MMP210411-4) */
#if defined (__MLX81346__) && _SUPPORT_APP_48V_ADC
        /* Only check VS */
        if ( (AdcInitResult.u16AdcVs - C_OADC) > C_MIN_VS)
#else  /* defined (__MLX81346__) && _SUPPORT_APP_48V_ADC */
        /* Check both VS and VMSF */
        if ( ((AdcInitResult.u16AdcVs - C_OADC) > C_MIN_VS) &&
             ((AdcInitResult.u16AdcVsmF - C_OADC) > C_MIN_VS) )
#endif /* defined (__MLX81346__) && _SUPPORT_APP_48V_ADC */
        {
            u16LoopCnt--;
        }
        else
        {
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_WRN_VS_UV);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            u16LoopCnt = C_ADC_SUPPLY_CHECK;
        }
    } while (u16LoopCnt != 0U);
#if (_DEBUG_SUPPLY_ROBUSTNESS != FALSE)
    DEBUG_CLR_IO_A();
#endif /* (_DEBUG_SUPPLY_ROBUSTNESS != FALSE) */

#if (_SUPPORT_VDDA_5V == FALSE) && (_SUPPORT_SENSOR_VDDA_5V == FALSE)
    /* VDDA check, when VS is above 4V */
    u16AdcValue = (AdcInitResult.u16AdcVdda - C_OADC);
#if (LIN_COMM != FALSE) && (_SUPPORT_MLX_DEBUG_MODE != FALSE)
    l_u16VddaADC = u16AdcValue;
#endif /* (LIN_COMM != FALSE) && (_SUPPORT_MLX_DEBUG_MODE != FALSE) */
    if ( (u16AdcValue < C_MIN_VDDA) || (u16AdcValue > C_MAX_VDDA) )
    {
        g_e8ErrorElectric = (uint8_t)C_ERR_SUP_VDDA;
#if (_SUPPORT_LOG_ERRORS != FALSE)
        if (u16AdcValue < C_MIN_VDDA)
        {
            SetLastError(C_ERR_VDDA | C_ERR_EXTW | 0x0A00U);
        }
        else
        {
            SetLastError(C_ERR_VDDA | C_ERR_EXTW | 0x0B00U);
        }
        SetLastError(u16AdcValue);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
    }
#endif /* (_SUPPORT_VDDA_5V == FALSE) && (_SUPPORT_SENSOR_VDDA_5V == FALSE) */

    /* VDDD check */
    u16AdcValue = (AdcInitResult.u16AdcVddd - C_OADC);
#if (LIN_COMM != FALSE) && (_SUPPORT_MLX_DEBUG_MODE != FALSE)
    l_u16VdddADC = u16AdcValue;
#endif /* (LIN_COMM != FALSE) && (_SUPPORT_MLX_DEBUG_MODE != FALSE) */
    if ( (u16AdcValue < C_MIN_VDDD) || (u16AdcValue > C_MAX_VDDD) )
    {
        g_e8ErrorElectric = (uint8_t)C_ERR_SUP_VDDD;
#if (_SUPPORT_LOG_ERRORS != FALSE)
        if (u16AdcValue < C_MIN_VDDD)
        {
            SetLastError(C_ERR_VDDD | C_ERR_EXTW | 0x0A00U);
        }
        else
        {
            SetLastError(C_ERR_VDDD | C_ERR_EXTW | 0x0B00U);
        }
        SetLastError(u16AdcValue);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
    }

    /* Current offset */
    l_u16CurrentZeroOffset = (AdcInitResult.u16AdcZeroCurrent_1 + AdcInitResult.u16AdcZeroCurrent_2) / 2U;
#if defined (__MLX81160__)
    l_u16CurrentZeroOffset2 = (AdcInitResult.u16AdcZeroCurrent_3 + AdcInitResult.u16AdcZeroCurrent_4) / 2U;
    IO_PORT_CURR_SENS1 &= ~B_PORT_CURR_SENS1_CSA1_ZERO;                         /* Open current sense input 1 */
    IO_PORT_CURR_SENS2 &= ~B_PORT_CURR_SENS2_CSA2_ZERO;                         /* Open current sense input 2 */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    IO_PORT_CURR_SENS &= ~B_PORT_CURR_SENS_CSA_ZERO;                            /* Shorten current sense input */
#endif /* defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */

    /* Voltage offset */
#if defined (__MLX81160__)
    {
        uint16_t u16VsmOffset = (AdcInitResult.u16AdcGnda_1 + AdcInitResult.u16AdcGnda_2) / 2U;
        Set_VsmOffset(u16VsmOffset);
        Set_HighVoltOffset(u16VsmOffset);
    }
#elif defined (__MLX81344__) || defined (__MLX81346__)
    {
        uint16_t u16VsmOffset = (AdcInitResult.u16AdcZeroVoltage_1 + AdcInitResult.u16AdcZeroVoltage_2) / 2U;
        Set_VsmOffset(u16VsmOffset);
        Set_HighVoltOffset(u16VsmOffset);
    }
#endif /* defined (__MLX81344__) || defined (__MLX81346__) */
#if (_SUPPORT_ADC_VSMF_OFF != FALSE)
    /* Overrule VSMF Offset voltage */
    Set_VsmOffset( (AdcInitResult.u16AdcZeroVSMF_1 + AdcInitResult.u16AdcZeroVSMF_2) / 2U);
#endif /* (_SUPPORT_ADC_VSMF_OFF != FALSE) */

#if FALSE
    /* Extended count mode */
    static const ADC_SDATA_t SBASE_EXT_CNT[] =
    {
        C_ADC_VS_EOC,
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u1AdcSin = C_ADC_SIN_SDATA,
                .u6AdcChannel = C_ADC_VS_HV,
                .u1AdcType = C_ADC_TYPE_EC,
                .u6AdcTrigger = C_ADC_HW_TRIGGER_EOCpls15xADC_CLOCK
            }
        }
    };
    uint16_t *pu16Src = &l_au16AdcSource[0];
    *pu16Src++ = (uint16_t)&AdcInitResult.u16AdcVs;
    *pu16Src++ = SBASE_EXT_CNT[0].u16;                                          /* Cyclic ADC, 12-bits */
    *pu16Src++ = SBASE_EXT_CNT[0].u16;                                          /* Cyclic ADC, 12-bits */
    *pu16Src++ = SBASE_EXT_CNT[1].u16;                                          /* Extended Count - Count  8, 14-bits */
    *pu16Src++ = SBASE_EXT_CNT[1].u16;                                          /* Extended Count - Count  8, 14-bits */
    *pu16Src++ = SBASE_EXT_CNT[1].u16;                                          /* Extended Count - Count  8, 14-bits */
    *pu16Src++ = C_ADC_EOS;
    IO_ADC_BLOCK_X &= ~M_ADC_BLOCK_COUNT;                                       /* EC Count = 8 */
    HAL_ADC_Setup(/*B_ADC_ADC_WIDTH |*/                                         /* Setup Extended Count measurement */
        C_ADC_ASB_NEVER |                                                       /* Auto Standby: Never */
        C_ADC_INT_SCHEME_NOINT |                                                /* Message interrupt: No */
        B_ADC_SATURATE |                                                        /* Saturation: Enabled */
        B_ADC_NO_INTERLEAVE |                                                   /* Interleave: No */
        C_ADC_SOC_SOURCE_SOFT_TRIG |                                            /* Start Of Conversion (SOC) triggered by: Hardware */
        C_ADC_SOS_SOURCE_SOFT_TRIG);                                            /* Start Of Sequence (SOS) triggered: Software */
    HAL_ADC_Start();                                                            /* Start ADC */
    DELAY_US(C_ADC_SETTLING_TIME);
    HAL_ADC_SoftTrigger();                                                      /* Send software-trigger now */
    DEBUG_SET_IO_A();
    while ( (IO_ADC_STATUS & M_ADC_STATE) != C_ADC_STATE_WAIT_FOR_TRIGGER) {}
    DEBUG_CLR_IO_A();

    HAL_ADC_SoftTrigger();                                                      /* Send software-trigger now */
    DEBUG_SET_IO_A();
    while ( (IO_ADC_STATUS & M_ADC_STATE) != C_ADC_STATE_WAIT_FOR_TRIGGER) {}
    DEBUG_CLR_IO_A();

    HAL_ADC_SoftTrigger();                                                      /* Send software-trigger now */
    DEBUG_SET_IO_A();
    while ( (IO_ADC_STATUS & M_ADC_STATE) != C_ADC_STATE_WAIT_FOR_TRIGGER) {}
    DEBUG_CLR_IO_A();

    IO_ADC_BLOCK_X = (IO_ADC_BLOCK_X & ~M_ADC_BLOCK_COUNT) | 0x0001U;           /* EC Count = 16 */
    HAL_ADC_SoftTrigger();                                                      /* Send software-trigger now */
    DEBUG_SET_IO_A();
    while ( (IO_ADC_STATUS & M_ADC_STATE) != C_ADC_STATE_WAIT_FOR_TRIGGER) {}
    DEBUG_CLR_IO_A();

    IO_ADC_BLOCK_X = (IO_ADC_BLOCK_X & ~M_ADC_BLOCK_COUNT) | 0x0002U;           /* EC Count = 16 */
    HAL_ADC_SoftTrigger();                                                      /* Send software-trigger now */
    DEBUG_SET_IO_A();
    while ( (IO_ADC_CTRL & B_ADC_STOP) == 0U) {}
    DEBUG_CLR_IO_A();
#endif /* FALSE */

#if (_SUPPORT_ADC_IO_CHECK != FALSE)
    ADC_IO_Check();
#endif /* (_SUPPORT_ADC_IO_CHECK != FALSE) */

#if (_SUPPORT_ADC_REF_HV_CALIB != FALSE)
    ADC_ReferenceCalibration();
#endif /* (_SUPPORT_ADC_REF_HV_CALIB != FALSE) */

#if (_DEBUG_IO_ADC != FALSE)
    HAL_ADC_SetPrio(C_MLX16_ITC_PRIO2_ADC_PRIO6);                               /* ADC IRQ Priority: 6 (3..6) */
#else  /* (_DEBUG_IO_ADC != FALSE) */
    HAL_ADC_SetPrio(C_MLX16_ITC_PRIO2_ADC_PRIO4);                               /* ADC IRQ Priority: 4 (3..6) */
#endif /* (_DEBUG_IO_ADC != FALSE) */
} /* End of ADC_Init() */

/*!*************************************************************************** *
 * ADC_MCurrOffCalib
 * \brief   Motor Current Offset calibration
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Disable PWM before measure ADC Motor Current Offset.
 *          Motor PWM switched to re-circulation mode
 * *************************************************************************** *
 * - Call Hierarchy: MotorDriverStart()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 1
 * - Function calling: 2 (HAL_ADC_StopSafe(), HAL_ADC_StartSoftTrig())
 * *************************************************************************** */
void ADC_MCurrOffCalib(void)
{
    static const ADC_SDATA_t SBASE_MCURR_OFF_CALIB[] =
    {
#if defined (__MLX81160__)
        C_ADC_MCUR1_EOC,                                                        /* MMP240412-1: Improve ADC Offset measurement by skipping 1st measurement (S&H) */
        C_ADC_MCUR1_EOC,
        C_ADC_MCUR1_EOC,
        C_ADC_MCUR2_EOC,
        C_ADC_MCUR2_EOC,
#elif defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || \
      defined (__MLX81339__) || defined (__MLX81350__) || \
      defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
        C_ADC_MCUR_EOC,                                                         /* MMP240412-1: Improve ADC Offset measurement by skipping 1st measurement (S&H) */
        C_ADC_MCUR_EOC,
        C_ADC_MCUR_EOC,
        C_ADC_MCUR_EOC,
        C_ADC_MCUR_EOC,
#endif
        {.u16 = C_ADC_EOS}                                                      /* End-of-Sequence */
    };

    ADC_MCURR_RESULTS AdcMCurrResult;                                           /*!< ADC Motor Current measurement results */
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
    uint16_t u16LastDrvCtrl;                                                    /*!< Motor driver current state */
#endif /* defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__) */

    if (l_u8AdcMode > C_ADC_MODE_IDLE)
    {
        HAL_ADC_StopSafe();
    }

#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
    u16LastDrvCtrl = IO_PORT_DRV_CTRL;                                          /* Save current motor driver state */
#if (C_MOTOR_PHASES == 3)
    DRVCFG_GND_UVW();                                                           /* Switch motor driver state to (ground) recirculation */
#else
    DRVCFG_GND_TUVW();                                                          /* Switch motor driver state to (ground) recirculation */
#endif
#endif /* defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__) */

    {
        uint16_t *pu16Src = &l_au16AdcSource[0];
        uint16_t *pu16SBase = (uint16_t *)&SBASE_MCURR_OFF_CALIB[0];
        *pu16Src++ = (uint16_t)&AdcMCurrResult.u16AdcZeroCurrent_1;
        do
        {
            *pu16Src++ = *pu16SBase++;
        } while (pu16SBase < (uint16_t *)&SBASE_MCURR_OFF_CALIB[sizeof(SBASE_MCURR_OFF_CALIB) / sizeof(uint16_t)]);
    }

#if defined (__MLX81160__)
    IO_PORT_CURR_SENS1 |= B_PORT_CURR_SENS1_CSA1_ZERO;                          /* Shorten current sense input 1 */
    IO_PORT_CURR_SENS2 |= B_PORT_CURR_SENS2_CSA2_ZERO;                          /* Shorten current sense input 2 */
#elif defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
    IO_PORT_DRV_OUT |= B_PORT_DRV_OUT_ENABLE_CSA;                               /* Enable Current Sense amplifier */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    IO_PORT_CURR_SENS |= B_PORT_CURR_SENS_CSA_ZERO;                             /* Shorten current sense input */
#endif
    DELAY_US(10);

    (void)HAL_ADC_StartSoftTrig(C_ADC_STATE_IDLE);

    /* Current offset */
    l_u16CurrentZeroOffset = ((AdcMCurrResult.u16AdcZeroCurrent_1 + AdcMCurrResult.u16AdcZeroCurrent_2) +
                              (AdcMCurrResult.u16AdcZeroCurrent_3 + AdcMCurrResult.u16AdcZeroCurrent_4)) / 4;
#if defined (__MLX81160__)
    IO_PORT_CURR_SENS1 &= ~B_PORT_CURR_SENS1_CSA1_ZERO;                         /* Open current sense input 1 */
    IO_PORT_CURR_SENS2 &= ~B_PORT_CURR_SENS2_CSA2_ZERO;                         /* Open current sense input 2 */
#elif defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
    IO_PORT_DRV_CTRL = u16LastDrvCtrl;                                          /* Restore motor driver control state */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    IO_PORT_CURR_SENS &= ~B_PORT_CURR_SENS_CSA_ZERO;                            /* Open current sense input */
#endif
} /* End of ADC_MCurrOffCalib() */

/*!*************************************************************************** *
 * ADC_Conv_Vsupply
 * \brief   Get Supply-voltage [10mV], based on ADC sampling of chip supply pin
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (uint16_t) Supply voltage [10mV]
 * *************************************************************************** *
 * \details Set local and return IC supply voltage in 10mV units
 * Note:    This function is not allowed to be called during Non Volatile Memory Write
 *          (due to Non Volatile Memory reads) (_SUPPORT_ADC_TEMPCOMP).
 * *************************************************************************** *
 * - Call Hierarchy: AppSupplyCheck(), HandleDiagnosticUVOV()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 3 (HAL_ADC_Conv_Vsupply(), Get_RawVsupplyChip(), Get_RawTemperature())
 * *************************************************************************** */
uint16_t ADC_Conv_Vsupply(void)
{
    return ( HAL_ADC_Conv_Vsupply(Get_RawTemperature(), Get_RawVsupplyChip()) );
} /* End of ADC_Conv_Vsupply() */

/*!*************************************************************************** *
 * ADC_Conv_Vmotor
 * \brief   Get Motor-voltage [10mV], based on ADC sampling of Motor supply pin
 *           (VsmF)
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (uint16_t) Motor voltage [10mV]
 * *************************************************************************** *
 * \details Set local and return IC motor voltage in 10mV units
 * Note:    This function is not allowed to be called during Non Volatile Memory Write
 *          (due to Non Volatile Memory reads) (_SUPPORT_ADC_TEMPCOMP).
 * *************************************************************************** *
 * - Call Hierarchy: AppSupplyCheck(), main_Init()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 3 (HAL_ADC_Conv_Vmotor(), Get_RawTemperature(), Get_RawVmotorF())
 * *************************************************************************** */
uint16_t ADC_Conv_Vmotor(void)
{
    return ( HAL_ADC_Conv_Vmotor(Get_RawTemperature(), Get_RawVmotorF()) );
} /* End of ADC_Conv_Vmotor() */

/*!*************************************************************************** *
 * ADC_Conv_TempJ
 * \brief   Get Chip Junction temperature [C]
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16Init: FALSE: Check new temperature measurement against previous
 *                               measurement, and allow a maximum temperature increase.
 *                        TRUE: Initial measurement (no check)
 * \return  (int16_t) IC junction temperature in degrees Celsius
 * *************************************************************************** *
 * \details Set local and return IC junction temperature in degrees Celsius
 * *************************************************************************** *
 * - Call Hierarchy: AppInit(), AppTemperatureCheck(), HandleDiagnosticsOT(),
 *                   NV_TemperatureBasedWriteCountIncrease()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 2 (HAL_ADC_Conv_TempJ(), Get_RawTemperature())
 * *************************************************************************** */
int16_t ADC_Conv_TempJ(uint16_t u16Init)
{
    return ( HAL_ADC_Conv_TempJ(Get_RawTemperature(), u16Init) );
} /* End of ADC_Conv_TempJ() */

/*!*************************************************************************** *
 * ADC_Conv_Cmotor
 * \brief   Get Motor Driver Current [mA]
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (uint16_t) Motor Driver current in [mA]
 * *************************************************************************** *
 * \details Get the Motor Driver current in mA
 * Note:    This function is not allowed to be called during Non Volatile Memory Write
 *          (due to Non Volatile Memory reads) (_SUPPORT_ADC_TEMPCOMP).
 * *************************************************************************** *
 * - Call Hierarchy: AppCurrentCheck()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 3 (HAL_ADC_Conv_Cmotor(), Get_RawTemperature(), ADC_GetRawMotorDriverCurrent())
 * *************************************************************************** */
uint16_t ADC_Conv_Cmotor(void)
{
#if (_SUPPORT_PWM_POL_CORR == FALSE)
    return ( HAL_ADC_Conv_Cmotor(Get_RawTemperature(), ADC_GetRawMotorDriverCurrent(0U, 0U)) );
#elif (_SUPPORT_APP_TYPE != C_APP_SOLENOID)
    return ( HAL_ADC_Conv_Cmotor(Get_RawTemperature(), ADC_GetRawMotorDriverCurrent(Get_MicroStepIdx())) );
#else  /* (_SUPPORT_PWM_POL_CORR == FALSE) */
    return ( HAL_ADC_Conv_Cmotor(Get_RawTemperature(), ADC_GetRawMotorDriverCurrent()) );
#endif /* (_SUPPORT_PWM_POL_CORR == FALSE) */
} /* End of ADC_Conv_Cmotor() */

#if (_SUPPORT_OSD != FALSE) && (!defined (__MLX81160__) && !defined (__MLX81330__) && !defined (__MLX81339__) && !defined (__MLX81350__))
/*!*************************************************************************** *
 * ADC_OSD_Start
 * \brief   Start ADC to perform OSD Phase voltage measurement
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: AppInit()
 * - Cyclomatic Complexity: 6+1
 * - Nesting: 2
 * - Function calling: 2 (HAL_ADC_StopSafe(), HAL_ADC_StartSoftTrig())
 * *************************************************************************** */
void ADC_OSD_Start(void)
{
#if (ADC_TRIGGER_OSD == ADC_TRIGGER_OSD_SW)
#if defined (__MLX81332__) || defined (__MLX81334__)
    static uint16_t const SBASE_MOTOR_OSD[6] =
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    static uint16_t const SBASE_MOTOR_OSD[5] =
#endif
    {
#if (defined (__MLX81332__) || defined (__MLX81334__)) && _SUPPORT_ADC_REF_HV_CALIB
        C_ADC_VSMF_REF1V5,                                                      /* 20           1.5V    Chip supply voltage (divided by 21) */
        C_ADC_VPHU_REF1V5,                                                      /* 12           1.5V    Junction Temperature */
        C_ADC_VPHV_REF1V5,                                                      /* 13           1.5V    Motor current (unfiltered) */
        C_ADC_VPHW_REF1V5,                                                      /* 14           1.5V    Driver supply voltage (21:1) */
        C_ADC_VPHT_REF1V5,                                                      /* 15           1.5V    Motor current (unfiltered) */
#else  /* (defined (__MLX81332__) || defined (__MLX81334__)) && _SUPPORT_ADC_REF_HV_CALIB */
        C_ADC_VSMF,                                                             /* 20           2.5V    Chip supply voltage (divided by 21) */
        C_ADC_VPHU,                                                             /* 12           2.5V    Junction Temperature */
        C_ADC_VPHV,                                                             /* 13           2.5V    Motor current (unfiltered) */
        C_ADC_VPHW,                                                             /* 14           2.5V    Driver supply voltage (21:1) */
#if defined (__MLX81332__) || defined (__MLX81334__)
        C_ADC_VPHT,                                                             /* 15           2.5V    Motor current (unfiltered) */
#endif /* defined (__MLX81332__) || defined (__MLX81334__) */
#endif /* (defined (__MLX81332__) || defined (__MLX81334__)) && _SUPPORT_ADC_REF_HV_CALIB */
        C_ADC_EOS                                                               /* End-of-Sequence */
    };
#elif (ADC_TRIGGER_OSD == ADC_TRIGGER_OSD_CTIMER1)
#if defined (__MLX81332__) || defined (__MLX81334__)
    static const ADC_SDATA_t SBASE_MOTOR_OSD[6] =
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    static const ADC_SDATA_t SBASE_MOTOR_OSD[5] =
#endif
    {
#if (defined (__MLX81332__) || defined (__MLX81334__))
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_VSMF_HV,
                .u3AdcVref = C_ADC_VREF_HV,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_CTIMER1_INT3,
                .u1AdcReserved = 0U
            }
        },
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_U_HV,
                .u3AdcVref = C_ADC_VREF_HV,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_CTIMER1_INT3,
                .u1AdcReserved = 0U
            }
        },
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_V_HV,
                .u3AdcVref = C_ADC_VREF_HV,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_CTIMER1_INT3,
                .u1AdcReserved = 0U
            }
        },
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_W_HV,
                .u3AdcVref = C_ADC_VREF_HV,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_CTIMER1_INT3,
                .u1AdcReserved = 0U
            }
        },
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u5AdcChannel = C_ADC_PH_T_HV,
                .u3AdcVref = C_ADC_VREF_HV,
                .u5AdcTrigger = C_ADC_HW_TRIGGER_CTIMER1_INT3,
                .u1AdcReserved = 0U
            }
        },
#else  /* (defined (__MLX81332__) || defined (__MLX81334__)) */
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u1AdcSin = C_ADC_SIN_SDATA,
                .u6AdcChannel = C_ADC_VSMF_HV,
                .u1AdcType = C_ADC_TYPE_CYCLIC,
                .u6AdcTrigger = C_ADC_HW_TRIGGER_CTIMER1_INT3
            }
        },
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u1AdcSin = C_ADC_SIN_SDATA,
                .u6AdcChannel = C_ADC_PH_U_HV,
                .u1AdcType = C_ADC_TYPE_CYCLIC,
                .u6AdcTrigger = C_ADC_HW_TRIGGER_CTIMER1_INT3
            }
        },
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u1AdcSin = C_ADC_SIN_SDATA,
                .u6AdcChannel = C_ADC_PH_V_HV,
                .u1AdcType = C_ADC_TYPE_CYCLIC,
                .u6AdcTrigger = C_ADC_HW_TRIGGER_CTIMER1_INT3
            }
        },
        {
            {
                .u2AdcMarker = C_ADC_NO_SIGN,
                .u1AdcSin = C_ADC_SIN_SDATA,
                .u6AdcChannel = C_ADC_PH_W_HV,
                .u1AdcType = C_ADC_TYPE_CYCLIC,
                .u6AdcTrigger = C_ADC_HW_TRIGGER_CTIMER1_INT3
            }
        },
#endif /* (defined (__MLX81332__) || defined (__MLX81334__)) && _SUPPORT_ADC_REF_HV_CALIB */
        {.u16 = C_ADC_EOS}                                                      /* End-of-Sequence */
    };
#endif

    HAL_ADC_StopSafe();
#if (ADC_TRIGGER_OSD == ADC_TRIGGER_OSD_CTIMER1)
    IO_CTIMER1_CTRL = B_CTIMER1_STOP;
#endif /* (ADC_TRIGGER_OSD == ADC_TRIGGER_OSD_CTIMER1) */

    /* Set SBASE structure in RAM, to prevent MCU performance losses due to ADC DMA accesses from Flash */
    {
        uint16_t *pu16Src = &l_au16AdcSource[0];
        const ADC_SDATA_t *pu16SBase = &SBASE_MOTOR_OSD[0];
        *pu16Src++ = (uint16_t)&AdcOsdResult.u16AdcVsmF;                        /* First entry is the pointer to the DBASE */
        do
        {
            *pu16Src = pu16SBase->u16;
            pu16Src++;
            pu16SBase++;
        } while (pu16SBase < (ADC_SDATA_t *)&SBASE_MOTOR_OSD[sizeof(SBASE_MOTOR_OSD) / sizeof(uint16_t)]);
    }
#if (ADC_TRIGGER_OSD == ADC_TRIGGER_OSD_SW)
    HAL_ADC_Setup(/*B_ADC_ADC_WIDTH |*/                                         /* Setup Extended Count measurement */
        C_ADC_ASB_NEVER |                                                       /* Auto Standby: Never */
        C_ADC_INT_SCHEME_NOINT |                                                /* Message interrupt: No */
        B_ADC_SATURATE |                                                        /* Saturation: Enabled */
        B_ADC_NO_INTERLEAVE |                                                   /* Interleave: No */
        C_ADC_SOC_SOURCE_SOFT_TRIG |                                            /* Start Of Conversion (SOC) triggered by: Software */
        C_ADC_SOS_SOURCE_SOFT_TRIG);                                            /* Start Of Sequence (SOS) triggered: Software */
    HAL_ADC_Start();                                                            /* Start ADC */

    do
    {
        if (HAL_ADC_StartSoftTrig(C_ADC_STATE_WAIT_FOR_TRIGGER) != C_HAL_ERR_NONE)  /* Sample VsmF */
        {
            break;
        }
        if (HAL_ADC_StartSoftTrig(C_ADC_STATE_WAIT_FOR_TRIGGER) != C_HAL_ERR_NONE)  /* Sample VsmF */
        {
            break;
        }
        if (HAL_ADC_StartSoftTrig(C_ADC_STATE_WAIT_FOR_TRIGGER) != C_HAL_ERR_NONE)  /* Sample VsmF */
        {
            break;
        }
        if (HAL_ADC_StartSoftTrig(C_ADC_STATE_WAIT_FOR_TRIGGER) != C_HAL_ERR_NONE)  /* Sample VsmF */
        {
            break;
        }
        HAL_ADC_StartSoftTrig(C_ADC_STATE_IDLE);                                /* Sample VphT; Last sample */
    } while (FALSE);
#elif (ADC_TRIGGER_OSD == ADC_TRIGGER_OSD_CTIMER1)
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    IO_MLX16_ITC_MASK1_S &= ~B_MLX16_ITC_MASK1_CTIMER1_3;
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    IO_CTIMER1_CTRL = C_CTIMER1_DIV_CPU | C_CTIMER1_MODE_TIMER;
    IO_CTIMER1_TREGB = (PLL_FREQ / 166667U);

    HAL_ADC_Setup(/*B_ADC_ADC_WIDTH |*/                                         /* Setup OSD measurement */
        C_ADC_ASB_NEVER |                                                       /* Auto Standby: Never */
#if (_DEBUG_IO_ADC != FALSE)
        C_ADC_INT_SCHEME_EOC |                                                  /* Message interrupt: End-of-Frame */
#else  /* (_DEBUG_IO_ADC != FALSE) */
        C_ADC_INT_SCHEME_NOINT |                                                /* Message interrupt: No */
#endif /* (_DEBUG_IO_ADC != FALSE) */
        B_ADC_SATURATE |                                                        /* Saturation: Enabled */
        B_ADC_NO_INTERLEAVE |                                                   /* Interleave: No */
        C_ADC_SOC_SOURCE_HARD_CTRIG |                                           /* Start Of Conversion (SOC) triggered by: Hardware (HARD_CTRIG) */
        C_ADC_SOS_SOURCE_2ND_HARD_CTRIG);                                       /* Start Of Sequence (SOS) triggered: 2nd Hardware */
    HAL_ADC_ClearErrors();                                                      /* Prior to start, first clear any error flag and enable Triggers */
#if (_DEBUG_IO_ADC != FALSE)
    HAL_ADC_EnableIRQ();                                                        /* Enable ADC Interrupt */
#endif /* (_DEBUG_IO_ADC != FALSE) */
    HAL_ADC_Start();                                                            /* Start ADC */
    IO_CTIMER1_CTRL = B_CTIMER1_START;

    while ( (IO_ADC_CTRL & B_ADC_STOP) == 0U) {}
    IO_CTIMER1_CTRL = B_CTIMER1_STOP;
#if (_DEBUG_CPU_CLOCK != FALSE)
    IO_CTIMER1_TREGB = 4U;
    IO_CTIMER1_TREGA = 2U;
    IO_CTIMER1_CTRL = (C_CTIMER1_DIV_CPU | C_CTIMER1_MODE_PWM | B_CTIMER1_ENCMP); /* PWM mode */
    IO_CTIMER1_CTRL = B_CTIMER1_START;
#endif /* (_DEBUG_CPU_CLOCK != FALSE) */
#endif /* (ADC_TRIGGER_OSD == ADC_TRIGGER_OSD_CTIMER1) */

} /* End of ADC_OSD_Start() */
#endif /* (_SUPPORT_OSD != FALSE) && (!defined (__MLX81160__) && !defined (__MLX81330__) && !defined (__MLX81339__) && !defined (__MLX81350__)) */

#if (LIN_COMM != FALSE) && (_SUPPORT_MLX_DEBUG_MODE != FALSE)
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE)
/*!*************************************************************************** *
 * LinDiag_VddaVddd
 * \brief   Fill LIN Diagnostics response buffer with VDDD & VDDA measurement.
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: DfrDiagMlxdebug()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0 (Get_AdcVdda(), Get_AdcVddd())
 * *************************************************************************** */
void LinDiag_VddaVddd(void)
{
    uint16_t u16ValueVdda, u16ValueVddd;

    l_u16VddaADC = Get_AdcVdda();
    l_u16VdddADC = Get_AdcVddd();
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
    /* 10-bit unsigned ADC */
    u16ValueVdda = (uint16_t) (p_MulU32_U16byU16(l_u16VddaADC, C_GADC_VDDA) >> 10U);  /* MMP221101-1 */
    u16ValueVddd = (uint16_t) (p_MulU32_U16byU16( (l_u16VdddADC + C_OADC_VDDD), C_GADC_VDDD) >> 10U);  /* MMP221101-1 */
#elif defined (__MLX81339__) || defined (__MLX81350__)
    /* 12-bit unsigned ADC */
    u16ValueVdda = (uint16_t) (p_MulU32_U16byU16(l_u16VddaADC, C_GADC_VDDA) >> 12U);
    u16ValueVddd = (uint16_t) (p_MulU32_U16byU16(l_u16VdddADC, C_GADC_VDDD) >> 12U);
#elif defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    /* 12-bit signed ADC */
    u16ValueVdda = (uint16_t) (p_MulU32_U16byU16(l_u16VddaADC, C_GADC_VDDA) >> 11U);
    u16ValueVddd = (uint16_t) (p_MulU32_U16byU16(l_u16VdddADC, C_GADC_VDDD) >> 11U);
#endif
    StoreD1to4(u16ValueVdda, u16ValueVddd);
} /* End of LinDiag_VddaVddd() */
#endif /* (_SUPPORT_ADC_VDDA_VDDD != FALSE) */

#if (_SUPPORT_ADC_BGD != FALSE)
/*!*************************************************************************** *
 * LinDiag_VauxVbgd
 * \brief   Fill LIN Diagnostics response buffer with VAUX & VBGD measurement.
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details MMP220307-1
 * *************************************************************************** *
 * - Call Hierarchy: DfrDiagMlxdebug()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0 (Get_AdcVaux(), Get_AdcVbgd())
 * *************************************************************************** */
void LinDiag_VauxVbgd(void)
{
    uint16_t u16ValueVaux, u16ValueVbgd;

    uint16_t u16VauxADC = Get_AdcVaux();
    uint16_t u16VbgdADC = Get_AdcVbgd();
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
    /* 10-bit unsigned ADC */
    u16ValueVaux = (uint16_t) (p_MulU32_U16byU16(u16VauxADC, C_GADC_VAUX) >> 10U);
    u16ValueVbgd = (uint16_t) (p_MulU32_U16byU16(u16VbgdADC, C_GADC_VBGD) >> 10U);
#elif defined (__MLX81339__) || defined (__MLX81350__)
    /* 12-bit unsigned ADC */
    u16ValueVaux = (uint16_t) (p_MulU32_U16byU16(u16VauxADC, C_GADC_VAUX) >> 12U);
    u16ValueVbgd = (uint16_t) (p_MulU32_U16byU16(u16VbgdADC, C_GADC_VBGD) >> 12U);
#elif defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    /* 12-bit signed ADC */
    u16ValueVaux = (uint16_t) (p_MulU32_U16byU16(u16VauxADC, C_GADC_VAUX) >> 11U);
    u16ValueVbgd = (uint16_t) (p_MulU32_U16byU16(u16VbgdADC, C_GADC_VBGD) >> 11U);
#endif
    StoreD1to4(u16ValueVaux, u16ValueVbgd);
} /* End of LinDiag_VauxVbgd() */
#endif /* (_SUPPORT_ADC_BGD != FALSE) */

#if (_SUPPORT_ADC_VBOOST != FALSE)
/*!*************************************************************************** *
 * LinDiag_Vboost
 * \brief   Fill LIN Diagnostics response buffer with VBOOST measurement.
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: DfrDiagMlxdebug()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0 (Get_AdcVboost())
 * *************************************************************************** */
void LinDiag_Vboost(void)
{
    uint16_t u16Value;

    l_u16VboostADC = Get_AdcVboost();
    u16Value = p_MulDivU16_U16byU16byU16(l_u16VboostADC, Get_VboostGain(), C_VOLTGAIN_DIV);
    StoreD1to4(u16Value, 0x0000U);
} /* End of LinDiag_Vboost() */
#endif /* (_SUPPORT_ADC_VBOOST != FALSE) */
#endif /* (LIN_COMM != FALSE) && (_SUPPORT_MLX_DEBUG_MODE != FALSE) */

/* EOF */

