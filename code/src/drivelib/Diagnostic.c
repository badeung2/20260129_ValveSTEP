/*!*************************************************************************** *
 * \file        Diagnostic.c
 * \brief       MLX8133x/4x Diagnostic handling
 *
 * \note        project MLX8133x/4x
 *
 * \author      Marcel Braat
 *
 * \date        2017-08-21
 *
 * \version     2.0
 *
 * A list of functions:
 *  - Global Functions:
 *           -# DiagnosticInit()
 *           -# DiagnosticReset()
 *           -# ISR_DIAG()
 *           -# ISR_UV_VDDA()
 *           -# ISR_UV_VDDAF()  (MLX8133x)
 *           -# ISR_UV_VBOOST() (MLX8134x)
 *           -# ISR_OV_VBOOST() (MLX8134x)
 *           -# ISR_OC()
 *           -# ISR_OT()
 *           -# ISR_UV_VS()
 *           -# ISR_OV_VS()
 *           -# DiagnosticPeriodicTimerEvent()
 *  - Internal Functions:
 *           -# HandleDiagnosticsVDS()
 *           -# HandleDiagnosticsOC()
 *           -# HandleDiagnosticsOT()
 *           -# VerifyDiagnosticsUVOV()
 *           -# HandleDiagnosticsUV()
 *           -# HandleDiagnosticsOV()
 *           -# HandleDiagnosticsUVOV()
 *           -# HandleDiagnosticsVDDA()
 *           -# HandleDiagnosticsVDDAF()  (MLX8133x)
 *           -# HandleDiagnosticsVBOOST() (MLX8134x)
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

/*!*************************************************************************** */
/*                           INCLUDES                                          */
/* *************************************************************************** */
#include "AppBuild.h"                                                           /* Application Build */

#include "drivelib/AppFunctions.h"                                              /* Application Functions support */
#include "drivelib/Diagnostic.h"                                                /* Chip Protection & Diagnostics support */
#if (_SUPPORT_NV_EMERGENCY_STORE != FALSE)
#include "drivelib/NV_Functions.h"                                              /* Non Volatile Memory Functions & Layout */
#endif /* (_SUPPORT_NV_EMERGENCY_STORE != FALSE) */
#include "drivelib/NV_UserPage.h"                                               /* Non Volatile Memory User Application Page Layout */
#include "drivelib/ErrorCodes.h"                                                /* Error logging support */
#include "drivelib/GlobalVars.h"                                                /* Global Variables support */
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
#include "drivelib/MotorDriver.h"                                               /* Motor Driver support */
#elif (_SUPPORT_APP_TYPE == C_APP_RELAY)
#include "drivelib/RelayDriver.h"                                               /* Relay Driver support */
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
#include "drivelib/SolenoidDriver.h"                                            /* Solenoid Driver support */
#endif /* (_SUPPORT_APP_TYPE) */
#include "camculib/private_mathlib.h"                                           /* Private Math Library support */

#if (LIN_COMM != FALSE) && (LINPROT == LIN22_SIMPLE_PCT)
#include "commlib/LIN_2x_SIMPLE_PCT.h"
#endif /* (LIN_COMM != FALSE) && (LINPROT == LIN22_SIMPLE_PCT) */

#include "../ActADC.h"                                                          /* Application ADC support */

#include <bist_inline_impl.h>
#if (_SUPPORT_NV_TYPE == C_NV_EEPROM)
#include <eeprom_map.h>
#endif /* (_SUPPORT_NV_TYPE == C_NV_EEPROM) */
#include <lib_clock.h>
#include <sys_tools.h>
#include <trimming.h>                                                           /* CPU Clock support */

/*!*************************************************************************** */
/*                           DEFINITIONS                                       */
/* *************************************************************************** */
#if defined (__MLX81346__)
static const uint16_t au16LevelOV[4] = {2100U, 2300U, 3900U, 6200U};            /* OV levels [10mV] (MMP230818-1) */
#else
static const uint16_t au16LevelOV[4] = {2100U, 2300U, 3900U, 3900U};            /* OV levels [10mV] */
#endif

/*!*************************************************************************** */
/*                           GLOBAL & LOCAL VARIABLES                          */
/* *************************************************************************** */
#pragma space dp                                                                /* __TINY_SECTION__ */
#if (_SUPPORT_DIAG_IRQCTLR_LEVEL != FALSE)
static uint16_t l_u16MASK0 = 0U;                                                /*!< MASK0 Storage */
static uint16_t l_u16MASK1 = 0U;                                                /*!< MASK1 Storage */
static uint16_t l_u16MASK2 = 0U;                                                /*!< MASK2 Storage */
#if defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
static uint16_t l_u16MASK3 = 0U;                                                /*!< MASK3 Storage */
#endif /* defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */
#pragma space none                                                              /* __TINY_SECTION__ */

#pragma space nodp                                                              /* __NEAR_SECTION__ */
#if (_SUPPORT_DIAG_RINNEN_ROBUSTNESS != FALSE)
static uint8_t l_u8OC_Count = 0U;                                               /*!< OC Count */
static uint8_t l_u8VDS_Count = 0U;                                              /*!< VDS Count */
#endif /* (_SUPPORT_DIAG_RINNEN_ROBUSTNESS != FALSE) */
#if defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
static uint8_t l_u8VDDAF_Count = 0U;                                            /*!< VDD-AF Count */
static uint16_t l_u16DrvProt = 0U;                                              /*!< DRV-Protection */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
static uint8_t l_u8VBOOST_Count = 0U;                                           /*!< VBOOST Count */
static uint16_t l_u16Drv1Prot = 0U;                                             /*!< DRV1-Protection */
static uint16_t l_u16Drv3Prot = 0U;                                             /*!< DRV3-Protection */
#endif
#endif /* (_SUPPORT_DIAG_IRQCTLR_LEVEL != FASLE) */
#pragma space none                                                              /* __NEAR_SECTION__ */

#if !defined (__MLX81160__) && (_SUPPORT_DRV_PROT_VDS != FALSE)
/*!*************************************************************************** *
 * HandleDiagnosticsVDS
 * \brief   Handle Diagnostic VDS Event
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: ISR_DIAG() or ISR_VDS()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 2
 * - Function calling: 2 (MotorDriverStop(), SetLastError())
 * *************************************************************************** */
static void HandleDiagnosticsVDS(void)
{
#if (_DEBUG_DIAG_VDS != FALSE)
    DEBUG_SET_IO_B();
#endif /* (_DEBUG_DIAG_VDS != FALSE) */

#if (_SUPPORT_LOG_ERRORS != FALSE)
    uint16_t u16VdsErrorMask = (IO_PORT_DIAG_IN & (M_PORT_DIAG_IN_OV_HS_VDS_MEM | M_PORT_DIAG_IN_OV_LS_VDS_MEM));  /* VDSx_HS|LS */
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
#if (_SUPPORT_DIAG_RINNEN_ROBUSTNESS != FALSE)
    if (l_u8VDS_Count != 0U)
#endif /* (_SUPPORT_DIAG_RINNEN_ROBUSTNESS != FALSE) */
    {
        g_e8ErrorElectric = (uint8_t)C_ERR_VDS;
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
        MotorDriverStop( (uint16_t)C_STOP_WO_HOLDING);                          /* VDS-error (w/o holding) */
#if (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_ROTOR)
        g_u16TargetPosition = g_u16ActualPosition;
#elif (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_SHAFT)
        g_u16TargetShaftAngle = g_u16ActualShaftAngle;
#endif /* (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_ROTOR) */
#elif (_SUPPORT_APP_TYPE == C_APP_RELAY)
        RelayDriverOff();                                                       /* VDS-error */
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
        SolenoidDriverDeactivate();                                             /* VDS-error */
#endif /* (_SUPPORT_APP_TYPE) */
#if (_SUPPORT_LOG_ERRORS != FALSE)
        SetLastError(C_ERR_DIAG_VDS | C_ERR_EXTW);
        SetLastError(u16VdsErrorMask);                                          /* VDSx_HS|LS */
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
#if (_SUPPORT_DRV_PROT_VDS != FALSE)
        IO_PORT_DRV1_PROT |= (B_PORT_DRV1_PROT_DIS_OV_LS_VDS | B_PORT_DRV1_PROT_DIS_OV_HS_VDS);  /* Clear VDS flag */
        IO_PORT_DRV1_PROT &= ~(B_PORT_DRV1_PROT_DIS_OV_LS_VDS | B_PORT_DRV1_PROT_DIS_OV_HS_VDS);
#endif /* (_SUPPORT_DRV_PROT_VDS != FALSE) */
    }
#if (_SUPPORT_DIAG_RINNEN_ROBUSTNESS != FALSE)
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
    else
    {
        IO_PORT_SUPP_CFG = (B_PORT_SUPP_CFG_OVT_FILT_SEL |                      /* Over Temperature: 100-200us */
                            B_PORT_SUPP_CFG_OV_VS_FILT_SEL |                    /* Over Voltage VS: 100-200us */
                            B_PORT_SUPP_CFG_UV_VS_FILT_SEL |                    /* Under Voltage VS: 100-200us */
#if defined (__MLX81330A01__) || defined (__MLX81332A01__)
                            /* Under Voltage VDDAF: 1-2us */
#else
                            C_PORT_SUPP_CFG_UV_VDDAF_FILT_SEL_00 |              /* Under Voltage VDDAF: 1-2us */
#endif
                            B_PORT_SUPP_CFG_UV_VDDA_FILT_SEL);                  /* Under Voltage VDDA: 100-200us */
        IO_PORT_DRV1_PROT |= (B_PORT_DRV1_PROT_DIS_OV_LS_VDS | B_PORT_DRV1_PROT_DIS_OV_HS_VDS);  /* Clear VDS flag (Disable VDS HS & LS) */
        {
            uint16_t u16DrvCtrl = IO_PORT_DRV_CTRL;
            uint16_t u16DrvOutput = IO_PORT_DRV_OUT & M_PORT_DRV_OUT_ENABLE_DRV;
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
            MotorDriverConfig(FALSE);
            MotorDriverConfig(TRUE);
#elif (_SUPPORT_APP_TYPE == C_APP_RELAY)
            RelayDriverConfig(FALSE);
            RelayDriverConfig(TRUE);
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
            SolenoidDriverConfig(FALSE);
            SolenoidDriverConfig(TRUE);
#endif /* (_SUPPORT_APP_TYPE) */
            IO_PORT_DRV_CTRL = u16DrvCtrl;
            IO_PORT_DRV_OUT |= u16DrvOutput;
        }
        IO_PORT_DRV1_PROT &= ~(B_PORT_DRV1_PROT_DIS_OV_LS_VDS | B_PORT_DRV1_PROT_DIS_OV_HS_VDS);  /* Enable VDS HS & LS */
        if ( (IO_MLX16_ITC_PEND0_S & B_MLX16_ITC_PEND0_UV_VDDAF) != 0U)         /* MMP200706-1 */
        {
            /* Ignore VDS error in case of VDDAF-UV */
            IO_PORT_DRV1_PROT |= B_PORT_DRV1_PROT_DIS_UV_VDDAF;                 /* Clear VDDAF flag */
            IO_PORT_DRV1_PROT &= ~B_PORT_DRV1_PROT_DIS_UV_VDDAF;
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_WRN_VREF);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
        }
        else if ( (IO_MLX16_ITC_PEND0_S & B_MLX16_ITC_PEND0_UV_VDDA) != 0U)     /* MMP200706-1 */
        {
            /* Ignore VDS error in case of VDDA-UV */
            IO_PORT_DRV1_PROT |= B_PORT_DRV1_PROT_DIS_UV_VDDA;                  /* Clear VDDA flag */
            IO_PORT_DRV1_PROT &= ~B_PORT_DRV1_PROT_DIS_UV_VDDA;
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_WRN_VDDA_UV);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
        }
        else
        {
            l_u8VDS_Count = 2U;                                                 /* Delay: 1 - 2 (Background) STimer-ticks */
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_WRN_DIAG_VDS | C_ERR_EXTW);
            SetLastError(u16VdsErrorMask);                                      /* VDSx_HS|LS */
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
        }
        IO_PORT_SUPP_CFG = (B_PORT_SUPP_CFG_OVT_FILT_SEL |                      /* Over Temperature: 100-200us */
                            B_PORT_SUPP_CFG_OV_VS_FILT_SEL |                    /* Over Voltage VS: 100-200us */
                            B_PORT_SUPP_CFG_UV_VS_FILT_SEL |                    /* Under Voltage VS: 100-200us */
#if defined (__MLX81330A01__) || defined (__MLX81332A01__)
                            B_PORT_SUPP_CFG_UV_VDDAF_FILT_SEL |                 /* Under Voltage VDDAF: 100-200us */
#else
                            C_PORT_SUPP_CFG_UV_VDDAF_FILT_SEL_11 |              /* Under Voltage VDDAF: 100-110us */
#endif
                            B_PORT_SUPP_CFG_UV_VDDA_FILT_SEL);                  /* Under Voltage VDDA: 100-200us */
    }
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    else
    {
        IO_PORT_DRV1_PROT |= (B_PORT_DRV1_PROT_DIS_OV_LS_VDS | B_PORT_DRV1_PROT_DIS_OV_HS_VDS);  /* Clear VDS flag (Disable VDS HS & LS) */
        MotorDriverConfig(FALSE);
        MotorDriverConfig(TRUE);
        DRVCFG_ENA();                                                           /* Enable the driver and the PWM phase W, V and U */
        IO_PORT_DRV1_PROT &= ~(B_PORT_DRV1_PROT_DIS_OV_LS_VDS | B_PORT_DRV1_PROT_DIS_OV_HS_VDS);  /* Enable VDS HS & LS */
        if ( (IO_MLX16_ITC_PEND0_S & B_MLX16_ITC_PEND0_UV_VDDA) != 0U)          /* MMP200706-1 */
        {
            /* Ignore VDS error in case of VDDA-UV */
            IO_PORT_DRV1_PROT |= B_PORT_DRV1_PROT_DIS_UV_VDDA;                  /* Clear VDDA flag */
            IO_PORT_DRV1_PROT &= ~B_PORT_DRV1_PROT_DIS_UV_VDDA;
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_WRN_VDDA_UV);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
        }
        else
        {
            l_u8VDS_Count = 2U;                                                 /* Delay: 1 - 2 Ticks */
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_WRN_DIAG_VDS);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
            DELAY(C_DELAY_mPWM);  /*lint !e522 */                               /* Wait for ESD pulse to be gone and a new ADC measurement have been take place */
        }
    }
#endif
#endif /* (_SUPPORT_DIAG_RINNEN_ROBUSTNESS != FALSE) */
#if (_DEBUG_DIAG_VDS != FALSE)
    DEBUG_CLR_IO_B();
#endif /* (_DEBUG_DIAG_VDS != FALSE) */
} /* End of HandleDiagnosticsVDS() */
#endif /* !defined (__MLX81160__) && (_SUPPORT_DRV_PROT_VDS != FALSE) */

#if (_SUPPORT_DRV_PROT_OC != FALSE) || (_SUPPORT_DIAG_OC != FALSE)
/*!*************************************************************************** *
 * HandleDiagnosticsOC
 * \brief   Handle Diagnostic Over Current Event
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: ISR_DIAG() or ISR_OC()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 2 (MotorDriverStop(), SetLastError())
 * *************************************************************************** */
static void HandleDiagnosticsOC(void)
{
#if (_DEBUG_DIAG_OC != FALSE)
    DEBUG_SET_IO_C();
#endif /* (_DEBUG_DIAG_OC != FALSE) */
#if (_SUPPORT_DIAG_RINNEN_ROBUSTNESS != FALSE)
    if (l_u8OC_Count != 0U)
#endif /* (_SUPPORT_DIAG_RINNEN_ROBUSTNESS != FALSE) */
    {
        g_e8ErrorElectric = (uint8_t)C_ERR_MOTOR_OVER_CURRENT;
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
        MotorDriverStop( (uint16_t)C_STOP_WO_HOLDING);                          /* Over-current (w/o holding) */
#if (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_ROTOR)
        g_u16TargetPosition = g_u16ActualPosition;
#elif (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_SHAFT)
        g_u16TargetShaftAngle = g_u16ActualShaftAngle;
#endif /* (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_ROTOR) */
#elif (_SUPPORT_APP_TYPE == C_APP_RELAY)
        RelayDriverOff();                                                       /* Over-current */
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
        SolenoidDriverDeactivate();                                             /* Over-current */
#endif /* (_SUPPORT_APP_TYPE) */
#if (_SUPPORT_LOG_ERRORS != FALSE)
        SetLastError(C_ERR_DIAG_OVER_CURRENT | C_ERR_EXTW);
        SetLastError(g_u16MotorCurrentPeak);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
    }
#if (_SUPPORT_DIAG_RINNEN_ROBUSTNESS != FALSE)
    else if ( (IO_MLX16_ITC_PEND0_S & B_MLX16_ITC_PEND0_UV_VDDA) != 0U)         /* MMP200706-1 */
    {
        /* Ignore VDS error in case of VDDA-UV */
    }
    else
    {
        l_u8OC_Count = 2U;                                                      /* Delay: 1 - 2 Ticks */
#if (_SUPPORT_LOG_ERRORS != FALSE)
        SetLastError(C_WRN_DIAG_OVER_CURRENT | C_ERR_EXTW);
        SetLastError(g_u16MotorCurrentPeak);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
        DELAY(C_DELAY_mPWM);  /*lint !e522 */                                   /* Wait for ESD pulse to be gone and a new ADC measurement have been take place */
    }
#endif /* (_SUPPORT_DIAG_RINNEN_ROBUSTNESS != FALSE) */
#if (_SUPPORT_DRV_PROT_OC != FALSE)
    IO_PORT_DRV1_PROT |= B_PORT_DRV1_PROT_DIS_OC;                               /* Clear OC flag */
    IO_PORT_DRV1_PROT &= ~B_PORT_DRV1_PROT_DIS_OC;
#endif /* (_SUPPORT_DRV_PROT_OC != FALSE) */
#if (_DEBUG_DIAG_OC != FALSE)
    DEBUG_CLR_IO_C();
#endif /* (_DEBUG_DIAG_OC != FALSE) */
} /* End of HandleDiagnosticsOC() */
#endif /* (_SUPPORT_DRV_PROT_OC != FALSE) || (_SUPPORT_DIAG_OC != FALSE) */

#if (_SUPPORT_DRV_PROT_OT != FALSE) || (_SUPPORT_DIAG_OT != FALSE)
/*!*************************************************************************** *
 * HandleDiagnosticsOT
 * \brief   Handle Diagnostic Over Temperature Event
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: DiagnosticInit(), ISR_DIAG() or ISR_OT()
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 1
 * - Function calling: 4 (GetChipTemperature(), MeasureVsupplyAndTemperature(),
 *                        MotorDriverStop(), SetLastError())
 * *************************************************************************** */
static void HandleDiagnosticsOT(void)
{
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
#if (_SUPPORT_CPU_HALT != FALSE)
    if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) != C_MOTOR_STATUS_STOP)
#else  /* (_SUPPORT_CPU_HALT != FALSE) */
    if ( (g_e8MotorStatus & C_MOTOR_STATUS_MASK) != C_MOTOR_STATUS_STOP)
#endif /* (_SUPPORT_CPU_HALT != FALSE) */
#elif (_SUPPORT_APP_TYPE == C_APP_RELAY)
#if (_SUPPORT_DUAL_RELAY == FALSE)
    if ( (g_e8RelayStatus & (uint8_t)C_RELAY_STATUS_MASK) != (uint8_t)C_RELAY_STATUS_OFF)
#else  /* (_SUPPORT_DUAL_RELAY == FALSE) */
    if ( ((g_e8RelayStatusA & (uint8_t)C_RELAY_STATUS_MASK) != (uint8_t)C_RELAY_STATUS_OFF) ||
         ((g_e8RelayStatusB & (uint8_t)C_RELAY_STATUS_MASK) != (uint8_t)C_RELAY_STATUS_OFF) )
#endif /* (_SUPPORT_DUAL_RELAY == FALSE) */
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
    if ( (g_e8SolenoidStatus & C_SOLENOID_STATUS_MASK) != C_SOLENOID_STATUS_DEACTIVATED)
#endif /* (_SUPPORT_APP_TYPE) */
    {
        DELAY(C_DELAY_mPWM);  /*lint !e522 */
    }
    else
    {
        ADC_MeasureVsupplyAndTemperature();
    }
    (void)ADC_Conv_TempJ(FALSE);
    if (Get_ChipTemperature() > C_CHIP_OVERTEMP_LEVEL)
    {
        /* Over-temperature level */
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
        MotorDriverStop( (uint16_t)C_STOP_WO_HOLDING);                          /* Over-temperature (w/o holding) */
#if (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_ROTOR)
        g_u16TargetPosition = g_u16ActualPosition;
#elif (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_SHAFT)
        g_u16TargetShaftAngle = g_u16ActualShaftAngle;
#endif /* (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_ROTOR) */
#elif (_SUPPORT_APP_TYPE == C_APP_RELAY)
        RelayDriverOff();                                                       /* Over-temperature */
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
        SolenoidDriverDeactivate();                                             /* Over-temperature */
#endif /* (_SUPPORT_APP_TYPE) */
        g_e8ErrorOverTemperature = (uint8_t)C_ERR_OTEMP_SHUTDOWN;
#if (_SUPPORT_OT_PERMANENT_ERROR != FALSE)
        if ( (g_e8ErrorElectric & (uint8_t)(C_ERR_PERMANENT | C_ERR_RPT_OVER_TEMPERATURE) == 0U)
        {
            g_u8OverTemperatureCount++;
            if (g_u8OverTemperatureCount >= (uint8_t)C_OVERTEMP_TO_PERMDEFECT_THRSHLD)
            {
                /* Turn off motor-driver; Permanent Electric Defect - Repeated Over Temperature */
                MotorDriverPermanentError( (uint8_t)(C_ERR_PERMANENT | C_ERR_RPT_OVER_TEMPERATURE));
            }
        }
#endif /* (_SUPPORT_OT_PERMANENT_ERROR != FALSE) */
#if (_SUPPORT_LOG_ERRORS != FALSE)
        SetLastError(C_ERR_DIAG_OVER_TEMP);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
#if (_SUPPORT_DEGRADED_MODE != FALSE)
        g_e8DegradeStatus = TRUE;
#endif /* (_SUPPORT_DEGRADED_MODE != FALSE) */
    }
} /* End of HandleDiagnosticsOT() */
#endif /* (_SUPPORT_DRV_PROT_OT != FALSE) || (_SUPPORT_DIAG_OT != FALSE) */

/*!*************************************************************************** *
 * HandleDiagnosticsUVOV
 * \brief   Handle Diagnostic under or Over Event
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] e8DiagVoltage: Voltage Diagnostics (UV and/or OV)
 * \return  -
 * *************************************************************************** *
 * \details Protect application / IC.
 * *************************************************************************** *
 * - Call Hierarchy: VerifyDiagnosticsUVOV(), HandleDiagnosticsUV(), HandleDiagnosticsOV()
 * - Cyclomatic Complexity: 3+1
 * - Nesting: 3
 * - Function calling: 1 (MotorDriverStop())
 * *************************************************************************** */
static void HandleDiagnosticsUVOV(uint8_t e8DiagVoltage)
{
    g_e8ErrorVoltage = (uint8_t)e8DiagVoltage;
    g_e8ErrorVoltageComm = (uint8_t)e8DiagVoltage;
#if (_SUPPORT_DEGRADED_MODE != FALSE)
    if (g_e8MotorRequest != (uint8_t)C_MOTOR_REQUEST_NONE)
    {
        g_e8DegradedMotorRequest = g_e8MotorRequest;
    }
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
    else if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) != C_MOTOR_STATUS_STOP)
#elif (_SUPPORT_APP_TYPE == C_APP_RELAY)
#if (_SUPPORT_DUAL_RELAY == FALSE)
    else if ( (g_e8RelayStatus & (uint8_t)C_RELAY_STATUS_MASK) != (uint8_t)C_RELAY_STATUS_OFF)
#else  /* (_SUPPORT_DUAL_RELAY == FALSE) */
    else if ( ((g_e8RelayStatusA & (uint8_t)C_RELAY_STATUS_MASK) != (uint8_t)C_RELAY_STATUS_OFF) ||
              ((g_e8RelayStatusB & (uint8_t)C_RELAY_STATUS_MASK) != (uint8_t)C_RELAY_STATUS_OFF) )
#endif /* (_SUPPORT_DUAL_RELAY == FALSE) */
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
    else if ( (g_e8SolenoidStatus & C_SOLENOID_STATUS_MASK) != C_SOLENOID_STATUS_DEACTIVATED)
#endif /* (_SUPPORT_APP_TYPE) */
    {
        /* Enter degraded-mode; Stop motor and resume when voltage decreases below upper-application threshold or raise above lower-application threshold */
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
        g_e8DegradedMotorRequest = (uint8_t)C_MOTOR_REQUEST_START;
#elif (_SUPPORT_APP_TYPE == C_APP_RELAY)
        g_e8DegradedMotorRequest = (uint8_t)C_RELAYS_REQUEST_OFF;
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
        g_e8DegradedMotorRequest = (uint8_t)C_SOLENOID_REQUEST_DEACTIVATE;
#endif /* (_SUPPORT_APP_TYPE) */
    }
    else if (g_e8DegradedMotorRequest == (uint8_t)C_MOTOR_REQUEST_NONE)
    {
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
        g_e8DegradedMotorRequest = (uint8_t)C_MOTOR_REQUEST_STOP;
#elif (_SUPPORT_APP_TYPE == C_APP_RELAY)
        g_e8DegradedMotorRequest = (uint8_t)C_RELAYS_REQUEST_OFF;
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
        g_e8DegradedMotorRequest = (uint8_t)C_SOLENOID_REQUEST_DEACTIVATE;
#endif /* (_SUPPORT_APP_TYPE) */
    }
    else
    {
        /* Nothing */
    }
    g_e8DegradeStatus = TRUE;
#endif /* (_SUPPORT_DEGRADED_MODE != FALSE) */
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
#if (_SUPPORT_FAST_STOP != FALSE)                                               /* MMP231121-1 */
    if (e8DiagVoltage == (uint8_t)C_ERR_VOLTAGE_UNDER)
    {
        MotorDriverStop( (uint16_t)C_STOP_FAST_STOP);                           /* Under-voltage: Fast Stop */
    }
    else
#endif /* (_SUPPORT_FAST_STOP != FALSE) */
    {
        MotorDriverStop( (uint16_t)C_STOP_EMERGENCY);                           /* Over-voltage: Normal Stop */
    }
#elif (_SUPPORT_APP_TYPE == C_APP_RELAY)
    RelayDriverOff();                                                           /* Over-voltage */
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
    SolenoidDriverDeactivate();                                                 /* Over-voltage */
#endif /* (_SUPPORT_APP_TYPE) */

#if (_SUPPORT_NV_EMERGENCY_STORE != FALSE)
    if (e8DiagVoltage == (uint8_t)C_ERR_VOLTAGE_UNDER)
    {
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
        NV_AppStore(g_u16ActualPosition, g_e8ErrorElectric);
#else  /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
        NV_AppStore(g_u16ActualMotorSpeedRPM, g_e8ErrorElectric);
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
    }
#endif /* (_SUPPORT_NV_EMERGENCY_STORE != FALSE) */
} /* End of HandleDiagnosticsUVOV() */

/*!*************************************************************************** *
 * VerifyDiagnosticsUVOV
 * \brief   Verify Diagnostic under or Over Voltage Event
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Verify the supply voltage by using an ADC measurement.
 * *************************************************************************** *
 * - Call Hierarchy: DiagnosticInit(), ISR_DIAG() or ISR_UV_OV_VS()
 * - Cyclomatic Complexity: 6+1
 * - Nesting: 3
 * - Function calling: 4 (MeasureVsupplyAndTemperature(), HandleDiagnosticsUVOV(),
 *                        SetLastError(), GetVsupplyChip())
 * *************************************************************************** */
static void VerifyDiagnosticsUVOV(void)
{
#if defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    if ( (IO_PORT_SUPP_IN & ((B_PORT_SUPP_IN_OV_VSM_SYNC | B_PORT_SUPP_IN_OV_VSM_IT) | /* VSM OV */
                             (B_PORT_SUPP_IN_UV_VSM_SYNC | B_PORT_SUPP_IN_UV_VSM_IT))) != 0U)  /* VSM UV */
#else
    if ( (IO_PORT_SUPP_IN & ((B_PORT_SUPP_IN_OV_VS_SYNC | B_PORT_SUPP_IN_OV_VS_IT) | /* VS OV */
                             (B_PORT_SUPP_IN_UV_VS_SYNC | B_PORT_SUPP_IN_UV_VS_IT))) != 0U)  /* VS UV */
#endif
    {
        uint8_t e8DiagVoltage = (uint8_t)C_ERR_VOLTAGE_IN_RANGE;
        uint16_t u16SupplyVoltage;

#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
#if (_SUPPORT_CPU_HALT != FALSE)
        if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) != C_MOTOR_STATUS_STOP)
#else  /* (_SUPPORT_CPU_HALT != FALSE) */
        if ( (g_e8MotorStatus & C_MOTOR_STATUS_MASK) != C_MOTOR_STATUS_STOP)
#endif /* (_SUPPORT_CPU_HALT != FALSE) */
#elif (_SUPPORT_APP_TYPE == C_APP_RELAY)
#if (_SUPPORT_DUAL_RELAY == FALSE)
        if ( (g_e8RelayStatus & (uint8_t)C_RELAY_STATUS_MASK) != (uint8_t)C_RELAY_STATUS_OFF)
#else  /* (_SUPPORT_DUAL_RELAY == FALSE) */
        if ( ((g_e8RelayStatusA & (uint8_t)C_RELAY_STATUS_MASK) != (uint8_t)C_RELAY_STATUS_OFF) ||
             ((g_e8RelayStatusB & (uint8_t)C_RELAY_STATUS_MASK) != (uint8_t)C_RELAY_STATUS_OFF) )
#endif /* (_SUPPORT_DUAL_RELAY == FALSE) */
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
        if ( (g_e8SolenoidStatus & C_SOLENOID_STATUS_MASK) != C_SOLENOID_STATUS_DEACTIVATED)
#endif /* (_SUPPORT_APP_TYPE) */
        {
            /* Average between two supply-voltage measurements */
            DELAY(C_DELAY_mPWM);  /*lint !e522 */                               /* Wait for ESD pulse to be gone and a new ADC measurement have been take place */
        }
        else
        {
            ADC_MeasureVsupplyAndTemperature();
        }
#if defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
        /* UV is based on VSM (MMP220818-1) */
        u16SupplyVoltage = ADC_Conv_Vmotor();
#else
        /* UV is based on VS */
        u16SupplyVoltage = ADC_Conv_Vsupply();
#endif
#if (FPLL == 32000) && !defined (__MLX81339__) && !defined (__MLX81350__)       /* MMP200319-1 */
        /* Restore 32MHz with extra flash WS */
        if (u16SupplyVoltage < 450U)
        {
            RC_Settings_t CpuSpeed;

            CpuSpeed.u = EE_MS_TRIM8_VALUE;
#if (_SUPPORT_APP_USER_MODE != FALSE)
            ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
            SetSystemSpeed(CpuSpeed, 0U);
#if (_SUPPORT_APP_USER_MODE != FALSE)
            EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        }
#endif /* (FPLL == 32000) && !defined (__MLX81339__) && !defined (__MLX81350__) */

#if (_SUPPORT_FAST_STOP != FALSE)                                               /* (MMP231121-1) */
        if (u16SupplyVoltage < 950U)
        {
            IO_PORT_MISC_OUT = (IO_PORT_MISC_OUT & ~IO_PORT_MISC_OUT) |
                               (NV_IC_UV_LEVEL * C_PORT_MISC_OUT_PRUV_0);       /* Restore UV-Level: 4V, 5V, 6V, 7V, 8V, 9V (0-5) */
#else  /* (_SUPPORT_FAST_STOP != FALSE) */
        if (u16SupplyVoltage < (450U + (NV_IC_UV_LEVEL * 100U)) )
        {
#endif /* (_SUPPORT_FAST_STOP != FALSE) */
            /* Chip under-voltage */
            e8DiagVoltage = (uint8_t)C_ERR_VOLTAGE_UNDER;
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_DIAG_UNDER_VOLT | C_ERR_EXTW);
            SetLastError(u16SupplyVoltage);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
        }
        else if (u16SupplyVoltage > au16LevelOV[NV_IC_OV_LEVEL])
        {
            /* Chip over-voltage */
            e8DiagVoltage = (uint8_t)C_ERR_VOLTAGE_OVER;
#if (_SUPPORT_LOG_ERRORS != FALSE)
            SetLastError(C_ERR_DIAG_OVER_VOLT | C_ERR_EXTW);
            SetLastError(u16SupplyVoltage);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
        }
        else
        {
            /* Nothing */
        }
        if (e8DiagVoltage != (uint8_t)C_ERR_VOLTAGE_IN_RANGE)
        {
            HandleDiagnosticsUVOV(e8DiagVoltage);
        }
    }
} /* End of VerifyDiagnosticsUVOV() */

#if (_SUPPORT_DRV_PROT_UV_VS != FALSE)
/*!*************************************************************************** *
 * HandleDiagnosticsUV
 * \brief   Handle Diagnostic under Event
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Protect application / IC.
 * *************************************************************************** *
 * - Call Hierarchy: ISR_DIAG()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 1 (HandleDiagnosticsUVOV())
 * *************************************************************************** */
static void HandleDiagnosticsUV(void)
{
    HandleDiagnosticsUVOV(C_ERR_VOLTAGE_UNDER);
#if (_SUPPORT_LOG_ERRORS != FALSE)
    SetLastError(C_ERR_DIAG_UNDER_VOLT);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
} /* End of HandleDiagnosticsUV() */
#endif /* (_SUPPORT_DRV_PROT_UV_VS != FALSE) */

#if (_SUPPORT_DRV_PROT_OV_VS != FALSE)
/*!*************************************************************************** *
 * HandleDiagnosticsUV
 * \brief   Handle Diagnostic over Event
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Protect application / IC.
 * *************************************************************************** *
 * - Call Hierarchy: ISR_DIAG()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 1 (HandleDiagnosticsUVOV())
 * *************************************************************************** */
static void HandleDiagnosticsOV(void)
{
    HandleDiagnosticsUVOV(C_ERR_VOLTAGE_OVER);
#if (_SUPPORT_LOG_ERRORS != FALSE)
    SetLastError(C_ERR_DIAG_OVER_VOLT);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
} /* End of HandleDiagnosticsOV() */
#endif /* (_SUPPORT_DRV_PROT_OV_VS != FALSE) */

/*!*************************************************************************** *
 * HandleDiagnosticsVDDA
 * \brief   8133x: Handle Diagnostic VDDA Under voltage (3.3V)
 *          8134x: Handle Diagnostic VDDA Under, Over Voltage and Over Current (3.3V)
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: DiagnosticInit(), ISR_DIAG() or ISR_UV_VDDA()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 1 (SetLastError())
 * *************************************************************************** */
void HandleDiagnosticsVDDA(void)
{
#if (_DEBUG_DIAG_UV_VDDA != FALSE)
    DEBUG_SET_IO_A();
#endif /* (_DEBUG_DIAG_UV_VDDA != FALSE) */

#if defined (__MLX81160__)
    l_u16DrvProt = IO_PORT_DRV1_PROT;                                           /* Save Protection mechanism state */
    /* Disable all protection mechanism as VDDA reached UV level */
    IO_PORT_DRV1_PROT = (B_PORT_DRV1_PROT_DIS_OC_VDDA
                         | B_PORT_DRV1_PROT_DIS_OV_VDDA
                         | B_PORT_DRV1_PROT_DIS_OC
                         | B_PORT_DRV1_PROT_DIS_UV_VDDAF
                         | B_PORT_DRV1_PROT_DIS_UV_VDDA
                         | B_PORT_DRV1_PROT_DIS_UV_VSM
                         | B_PORT_DRV1_PROT_DIS_OV_VSM
                         | B_PORT_DRV1_PROT_DIS_OVT);
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    IO_MLX16_ITC_PEND0_S = (B_MLX16_ITC_PEND0_UV_VDDA                           /* Clear VDDA pending */
                            | B_MLX16_ITC_PEND0_UV_VSM                          /* Clear VSM pending UV (MMP220818-1) */
                            | B_MLX16_ITC_PEND0_UV_VDDAF                        /* Clear VDDAF pending UV */
                            | B_MLX16_ITC_PEND0_OVT                             /* Clear pending Over-temperature */
                            | B_MLX16_ITC_PEND0_OVC0                            /* Clear pending Over-current 0 */
                            | B_MLX16_ITC_PEND0_OVC1                            /* Clear pending Over-current 1 */
                            | B_MLX16_ITC_PEND0_OC_VDDA                         /* Clear pending Over-current VDDA */
                            );
    IO_MLX16_ITC_PEND2_S = (B_MLX16_ITC_PEND2_OV_VSM                            /* Clear pending Over-voltage (MMP220818-1) */
                            | B_MLX16_ITC_PEND2_DIAG);                          /* Clear diagnostics pending */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#elif defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
#if (_SUPPORT_DIAG_VDDA_UV_SLEEP != FALSE)
    /* SLEEP */
    AppSleep();
#elif (_SUPPORT_DIAG_VDDA_UV_RESET != FALSE)
    AppReset();
#elif (_SUPPORT_DIAG_VDDA_UV_WAKE_UP_TIMER != FALSE)
    AppSleepWithWakeUpTimer();
#else  /* (_SUPPORT_DIAG_VDDA_UV_SLEEP != FALSE) */
    l_u16DrvProt = IO_PORT_DRV1_PROT;                                           /* Save Protection mechanism state */
    /* Disable all protection mechanism as VDDA reached UV level */
    IO_PORT_DRV1_PROT = (B_PORT_DRV1_PROT_DIS_OV_LS_VDS
                         | B_PORT_DRV1_PROT_DIS_OV_HS_VDS
                         | B_PORT_DRV1_PROT_DIS_OC
                         | B_PORT_DRV1_PROT_DIS_UV_VDDAF
                         | B_PORT_DRV1_PROT_DIS_UV_VDDA
                         | B_PORT_DRV1_PROT_DIS_UV_VS
                         | B_PORT_DRV1_PROT_DIS_OV_VS
                         | B_PORT_DRV1_PROT_DIS_OVT);
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    IO_MLX16_ITC_PEND0_S = (B_MLX16_ITC_PEND0_UV_VDDA                           /* Clear VDDA pending */
                            | B_MLX16_ITC_PEND0_UV_VS                           /* Clear VS pending UV */
                            | B_MLX16_ITC_PEND0_UV_VDDAF                        /* Clear VDDAF pending UV */
                            | B_MLX16_ITC_PEND0_OVT                             /* Clear pending Over-temperature */
                            | B_MLX16_ITC_PEND0_OVC                             /* Clear pending Over-current */
#if (_SUPPORT_DIAG_VDS != FALSE)
                            | B_MLX16_ITC_PEND0_OV_HS_VDS0
                            | B_MLX16_ITC_PEND0_OV_HS_VDS1
                            | B_MLX16_ITC_PEND0_OV_HS_VDS2
                            | B_MLX16_ITC_PEND0_OV_HS_VDS3
#endif /* (_SUPPORT_DIAG_VDS != FALSE) */
                            );
#if (_SUPPORT_DIAG_VDS != FALSE)
    IO_MLX16_ITC_PEND1_S = (B_MLX16_ITC_PEND1_OV_LS_VDS0
                            | B_MLX16_ITC_PEND1_OV_LS_VDS1
                            | B_MLX16_ITC_PEND1_OV_LS_VDS2
                            | B_MLX16_ITC_PEND1_OV_LS_VDS3);
#endif /* (_SUPPORT_DIAG_VDS != FALSE) */
    IO_MLX16_ITC_PEND2_S = (B_MLX16_ITC_PEND2_OV_VS                             /* Clear pending Over-voltage */
                            | B_MLX16_ITC_PEND2_DIAG);                          /* Clear diagnostics pending */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#endif /* (_SUPPORT_DIAG_VDDA_UV_SLEEP != FALSE) */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    l_u16Drv1Prot = IO_PORT_DRV1_PROT;                                          /* Save Protection mechanism state */
    l_u16Drv3Prot = IO_PORT_DRV3_PROT;                                          /* Save Protection mechanism state */
    /* Disable all protection mechanism as VDDA reached UV level */
    IO_PORT_DRV1_PROT = (B_PORT_DRV1_PROT_DIS_OV_LS_VDS
                         | B_PORT_DRV1_PROT_DIS_OV_HS_VDS
                         | B_PORT_DRV1_PROT_DIS_OC
                         | B_PORT_DRV1_PROT_DIS_UV_VDDA
                         | B_PORT_DRV1_PROT_DIS_UV_VSM
                         | B_PORT_DRV1_PROT_DIS_OV_VSM
                         | B_PORT_DRV1_PROT_DIS_OVT);
    IO_PORT_DRV3_PROT = (B_PORT_DRV3_PROT_DIS_OC_VDDA
                         | B_PORT_DRV3_PROT_DIS_OV_VDDA
                         | B_PORT_DRV3_PROT_DIS_UV_BOOST
                         | B_PORT_DRV3_PROT_DIS_OV_BOOST);
#endif
#if (_SUPPORT_LOG_ERRORS != FALSE)
    SetLastError(C_ERR_VDDA | C_ERR_EXT | 0x0D00U);                             /* IC Diagnostics VDDA-UV */
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */

#if (_DEBUG_DIAG_UV_VDDA != FALSE)
    DEBUG_CLR_IO_A();
#endif /* (_DEBUG_DIAG_UV_VDDA != FALSE) */
} /* End of HandleDiagnosticsVDDA() */

#if defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
#if (_SUPPORT_DRVSUP_ALWAYS_ENA != FALSE)
/*!*************************************************************************** *
 * HandleDiagnosticsVDDAF
 * \brief   Handle Diagnostic VDDAF Under voltage (3.7V)
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  (uint16_t) FALSE: First time VDDAF_UV
 *                     TRUE: Second time VDDAF_UV
 * *************************************************************************** *
 * \details Motor Driver N-FET supply under-voltage diagnostics interrupt.
 *          Check for second occurrence within 3 PWM periods before shut-down
 *          actuator operation.
 *          The MotorDriverStop()-function will shut-down the motor driver; The
 *          DVRSUP remains active (_SUPPORT_DRVSUP_ALWAYS_ENA = TRUE), therefore
 *          the VDDAF_UV mask has to be disabled.
 *          Note: Voltage can't be measured by ADC
 * *************************************************************************** *
 * - Call Hierarchy: DiagnosticInit(), ISR_DIAG() or ISR_UV_VDDAF()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 2 (MotorDriverStop(), SetLastError())
 * *************************************************************************** */
uint16_t HandleDiagnosticsVDDAF(void)
{
    uint16_t u16Result = FALSE;                                                 /* Assume first time (MMP210618-1) */
#else  /* (_SUPPORT_DRVSUP_ALWAYS_ENA != FALSE) */
/*!*************************************************************************** *
 * HandleDiagnosticsVDDAF
 * \brief   Handle Diagnostic VDDAF Under voltage (3.7V)
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Motor Driver N-FET supply under-voltage diagnostics interrupt.
 *          Check for second occurrence within 3 PWM periods before shut-down
 *          actuator operation.
 *          The MotorDriverStop()-function will shut-down the motor driver; The
 *          DVRSUP remains active (_SUPPORT_DRVSUP_ALWAYS_ENA = TRUE), therefore
 *          the VDDAF_UV mask has to be disabled.
 *          Note: Voltage can't be measured by ADC
 * *************************************************************************** *
 * - Call Hierarchy: DiagnosticInit(), ISR_DIAG() or ISR_UV_VDDAF()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 2 (MotorDriverStop(), SetLastError())
 * *************************************************************************** */
void HandleDiagnosticsVDDAF(void)
{
#endif /*(_SUPPORT_DRVSUP_ALWAYS_ENA != FALSE) */

    if (l_u8VDDAF_Count != 0U)
    {
        g_e8ErrorElectric = (uint8_t)(C_ERR_SEMI_PERMANENT | C_ERR_SUP_VDDAF);
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
        MotorDriverStop( (uint16_t)C_STOP_WO_HOLDING);                          /* VDDAF Under Voltage (w/o holding) */
#elif (_SUPPORT_APP_TYPE == C_APP_RELAY)
        RelayDriverOff();                                                       /* VDDAF Under Voltage */
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
        SolenoidDriverDeactivate();                                             /* VDDAF Under Voltage */
#endif /* (_SUPPORT_APP_TYPE) */
#if (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_ROTOR)
        g_u16TargetPosition = g_u16ActualPosition;
#elif (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_SHAFT)
        g_u16TargetShaftAngle = g_u16ActualShaftAngle;
#endif /* (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_ROTOR) */
#if (_SUPPORT_LOG_ERRORS != FALSE)
        SetLastError(C_ERR_VDDAF | C_ERR_EXT | 0x0A00U);                        /* VDDAF UV */
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
#if (_SUPPORT_DRVSUP_ALWAYS_ENA != FALSE)
        u16Result = TRUE;                                                       /* Second time */
#endif /* (_SUPPORT_DRVSUP_ALWAYS_ENA != FALSE) */
    }
    else
    {
        l_u8VDDAF_Count = 2U;                                                   /* Delay: 1 - 2 Ticks */
#if (_SUPPORT_LOG_ERRORS != FALSE)
        SetLastError(C_WRN_VREF);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
        DELAY(C_DELAY_mPWM);  /*lint !e522 */                                   /* Wait for ESD pulse to be gone and a new ADC measurement have been take place */
    }
#if (_SUPPORT_DRV_PROT_UV_VDDAF != FALSE)
    IO_PORT_DRV1_PROT |= B_PORT_DRV1_PROT_DIS_UV_VDDAF;                         /* Clear VDDAF-UV flag */
    IO_PORT_DRV1_PROT &= ~B_PORT_DRV1_PROT_DIS_UV_VDDAF;
#endif /* (_SUPPORT_DRV_PROT_UV_VDDAF != FALSE) */
#if (_SUPPORT_DRVSUP_ALWAYS_ENA != FALSE)
    return (u16Result);
#endif /* (_SUPPORT_DRVSUP_ALWAYS_ENA != FALSE) */
} /* End of HandleDiagnosticsVDDAF() */
#elif (defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)) && (_SUPPORT_DIAG_BOOST != FALSE)
/*!*************************************************************************** *
 * HandleDiagnosticsVBOOST
 * \brief   Handle Diagnostic VBOOST Under and Over voltage
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16VBoostError: Kind of VBoost error; 1 = UV, 2 = OV
 * \return  (uint16_t) FALSE: First time VDDAF_UV
 *                     TRUE: Second time VDDAF_UV
 * *************************************************************************** *
 * \details Motor Driver N-FET supply under- or over-voltage diagnostics interrupt.
 *          Check for second occurrence within 3 PWM periods before shut-down
 *          actuator operation.
 *          Note: Voltage can't be measured by ADC
 * *************************************************************************** *
 * - Call Hierarchy: DiagnosticInit(), ISR_DIAG() or ISR_UV_VDDAF()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 2 (MotorDriverStop(), SetLastError())
 * *************************************************************************** */
uint16_t HandleDiagnosticsVBOOST(uint16_t u16VBoostError)
{
    uint16_t u16Result = FALSE;                                                 /* Assume first time (MMP210818-1) */
#if (_DEBUG_DIAG_VBOOST != FALSE)
    DEBUG_SET_IO_E();
#endif /* (_DEBUG_DIAG_VBOOST != FALSE) */
    /*if ( l_u8VBOOST_Count != 0U ) */
    {
        g_e8ErrorElectric = (uint8_t)(C_ERR_SEMI_PERMANENT | C_ERR_SUP_VBOOST);
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
#if (_SUPPORT_FAST_STOP == FALSE)
        MotorDriverStop( (uint16_t)C_STOP_WO_HOLDING);                          /* VBoost error (w/o holding) */
#else  /* (_SUPPORT_FAST_STOP == FALSE) */
        MotorDriverStop( (uint16_t)C_STOP_FAST_STOP);                           /* VBoost error (Fast Stop) (MMP231121-1) */
#endif /* (_SUPPORT_FAST_STOP == FALSE) */
#elif (_SUPPORT_APP_TYPE == C_APP_RELAY)
        RelayDriverOff();                                                       /* VBoost error */
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
        SolenoidDriverDeactivate();                                             /* VBoost error */
#endif /* (_SUPPORT_APP_TYPE) */
#if (_SUPPORT_LOG_ERRORS != FALSE)
#if (_SUPPORT_ADC_VBOOST != FALSE)
        SetLastError(C_ERR_VBOOST | C_ERR_EXTW | (u16VBoostError << 8));
        SetLastError(Get_AdcVboost());
#else  /* (_SUPPORT_ADC_VBOOST != FALSE) */
        SetLastError(C_ERR_VBOOST | C_ERR_EXT | (u16VBoostError << 8));
#endif /* (_SUPPORT_ADC_VBOOST != FALSE) */
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
        u16Result = TRUE;                                                       /* Second time */
    }
#if FALSE
    else
    {
        uint16_t u16DrvCtrl = IO_PORT_DRV_CTRL;                                 /* Save driver control state */
        IO_PORT_DRV_CTRL =
            (C_PORT_DRV_CTRL_DRV2_TRISTATE | C_PORT_DRV_CTRL_DRV1_TRISTATE | C_PORT_DRV_CTRL_DRV0_TRISTATE);                 /* Switch driver to tristate to allow VBOOST to recover */
        l_u8VBOOST_Count++;
#if (_SUPPORT_LOG_ERRORS != FALSE)
        SetLastError(C_WRN_VBOOST | C_ERR_EXT | (u16VBoostError << 8));
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
        DELAY(C_DELAY_mPWM);  /*lint !e522 */                                   /* Wait for ESD pulse to be gone and a new ADC measurement have been take place */
        IO_PORT_DRV_CTRL = u16DrvCtrl;                                          /* Restore driver control state */
    }
#endif
#if (_SUPPORT_DRV_PROT_UV_VBOOST != FALSE) || (_SUPPORT_DRV_PROT_OV_VBOOST != FALSE)
    IO_PORT_DRV3_PROT |= (B_PORT_DRV3_PROT_DIS_UV_BOOST | B_PORT_DRV3_PROT_DIS_OV_BOOST);  /* Clear VBOOST UV or OV flag */
    IO_PORT_DRV3_PROT &= ~(B_PORT_DRV3_PROT_DIS_UV_BOOST | B_PORT_DRV3_PROT_DIS_OV_BOOST);  /* Enable VBOOST */
#endif /* (_SUPPORT_DRV_PROT_UV_VBOOST != FALSE) || (_SUPPORT_DRV_PROT_OV_VBOOST != FALSE) */
#if (_SUPPORT_LOG_ERRORS == FALSE)
    (void)u16VBoostError;
#endif /* (_SUPPORT_LOG_ERRORS == FALSE) */
#if (_DEBUG_DIAG_VBOOST != FALSE)
    DEBUG_CLR_IO_E();
#endif /* (_DEBUG_DIAG_VBOOST != FALSE) */
    return (u16Result);
} /* End of HandleDiagnosticsVBOOST() */
#endif

/*!*************************************************************************** *
 * DiagnosticInit
 * \brief   Diagnostic initialisation
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Enable motor-driver automatically shut-off in case of over-current.
 *          Don't automatically shut-off motor-driver on over- or
 *          under-voltage, or over-temperature. Diagnostic ISR priority: 3.
 * *************************************************************************** *
 * - Call Hierarchy: main_Init()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 1 (HandleDiagnosticEvent())
 * *************************************************************************** */
void DiagnosticInit(void)
{
    IO_PORT_MISC2_OUT |= (B_PORT_MISC2_OUT_ENABLE_OTD                           /* Enable Over temperature detector (MMP180623-1) */
#if defined (__MLX81340B01__) || defined (__MLX81344B01__)
#if (_SUPPORT_EVB == MLX8134xEVB_100W)
#if (_APP_SUPPLY_RANGE == C_APP_SUPPLY_RANGE_24V)
                          | C_PORT_MISC2_OUT_VDSMON_VTH_SEL_11                  /* VDS Monitor threshold selection: 2.0V */
#else  /* (_APP_SUPPLY_RANGE == C_APP_SUPPLY_RANGE_24V) */
                          | C_PORT_MISC2_OUT_VDSMON_VTH_SEL_10                  /* VDS Monitor threshold selection: 1.5V */
#endif /* (_APP_SUPPLY_RANGE == C_APP_SUPPLY_RANGE_24V) */
#else  /* (_SUPPORT_EVB == MLX8134xEVB_100W) */
                          | C_PORT_MISC2_OUT_VDSMON_VTH_SEL_10                  /* VDS Monitor threshold selection: 1.5V */
#endif /* (_SUPPORT_EVB == MLX8134xEVB_100W) */
#elif defined (__MLX81346B01__)
#if (_APP_SUPPLY_RANGE == C_APP_SUPPLY_RANGE_12V)
                          | C_PORT_MISC2_OUT_VDSMON_VTH_SEL_1                   /* VDS Monitor threshold selection: 1.0V */
#elif (_APP_SUPPLY_RANGE == C_APP_SUPPLY_RANGE_24V)
                          | C_PORT_MISC2_OUT_VDSMON_VTH_SEL_2                   /* VDS Monitor threshold selection: 1.5V */
#elif (_APP_SUPPLY_RANGE == C_APP_SUPPLY_RANGE_48V)
                          | C_PORT_MISC2_OUT_VDSMON_VTH_SEL_3                   /* VDS Monitor threshold selection: 2.0V */
#endif
#endif /*defined (__MLX81346B01__) */
                          );

    /* Driver protection */                                                     /* Select 100-200us debounce for: */
    IO_PORT_SUPP_CFG = (B_PORT_SUPP_CFG_OVT_FILT_SEL |                          /* Over Temperature */
#if defined (__MLX81160__)
                        /*B_PORT_SUPP_CFG_OV_VSM_FILT_SEL |*/                   /* Over Voltage VSM (MMP220613-1) */
                        B_PORT_SUPP_CFG_UV_VSM_FILT_SEL |                       /* Under Voltage VSM */
                        B_PORT_SUPP_CFG_OC_VDDA_FILT_SEL |                      /* Over current VDDA */
                        B_PORT_SUPP_CFG_OV_VDDA_FILT_SEL |                      /* Over voltage VDDA */
                        C_PORT_SUPP_CFG_UV_VDDAF_FILT_SEL_11 |                  /* Under Voltage VDDAF */
#elif defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
                        /*B_PORT_SUPP_CFG_OV_VS_FILT_SEL |*/                    /* Over Voltage VS (MMP220613-1) */
                        B_PORT_SUPP_CFG_UV_VS_FILT_SEL |                        /* Under Voltage VS */
#if defined (__MLX81330A01__) || defined (__MLX81332A01__)
                        B_PORT_SUPP_CFG_UV_VDDAF_FILT_SEL |                     /* Under Voltage VDDAF */
#else
                        C_PORT_SUPP_CFG_UV_VDDAF_FILT_SEL_11 |                  /* Under Voltage VDDAF */
#endif
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
                        /*B_PORT_SUPP_CFG_OV_VSM_FILT_SEL |*/                   /* Over Voltage VSM (MMP220613-1) */
                        B_PORT_SUPP_CFG_UV_VSM_FILT_SEL |                       /* Under Voltage VSM */
                        B_PORT_SUPP_CFG_OV_BOOST_FILT_SEL |                     /* Over Voltage VBOOST */
                        B_PORT_SUPP_CFG_UV_BOOST_FILT_SEL |                     /* Under Voltage VBOOST */
                        B_PORT_SUPP_CFG_OC_VDDA_FILT_SEL |                      /* Over Current VDDA */
                        B_PORT_SUPP_CFG_OV_VDDA_FILT_SEL |                      /* Over Voltage VDDA */
#endif /* defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */
                        B_PORT_SUPP_CFG_UV_VDDA_FILT_SEL);                      /* Under Voltage VDDA */

    IO_PORT_DRV1_PROT = ((IO_PORT_DRV1_PROT &
                          ~(0                                                   /* Enable .... */
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
#if (_SUPPORT_DRV_PROT_VDS_PM_TRI == FALSE)
                            | B_PORT_DRV1_PROT_OV_LS_VDS_PM                     /* Switch to HS-state LS_VDS */
                            | B_PORT_DRV1_PROT_OV_HS_VDS_PM                     /* Switch to LS-state HS_VDS */
#endif /* (_SUPPORT_DRV_PROT_VDS_PM_TRI == FALSE) */
#if (_SUPPORT_DRV_PROT_TRISTATE == FALSE)
                            | B_PORT_DRV1_PROT_OC_PM                            /* Switch to LS-state OC */
                            | B_PORT_DRV1_PROT_UV_VDDAF_PM                      /* Switch to LS-state UV_VDDAF */
                            | B_PORT_DRV1_PROT_UV_VDDA_PM                       /* Switch to LS-state UV_VDDA */
                            | B_PORT_DRV1_PROT_UV_VS_PM                         /* Switch to LS-state UV_VS */
                            | B_PORT_DRV1_PROT_OV_VS_PM                         /* Switch to LS-state OV_VS */
                            | B_PORT_DRV1_PROT_OVT_PM                           /* Switch to LS-state OVT */
#endif /* (_SUPPORT_DRV_PROT_TRISTATE) */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#if (_SUPPORT_DRV_PROT_VDS_PM_TRI != FALSE)
                            | B_PORT_DRV1_PROT_OV_LS_VDS_PM                     /* Switch to tri-state LS_VDS */
                            | B_PORT_DRV1_PROT_OV_HS_VDS_PM                     /* Switch to tri-state HS_VDS */
#endif /* (_SUPPORT_DRV_PROT_VDS_PM_TRI != FALSE) */
#if (_SUPPORT_DRV_PROT_TRISTATE != FALSE)
                            | B_PORT_DRV1_PROT_OC_PM                            /* Switch to tri-state OC */
                            | B_PORT_DRV1_PROT_UV_VDDA_PM                       /* Switch to tri-state UV_VDDA */
                            | B_PORT_DRV1_PROT_UV_VSM_PM                        /* Switch to tri-state UV_VSM */
                            | B_PORT_DRV1_PROT_OV_VSM_PM                        /* Switch to tri-state OV_VSM */
                            | B_PORT_DRV1_PROT_OVT_PM                           /* Switch to tri-state OVT */
#endif /* (_SUPPORT_DRV_PROT_TRISTATE != FALSE) */
#elif defined (__MLX81160__)
#if (_SUPPORT_DRV_PROT_TRISTATE != FALSE)
                            | B_PORT_DRV1_PROT_OC_VDDA_PM                       /* Switch to tri-state OC_VDDA */
                            | B_PORT_DRV1_PROT_OV_VDDA_PM                       /* Switch to tri-state OV_VDDA */
                            | B_PORT_DRV1_PROT_OC_PM                            /* Switch to tri-state OC */
                            | B_PORT_DRV1_PROT_UV_VDDA_PM                       /* Switch to tri-state UV_VDDA */
                            | B_PORT_DRV1_PROT_UV_VSM_PM                        /* Switch to tri-state UV_VSM */
                            | B_PORT_DRV1_PROT_OV_VSM_PM                        /* Switch to tri-state OV_VSM */
                            | B_PORT_DRV1_PROT_OVT_PM                           /* Switch to tri-state OVT */
#endif /* (_SUPPORT_DRV_PROT_TRISTATE != FALSE) */
#endif /* defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__) */
#if !defined (__MLX81160__) && (_SUPPORT_DRV_PROT_VDS != FALSE)
                            | B_PORT_DRV1_PROT_DIS_OV_LS_VDS                    /* Enable Driver protection against OV Low-side VDS */
                            | B_PORT_DRV1_PROT_DIS_OV_HS_VDS                    /* Enable Driver protection against OV High-side VDS */
#endif /* !defined (__MLX81160__) && (_SUPPORT_DRV_PROT_VDS != FALSE) */
#if (_SUPPORT_DRV_PROT_OC != FALSE)
                            | B_PORT_DRV1_PROT_DIS_OC                           /* Enable Driver protection against Over-Current */
#endif /* (_SUPPORT_DRV_PROT_OC != FALSE) */
#if (defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)) && \
    (_SUPPORT_DRV_PROT_UV_VDDAF != FALSE)
                            | B_PORT_DRV1_PROT_DIS_UV_VDDAF                     /* Enable Driver protection against UV VDDAF */
#endif /* (defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)) && (_SUPPORT_DRV_PROT_UV_VDDAF != FALSE) */
#if (_SUPPORT_DRV_PROT_UV_VDDA != FALSE)
                            | B_PORT_DRV1_PROT_DIS_UV_VDDA                      /* Enable Driver protection against UV VDDA */
#endif /* (_SUPPORT_DRV_PROT_UV_VDDA != FALSE) */
#if (_SUPPORT_DRV_PROT_UV_VS != FALSE)
                            | B_PORT_DRV1_PROT_DIS_UV_VS                        /* Enable Driver protection against UV VS */
#endif /* (_SUPPORT_DRV_PROT_UV_VS != FALSE) */
#if (_SUPPORT_DRV_PROT_OV_VS != FALSE)
                            | B_PORT_DRV1_PROT_DIS_OV_VS                        /* Enable Driver protection against OV VS */
#endif /* (_SUPPORT_DRV_PROT_OV_VS != FALSE) */
#if (_SUPPORT_DRV_PROT_OT != FALSE)
                            | B_PORT_DRV1_PROT_DIS_OVT                          /* Enable Driver protection against OT */
#endif /* (_SUPPORT_DRV_PROT_OT != FALSE) */
                            )
                          ) |                                                   /* Disable .... */
                         (0
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
#if (_SUPPORT_DRV_PROT_VDS_PM_TRI != FALSE)
                          | B_PORT_DRV1_PROT_OV_LS_VDS_PM                       /* Switch to tri-state LS_VDS */
                          | B_PORT_DRV1_PROT_OV_HS_VDS_PM                       /* Switch to tri-state HS_VDS */
#endif /* (_SUPPORT_DRV_PROT_VDS_PM_TRI != FALSE) */
#if (_SUPPORT_DRV_PROT_TRISTATE != FALSE)
                          | B_PORT_DRV1_PROT_OC_PM                              /* Switch to tri-state OC */
                          | B_PORT_DRV1_PROT_UV_VDDAF_PM                        /* Switch to tri-state UV_VDDAF */
                          | B_PORT_DRV1_PROT_UV_VDDA_PM                         /* Switch to tri-state UV_VDDA */
                          | B_PORT_DRV1_PROT_UV_VS_PM                           /* Switch to tri-state UV_VS */
                          | B_PORT_DRV1_PROT_OV_VS_PM                           /* Switch to tri-state OV_VS */
                          | B_PORT_DRV1_PROT_OVT_PM                             /* Switch to tri-state OVT */
#endif /* (_SUPPORT_DRV_PROT_TRISTATE) */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#if (_SUPPORT_DRV_PROT_VDS_PM_TRI == FALSE)
                          | B_PORT_DRV1_PROT_OV_LS_VDS_PM                       /* Switch to HS-state LS_VDS */
                          | B_PORT_DRV1_PROT_OV_HS_VDS_PM                       /* Switch to LS-state HS_VDS */
#endif /* (_SUPPORT_DRV_PROT_VDS_PM_TRI == FALSE) */
#if (_SUPPORT_DRV_PROT_TRISTATE == FALSE)
                          | B_PORT_DRV1_PROT_OC_PM                              /* Switch to LS-state OC */
                          | B_PORT_DRV1_PROT_UV_VDDA_PM                         /* Switch to LS-state UV_VDDA */
                          | B_PORT_DRV1_PROT_UV_VSM_PM                          /* Switch to LS-state UV_VS */
                          | B_PORT_DRV1_PROT_OV_VSM_PM                          /* Switch to LS-state OV_VS */
                          | B_PORT_DRV1_PROT_OVT_PM                             /* Switch to LS-state OVT */
#endif /* (_SUPPORT_DRV_PROT_TRISTATE == FALSE) */
#elif defined (__MLX81160__)
#if (_SUPPORT_DRV_PROT_TRISTATE == FALSE)
                          | B_PORT_DRV1_PROT_OC_VDDA_PM                         /* Switch to LS-state OC_VDDA */
                          | B_PORT_DRV1_PROT_OV_VDDA_PM                         /* Switch to LS-state OV_VDDA */
                          | B_PORT_DRV1_PROT_OC_PM                              /* Switch to LS-state OC */
                          | B_PORT_DRV1_PROT_UV_VDDA_PM                         /* Switch to LS-state UV_VDDA */
                          | B_PORT_DRV1_PROT_UV_VSM_PM                          /* Switch to LS-state UV_VSM */
                          | B_PORT_DRV1_PROT_OV_VSM_PM                          /* Switch to LS-state OV_VSM */
                          | B_PORT_DRV1_PROT_OVT_PM                             /* Switch to LS-state OVT */
#endif /* (_SUPPORT_DRV_PROT_TRISTATE == FALSE) */
#endif /* defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__) */
#if !defined (__MLX81160__) && (_SUPPORT_DRV_PROT_VDS == FALSE)
                          | B_PORT_DRV1_PROT_DIS_OV_LS_VDS                      /* Disable Driver protection against OV Low-side VDS */
                          | B_PORT_DRV1_PROT_DIS_OV_HS_VDS                      /* Disable Driver protection against OV High-side VDS */
#endif /* !defined (__MLX81160__) && (_SUPPORT_DRV_PROT_VDS != FALSE) */
#if (_SUPPORT_DRV_PROT_OC == FALSE)
                          | B_PORT_DRV1_PROT_DIS_OC                             /* Disable Driver protection against Over-Current */
#endif /* (_SUPPORT_DRV_PROT_OC != FALSE) */
#if (defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)) && \
    (_SUPPORT_DRV_PROT_UV_VDDAF == FALSE)
                          | B_PORT_DRV1_PROT_DIS_UV_VDDAF                       /* Disable UV VDDAF */
#endif /* (defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)) && (_SUPPORT_DRV_PROT_UV_VDDAF == FALSE) */
#if (_SUPPORT_DRV_PROT_UV_VDDA == FALSE)
                          | B_PORT_DRV1_PROT_DIS_UV_VDDA                        /* Disable UV VDDA */
#endif /* (_SUPPORT_DRV_PROT_UV_VDDA == FALSE) */
#if (_SUPPORT_DRV_PROT_UV_VS == FALSE)
#if defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
                          | B_PORT_DRV1_PROT_DIS_UV_VSM                         /* Disable Under-voltage */
#else
                          | B_PORT_DRV1_PROT_DIS_UV_VS                          /* Disable Under-voltage */
#endif
#endif /* (_SUPPORT_DRV_PROT_UV_VS == FALSE) */
#if (_SUPPORT_DRV_PROT_OV_VS == FALSE)
#if defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
                          | B_PORT_DRV1_PROT_DIS_OV_VSM                         /* Disable Over-Voltage */
#else
                          | B_PORT_DRV1_PROT_DIS_OV_VS                          /* Disable Over-Voltage */
#endif
#endif /* (_SUPPORT_DRV_PROT_OV_VS == FALSE) */
#if (_SUPPORT_DRV_PROT_OT == FALSE)
                          | B_PORT_DRV1_PROT_DIS_OVT                            /* Disable Over-Temperature */
#endif /* (_SUPPORT_DRV_PROT_OT == FALSE) */
                         ));

#if defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    IO_PORT_DRV3_PROT = ((IO_PORT_DRV3_PROT &
                          ~(0                                                   /* Enable .... */
#if (_SUPPORT_DRV_PROT_TRISTATE != FALSE)
                            | B_PORT_DRV3_PROT_OC_VDDA_PM                       /* Switch to tri-state OC_VDDA */
                            | B_PORT_DRV3_PROT_OV_VDDA_PM                       /* Switch to tri-state OV_VDDA */
                            | B_PORT_DRV3_PROT_UV_BOOST_PM                      /* Switch to tri-state UV_BOOST */
                            | B_PORT_DRV3_PROT_OV_BOOST_PM                      /* Switch to tri-state OV_BOOST */
#endif /* (_SUPPORT_DRV_PROT_TRISTATE == FALSE) */
#if (_SUPPORT_DRV_PROT_OC_VDDA != FALSE)
                            | B_PORT_DRV3_PROT_DIS_OC_VDDA                      /* Enable Driver protection against Over-Current VDDA */
#endif /* (_SUPPORT_DRV_PROT_OC_VDDA != FALSE) */
#if (_SUPPORT_DRV_PROT_OV_VDDA != FALSE)
                            | B_PORT_DRV3_PROT_DIS_OV_VDDA                      /* Enable Driver protection against Over-Voltage VDDA */
#endif /* (_SUPPORT_DRV_PROT_OV_VDDA != FALSE) */
#if (_SUPPORT_DRV_PROT_UV_VBOOST != FALSE)
                            | B_PORT_DRV3_PROT_DIS_UV_VBOOST                    /* Enable Driver protection against Under-Voltage VBOOST */
#endif /* (_SUPPORT_DRV_PROT_UV_VBOOST != FALSE) */
#if (_SUPPORT_DRV_PROT_OV_VBOOST != FALSE)
                            | B_PORT_DRV3_PROT_DIS_OV_VBOOST                    /* Enable Driver protection against Over-Voltage VBOOST */
#endif /* (_SUPPORT_DRV_PROT_OV_VBOOST != FALSE) */
                            )
                          ) |                                                   /* Disable .... */
                         (0
#if (_SUPPORT_DRV_PROT_TRISTATE == FALSE)
                          | B_PORT_DRV3_PROT_OC_VDDA_PM                         /* Switch to LS-state OC_VDDA */
                          | B_PORT_DRV3_PROT_OV_VDDA_PM                         /* Switch to LS-state OV_VDDA */
                          | B_PORT_DRV3_PROT_UV_BOOST_PM                        /* Switch to LS-state UV_BOOST */
                          | B_PORT_DRV3_PROT_OV_BOOST_PM                        /* Switch to LS-state OV_BOOST */
#endif /* (_SUPPORT_DRV_PROT_TRISTATE == FALSE) */
#if (_SUPPORT_DRV_PROT_OC_VDDA == FALSE)
                          | B_PORT_DRV3_PROT_DIS_OC_VDDA                        /* Disable Driver protection against Over-Current VDDA */
#endif /* (_SUPPORT_DRV_PROT_OC_VDDA == FALSE) */
#if (_SUPPORT_DRV_PROT_OV_VDDA == FALSE)
                          | B_PORT_DRV3_PROT_DIS_OV_VDDA                        /* Disable Driver protection against Over-Voltage VDDA */
#endif /* (_SUPPORT_DRV_PROT_OV_VDDA != FALSE) */
#if (_SUPPORT_DRV_PROT_UV_VBOOST == FALSE)
                          | B_PORT_DRV3_PROT_DIS_UV_BOOST                       /* Disable Driver protection against Under-Voltage VBOOST */
#endif /* (_SUPPORT_DRV_PROT_UV_VBOOST != FALSE) */
#if (_SUPPORT_DRV_PROT_OV_VBOOST == FALSE)
                          | B_PORT_DRV3_PROT_DIS_OV_BOOST                       /* Disable Driver protection against Over-Voltage VBOOST */
#endif /* (_SUPPORT_DRV_PROT_OV_VBOOST != FALSE) */
                         ));
#endif /* defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */

    IO_PORT_MISC_OUT = (IO_PORT_MISC_OUT & ~(
#if !defined (__MLX81339__)
                                             M_PORT_MISC_OUT_SEL_TEMP |
#endif /* !defined (__MLX81339__) */
                                             M_PORT_MISC_OUT_PROV_VS |
                                             M_PORT_MISC_OUT_PRUV_VS)) |
                       (
#if defined (__MLX81160__)
                        C_PORT_MISC_OUT_SEL_TEMP_VBGA |
#elif defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81350__)
                        C_PORT_MISC_OUT_SEL_TEMP_MAIN |                         /* Main temperature sensor */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
                        C_PORT_MISC_OUT_SEL_TEMP_VBGA |                         /* Main temperature sensor (VBGA regulator) */
#endif
#if !defined (__MLX81160__) && !defined (__MLX81339__)
                        (0 * B_PORT_MISC_OUT_PRUV_VDDA) |                       /* VDDA UV-level: 0: 2.85V or 1: 3.0V */
#endif /* !defined (__MLX81160__) */
                        (NV_IC_UV_LEVEL * C_PORT_MISC_OUT_PRUV_0) |             /* UV-Level: 4V, 5V, 6V, 7V, 8V, 9V (0-5) */
                        (NV_IC_OV_LEVEL * C_PORT_MISC_OUT_PROV_0));             /* OV-Level: 22V, 24V, 40V (0-2) */

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
#if (_SUPPORT_EVB == MLX8134xEVB_100W)
#if (_APP_SUPPLY_RANGE == C_APP_SUPPLY_RANGE_48V)
    IO_PORT_DIV_VDS_DEB = 1U;                                                   /* VDS Debounce Pre-divider FW_FREQ / 2^n: 32MHz --> 16MHz */
    IO_PORT_CNT_VDS_DEB = (8U << 8) | (8U << 4) | (8U << 0);                    /* VDS Debounce count = c * (FW_FREQ / 2^n) = 0.5 us */
    IO_PORT_DIV_MASK_VDS = 2U;                                                  /* MASK Pre-divider FW_FREQ / 2^n: 32MHz --> 8MHz */
    IO_PORT_CNT_MASK_VDS_LS = (9U << 8) | (9U << 4) | (9U << 0);                /* MASK VDS-LS = c * (FW_FREQ / 2^n) = 1.125 us */
    IO_PORT_CNT_MASK_VDS_HS = (9U << 8) | (9U << 4) | (9U << 0);                /* MASK VDS-HS = c * (FW_FREQ / 2^n) = 1.125 us */
#elif (_APP_SUPPLY_RANGE == C_APP_SUPPLY_RANGE_24V)
    IO_PORT_DIV_VDS_DEB = 1U;                                                   /* VDS Debounce Pre-divider FW_FREQ / 2^n: 32MHz --> 32MHz */
    IO_PORT_CNT_VDS_DEB = (8U << 8) | (8U << 4) | (8U << 0);                    /* VDS Debounce count = c * (FW_FREQ / 2^n) = 0.25 us */
    IO_PORT_DIV_MASK_VDS = 2U;                                                  /* MASK Pre-divider FW_FREQ / 2^n: 32MHz --> 16MHz */
    IO_PORT_CNT_MASK_VDS_LS = (9U << 8) | (9U << 4) | (9U << 0);                /* MASK VDS-LS = c * (FW_FREQ / 2^n) = 0.5625 us */
    IO_PORT_CNT_MASK_VDS_HS = (9U << 8) | (9U << 4) | (9U << 0);                /* MASK VDS-HS = c * (FW_FREQ / 2^n) = 0.5625 us */
#else  /* (_APP_SUPPLY_RANGE == C_APP_SUPPLY_RANGE_48V) */
    IO_PORT_DIV_VDS_DEB = 0U;                                                   /* VDS Debounce Pre-divider FW_FREQ / 2^n: 32MHz --> 32MHz */
    IO_PORT_CNT_VDS_DEB = (8U << 8) | (8U << 4) | (8U << 0);                    /* VDS Debounce count = c * (FW_FREQ / 2^n) = 0.25 us */
    IO_PORT_DIV_MASK_VDS = 1U;                                                  /* MASK Pre-divider FW_FREQ / 2^n: 32MHz --> 16MHz */
    IO_PORT_CNT_MASK_VDS_LS = (9U << 8) | (9U << 4) | (9U << 0);                /* MASK VDS-LS = c * (FW_FREQ / 2^n) = 0.5625 us */
    IO_PORT_CNT_MASK_VDS_HS = (9U << 8) | (9U << 4) | (9U << 0);                /* MASK VDS-HS = c * (FW_FREQ / 2^n) = 0.5625 us */
#endif /* (_APP_SUPPLY_RANGE == C_APP_SUPPLY_RANGE_48V) */
    IO_PORT_DIV_EILD = 0U;                                                      /* External Inter-lock Delay Pre-divider FW_FREQ / 2^n: 32MHz --> 32MHz */
    IO_PORT_CNT_EILD_LSOFF = (6U << 8) | (6U << 4) | (6U << 0);                 /* External Inter-lock Delay count = c * (FW_FREQ / 2^n) = 0.1875us */
    IO_PORT_CNT_EILD_HSOFF = (6U << 8) | (6U << 4) | (6U << 0);                 /* External Inter-lock Delay count = c * (FW_FREQ / 2^n) = 0.1875us */
    IO_PORT_OC_DEB = (3U << 4) | 12U;                                           /* TOC_DEB = 3us; 12 * (FW_FREQ / 2^3) = 3.0us @ 32MHz */
#elif (_SUPPORT_EVB == MLX8134xEVB_400W)
    IO_PORT_DIV_VDS_DEB = 2U;                                                   /* VDS Debounce Pre-divider FW_FREQ / 2^n: 32MHz --> 8MHz */
    IO_PORT_CNT_VDS_DEB = (8U << 8) | (8U << 4) | (8U << 0);                    /* VDS Debounce count = c * (FW_FREQ / 2^n) = 0.125 us*8 */
    IO_PORT_DIV_MASK_VDS = 2U;                                                  /* MASK Pre-divider FW_FREQ / 2^n: 32MHz --> 8MHz */
    IO_PORT_CNT_MASK_VDS_LS = (12U << 8) | (12U << 4) | (12U << 0);             /* MASK VDS-LS = c * (FW_FREQ / 2^n) = 0.125 us*12 */
    IO_PORT_CNT_MASK_VDS_HS = (12U << 8) | (12U << 4) | (12U << 0);             /* MASK VDS-HS = c * (FW_FREQ / 2^n) = 0.125 us*12 */
    IO_PORT_DIV_EILD = 0U;                                                      /* External Inter-lock Delay Pre-divider FW_FREQ / 2^n: 32MHz --> 32MHz */
    IO_PORT_CNT_EILD_LSOFF = (6U << 8) | (6U << 4) | (6U << 0);                 /* External Inter-lock Delay count = c * (FW_FREQ / 2^n) = 0.03125us*6 */
    IO_PORT_CNT_EILD_HSOFF = (6U << 8) | (6U << 4) | (6U << 0);                 /* External Inter-lock Delay count = c * (FW_FREQ / 2^n) = 0.03125us*6 */
    IO_PORT_OC_DEB = (3U << 4) | 11U;                                           /* TOC_DEB = 3us; 11 * (FW_FREQ / 2^3) = 3.0 us @ 32MHz */
#endif
#endif /* defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */

    /* Important note: Any OV/UV or OVT at power-up of the chip,
     * will be cleared below and therefore not given an IRQ. OC should not
     * happen as driver is not enabled. UV can also be caused by a slow ramp-up
     * of the supply-voltage!! */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#if defined (__MLX81160__)
    IO_MLX16_ITC_PEND0_S = (B_MLX16_ITC_PEND0_UV_VDDA                           /* Clear VDDA pending */
                            | B_MLX16_ITC_PEND0_UV_VSM                          /* Clear VSM pending UV (MMP220818-1) */
                            | B_MLX16_ITC_PEND0_UV_VDDAF                        /* Clear VDDAF pending UV */
                            | B_MLX16_ITC_PEND0_OVT                             /* Clear pending Over-temperature */
                            | B_MLX16_ITC_PEND0_OVC0                            /* Clear pending Over-current 0 */
                            | B_MLX16_ITC_PEND0_OVC1                            /* Clear pending Over-current 1 */
                            | B_MLX16_ITC_PEND0_OC_VDDA);                       /* Clear pending Over-current VDDA */
    IO_MLX16_ITC_PEND2_S = (B_MLX16_ITC_PEND2_OV_VSM                            /* Clear pending Over-voltage (MMP220818-1) */
                            | B_MLX16_ITC_PEND2_DIAG);                          /* Clear diagnostics pending */
    IO_MLX16_ITC_PRIO2_S = (IO_MLX16_ITC_PRIO2_S & ~M_MLX16_ITC_PRIO2_OV_VSM) | C_MLX16_ITC_PRIO2_OV_VSM_PRIO3;  /* MMP220818-1 */
#if (_SUPPORT_DIAG_DRV_PROT != FALSE)
    IO_MLX16_ITC_PRIO3_S = (IO_MLX16_ITC_PRIO3_S & ~M_MLX16_ITC_PRIO3_DIAG) | C_MLX16_ITC_PRIO3_DIAG_PRIO3;
#endif /* (_SUPPORT_DIAG_DRV_PROT != FALSE) */
    IO_MLX16_ITC_MASK0_S |= (B_MLX16_ITC_MASK0_AWD_ATT                          /* Enable Absolute Watchdog Attention */
                             | B_MLX16_ITC_MASK0_UV_VDDA                        /* Enable VDDA UV */
                             | B_MLX16_ITC_MASK0_UV_VSM                         /* Enable VSM UV (MMP220818-1) */
/* Don't enable UV VDDAF as long as the driver is not supplied (B_PORT_DRV_OUT_ENABLE_DRVSUP) */
/*                            | B_MLX16_ITC_MASK0_UV_VDDAF */                   /* Enable VDDAF UV */
#if (_SUPPORT_DIAG_OT != FALSE)
                             | B_MLX16_ITC_MASK0_OVT                            /* Enable Over-temperature */
#endif /* (_SUPPORT_DIAG_OT != FALSE) */
#if (_SUPPORT_DIAG_OC != FALSE)
#if (_SUPPORT_ADC_CUR_SENSE_CH == ADC_CUR_SENSE_CH1) || (_SUPPORT_ADC_CUR_SENSE_CH == ADC_CUR_SENSE_CHX)
                             | B_MLX16_ITC_MASK0_OVC0                           /* Enable Over-current */
#endif /* (_SUPPORT_ADC_CUR_SENSE_CH == ADC_CUR_SENSE_CH1) || (_SUPPORT_ADC_CUR_SENSE_CH == ADC_CUR_SENSE_CHX) */
#if (_SUPPORT_ADC_CUR_SENSE_CH == ADC_CUR_SENSE_CH2) || (_SUPPORT_ADC_CUR_SENSE_CH == ADC_CUR_SENSE_CHX)
                             | B_MLX16_ITC_MASK0_OVC1                           /* Enable Over-current */
#endif /* (_SUPPORT_ADC_CUR_SENSE_CH == ADC_CUR_SENSE_CH2) || (_SUPPORT_ADC_CUR_SENSE_CH == ADC_CUR_SENSE_CHX) */
#endif /* (_SUPPORT_DIAG_OC != FALSE) */
                             );
    IO_MLX16_ITC_MASK2_S |= (B_MLX16_ITC_MASK2_OV_VSM                           /* Enable Over-voltage (MMP220818-1) */
#if (_SUPPORT_DIAG_DRV_PROT != FALSE)
                             | B_MLX16_ITC_MASK2_DIAG                           /* Diagnostic */
#endif /* (_SUPPORT_DIAG_DRV_PROT != FALSE) */
                             );
#elif defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
    IO_MLX16_ITC_PEND0_S = (B_MLX16_ITC_PEND0_UV_VDDA                           /* Clear VDDA pending */
                            | B_MLX16_ITC_PEND0_UV_VS                           /* Clear VS pending UV */
                            | B_MLX16_ITC_PEND0_UV_VDDAF                        /* Clear VDDAF pending UV */
                            | B_MLX16_ITC_PEND0_OVT                             /* Clear pending Over-temperature */
                            | B_MLX16_ITC_PEND0_OVC                             /* Clear pending Over-current */
                            | B_MLX16_ITC_PEND0_OV_HS_VDS0
                            | B_MLX16_ITC_PEND0_OV_HS_VDS1
                            | B_MLX16_ITC_PEND0_OV_HS_VDS2
                            | B_MLX16_ITC_PEND0_OV_HS_VDS3);
#if (_SUPPORT_DIAG_VDS != FALSE)
    IO_MLX16_ITC_PEND1_S = (B_MLX16_ITC_PEND1_OV_LS_VDS0
                            | B_MLX16_ITC_PEND1_OV_LS_VDS1
                            | B_MLX16_ITC_PEND1_OV_LS_VDS2
                            | B_MLX16_ITC_PEND1_OV_LS_VDS3);
#endif /* (_SUPPORT_DIAG_VDS != FALSE) */
    IO_MLX16_ITC_PEND2_S = (B_MLX16_ITC_PEND2_OV_VS                             /* Clear pending Over-voltage */
                            | B_MLX16_ITC_PEND2_DIAG);                          /* Clear diagnostics pending */
    /* Set PWM Master1 End IRQ Priority to 5; Used when VDS/OC Motor protection
     * clicks in; If after 'x' PWM-periods no second VDS/OC, the VDS-OC counter
     * is cleared */
#if (_SUPPORT_DIAG_DRV_PROT != FALSE)
    IO_MLX16_ITC_PRIO1_S = (IO_MLX16_ITC_PRIO1_S & ~M_MLX16_ITC_PRIO1_PWM_MASTER1_END) |
                           C_MLX16_ITC_PRIO1_PWM_MASTER1_END_PRIO5;
#endif /* (_SUPPORT_DIAG_DRV_PROT != FALSE) */
    IO_MLX16_ITC_PRIO2_S = (IO_MLX16_ITC_PRIO2_S & ~M_MLX16_ITC_PRIO2_OV_VS) | C_MLX16_ITC_PRIO2_OV_VS_PRIO3;
#if (_SUPPORT_DIAG_DRV_PROT != FALSE)
    IO_MLX16_ITC_PRIO2_S = (IO_MLX16_ITC_PRIO2_S & ~M_MLX16_ITC_PRIO2_DIAG) | C_MLX16_ITC_PRIO2_DIAG_PRIO3;
#endif /* (_SUPPORT_DIAG_DRV_PROT != FALSE) */
    IO_MLX16_ITC_MASK0_S |= (B_MLX16_ITC_MASK0_AWD_ATT                          /* Enable Absolute Watchdog Attention */
                             | B_MLX16_ITC_MASK0_UV_VDDA                        /* Enable VDDA UV */
                             | B_MLX16_ITC_MASK0_UV_VS                          /* Enable VS UV */
/* Don't enable UV VDDAF as long as the driver is not supplied (B_PORT_DRV_OUT_ENABLE_DRVSUP) */
/*                            | B_MLX16_ITC_MASK0_UV_VDDAF */                   /* Enable VDDAF UV */
#if (_SUPPORT_DIAG_OT != FALSE)
                             | B_MLX16_ITC_MASK0_OVT                            /* Enable Over-temperature */
#endif /* (_SUPPORT_DIAG_OT != FALSE) */
#if (_SUPPORT_DIAG_OC != FALSE)
                             | B_MLX16_ITC_MASK0_OVC                            /* Enable Over-current */
#endif /* (_SUPPORT_DIAG_OC != FALSE) */
#if (_SUPPORT_DIAG_VDS != FALSE)
                             | B_MLX16_ITC_MASK0_OV_HS_VDS0                     /* Enable VDS0 HS */
                             | B_MLX16_ITC_MASK0_OV_HS_VDS1                     /* Enable VDS1 HS */
                             | B_MLX16_ITC_MASK0_OV_HS_VDS2                     /* Enable VDS2 HS */
                             | B_MLX16_ITC_MASK0_OV_HS_VDS3                     /* Enable VDS3 HS */
#endif /* (_SUPPORT_DIAG_VDS != FALSE) */
                             );
#if (_SUPPORT_DIAG_VDS != FALSE)
    IO_MLX16_ITC_MASK1_S |= (B_MLX16_ITC_MASK1_OV_LS_VDS0                       /* Enable VDS0 LS */
                             | B_MLX16_ITC_MASK1_OV_LS_VDS1                     /* Enable VDS1 LS */
                             | B_MLX16_ITC_MASK1_OV_LS_VDS2                     /* Enable VDS2 LS */
                             | B_MLX16_ITC_MASK1_OV_LS_VDS3);                   /* Enable VDS3 LS */
#endif /* (_SUPPORT_DIAG_VDS != FALSE) */
    IO_MLX16_ITC_MASK2_S |= (B_MLX16_ITC_MASK2_OV_VS                            /* Enable Over-voltage */
#if (_SUPPORT_DIAG_DRV_PROT != FALSE)
                             | B_MLX16_ITC_MASK2_DIAG                           /* Diagnostic */
#endif /* (_SUPPORT_DIAG_DRV_PROT != FALSE) */
                             );
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    IO_MLX16_ITC_PEND0_S = (B_MLX16_ITC_PEND0_UV_VDDA                           /* Clear VDDA pending */
                            | B_MLX16_ITC_PEND0_UV_VSM                          /* Clear VSM pending UV */
                            | B_MLX16_ITC_PEND0_UV_BOOST                        /* Clear VBOOST pending UV */
                            | B_MLX16_ITC_PEND0_OVT                             /* Clear pending Over-temperature */
                            | B_MLX16_ITC_PEND0_OVC                             /* Clear pending Over-current */
                            | B_MLX16_ITC_PEND0_OV_HS_VDS0);
#if (_SUPPORT_DIAG_VDS != FALSE)
    IO_MLX16_ITC_PEND1_S = (B_MLX16_ITC_PEND1_OV_HS_VDS1
                            | B_MLX16_ITC_PEND1_OV_HS_VDS2
                            | B_MLX16_ITC_PEND1_OV_LS_VDS0
                            | B_MLX16_ITC_PEND1_OV_LS_VDS1
                            | B_MLX16_ITC_PEND1_OV_LS_VDS2);
#endif /* (_SUPPORT_DIAG_VDS != FALSE) */
    IO_MLX16_ITC_PEND2_S = (B_MLX16_ITC_PEND2_DIAG);                            /* Clear diagnostics pending */
    IO_MLX16_ITC_PEND3_S = (B_MLX16_ITC_PEND3_OV_VSM                            /* Clear Over-Voltage VSM */
                            | B_MLX16_ITC_PEND3_OV_VDDA                         /* Clear Over-Voltage VDDA */
                            | B_MLX16_ITC_PEND3_OV_BOOST);                      /* Clear Over-Voltage VBOOST */
    /* Set PWM Master1 End IRQ Priority to 5; Used when VDS/OC Motor protection
     * clicks in; If after 'x' PWM-periods no second VDS/OC, the VDS-OC counter
     * is cleared */
#if (_SUPPORT_DIAG_DRV_PROT != FALSE)
    IO_MLX16_ITC_PRIO1_S = (IO_MLX16_ITC_PRIO1_S & ~M_MLX16_ITC_PRIO1_PWM_MASTER1_END) |
                           C_MLX16_ITC_PRIO1_PWM_MASTER1_END_PRIO5;
#endif /* (_SUPPORT_DIAG_DRV_PROT != FALSE) */
    IO_MLX16_ITC_PRIO3_S = (IO_MLX16_ITC_PRIO3_S & ~(M_MLX16_ITC_PRIO3_OV_VSM |
                                                     M_MLX16_ITC_PRIO3_OV_VDDA |
                                                     M_MLX16_ITC_PRIO3_OV_BOOST)) |
                           (C_MLX16_ITC_PRIO3_OV_VSM_PRIO3 |
                            C_MLX16_ITC_PRIO3_OV_VDDA_PRIO3 |
                            C_MLX16_ITC_PRIO3_OV_BOOST_PRIO3);

#if (_SUPPORT_DIAG_DRV_PROT != FALSE)
    IO_MLX16_ITC_PRIO2_S = (IO_MLX16_ITC_PRIO2_S & ~M_MLX16_ITC_PRIO2_DIAG) | C_MLX16_ITC_PRIO2_DIAG_PRIO3;
#endif /* (_SUPPORT_DIAG_DRV_PROT != FALSE) */
    IO_MLX16_ITC_MASK0_S |= (B_MLX16_ITC_MASK0_AWD_ATT                          /* Enable Absolute Watchdog Attention */
                             | B_MLX16_ITC_MASK0_UV_VDDA                        /* Enable VDDA UV */
                             | B_MLX16_ITC_MASK0_UV_VSM                         /* Enable VSM UV */
/* Don't enable UV VDDAF as long as the driver is not supplied (B_PORT_DRV_OUT_ENABLE_DRVSUP) */
/*                            | B_MLX16_ITC_MASK0_UV_VDDAF */                   /* Enable VDDAF UV */
#if (_SUPPORT_DIAG_OT != FALSE)
                             | B_MLX16_ITC_MASK0_OVT                            /* Enable Over-temperature */
#endif /* (_SUPPORT_DIAG_OT != FALSE) */
#if (_SUPPORT_DIAG_OC != FALSE)
                             | B_MLX16_ITC_MASK0_OVC                            /* Enable Over-current */
#endif /* (_SUPPORT_DIAG_OC != FALSE) */
#if (_SUPPORT_DIAG_VDS != FALSE)
                             | B_MLX16_ITC_MASK0_OV_HS_VDS0                     /* Enable VDS0 HS */
#endif /* (_SUPPORT_DIAG_VDS != FALSE) */
                             );
#if (_SUPPORT_DIAG_VDS != FALSE)
    IO_MLX16_ITC_MASK1_S |= (B_MLX16_ITC_MASK1_OV_HS_VDS1                       /* Enable VDS1 HS */
                             | B_MLX16_ITC_MASK1_OV_HS_VDS2                     /* Enable VDS2 HS */
                             | B_MLX16_ITC_MASK1_OV_LS_VDS0                     /* Enable VDS0 LS */
                             | B_MLX16_ITC_MASK1_OV_LS_VDS1                     /* Enable VDS1 LS */
                             | B_MLX16_ITC_MASK1_OV_LS_VDS2);                   /* Enable VDS2 LS */
#endif /* (_SUPPORT_DIAG_VDS != FALSE) */
#if (_SUPPORT_DIAG_DRV_PROT != FALSE)
    IO_MLX16_ITC_MASK2_S |= B_MLX16_ITC_MASK2_DIAG;                             /* Diagnostic */
#endif /* (_SUPPORT_DIAG_DRV_PROT != FALSE) */
    IO_MLX16_ITC_MASK3_S |= (B_MLX16_ITC_MASK3_OV_VSM                           /* Enable Over-voltage VSM */
                             | B_MLX16_ITC_MASK3_OV_VDDA                        /* Enable Over-Voltage VDDA */
                             | B_MLX16_ITC_MASK3_OV_BOOST                       /* Enable Over-Voltage VBOOST */
                             );
#endif
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */

    /* Check for OVT and OV. Perform Diagnostics handling if required */
#if (_SUPPORT_DIAG_OT != FALSE)
    if ( ((IO_PORT_SUPP_IN & B_PORT_SUPP_IN_OVT_IT) != 0U) ||
         ((IO_PORT_DIAG_IN & B_PORT_DIAG_IN_OVT_MEM) != 0U) )
    {
        HandleDiagnosticsOT();
#if (_SUPPORT_DIAG_IRQCTLR_LEVEL != FALSE)
#if (_SUPPORT_APP_USER_MODE != FALSE)
        ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        IO_MLX16_ITC_MASK0_S &= ~B_MLX16_ITC_MASK0_OVT;                         /* Disable OT */
        l_u16MASK0 |= B_MLX16_ITC_MASK0_OVT;
        IO_MLX16_ITC_PEND0_S = B_MLX16_ITC_PEND0_OVT;                           /* Clear pending flag OVT */
#if (_SUPPORT_APP_USER_MODE != FALSE)
        EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#endif /* (_SUPPORT_DIAG_IRQCTLR_LEVEL != FALSE) */
    }
#endif /* (_SUPPORT_DIAG_OT != FALSE) */
#if defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    if ( ((IO_PORT_SUPP_IN & B_PORT_SUPP_IN_OV_VSM_IT) != 0U) ||
         ((IO_PORT_DIAG_IN & B_PORT_DIAG_IN_OV_VSM_MEM) != 0U) )
#else
    if ( ((IO_PORT_SUPP_IN & B_PORT_SUPP_IN_OV_VS_IT) != 0U) ||
         ((IO_PORT_DIAG_IN & B_PORT_DIAG_IN_OV_VS_MEM) != 0U) )
#endif
    {
        VerifyDiagnosticsUVOV();
#if (_SUPPORT_APP_USER_MODE != FALSE)
        ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#if defined (__MLX81160__)
        IO_MLX16_ITC_MASK2_S &= ~B_MLX16_ITC_MASK2_OV_VSM;                      /* Disable OV VSM (MMP220818-1) */
        l_u16MASK2 |= B_MLX16_ITC_MASK2_OV_VSM;
        IO_MLX16_ITC_PEND2_S = B_MLX16_ITC_PEND2_OV_VSM;                        /* Clear pending flag OV VSM (MMP220818-1) */
#elif defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
        IO_MLX16_ITC_MASK2_S &= ~B_MLX16_ITC_MASK2_OV_VS;                       /* Disable OV VS */
        l_u16MASK2 |= B_MLX16_ITC_MASK2_OV_VS;
        IO_MLX16_ITC_PEND2_S = B_MLX16_ITC_PEND2_OV_VS;                         /* Clear pending flag OV VS */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
        IO_MLX16_ITC_MASK3_S &= ~B_MLX16_ITC_MASK3_OV_VSM;                      /* Disable OV VSM */
        l_u16MASK3 |= B_MLX16_ITC_MASK3_OV_VSM;
        IO_MLX16_ITC_PEND3_S = B_MLX16_ITC_PEND3_OV_VSM;                        /* Clear pending flag OV VSM */
#endif
#if (_SUPPORT_APP_USER_MODE != FALSE)
        EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    }

} /* End of DiagnosticInit() */

/*!*************************************************************************** *
 * DiagnosticReset
 * \brief   Diagnostic Reset (counters)
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Reset Diagnostic counters.
 * *************************************************************************** *
 * - Call Hierarchy: MotorDriverStart()
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
void DiagnosticReset(void)
{
#if (_SUPPORT_DIAG_RINNEN_ROBUSTNESS != FALSE)
    l_u8OC_Count = (uint8_t)0U;
    l_u8VDS_Count = (uint8_t)0U;
#endif /* (_SUPPORT_DIAG_RINNEN_ROBUSTNESS != FALSE) */
#if defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
    l_u8VDDAF_Count = (uint8_t)0U;
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    l_u8VBOOST_Count = 0U;
#endif
} /* End of DiagnosticReset() */

#if (_SUPPORT_DIAG_DRV_PROT != FALSE)
/*!*************************************************************************** *
 * ISR_DIAG
 * \brief   Handle DRV PROT Diagnostic Events
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details IRQ-Priority: 3
 * *************************************************************************** *
 * - Call Hierarchy: IRQ
 * - Cyclomatic Complexity: 6+1
 * - Nesting: 1
 * - Function calling: 7 (HandleDiagnosticsVDS(), HandleDiagnosticsOC(),
 *                        HandleDiagnosticsVDDAF(), HandleDiagnosticsVDDA(),
 *                        HandleDiagnosticsUV(), HandleDiagnosticsOV(),
 *                        HandleDiagnosticsOT())
 * *************************************************************************** */
__attribute__((interrupt)) void ISR_DIAG(void)
{
#if (_DEBUG_CPU_LOAD != FALSE)
    DEBUG_SET_IO_A();                                                           /* IRQ-Priority: 3 */
#endif /* (_DEBUG_CPU_LOAD != FALSE) */
#if (_DEBUG_DIAG_PROT != FALSE)
    DEBUG_SET_IO_A();
#endif /* (_DEBUG_DIAG_PROT != FALSE) */

#if !defined (__MLX81160__) && (_SUPPORT_DRV_PROT_VDS != FALSE)
    if ( (IO_PORT_DIAG_IN & (M_PORT_DIAG_IN_OV_LS_VDS_MEM | M_PORT_DIAG_IN_OV_HS_VDS_MEM)) != 0U)
    {
        HandleDiagnosticsVDS();
    }
#endif /* !defined (__MLX81160__) && (_SUPPORT_DRV_PROT_VDS != FALSE) */

#if (_SUPPORT_DRV_PROT_OC != FALSE)
#if defined (__MLX81160__)
    if ( (IO_PORT_DIAG_IN & M_PORT_DIAG_IN_OVC_MEM) != 0U)
#else  /* defined (__MLX81160__) */
    if ( (IO_PORT_DIAG_IN & B_PORT_DIAG_IN_OVC_MEM) != 0U)
#endif /* defined (__MLX81160__) */
    {
        HandleDiagnosticsOC();
    }
#endif /*  (_SUPPORT_DRV_PROT_OC != FALSE) */
#if defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__) || defined (__MLX81350__)
#if (_SUPPORT_DRV_PROT_UV_VDDAF != FALSE)
    if ( (IO_PORT_DIAG_IN & B_PORT_DIAG_IN_UV_VDDAF_MEM) != 0U)
    {
#if (_SUPPORT_DRVSUP_ALWAYS_ENA != FALSE)
        (void)HandleDiagnosticsVDDAF();
#else  /* (_SUPPORT_DRVSUP_ALWAYS_ENA != FALSE) */
        HandleDiagnosticsVDDAF();
#endif /* (_SUPPORT_DRVSUP_ALWAYS_ENA != FALSE) */
    }
#endif /* (_SUPPORT_DRV_PROT_UV_VDDAF != FALSE) */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#if (_SUPPORT_DRV_PROT_UV_VBOOST != FALSE) || (_SUPPORT_DRV_PROT_OV_VBOOST != FALSE)
    if ( (IO_PORT_DIAG_IN & (B_PORT_DIAG_IN_UV_BOOST_MEM | B_PORT_DIAG_IN_OV_BOOST_MEM)) != 0U)
    {
        HandleDiagnosticsVBOOST(2U);
    }
#endif /* (_SUPPORT_DRV_PROT_UV_VBOOST != FALSE) || (_SUPPORT_DRV_PROT_OV_VBOOST != FALSE) */
#endif
#if (_SUPPORT_DRV_PROT_UV_VDDA != FALSE)
#if defined (__MLX81160__)
    if ( (IO_PORT_DIAG_IN & (B_PORT_DIAG_IN_UV_VDDA_MEM | B_PORT_DIAG_IN_OV_VDDA_MEM | B_PORT_DIAG_IN_OC_VDDA_MEM)) !=
         0U)
#elif defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
    if ( (IO_PORT_DIAG_IN & B_PORT_DIAG_IN_UV_VDDA_MEM) != 0U)
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    if ( (IO_PORT_DIAG_IN & (B_PORT_DIAG_IN_UV_VDDA_MEM | B_PORT_DIAG_IN_OV_VDDA_MEM | B_PORT_DIAG_IN_OC_VDDA_MEM)) !=
         0U)
#endif
    {
        HandleDiagnosticsVDDA();
    }
#endif /* (_SUPPORT_DRV_PROT_UV_VDDA != FALSE) */
#if (_SUPPORT_DRV_PROT_UV_VS != FALSE)                                          /* (MMP220613-1) */
    if ( (IO_PORT_DIAG_IN & B_PORT_DIAG_IN_UV_VS_MEM) != 0U)
    {
        HandleDiagnosticsUV();
    }
#endif /* (_SUPPORT_DRV_PROT_UV_VS != FALSE) */
#if (_SUPPORT_DRV_PROT_OV_VS != FALSE)                                          /* (MMP220613-1) */
    if ( (IO_PORT_DIAG_IN & B_PORT_DIAG_IN_OV_VS_MEM) != 0U)
    {
        HandleDiagnosticsOV();
    }
#endif /* (_SUPPORT_DRV_PROT_OV_VS != FALSE) */
#if (_SUPPORT_DRV_PROT_OT != FALSE)
    if ( (IO_PORT_DIAG_IN & B_PORT_DIAG_IN_OVT_MEM) != 0U)
    {
        HandleDiagnosticsOT();
    }
#endif /* (_SUPPORT_DRV_PROT_OT != FALSE) */

#if (_SUPPORT_DIAG_IRQCTLR_LEVEL != FALSE)
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#if FALSE                                                                       /* MMP220316-1 */
    IO_MLX16_ITC_MASK2_S &= ~B_MLX16_ITC_MASK2_DIAG;                            /* Disable DIAG (DRV_PROT) */
    l_u16MASK2 |= B_MLX16_ITC_MASK2_DIAG;
#endif /* FALSE */
    IO_MLX16_ITC_PEND2_S = B_MLX16_ITC_PEND2_DIAG;                              /* Clear DIAG pending (DRV_PROT) */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#endif /* (_SUPPORT_DIAG_IRQCTLR_LEVEL != FALSE) */

#if (_DEBUG_DIAG_PROT != FALSE)
    DEBUG_CLR_IO_A();
#endif /* (_DEBUG_DIAG_PROT != FALSE) */
#if (_DEBUG_CPU_LOAD != FALSE)
    DEBUG_CLR_IO_A();                                                           /* IRQ-Priority: 3 */
#endif /* (_DEBUG_CPU_LOAD != FALSE) */
} /* End of ISR_DIAG() */
#endif /* (_SUPPORT_DIAG_DRV_PROT != FALSE) */

/*!*************************************************************************** *
 * ISR_UV_VDDA
 * \brief   Handle Diagnostic VDDA Under Voltage Event
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details IRQ-Priority: 1
 * *************************************************************************** *
 * - Call Hierarchy: IRQ
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 1 (HandleDiagnosticsVDDA())
 * *************************************************************************** */
__attribute__((interrupt)) void ISR_UV_VDDA(void)
{
#if (_DEBUG_CPU_LOAD != FALSE)
    DEBUG_SET_IO_E();                                                           /* IRQ-Priority: 1 */
#endif /* (_DEBUG_CPU_LOAD != FALSE) */
#if (_DEBUG_DIAG_UV_VDDA != FALSE)
    DEBUG_SET_IO_A();
#endif /* (_DEBUG_DIAG_UV_VDDA != FALSE) */
    HandleDiagnosticsVDDA();
#if (_SUPPORT_DIAG_IRQCTLR_LEVEL != FALSE)
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    IO_MLX16_ITC_MASK0_S &= ~B_MLX16_ITC_MASK0_UV_VDDA;                         /* Disable UV VDDA */
    l_u16MASK0 |= B_MLX16_ITC_MASK0_UV_VDDA;
    IO_MLX16_ITC_PEND0_S = B_MLX16_ITC_PEND0_UV_VDDA;                           /* Clear pending flag UV VDDA */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#endif /* (_SUPPORT_DIAG_IRQCTLR_LEVEL != FALSE) */
#if (_DEBUG_DIAG_UV_VDDA != FALSE)
    DEBUG_CLR_IO_A();
#endif /* (_DEBUG_DIAG_UV_VDDA != FALSE) */
#if (_DEBUG_CPU_LOAD != FALSE)
    DEBUG_CLR_IO_E();                                                           /* IRQ-Priority: 1 */
#endif /* (_DEBUG_CPU_LOAD != FALSE) */
} /* End of ISR_UV_VDDA() */

#if defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
/*!*************************************************************************** *
 * ISR_UV_VDDAF
 * \brief   Handle Diagnostic VDDAF Under Voltage Event
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details IRQ-Priority: 1
 * *************************************************************************** *
 * - Call Hierarchy: IRQ
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 1 (HandleDiagnosticsVDDAF())
 * *************************************************************************** */
__attribute__((interrupt)) void ISR_UV_VDDAF(void)
{
#if (_DEBUG_CPU_LOAD != FALSE)
    DEBUG_SET_IO_E();                                                           /* IRQ-Priority: 1 */
#endif /* (_DEBUG_CPU_LOAD != FALSE) */
#if (_DEBUG_DIAG_VDDAF_UV != FALSE)
    DEBUG_SET_IO_E();
#endif /* (_DEBUG_DIAG_VDDAF_UV != FALSE) */

#if (_SUPPORT_DRVSUP_ALWAYS_ENA != FALSE)
    if (HandleDiagnosticsVDDAF() != FALSE)                                      /* MMP210618-1 */
    {
#if (_SUPPORT_DIAG_IRQCTLR_LEVEL != FALSE)
#if (_SUPPORT_APP_USER_MODE != FALSE)
        ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        IO_MLX16_ITC_MASK0_S &= ~B_MLX16_ITC_MASK0_UV_VDDAF;                    /* Disable UV VDDAF */
        l_u16MASK0 |= B_MLX16_ITC_MASK0_UV_VDDAF;
        IO_MLX16_ITC_PEND0_S = B_MLX16_ITC_PEND0_UV_VDDAF;                      /* Clear pending flag UV VDDAF */
#if (_SUPPORT_APP_USER_MODE != FALSE)
        EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#endif /* (_SUPPORT_DIAG_IRQCTLR_LEVEL != FALSE) */
    }
#else  /* (_SUPPORT_DRVSUP_ALWAYS_ENA != FALSE) */
    HandleDiagnosticsVDDAF();
#endif /* (_SUPPORT_DRVSUP_ALWAYS_ENA != FALSE) */
#if (_DEBUG_DIAG_VDDAF_UV != FALSE)
    DEBUG_CLR_IO_E();
#endif /* (_DEBUG_DIAG_VDDAF_UV != FALSE) */
#if (_DEBUG_CPU_LOAD != FALSE)
    DEBUG_CLR_IO_E();                                                           /* IRQ-Priority: 1 */
#endif /* (_DEBUG_CPU_LOAD != FALSE) */
} /* End of ISR_UV_VDDAF() */
#endif /* defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__) */

#if defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
/*!*************************************************************************** *
 * ISR_OC_VDDA
 * \brief   Handle Diagnostic VDDA Over Current Event
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details IRQ-Priority: 2
 * *************************************************************************** *
 * - Call Hierarchy: IRQ
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 1 (HandleDiagnosticsVDDA())
 * *************************************************************************** */
__attribute__((interrupt)) void ISR_OC_VDDA(void)
{
#if (_DEBUG_CPU_LOAD != FALSE)
    DEBUG_SET_IO_A();                                                           /* IRQ-Priority: 2 */
#endif /* (_DEBUG_CPU_LOAD != FALSE) */
#if (_DEBUG_DIAG_UV_VDDA != FALSE)
    DEBUG_SET_IO_A();
#endif /* (_DEBUG_DIAG_UV_VDDA != FALSE) */
    HandleDiagnosticsVDDA();
#if (_SUPPORT_DIAG_IRQCTLR_LEVEL != FALSE)
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    IO_MLX16_ITC_MASK0_S &= ~B_MLX16_ITC_MASK0_OC_VDDA;                         /* Disable OC VDDA */
    l_u16MASK0 |= B_MLX16_ITC_MASK0_OC_VDDA;
    IO_MLX16_ITC_PEND0_S = B_MLX16_ITC_PEND0_OC_VDDA;                           /* Clear pending flag OC VDDA */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#endif /* (_SUPPORT_DIAG_IRQCTLR_LEVEL != FALSE) */
#if (_DEBUG_DIAG_UV_VDDA != FALSE)
    DEBUG_CLR_IO_A();
#endif /* (_DEBUG_DIAG_UV_VDDA != FALSE) */
#if (_DEBUG_CPU_LOAD != FALSE)
    DEBUG_CLR_IO_A();                                                           /* IRQ-Priority: 2 */
#endif /* (_DEBUG_CPU_LOAD != FALSE) */
} /* End of ISR_OC_VDDA() */

/*!*************************************************************************** *
 * ISR_OV_VDDA
 * \brief   Handle Diagnostic VDDA Over Voltage Event
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details IRQ-Priority: 3
 * *************************************************************************** *
 * - Call Hierarchy: IRQ
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 1 (HandleDiagnosticsVDDA())
 * *************************************************************************** */
__attribute__((interrupt)) void ISR_OV_VDDA(void)
{
#if (_DEBUG_CPU_LOAD != FALSE)
    DEBUG_SET_IO_A();                                                           /* IRQ-Priority: 3 */
#endif /* (_DEBUG_CPU_LOAD != FALSE) */
#if (_DEBUG_DIAG_UV_VDDA != FALSE)
    DEBUG_SET_IO_A();
#endif /* (_DEBUG_DIAG_UV_VDDA != FALSE) */
    HandleDiagnosticsVDDA();
#if (_SUPPORT_DIAG_IRQCTLR_LEVEL != FALSE)
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#if defined (__MLX81160__)
    IO_MLX16_ITC_MASK2_S &= ~B_MLX16_ITC_MASK2_OV_VDDA;                           /* Disable OV VDDA */
    l_u16MASK2 |= B_MLX16_ITC_MASK2_OV_VDDA;
    IO_MLX16_ITC_PEND2_S = B_MLX16_ITC_PEND2_OV_VDDA;                             /* Clear pending flag OV VDDA */
#else /* defined (__MLX81160__) */
    IO_MLX16_ITC_MASK3_S &= ~B_MLX16_ITC_MASK3_OV_VDDA;                           /* Disable OV VDDA */
    l_u16MASK3 |= B_MLX16_ITC_MASK3_OV_VDDA;
    IO_MLX16_ITC_PEND3_S = B_MLX16_ITC_PEND3_OV_VDDA;                             /* Clear pending flag OV VDDA */
#endif /* defined (__MLX81160__) */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#endif /* (_SUPPORT_DIAG_IRQCTLR_LEVEL != FALSE) */
#if (_DEBUG_DIAG_UV_VDDA != FALSE)
    DEBUG_CLR_IO_A();
#endif /* (_DEBUG_DIAG_UV_VDDA != FALSE) */
#if (_DEBUG_CPU_LOAD != FALSE)
    DEBUG_CLR_IO_A();                                                           /* IRQ-Priority: 3 */
#endif /* (_DEBUG_CPU_LOAD != FALSE) */
} /* End of ISR_OV_VDDA() */
#endif /* defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */

#if (defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)) && _SUPPORT_DIAG_BOOST
/*!*************************************************************************** *
 * ISR_UV_VBOOST
 * \brief   Handle Diagnostic BOOST Under Voltage Event
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details IRQ-Priority: 1
 * *************************************************************************** *
 * - Call Hierarchy: IRQ
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 1 (HandleDiagnosticsVBOOST())
 * *************************************************************************** */
__attribute__((interrupt)) void ISR_UV_VBOOST(void)
{
#if (_DEBUG_CPU_LOAD != FALSE)
    DEBUG_SET_IO_A();                                                           /* IRQ-Priority: 1 */
#endif /* (_DEBUG_CPU_LOAD */
#if (_DEBUG_DIAG_VBOOST != FALSE)
    DEBUG_SET_IO_D();
#endif /* (_DEBUG_DIAG_VBOOST != FALSE) */
#if (_SUPPORT_DIAG_IRQCTLR_LEVEL != FALSE)
    if (HandleDiagnosticsVBOOST(1U) != FALSE)                                   /* UV VBoost */
    {
#if (_SUPPORT_APP_USER_MODE != FALSE)
        ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        IO_MLX16_ITC_MASK0_S &= ~B_MLX16_ITC_MASK0_UV_BOOST;                    /* Disable UV BOOST */
        l_u16MASK0 |= B_MLX16_ITC_MASK0_UV_BOOST;
        IO_MLX16_ITC_PEND0_S = B_MLX16_ITC_PEND0_UV_BOOST;                      /* Clear pending flag UV BOOST */
#if (_SUPPORT_APP_USER_MODE != FALSE)
        EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    }
#else  /* _SUPPORT_DIAG_IRQCTLR_LEVEL */
    (void)HandleDiagnosticsVBOOST(1U);                                          /* UV VBoost */
#endif /* (_SUPPORT_DIAG_IRQCTLR_LEVEL != FALSE) */
#if (_DEBUG_DIAG_VBOOST != FALSE)
    DEBUG_CLR_IO_D();
    DEBUG_SET_IO_D();
    DEBUG_CLR_IO_D();
#endif /* (_DEBUG_DIAG_VBOOST != FALSE) */
#if (_DEBUG_CPU_LOAD != FALSE)
    DEBUG_CLR_IO_A();                                                           /* IRQ-Priority: 1 */
#endif /* (_DEBUG_CPU_LOAD != FALSE) */
} /* End of ISR_UV_VBOOST() */

/*!*************************************************************************** *
 * ISR_OV_VBOOST
 * \brief   Handle Diagnostic BOOST Over Voltage Event
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details IRQ-Priority: 3
 * *************************************************************************** *
 * - Call Hierarchy: IRQ
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 1 (HandleDiagnosticsVBOOST())
 * *************************************************************************** */
__attribute__((interrupt)) void ISR_OV_VBOOST(void)
{
#if (_DEBUG_CPU_LOAD != FALSE)
    DEBUG_SET_IO_A();                                                           /* IRQ-Priority: 3 */
#endif /* (_DEBUG_CPU_LOAD != FALSE) */
#if (_DEBUG_DIAG_VBOOST != FALSE)
    DEBUG_SET_IO_D();
#endif /* (_DEBUG_DIAG_VBOOST != FALSE) */
#if (_SUPPORT_DIAG_IRQCTLR_LEVEL != FALSE)
    if (HandleDiagnosticsVBOOST(2U) != FALSE)                                   /* OV VBoost */
    {
#if (_SUPPORT_APP_USER_MODE != FALSE)
        ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        IO_MLX16_ITC_MASK3_S &= ~B_MLX16_ITC_MASK3_OV_BOOST;                    /* Disable OV BOOST */
        l_u16MASK3 |= B_MLX16_ITC_MASK3_OV_BOOST;
        IO_MLX16_ITC_PEND3_S = B_MLX16_ITC_PEND3_OV_BOOST;                      /* Clear pending flag OV BOOST */
#if (_SUPPORT_APP_USER_MODE != FALSE)
        EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    }
#else  /* _SUPPORT_DIAG_IRQCTLR_LEVEL */
    (void)HandleDiagnosticsVBOOST(2U);                                           /* OV VBoost */
#endif /* (_SUPPORT_DIAG_IRQCTLR_LEVEL != FALSE) */
#if (_DEBUG_DIAG_VBOOST != FALSE)
    DEBUG_CLR_IO_D();
#endif /* (_DEBUG_DIAG_VBOOST != FALSE) */
#if (_DEBUG_CPU_LOAD != FALSE)
    DEBUG_CLR_IO_A();                                                           /* IRQ-Priority: 3 */
#endif /* (_DEBUG_CPU_LOAD != FALSE) */
} /* End of ISR_OV_VBOOST() */
#endif /* (defined (__MLX81340__) || (__MLX81344__) || (__MLX81346__)) && _SUPPORT_DIAG_BOOST */

#if !defined (__MLX81160__) && (_SUPPORT_DIAG_VDS != FALSE)
/*!*************************************************************************** *
 * ISR_VDS
 * \brief   Handle Diagnostic VDS Event
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details IRQ-Priority: 2
 * *************************************************************************** *
 * - Call Hierarchy: IRQ
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 1 (HandleDiagnosticsVDS())
 * *************************************************************************** */
__attribute__((interrupt)) void ISR_VDS(void)
{
#if (_DEBUG_CPU_LOAD != FALSE)
    DEBUG_SET_IO_A();                                                           /* IRQ-Priority: 2 */
#endif /* (_DEBUG_CPU_LOAD != FALSE) */
#if (_DEBUG_DIAG_VDS != FALSE)
    DEBUG_SET_IO_B();
#endif /* (_DEBUG_DIAG_VDS != FALSE) */

    HandleDiagnosticsVDS();
#if (_SUPPORT_DIAG_IRQCTLR_LEVEL != FALSE)
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
    IO_MLX16_ITC_MASK0_S &= ~(B_MLX16_ITC_MASK0_OV_HS_VDS0 | B_MLX16_ITC_MASK0_OV_HS_VDS1 |
                              B_MLX16_ITC_MASK0_OV_HS_VDS2 | B_MLX16_ITC_MASK0_OV_HS_VDS3); /* Disable HS_VDS */
    l_u16MASK0 |= (B_MLX16_ITC_MASK0_OV_HS_VDS0 | B_MLX16_ITC_MASK0_OV_HS_VDS1 |
                   B_MLX16_ITC_MASK0_OV_HS_VDS2 | B_MLX16_ITC_MASK0_OV_HS_VDS3);
    IO_MLX16_ITC_PEND0_S = (B_MLX16_ITC_PEND0_OV_HS_VDS0 | B_MLX16_ITC_PEND0_OV_HS_VDS1 |
                            B_MLX16_ITC_PEND0_OV_HS_VDS2 | B_MLX16_ITC_PEND0_OV_HS_VDS3);  /* Clear pending flag HS_VDS */
    IO_MLX16_ITC_MASK1_S &= ~(B_MLX16_ITC_MASK1_OV_LS_VDS0 | B_MLX16_ITC_MASK1_OV_LS_VDS1 |
                              B_MLX16_ITC_MASK1_OV_LS_VDS2 | B_MLX16_ITC_MASK1_OV_LS_VDS3); /* Disable LS_VDS */
    l_u16MASK1 |= (B_MLX16_ITC_MASK1_OV_LS_VDS0 | B_MLX16_ITC_MASK1_OV_LS_VDS1 |
                   B_MLX16_ITC_MASK1_OV_LS_VDS2 | B_MLX16_ITC_MASK1_OV_LS_VDS3);
    IO_MLX16_ITC_PEND1_S = (B_MLX16_ITC_PEND1_OV_LS_VDS0 | B_MLX16_ITC_PEND1_OV_LS_VDS1 |
                            B_MLX16_ITC_PEND1_OV_LS_VDS2 | B_MLX16_ITC_PEND1_OV_LS_VDS3);  /* Clear pending flag HS_VDS */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    IO_MLX16_ITC_MASK0_S &= ~(B_MLX16_ITC_MASK0_OV_HS_VDS0);                      /* Disable HS_VDS */
    l_u16MASK0 |= B_MLX16_ITC_MASK0_OV_HS_VDS0;
    IO_MLX16_ITC_PEND0_S = B_MLX16_ITC_PEND0_OV_HS_VDS0;                          /* Clear pending flag HS_VDS */
    IO_MLX16_ITC_MASK1_S &= ~(B_MLX16_ITC_MASK1_OV_HS_VDS1 | B_MLX16_ITC_MASK1_OV_HS_VDS2 |
                              B_MLX16_ITC_MASK1_OV_LS_VDS0 | B_MLX16_ITC_MASK1_OV_LS_VDS1 |
                              B_MLX16_ITC_MASK1_OV_LS_VDS2);                    /* Disable LS_VDS */
    l_u16MASK1 |= (B_MLX16_ITC_MASK1_OV_HS_VDS1 | B_MLX16_ITC_MASK1_OV_HS_VDS2 |
                   B_MLX16_ITC_MASK1_OV_LS_VDS0 | B_MLX16_ITC_MASK1_OV_LS_VDS1 |
                   B_MLX16_ITC_MASK1_OV_LS_VDS2);
    IO_MLX16_ITC_PEND1_S = (B_MLX16_ITC_PEND1_OV_HS_VDS1 | B_MLX16_ITC_PEND1_OV_HS_VDS2 |
                            B_MLX16_ITC_PEND1_OV_LS_VDS0 | B_MLX16_ITC_PEND1_OV_LS_VDS1 |
                            B_MLX16_ITC_PEND1_OV_LS_VDS2);                      /* Clear pending flag HS_VDS */
#endif
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#endif /* (_SUPPORT_DIAG_IRQCTLR_LEVEL != FALSE) */

#if (_DEBUG_DIAG_VDS != FALSE)
    DEBUG_CLR_IO_B();
#endif /* (_DEBUG_DIAG_VDS != FALSE) */
#if (_DEBUG_CPU_LOAD != FALSE)
    DEBUG_CLR_IO_A();                                                           /* IRQ-Priority: 2 */
#endif /* (_DEBUG_CPU_LOAD != FALSE) */
} /* End of ISR_VDS() */
#endif /* !defined (__MLX81160__) && (_SUPPORT_DIAG_VDS != FALSE) */

#if (_SUPPORT_DIAG_OC != FALSE)
/*!*************************************************************************** *
 * ISR_OC
 * \brief   Handle Diagnostic Over Current Event
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details IRQ-Priority: 2
 * *************************************************************************** *
 * - Call Hierarchy: IRQ
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 1 (HandleDiagnosticsOC())
 * *************************************************************************** */
__attribute__((interrupt)) void ISR_OC(void)
{
#if (_DEBUG_CPU_LOAD != FALSE)
    DEBUG_SET_IO_E();                                                           /* IRQ-Priority: 2 */
#endif /* (_DEBUG_CPU_LOAD != FALSE) */
#if (_DEBUG_DIAG_OC != FALSE)
    DEBUG_SET_IO_C();
#endif /* (_DEBUG_DIAG_OC != FALSE) */
    HandleDiagnosticsOC();
#if (_SUPPORT_DIAG_IRQCTLR_LEVEL != FALSE)
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#if defined (__MLX81160__)
#if (_SUPPORT_ADC_CUR_SENSE_CH == ADC_CUR_SENSE_CH1)
    IO_MLX16_ITC_MASK0_S &= ~B_MLX16_ITC_MASK0_OVC0;                            /* Disable OC */
    l_u16MASK0 = B_MLX16_ITC_MASK0_OVC0;
    IO_MLX16_ITC_PEND0_S = B_MLX16_ITC_PEND0_OVC0;                              /* Clear pending flag OVC */
#elif (_SUPPORT_ADC_CUR_SENSE_CH == ADC_CUR_SENSE_CH2)
    IO_MLX16_ITC_MASK0_S &= ~B_MLX16_ITC_MASK0_OVC1;                            /* Disable OC */
    l_u16MASK0 |= B_MLX16_ITC_MASK0_OVC1;
    IO_MLX16_ITC_PEND0_S = B_MLX16_ITC_PEND0_OVC1;                              /* Clear pending flag OVC */
#else /* (_SUPPORT_ADC_CUR_SENSE_CH == ADC_CUR_SENSE_CHX) */
    IO_MLX16_ITC_MASK0_S &= ~(B_MLX16_ITC_MASK0_OVC0 | B_MLX16_ITC_MASK0_OVC1);  /* Disable OC */
    l_u16MASK0 |= (B_MLX16_ITC_MASK0_OVC0 | B_MLX16_ITC_MASK0_OVC1);
    IO_MLX16_ITC_PEND0_S = (B_MLX16_ITC_PEND0_OVC0 | B_MLX16_ITC_PEND0_OVC1);   /* Clear pending flag OVC */
#endif
#else  /* defined (__MLX81160__) */
    IO_MLX16_ITC_MASK0_S &= ~B_MLX16_ITC_MASK0_OVC;                             /* Disable OC */
    l_u16MASK0 |= B_MLX16_ITC_MASK0_OVC;
    IO_MLX16_ITC_PEND0_S = B_MLX16_ITC_PEND0_OVC;                               /* Clear pending flag OVC */
#endif /* defined (__MLX81160__) */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#endif /* (_SUPPORT_DIAG_IRQCTLR_LEVEL != FALSE) */
#if (_DEBUG_DIAG_OC != FALSE)
    DEBUG_CLR_IO_C();
#endif /* (_DEBUG_DIAG_OC != FALSE) */
#if (_DEBUG_CPU_LOAD != FALSE)
    DEBUG_CLR_IO_E();                                                           /* IRQ-Priority: 2 */
#endif /* (_DEBUG_CPU_LOAD != FALSE) */
} /* ISR_OC() */
#endif /* (_SUPPORT_DIAG_OC != FALSE) */

#if (_SUPPORT_DIAG_OT != FALSE)
/*!*************************************************************************** *
 * ISR_OT
 * \brief   Handle Diagnostic Over Temperature Event
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details IRQ-Priority: 2
 * *************************************************************************** *
 * - Call Hierarchy: IRQ
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 1 (HandleDiagnosticsOT())
 * *************************************************************************** */
__attribute__((interrupt)) void ISR_OT(void)
{
#if (_DEBUG_CPU_LOAD != FALSE)
    DEBUG_SET_IO_E();                                                           /* IRQ-Priority: 2 */
#endif /* (_DEBUG_CPU_LOAD != FALSE) */
#if (_DEBUG_DIAG_OT != FALSE)
    DEBUG_SET_IO_A();
#endif /* (_DEBUG_DIAG_OT != FALSE) */
    HandleDiagnosticsOT();
#if (_SUPPORT_DIAG_IRQCTLR_LEVEL != FALSE)
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    IO_MLX16_ITC_MASK0_S &= ~B_MLX16_ITC_MASK0_OVT;                             /* Disable OT */
    l_u16MASK0 |= B_MLX16_ITC_MASK0_OVT;
    IO_MLX16_ITC_PEND0_S = B_MLX16_ITC_PEND0_OVT;                               /* Clear pending flag OVT */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#endif /* (_SUPPORT_DIAG_IRQCTLR_LEVEL != FALSE) */
#if (_DEBUG_DIAG_OT != FALSE)
    DEBUG_CLR_IO_A();
#endif /* (_DEBUG_DIAG_OT != FALSE) */
#if (_DEBUG_CPU_LOAD != FALSE)
    DEBUG_CLR_IO_E();                                                           /* IRQ-Priority: 2 */
#endif /* (_DEBUG_CPU_LOAD != FALSE) */
} /* ISR_OT() */
#endif /* (_SUPPORT_DIAG_OT != FALSE) */

/*!*************************************************************************** *
 * ISR_UV_VS
 * \brief   Handle Diagnostic Under Voltage Event
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details IRQ-Priority: 1
 * *************************************************************************** *
 * - Call Hierarchy: IRQ
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 1 (VerifyDiagnosticsUVOV())
 * *************************************************************************** */
__attribute__((interrupt)) void ISR_UV_VS(void)
{
#if (_DEBUG_CPU_LOAD != FALSE) || (_DEBUG_DIAG_OV_UV != FALSE) || (_DEBUG_SUPPLY_ROBUSTNESS != FALSE)
    DEBUG_SET_IO_E();                                                           /* IRQ-Priority: 1 */
#if (_DEBUG_SUPPLY_ROBUSTNESS != FALSE)
    DEBUG_SET_IO_B();
    DEBUG_CLR_IO_B();
#endif /* (_DEBUG_SUPPLY_ROBUSTNESS != FALSE) */
#endif /* (_DEBUG_CPU_LOAD != FALSE) || (_DEBUG_DIAG_OV_UV != FALSE) || (_DEBUG_SUPPLY_ROBUSTNESS != FALSE) */
    VerifyDiagnosticsUVOV();
#if (_SUPPORT_DIAG_IRQCTLR_LEVEL != FALSE)
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#if defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    IO_MLX16_ITC_MASK0_S &= ~B_MLX16_ITC_MASK0_UV_VSM;                          /* Disable UV VSM (MMP220818-1) */
    l_u16MASK0 |= B_MLX16_ITC_MASK0_UV_VSM;
    IO_MLX16_ITC_PEND0_S = B_MLX16_ITC_PEND0_UV_VSM;                            /* Clear pending flag UV VSM (MMP220818-1) */
#else  /* defined (__MLX81160__) */
    IO_MLX16_ITC_MASK0_S &= ~B_MLX16_ITC_MASK0_UV_VS;                           /* Disable UV VS */
    l_u16MASK0 |= B_MLX16_ITC_MASK0_UV_VS;
    IO_MLX16_ITC_PEND0_S = B_MLX16_ITC_PEND0_UV_VS;                             /* Clear pending flag UV VS */
#endif /* defined (__MLX81160__) */
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#endif /* (_SUPPORT_DIAG_IRQCTLR_LEVEL != FALSE) */
#if (_DEBUG_CPU_LOAD != FALSE) || (_DEBUG_DIAG_OV_UV != FALSE) || (_DEBUG_SUPPLY_ROBUSTNESS != FALSE)
#if (_DEBUG_SUPPLY_ROBUSTNESS != FALSE)
    DEBUG_SET_IO_B();
    DEBUG_CLR_IO_B();
#endif /* (_DEBUG_SUPPLY_ROBUSTNESS != FALSE) */
    DEBUG_CLR_IO_E();                                                           /* IRQ-Priority: 1 */
#endif /* (_DEBUG_CPU_LOAD != FALSE) || (_DEBUG_DIAG_OV_UV != FALSE) || (_DEBUG_SUPPLY_ROBUSTNESS != FALSE) */
} /* ISR_UV_VS() */

/*!*************************************************************************** *
 * ISR_OV_VS
 * \brief   Handle Diagnostic Over Voltage Event
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details IRQ-Priority: 3
 * *************************************************************************** *
 * - Call Hierarchy: IRQ
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 1 (VerifyDiagnosticsUVOV())
 * *************************************************************************** */
__attribute__((interrupt)) void ISR_OV_VS(void)
{
#if (_DEBUG_CPU_LOAD != FALSE) || (_DEBUG_DIAG_OV_UV != FALSE)
    DEBUG_SET_IO_A();                                                           /* IRQ-Priority: 3 */
#endif /* (_DEBUG_CPU_LOAD != FALSE) || (_DEBUG_DIAG_OV_UV != FALSE) */
    VerifyDiagnosticsUVOV();
#if (_SUPPORT_DIAG_IRQCTLR_LEVEL != FALSE)
#if (_SUPPORT_APP_USER_MODE != FALSE)
    ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#if defined (__MLX81160__)
    IO_MLX16_ITC_MASK2_S &= ~B_MLX16_ITC_MASK2_OV_VSM;                          /* Disable OV VSM (MMP220818-1) */
    l_u16MASK2 |= B_MLX16_ITC_MASK2_OV_VSM;
    IO_MLX16_ITC_PEND2_S = B_MLX16_ITC_PEND2_OV_VSM;                            /* Clear pending flag OV VSM (MMP220818-1) */
#elif defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
    IO_MLX16_ITC_MASK2_S &= ~B_MLX16_ITC_MASK2_OV_VS;                           /* Disable OV VS */
    l_u16MASK2 |= B_MLX16_ITC_MASK2_OV_VS;
    IO_MLX16_ITC_PEND2_S = B_MLX16_ITC_PEND2_OV_VS;                             /* Clear pending flag OV VS */
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    IO_MLX16_ITC_MASK3_S &= ~B_MLX16_ITC_MASK3_OV_VSM;                           /* Disable OV VS */
    l_u16MASK3 |= B_MLX16_ITC_MASK3_OV_VSM;
    IO_MLX16_ITC_PEND3_S = B_MLX16_ITC_PEND3_OV_VSM;                             /* Clear pending flag OV VS */
#endif
#if (_SUPPORT_APP_USER_MODE != FALSE)
    EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#endif /* (_SUPPORT_DIAG_IRQCTLR_LEVEL != FALSE) */
#if (_DEBUG_CPU_LOAD != FALSE) || (_DEBUG_DIAG_OV_UV != FALSE)
    DEBUG_CLR_IO_A();                                                           /* IRQ-Priority: 3 */
#endif /* (_DEBUG_CPU_LOAD != FALSE) || (_DEBUG_DIAG_OV_UV != FALSE) */
} /* ISR_OV_VS() */

#if (_SUPPORT_DIAG_IRQCTLR_LEVEL != FALSE)
/*!*************************************************************************** *
 * DiagnosticPeriodicTimerEvent
 * \brief   Restore Diagnostics IRQ MASK (Level sensitive only)
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: ISR_STIMER
 * - Cyclomatic Complexity: 11+1
 * - Nesting: 3
 * - Function calling: 0
 * *************************************************************************** */
void DiagnosticPeriodicTimerEvent(void)
{
    if (l_u16MASK0 != 0U)
    {
        /* Some MASK0 bits have been disabled; Check if pending bits are cleared */
#if (_DEBUG_DIAG_OV_UV != FALSE)
        DEBUG_SET_IO_A();
#endif /* (_DEBUG_DIAG_OV_UV != FALSE) */
        uint16_t u16Resolved = ((IO_MLX16_ITC_PEND0_S ^ l_u16MASK0) & l_u16MASK0);
#if (_SUPPORT_APP_USER_MODE != FALSE)
        ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        IO_MLX16_ITC_PEND0_S = l_u16MASK0;
        if (u16Resolved != 0U)
        {
#if (_DEBUG_DIAG_OT != FALSE)
            if ((u16Resolved & B_MLX16_ITC_MASK0_OVT) != 0U)
            {
                DEBUG_SET_IO_A();
                DELAY_US(10U);
                DEBUG_CLR_IO_A();
            }
#endif /* (_DEBUG_DIAG_OT != FALSE) */
            /* Some of Pending #0 bits are cleared; Re-enable Mask #0 bits */
            l_u16MASK0 ^= u16Resolved;                                          /* Clear bits from MASK0 variable */
            IO_MLX16_ITC_MASK0_S |= u16Resolved;                                /* Re-enable MASK0 bits that are "cleared" */
#if defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
            if ( ((u16Resolved & B_MLX16_ITC_MASK0_UV_VDDA) != 0U) && (l_u16DrvProt != 0U) )
            {
                /* Recover Driver protection too */
                IO_PORT_DRV1_PROT = l_u16DrvProt;
                l_u16DrvProt = 0U;
            }
#endif /* defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__) */
#if (FPLL == 32000) || (_SUPPORT_DEGRADED_MODE == FALSE)                        /* MMP200319-1 */
#if defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
            if ( (u16Resolved & B_MLX16_ITC_MASK0_UV_VSM) != 0U)
#else  /* defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__) */
            if ( (u16Resolved & B_MLX16_ITC_MASK0_UV_VS) != 0U)
#endif
            {
#if !defined (__MLX81339__) && !defined (__MLX81350__) && (FPLL == 32000)
                uint16_t u16CpuSpeed4;
#endif /* !defined (__MLX81339__) && !defined (__MLX81350__) && (FPLL == 32000) */

#if (_SUPPORT_DEGRADED_MODE == FALSE)
                g_e8ErrorVoltage = (uint8_t)C_ERR_VOLTAGE_IN_RANGE;
#endif /* (_SUPPORT_DEGRADED_MODE == FALSE) */
#if (FPLL == 32000) && !defined (__MLX81339__) && !defined (__MLX81350__)
#if (_SUPPORT_CPUSPEED4 != FALSE)
                u16CpuSpeed4 = CalibrationParams.u16APP_TRIM39_Speed4;
#else  /* (_SUPPORT_CPUSPEED4 != FALSE) */
                u16CpuSpeed4 = (TrimParams.MS_Trim.u16MS_TRIM8_RCO32M_32M - 0x1000U);
#endif /* (_SUPPORT_CPUSPEED4 != FALSE) */
                if (u16CpuSpeed4 != 0U)
                {
                    RC_Settings_t CpuSpeed;

                    CpuSpeed.u = u16CpuSpeed4;
                    SetSystemSpeed(CpuSpeed, 0U);                               /* Set RCO32 at 32MHz, with reduced flash wait-cycles */
                }
#endif /* (FPLL == 32000) && !defined (__MLX81339__) && !defined (__MLX81350__) */
            }
#endif /* (FPLL == 32000) || (_SUPPORT_DEGRADED_MODE == FALSE) */
#if defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
            if ( ((u16Resolved & B_MLX16_ITC_MASK0_UV_VDDA) != 0U) && ((l_u16Drv1Prot != 0U) || (l_u16Drv3Prot != 0U)) )
            {
                /* Recover Driver protection too */
                IO_PORT_DRV1_PROT = l_u16Drv1Prot;
                IO_PORT_DRV3_PROT = l_u16Drv3Prot;
                l_u16Drv1Prot = 0U;
                l_u16Drv3Prot = 0U;
            }
#endif /* defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */
            if (l_u16MASK0 == 0U)
            {
                g_e8ErrorElectric &= (uint8_t) ~C_ERR_SEMI_PERMANENT;
            }
        }
#if (_SUPPORT_APP_USER_MODE != FALSE)
        EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#if (_DEBUG_DIAG_OV_UV != FALSE)
        DEBUG_CLR_IO_A();
#endif /* (_DEBUG_DIAG_OV_UV != FALSE) */
    }
    if (l_u16MASK1 != 0U)
    {
        /* Some MASK1 bits have been disabled; Check if pending bits are cleared */
        uint16_t u16Resolved = ((IO_MLX16_ITC_PEND1_S ^ l_u16MASK1) & l_u16MASK1);
#if (_SUPPORT_APP_USER_MODE != FALSE)
        ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        IO_MLX16_ITC_PEND1_S = l_u16MASK1;
        if (u16Resolved != 0U)
        {
            /* Some of Pending #1 bits are cleared; Re-enable Mask #1 bits */
            l_u16MASK1 ^= u16Resolved;                                          /* Clear bits from MASK1 variable */
            IO_MLX16_ITC_MASK1_S |= u16Resolved;                                /* Re-enable MASK1 bits that are "cleared" */
        }
#if (_SUPPORT_APP_USER_MODE != FALSE)
        EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
    }
    if (l_u16MASK2 != 0U)
    {
        /* Some MASK2 bits have been disabled; Check if pending bits are cleared */
#if (_DEBUG_DIAG_OV_UV != FALSE)
        DEBUG_SET_IO_A();
#endif /* (_DEBUG_DIAG_OV_UV != FALSE) */
        uint16_t u16Resolved = ((IO_MLX16_ITC_PEND2_S ^ l_u16MASK2) & l_u16MASK2);
#if (_SUPPORT_APP_USER_MODE != FALSE)
        ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        IO_MLX16_ITC_PEND2_S = l_u16MASK2;
        if ( (l_u16MASK2 & B_MLX16_ITC_PEND2_DIAG) != 0U)
        {
            /* Driver Protection activated */
            uint16_t u16CopyDrvProt = IO_PORT_DRV1_PROT;
            uint16_t u16ClrDrvProt = 0U;

#if (defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)) && (_SUPPORT_DEGRADED_MODE == FALSE)
            if ( (u16Resolved & B_MLX16_ITC_MASK2_OV_VS) != 0U)
            {
                g_e8ErrorVoltage = (uint8_t)C_ERR_VOLTAGE_IN_RANGE;
            }
#endif /* (_SUPPORT_DEGRADED_MODE == FALSE) */

#if !defined (__MLX81160__) && (_SUPPORT_DRV_PROT_VDS != FALSE)
            if ( (IO_PORT_DIAG_IN & M_PORT_DIAG_IN_OV_LS_VDS_MEM) != 0U)
            {
                u16ClrDrvProt |= B_PORT_DRV1_PROT_DIS_OV_LS_VDS;
            }
            if ( (IO_PORT_DIAG_IN & M_PORT_DIAG_IN_OV_HS_VDS_MEM) != 0U)
            {
                u16ClrDrvProt |= B_PORT_DRV1_PROT_DIS_OV_HS_VDS;
            }
#endif /* !defined (__MLX81160__) && (_SUPPORT_DRV_PROT_VDS != FALSE) */
#if (_SUPPORT_DRV_PROT_OC != FALSE)
#if defined (__MLX81160__)
            if ( (IO_PORT_DIAG_IN & M_PORT_DIAG_IN_OVC_MEM) != 0U)
#else  /* defined (__MLX81160__) */
            if ( (IO_PORT_DIAG_IN & B_PORT_DIAG_IN_OVC_MEM) != 0U)
#endif /* defined (__MLX81160__) */
            {
                u16ClrDrvProt |= B_PORT_DRV1_PROT_DIS_OC;
            }
#endif /* (_SUPPORT_DRV_PROT_OC != FALSE) */
#if (_SUPPORT_DRV_PROT_UV_VDDA != FALSE)
            if ( (IO_PORT_DIAG_IN & B_PORT_DIAG_IN_UV_VDDA_MEM) != 0U)
            {
                u16ClrDrvProt |= B_PORT_DRV1_PROT_DIS_UV_VDDA;
            }
#endif /* (_SUPPORT_DRV_PROT_UV_VDDA != FALSE) */
#if defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
#if (_SUPPORT_DRV_PROT_UV_VDDAF != FALSE)
            if ( (IO_PORT_DIAG_IN & B_PORT_DIAG_IN_UV_VDDAF_MEM) != 0U)
            {
                u16ClrDrvProt |= B_PORT_DRV1_PROT_DIS_UV_VDDAF;
            }
#endif /* (_SUPPORT_DRV_PROT_UV_VDDAF != FALSE) */
#endif /* defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__) */
#if defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#if (_SUPPORT_DRV_PROT_OV_VDDA != FALSE)
            if ( (IO_PORT_DIAG_IN & B_PORT_DIAG_IN_OV_VDDA_MEM) != 0U)
            {
                u16ClrDrvProt |= B_PORT_DRV1_PROT_DIS_OV_VDDA;
            }
#endif /* (_SUPPORT_DRV_PROT_OV_VDDA != FALSE) */
#if (_SUPPORT_DRV_PROT_OC_VDDA != FALSE)
            if ( (IO_PORT_DIAG_IN & B_PORT_DIAG_IN_OC_VDDA_MEM) != 0U)
            {
                u16ClrDrvProt |= B_PORT_DRV1_PROT_DIS_OC_VDDA;
            }
#endif /* (_SUPPORT_DRV_PROT_OC_VDDA != FALSE) */
#endif /* defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */
#if defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#if (_SUPPORT_DRV_PROT_UV_VBOOST != FALSE)
            if ( (IO_PORT_DIAG_IN & B_PORT_DIAG_IN_UV_BOOST_MEM) != 0U)
            {
                u16ClrDrvProt |= B_PORT_DRV1_PROT_DIS_UV_BOOST
            }
#endif /* (_SUPPORT_DRV_PROT_UV_VBOOST != FALSE) */
#if (_SUPPORT_DRV_PROT_OV_VBOOST != FALSE)
            if ( (IO_PORT_DIAG_IN & B_PORT_DIAG_IN_OV_BOOST_MEM) != 0U)
            {
                u16ClrDrvProt |= B_PORT_DRV1_PROT_DIS_OV_BOOST
            }
#endif /* (_SUPPORT_DRV_PROT_OV_VBOOST != FALSE) */
#endif /*  defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */
#if (_SUPPORT_DRV_PROT_UV_VS != FALSE)
            if ( (IO_PORT_DIAG_IN & B_PORT_DIAG_IN_UV_VS_MEM) != 0U)
            {
                u16ClrDrvProt |= B_PORT_DRV1_PROT_DIS_UV_VS;
            }
#endif /* (_SUPPORT_DRV_PROT_UV_VS != FALSE) */
#if (_SUPPORT_DRV_PROT_OV_VS != FALSE)
            if ( (IO_PORT_DIAG_IN & B_PORT_DIAG_IN_OV_VS_MEM) != 0U)
            {
                u16ClrDrvProt |= B_PORT_DRV1_PROT_DIS_OV_VS;
            }
#endif /* (_SUPPORT_DRV_PROT_OV_VS != FALSE) */
#if (_SUPPORT_DRV_PROT_OT != FALSE)
            if ( (IO_PORT_DIAG_IN & B_PORT_DIAG_IN_OVT_MEM) != 0U)
            {
                u16ClrDrvProt |= B_PORT_DRV1_PROT_DIS_OVT;
            }
#endif /* (_SUPPORT_DRV_PROT_OT != FALSE) */

            IO_PORT_DRV1_PROT = u16CopyDrvProt | u16ClrDrvProt;
            IO_PORT_DRV1_PROT = u16CopyDrvProt;
        }
        if (u16Resolved != 0U)
        {
            /* Some of Pending #2 bits are cleared; Re-enable Mask #2 bits */
            l_u16MASK2 ^= u16Resolved;                                          /* Clear bits from MASK2 variable */
            IO_MLX16_ITC_MASK2_S |= u16Resolved;                                /* Re-enable MASK2 bits that are "cleared" */
        }
#if (_SUPPORT_APP_USER_MODE != FALSE)
        EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#if (_DEBUG_DIAG_OV_UV != FALSE)
        DEBUG_CLR_IO_A();
#endif /* (_DEBUG_DIAG_OV_UV != FALSE) */
    }
#if !defined (__MLX81160__) && (_SUPPORT_DRV_PROT_VDS != FALSE)
    else if ( ((IO_PORT_DIAG_IN & M_PORT_DIAG_IN_OV_LS_VDS_MEM) != 0U) ||       /* MMP200615-1: Re-check VDS Monitors */
              ((IO_PORT_DIAG_IN & M_PORT_DIAG_IN_OV_HS_VDS_MEM) != 0U) )
    {
        l_u16MASK2 = B_MLX16_ITC_PEND2_DIAG;                                    /* VDS MEM is still active; Clear it again */
    }
#endif /* !defined (__MLX81160__) && (_SUPPORT_DRV_PROT_VDS != FALSE) */

#if defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    if (l_u16MASK3 != 0U)
    {
#if (_DEBUG_DIAG_OV_UV != FALSE)
        DEBUG_SET_IO_B();
#endif /* (_DEBUG_DIAG_OV_UV != FALSE) */
        uint16_t u16Resolved = ((IO_MLX16_ITC_PEND3_S ^ l_u16MASK3) & l_u16MASK3);
#if (_SUPPORT_APP_USER_MODE != FALSE)
        ENTER_SECTION(SYSTEM_MODE); /*lint !e534 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
        IO_MLX16_ITC_PEND3_S = l_u16MASK3;
        if (u16Resolved != 0U)
        {
            /* Some of Pending #3 bits are cleared; Re-enable Mask #3 bits */
            l_u16MASK3 ^= u16Resolved;                                          /* Clear bits from MASK3 variable */
            IO_MLX16_ITC_MASK3_S |= u16Resolved;                                /* Re-enable MASK3 bits that are "cleared" */

#if (defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)) && (_SUPPORT_DEGRADED_MODE == FALSE)
            if ( (u16Resolved & B_MLX16_ITC_MASK3_OV_VSM) != 0U)
            {
                g_e8ErrorVoltage = (uint8_t)C_ERR_VOLTAGE_IN_RANGE;
            }
#endif /* (_SUPPORT_DEGRADED_MODE == FALSE) */
        }
#if (_SUPPORT_APP_USER_MODE != FALSE)
        EXIT_SECTION(); /*lint !e438 */
#endif /* (_SUPPORT_APP_USER_MODE != FALSE) */
#if (_DEBUG_DIAG_OV_UV != FALSE)
        DEBUG_CLR_IO_B();
#endif /* (_DEBUG_DIAG_OV_UV != FALSE) */
    }
#endif /* defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */

#if (_SUPPORT_DIAG_RINNEN_ROBUSTNESS != FALSE)
    l_u8OC_Count = p_DecNzU8(l_u8OC_Count);
    l_u8VDS_Count = p_DecNzU8(l_u8VDS_Count);
#endif /* (_SUPPORT_DIAG_RINNEN_ROBUSTNESS != FALSE) */
#if defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
    l_u8VDDAF_Count = p_DecNzU8(l_u8VDDAF_Count);
#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
    l_u8VBOOST_Count = p_DecNzU8(l_u8VBOOST_Count);
#endif
} /* End of DiagnosticPeriodicTimerEvent() */
#endif /* (_SUPPORT_DIAG_IRQCTLR_LEVEL != FALSE) */

/*!*************************************************************************** *
 * ISR_AWD_ATT
 * \brief   Handle Absolute Watch-dog pre-alert Event (Attention)
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details IRQ-Priority: 1
 * *************************************************************************** *
 * - Call Hierarchy: IRQ
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 1 (SetLastError())
 * *************************************************************************** */
__attribute__((interrupt)) void ISR_AWD_ATT(void)
{
#if (_DEBUG_CPU_LOAD != FALSE)
    DEBUG_SET_IO_E();                                                           /* IRQ-Priority: 1 */
#endif /* (_DEBUG_CPU_LOAD != FALSE) */
#if (_SUPPORT_LOG_ERRORS != FALSE)
    SetLastError(C_WRN_AWD_ATT);
#endif /* (_SUPPORT_LOG_ERRORS != FALSE) */
#if (_DEBUG_CPU_LOAD != FALSE)
    DEBUG_CLR_IO_E();                                                           /* IRQ-Priority: 1 */
#endif /* (_DEBUG_CPU_LOAD != FALSE) */
} /* End of ISR_AWD_ATT() */
/* EOF */
