/*!*************************************************************************** *
 * \file        PID_Control.h
 * \brief       MLX8133x/4x PID Controller handling
 *
 * \note        project MLX8133x/4x
 *
 * \author      Marcel Braat
 *
 * \date        2017-09-08
 *
 * \version     2.0
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
 * *************************************************************************** */

#ifndef DRIVE_LIB_PID_CONTROL_H
#define DRIVE_LIB_PID_CONTROL_H

/*!*************************************************************************** *
 *                           INCLUDES                                          *
 * *************************************************************************** */
#include "AppBuild.h"                                                           /* Application Build */

#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)

#if ((LIN_COMM != FALSE) && (_SUPPORT_MLX_DEBUG_MODE != FALSE))
#include "commlib/LIN_Communication.h"                                          /* LIN Communication support */
#endif /* ((LIN_COMM != FALSE) && (_SUPPORT_MLX_DEBUG_MODE != FALSE)) */

#include "drivelib/NV_UserPage.h"                                               /* Non Volatile Memory User application Pages */
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ)
#include "drivelib/MotorDriver.h"                                               /* Motor driver support */
#include "drivelib/MotorDriverTables.h"                                         /* Wave-form vector tables */
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) */
#if (_SUPPORT_PID_INLINE != FALSE)
#include "camculib/private_mathlib.h"                                           /* Private Math Library support */
#endif /* (_SUPPORT_PID_INLINE != FALSE) */

/*!*************************************************************************** *
 *                            DEFINES                                          *
 * *************************************************************************** */
#define C_GN_PID                12U                                             /*!< PID Coefficient Gain: 2^n */
#define C_PID_ANGLE_ACCURACY    (0U + 0U)                                       /*!< PID Angle accuracy (MMP240524-1) */
#define C_PID_ANGLE_ACCURACY_IV (0U + 5U)                                       /*!< PID Angle accuracy (MMP240524-1) */

#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR)
/*! PID Load Angle parameter structure */
typedef struct
{
    int16_t i16PidLACoefI;                                                      /**< PID Load-Angle I-Coefficient */
    int16_t i16PidLACoefP;                                                      /**< PID Load-Angle P-Coefficient */
    int16_t i16PID_LA_E;                                                        /**< PID Load-Angle Error */
    uint16_t u16PID_LA_I;                                                       /**< PID Load-Angle Integrator */
} PID_LA_Commut_t;
#endif /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR) */

/*! Generic PID parameters structure */
typedef struct
{
    int16_t i16CoefI;                                                           /**< [00] PID I-Coefficient */
    uint32_t u32SumError;                                                       /**< [02] PID Error Sum */
    uint32_t u32SumErrorMax;                                                    /**< [06] PID Error Sum Max */
    int16_t i16CoefP;                                                           /**< [10] PID P-Coefficient */
#if (_SUPPORT_PID_D_COEF != FALSE)
    int16_t i16PrevError;                                                       /**< [12] PID Previous Error */
    int16_t i16CoefD;                                                           /**< [14] PID D-Coefficient */
#endif /* (_SUPPORT_PID_D_COEF != FALSE) */
    uint16_t u16MinOutput;                                                      /**< [16] PID Minimum Output */
    uint32_t u32MaxOutput;                                                      /**< [18] PID Maximum Output */
} PID_PARAMS_t;

#define C_SQRT3_DIV2 Q15(0.86602540378443864676372317075294)                    /*!< SQRT(3)/2 */

/*!*************************************************************************** *
 *                           GLOBAL VARIABLES                                  *
 * *************************************************************************** */
#pragma space dp                                                                /* __TINY_SECTION__ */
#if (_SUPPORT_PID_INLINE != FALSE)
extern uint16_t l_u16MaxSpeedTgtLoadAngle;                                      /*!< Target EE Load-angle at Maximum Speed (ACT_SPEED4) */
#endif /* (_SUPPORT_PID_INLINE != FALSE) */
#pragma space none                                                              /* __TINY_SECTION__ */

#pragma space nodp                                                              /* __NEAR_SECTION__ */
#pragma space none                                                              /* __NEAR_SECTION__ */

/*!*************************************************************************** */
/*                           GLOBAL FUNCTIONS                                  */
/* *************************************************************************** */
extern void PID_SetRunningCurrent(uint16_t u16CurrentLevel);
extern void PID_Init(void);
extern uint16_t PID_Start(uint16_t u16Losses, uint16_t u16Bemf);
#if ((_SUPPORT_FOC_MODE != FOC_MODE_NONE) && ((_SUPPORT_CLOSED_LOOP_STARTUP == FALSE) || \
    (_SUPPORT_TRIAXIS_MLX9038x != FALSE))) || \
    ((_SUPPORT_FOC_MODE == FOC_MODE_NONE) && ((_SUPPORT_HALL_LATCH == FALSE) || (_SUPPORT_NR_OF_HL == 1) || (_SUPPORT_HALL_LATCH_DIAG != FALSE)))
extern void PID_SpeedCompensate(uint16_t u16NewSpeed, uint16_t u16OldSpeed);
#endif /* ((_SUPPORT_FOC_MODE != FOC_MODE_NONE) && ((_SUPPORT_CLOSED_LOOP_STARTUP == FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE))) || \
        * ((_SUPPORT_FOC_MODE == FOC_MODE_NONE) && ((_SUPPORT_HALL_LATCH == FALSE) || (_SUPPORT_NR_OF_HL == 1) || (_SUPPORT_HALL_LATCH_DIAG != FALSE))) */
#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM)
extern uint16_t PID_RampUp(uint16_t u16RampUpPeriod);                           /*!< PID Ramp-up for DC */
extern uint16_t PID_RampDown(uint16_t u16RampDownPeriod);                       /*!< PID Ramp-down for DC */
#endif /* (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) */
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR)
extern void PID_Control(void);
#if ((_SUPPORT_HALL_LATCH == FALSE) && (_SUPPORT_FOC_MODE != FOC_MODE_NONE)) || (_SUPPORT_WINDMILL != FALSE)
extern void PID_Open2Close(void);
#endif /* ((_SUPPORT_HALL_LATCH == FALSE) && (_SUPPORT_FOC_MODE != FOC_MODE_NONE)) || (_SUPPORT_WINDMILL != FALSE) */
#if (_SUPPORT_HALL_LATCH == FALSE)
extern uint16_t PID_LA_Speed_Control(int16_t iActualIV);
#endif /* (_SUPPORT_HALL_LATCH == FALSE) */
#else  /* (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM_BIPOLAR) */
extern void PID_WindmillStart(uint16_t u16ActualMotorSpeed);
extern uint16_t PID_Control(void);
extern void PID_SwitchOpen2Close(void);
#if (_SUPPORT_CDI != FALSE)
extern void PID_SwitchCDI2FOC(void);
#endif /* (_SUPPORT_CDI != FALSE) */
#if (_SUPPORT_PID_INLINE == FALSE)
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ)
extern void PID_FOC_IDIQ(int16_t i16Id, int16_t i16Iq, int16_t i16Sine, int16_t i16Cosine);
#else  /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) */
#if (_SUPPORT_PID_U32 == FALSE)
extern uint16_t PID_LoadAngleToActSpeed(int16_t i16ActLoadAngle);
#else  /* (_SUPPORT_PID_U32 == FALSE) */
extern uint32_t PID_LoadAngleToActSpeed(int16_t i16ActLoadAngle);
#endif /* (_SUPPORT_PID_U32 == FALSE) */
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK)
extern void PID_Ipk_Control(uint16_t u16Ipk);
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK) */
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) */
#endif /* (_SUPPORT_PID_INLINE == FALSE) */
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ)
extern void PID_IDIQ_Start(uint16_t u16Ipk);
#elif ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK)
extern void PID_Ipk_Start(uint16_t u16Ipk);
#endif /* (_SUPPORT_FOC_MODE & FOC_MODE_MASK) */
#if (_SUPPORT_CLOSED_LOOP_STARTUP != FALSE)
#if (_SUPPORT_PID_U32 == FALSE)
extern uint16_t PID_LoadAngleToActSpeed_IV(int16_t i16ActLoadAngle);
#else  /* (_SUPPORT_PID_U32 == FALSE) */
extern uint32_t PID_LoadAngleToActSpeed_IV(int16_t i16ActLoadAngle);
#endif /* (_SUPPORT_PID_U32 == FALSE) */
#endif /* (_SUPPORT_CLOSED_LOOP_STARTUP != FALSE) */
#endif /* (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM_BIPOLAR) */
#else  /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
#if (_SUPPORT_HALL_LATCH == FALSE) || (_SUPPORT_WINDMILL != FALSE)
extern void PID_Open2Close(void);
#endif /* (_SUPPORT_HALL_LATCH == FALSE) || (_SUPPORT_WINDMILL != FALSE) */
extern void PID_Control(void);
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) != FOC_MODE_ID_IQ)                     /* MMP230802-2 */
extern uint16_t VoltageCorrection(void);
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) != FOC_MODE_ID_IQ) */
#if (_SUPPORT_CURRENT_TEMP_COMPENSATION != FALSE)
extern void ThresholdControl(void);
#endif /* (_SUPPORT_CURRENT_TEMP_COMPENSATION != FALSE) */
#if (_SUPPORT_STALLDET_BZC != FALSE)
extern void PID_SwitchStepper2Bemf(void);
#endif /* (_SUPPORT_STALLDET_BZC != FALSE) */

#if ((LIN_COMM != FALSE) && (_SUPPORT_MLX_DEBUG_MODE != FALSE)) || _SUPPORT_STALLDET_S
#pragma space dp
extern uint16_t l_u16PidCtrlRatio;
#pragma space none
#endif /* ((LIN_COMM != FALSE) && (_SUPPORT_MLX_DEBUG_MODE != FALSE)) || _SUPPORT_STALLDET_S */

#if ((LIN_COMM != FALSE) && (_SUPPORT_MLX_DEBUG_MODE != FALSE)) && (_SUPPORT_SPEED_CTRL != FALSE)
/*!*************************************************************************** *
 * StorePID_Info
 * \brief   Store PID info in LIN Diagnostics frame (Melexis)
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details in-line function to clear 'local' variable
 * *************************************************************************** */
static inline void StorePID_Info(void)
{
#if (_DEBUG_ADC != FALSE)
    extern uint16_t g_u16AdcDebugA;
    extern uint16_t g_u16AdcDebugB;

    uint16_t *pDiagResponse = ((uint16_t *)(void *)&g_DiagResponse.u.SF.byD2);

    pDiagResponse[0] = g_u16AdcDebugA;
    pDiagResponse[1] = g_u16AdcDebugB;
#else  /* (_DEBUG_ADC != FALSE) */
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) || ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK)
    extern uint16_t l_u16TargetIpk;
#else  /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) || ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK) */
    extern PID_PARAMS_t sPIDpSE2VA;
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) || ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK) */
    uint16_t *pDiagResponse = ((uint16_t *)(void *)&g_DiagResponse.u.SF.byD2);

    pDiagResponse[0] = l_u16PidCtrlRatio;
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) || ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK)
    pDiagResponse[1] = l_u16TargetIpk;
#else  /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) || ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK) */
    pDiagResponse[1] = (uint16_t)(sPIDpSE2VA.u32SumError >> C_GN_PID);  /*lint !e415 */
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) || ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK) */
#endif /* (_DEBUG_ADC != FALSE) */
} /* End of StorePID_Info() */
#endif /* ((LIN_COMM != FALSE) && (_SUPPORT_MLX_DEBUG_MODE != FALSE)) */

#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) || (_SUPPORT_COPRO != FALSE)
extern uint16_t l_u16MotorRefVoltageADC;                                        /*!< Motor Reference Voltage */
extern uint16_t l_u16MinCorrectionRatio;                                        /*!< PID Minimum correction ratio */
extern uint16_t l_u16MaxCorrectionRatio;                                        /*!< PID Maximum correction ratio */
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) || (_SUPPORT_COPRO != FALSE) */

#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ)                     /* MMP230802-2 */
/*!*************************************************************************** *
 * VoltageCorrection
 * \brief   Compensate Motor PWM Duty Cycle for voltage changes
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details Tuned for MLX-GNU V3.0.31 (MMP181206)
 * Note:    This function is not allowed to be called during Non Volatile Memory Write
 *          (due to Non Volatile Memory reads) (C_APP_POSITIONING_ACT).
 * *************************************************************************** *
 * - Call Hierarchy: ISR_CTIME0_3(), PID_Control()
 * - Cyclomatic Complexity: 5+1
 * - Nesting: 4
 * - Function calling: 2 (Get_RawVmotorF(), Set_CorrectionRatio())
 * *************************************************************************** */
static inline uint16_t VoltageCorrection(void)
{
    register uint16_t u16NewCorrectionRatio = l_u16PidCtrlRatio;
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) != FOC_MODE_ID_IQ)                     /* MMP230802-2 */
    register uint16_t u16MotorVoltageADC = Get_RawVmotorF();
    if ( (u16MotorVoltageADC > 0U) && (l_u16MotorRefVoltageADC > 0U) )
    {
        /* Correct Motor PWM duty cycle instantly based on change of supply voltage */
        u16NewCorrectionRatio = p_MulDivU16_U16byU16byU16(l_u16MotorRefVoltageADC,
                                                          u16NewCorrectionRatio,
                                                          u16MotorVoltageADC);
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT)
        if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) != C_MOTOR_STATUS_STOP)
#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) */
        {
            /* Running-mode */
            if (u16NewCorrectionRatio > l_u16MaxCorrectionRatio)
            {
                /* Overflow */
                u16NewCorrectionRatio = l_u16MaxCorrectionRatio;
            }
            else if (u16NewCorrectionRatio < l_u16MinCorrectionRatio)
            {
                /* Underflow */
                u16NewCorrectionRatio = l_u16MinCorrectionRatio;
            }
            else
            {
                /* Nothing */
            }
        }
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT)
        else if (u16NewCorrectionRatio < NV_MIN_HOLDCORR_RATIO)
        {
            /* Holding-mode: Underflow */
            u16NewCorrectionRatio = NV_MIN_HOLDCORR_RATIO;
        }
        else
        {
            /* Nothing */
        }
#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) */
    }
    else
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) != FOC_MODE_ID_IQ) */
    return (u16NewCorrectionRatio);
} /* End of VoltageCorrection() */
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) */

#if (_SUPPORT_PID_INLINE != FALSE)
#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ)
#pragma space dp                                                                /* __TINY_SECTION__ */
extern uint16_t l_u16TargetIpk;                                                 /*!< Target Ipeak */
#pragma space none                                                              /* __TINY_SECTION__ */
extern PID_PARAMS_t sPID_IdCtrl;                                                /*!< PID parameter structure for Id */
extern PID_PARAMS_t sPID_IqCtrl;                                                /*!< PID parameter structure for Iq */
extern uint16_t l_u16MotorRefVoltageADC;                                        /*!< Motor Reference Voltage */
extern uint16_t l_u16MinCorrectionRatio;                                        /*!< PID Minimum correction ratio */
extern uint16_t l_u16MaxCorrectionRatio;                                        /*!< PID Maximum correction ratio */

/*!*************************************************************************** *
 * PID_FOC_IDIQ
 * \brief   PID for FOC Id / Iq.
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] i16Id: FOC Id
 *          [in] i16Iq: FOC Iq
 *          [in] i16Sine: Sine of  Rotor-angle
 *          [in] i16Cosine: Cosine of Rotor-angle
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy:
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 1 (p_PID_Control())
 * *************************************************************************** */
#if (_DEBUG_MOTOR_CURRENT_FLT != FALSE) && (_DEBUG_MCUR_FOC_PID != FALSE) && (_DEBUG_MCUR_SUB_SAMPLED != FALSE)
static inline void PID_FOC_IDIQ(int16_t i16Id, int16_t i16Iq, int16_t i16Sine, int16_t i16Cosine, uint16_t u16Tmp)
#else  /* (_DEBUG_MOTOR_CURRENT_FLT != FALSE) && (_DEBUG_MCUR_FOC_PID != FALSE) && (_DEBUG_MCUR_SUB_SAMPLED != FALSE) */
static inline void PID_FOC_IDIQ(int16_t i16Id, int16_t i16Iq, int16_t i16Sine, int16_t i16Cosine)
#endif /* (_DEBUG_MOTOR_CURRENT_FLT != FALSE) && (_DEBUG_MCUR_FOC_PID != FALSE) && (_DEBUG_MCUR_SUB_SAMPLED != FALSE) */
{
    /* Target I_d = 0. TODO: when field weakening is required, a target I_d current must be added */
    int16_t i16Vd = p_iPID_Control( (0 - i16Id), (void *)&sPID_IdCtrl);

    /* Vector-Amplitude Control (Motor-PWM Duty-Cycle) */
    int16_t i16Vq = (uint16_t)p_PID_Control( (int16_t)(l_u16TargetIpk - i16Iq), (void *)&sPID_IqCtrl);


    /* Valpha = Vd * cos(theta) - Vq * sin(theta)
     * Vbeta  = Vd * sin(theta) + Vq * cos(theta)
     * (theta = rotor-angle)
     */
    /* Inverse Park Transformation (Part I) */
    int16_t i16Vbeta = p_MulI16_I16byQ15x(i16Vq, i16Cosine);
    int16_t i16Valpha = p_MulI16_I16byQ15x(i16Vd, i16Cosine);
    /* Inverse Park Transformation (Part II) */
    l_i16Vbeta = p_MulI16_I16byQ15x(i16Vd, i16Sine) + i16Vbeta;
    i16Valpha = i16Valpha - p_MulI16_I16byQ15x(i16Vq, i16Sine);
    l_i16Valpha = i16Valpha;
#if (_SUPPORT_FOC_ID_IQ_MODE == FOC_ID_IQ_LUT)

    /* Voltage angle: atan(Valpha/Vbeta) (MMP230803-2) */
#if (_SUPPORT_FAST_ATAN2 != FALSE)
    l_u16VoltageAngle = p_atan2I16(i16Valpha, l_i16Vbeta);                      /* 2^16 = 360 deg or 2pi */
#else  /* (_SUPPORT_FAST_ATAN2 != FALSE) */
    l_u16VoltageAngle = atan2I16(i16Valpha, l_i16Vbeta);                        /* 2^16 = 360 deg or 2pi */
#endif /* (_SUPPORT_FAST_ATAN2 != FALSE) */

    /* Voltage amplitude */
#if (_SUPPORT_BEMF_SINUSOIDAL != FALSE)
    /* Vpk = (Valpha * Sine + Vbeta * Cosine) (MMP230803-2) */
#if (_SUPPORT_SINCOS_TABLE_SZ == 256)
    uint16_t u16Idx = l_u16VoltageAngle / _SUPPORT_SINCOS_TABLE_SZ;
    int16_t *pi16SinCos = (int16_t *) &c_ai16MicroStepVector3PH_SinCos256[u16Idx];
#elif (_SUPPORT_SINCOS_TABLE_SZ == 192)
    uint16_t u16Idx = (uint16_t)p_MulU16hi_U16byU16((uint16_t)l_u16VoltageAngle, _SUPPORT_SINCOS_TABLE_SZ);
    int16_t *pi16SinCos = (int16_t *) &c_ai16MicroStepVector3PH_SinCos192[u16Idx];
#elif (_SUPPORT_SINCOS_TABLE_SZ == 1024)
    int16_t *pi16SinCos = p_SinDirectLutInlined_2_5k(l_u16VoltageAngle);
#else
#error "ERROR: Unsupported table"
#endif
    /* TODO[MMP]: 90 deg */
    uint16_t u16Vpk = p_MulI16_I16bypQ15(l_i16Valpha, pi16SinCos) +
                      p_MulI16_I16bypQ15(l_i16Vbeta, (pi16SinCos + C_COS_OFFSET));
#else  /* (_SUPPORT_BEMF_SINUSOIDAL != FALSE) */
    /* Vpk = SQRT(Valpha^2 + Vbeta^2) */
    uint16_t u16Vpk = p_AproxSqrtU16_I16byI16(l_i16Valpha, l_i16Vbeta);
#endif /* (_SUPPORT_BEMF_SINUSOIDAL != FALSE) */

    uint16_t u16PidCtrlRatio = p_MulDivU16_U16byU16byU16(u16Vpk,
                                                         (PWM_REG_PERIOD << (4 + PWM_PRESCALER_N)),
                                                         Get_RawVmotorF());     /* MMP230802-2 */
#if (_DEBUG_MOTOR_CURRENT_FLT != FALSE) && (_DEBUG_MCUR_FOC_PID != FALSE)
#if (_DEBUG_MCUR_SUB_SAMPLED != FALSE)
    if ( (u16Tmp & C_SUB_SAMPLE_MASK) == 0U)
#endif /* (_DEBUG_MCUR_SUB_SAMPLED != FALSE) */
    {
        uint8_t *pBfr = &g_au8DebugBuf[g_u16DebugBufWrIdx];
        pBfr[0] = (uint8_t)(i16Vd >> 1);                                        /* FOC Vd */
        pBfr[1] = (uint8_t)(i16Vq >> 1);                                        /* FOC Vq */
        pBfr[2] = (uint8_t)(u16Vpk >> 1);                                       /* Vpk */
        g_u16DebugBufWrIdx += 3U;
#if (_DEBUG_MCUR_CYCLIC != FALSE)
        if (g_u16DebugBufWrIdx >= C_DEBUG_BUF_SZ)
        {
            g_u16DebugBufWrIdx = 0U;
        }
#endif /* (_DEBUG_MCUR_CYCLIC != FALSE) */
    }
#endif /* _DEBUG_MOTOR_CURRENT_FLT && (_DEBUG_MCUR_FOC != FALSE) */

    if (u16PidCtrlRatio > l_u16MaxCorrectionRatio)
    {
        /* Overflow */
        u16PidCtrlRatio = l_u16MaxCorrectionRatio;
    }
    else if (u16PidCtrlRatio < l_u16MinCorrectionRatio)
    {
        /* Underflow */
        u16PidCtrlRatio = l_u16MinCorrectionRatio;
    }
    else
    {
        /* Nothing */
    }
    l_u16PidCtrlRatio = u16PidCtrlRatio;
#endif /* (_SUPPORT_FOC_ID_IQ_MODE == FOC_ID_IQ_LUT) */

} /* End of PID_FOC_IDIQ() */
#else  /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) */
#if (_SUPPORT_PID_U32 == FALSE)
/*!*************************************************************************** *
 * PID_LoadAngleToActSpeed
 * \brief   Speed correction
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] i16ActualLA: Actual LA angle in radial (2pi = 2^16)
 * \return  (uint16_t) u16ActualSpeed
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: ADC_ISR()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 1 (p_PID_Control())
 * *************************************************************************** */
static inline uint16_t PID_LoadAngleToActSpeed(int16_t i16ActualLA)
{
    extern PID_PARAMS_t sPIDpLA2AS;                                             /*!< PID parameter structure for Load-Angle to Actual Speed */
    extern uint16_t g_u16ActualMotorSpeedRPM;
#if (_SUPPORT_MLX_DEBUG_MODE != FALSE)
    extern uint16_t l_u16ActSpeedTgtLoadAngle;                                  /*!< Speed dependent Target Load-angle */
#endif /* (_SUPPORT_MLX_DEBUG_MODE != FALSE) */

#if (_SUPPORT_MLX_DEBUG_MODE != FALSE)
    int16_t i16ErrorLA;

    l_u16ActSpeedTgtLoadAngle = p_MulDivU16_U16byU16byU16(l_u16MaxSpeedTgtLoadAngle,
                                                          g_u16ActualMotorSpeedRPM,
                                                          g_u16MaxSpeedRPM);
    i16ErrorLA = (int16_t)(i16ActualLA - l_u16ActSpeedTgtLoadAngle);
#else  /* (_SUPPORT_MLX_DEBUG_MODE != FALSE) */
    uint16_t u16TgtLoadAngle = p_MulDivU16_U16byU16byU16(l_u16MaxSpeedTgtLoadAngle,
                                                         g_u16ActualMotorSpeedRPM,
                                                         g_u16MaxSpeedRPM);
    int16_t i16ErrorLA = (int16_t)(i16ActualLA - u16TgtLoadAngle);
#endif /* (_SUPPORT_MLX_DEBUG_MODE != FALSE) */

    /* Load-Angle to Actual-Speed Control */
#if (C_PID_ANGLE_ACCURACY > 0)
    i16ErrorLA >>= C_PID_ANGLE_ACCURACY;                                        /* Reduced ATAN angle accuracy (MMP240524-1) */
#endif /* (C_PID_ANGLE_ACCURACY > 0) */
    return ( (uint16_t)p_PID_Control(i16ErrorLA, (void *)&sPIDpLA2AS) );
} /* End of PID_LoadAngleToActSpeed() */
#else  /* (_SUPPORT_PID_U32 == FALSE) */
/*!*************************************************************************** *
 * PID_LoadAngleToActSpeed
 * \brief   Speed correction
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] i16ActualLA: Actual LA angle in radial (2pi = 2^16)
 * \return  (uint32_t) u32ActualSpeed
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: ADC_ISR()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 1 (p_PID_Control())
 * *************************************************************************** */
static inline uint32_t PID_LoadAngleToActSpeed(int16_t i16ActualLA)
{
    extern PID_PARAMS_t sPIDpLA2AS;                                             /*!< PID parameter structure for Load-Angle to Actual Speed */
    extern uint16_t g_u16ActualMotorSpeedRPM;
#if (_SUPPORT_MLX_DEBUG_MODE != FALSE)
    extern uint16_t l_u16ActSpeedTgtLoadAngle;                                  /*!< Speed dependent Target Load-angle */
#endif /* (_SUPPORT_MLX_DEBUG_MODE != FALSE) */

#if (_SUPPORT_MLX_DEBUG_MODE != FALSE)
    int16_t i16ErrorLA;

    l_u16ActSpeedTgtLoadAngle = p_MulDivU16_U16byU16byU16(l_u16MaxSpeedTgtLoadAngle,
                                                          g_u16ActualMotorSpeedRPM,
                                                          g_u16MaxSpeedRPM);
    i16ErrorLA = (int16_t)(i16ActualLA - l_u16ActSpeedTgtLoadAngle);
#else  /* (_SUPPORT_MLX_DEBUG_MODE != FALSE) */
    int16_t i16TgtLoadAngle = p_MulDivU16_U16byU16byU16(l_u16MaxSpeedTgtLoadAngle,
                                                        g_u16ActualMotorSpeedRPM,
                                                        g_u16MaxSpeedRPM);
    int16_t i16ErrorLA = (int16_t)(i16ActualLA - l_u16ActSpeedTgtLoadAngle);
#endif /* (_SUPPORT_MLX_DEBUG_MODE != FALSE) */

    /* Load-Angle to Actual-Speed Control */
#if (C_PID_ANGLE_ACCURACY > 0)
    i16ErrorLA >>= C_PID_ANGLE_ACCURACY;                                        /* Reduced ATAN angle accuracy (MMP240524-1) */
#endif /* (C_PID_ANGLE_ACCURACY > 0) */
    return (p_PID_Control(i16ErrorLA, (void *)&sPIDpLA2AS) );
} /* End of PID_LoadAngleToActSpeed() */
#endif /* (_SUPPORT_PID_U32 == FALSE) */
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_ID_IQ) */

#if ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK)
#pragma space dp                                                                /* __TINY_SECTION__ */
extern uint16_t l_u16TargetIpk;                                                 /*!< Target Ipeak */
#pragma space none                                                              /* __TINY_SECTION__ */
/*!*************************************************************************** *
 * PID_Ipk_Control
 * \brief   IPeak control
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16Ipk: Actual Peak-current
 * \return  -
 * *************************************************************************** *
 * \details
 * *************************************************************************** *
 * - Call Hierarchy: ADC_ISR()
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 1 (p_PID_Control())
 * *************************************************************************** */
static inline void PID_Ipk_Control(uint16_t u16Ipk)
{
    extern PID_PARAMS_t sPID_IpkCtrl;                                           /*!< PID I-Peak Control structure */

    int16_t i16ErrorIpk = (int16_t)(l_u16TargetIpk - u16Ipk);

/*#define _SUPPORT_RAMP_LIMITATION TRUE
 #define C_PID_IPK_RAMP_DOWN 10
 #if (_SUPPORT_RAMP_LIMITATION != FALSE)
 *  if ( i16ErrorIpk < -(int16_t)C_PID_IPK_RAMP_DOWN )
 *  {
 *      i16ErrorIpk = -(int16_t)C_PID_IPK_RAMP_DOWN;
 *  }
 #endif */

    /* Vector-Amplitude Control (Motor-PWM Duty-Cycle) */
    l_u16PidCtrlRatio = (uint16_t)p_PID_Control(i16ErrorIpk, (void *)&sPID_IpkCtrl);
} /* End of PID_Ipk_Control() */
#endif /* ((_SUPPORT_FOC_MODE & FOC_MODE_MASK) == FOC_MODE_IB_IPK) */
#endif /* (_SUPPORT_PID_INLINE != FALSE) */

/*!*************************************************************************** *
 * Get_ActCurrRunMax_mA
 * \brief   Get variable l_u16ActCurrRunMax_mA
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_u16ActCurrRunMax_mA
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_ActCurrRunMax_mA(void)
{
    extern uint16_t l_u16ActCurrRunMax_mA;
    return (l_u16ActCurrRunMax_mA);
} /* End of Get_ActCurrRunMax_mA() */

/*!*************************************************************************** *
 * Get_ActCurrRunMax_LSB
 * \brief   Get variable l_u16ActCurrRunMax_LSB
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_u16ActCurrRunMax_LSB
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_ActCurrRunMax_LSB(void)
{
    extern uint16_t l_u16ActCurrRunMax_LSB;
    return (l_u16ActCurrRunMax_LSB);
} /* End of Get_ActCurrRunMax_LSB() */

#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
#if ((LIN_COMM != FALSE) && (_SUPPORT_MLX_DEBUG_MODE != FALSE))
/*!*************************************************************************** *
 * Get_TgtLoadAngle
 * \brief   Get variable l_u16ActSpeedTgtLoadAngle
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_u16ActSpeedTgtLoadAngle
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_TgtLoadAngle(void)
{
    extern uint16_t l_u16ActSpeedTgtLoadAngle;                                  /* Actual Speed Target Load-angle */
    return (l_u16ActSpeedTgtLoadAngle);
} /* End of Get_TgtLoadAngle() */
#endif /* ((LIN_COMM != FALSE) && (_SUPPORT_MLX_DEBUG_MODE != FALSE)) */
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */

/*!*************************************************************************** *
 * PID_CtrlCounter
 * \brief   Update PID periodic timer event
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16Period
 * \return  -
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline void PID_CtrlCounter(uint16_t u16Period)
{
    extern uint16_t l_u16PID_CtrlCounter;
#if (_SUPPORT_CURRENT_TEMP_COMPENSATION != FALSE)
    extern uint16_t l_u16PID_ThrshldCtrlCounter;
#endif /* (_SUPPORT_CURRENT_TEMP_COMPENSATION != FALSE) */
#if (_SUPPORT_AMBIENT_TEMP != FALSE)
    extern uint16_t l_u16SelfHeatingCounter;
#endif /* (_SUPPORT_AMBIENT_TEMP != FALSE) */

    l_u16PID_CtrlCounter += u16Period;
#if (_SUPPORT_CURRENT_TEMP_COMPENSATION != FALSE)
    l_u16PID_ThrshldCtrlCounter += u16Period;                                   /* PID Current/Speed Threshold control */
#endif /* (_SUPPORT_CURRENT_TEMP_COMPENSATION != FALSE) */
#if (_SUPPORT_AMBIENT_TEMP != FALSE)
    l_u16SelfHeatingCounter += u16Period;
#endif /* (_SUPPORT_AMBIENT_TEMP != FALSE) */
} /* End of PID_CtrlCounter() */

#if (LINPROT == LIN13_HVACTB)
/*!*************************************************************************** *
 * Get_MaxCorrectionRatio
 * \brief   Set variable Set_MaxCorrectionRatio
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_u16MaxCorrectionRatio
 * *************************************************************************** *
 * \details in-line function to set 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_MaxCorrectionRatio(void)
{
    extern uint16_t l_u16MaxCorrectionRatio;
    return (l_u16MaxCorrectionRatio);
} /* End of Get_MaxCorrectionRatio() */

/*!*************************************************************************** *
 * Set_MaxCorrectionRatio
 * \brief   Set variable Set_MaxCorrectionRatio
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16Value
 * \return  -
 * *************************************************************************** *
 * \details in-line function to set 'local' variable
 * *************************************************************************** */
static inline void Set_MaxCorrectionRatio(uint16_t u16Value)
{
    extern uint16_t l_u16MaxCorrectionRatio;
    l_u16MaxCorrectionRatio = u16Value;
} /* End of Set_MaxCorrectionRatio() */
#endif /* (LINPROT == LIN13_HVACTB) */

#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT)
/*!*************************************************************************** *
 * Get_HoldingThreshold
 * \brief   Get variable l_u16PidHoldingThreshold
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_u16PidHoldingThreshold
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_HoldingThreshold(void)
{
    extern uint16_t l_u16PidHoldingThreshold;
    return (l_u16PidHoldingThreshold);
} /* End of Get_HoldingThreshold() */
#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) */

#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID) || (_SUPPORT_APP_TYPE == C_APP_RELAY)

#if ((LIN_COMM != FALSE) && (_SUPPORT_MLX_DEBUG_MODE != FALSE))
#include "commlib/LIN_Communication.h"                                          /* LIN Communication support */
#endif /* ((LIN_COMM != FALSE) && (_SUPPORT_MLX_DEBUG_MODE != FALSE)) */

/*!*************************************************************************** *
 *                            DEFINES                                          *
 * *************************************************************************** */
#define C_GN_PID                12U                                             /*!< PID Coefficient Gain: 2^n */

/*! Generic PID parameters structure */
typedef struct
{
    int16_t i16CoefI;                                                           /**< [00] PID I-Coefficient */
    uint32_t u32SumError;                                                       /**< [02] PID Error Sum */
    uint32_t u32SumErrorMax;                                                    /**< [06] PID Error Sum Max */
    int16_t i16CoefP;                                                           /**< [10] PID P-Coefficient */
#if (_SUPPORT_PID_D_COEF != FALSE)
    int16_t i16PrevError;                                                       /**< [12] PID Previous Error */
    int16_t i16CoefD;                                                           /**< [14] PID D-Coefficient */
#endif /* (_SUPPORT_PID_D_COEF != FALSE) */
    uint16_t u16MinOutput;                                                      /**< [16] PID Minimum Output */
    uint32_t u32MaxOutput;                                                      /**< [18] PID Maximum Output */
} PID_PARAMS_t;

/*!*************************************************************************** *
 *                           GLOBAL VARIABLES                                  *
 * *************************************************************************** */
#pragma space dp                                                                /* __TINY_SECTION__ */
#pragma space none                                                              /* __TINY_SECTION__ */

#pragma space nodp                                                              /* __NEAR_SECTION__ */
#pragma space none                                                              /* __NEAR_SECTION__ */

/*!*************************************************************************** */
/*                           GLOBAL FUNCTIONS                                  */
/* *************************************************************************** */
extern void PID_SetRunningCurrent(uint16_t u16CurrentLevel);
extern void PID_Init(void);
extern uint16_t PID_Start(uint16_t u16Losses);
#if (_SUPPORT_APP_TYPE != C_APP_RELAY)
extern uint16_t VoltageCorrection(void);
#endif /* (_SUPPORT_APP_TYPE != C_APP_RELAY) */
extern void PID_Control(void);

#if ((LIN_COMM != FALSE) && (_SUPPORT_MLX_DEBUG_MODE != FALSE))
#pragma space dp
#if (_SUPPORT_DUAL_RELAY == FALSE)
extern uint16_t l_u16PidCtrlRatio;                                              /*!< PID Control Ratio (at ref. voltage) */
#else  /* (_SUPPORT_DUAL_RELAY == FALSE) */
extern uint16_t l_u16PidCtrlRatioA;                                             /*!< PID Control Ratio (at ref. voltage) */
extern uint16_t l_u16PidCtrlRatioB;                                             /*!< PID Control Ratio (at ref. voltage) */
#endif /* (_SUPPORT_DUAL_RELAY == FALSE) */
#pragma space none

/*!*************************************************************************** *
 * StorePID_Info
 * \brief   Store PID info in LIN Diagnostics frame (Melexis)
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details in-line function to clear 'local' variable
 * *************************************************************************** */
static inline void StorePID_Info(void)
{
    extern PID_PARAMS_t sPIDpSE2VA;
    uint16_t *pDiagResponse = ((uint16_t *)(void *)&g_DiagResponse.u.SF.byD2);

    pDiagResponse[0] = l_u16PidCtrlRatio;
    pDiagResponse[1] = (uint16_t)(sPIDpSE2VA.u32SumError >> C_GN_PID);  /*lint !e415 */
} /* End of StorePID_Info() */
#endif /* ((LIN_COMM != FALSE) && (_SUPPORT_MLX_DEBUG_MODE != FALSE)) */

/*!*************************************************************************** *
 * Get_ActCurrRunMax_mA
 * \brief   Get variable l_u16ActCurrRunMax_mA
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_u16ActCurrRunMax_mA
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_ActCurrRunMax_mA(void)
{
    extern uint16_t l_u16ActCurrRunMax_mA;
    return (l_u16ActCurrRunMax_mA);
} /* End of Get_ActCurrRunMax_mA() */

/*!*************************************************************************** *
 * Get_HoldingThreshold
 * \brief   Get variable l_u16PidHoldingThreshold
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_u16PidHoldingThreshold
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_HoldingThreshold(void)
{
    extern uint16_t l_u16PidHoldingThreshold;
    return (l_u16PidHoldingThreshold);
} /* End of Get_HoldingThreshold() */

/*!*************************************************************************** *
 * Get_ActCurrRunMax_LSB
 * \brief   Get variable l_u16ActCurrRunMax_LSB
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_u16ActCurrRunMax_LSB
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_ActCurrRunMax_LSB(void)
{
    extern uint16_t l_u16ActCurrRunMax_LSB;
    return (l_u16ActCurrRunMax_LSB);
} /* End of Get_ActCurrRunMax_LSB() */

/*!*************************************************************************** *
 * PID_CtrlCounter
 * \brief   Update PID periodic timer event
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16Period
 * \return  -
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline void PID_CtrlCounter(uint16_t u16Period)
{
    extern uint16_t l_u16PID_CtrlCounter;
#if (_SUPPORT_CURRENT_TEMP_COMPENSATION != FALSE)
    extern uint16_t l_u16PID_ThrshldCtrlCounter;
#endif /* (_SUPPORT_CURRENT_TEMP_COMPENSATION != FALSE) */
#if (_SUPPORT_AMBIENT_TEMP != FALSE)
    extern uint16_t l_u16SelfHeatingCounter;
#endif /* (_SUPPORT_AMBIENT_TEMP  != FALSE) */

    l_u16PID_CtrlCounter += u16Period;
#if (_SUPPORT_CURRENT_TEMP_COMPENSATION != FALSE)
    l_u16PID_ThrshldCtrlCounter += u16Period;                                   /* PID Current/Speed Threshold control */
#endif /* (_SUPPORT_CURRENT_TEMP_COMPENSATION != FALSE) */
#if (_SUPPORT_AMBIENT_TEMP != FALSE)
    l_u16SelfHeatingCounter += u16Period;
#endif /* (_SUPPORT_AMBIENT_TEMP != FALSE) */
} /* End of PID_CtrlCounter() */
#endif /* (_SUPPORT_APP_TYPE) */

#endif /* DRIVE_LIB_PID_CONTROL_H */

/* EOF */
