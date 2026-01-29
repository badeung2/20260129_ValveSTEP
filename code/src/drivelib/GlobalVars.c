/*!*************************************************************************** *
 * \file        GlobalVars.c
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
 *           -# GlobalInit()
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

#if (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE)
#include "drivelib/NV_UserPage.h"                                               /* Non Volatile Memory User application Pages */
#endif /* (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE) */
#include "drivelib/GlobalVars.h"                                                /* Global Variables support */
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
#include "drivelib/MotorStall.h"                                                /* Motor Stall Detectors */
#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT) */

/*!*************************************************************************** */
/*                           GLOBAL & LOCAL VARIABLES                          */
/* *************************************************************************** */
#pragma space dp                                                                /* __TINY_SECTION__ */
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
uint16_t g_u16MinSpeedRPM;                                                      /*!< Minimum Speed [RPM] (NV_MIN_SPEED) */
uint16_t g_u16StartupSpeedRPM;                                                  /*!< Initial Start-up Speed [RPM] (NV_ALIGNMENT_SPEED or NV_MIN_SPEED) */
uint16_t g_u16LowSpeedRPM;                                                      /*!< Low Speed [RPM] (NV_ACT_SPEED1) */
uint16_t g_u16MaxSpeedRPM;                                                      /*!< Maximum Speed [RPM] (NV_ACT_SPEED3 or NV_ACT_SPEED4) */
#if (LINPROT == LIN2X_AIRVENT12)
uint16_t g_u16TorqueSpeedRPM;                                                   /*!< Torque-mode speed [RPM] (NV_ACT_SPEED4) */
#endif /* (LINPROT == LIN2X_AIRVENT12) */
uint16_t g_u16MotorPolePairs;                                                   /*!< Number of Motor rotor pole-pairs (MMP221101-1) */
uint8_t g_u8StallTypeComm = C_STALL_NOT_FOUND;                                  /*!< (Status) Stall Detection Type, communication */
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
uint8_t g_u8MotorStatusSpeed = (uint8_t)C_MOTOR_SPEED_STOP;                     /*!< (Status) Actual motor-speed */
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
#if (LINPROT == LIN2X_AAB) || (LINPROT == LIN2X_KVA)
uint8_t g_u8ActPosValid = FALSE;                                                /*!< Initial Position Set */
#endif /* (LINPROT == LIN2X_AAB) || (LINPROT == LIN2X_KVA) */
#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT) */
uint8_t g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;                       /*!< Control Motor Request */
volatile uint8_t g_e8ErrorElectric = (uint8_t)C_ERR_NONE;                       /*!< Status-flags electric error */
volatile uint8_t g_e8ErrorVoltage = (uint8_t)C_ERR_VOLTAGE_IN_RANGE;            /*!< Status-flags voltage */
volatile uint8_t g_u8ChipResetOcc = TRUE;                                       /*!< Status-flag indicate chip-reset occurred (POR) */
volatile uint8_t g_u8StallOcc = FALSE;                                          /*!< Status-flag indicate stall occurred */
volatile uint8_t g_e8EmergencyRunOcc = (uint8_t)C_SAFETY_RUN_NO;                /*!< Status-flag indicate Emergency/Safety-run occurred */
volatile uint8_t g_e8ErrorOverTemperature = (uint8_t)C_ERR_OTEMP_NO;            /*!< Status-flag over-temperature */
#if (_SUPPORT_DEGRADED_MODE != FALSE)
uint8_t g_e8DegradedMotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;               /*!< Degraded Motor Request */
#endif /* (_SUPPORT_DEGRADED_MODE != FALSE) */
#if (_SUPPORT_CALIBRATION != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
uint8_t g_e8CalibrationStep = (uint8_t)C_CALIB_NONE;                            /*!< Calibration step */
uint8_t g_e8CalibPostMotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;              /*!< Post calibration Motor Request */
#if (_SUPPORT_HALF_AUTO_CALIB != FALSE)
uint8_t g_u8HalfAutomaticCalibration = (uint8_t)C_CALIB_UNKNOWN;                /*!< Calibration mode */
#endif /* (_SUPPORT_HALF_AUTO_CALIB  != FALSE) */
#endif /* (_SUPPORT_CALIBRATION != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE) */
#pragma space none                                                              /* __TINY_SECTION__ */

#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
#if (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_ROTOR)
uint16_t g_u16ActualPosition;                                                   /*!< (Control/Status) Actual motor-rotor position */
uint16_t g_u16TargetPosition;                                                   /*!< (Control) Target motor-rotor position (invalid) */
#elif (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_SHAFT) && \
      ((_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
       (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || \
       (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE))
uint16_t g_u16ActualShaftAngle;                                                 /*!< (Control/Status) Actual (outer) shaft angle */
uint16_t g_u16TargetShaftAngle;                                                 /*!< (Control) Target (outer) shaft angle */
#endif /* (_SUPPORT_MOTOR_POSITION) */
#if (_SUPPORT_CALIBRATION != FALSE)
uint16_t g_u16RealTravel;                                                       /*!< Number of steps between two end-stops */
#endif /* (_SUPPORT_CALIBRATION != FALSE) */
#if (_SUPPORT_VOLTAGE_CTRL != FALSE)
uint16_t g_u16ActualMotorVoltage;                                               /*!< (Control) Actual motor-voltage (mode) */
uint16_t g_u16TargetMotorVoltage;                                               /*!< (Control) Target motor-voltage (mode) */
#endif /* (_SUPPORT_VOLTAGE_CTRL != FALSE) */
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
uint16_t g_u16ActualMotorSpeedRPM;                                              /*!< Actual motor-speed [RPM] */
uint16_t g_u16TargetMotorSpeedRPM;                                              /*!< Target motor-speed [RPM] */
#endif /* (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT) */
volatile uint8_t g_e8MotorStatus;                                               /*!< Status-flags application mode */
#if (_SUPPORT_DEGRADED_MODE != FALSE)
volatile uint8_t g_e8DegradeStatus;                                             /*!< Status-flags degraded mode */
#endif /*(_SUPPORT_DEGRADED_MODE != FALSE) */
uint8_t g_e8StallDetectorEna;                                                   /*!< Control-flag Stall-detector enabled */
uint8_t g_e8MotorDirectionCCW;                                                  /*!< Control/Status-flag motor rotational direction Counter Clock-wise */
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
uint8_t g_u8MotorHoldingCurrEna;                                                /*!< Control-flag motor Holding-current enabled */
#if (_SUPPORT_ACT_SPEED_BY_LIN == FALSE) || (CAN_COMM != FALSE) || (SPI_COMM != FALSE)
uint8_t g_u8MotorCtrlSpeed;                                                     /*!< (Control) Selected motor-speed */
#endif /* (_SUPPORT_ACT_SPEED_BY_LIN == FALSE) || (CAN_COMM != FALSE) || (SPI_COMM != FALSE) */
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) */
#if (LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52) || (LINPROT == LIN2X_AIRVENT12) || (LINPROT == LIN2X_FAN01) || (LINPROT == LIN2X_AGS) || \
    (I2C_SLAVE_PROT == I2C_SLAVE_PROT_POSITIONING) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_CONTINUOUS) || \
    (CAN_COMM != FALSE)
uint8_t g_e8MotorCtrlMode;                                                      /*!< Control-flags motor mode (from Master) */
#endif /* (LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52) || (LINPROT == LIN2X_AIRVENT12) || (LINPROT == LIN2X_FAN01) || (LINPROT == LIN2X_AGS) || \
        * (I2C_SLAVE_PROT == I2C_SLAVE_PROT_POSITIONING) || (I2C_SLAVE_PROT == I2C_SLAVE_PROT_CONTINUOUS) || (CAN_COMM != FALSE) */
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) && (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM)
volatile uint32_t l_u32ActualPosition;                                          /*!< (Motor-driver) Actual motor-rotor position */
uint32_t g_u32TargetPosition;                                                   /*!< (Motor-driver) Target motor-rotor position */
#endif /* (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) && (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM) */
#elif (_SUPPORT_APP_TYPE == C_APP_RELAY)
#if (_SUPPORT_DUAL_RELAY == FALSE)
volatile uint8_t g_e8RelayStatus;                                               /*!< Status-flags relay mode */
#else  /* (_SUPPORT_DUAL_RELAY == FALSE) */
volatile uint8_t g_e8RelayStatusA;                                              /*!< Status-flags relay A mode */
volatile uint8_t g_e8RelayStatusB;                                              /*!< Status-flags relay B mode */
#endif /* (_SUPPORT_DUAL_RELAY == FALSE) */
volatile uint8_t g_e8DegradeStatus;                                             /*!< Status-flags degraded mode */
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
volatile uint8_t g_e8SolenoidStatus;                                            /*!< Status-flags application mode */
uint8_t g_e8StallDetectorEna;                                                   /*!< Control-flag Stall-detector enabled */
#if (_SUPPORT_DEGRADED_MODE != FALSE)
volatile uint8_t g_e8DegradeStatus;                                             /*!< Status-flags degraded mode */
#endif /*(_SUPPORT_DEGRADED_MODE != FALSE) */
#endif /* (_SUPPORT_APP_TYPE) */

#pragma space nodp                                                              /* __NEAR_SECTION__ */
uint16_t g_u16MotorStartDelay = 0U;                                             /*!< Motor start delay (500us) */
uint16_t g_u16PorPollCounter = 0U;                                              /*!< Power-on/Reset poll-counter */

uint8_t g_u8OverTemperatureCount = 0U;                                          /*!< Number of over-temperature events */
uint8_t g_e8ErrorVoltageComm = (uint8_t)C_ERR_VOLTAGE_IN_RANGE;                 /*!< Status-flags voltage (Communication) */
uint8_t g_u8NewMotorDirectionCCW;                                               /*!< New motor direction rotation (CW/CCW) */
#if (_SUPPORT_MECHANICAL_ERROR != FALSE)
uint8_t g_u8MechError = FALSE;                                                  /*!< No mechanical error */
#endif /* (_SUPPORT_MECHANICAL_ERROR != FALSE) */
#if (_SUPPORT_TNCTOC != FALSE)
volatile uint8_t g_e8TNCTOC = C_TNCTOC_NONE;                                    /*!< TNCTOC done flags */
#endif /* (_SUPPORT_TNCTOC !=  FALSE) */
#if (_SUPPORT_MOTOR_SELFTEST != FALSE)
uint8_t g_u8ForceMotorDriverSelfTest = FALSE;                                   /*!< Force Motor Driver Self-test flag; MMP180917-1 */
#endif /* (_SUPPORT_MOTOR_SELFTEST != FALSE) */
#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || (_SUPPORT_TRIAXIS_MLX90427 != FALSE)
uint8_t g_u8PorMovement = FALSE;                                                /*!< Power-on Movement */
#endif /* (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX90395 != FALSE) || (_SUPPORT_TRIAXIS_MLX90427 != FALSE) */

#if (_SUPPORT_SPEED_CTRL != FALSE) && (_SUPPORT_VOLTAGE_CTRL != FALSE)
uint8_t g_e8ControlType = C_SPEED_CTRL;                                         /*!< Motor Control Type: SPEED or VOLTAGE */
#endif /* (_SUPPORT_SPEED_CTRL != FALSE) && (_SUPPORT_VOLTAGE_CTRL != FALSE) */

#if (_SUPPORT_REWIND != FALSE) || (_SUPPORT_STALL_REVERSE != FALSE)
volatile uint8_t g_u8RewindFlags = 0U;                                          /*!< Rewind flags */
#endif /* (_SUPPORT_REWIND != FALSE) || (_SUPPORT_STALL_REVERSE != FALSE) */

#if (_SUPPORT_SPECIAL_COMM_FIELD != FALSE)
uint8_t g_u8Special[6];                                                         /*!< Special communication field info */
#endif /* (_SUPPORT_SPECIAL_COMM_FIELD != FALSE) */

#if (LINPROT == LIN13_HVACTB)
uint8_t g_u8MotorOperationMode = (uint8_t)C_MOTOR_MODE_NORMAL;                  /*!< Motor operation mode (default: Normal) */
#endif /* (LINPROT == LIN13_HVACTB) */

#if (CAN_COMM != FALSE) && (LIN_COMM != FALSE)
uint8_t g_u8CanConnected = C_COMM_DISCONNECTED;                                 /*!< CAN Communication state */
#endif /* (CAN_COMM != FALSE) && (LIN_COMM != FALSE) */
#if (I2C_COMM != FALSE) && \
    (((LIN_COMM != FALSE) && (I2C_SLAVE_PROT != I2C_SLAVE_PROT_NONE) && (C_I2C_SLAVE_SCK_IO == PIN_FUNC_LIN)) ||\
     (PWM_COMM != FALSE))
uint8_t g_u8I2cConnected = C_COMM_DISCONNECTED;                                 /*!< I2C Communication state */
#endif /* (I2C_COMM != FALSE) && ((LIN_COMM != FALSE) || (PWM_COMM != FALSE)) */
#if (LIN_COMM != FALSE) && ((CAN_COMM != FALSE) || \
    ((I2C_COMM != FALSE) && (I2C_SLAVE_PROT != I2C_SLAVE_PROT_NONE) && (C_I2C_SLAVE_SCK_IO == PIN_FUNC_LIN)) || \
    (PWM_COMM != FALSE) || (SPI_COMM != FALSE) || \
    ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR)))
uint8_t g_u8LinConnected = C_COMM_DISCONNECTED;                                 /*!< LIN Communication state */
#endif /* (LIN_COMM != FALSE) && ((CAN_COMM != FALSE) || (PWM_COMM != FALSE) || (SPI_COMM != FALSE) || ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR))) */
#if (PWM_COMM != FALSE) && ((LIN_COMM != FALSE) || (I2C_COMM != FALSE))
uint8_t g_u8PwmConnected = C_COMM_DISCONNECTED;                                 /*!< PWM Communication state */
#endif /* (PWM_COMM != FALSE) && ((LIN_COMM != FALSE) || (I2C_COMM != FALSE)) */
#if (SPI_COMM != FALSE) && (LIN_COMM != FALSE)
uint8_t g_u8SpiConnected = C_COMM_DISCONNECTED;                                 /*!< SPI Communication state */
#endif /* (SPI_COMM != FALSE) && (LIN_COMM != FALSE) */
#if (LIN_COMM != FALSE) && (_SUPPORT_LIN_BUS_ACTIVITY_CHECK != FALSE)
uint16_t g_u16MLX4_RAM_Dynamic_CRC1 = 0x0000U;                                  /*!< MLX4-RAM Dynamic Frame-IDs CRC (Part 1) */
uint16_t g_u16MLX4_RAM_Dynamic_CRC2 = 0x0000U;                                  /*!< MLX4-RAM Dynamic Frame-IDs CRC (Part 2) */
#endif /* (LIN_COMM != FALSE) && (_SUPPORT_LIN_BUS_ACTIVITY_CHECK != FALSE) */
#if (_SUPPORT_STALL_AUTO_CLEAR != FALSE)
uint16_t g_u16StallPeriod = 0U;                                                 /*!< Stall automatic clear period */
#endif /* (_SUPPORT_STALL_AUTO_CLEAR != FALSE) */

#if (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE)
uint16_t g_u16ActSpeed2_NV;                                                     /*!< Actuator speed #2 [RPM] */
uint16_t g_u16ActSpeed3_NV;                                                     /*!< Actuator speed #3 [RPM] */
uint16_t g_u16MotorConst_NV;                                                    /*!< Motor (BEMF) constant */
uint16_t g_u16StallDetectorDelay_NV;                                            /*!< Stall Detector Delay */
uint16_t g_u16MaxCorrectionRatio_NV;                                            /*!< Maximum Correction Ratio */
uint16_t g_u16StartUpCorrectionRatio_NV;                                        /*!< Start-up Correction Ratio */
uint16_t g_u16HoldingCurrentLevel_NV;                                           /*!< Holding Current Level */
uint16_t g_u16StartUpCurrentLevel_NV;                                           /*!< Start-up Current level */
uint16_t g_u16TargetLoadAngle_NV;                                               /*!< Target Load-Angle */
uint16_t g_u16MotorCoilResistanceTotal_NV;                                      /*!< Motor Coil Resistance (Total) */
uint8_t g_u8EmergencyRunPosEna_NV;                                              /*!< Emergency Run Position Enable flag */
uint8_t g_u8EmergencyRunPos_NV;                                                 /*!< Emergency Run Position */
uint8_t g_u8MotorDirectionCCW_NV;                                               /*!< Rotational Direction Motor */
uint8_t g_u8BusTimeOutSleep_NV;                                                 /*!< Bus Time-out Sleep flag */
#endif /* (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE) */
#pragma space none                                                              /* __NEAR_SECTION__ */

#if (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE)
/*!*************************************************************************** *
 * GlobalInit
 * \brief   Initialise the global variables copied from Non Volatile Memory
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  -
 * *************************************************************************** *
 * \details This function copies application used Non Volatile Memory parameters to global
 *          variables. This allow to use non-blocking Non Volatile Memory Write function, as
 *          the application only read-access the copied Non Volatile Memory via system RAM.
 * *************************************************************************** *
 * - Call Hierarchy: AppInit()
 * - Cyclomatic Complexity: 0/1+1
 * - Nesting: 0/1
 * - Function calling: 0
 * *************************************************************************** */
void GlobalInit(void)
{
    g_u16ActSpeed2_NV = NV_ACT_SPEED2;                                          /*!< Actuator speed #2 [RPM] */
    g_u16ActSpeed3_NV = NV_ACT_SPEED3;                                          /*!< Actuator speed #3 [RPM] */
    g_u16MotorConst_NV = NV_MOTOR_CONSTANT;                                     /*!< Motor (BEMF) constant */
    g_u16StallDetectorDelay_NV = NV_STALL_DETECTOR_DELAY;                       /*!< Stall Detector Delay */
    g_u16MaxCorrectionRatio_NV = NV_MAX_CORR_RATIO;                             /*!< Maximum Correction Ratio */
    g_u16StartUpCorrectionRatio_NV = NV_STARTUP_CORR_RATIO;                     /*!< Start-up Correction Ratio */
    g_u16HoldingCurrentLevel_NV = NV_HOLDING_CURR_LEVEL;                        /*!< Holding Current Level */
    g_u16StartUpCurrentLevel_NV = NV_STARTUP_CURR_MAX;                          /*!< Start-up Current level */
    g_u16TargetLoadAngle_NV = NV_TARGET_LA;                                     /*!< Target Load-Angle */
    g_u16MotorCoilResistanceTotal_NV = NV_MOTOR_COIL_RTOT;                      /*!< Motor Coil Resistance (Total) */
    {
        APP_EOL_t *pEOL = (APP_EOL_t *)ADDR_NV_EOL;
        g_u8EmergencyRunPosEna_NV = pEOL->u1EmergencyRunPosEna;                 /*!< Emergency Run Position Enable flag */
        g_u8EmergencyRunPos_NV = pEOL->u7EmergencyRunPos;                       /*!< Emergency Run Position */
        g_u8MotorDirectionCCW_NV = pEOL->u1MotorDirectionCCW;                   /*!< Rotational Direction Motor */
    }
    g_u8BusTimeOutSleep_NV = NV_BUSTIMEOUT_SLEEP;                               /*!< Bus Time-out Sleep flag */
} /* End of GlobalInit() */
#endif /* (_SUPPORT_NV_NON_BLOCK_WRITE != FALSE) */

/* EOF */
