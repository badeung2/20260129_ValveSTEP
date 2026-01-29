#include "AppBuild.h"
#include "main.h"
#include "ActADC.h"
#include "drivelib/AppFunctions.h"
#include "drivelib/ErrorCodes.h"
#include "drivelib/GlobalVars.h"
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
#include "drivelib/MotorDriver.h"
#include "drivelib/MotorStall.h"
#if (_SUPPORT_MOTION_DET != C_MOTION_DET_NONE)
#include "drivelib/MotionDetector.h"
#endif
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
#include "drivelib/SolenoidDriver.h"
#endif
#include "drivelib/PID_Control.h"
#include "camculib/private_mathlib.h"
#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE)
#include "senselib/Triaxis_MLX90363.h"
#elif (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE)
#include "senselib/Triaxis_MLX90367_372.h"
#elif (_SUPPORT_TRIAXIS_MLX90377 != FALSE)
#include "senselib/Triaxis_MLX90377.h"
#elif (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
#include "senselib/Triaxis_MLX9038x.h"
#elif (_SUPPORT_TRIAXIS_MLX90395 != FALSE)
#include "senselib/Triaxis_MLX90395.h"
#elif (_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90425 != FALSE)
#include "senselib/Triaxis_MLX90421_425.h"
#elif (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE)
#include "senselib/Triaxis_MLX90422_426.h"
#elif (_SUPPORT_TRIAXIS_MLX90427 != FALSE)
#include "senselib/Triaxis_MLX90427.h"
#elif (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE)
#include "senselib/InductivePosSensor_MLX90513.h"
#elif (_SUPPORT_PRESSURE_MLX90829 != FALSE)
#include "senselib/Pressure_MLX90829.h"
#endif
#if (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE)
#include "senselib/DualHallLatch_MLX92251.h"
#endif
#if (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE)
#include "senselib/DualHallLatch_MLX92255.h"
#endif
#if (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL == 1U)
#include "senselib/HallLatch.h"
#endif
#if (_SUPPORT_NTC != FALSE)
#include "senselib/NTC.h"
#endif
#if (_SUPPORT_HUMIDITY_HDC302x != FALSE)
#include "senselib/Humidity_TI_HDC302x.h"
#endif
#if (CAN_COMM != FALSE)
#include "commlib/CAN_Communication.h"
#endif
#if (GPIO_COMM != FALSE)
#include "commlib/GPIO_OnOff.h"
#endif
#if (I2C_COMM != FALSE) && (_SUPPORT_I2C != FALSE) && ((I2C_MASTER_PROT != I2C_MASTER_PROT_NONE) || (I2C_SLAVE_PROT != I2C_SLAVE_PROT_NONE))
#include "commlib/I2C_Generic.h"
#endif
#if (LIN_COMM != FALSE)
#include "commlib/LIN_Communication.h"
#endif
#if (PWM_COMM != FALSE)
#include "commlib/PWM_Communication.h"
#endif
#if (SPI_COMM != FALSE)
#include "commlib/SPI_Communication.h"
#elif (_SUPPORT_SPI != FALSE) || (_SUPPORT_3WIRE_SPI != FALSE)
#include "commlib/SPI.h"
#endif
#include <atomic.h>
#include <bl_tools.h>
#include <mathlib.h>
#include <memory_map.h>
#if (LIN_COMM != FALSE)
#include <mls_api.h>
#endif
#include <plib.h>
#include <sys_tools.h>
#if (_DEBUG_NV_WRITE_BACKGROUND != FALSE)
#include <eeprom_drv.h>
#endif
#pragma space dp
#pragma space none
#pragma space nodp
#if (_SUPPORT_CALIBRATION != FALSE)
static uint16_t l_u16CalibTravel;
#endif
#if (_SUPPORT_CALIBRATION != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
static uint16_t l_u16CalibPauseCounter = 0U;
#endif
#if (_SUPPORT_LIN_UV != FALSE)
static uint16_t l_u16LinUVTimeCounter = 0U;
#endif
#if ((_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX90427 != FALSE)) && (_SUPPORT_TRIAXIS_STANDBY == FALSE)
static uint16_t l_u16TriaxisPollCounter = 0U;
#endif
#if (_SUPPORT_NTC != FALSE)
volatile int16_t l_i16NtcTemperature = 0;
#endif
#if (_SUPPORT_AUTONOMOUS_DEMO != FALSE)
uint8_t l_u8AutonomousMode = TRUE;
#endif
#pragma space none
void main_PeriodicTimerEvent(uint16_t u16Period)
{
AppPeriodicTimerEvent(u16Period);
#if (_SUPPORT_CALIBRATION != FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
if (l_u16CalibPauseCounter != 0U)
{
if (l_u16CalibPauseCounter > u16Period)
{
l_u16CalibPauseCounter -= u16Period;
}
else
{
l_u16CalibPauseCounter = 0U;
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) != C_MOTOR_STATUS_STOP)
{
MotorDriverStop( (uint16_t)C_STOP_IMMEDIATE);
}
#endif
}
}
#endif
#if ((_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX90427 != FALSE)) && (_SUPPORT_TRIAXIS_STANDBY == FALSE)
l_u16TriaxisPollCounter += u16Period;
#endif
if (g_u16MotorStartDelay != 0U)
{
if (g_u16MotorStartDelay > u16Period)
{
g_u16MotorStartDelay -= u16Period;
}
else
{
g_u16MotorStartDelay = 0U;
}
}
}
void main_noinit_section_init(void)
{
#if (_SUPPORT_CALIBRATION != FALSE)
l_u16CalibTravel = NV_CALIB_TRAVEL;
g_u16RealTravel = NV_REAL_TRAVEL;
if (g_u16RealTravel == 0U)
{
g_u16RealTravel = l_u16CalibTravel;
}
#endif
}
#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX90427 != FALSE)
static void TriaxisPostInit(void)
{
uint16_t u16TriaxisError = 0U;
uint16_t u16RetryCount = 3U;
do
{
DELAY(C_DELAY_3AXIS);
#if (_SUPPORT_TRIAXIS_MLX90363_XYZ == FALSE)
if ( (u16TriaxisError = Triaxis_SendCmd(CMD_TRIAXIS_ALPHA)) == ERR_TRIAXIS_OK)
#else
if ( (u16TriaxisError = Triaxis_SendCmd(CMD_TRIAXIS_XYZ)) == ERR_TRIAXIS_OK)
#endif
{
break;
}
u16RetryCount--;
} while (u16RetryCount != 0U);
if (u16RetryCount == 0U)
{
g_u16ActualPosition = C_INV_POS;
g_e8ErrorElectric = (uint8_t)C_ERR_SENSOR;
#if (_SUPPORT_LOG_ERRORS != FALSE)
SetLastError(C_ERR_TRIAXIS_FAILS | (C_ERR_EXT | ((u16TriaxisError & 0x0FU) << 8)));
#endif
}
#define C_TRIAXIS_HYST (uint16_t)(((2U * 65536UL) + (C_SHAFT_STEPS_PER_ROTATION / 2U)) / C_SHAFT_STEPS_PER_ROTATION)
u16RetryCount = 3U;
do
{
DELAY(C_DELAY_3AXIS);
if ( (u16TriaxisError = Triaxis_GetAbsPos(FALSE)) == ERR_TRIAXIS_OK)
{
uint16_t u16TriaxisPositionAvg = g_u16TriaxisActualPos;
DELAY(C_DELAY_3AXIS);
if (Triaxis_GetAbsPos(FALSE) == ERR_TRIAXIS_OK)
{
u16TriaxisPositionAvg = g_u16TriaxisActualPos +
((int16_t)(u16TriaxisPositionAvg - g_u16TriaxisActualPos) / 2);
}
if (u16TriaxisPositionAvg > (C_TRIAXIS_APP_END + C_TRIAXIS_HYST) )
{
#if (_SUPPORT_LOG_ERRORS != FALSE)
SetLastError(C_ERR_TRIAXIS_DEADZONE_CCW);
#endif
Adapt_TriaxisAngleOffset(C_TRIAXIS_APP_END - u16TriaxisPositionAvg);
g_u16TriaxisActualPos = C_TRIAXIS_APP_END;
ConvTriaxisPos2ShaftSteps();
g_u16TargetPosition = g_u16ActualPosition -
p_MulU16hi_U16byU16( (u16TriaxisPositionAvg - C_TRIAXIS_APP_END),
C_SHAFT_STEPS_PER_ROTATION);
g_u8PorMovement = TRUE;
}
else if (u16TriaxisPositionAvg < (C_TRIAXIS_APP_BGN - C_TRIAXIS_HYST) )
{
#if (_SUPPORT_LOG_ERRORS != FALSE)
SetLastError(C_ERR_TRIAXIS_DEADZONE_CW);
#endif
Adapt_TriaxisAngleOffset(C_TRIAXIS_APP_BGN - u16TriaxisPositionAvg);
g_u16TriaxisActualPos = C_TRIAXIS_APP_BGN;
ConvTriaxisPos2ShaftSteps();
g_u16TargetPosition = g_u16ActualPosition +
p_MulU16hi_U16byU16( (C_TRIAXIS_APP_BGN - u16TriaxisPositionAvg),
C_SHAFT_STEPS_PER_ROTATION);
g_u8PorMovement = TRUE;
}
else
{
if (u16TriaxisPositionAvg > C_TRIAXIS_APP_END)
{
g_u16TriaxisActualPos = C_TRIAXIS_APP_END;
}
else if (u16TriaxisPositionAvg < C_TRIAXIS_APP_BGN)
{
g_u16TriaxisActualPos = C_TRIAXIS_APP_BGN;
}
else
{
g_u16TriaxisActualPos = u16TriaxisPositionAvg;
}
ConvTriaxisPos2ShaftSteps();
g_u8PorMovement = FALSE;
}
break;
}
u16RetryCount--;
} while (u16RetryCount != 0U);
if (u16RetryCount == 0U)
{
g_u16ActualPosition = C_INV_POS;
g_e8ErrorElectric = (uint8_t)C_ERR_SENSOR;
#if (_SUPPORT_LOG_ERRORS != FALSE)
SetLastError(C_ERR_TRIAXIS_FAILS | (C_ERR_EXT | ((u16TriaxisError & 0x0FU) << 8)));
#endif
}
#if (_SUPPORT_TRIAXIS_STANDBY != FALSE)
if (g_u8PorMovement == FALSE)
{
(void)Triaxis_SendCmd(CMD_TRIAXIS_STANDBY);
}
#endif
}
#endif
static void main_Init(void)
{
AppInit();
#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX90427 != FALSE)
TriaxisPostInit();
#endif
#if (_SUPPORT_HUMIDITY_HDC302x != FALSE)
Humidity_Init();
#endif
}
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
#if (LIN_COMM != FALSE) && (_SUPPORT_EMERGENCY_RUN != FALSE)
static void HandleEmergencyRunMotorRequest(void)
{
#if (_SUPPORT_BUSTIMEOUT != FALSE)
if (g_e8MotorRequest == (uint8_t)C_MOTOR_REQUEST_EMRUN)
{
g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
if (NV_SAFETY_POSITION == 0U)
{
g_u16TargetPosition = C_MIN_POS;
}
else
{
g_u16TargetPosition = C_MAX_POS;
}
if (g_u16ActualPosition != g_u16TargetPosition)
{
g_e8EmergencyRunOcc = (uint8_t)C_SAFETY_RUN_ACTIVE;
g_e8StallDetectorEna = StallDetectorEna();
g_u8StallOcc = FALSE;
if (g_e8MotorStatus != C_MOTOR_STATUS_RUNNING)
{
#if (_SUPPORT_ACT_SPEED_BY_LIN != FALSE)
g_u16TargetMotorSpeedRPM = g_u16LowSpeedRPM;
#else
g_u8MotorCtrlSpeed = (uint8_t)C_MOTOR_SPEED_1;
#endif
}
#if (_SUPPORT_DEGRADED_MODE != FALSE)
if (g_e8DegradeStatus != FALSE)
{
g_e8DegradedMotorRequest = (uint8_t)C_MOTOR_REQUEST_START;
}
#endif
else
{
g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_START;
}
}
#else
if (g_e8MotorStatus != C_MOTOR_STATUS_STOP)
{
g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_STOP;
}
#endif
#if (_SUPPORT_BUSTIMEOUT_SLEEP != FALSE) && (_SUPPORT_LIN_SLEEP != FALSE)
else if (NV_BUSTIMEOUT_SLEEP != 0U)
{
g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_SLEEP;
}
#endif
else
{
}
}
#endif
}
#endif
#endif
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) && (_SUPPORT_SPEED_AUTO != FALSE)
uint8_t HandleAutoSpeedMode(void)
{
uint8_t u8MotorSpeedIdx;
int16_t i16ChipTemperature = Get_ChipTemperature();
uint16_t u16SupplyVoltage = Get_SupplyVoltage();
int16_t i16TemperatureHyst = C_TEMPERATURE_HYS;
uint16_t u16SupplyHyst = 25U;
if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) == C_MOTOR_STATUS_STOP)
{
i16TemperatureHyst = 0;
u16SupplyHyst = 0U;
g_u8MotorStatusSpeed = (uint8_t)C_MOTOR_SPEED_2;
}
u8MotorSpeedIdx = (uint8_t)g_u8MotorStatusSpeed;
if ( (i16ChipTemperature < (C_AUTOSPEED_TEMP_1 - i16TemperatureHyst)) ||
(u16SupplyVoltage < (((C_AUTOSPEED_VOLT_1 * 25U) / 2U) - u16SupplyHyst)) )
{
u8MotorSpeedIdx = (uint8_t)C_MOTOR_SPEED_1;
}
else if ( ((u16SupplyVoltage >= (((C_AUTOSPEED_VOLT_1 * 25U) / 2U) + u16SupplyHyst)) &&
(u16SupplyVoltage <= (((C_AUTOSPEED_VOLT_2 * 25U) / 2U) - u16SupplyHyst))) ||
(((i16ChipTemperature >= (C_AUTOSPEED_TEMP_1 + i16TemperatureHyst)) &&
(i16ChipTemperature <= (C_AUTOSPEED_TEMP_2 - i16TemperatureHyst))) ||
(i16ChipTemperature > (C_AUTOSPEED_TEMP_3 + i16TemperatureHyst))) )
{
u8MotorSpeedIdx = (uint8_t)C_MOTOR_SPEED_2;
}
else if ( (i16ChipTemperature > (C_AUTOSPEED_TEMP_2 + i16TemperatureHyst)) &&
(i16ChipTemperature < (C_AUTOSPEED_TEMP_3 - i16TemperatureHyst)) &&
(u16SupplyVoltage > (((C_AUTOSPEED_VOLT_2 * 25U) / 2U) + u16SupplyHyst)) )
{
#if defined (C_MOTOR_SPEED_4)
u8MotorSpeedIdx = (uint8_t)C_MOTOR_SPEED_4;
#else
u8MotorSpeedIdx = (uint8_t)C_MOTOR_SPEED_3;
#endif
}
else
{
}
return (u8MotorSpeedIdx);
}
#endif
static uint16_t HandleStartMotorRequest(void)
{
uint16_t u16RequestHandled = TRUE;
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT)
uint16_t u16DeltaPosition;
uint8_t u8NewMotorDirectionCCW;
#if (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_ROTOR)
if (g_u16ActualPosition > g_u16TargetPosition)
{
u16DeltaPosition = g_u16ActualPosition - g_u16TargetPosition;
u8NewMotorDirectionCCW = TRUE;
}
else
{
u16DeltaPosition = g_u16TargetPosition - g_u16ActualPosition;
u8NewMotorDirectionCCW = FALSE;
}
if (u16DeltaPosition != 0U)
#elif (_SUPPORT_MOTOR_POSITION == C_MOTOR_POS_SHAFT)
if (g_u16ActualShaftAngle > g_u16TargetShaftAngle)
{
u16DeltaPosition = g_u16ActualShaftAngle - g_u16TargetShaftAngle;
if (u16DeltaPosition > 0x7FFFU)
{
u8NewMotorDirectionCCW = FALSE;
}
else
{
u8NewMotorDirectionCCW = TRUE;
}
}
else
{
u16DeltaPosition = g_u16TargetShaftAngle - g_u16ActualShaftAngle;
if (u16DeltaPosition > 0x7FFFU)
{
u8NewMotorDirectionCCW = TRUE;
}
else
{
u8NewMotorDirectionCCW = FALSE;
}
}
if (u16DeltaPosition > C_SHAFT_TOLERANCE_ANGLE)
#endif
{
#if (_SUPPORT_ACT_SPEED_BY_LIN == FALSE)
#if (LINPROT == LIN22_SIMPLE_PCT)
uint8_t u8MotorSpeedIdx = (uint8_t)(g_u8MotorCtrlSpeed & 0x03U);
#elif (LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52) || (LINPROT == LIN2X_AIRVENT12) || (LINPROT == LIN13_HVACTB)
uint8_t u8MotorSpeedIdx = (uint8_t)(g_u8MotorCtrlSpeed & 0x07U);
#else
uint8_t u8MotorSpeedIdx = C_DEFAULT_MOTOR_SPEED;
#endif
#if (_SUPPORT_SPEED_AUTO != FALSE)
#if (_SUPPORT_FOC_MODE == FOC_MODE_NONE)
if (g_u8MotorCtrlSpeed == (uint8_t)C_MOTOR_SPEED_AUTO)
{
u8MotorSpeedIdx = HandleAutoSpeedMode();
}
g_u8MotorStatusSpeed = u8MotorSpeedIdx;
#endif
#endif
#endif
if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) == C_MOTOR_STATUS_STOP)
{
g_e8MotorDirectionCCW = u8NewMotorDirectionCCW;
#if (_SUPPORT_ACT_SPEED_BY_LIN == FALSE)
MotorDriverStart(u8MotorSpeedIdx);
#else
MotorDriverStart(g_u16TargetMotorSpeedRPM);
#endif
#if (_SUPPORT_TNCTOC != FALSE)
g_e8TNCTOC |= C_TNCTOC_MOTOR;
#endif
g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
}
else if (u8NewMotorDirectionCCW != g_e8MotorDirectionCCW)
{
MotorDriverStop( (uint16_t)C_STOP_RAMPDOWN);
u16RequestHandled = FALSE;
}
else
{
g_u32TargetPosition = ConvShaftSteps2MicroSteps(g_u16TargetPosition);
#if (_SUPPORT_ACT_SPEED_BY_LIN == FALSE)
MotorDriverSpeed(u8MotorSpeedIdx);
#else
MotorDriverSpeed(g_u16TargetMotorSpeedRPM);
#endif
g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
}
}
else if (g_u8MotorHoldingCurrEna != Get_MotorHoldingCurrState() )
{
MotorDriverStop( (uint16_t)C_STOP_IMMEDIATE);
}
else
{
g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
}
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
SolenoidDriverActivate();
g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
#elif (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
if (g_e8MotorStatus == C_MOTOR_STATUS_STOP)
{
g_u8StallOcc = FALSE;
g_u8ChipResetOcc = FALSE;
g_e8MotorDirectionCCW = g_u8NewMotorDirectionCCW;
MotorDriverStart(g_u16TargetMotorSpeedRPM);
#if (_SUPPORT_TNCTOC != FALSE)
g_e8TNCTOC |= C_TNCTOC_MOTOR;
#endif
g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
}
else if (g_u8NewMotorDirectionCCW != g_e8MotorDirectionCCW)
{
MotorDriverStop( (uint16_t)C_STOP_IMMEDIATE);
u16RequestHandled = FALSE;
}
else
{
MotorDriverSpeed(g_u16TargetMotorSpeedRPM);
g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
}
#endif
return (u16RequestHandled);
}
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
#if (_SUPPORT_CALIBRATION != FALSE)
static void HandleCalibrationMotorRequest(void)
{
if (g_e8CalibrationStep == (uint8_t)C_CALIB_START)
{
if (NV_ROTATION_DIRECTION != 0U)
{
g_u16ActualPosition = (l_u16CalibTravel + (2U * C_TRAVEL_OFFSET));
g_u16TargetPosition = 0U;
}
else
{
g_u16ActualPosition = 0U;
g_u16TargetPosition = (l_u16CalibTravel + (2U * C_TRAVEL_OFFSET));
}
g_u16RealTravel = g_u16ActualPosition;
g_e8CalibrationStep = (uint8_t)C_CALIB_SETUP_HI_ENDPOS;
if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) != C_MOTOR_STATUS_STOP)
{
MotorDriverStop( (uint16_t)C_STOP_IMMEDIATE);
l_u16CalibPauseCounter = NV_CALIB_ENDSTOP_TIME * C_PI_TICKS_10MS;
}
}
if ( (g_e8CalibrationStep == (uint8_t)C_CALIB_SETUP_HI_ENDPOS) && (l_u16CalibPauseCounter == 0U) )
{
g_e8MotorDirectionCCW =
(g_u16TargetPosition < g_u16ActualPosition) ? (uint8_t)C_MOTOR_DIR_CLOSING : (uint8_t)C_MOTOR_DIR_OPENING;
MotorDriverPosInit(g_u16ActualPosition);
g_u8MotorCtrlSpeed = (uint8_t)C_DEFAULT_MOTOR_SPEED;
g_e8StallDetectorEna |= (uint8_t)C_STALLDET_CALIB;
AppResetFlags();
MotorDriverStart(C_CALIB_MOTOR_SPEED);
g_e8CalibrationStep = (uint8_t)C_CALIB_CHECK_HI_ENDPOS;
}
else if (g_e8CalibrationStep == (uint8_t)C_CALIB_CHECK_HI_ENDPOS)
{
if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) == C_MOTOR_STATUS_STOP)
{
if (g_u8StallOcc != FALSE)
{
g_e8CalibrationStep = (uint8_t)C_CALIB_SETUP_LO_ENDPOS;
l_u16CalibPauseCounter = (NV_CALIB_ENDSTOP_TIME * C_PI_TICKS_10MS);
}
else
{
g_e8CalibrationStep = (uint8_t)C_CALIB_FAILED_NO_ENDSTOP;
g_e8MotorRequest = g_e8CalibPostMotorRequest;
g_e8CalibPostMotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
g_e8StallDetectorEna &= (uint8_t) ~C_STALLDET_CALIB;
g_u16RealTravel = l_u16CalibTravel;
}
}
}
else if ( (g_e8CalibrationStep == (uint8_t)C_CALIB_SETUP_LO_ENDPOS) && (l_u16CalibPauseCounter == 0) )
{
g_u16ActualPosition = g_u16TargetPosition;
g_u16TargetPosition = g_u16RealTravel;
g_e8MotorDirectionCCW = (g_u16TargetPosition < g_u16ActualPosition) ? C_MOTOR_DIR_CLOSING : C_MOTOR_DIR_OPENING;
g_u16RealTravel = g_u16ActualPosition;
MotorDriverPosInit(g_u16ActualPosition);
g_u8MotorCtrlSpeed = (uint8_t)C_DEFAULT_MOTOR_SPEED;
g_e8StallDetectorEna |= (uint8_t)C_STALLDET_CALIB;
AppResetFlags();
MotorDriverStart(C_CALIB_MOTOR_SPEED);
g_e8CalibrationStep = (uint8_t)C_CALIB_CHECK_LO_ENDPOS;
}
else if (g_e8CalibrationStep == (uint8_t)C_CALIB_CHECK_LO_ENDPOS)
{
if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) == C_MOTOR_STATUS_STOP)
{
if (g_u8StallOcc != FALSE)
{
if (g_u16RealTravel < g_u16ActualPosition)
{
g_u16RealTravel = g_u16ActualPosition;
}
else
{
g_u16RealTravel = g_u16RealTravel - g_u16ActualPosition;
}
g_e8CalibrationStep = C_CALIB_DONE;
g_u16ActualPosition = ((NV_ROTATION_DIRECTION != FALSE) ? g_u16RealTravel : 0U);
g_u16ActualPosition += C_TRAVEL_OFFSET;
MotorDriverPosInit(g_u16ActualPosition);
g_u8StallOcc = FALSE;
g_u8StallTypeComm &= (uint8_t) ~M_STALL_MODE;
g_e8StallDetectorEna &= (uint8_t) ~C_STALLDET_CALIB;
g_u16MotorStartDelay = C_PI_TICKS_250MS;
g_e8MotorRequest = g_e8CalibPostMotorRequest;
#if (_SUPPORT_DEGRADED_MODE != FALSE)
g_e8DegradedMotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
#endif
g_e8CalibPostMotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
}
else
{
g_e8CalibrationStep = (uint8_t)C_CALIB_FAILED_NO_ENDSTOP;
g_e8MotorRequest = g_e8CalibPostMotorRequest;
g_e8CalibPostMotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
g_e8StallDetectorEna &= (uint8_t) ~C_STALLDET_CALIB;
g_u16RealTravel = l_u16CalibTravel;
}
}
}
else
{
}
}
#endif
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
static void HandleCalibrationFactoryRequest(void)
{
if (g_e8CalibrationStep == (uint8_t)C_CALIB_START)
{
if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) != C_MOTOR_STATUS_STOP)
{
MotorDriverStop( (uint16_t)C_STOP_IMMEDIATE);
}
g_u8ChipResetOcc = FALSE;
g_u8StallOcc = FALSE;
g_e8StallDetectorEna = 0;
Triaxis_Calibrate(C_TRIAXIS_CALIB_RESET);
g_e8CalibrationStep = (uint8_t)C_CALIB_SETUP_HI_ENDPOS;
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
g_u16ActualPosition = 0U;
MotorDriverPosInit(g_u16ActualPosition);
g_u16TargetPosition = (24U * Get_MotorMicroStepsPerMechRotation()) / C_MICROSTEP_PER_FULLSTEP;
g_e8MotorDirectionCCW = FALSE;
MotorDriverStart(0U);
#else
l_u16CalibPauseCounter = C_CALIB_PERIOD_LONG;
g_e8MotorDirectionCCW = FALSE;
MotorDriverStart(p_MulU16hi_U16byU16(g_u16LowSpeedRPM, C_CALIB_SPEED));
#endif
}
else if (g_e8CalibrationStep == (uint8_t)C_CALIB_SETUP_HI_ENDPOS)
{
if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) == C_MOTOR_STATUS_STOP)
{
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
if ( (g_u16TargetPosition == g_u16ActualPosition) && (g_u8StallOcc == FALSE) )
#else
if (g_u8StallOcc == FALSE)
#endif
{
if (Triaxis_Calibrate(C_TRIAXIS_CALIB_RESULT) != C_ERR_NONE)
{
g_e8ErrorElectric = (uint8_t)C_ERR_SENSOR;
g_e8CalibrationStep = (uint8_t)C_CALIB_FAILED;
g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
}
else
{
l_u16CalibPauseCounter = NV_CALIB_ENDSTOP_TIME * C_PI_TICKS_10MS;
g_e8CalibPostMotorRequest = (uint8_t)C_CALIB_CHECK_HI_ENDPOS;
g_e8CalibrationStep = (uint8_t)C_CALIB_PAUSE_HI_ENDSTOP;
}
}
else
{
#if (_SUPPORT_LOG_ERRORS != FALSE)
SetLastError(C_ERR_TRIAXIS_CALIB | C_ERR_EXT | 0x100U);
#endif
g_e8ErrorElectric = (uint8_t)C_ERR_SENSOR;
g_e8CalibrationStep = (uint8_t)C_CALIB_FAILED;
g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
}
}
}
else if ( (g_e8CalibrationStep == (uint8_t)C_CALIB_PAUSE_HI_ENDSTOP) && (l_u16CalibPauseCounter == 0U) )
{
if (g_e8CalibPostMotorRequest == (uint8_t)C_CALIB_CHECK_HI_ENDPOS)
{
g_e8CalibrationStep = (uint8_t)C_CALIB_CHECK_HI_ENDPOS;
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
g_u16ActualPosition = 0U;
MotorDriverPosInit(g_u16ActualPosition);
g_u16TargetPosition = (9U * Get_MotorMicroStepsPerMechRotation()) / C_MICROSTEP_PER_FULLSTEP;
g_e8MotorDirectionCCW = FALSE;
MotorDriverStart(0U);
#else
l_u16CalibPauseCounter = C_CALIB_PERIOD_SHORT;
g_e8MotorDirectionCCW = FALSE;
MotorDriverStart(p_MulU16hi_U16byU16(g_u16LowSpeedRPM, C_CALIB_SPEED));
#endif
}
else if (g_e8CalibPostMotorRequest == (uint8_t)C_CALIB_SETUP_LO_ENDPOS)
{
Triaxis_Calibrate(C_TRIAXIS_CALIB_INIT);
g_e8CalibrationStep = (uint8_t)C_CALIB_SETUP_LO_ENDPOS;
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
g_u16TargetPosition = 0U;
g_u16ActualPosition = (24U * Get_MotorMicroStepsPerMechRotation()) / C_MICROSTEP_PER_FULLSTEP;
MotorDriverPosInit(g_u16ActualPosition);
g_e8MotorDirectionCCW = TRUE;
MotorDriverStart(0U);
#else
l_u16CalibPauseCounter = C_CALIB_PERIOD_LONG;
g_e8MotorDirectionCCW = TRUE;
MotorDriverStart(p_MulU16hi_U16byU16(g_u16LowSpeedRPM, C_CALIB_SPEED));
#endif
}
else if (g_e8CalibPostMotorRequest == (uint8_t)C_CALIB_CHECK_LO_ENDPOS)
{
g_e8CalibrationStep = (uint8_t)C_CALIB_CHECK_LO_ENDPOS;
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
g_u16TargetPosition = 0U;
g_u16ActualPosition = (9U * Get_MotorMicroStepsPerMechRotation()) / C_MICROSTEP_PER_FULLSTEP;
MotorDriverPosInit(g_u16ActualPosition);
g_e8MotorDirectionCCW = TRUE;
MotorDriverStart(0U);
#else
l_u16CalibPauseCounter = C_CALIB_PERIOD_SHORT;
g_e8MotorDirectionCCW = TRUE;
MotorDriverStart(p_MulU16hi_U16byU16(g_u16LowSpeedRPM, C_CALIB_SPEED));
#endif
}
else
{
}
g_e8CalibPostMotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
}
else if (g_e8CalibrationStep == (uint8_t)C_CALIB_CHECK_HI_ENDPOS)
{
if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) == C_MOTOR_STATUS_STOP)
{
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
if ( (g_u16TargetPosition == g_u16ActualPosition) && (g_u8StallOcc == FALSE) )
#else
if (g_u8StallOcc == FALSE)
#endif
{
if (Triaxis_Calibrate(C_TRIAXIS_CALIB_RESULT) != C_ERR_NONE)
{
g_e8ErrorElectric = (uint8_t)C_ERR_SENSOR;
g_e8CalibrationStep = (uint8_t)C_CALIB_FAILED;
g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
}
else
{
l_u16CalibPauseCounter = NV_CALIB_ENDSTOP_TIME * C_PI_TICKS_10MS;
g_e8CalibPostMotorRequest = (uint8_t)C_CALIB_SETUP_LO_ENDPOS;
g_e8CalibrationStep = (uint8_t)C_CALIB_PAUSE_HI_ENDSTOP;
}
}
else
{
#if (_SUPPORT_LOG_ERRORS != FALSE)
SetLastError(C_ERR_TRIAXIS_CALIB | C_ERR_EXT | 0x200U);
#endif
g_e8ErrorElectric = (uint8_t)C_ERR_SENSOR;
g_e8CalibrationStep = (uint8_t)C_CALIB_FAILED;
g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
}
}
}
else if (g_e8CalibrationStep == (uint8_t)C_CALIB_SETUP_LO_ENDPOS)
{
if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) == C_MOTOR_STATUS_STOP)
{
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
if ( (g_u16TargetPosition == g_u16ActualPosition) && (g_u8StallOcc == FALSE) )
#else
if (g_u8StallOcc == FALSE)
#endif
{
if (Triaxis_Calibrate(C_TRIAXIS_CALIB_RESULT) != C_ERR_NONE)
{
g_e8ErrorElectric = (uint8_t)C_ERR_SENSOR;
g_e8CalibrationStep = (uint8_t)C_CALIB_FAILED;
g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
}
else
{
l_u16CalibPauseCounter = NV_CALIB_ENDSTOP_TIME * C_PI_TICKS_10MS;
g_e8CalibPostMotorRequest = (uint8_t)C_CALIB_CHECK_LO_ENDPOS;
g_e8CalibrationStep = (uint8_t)C_CALIB_PAUSE_HI_ENDSTOP;
}
}
else
{
#if (_SUPPORT_LOG_ERRORS != FALSE)
SetLastError(C_ERR_TRIAXIS_CALIB | C_ERR_EXT | 0x300U);
#endif
g_e8ErrorElectric = (uint8_t)C_ERR_SENSOR;
g_e8CalibrationStep = (uint8_t)C_CALIB_FAILED;
g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
}
}
}
else if (g_e8CalibrationStep == (uint8_t)C_CALIB_CHECK_LO_ENDPOS)
{
if ( (g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) == C_MOTOR_STATUS_STOP)
{
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE)
if ( (g_u16TargetPosition == g_u16ActualPosition) && (g_u8StallOcc == FALSE) )
#else
if (g_u8StallOcc == FALSE)
#endif
{
if (Triaxis_Calibrate(C_TRIAXIS_CALIB_RESULT) != C_ERR_NONE)
{
g_e8ErrorElectric = (uint8_t)C_ERR_SENSOR;
}
g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
g_e8CalibrationStep = (uint8_t)C_CALIB_NONE;
(void)Triaxis_Calibrate(C_TRIAXIS_CALIB_SAVE);
}
else
{
#if (_SUPPORT_LOG_ERRORS != FALSE)
SetLastError(C_ERR_TRIAXIS_CALIB | C_ERR_EXT | 0x400U);
#endif
g_e8ErrorElectric = (uint8_t)C_ERR_SENSOR;
g_e8CalibrationStep = (uint8_t)C_CALIB_FAILED;
g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
}
}
}
}
#endif
#endif
#if (LIN_COMM != FALSE) && (_SUPPORT_LIN_SLEEP != FALSE)
#if (_SUPPORT_APP_SAVE != FALSE) && (_SUPPORT_NV_EMERGENCY_STORE != FALSE)
void HandleSleepMotorRequest( void) __attribute__((noreturn));
void HandleSleepMotorRequest(void)
#else
static void HandleSleepMotorRequest( void) __attribute__((noreturn));
static void HandleSleepMotorRequest(void)
#endif
{
AppSleep();
__builtin_unreachable();
}
#endif
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
static uint16_t HandleMotorRequest(void)
{
uint16_t u16RequestHandled = TRUE;
#if (LIN_COMM != FALSE) && (_SUPPORT_EMERGENCY_RUN != FALSE)
HandleEmergencyRunMotorRequest();
#endif
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) && (_SUPPORT_POS_INIT != FALSE)
if (g_e8MotorRequest == (uint8_t)C_MOTOR_REQUEST_START_wINIT)
{
MotorDriverPosInit(g_u16ActualPosition);
g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_START;
}
#endif
if (g_e8MotorRequest == (uint8_t)C_MOTOR_REQUEST_NONE)
{
}
else if (g_e8MotorRequest == (uint8_t)C_MOTOR_REQUEST_STOP)
{
#if (_SUPPORT_FAST_STOP == FALSE)
MotorDriverStop( (uint16_t)C_STOP_RAMPDOWN);
#else
MotorDriverStop( (uint16_t)C_STOP_FAST_STOP);
#endif
g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
#if (_SUPPORT_DEGRADED_MODE != FALSE)
g_e8DegradedMotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
#endif
#if (_SUPPORT_CALIBRATION != FALSE)
g_e8CalibPostMotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
if (g_e8CalibrationStep < (uint8_t)C_CALIB_DONE)
{
g_e8CalibrationStep = (uint8_t)C_CALIB_NONE;
g_e8StallDetectorEna &= (uint8_t) ~C_STALLDET_CALIB;
g_u16RealTravel = l_u16CalibTravel;
}
#endif
}
#if (_SUPPORT_MOTOR_POSITION != C_MOTOR_POS_NONE) && (_SUPPORT_POS_INIT != FALSE)
else if (g_e8MotorRequest == (uint8_t)C_MOTOR_REQUEST_INIT)
{
#if TRUE
MotorDriverInit(FALSE);
#else
MotorDriverPosInit(g_u16ActualPosition);
#endif
g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
#if (_SUPPORT_DEGRADED_MODE != FALSE)
g_e8DegradedMotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
#endif
}
#endif
else if (g_e8MotorRequest == (uint8_t)C_MOTOR_REQUEST_START)
{
if (g_u16MotorStartDelay == 0U)
{
u16RequestHandled = HandleStartMotorRequest();
}
}
#if (_SUPPORT_CALIBRATION != FALSE)
else if (g_e8MotorRequest == (uint8_t)C_MOTOR_REQUEST_CALIBRATION)
{
HandleCalibrationMotorRequest();
}
#endif
#if (_SUPPORT_SPEED_CHANGE != FALSE)
#if FALSE
else if ( (g_e8MotorRequest == (uint8_t)C_MOTOR_REQUEST_SPEED_CHANGE) &&
((g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) != C_MOTOR_STATUS_STOP) )
{
#if (_SUPPORT_SPEED_AUTO != FALSE)
#if (LINPROT == LIN22_SIMPLE_PCT)
uint8_t u8MotorSpeedIdx = (uint8_t)(g_u8MotorCtrlSpeed & 0x03U);
#elif (LINPROT == LIN2X_AGS) || (LINPROT == LIN2X_HVAC49) || (LINPROT == LIN2X_HVAC52) || (LINPROT == LIN2X_AIRVENT12)
uint8_t u8MotorSpeedIdx = (uint8_t)(g_u8MotorCtrlSpeed & 0x07U);
#else
#error "ERROR: Speed index not handled"
#endif
if (g_u8MotorCtrlSpeed == (uint8_t)C_MOTOR_SPEED_AUTO)
{
u8MotorSpeedIdx = HandleAutoSpeedMode();
}
g_u8MotorStatusSpeed = u8MotorSpeedIdx;
#else
uint8_t u8MotorSpeedIdx = C_DEFAULT_MOTOR_SPEED;
#endif
MotorDriverSpeed(u8MotorSpeedIdx);
}
#else
else if (g_e8MotorRequest == (uint8_t)C_MOTOR_REQUEST_SPEED_CHANGE)
{
MotorDriverSpeed(g_u16TargetMotorSpeedRPM);
}
#endif
#endif
#if (LIN_COMM != FALSE) && (_SUPPORT_LIN_SLEEP != FALSE)
else if (g_e8MotorRequest == (uint8_t)C_MOTOR_REQUEST_SLEEP)
{
HandleSleepMotorRequest();
}
#endif
else if (g_e8MotorRequest == (uint8_t)C_MOTOR_REQUEST_RESET)
{
MotorDriverStop( (uint16_t)C_STOP_IMMEDIATE);
g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
#if (LIN_COMM != FALSE)
ml_ResetDrv();
g_u8LinInFrameBufState = (uint8_t)C_LIN_IN_FREE;
#endif
MLX16_RESET_SIGNED( (BistResetInfo_t)C_CHIP_STATE_CMD_RESET);
}
else
{
g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
}
return (u16RequestHandled);
}
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
static void HandleSolenoidRequest(void)
{
if (g_e8MotorRequest == (uint8_t)C_MOTOR_REQUEST_NONE)
{
}
else if (g_e8MotorRequest == (uint8_t)C_SOLENOID_REQUEST_DEACTIVATE)
{
SolenoidDriverDeactivate();
g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
#if (_SUPPORT_DEGRADED_MODE != FALSE)
g_e8DegradedMotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
#endif
}
else if ( (g_e8MotorRequest == (uint8_t)C_SOLENOID_REQUEST_ACTIVATE) && (g_u16MotorStartDelay == 0U) )
{
if (g_e8SolenoidStatus == (uint8_t)C_SOLENOID_STATUS_DEACTIVATED)
{
SolenoidDriverActivate();
}
g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
}
#if (_SUPPORT_LIN_SLEEP != FALSE)
else if (g_e8MotorRequest == (uint8_t)C_MOTOR_REQUEST_SLEEP)
{
HandleSleepMotorRequest();
}
#endif
else
{
}
}
#endif
static void AppBackgroundTaskHandler(void)
{
#if (_SUPPORT_CURRENT_TEMP_COMPENSATION != FALSE)
ThresholdControl();
#endif
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_PWM_MODE != SINGLE_COIL_PWM_BIPOLAR)
#if (_SUPPORT_STALLDET_LA != FALSE)
if ( (PID_Control() == 0U) &&
((g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) != C_MOTOR_STATUS_STOP) )
{
g_u8StallTypeComm |= (uint8_t)C_STALL_FOUND_PID_LA;
if ( (g_e8StallDetectorEna & ((uint8_t)C_STALLDET_LA | (uint8_t)C_STALLDET_CALIB)) != 0U)
{
g_u8StallOcc = TRUE;
MotorDriverStop( (uint16_t)C_STOP_EMERGENCY);
}
}
#else
(void)PID_Control();
#endif
#else
PID_Control();
#endif
#endif
AppBackgroundHandler();
}
int main(void)
{
#if (_SUPPORT_MOTOR_POSITION == FALSE) || (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
uint32_t u32ActualMotorSpeedLPFx64 = 0U;
#endif
#if (_SUPPORT_PWM_SYNC != FALSE)
#if (defined (__MLX81332B02__) || defined (__MLX81334__) || defined (__MLX81344B01__) || defined (__MLX81346B01__))
IO_TRIM_VDD = IO_TRIM_VDD | ((1U << 2) << 6);
#elif defined (__MLX81339__) || defined (__MLX81350__)
IO_TRIM_MISC |= B_TRIM_MISC_PWM_SYNC_MODE;
#endif
#endif
main_Init();
#if (_DEBUG_NV_WRITE_BACKGROUND != FALSE)
static uint8_t NV_Test[8];
NV_Test[0] = (uint8_t)0x00U;
NV_Test[1] = (uint8_t)0x11U;
NV_Test[2] = (uint8_t)0x22U;
NV_Test[3] = (uint8_t)0x33U;
NV_Test[4] = (uint8_t)0x44U;
NV_Test[5] = (uint8_t)0x55U;
NV_Test[6] = (uint8_t)0x66U;
NV_Test[7] = (uint8_t)0x77U;
#if (_DEBUG_COMMUT_ISR != FALSE)
DEBUG_SET_IO_A();
#endif
ENTER_SECTION(SYSTEM_MODE);
IO_MLX16_ITC_PEND2_S = B_MLX16_ITC_PEND2_EE_COMPLETE;
EEPROM_WriteWord64_non_blocking( (const uint16_t)0x980U, (const uint16_t *)((void *)NV_Test), 07U);
EXIT_SECTION();
while ( (IO_MLX16_ITC_PEND2_S & B_MLX16_ITC_PEND2_EE_COMPLETE) == 0U)
{
#if (_DEBUG_COMMUT_ISR != FALSE)
DEBUG_TOG_IO_A();
#endif
}
#if (_DEBUG_COMMUT_ISR != FALSE)
DEBUG_CLR_IO_A();
#endif
g_u16TestCnt = 1U;
#endif
#if (_SUPPORT_CHIP_TEST != FALSE) && (_APP_DMA_STRESS_TEST != FALSE)
ChipTestDMA_Init();
#endif
for(;;)
{
#if (_DEBUG_WDACK != FALSE)
DEBUG_SET_IO_A();
#endif
#if FALSE
ENTER_SECTION(ATOMIC_KEEP_MODE);
p_AwdAck();
#if (_SUPPORT_DWD != FALSE)
WDG_conditionalIwdRefresh(C_IWD_DIV, C_IWD_TO);
#endif
EXIT_SECTION();
#else
p_AwdAck();
#if (_SUPPORT_DWD != FALSE)
WDG_conditionalIwdRefresh(C_IWD_DIV, C_IWD_TO);
#endif
#endif
#if (_DEBUG_WDACK != FALSE)
DEBUG_CLR_IO_A();
#endif
#if (_SUPPORT_AUTONOMOUS_DEMO != FALSE)
#define C_DEMO_POS_1 0U
#define C_DEMO_POS_2 12500U
#define C_DEMO_SPEEDMODE 1U
if ( (l_u8AutonomousMode != FALSE) &&
(g_e8MotorRequest == (uint8_t)C_MOTOR_REQUEST_NONE) &&
(g_u16MotorStartDelay == 0U) &&
((g_e8MotorStatus & (uint8_t)C_MOTOR_STATUS_STOP_MASK) == (uint8_t)C_MOTOR_STATUS_STOP) )
{
if ( (g_u16ActualPosition == C_INI_POS) && (g_u16TargetPosition == C_INV_POS) )
{
g_u8ChipResetOcc = FALSE;
g_u16ActualPosition = C_DEMO_POS_1;
g_u16TargetPosition = C_DEMO_POS_2;
}
else if (g_u16TargetPosition < (C_DEMO_POS_2/2U) )
{
g_u16ActualPosition = C_DEMO_POS_1;
g_u16TargetPosition = C_DEMO_POS_2;
}
else
{
g_u16ActualPosition = C_DEMO_POS_2;
g_u16TargetPosition = C_DEMO_POS_1;
}
g_u8StallOcc = FALSE;
g_e8EmergencyRunOcc = (uint8_t)C_SAFETY_RUN_NO;
g_e8MotorCtrlMode = (uint8_t)C_MOTOR_CTRL_NORMAL;
g_e8StallDetectorEna = C_STALLDET_ALL;
g_u8MotorCtrlSpeed = C_DEMO_SPEEDMODE;
g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_START_wINIT;
}
#endif
#if (LIN_COMM != FALSE)
if (g_u8LinInFrameBufState != (uint8_t)C_LIN_IN_FREE)
{
#if (_SUPPORT_AUTONOMOUS_DEMO != FALSE)
l_u8AutonomousMode = FALSE;
#endif
HandleLinInMsg();
}
#endif
#if (_SUPPORT_CHIP_TEST != FALSE) && (_APP_DMA_STRESS_TEST != FALSE)
ChipTestDMA();
#endif
AppDegradedCheck();
#if (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT) || (_SUPPORT_APP_TYPE == C_APP_CONTINUOUS_ACT)
if (HandleMotorRequest() == FALSE)
{
continue;
}
#if (_SUPPORT_TRIAXIS_MLX90363 != FALSE) || (_SUPPORT_TRIAXIS_MLX90427 != FALSE)
#if (_SUPPORT_TRIAXIS_STANDBY == FALSE)
if (l_u16TriaxisPollCounter >= C_TRIAXIS_POLL_RATE)
{
l_u16TriaxisPollCounter = 0U;
if (g_u8LinTriaxisCmd != CMD_TRIAXIS_STANDBY)
{
if (Triaxis_GetAbsPos(TRUE) == ERR_TRIAXIS_OK)
{
ConvTriaxisPos2ShaftSteps();
}
}
}
else
{
}
#else
if (g_u8PorMovement != FALSE)
{
#define C_TRIAXIS_POR_POLL_COUNT 50U
if (l_u16TriaxisPollCounter > C_TRIAXIS_POR_POLL_COUNT)
{
ATOMIC_CODE( (void)Triaxis_GetAbsPos(TRUE); );
l_u16TriaxisPollCounter = 0U;
}
}
#endif
#elif (_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE)
if (Triaxis_Data() == ERR_TRIAXIS_OK)
{
}
#elif (_SUPPORT_TRIAXIS_MLX90421 != FALSE)
g_u16ActualPosition = g_u16TriaxisAbsPos;
#elif (_SUPPORT_APP_TYPE == C_APP_POSITIONING_ACT)
ConvMicroSteps2ShaftSteps();
#endif
#if (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE)
(void)Triaxis_Data();
#endif
#if (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE)
if (DualHallLatch_Direction() != C_DIR_UNKNOWN)
{
uint16_t u16Speed = DualHallLatch_Speed();
if ( (u16Speed != 0x0000U) && (u16Speed != 0xFFFFU) )
{
Set_ActualMotorSpeedRPM(u16Speed);
}
}
#endif
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
{
uint16_t u16CopyActualCommutTimerPeriod = g_u16ActualCommutTimerPeriod;
if ( ((g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK) != C_MOTOR_STATUS_STOP) &&
(u16CopyActualCommutTimerPeriod != 0U) )
{
if ( (l_e8MotorStartupMode & E_MSM_MODE_MASK) == E_MSM_STEPPER)
{
uint16_t u16ActualMotorSpeed = p_DivU16_U32byU16(Get_MicroStepPeriodOneRPM(),
u16CopyActualCommutTimerPeriod);
g_u16ActualMotorSpeedRPM = u16ActualMotorSpeed;
u32ActualMotorSpeedLPFx64 = ((uint32_t)u16ActualMotorSpeed << 16U);
}
else
{
uint16_t u16LPF_Coef;
uint16_t u16ActualMotorSpeed =
p_DivU16_U32byU16( (uint32_t)g_u16ActualMotorSpeedRPMe, g_u16MotorPolePairs);
if (g_u16ActualMotorSpeedRPM > g_u16MinSpeedRPM)
{
u16LPF_Coef = (g_u16ActualMotorSpeedRPM >> 1);
}
else
{
u16LPF_Coef = 1024U;
}
g_u16ActualMotorSpeedRPM =
p_LpfU16_I16byI16(&u32ActualMotorSpeedLPFx64, u16LPF_Coef,
(int16_t)(u16ActualMotorSpeed - g_u16ActualMotorSpeedRPM));
}
}
else
{
#if (_SUPPORT_MOTION_DET == C_MOTION_DET_NONE)
g_u16ActualMotorSpeedRPM = 0U;
#endif
u32ActualMotorSpeedLPFx64 = 0U;
}
}
#elif (_SUPPORT_STALLDET_LA != FALSE)
{
extern uint16_t l_u16CommutTimerPeriod;
uint16_t u16CopyActualCommutTimerPeriod = l_u16CommutTimerPeriod;
if ( ((g_e8MotorStatus & (uint8_t)C_MOTOR_STATUS_STOP_MASK) != (uint8_t)C_MOTOR_STATUS_STOP) &&
(u16CopyActualCommutTimerPeriod != 0U) )
{
g_u16ActualMotorSpeedRPM = p_DivU16_U32byU16(Get_MicroStepPeriodOneRPM(), u16CopyActualCommutTimerPeriod);
}
else
{
g_u16ActualMotorSpeedRPM = 0U;
}
}
#else
if (l_u8MotorStartupMode != (uint8_t)MSM_STOP)
{
g_u16ActualMotorSpeedRPM = g_u16ForcedSpeedRPM;
}
else
{
g_u16ActualMotorSpeedRPM = 0U;
}
#endif
#elif (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
HandleSolenoidRequest();
#endif
#if (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE)
InductivePosSensor_Data();
#endif
#if (_SUPPORT_PRESSURE_MLX90829 != FALSE)
Pressure_Data();
#endif
#if (_SUPPORT_TRIAXIS_MLX90377 != FALSE) && (_SUPPORT_SENSOR_SENT_IO != PIN_FUNC_IO_NONE)
Triaxis_Trigger(u16TriaxisSPC_ID);
#if FALSE
u16TriaxisSPC_ID = ((u16TriaxisSPC_ID + 1U) & 0x03U);
#endif
DELAY_US(C_TRIAXIS_SPC_FRAME_TIME);
(void)Triaxis_Data();
#endif
#if (_SUPPORT_NTC != FALSE)
l_i16NtcTemperature = NTC_Temperature();
#endif
#if (_SUPPORT_HUMIDITY_HDC302x != FALSE)
HumiditySensorHandler();
#endif
AppBackgroundTaskHandler();
}
return 0;
}