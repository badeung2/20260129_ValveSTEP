#include "AppBuild.h"
#include "drivelib/ADC.h"
#include "drivelib/AppFunctions.h"
#include "drivelib/ErrorCodes.h"
#include "drivelib/GlobalVars.h"
#include "drivelib/MotorStall.h"
#include "drivelib/NV_Functions.h"
#include "drivelib/NV_UserPage.h"
#include "drivelib/PID_Control.h"
#include "drivelib/Timer.h"
#include "camculib/private_mathlib.h"
#include "commlib/LIN_AutoAddressing.h"
#include "commlib/LIN_Communication.h"
#if (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL == 1U)
#include "senselib/HallLatch.h"
#endif
#include <mls_types.h>

#pragma space dp
uint8_t g_u8NAD = C_DEFAULT_NAD;
uint8_t g_u8CtrlPID;
uint8_t g_u8StsPID;
static uint8_t l_u8ActDirection = 0U;
#pragma space none
#pragma space nodp
static uint8_t l_e8PositionType = (uint8_t)C_POSTYPE_INIT;
static uint8_t l_u8PrevProgramMode = C_CTRL_PROGRAM_INV;
#pragma space none
static INLINE uint16_t ActPosition(uint16_t u16Pos, uint8_t u8Direction)
{
	if((u8Direction  !=  FALSE)  &&  (u16Pos  <=  C_MAX_POS)){u16Pos = (C_MAX_POS - u16Pos);}
	return(u16Pos);
}

void LIN_2x_Init(void)
{
	STD_LIN_PARAMS_t *pStdLin = (STD_LIN_PARAMS_t *)ADDR_NV_STD_LIN_1;
	APP_EOL_t *pEOL = (APP_EOL_t *)ADDR_NV_EOL;
	if((pStdLin  ->  u8NAD  !=  0x00U)  &&  ((pStdLin  ->  u8NAD & 0x80U)  ==  0x00U)){g_u8NAD = pStdLin  ->  u8NAD;}
	else{SetLastError(C_ERR_INV_NAD);}
	l_u8ActDirection = pEOL  ->  u1MotorDirectionCCW;
	g_u8CtrlPID = pStdLin  ->  u8ControlFrameID;
	(void)ml_AssignFrameToMessageID(MSG_CONTROL, g_u8CtrlPID);
	g_u8StsPID = pStdLin  ->  u8StatusFrameID;
	(void)ml_AssignFrameToMessageID(MSG_STATUS, g_u8StsPID);
}

static inline uint8_t ClearEventFlags(HVAC_CTRL *pCtrl)
{
	if((pCtrl  ->  u4ClearEventFlags  !=  (uint8_t)C_CTRL_CLREVENT_NONE)  &&  (pCtrl  ->  u4ClearEventFlags  !=  (uint8_t)C_CTRL_CLREVENT_INV))
	{
		if((pCtrl  ->  u4ClearEventFlags & (uint8_t)C_CTRL_CLREVENT_RESET)  !=  0U){g_u8ChipResetOcc = FALSE;}
		if((pCtrl  ->  u4ClearEventFlags & (uint8_t)C_CTRL_CLREVENT_STALL)  !=  0U){g_u8StallOcc = FALSE;}
		if((pCtrl  ->  u4ClearEventFlags & (uint8_t)C_CTRL_CLREVENT_EMRUN)  !=  0U)
		{
			if(g_e8EmergencyRunOcc  !=  (uint8_t)C_SAFETY_RUN_NO){g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_STOP;}
			g_e8EmergencyRunOcc = (uint8_t)C_SAFETY_RUN_NO;
		}
	}
	return(g_u8ChipResetOcc | g_u8StallOcc | g_e8EmergencyRunOcc);
}

static inline void ModeChange(HVAC_CTRL *pCtrl)
{
	if(pCtrl  ->  u2StopMode  ==  (uint8_t)C_CTRL_STOPMODE_NORMAL){g_e8MotorCtrlMode = (uint8_t)C_MOTOR_CTRL_NORMAL;}
	else if(pCtrl  ->  u2StopMode  ==  (uint8_t)C_CTRL_STOPMODE_STOP)
	{
		g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_STOP;
		g_e8MotorCtrlMode = (uint8_t)C_MOTOR_CTRL_STOP;
	}
	else{}
}

static inline void SaveProgrammingData(HVAC_CTRL *pCtrl)
{
	APP_EOL_t EOL_New;
	uint8_t u8OrgActDirection = l_u8ActDirection;
	p_CopyU16( (sizeof(APP_EOL_t) / sizeof(uint16_t)),
	(uint16_t *)&EOL_New,
	(const uint16_t *)ADDR_NV_EOL);
	if(pCtrl  ->  u2RotationDirection  ==  (uint8_t)C_CTRL_DIR_CW)
	{
		l_u8ActDirection = FALSE;
		EOL_New.u1MotorDirectionCCW = l_u8ActDirection;
	}
	else if(pCtrl  ->  u2RotationDirection  ==  (uint8_t)C_CTRL_DIR_CCW)
	{
		l_u8ActDirection = TRUE;
		EOL_New.u1MotorDirectionCCW = l_u8ActDirection;
	}
	else{}
	if(pCtrl  ->  u2EmergencyRun  ==  (uint8_t)C_CTRL_EMRUN_ENA){EOL_New.u1EmergencyRunPosEna = TRUE;}
	else if(pCtrl  ->  u2EmergencyRun  ==  (uint8_t)C_CTRL_EMRUN_DIS){EOL_New.u1EmergencyRunPosEna = FALSE;}
	else{}
	if(pCtrl  ->  u2EmergencyEndStop  ==  (uint8_t)C_CTRL_ENRUN_ENDSTOP_HI){EOL_New.u7EmergencyRunPos = 100U;}
	else if(pCtrl  ->  u2EmergencyEndStop  ==  (uint8_t)C_CTRL_ENRUN_ENDSTOP_LO){EOL_New.u7EmergencyRunPos = 0U;}
	else{}
	(void)NV_WriteEOL(&EOL_New);
	g_e8MotorDirectionCCW = (uint8_t)C_MOTOR_DIR_UNKNOWN;
	if((pCtrl  ->  u2Program  ==  (uint8_t)C_CTRL_PROGRAM_DIS)  ||  (pCtrl  ->  u2Program  ==  (uint8_t)C_CTRL_PROGRAM_ENA)){l_u8PrevProgramMode = pCtrl  ->  u2Program;}
	if(u8OrgActDirection  !=  l_u8ActDirection)
	{
		g_u16ActualPosition = ActPosition(g_u16ActualPosition, 1);
		g_u16TargetPosition = ActPosition(g_u16TargetPosition, 1);
	}
}

static inline void SetupStallDetector(HVAC_CTRL *pCtrl)
{
	if(g_e8EmergencyRunOcc  ==  (uint8_t)C_SAFETY_RUN_NO)
	{
		if(pCtrl  ->  u2StallDetector  ==  (uint8_t)C_CTRL_STALLDET_DIS){g_e8StallDetectorEna = C_STALLDET_NONE;}
		else if(pCtrl  ->  u2StallDetector  ==  (uint8_t)C_CTRL_STALLDET_ENA){g_e8StallDetectorEna = StallDetectorEna();}
		else{}
	}
}

static inline void SetSpeedMode(HVAC_CTRL *pCtrl)
{
	if((pCtrl  ->  u4Speed  >=  (uint8_t)C_CTRL_SPEED_1)  &&  (pCtrl  ->  u4Speed  <=  (uint8_t)C_CTRL_SPEED_AUTO))
	{
		if(g_u8MotorCtrlSpeed  !=  pCtrl  ->  u4Speed)
		{
			if(g_e8MotorRequest  !=  (uint8_t)C_MOTOR_REQUEST_STOP){g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_SPEED_CHANGE;}
			g_u8MotorCtrlSpeed = pCtrl  ->  u4Speed;
		}
	}
	else{g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_STOP;}
}

static inline void SetPosition(HVAC_CTRL *pCtrl)
{
	if(pCtrl  ->  u2PositionType  ==  (uint8_t)C_CTRL_POSITION_TARGET)
	{
		uint16_t u16Value = (((uint16_t)pCtrl  ->  u8TargetPositionMSB)  <<  8) |
		((uint16_t)pCtrl  ->  u8TargetPositionLSB);
		if(u16Value  !=  C_INV_POS)
		{
			g_u16TargetPosition = ActPosition(u16Value, l_u8ActDirection);
			l_e8PositionType = (uint8_t)C_POSTYPE_TARGET;
			if(g_e8MotorRequest  !=  (uint8_t)C_MOTOR_REQUEST_STOP)
			{
				if(g_e8DegradeStatus  !=  FALSE){g_e8DegradedMotorRequest = (uint8_t)C_MOTOR_REQUEST_START;}
				else{g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_START;}
			}
		}
	}
	else if((pCtrl  ->  u2PositionType  ==  (uint8_t)C_CTRL_POSITION_INITIAL)  &&  ((g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK)  ==  C_MOTOR_STATUS_STOP))
	{
		uint16_t u16Value = (((uint16_t)pCtrl  ->  u8StartPositionMSB)  <<  8) |
		((uint16_t)pCtrl  ->  u8StartPositionLSB);
		if(u16Value  !=  C_INV_POS)
		{
			g_u16ActualPosition = g_u16TargetPosition = ActPosition(u16Value, l_u8ActDirection);
			l_e8PositionType = (uint8_t)C_POSTYPE_INIT;
			g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_INIT;
			if(g_e8DegradeStatus  !=  FALSE){g_e8DegradedMotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;}
		}
	}
	else{}
}

static inline void SetHoldingCurrent(HVAC_CTRL *pCtrl)
{
	if((pCtrl  ->  u2HoldingCurrent  ==  (uint8_t)C_CTRL_MHOLDCUR_DIS)  ||  (pCtrl  ->  u2HoldingCurrent  ==  (uint8_t)C_CTRL_MHOLDCUR_ENA))
	{
		uint8_t u8HoldingCurrEna = (pCtrl  ->  u2HoldingCurrent  ==  (uint8_t)C_CTRL_MHOLDCUR_ENA) ? TRUE : FALSE;
		if(g_u8MotorHoldingCurrEna  !=  u8HoldingCurrEna)
		{
			if((g_e8MotorRequest  ==  (uint8_t)C_MOTOR_REQUEST_NONE)  &&  ((g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK)  ==  C_MOTOR_STATUS_STOP)){g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_STOP;}
			g_u8MotorHoldingCurrEna = u8HoldingCurrEna;
		}
	}
}

void HandleActCtrl(void)
{
	{
		HVAC_CTRL *pCtrl = (HVAC_CTRL *)&(g_LinCmdFrameBuffer.Ctrl);
		if(((pCtrl  ->  u8NAD  ==  g_u8NAD)  ||  (pCtrl  ->  u8NAD  ==  (uint8_t)C_BROADCAST_NAD))  &&  (g_u8LinAAMode  ==  (uint8_t)C_SNPD_SUBFUNC_INACTIVE))
		{
			uint8_t u8EventMode = ClearEventFlags(pCtrl);
			if(u8EventMode  ==  FALSE){ModeChange(pCtrl);}
			if((pCtrl  ->  u2Program  ==  (uint8_t)C_CTRL_PROGRAM_ENA)  &&  (g_e8MotorCtrlMode  ==  (uint8_t)C_MOTOR_CTRL_STOP)  &&  (u8EventMode  ==  FALSE)  &&  (l_u8PrevProgramMode  ==  (uint8_t)C_CTRL_PROGRAM_DIS)){SaveProgrammingData(pCtrl);}
			SetupStallDetector(pCtrl);
			SetSpeedMode(pCtrl);
			if(u8EventMode  ==  FALSE){SetPosition(pCtrl);}
			SetHoldingCurrent(pCtrl);
		}
	}
}

static inline void GetErrorEvents(volatile HVAC_STATUS *pStatus)
{
	if(g_u8ErrorCommunication  !=  FALSE){pStatus  ->  u1ResponseError = TRUE;}
	else{pStatus  ->  u1ResponseError = FALSE;}
	g_u8ErrorCommunication = FALSE;
	pStatus  ->  u1Reserved1 = 1U;
	if(g_e8ErrorOverTemperature  !=  (uint8_t)C_ERR_OTEMP_NO){pStatus  ->  u2OverTemperature = (uint8_t)C_STATUS_OTEMP_YES;}
	else{pStatus  ->  u2OverTemperature = (uint8_t)C_STATUS_OTEMP_NO;}
	if(g_e8ErrorElectric  ==  (uint8_t)C_ERR_NONE){pStatus  ->  u2ElectricDefect = C_STATUS_ELECDEFECT_NO;}
	else if((g_e8ErrorElectric & (uint8_t)C_ERR_PERMANENT)  !=  0U){pStatus  ->  u2ElectricDefect = C_STATUS_ELECDEFECT_PERM;}
	else
	{
		pStatus  ->  u2ElectricDefect = C_STATUS_ELECDEFECT_YES;
		if((g_e8ErrorElectric & (uint8_t)C_ERR_SEMI_PERMANENT)  ==  0U){g_e8ErrorElectric = (uint8_t)C_ERR_NONE;}
	}
	pStatus  ->  u2VoltageError = (uint8_t)(g_e8ErrorVoltage & 0x03);
	if((pStatus  ->  u2VoltageError  ==  0U)  &&  (g_e8ErrorVoltageComm  !=  0U)){pStatus  ->  u2VoltageError = (uint8_t)(g_e8ErrorVoltageComm & 0x03U);}
	g_e8ErrorVoltageComm = g_e8ErrorVoltage;
	if(g_e8EmergencyRunOcc  !=  (uint8_t)C_SAFETY_RUN_NO){pStatus  ->  u2EmergencyOccurred = (uint8_t)C_STATUS_EMRUNOCC_YES;}
	else{pStatus  ->  u2EmergencyOccurred = (uint8_t)C_STATUS_EMRUNOCC_NO;}
	if(g_e8StallDetectorEna  !=  (uint8_t)C_STALLDET_NONE){pStatus  ->  u2StallDetector = (uint8_t)C_STATUS_STALLDET_ENA;}
	else{pStatus  ->  u2StallDetector = (uint8_t)C_STATUS_STALLDET_DIS;}
	if(g_u8StallOcc  !=  FALSE){pStatus  ->  u2StallOccurred = (uint8_t)C_STATUS_STALLOCC_YES;}
	else{pStatus  ->  u2StallOccurred = (uint8_t)C_STATUS_STALLOCC_NO;}
	if(g_u8ChipResetOcc  !=  FALSE){pStatus  ->  u2Reset = (uint8_t)C_STATUS_RESETOCC_YES;}
	else{pStatus  ->  u2Reset = (uint8_t)C_STATUS_RESETOCC_NO;}
}

static inline void GetProgrammingData(volatile HVAC_STATUS *pStatus)
{
	APP_EOL_t *pEOL = (APP_EOL_t *)ADDR_NV_EOL;
	if(pEOL  ->  u1EmergencyRunPosEna  !=  0U){pStatus  ->  u2EmergencyRun = (uint8_t)C_STATUS_EMRUN_ENA;}
	else{pStatus  ->  u2EmergencyRun = (uint8_t)C_STATUS_EMRUN_DIS;}
	if(pEOL  ->  u7EmergencyRunPos  !=  0U){pStatus  ->  u2EmergencyRunEndStop = (uint8_t)C_STATUS_EMRUN_ENDPOS_HI;}
	else{pStatus  ->  u2EmergencyRunEndStop = (uint8_t)C_STATUS_EMRUN_ENDPOS_LO;}
	if(pEOL  ->  u1MotorDirectionCCW  !=  0U){pStatus  ->  u2RotationDirection = (uint8_t)C_STATUS_DIRECTION_CCW;}
	else{pStatus  ->  u2RotationDirection = (uint8_t)C_STATUS_DIRECTION_CW;}
}

void HandleActStatus(void)
{
	if(g_u8LinAAMode  !=  (uint8_t)C_SNPD_SUBFUNC_INACTIVE){(void)ml_DiscardFrame();}
	else
	{
		volatile HVAC_STATUS *pStatus = (HVAC_STATUS *)((void *)ML_DATA_LIN_FRAME_DATA_BUFFER);
		uint16_t i;
		for(i = 0U; i < sizeof(HVAC_STATUS) / sizeof(uint16_t); i++){((uint16_t *)pStatus)[i] = 0xFFFFU;}
		GetErrorEvents(pStatus);
		if(g_u8MotorHoldingCurrEna  !=  FALSE){pStatus  ->  u2HoldingCurrent = (uint8_t)C_STATUS_MHOLDCUR_ENA;}
		else{pStatus  ->  u2HoldingCurrent = (uint8_t)C_STATUS_MHOLDCUR_DIS;}
		if(l_e8PositionType  ==  (uint8_t)C_POSTYPE_INIT){pStatus  ->  u2PositionTypeStatus = (uint8_t)C_STATUS_POSITION_INIT;}
		else{pStatus  ->  u2PositionTypeStatus = (uint8_t)C_STATUS_POSITION_ACTUAL;}
		if((g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK)  !=  C_MOTOR_STATUS_STOP){pStatus  ->  u4SpeedStatus = g_u8MotorCtrlSpeed;}
		else{pStatus  ->  u4SpeedStatus = 0U;}
		{
			uint16_t u16CopyPosition = ActPosition(g_u16ActualPosition, l_u8ActDirection);
			pStatus  ->  u8ActualPositionLSB = (u16CopyPosition & 0xFFU);
			pStatus  ->  u8ActualPositionMSB = (u16CopyPosition  >>  8);
		}
		if(g_e8MotorDirectionCCW  ==  (uint8_t)C_MOTOR_DIR_UNKNOWN){pStatus  ->  u2ActualRotationalDir = (uint8_t)C_STATUS_ACT_DIR_UNKNOWN;}
		else if(((g_e8MotorDirectionCCW & 1U) ^ l_u8ActDirection)  !=  0U){pStatus  ->  u2ActualRotationalDir = (uint8_t)C_STATUS_ACT_DIR_CLOSING;}
		else{pStatus  ->  u2ActualRotationalDir = (uint8_t)C_STATUS_ACT_DIR_OPENING;}
		pStatus  ->  u2SelfHoldingTorque = (uint8_t)C_STATUS_HOLDING_TORQUE_INV;
		{
			pStatus  ->  u2SpecialFunctionActive = (uint8_t)C_STATUS_SFUNC_ACTIVE_NO;
		}
#if (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL == 1U)
		if(g_u16IoState  !=  0U){pStatus  ->  u2Reserved = 1U;}
		else{pStatus  ->  u2Reserved = 0U;}
#elif (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
		if((l_e8MotorStartupMode & E_MSM_MODE_MASK)  ==  E_MSM_STEPPER){pStatus  ->  u2Reserved = 0U;}
		else{pStatus  ->  u2Reserved = 1U;}
#else
		pStatus  ->  u2Reserved = 3U;
#endif
		{
			pStatus  ->  u8NAD = g_u8NAD;
		}
		GetProgrammingData(pStatus);
		if(g_e8MotorCtrlMode  ==  (uint8_t)C_MOTOR_CTRL_STOP){pStatus  ->  u2StopMode = (uint8_t)C_STATUS_STOPMODE_STOP;}
		else{pStatus  ->  u2StopMode = (uint8_t)C_STATUS_STOPMODE_NORMAL;}
		if(COLIN_LINstatus.buffer_used  ==  0U)
		{
			if(ml_DataReady(ML_END_OF_TX_DISABLED)  !=  ML_SUCCESS)
			{
				g_u8ErrorCommunication = TRUE;
				SetLastError(C_ERR_LIN_API);
			}
		}
		else
		{
			g_u8ErrorCommunication = TRUE;
			SetLastError(C_ERR_LIN_BUF_NOT_FREE);
		}
	}
}

void HandleBusTimeout(void)
{
	if(g_u8LinAAMode  !=  (uint8_t)C_SNPD_SUBFUNC_INACTIVE){LinAATimeoutControl();}
	if(g_u8ErrorCommBusTimeout  ==  FALSE)
	{
		g_u8ErrorCommBusTimeout = TRUE;
		SetLastError(C_ERR_LIN_BUS_TIMEOUT);
		{
			APP_EOL_t *pEOL = (APP_EOL_t *)ADDR_NV_EOL;
			if(pEOL  ->  u1EmergencyRunPosEna  !=  0U)
			{
				if(g_e8DegradeStatus  !=  FALSE)
				{
					g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_EMRUN;
					g_e8DegradedMotorRequest = g_e8MotorRequest;
				}
				else{g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_EMRUN;}
			}
		}
	}
}

void HandleDataTransmitted(ml_MessageID_t Index)
{
	(void)Index;
}

void HandleLinError(ml_LinError_t Error)
{
	STD_LIN_PARAMS_t *pLIN = (STD_LIN_PARAMS_t *)ADDR_NV_STD_LIN_1;
	uint8_t u8FrameID = (uint8_t)(LinProtectedID & 0x3FU);
	if((u8FrameID  ==  (uint8_t)ML_MRF_ID)  &&  ((Error  ==  ml_erDataFraming)  ||  (Error  ==  ml_erCheckSum))){g_u8BufferOutID = (uint8_t)QR_INVALID;}
	if(Error  ==  ml_erLinModuleReset){Set_Mlx4ErrorState(C_MLX4_STATE_IMMEDIATE_RST);}
	else if(Error  ==  ml_erIdParity){}
	else if(u8FrameID  ==  (pLIN  ->  u8StatusFrameID & 0x3FU))
	{
		uint8_t u8CommNAD = LinFrame[6];
		if(u8CommNAD  ==  g_u8NAD){g_u8ErrorCommunication = TRUE;}
	}
	else if((u8FrameID  ==  (pLIN  ->  u8ControlFrameID & 0x3FU))  ||  (u8FrameID  ==  ML_MRF_ID)  ||  (u8FrameID  ==  ML_SRF_ID))
	{
		uint8_t u8CommNAD = LinFrame[0];
		if(u8CommNAD  ==  g_u8NAD){g_u8ErrorCommunication = TRUE;}
	}
}
