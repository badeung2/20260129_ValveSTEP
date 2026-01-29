#include "AppBuild.h"
#include "main.h"
#include "ActADC.h"
#include "drivelib/AppFunctions.h"
#include "drivelib/ErrorCodes.h"
#include "drivelib/GlobalVars.h"
#include "drivelib/MotorDriver.h"
#include "drivelib/MotorStall.h"
#include "drivelib/PID_Control.h"
#include "camculib/private_mathlib.h"
#if (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL == 1U)
#include "senselib/HallLatch.h"
#endif
#include "commlib/LIN_Communication.h"
#include <atomic.h>
#include <bl_tools.h>
#include <mathlib.h>
#include <memory_map.h>
#include <mls_api.h>
#include <plib.h>
#include <sys_tools.h>

#pragma space dp
#pragma space none
#pragma space nodp
#pragma space none
void main_PeriodicTimerEvent(uint16_t u16Period)
{
	AppPeriodicTimerEvent(u16Period);
	if(g_u16MotorStartDelay  !=  0U)
	{
		if(g_u16MotorStartDelay > u16Period){g_u16MotorStartDelay  -=  u16Period;}
		else{g_u16MotorStartDelay = 0U;}
	}
}

void main_noinit_section_init(void)
{
}

static void main_Init(void)
{
	AppInit();
}

static void HandleEmergencyRunMotorRequest(void)
{
	if(g_e8MotorRequest  ==  (uint8_t)C_MOTOR_REQUEST_EMRUN)
	{
		g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
		if(NV_SAFETY_POSITION  ==  0U){g_u16TargetPosition = C_MIN_POS;}
		else{g_u16TargetPosition = C_MAX_POS;}
		if(g_u16ActualPosition  !=  g_u16TargetPosition)
		{
			g_e8EmergencyRunOcc = (uint8_t)C_SAFETY_RUN_ACTIVE;
			g_e8StallDetectorEna = StallDetectorEna();
			g_u8StallOcc = FALSE;
			if(g_e8MotorStatus  !=  C_MOTOR_STATUS_RUNNING){g_u8MotorCtrlSpeed = (uint8_t)C_MOTOR_SPEED_1;}
			if(g_e8DegradeStatus  !=  FALSE){g_e8DegradedMotorRequest = (uint8_t)C_MOTOR_REQUEST_START;}
			else{g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_START;}
		}
		else{}
	}
}

uint8_t HandleAutoSpeedMode(void)
{
	uint8_t u8MotorSpeedIdx;
	int16_t i16ChipTemperature = Get_ChipTemperature();
	uint16_t u16SupplyVoltage = Get_SupplyVoltage();
	int16_t i16TemperatureHyst = C_TEMPERATURE_HYS;
	uint16_t u16SupplyHyst = 25U;
	if((g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK)  ==  C_MOTOR_STATUS_STOP)
	{
		i16TemperatureHyst = 0;
		u16SupplyHyst = 0U;
		g_u8MotorStatusSpeed = (uint8_t)C_MOTOR_SPEED_2;
	}
	u8MotorSpeedIdx = (uint8_t)g_u8MotorStatusSpeed;
	if((i16ChipTemperature < (C_AUTOSPEED_TEMP_1 - i16TemperatureHyst))  ||  (u16SupplyVoltage < (((C_AUTOSPEED_VOLT_1 * 25U) / 2U) - u16SupplyHyst))){u8MotorSpeedIdx = (uint8_t)C_MOTOR_SPEED_1;}
	else if(((u16SupplyVoltage  >=  (((C_AUTOSPEED_VOLT_1 * 25U) / 2U) + u16SupplyHyst))  &&  (u16SupplyVoltage  <=  (((C_AUTOSPEED_VOLT_2 * 25U) / 2U) - u16SupplyHyst)))  ||  (((i16ChipTemperature  >=  (C_AUTOSPEED_TEMP_1 + i16TemperatureHyst))  &&  (i16ChipTemperature  <=  (C_AUTOSPEED_TEMP_2 - i16TemperatureHyst)))  ||  (i16ChipTemperature > (C_AUTOSPEED_TEMP_3 + i16TemperatureHyst)))){u8MotorSpeedIdx = (uint8_t)C_MOTOR_SPEED_2;}
	else if((i16ChipTemperature > (C_AUTOSPEED_TEMP_2 + i16TemperatureHyst))  &&  (i16ChipTemperature < (C_AUTOSPEED_TEMP_3 - i16TemperatureHyst))  &&  (u16SupplyVoltage > (((C_AUTOSPEED_VOLT_2 * 25U) / 2U) + u16SupplyHyst))){u8MotorSpeedIdx = (uint8_t)C_MOTOR_SPEED_3;}
	else{}
	return(u8MotorSpeedIdx);
}

static uint16_t HandleStartMotorRequest(void)
{
	uint16_t u16RequestHandled = TRUE;
	uint16_t u16DeltaPosition;
	uint8_t u8NewMotorDirectionCCW;
	if(g_u16ActualPosition > g_u16TargetPosition)
	{
		u16DeltaPosition = g_u16ActualPosition - g_u16TargetPosition;
		u8NewMotorDirectionCCW = TRUE;
	}
	else
	{
		u16DeltaPosition = g_u16TargetPosition - g_u16ActualPosition;
		u8NewMotorDirectionCCW = FALSE;
	}
	if(u16DeltaPosition  !=  0U)
	{
		uint8_t u8MotorSpeedIdx = (uint8_t)(g_u8MotorCtrlSpeed & 0x07U);
		if(g_u8MotorCtrlSpeed  ==  (uint8_t)C_MOTOR_SPEED_AUTO){u8MotorSpeedIdx = HandleAutoSpeedMode();}
		g_u8MotorStatusSpeed = u8MotorSpeedIdx;
		if((g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK)  ==  C_MOTOR_STATUS_STOP)
		{
			g_e8MotorDirectionCCW = u8NewMotorDirectionCCW;
			MotorDriverStart(u8MotorSpeedIdx);
			g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
		}
		else if(u8NewMotorDirectionCCW  !=  g_e8MotorDirectionCCW)
		{
			MotorDriverStop( (uint16_t)C_STOP_RAMPDOWN);
			u16RequestHandled = FALSE;
		}
		else
		{
			g_u32TargetPosition = ConvShaftSteps2MicroSteps(g_u16TargetPosition);
			MotorDriverSpeed(u8MotorSpeedIdx);
			g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
		}
	}
	else if(g_u8MotorHoldingCurrEna  !=  Get_MotorHoldingCurrState()){MotorDriverStop( (uint16_t)C_STOP_IMMEDIATE);}
	else{g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;}
	return(u16RequestHandled);
}

static void HandleSleepMotorRequest( void) __attribute__((noreturn));
static void HandleSleepMotorRequest(void)
{
	AppSleep();
	__builtin_unreachable();
}

static uint16_t HandleMotorRequest(void)
{
	uint16_t u16RequestHandled = TRUE;
	HandleEmergencyRunMotorRequest();
	if(g_e8MotorRequest  ==  (uint8_t)C_MOTOR_REQUEST_START_wINIT)
	{
		MotorDriverPosInit(g_u16ActualPosition);
		g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_START;
	}
	if(g_e8MotorRequest  ==  (uint8_t)C_MOTOR_REQUEST_NONE){}
	else if(g_e8MotorRequest  ==  (uint8_t)C_MOTOR_REQUEST_STOP)
	{
		MotorDriverStop( (uint16_t)C_STOP_RAMPDOWN);
		g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
		g_e8DegradedMotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
	}
	else if(g_e8MotorRequest  ==  (uint8_t)C_MOTOR_REQUEST_INIT)
	{
		MotorDriverInit(FALSE);
		g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
		g_e8DegradedMotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
	}
	else if(g_e8MotorRequest  ==  (uint8_t)C_MOTOR_REQUEST_START)
	{
		if(g_u16MotorStartDelay  ==  0U){u16RequestHandled = HandleStartMotorRequest();}
	}
	else if(g_e8MotorRequest  ==  (uint8_t)C_MOTOR_REQUEST_SPEED_CHANGE){MotorDriverSpeed(g_u16TargetMotorSpeedRPM);}
	else if(g_e8MotorRequest  ==  (uint8_t)C_MOTOR_REQUEST_SLEEP){HandleSleepMotorRequest();}
	else if(g_e8MotorRequest  ==  (uint8_t)C_MOTOR_REQUEST_RESET)
	{
		MotorDriverStop( (uint16_t)C_STOP_IMMEDIATE);
		g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
		ml_ResetDrv();
		g_u8LinInFrameBufState = (uint8_t)C_LIN_IN_FREE;
		MLX16_RESET_SIGNED( (BistResetInfo_t)C_CHIP_STATE_CMD_RESET);
	}
	else{g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;}
	return(u16RequestHandled);
}

static void AppBackgroundTaskHandler(void)
{
	PID_Control();
	AppBackgroundHandler();
}

int main(void)
{
	main_Init();
	for(;;)
	{
		p_AwdAck();
		if(g_u8LinInFrameBufState  !=  (uint8_t)C_LIN_IN_FREE){HandleLinInMsg();}
		AppDegradedCheck();
		if(HandleMotorRequest()  ==  FALSE){continue;}
		ConvMicroSteps2ShaftSteps();
		{
			extern uint16_t l_u16CommutTimerPeriod;
			uint16_t u16CopyActualCommutTimerPeriod = l_u16CommutTimerPeriod;
			if(((g_e8MotorStatus & (uint8_t)C_MOTOR_STATUS_STOP_MASK)  !=  (uint8_t)C_MOTOR_STATUS_STOP)  &&  (u16CopyActualCommutTimerPeriod  !=  0U)){g_u16ActualMotorSpeedRPM = p_DivU16_U32byU16(Get_MicroStepPeriodOneRPM(), u16CopyActualCommutTimerPeriod);}
			else{g_u16ActualMotorSpeedRPM = 0U;}
		}
		AppBackgroundTaskHandler();
	}
	return 0;
}
