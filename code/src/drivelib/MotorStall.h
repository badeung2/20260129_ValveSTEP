#ifndef DRIVE_LIB_MOTOR_STALL_H
#define DRIVE_LIB_MOTOR_STALL_H
#include "AppBuild.h"
#include "drivelib/NV_UserPage.h"
#include "drivelib/MotorDriver.h"
#include "camculib/private_mathlib.h"
#define C_STALL_NOT_FOUND 0x00U
#define C_STALL_FOUND 0x01U
#define M_STALL_MODE 0x78U
#define C_STALL_FOUND_A 0x40U
#define C_STALL_FOUND_PID_LA 0x10U
#define C_STALL_FOUND_LA 0x08U
#define C_CURROSC_SZ (2U * C_MICROSTEP_PER_FULLSTEP)
#define C_ANGLE_2DEG (uint16_t)((65536UL * 2U) / 720U)
#define C_ANGLE_3DEG (uint16_t)((65536UL * 3U) / 720U)
#define C_ANGLE_5DEG (uint16_t)((65536UL * 5U) / 720U)
#define C_LA_LPF_COEF_CL 256U
#define C_LA_LPF_COEF_OL 4096U
#define C_FLUX_UNITS 5U
#pragma space dp
#pragma space none
#pragma space nodp
#pragma space none
extern uint8_t StallDetectorEna(void);
extern void MotorStallInitA(void);
extern uint16_t MotorStallCheckA(void);
extern void LinDiag_MotorStall(void);
#pragma space dp
extern int16_t l_i16StallThresholdLA;
extern int16_t l_i16StallThresholdLA_Max;
extern int16_t l_i16ActLoadAngleLPF;
extern int32_t l_i32ActLoadAngleLPF;
extern volatile uint8_t l_u8StallCountLA;
#pragma space none
static inline void MotorStallInitLA(uint16_t u16MotorTargetSpeed)
{
	l_u8StallCountLA = 0U;
	l_i16ActLoadAngleLPF = 0;
	l_i32ActLoadAngleLPF = 0;
	if(u16MotorTargetSpeed  <=  C_MOTOR_SPEED_4){l_i16StallThresholdLA_Max = (NV_STALL_LA_THRSHLD_OL * C_ANGLE_3DEG);}
	else
	{
		l_i16StallThresholdLA_Max = -(NV_STALL_LA_THRSHLD * C_ANGLE_3DEG);
		l_i16StallThresholdLA = l_i16StallThresholdLA_Max;
		(void) u16MotorTargetSpeed;
	}
}

static inline void MotorStallSwitchOpen2CloseLA(void)
{
	l_u8StallCountLA = 0U;
	l_i16StallThresholdLA_Max = -(NV_STALL_LA_THRSHLD * C_ANGLE_3DEG);
	l_i16StallThresholdLA = l_i16StallThresholdLA_Max;
}

static inline uint16_t MotorStallCheckLA(void)
{
	if((Get_StartupDelay()  ==  0U)  &&  (g_u16ActualMotorSpeedRPM  >=  C_STALL_LA_SPEED_OFFSET)  &&  (l_i16StallThresholdLA_Max  !=  0))
	{
		if(l_i16StallThresholdLA_Max > 0)
		{
			l_i16StallThresholdLA =
			(int16_t)p_MulDivU16_U16byU16byU16(g_u16ForcedSpeedRPM,
			(uint16_t)l_i16StallThresholdLA_Max,
			g_u16MaxSpeedRPM);
		}
		if(l_i16ActLoadAngleLPF > l_i16StallThresholdLA){l_u8StallCountLA = p_DecNzU8(l_u8StallCountLA);}
		else
		{
			uint8_t u8StallCountLA = l_u8StallCountLA + 1U;
			l_u8StallCountLA = u8StallCountLA;
			if(u8StallCountLA  >=  NV_STALL_LA_WIDTH_OL){return(C_STALL_FOUND);}
		}
	}
	return(C_STALL_NOT_FOUND);
}

#endif
