#ifndef DRIVE_LIB_PID_CONTROL_H
#define DRIVE_LIB_PID_CONTROL_H
#include "AppBuild.h"
#include "commlib/LIN_Communication.h"
#include "drivelib/NV_UserPage.h"
#define C_GN_PID 12U
#define C_PID_ANGLE_ACCURACY (0U + 0U)
#define C_PID_ANGLE_ACCURACY_IV (0U + 5U)
typedef struct
{
	int16_t i16CoefI;
	uint32_t u32SumError;
	uint32_t u32SumErrorMax;
	int16_t i16CoefP;
	int16_t i16PrevError;
	int16_t i16CoefD;
	uint16_t u16MinOutput;
	uint32_t u32MaxOutput;
}

PID_PARAMS_t;
#define C_SQRT3_DIV2 Q15(0.86602540378443864676372317075294)
#pragma space dp
#pragma space none
#pragma space nodp
#pragma space none
extern void PID_SetRunningCurrent(uint16_t u16CurrentLevel);
extern void PID_Init(void);
extern uint16_t PID_Start(uint16_t u16Losses, uint16_t u16Bemf);
#if ((_SUPPORT_FOC_MODE != FOC_MODE_NONE) && ((_SUPPORT_CLOSED_LOOP_STARTUP == FALSE) || (_SUPPORT_TRIAXIS_MLX9038x != FALSE))) || ((_SUPPORT_FOC_MODE == FOC_MODE_NONE) && ((_SUPPORT_HALL_LATCH == FALSE) || (_SUPPORT_NR_OF_HL == 1) || (_SUPPORT_HALL_LATCH_DIAG != FALSE)))
extern void PID_SpeedCompensate(uint16_t u16NewSpeed, uint16_t u16OldSpeed);
#endif
extern void PID_Open2Close(void);
extern void PID_Control(void);
extern uint16_t VoltageCorrection(void);
#pragma space dp
extern uint16_t l_u16PidCtrlRatio;
#pragma space none
static inline void StorePID_Info(void)
{
	extern PID_PARAMS_t sPIDpSE2VA;
	uint16_t *pDiagResponse = ((uint16_t *)(void *)&g_DiagResponse.u.SF.byD2);
	pDiagResponse[0] = l_u16PidCtrlRatio;
	pDiagResponse[1] = (uint16_t)(sPIDpSE2VA.u32SumError  >>  C_GN_PID);
}

static inline uint16_t Get_ActCurrRunMax_mA(void)
{
	extern uint16_t l_u16ActCurrRunMax_mA;
	return(l_u16ActCurrRunMax_mA);
}

static inline uint16_t Get_ActCurrRunMax_LSB(void)
{
	extern uint16_t l_u16ActCurrRunMax_LSB;
	return(l_u16ActCurrRunMax_LSB);
}

static inline void PID_CtrlCounter(uint16_t u16Period)
{
	extern uint16_t l_u16PID_CtrlCounter;
	extern uint16_t l_u16SelfHeatingCounter;
	l_u16PID_CtrlCounter  +=  u16Period;
	l_u16SelfHeatingCounter  +=  u16Period;
}

static inline uint16_t Get_HoldingThreshold(void)
{
	extern uint16_t l_u16PidHoldingThreshold;
	return(l_u16PidHoldingThreshold);
}

#endif
