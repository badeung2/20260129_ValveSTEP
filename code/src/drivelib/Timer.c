#include "AppBuild.h"
#include "main.h"
#include "drivelib/Diagnostic.h"
#include "drivelib/ErrorCodes.h"
#include "drivelib/GlobalVars.h"
#include "drivelib/MotorDriver.h"
#include "drivelib/PID_Control.h"
#include "drivelib/Timer.h"
#include "camculib/private_mathlib.h"
#include "hal_lib/hal_STimer.h"
#if (I2C_COMM != FALSE) && (_SUPPORT_I2C_RW_ISR == FALSE)
#include "commlib/I2C_Generic.h"
#endif
#include "commlib/LIN_Diagnostics.h"
#include "commlib/LIN_AutoAddressing.h"
#if (_SUPPORT_UART != FALSE) && ((_SUPPORT_UART_IF_APP == C_UART_IF_SCOPE) || (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR) || (_SUPPORT_UART_IF_APP == C_UART_IF_ACTUATOR) || (_SUPPORT_UART_IF_APP == C_UART_IF_BLUETOOTH) || (_SUPPORT_UART2_IF_APP == C_UART_IF_BLUETOOTH) || (_SUPPORT_UART_IF_APP == C_UART_IF_TERMINAL) || (_SUPPORT_UART2_IF_APP == C_UART_IF_TERMINAL) || (_SUPPORT_UART_IF_APP == C_UART_IF_MLX_ACT))
#include "commlib/UART.h"
#endif
#include <atomic.h>

#pragma space dp
#pragma space none
#pragma space nodp
#pragma space none
void TimerInit(void)
{
	HAL_STimer_Init();
}

void TimerStop(void)
{
	HAL_STimer_Stop();
}

uint16_t TimerCheck(void)
{
	if(HAL_STimer_Check()  !=  FALSE)
	{
		SetLastError(C_ERR_PRIO | (C_ERR_EXT | 0x0A00U));
		TimerInit();
	}
	return(C_ERR_APP_OK);
}

void TimerSleepCompensation(uint16_t u16SleepCompensationPeriod)
{
	uint16_t u16SleepPeriod =
	p_MulDivU16_U16byU16byU16(u16SleepCompensationPeriod, 256U,
	(uint16_t)(CT_PERIODIC_RATE * (PLL_FREQ / 1000000U)));
	ENTER_SECTION(ATOMIC_KEEP_MODE);
	{
		MotorDriverPeriodicTimer(u16SleepPeriod);
		PID_CtrlCounter(u16SleepPeriod);
		LinDiagResponseTimeoutCount(u16SleepPeriod);
		main_PeriodicTimerEvent(u16SleepPeriod);
	}
	EXIT_SECTION();
}

__attribute__((interrupt)) void ISR_STIMER(void)
{
	MotorDriverPeriodicTimer(1U);
	PID_CtrlCounter(1U);
#if (I2C_COMM != FALSE) && (_SUPPORT_I2C_SLAVE != FALSE) && (_SUPPORT_I2C_RW_ISR == FALSE)
	I2C_PeriodicTimer();
#endif
	LinDiagResponseTimeoutCount(1U);
	LinAutoAddressingTimer();
	main_PeriodicTimerEvent(1U);
	DiagnosticPeriodicTimerEvent();
#if (_SUPPORT_UART != FALSE) && ((_SUPPORT_UART_IF_APP == C_UART_IF_SCOPE) || (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR) || (_SUPPORT_UART_IF_APP == C_UART_IF_ACTUATOR) || (_SUPPORT_UART_IF_APP == C_UART_IF_BLUETOOTH) || (_SUPPORT_UART2_IF_APP == C_UART_IF_BLUETOOTH) || (_SUPPORT_UART_IF_APP == C_UART_IF_TERMINAL) || (_SUPPORT_UART2_IF_APP == C_UART_IF_TERMINAL) || (_SUPPORT_UART_IF_APP == C_UART_IF_MLX_ACT))
	UART_PeriodicTimer();
#endif
}
