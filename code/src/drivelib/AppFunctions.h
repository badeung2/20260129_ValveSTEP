#ifndef DRIVE_LIB_APPFUNCTIONS_H
#define DRIVE_LIB_APPFUNCTIONS_H
#include "AppBuild.h"
#include "drivelib/Timer.h"
#define C_IWD_DIV 5U
#define C_IWD_TO (uint8_t)((100UL * FPLL) / (1U << (5U + (2U * C_IWD_DIV))))
#define C_SELFHEAT_COMP_PERIOD 500U
#define C_SELFHEAT_INTEGRATOR 64946U
#define C_SELFHEAT_CONST 85U
#define C_SELFHEAT_IC 2U
#define C_SELFHEAT_IC_IDLE 200
#define C_SELFHEAT_IC_ACTIVE 80
#define C_UV_FILTER_COUNT C_PI_TICKS_500MS
#define C_OV_FILTER_COUNT C_PI_TICKS_500MS
#define C_MLX4_STATE_TIMEOUT (300U * PI_TICKS_PER_MILLISECOND)
#define C_MLX4_STATE_ERROR_THRSHLD 4U
#define C_MLX4_STATE_IMMEDIATE_RST 0x80U
#define C_LIN_19200_MAX (uint16_t)(19200 * 1.1)
#define C_LIN_19200_MIN (uint16_t)(19200 * 0.9)
#define C_LIN_10417_MAX (uint16_t)(10417 * 1.1)
#define C_LIN_10417_MIN (uint16_t)(10417 * 0.9)
#define C_LIN_9600_MAX (uint16_t)(9600 * 1.1)
#define C_LIN_9600_MIN (uint16_t)(9600 * 0.9)
#define C_CHIP_CODE(x, y, z) ( (uint16_t)( (((uint16_t)(x) - (uint16_t)'@') << 10) | (((uint16_t)(y) - (uint16_t)'@') << 5) | ((uint16_t)(z) - (uint16_t)'@') ) )
#define C_CHIP_STATE_FATAL_RECOVER_ENA C_CHIP_CODE('F','R','E')
#define C_CHIP_STATE_FATAL_CRASH_RECOVERY C_CHIP_CODE('F','C','R')
#pragma space nodp
extern uint16_t l_u16ReversePolarityVdrop;
extern uint8_t g_u8MLX4_RAM_Dynamic_CRC;
#pragma space none
extern void AppInit(void);
extern void AppResetFlags(void);
extern void AppDegradedCheck(void);
extern void AppPeriodicTimerEvent(uint16_t u16Period);
extern void AppBackgroundHandler(void);
extern void AppStop(void);
extern void AppSleepWithWakeUpTimer(void);
extern void AppSleep(void);
extern void AppReset(void);
static inline int16_t Get_AmbientTemperature(void)
{
	extern int16_t l_i16AmbientTemperature;
	return(l_i16AmbientTemperature);
}

static inline void Set_Mlx4ErrorState(uint8_t u8Value)
{
	extern uint8_t l_u8Mlx4ErrorState;
	l_u8Mlx4ErrorState = u8Value;
}

static inline void ClearMlx4CheckPeriodCount(void)
{
	extern uint16_t l_u16Mlx4CheckPeriodCount;
	l_u16Mlx4CheckPeriodCount  &=  (uint8_t) ~C_MLX4_STATE_IMMEDIATE_RST;
}

#endif
