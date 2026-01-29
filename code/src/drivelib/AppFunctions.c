#include "AppBuild.h"
#include "main.h"
#include "drivelib/ADC.h"
#include "drivelib/AppFunctions.h"
#include "drivelib/Diagnostic.h"
#include "drivelib/ErrorCodes.h"
#include "drivelib/GlobalVars.h"
#include "drivelib/MotorDriver.h"
#include "drivelib/MotorStall.h"
#include "drivelib/NV_Functions.h"
#include "drivelib/PID_Control.h"
#include "camculib/private_mathlib.h"
#include "drivelib/Timer.h"
#include "commlib/LIN_AutoAddressing.h"
#include "commlib/LIN_Communication.h"
#include "commlib/LIN_Diagnostics.h"
#include "../ActADC.h"
#include <bist_inline_impl.h>
#include <bl_bist.h>
#include <bl_tools.h>
#include <flash_defines.h>
#include <lib_clock.h>
#include <string.h>
#define _SUPPORT_IO_STYLE FALSE
#define _SUPPORT_BG_MEM_TEST_SEGMENTS TRUE
#define C_VS_LIN_MIN 525U
#define C_VS_LIN_HYST 25U
#define C_MIN_POS 0x0000U
#define C_MAX_POS 0xFFFEU
#define C_INI_POS 0x7FFFU
#define C_INV_POS 0xFFFFU
#define SSCM_DUR_CNT 4U
#define SSCM_STEP_CNT (uint8_t)(PLL_FREQ / (2UL * PWM_FREQ * SSCM_DUR_CNT))
#pragma space nodp
int16_t l_i16AmbientTemperature = 25;
uint16_t l_u16SelfHeatingCounter = 0U;
static uint32_t l_u32SelfHeatingIntegrator = 0U;
uint16_t l_u16ReversePolarityVdrop = 0U;
static uint16_t l_u16UnderVoltageFilterCount = 0U;
static uint16_t l_u16OverVoltageFilterCount = 0U;
uint16_t l_u16Mlx4CheckPeriodCount = 0U;
static uint16_t l_u16MLX4_RAM_Static_CRC = 0x0000U;
uint8_t l_u8Mlx4ErrorState = 0U;
static uint8_t l_u8Mlx4Connected = FALSE;
static uint32_t l_u32LpfAdcVdda = (C_MIN_VDDA + C_MAX_VDDA) * 32768UL;
static uint32_t l_u32LpfAdcVddd = (C_MIN_VDDD + C_MAX_VDDD) * 32768UL;
static uint16_t l_u16AdcVdda = (C_MIN_VDDA + C_MAX_VDDA) / 2U;
static uint16_t l_u16AdcVddd = (C_MIN_VDDD + C_MAX_VDDD) / 2U;
#pragma space none
static void App_noinit_section_init(void)
{
	g_e8StallDetectorEna = StallDetectorEna();
	if(NV_HOLDING_CURR_LEVEL  !=  0U){g_u8MotorHoldingCurrEna = TRUE;}
	else{g_u8MotorHoldingCurrEna = FALSE;}
	g_u16ActualPosition = C_INI_POS;
	g_u16TargetPosition = C_INV_POS;
	g_u8MotorCtrlSpeed = (uint8_t)C_DEFAULT_MOTOR_SPEED;
	g_e8MotorStatus = C_MOTOR_STATUS_STOP;
	g_e8DegradeStatus = FALSE;
	g_e8MotorCtrlMode = (uint8_t)C_MOTOR_CTRL_STOP;
}

void AppInit(void)
{
#if ((_SUPPORT_I2C_SLAVE != FALSE) && defined (PIN_FUNC_LIN) && (C_I2C_SLAVE_SCK_IO == PIN_FUNC_LIN))
	I2C_PreInit();
#endif
	{
		uint16_t u16FlashPD =
		(IO_EEPROM_FLASH_FL_CTRL & ~(M_EEPROM_FLASH_FL_PREDICTION_BEHAVIOR | M_EEPROM_FLASH_FL_DED_RETRY)) |
		(2U  <<  6);
		IO_EEPROM_FLASH_FL_CTRL = u16FlashPD;
	}
	builtin_mlx16_enter_user_mode();
	IO_PORT_STEP_CONF = (SSCM_STEP_CNT  <<  8) |
	(SSCM_DUR_CNT  <<  4) |
	(2U);
	IO_PORT_SSCM_CONF  |=  (B_PORT_SSCM_CONF_SSCM_CENTERED |
	B_PORT_SSCM_CONF_SSCM_EN);
	ErrorLogInit();
	l_u16MLX4_RAM_Static_CRC = p_CalcCRC_U16((const uint16_t *)C_RAM_MLX4_STATIC_BGN_ADDR,
	(C_RAM_MLX4_STATIC_END_ADDR - C_RAM_MLX4_STATIC_BGN_ADDR)/sizeof(uint16_t));
	if(NV_MlxCalib()  !=  FALSE)
	{
		g_e8ErrorElectric = (uint8_t)C_ERR_MEM_NVM_CALIB;
		SetLastError(C_ERR_INV_MLXPAGE_CRC1);
	}
	else{HAL_ADC_Conv_Init();}
	p_AwdAck();
	HAL_ADC_PreCheck();
	ADC_Init();
	{
		uint16_t u16NV_Error = NV_CheckCRC();
		if(u16NV_Error  !=  C_ERR_NONE)
		{
			SetLastError(C_ERR_INV_USERPAGE_1 | C_ERR_EXTW);
			SetLastError(u16NV_Error);
			(void)NV_WriteUserDefaults(u16NV_Error);
		}
	}
	if(((IO_PORT_MISC_IN & B_PORT_MISC_IN_RSTAT)  ==  0U)  ||  (bistResetInfo  !=  C_CHIP_STATE_CMD_RESET))
	{
		App_noinit_section_init();
		main_noinit_section_init();
	}
	DiagnosticInit();
	ADC_MeasureVsupplyAndTemperature();
	{
		uint16_t u16MotorVoltage = ADC_Conv_Vmotor();
		if(u16MotorVoltage < ((NV_APPL_UVOLT - C_VOLTAGE_HYS) - l_u16ReversePolarityVdrop)){g_e8ErrorVoltage = (uint8_t)C_ERR_VOLTAGE_UNDER;}
		else if(u16MotorVoltage > ((NV_APPL_OVOLT + C_VOLTAGE_HYS) - l_u16ReversePolarityVdrop)){g_e8ErrorVoltage = (uint8_t)C_ERR_VOLTAGE_OVER;}
		else{}
		g_e8ErrorVoltageComm = g_e8ErrorVoltage;
	}
	(void)ADC_Conv_TempJ(TRUE);
	MotorDriverInit(TRUE);
	builtin_mlx16_set_priority(15U);
	PID_Init();
	g_e8MotorStatus = C_MOTOR_STATUS_STOP;
	TimerInit();
	LIN_Init();
	l_u8Mlx4Connected = TRUE;
	if(bistResetInfo  ==  (uint16_t)C_CHIP_STATE_CMD_RESET)
	{
		RfrDiagReset();
		bistResetInfo = (BistResetInfo_t)C_CHIP_STATE_COLD_START;
	}
	MotorDriverStop( (uint16_t)C_STOP_IMMEDIATE);
	bistHeader = (BistHeader_t)C_CHIP_HEADER;
	bistResetInfo = (BistResetInfo_t)C_CHIP_STATE_FATAL_RECOVER_ENA;
}

void AppResetFlags(void)
{
	g_u8ChipResetOcc = FALSE;
	g_u8StallOcc = FALSE;
	g_e8EmergencyRunOcc = (uint8_t)C_SAFETY_RUN_NO;
}

void SelfHeatCompensation(void)
{
	if(l_u16SelfHeatingCounter  >=  C_SELFHEAT_COMP_PERIOD)
	{
		uint16_t u16SelfHeatingDrv;
		uint16_t u16SelfHeatingIC;
		l_u16SelfHeatingCounter  -=  C_SELFHEAT_COMP_PERIOD;
		l_u32SelfHeatingIntegrator = p_MulU32byU16(l_u32SelfHeatingIntegrator, C_SELFHEAT_INTEGRATOR) +
		((Get_MotorCurrentMovAvgxN() + (1U  <<  (C_MOVAVG_SSZ - 1U)))  >>  C_MOVAVG_SSZ);
		u16SelfHeatingDrv = (uint16_t)p_MulU32byU16(l_u32SelfHeatingIntegrator, C_SELFHEAT_CONST);
		if((g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK)  ==  C_MOTOR_STATUS_STOP){u16SelfHeatingIC = p_DivU16_U32byU16( (uint32_t)Get_SupplyVoltage(), C_SELFHEAT_IC_IDLE);}
		else{u16SelfHeatingIC = p_DivU16_U32byU16( (uint32_t)Get_SupplyVoltage(), C_SELFHEAT_IC_ACTIVE);}
		u16SelfHeatingIC  +=  C_SELFHEAT_IC;
		l_i16AmbientTemperature = (Get_ChipTemperature() - (int16_t)(u16SelfHeatingDrv + u16SelfHeatingIC));
		{
			int16_t i16TempDiff = (Get_RawTemperature() - Get_TempMidADC());
			uint16_t u16TrimChargePumpClock = g_u16CP_FreqTrim_RT;
			int16_t i16CPCLK82_TempCoef = (int16_t)CalibrationParams.u8APP_TRIM29_CPCLK82_HighT;
			int16_t i16CPCLK60_TempCoef = (int16_t)CalibrationParams.u8APP_TRIM29_CPCLK60_HighT;
			int16_t i16CPCLK_TempCoef;
			if(i16CPCLK60_TempCoef  ==  0){i16CPCLK60_TempCoef = C_DEF_CPCLK60_TC;}
			if(i16CPCLK82_TempCoef  ==  0){i16CPCLK82_TempCoef = C_DEF_CPCLK82_TC;}
			i16CPCLK_TempCoef = i16CPCLK60_TempCoef +
			p_MulDivI16_I16byI16byI16( (i16CPCLK82_TempCoef -
			i16CPCLK60_TempCoef),
			(_SUPPORT_CPFREQ - 60), (82 - 60));
			u16TrimChargePumpClock =
			(u16TrimChargePumpClock + (int16_t)(p_MulI32_I16byI16(i16CPCLK_TempCoef, i16TempDiff)  >>  7));
			u16TrimChargePumpClock = ((u16TrimChargePumpClock  <<  2) & M_TRIM1_DRV_PRE_TRIM_DRVMOD_CPCLK);
			IO_PORT_SSCM2_CONF  &=  ~B_PORT_SSCM2_CONF_SSCM2_EN;
			IO_TRIM1_DRV = (IO_TRIM1_DRV & ~M_TRIM1_DRV_PRE_TRIM_DRVMOD_CPCLK) | u16TrimChargePumpClock;
			IO_PORT_SSCM2_CONF  |=  B_PORT_SSCM2_CONF_SSCM2_EN;
		}
	}
}

static void AppCurrentCheck(void)
{
	if((g_e8MotorStatus  ==  C_MOTOR_STATUS_STOP)  &&  (l_u8AdcMode  !=  (uint8_t)C_ADC_MODE_LINAA)){ADC_MeasureVsupplyAndTemperature();}
	else if(g_e8MotorStatus  ==  C_MOTOR_STATUS_HOLD){MotorDriverHoldCurrentMeasure();}
	else{}
	(void)ADC_Conv_Cmotor();
}

static void AppInternalSupplyCheck(void)
{
	uint16_t u16AdcValue = l_u16AdcVdda;
	if((u16AdcValue < C_MIN_VDDA)  ||  (u16AdcValue > C_MAX_VDDA))
	{
		g_e8ErrorElectric = (uint8_t)(C_ERR_SUP_VDDA & ~C_ERR_PERMANENT);
		if(u16AdcValue < C_MIN_VDDA){SetLastError(C_ERR_VDDA | C_ERR_EXTW | 0x0C00U);}
		else{SetLastError(C_ERR_VDDA | C_ERR_EXTW | 0x0D00U);}
		SetLastError(u16AdcValue);
	}
	u16AdcValue = l_u16AdcVddd;
	if((u16AdcValue < C_MIN_VDDD)  ||  (u16AdcValue > C_MAX_VDDD))
	{
		g_e8ErrorElectric = (uint8_t)(C_ERR_SUP_VDDD & ~C_ERR_PERMANENT);
		if(u16AdcValue < C_MIN_VDDD){SetLastError(C_ERR_VDDD | C_ERR_EXTW | 0x0C00U);}
		else{SetLastError(C_ERR_VDDD | C_ERR_EXTW | 0x0D00U);}
		SetLastError(u16AdcValue);
	}
}

static void AppSupplyCheck(void)
{
	static uint8_t e8ErrorDebounceFilter = 0U;
	uint16_t u16MotorVoltage = ADC_Conv_Vmotor();
	(void)ADC_Conv_Vsupply();
	if((u16MotorVoltage < ((NV_APPL_UVOLT - C_VOLTAGE_HYS) - l_u16ReversePolarityVdrop))  &&  (g_e8ErrorVoltage  !=  (uint8_t)C_ERR_VOLTAGE_UNDER))
	{
		if((e8ErrorDebounceFilter & (uint8_t)C_DEBFLT_ERR_UV)  ==  (uint8_t)0x00U)
		{
			e8ErrorDebounceFilter  |=  (uint8_t)C_DEBFLT_ERR_UV;
			l_u16UnderVoltageFilterCount = C_UV_FILTER_COUNT;
		}
		else if(l_u16UnderVoltageFilterCount  ==  0U)
		{
			g_e8ErrorVoltage = (uint8_t)C_ERR_VOLTAGE_UNDER;
			g_e8ErrorVoltageComm = g_e8ErrorVoltage;
			SetLastError(C_ERR_APPL_UNDER_VOLT);
		}
		else{}
	}
	else if((u16MotorVoltage > ((NV_APPL_OVOLT + C_VOLTAGE_HYS) - l_u16ReversePolarityVdrop))  &&  (g_e8ErrorVoltage  !=  (uint8_t)C_ERR_VOLTAGE_OVER))
	{
		if((e8ErrorDebounceFilter & C_DEBFLT_ERR_OV)  ==  (uint8_t)0x00U)
		{
			e8ErrorDebounceFilter  |=  (uint8_t)C_DEBFLT_ERR_OV;
			l_u16OverVoltageFilterCount = C_OV_FILTER_COUNT;
		}
		else if(l_u16OverVoltageFilterCount  ==  0U)
		{
			g_e8ErrorVoltage = (uint8_t)C_ERR_VOLTAGE_OVER;
			g_e8ErrorVoltageComm = g_e8ErrorVoltage;
			SetLastError(C_ERR_APPL_OVER_VOLT);
		}
		else{}
	}
	else if((u16MotorVoltage  >=  ((NV_APPL_UVOLT + C_VOLTAGE_HYS) - l_u16ReversePolarityVdrop))  &&  (u16MotorVoltage  <=  ((NV_APPL_OVOLT - C_VOLTAGE_HYS) - l_u16ReversePolarityVdrop)))
	{
		g_e8ErrorVoltage = (uint8_t)C_ERR_VOLTAGE_IN_RANGE;
		e8ErrorDebounceFilter  &=  (uint8_t) ~(C_DEBFLT_ERR_UV | C_DEBFLT_ERR_OV);
		l_u16UnderVoltageFilterCount = 0U;
		l_u16OverVoltageFilterCount = 0U;
	}
	else
	{
		e8ErrorDebounceFilter  &=  (uint8_t) ~(C_DEBFLT_ERR_UV | C_DEBFLT_ERR_OV);
		l_u16UnderVoltageFilterCount = 0U;
		l_u16OverVoltageFilterCount = 0U;
	}
	AppInternalSupplyCheck();
}

static void AppTemperatureCheck(void)
{
	static uint8_t e8ErrorDebounceFilter = 0U;
	int16_t i16ChipTemperature = ADC_Conv_TempJ(FALSE);
	if((g_e8ErrorOverTemperature & C_ERR_OTEMP_ERROR)  ==  0U)
	{
		SelfHeatCompensation();
		if((((l_i16AmbientTemperature > (int16_t)(NV_APPL_OTEMP + C_TEMPERATURE_HYS))  &&  ((g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK)  ==  C_MOTOR_STATUS_STOP))  ||  (i16ChipTemperature > (int16_t)(C_CHIP_OVERTEMP_LEVEL + C_TEMPERATURE_HYS)))  &&  (g_e8ErrorOverTemperature  !=  (uint8_t)C_ERR_OTEMP_SHUTDOWN))
		{
			if((e8ErrorDebounceFilter & (uint8_t)C_DEBFLT_ERR_OVT)  ==  (uint8_t)0x00U){e8ErrorDebounceFilter  |=  (uint8_t)C_DEBFLT_ERR_OVT;}
			else
			{
				g_e8ErrorOverTemperature = (uint8_t)C_ERR_OTEMP_SHUTDOWN;
				g_u16TargetPosition = g_u16ActualPosition;
				SetLastError(C_ERR_APPL_OVER_TEMP);
			}
		}
		else if(l_i16AmbientTemperature < (NV_APPL_OTEMP - C_TEMPERATURE_HYS))
		{
			g_e8ErrorOverTemperature = (uint8_t)C_ERR_OTEMP_NO;
			e8ErrorDebounceFilter  &=  (uint8_t) ~C_DEBFLT_ERR_OVT;
		}
		else{}
	}
}

void AppDegradedCheck(void)
{
	AppCurrentCheck();
	AppSupplyCheck();
	AppTemperatureCheck();
	if(((g_e8ErrorVoltage  !=  (uint8_t)C_ERR_VOLTAGE_IN_RANGE)  ||  (g_e8ErrorOverTemperature  !=  (uint8_t)C_ERR_OTEMP_NO))  &&  (g_e8DegradeStatus  ==  FALSE))
	{
		if(g_e8MotorRequest  !=  C_MOTOR_REQUEST_NONE)
		{
			g_e8DegradedMotorRequest = g_e8MotorRequest;
			MotorDriverStop( (uint16_t)C_STOP_RAMPDOWN);
		}
		else if((g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK)  !=  C_MOTOR_STATUS_STOP)
		{
			g_e8DegradedMotorRequest = (uint8_t)C_MOTOR_REQUEST_START;
			MotorDriverStop( (uint16_t)C_STOP_RAMPDOWN);
		}
		else if(g_e8DegradedMotorRequest  ==  (uint8_t)C_MOTOR_REQUEST_NONE)
		{
			g_e8DegradedMotorRequest = (uint8_t)C_MOTOR_REQUEST_STOP;
			MotorDriverStop( (uint16_t)C_STOP_EMERGENCY);
		}
		else{}
		g_e8DegradeStatus = TRUE;
	}
	else if((g_e8DegradeStatus  !=  FALSE)  &&  (g_e8ErrorVoltage  ==  (uint8_t)C_ERR_VOLTAGE_IN_RANGE)  &&  (g_e8ErrorOverTemperature  ==  (uint8_t)C_ERR_OTEMP_NO))
	{
		if(g_e8DegradedMotorRequest  !=  (uint8_t)C_MOTOR_REQUEST_NONE)
		{
			g_e8MotorRequest = (uint8_t)g_e8DegradedMotorRequest;
			g_e8DegradedMotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
		}
		g_e8DegradeStatus = FALSE;
	}
	else{}
}

static void AppCheckLinProc(void)
{
	static uint8_t l_u8LinBufferFull = 0U;
	if((l_u16Mlx4CheckPeriodCount > C_MLX4_STATE_TIMEOUT)  &&  (g_u8LinAAMode  ==  (uint8_t)C_SNPD_SUBFUNC_INACTIVE))
	{
		ml_LinState_t eLinState;
		ENTER_SECTION(ATOMIC_KEEP_MODE);
		eLinState = ml_GetState(ML_CLR_LIN_BUS_ACTIVITY);
		EXIT_SECTION();
		if(eLinState  !=  ml_stINVALID)
		{
			l_u8Mlx4ErrorState  &=  (uint8_t) ~C_MLX4_STATE_IMMEDIATE_RST;
			if((ML_DATA_LIN_STATUS & ML_LIN_BUFFER_NOT_FREE)  !=  0U)
			{
				l_u8LinBufferFull++;
				if(l_u8LinBufferFull  >=  2U)
				{
					SetLastError(C_ERR_LIN_BUF_NOT_FREE);
					l_u8LinBufferFull = 0U;
				}
			}
			else if((ML_DATA_LIN_STATUS & ML_LIN_CMD_OVERFLOW)  !=  0U)
			{
				ENTER_SECTION(ATOMIC_KEEP_MODE);
				(void)ml_GetState(ML_CLR_LIN_CMD_OVERFLOW);
				EXIT_SECTION();
				ml_SetSLVCMD(0x42U);
				SetLastError(C_ERR_LIN_CMD_OVF);
				l_u8LinBufferFull = 0U;
			}
			else if((l_u8Mlx4Connected  !=  FALSE)  &&  (eLinState  ==  ml_stDISCONNECTED)){(void)ml_Connect();}
			else
			{
				uint16_t u16LinBaudRate = p_ml_GetBaudRate(MLX4_FPLL);
				if((u16LinBaudRate  !=  0U)  &&  ((u16LinBaudRate > C_LIN_19200_MAX)  ||  ((u16LinBaudRate < C_LIN_19200_MIN)  &&  (u16LinBaudRate > C_LIN_10417_MAX))  ||  (u16LinBaudRate < C_LIN_9600_MIN)))
				{
					(void)ml_SetAutoBaudRateMode(ML_ABR_ON_FIRST_FRAME);
					SetLastError(C_ERR_LIN_BAUDRATE | C_ERR_EXTW);
					SetLastError(u16LinBaudRate);
				}
				l_u8LinBufferFull = 0U;
			}
		}
		else
		{
			l_u8Mlx4ErrorState++;
			SetLastError(C_WRN_LIN_TIMEOUT | C_ERR_EXT | ((l_u8Mlx4ErrorState & 0x0FU)  <<  8));
		}
		if(l_u16MLX4_RAM_Static_CRC  !=  p_CalcCRC_U16((const uint16_t *)C_RAM_MLX4_STATIC_BGN_ADDR, (C_RAM_MLX4_STATIC_END_ADDR - C_RAM_MLX4_STATIC_BGN_ADDR)/sizeof(uint16_t)))
		{
			SetLastError(C_ERR_LIN_RAM_STATIC);
			(void)mlu_ApplicationStop();
			MLX16_RESET_COLD();
		}
		if(g_u16MLX4_RAM_Dynamic_CRC1  !=  p_CalcCRC_U16((const uint16_t *)C_RAM_MLX4_DYNAMIC_BGN_ADR1, (C_RAM_MLX4_DYNAMIC_END_ADR1 - C_RAM_MLX4_DYNAMIC_BGN_ADR1)/sizeof(uint16_t)))
		{
			SetLastError(C_ERR_LIN_RAM_DYNAMIC | C_ERR_EXT | 0x0100U);
			l_u8Mlx4ErrorState = (uint8_t)C_MLX4_STATE_IMMEDIATE_RST;
		}
		if(g_u16MLX4_RAM_Dynamic_CRC2  !=  p_CalcCRC_U16((const uint16_t *)C_RAM_MLX4_DYNAMIC_BGN_ADR2, (C_RAM_MLX4_DYNAMIC_END_ADR2 - C_RAM_MLX4_DYNAMIC_BGN_ADR2)/sizeof(uint16_t)))
		{
			SetLastError(C_ERR_LIN_RAM_DYNAMIC | C_ERR_EXT | 0x0200U);
			l_u8Mlx4ErrorState = (uint8_t)C_MLX4_STATE_IMMEDIATE_RST;
		}
		{
			uint16_t u16LinBaudRate = p_ml_GetLastBaudRate(MLX4_FPLL);
			uint16_t u16LinAutoBaudRate = p_ml_GetAutoBaudRate(MLX4_FPLL);
			if((u16LinAutoBaudRate  !=  0U)  &&  ((u16LinAutoBaudRate > C_LIN_19200_MAX)  ||  ((u16LinAutoBaudRate < C_LIN_19200_MIN)  &&  (u16LinAutoBaudRate > C_LIN_10417_MAX))  ||  (u16LinAutoBaudRate < C_LIN_9600_MIN)))
			{
				(void)mlu_ApplicationStop();
				SetLastError(C_ERR_LIN_RAM_DYNAMIC | C_ERR_EXT | 0x0300U);
				l_u8Mlx4ErrorState = (uint8_t)C_MLX4_STATE_IMMEDIATE_RST;
			}
			if((u16LinBaudRate  !=  0U)  &&  ((u16LinBaudRate > (u16LinAutoBaudRate + (u16LinAutoBaudRate  >>  3)))  ||  (u16LinBaudRate < (u16LinAutoBaudRate - (u16LinAutoBaudRate  >>  3)))))
			{
				(void)mlu_ApplicationStop();
				SetLastError(C_ERR_LIN_RAM_DYNAMIC | C_ERR_EXT | 0x0400U);
				l_u8Mlx4ErrorState = (uint8_t)C_MLX4_STATE_IMMEDIATE_RST;
			}
		}
		l_u16Mlx4CheckPeriodCount = 0U;
	}
	if((l_u8Mlx4ErrorState  >=  (uint8_t)C_MLX4_STATE_ERROR_THRSHLD)  &&  ((g_e8MotorStatus & (uint8_t)C_MOTOR_STATUS_APPL_STOP)  ==  0x00U)  &&  (g_e8MotorRequest  !=  (uint8_t)C_MOTOR_REQUEST_SLEEP))
	{
		ENTER_SECTION(SYSTEM_MODE);
		ml_ResetDrv();
		EXIT_SECTION();
		NOP();
		NOP();
		NOP();
		ml_StartDrv();
		SetLastError(C_ERR_MLX4_RESTART);
		LIN_Init();
		l_u8Mlx4ErrorState = 0U;
	}
}

void AppPeriodicTimerEvent(uint16_t u16Period)
{
	if(l_u16UnderVoltageFilterCount > u16Period){l_u16UnderVoltageFilterCount  -=  u16Period;}
	else{l_u16UnderVoltageFilterCount = 0U;}
	if(l_u16OverVoltageFilterCount > u16Period){l_u16OverVoltageFilterCount  -=  u16Period;}
	else{l_u16OverVoltageFilterCount = 0U;}
	l_u16Mlx4CheckPeriodCount  +=  u16Period;
	l_u16AdcVdda = p_LpfU16_I16byI16(&l_u32LpfAdcVdda, C_VDDA_LPF_COEF, (int16_t)(Get_AdcVdda() - l_u16AdcVdda));
	l_u16AdcVddd = p_LpfU16_I16byI16(&l_u32LpfAdcVddd, C_VDDD_LPF_COEF, (int16_t)(Get_AdcVddd() - l_u16AdcVddd));
	(void) u16Period;
}

static void AppTemperatureProfileCheck(void)
{
}

static void AppProcPowerSave(void)
{
	uint16_t u16TimerCnt = 0U;
	if( ((g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK)  ==  C_MOTOR_STATUS_STOP)
	&&  (Get_MotorDriverDisconDelay()  ==  0U)
#if ((CAN_COMM != FALSE) || (PWM_COMM != FALSE) || (SPI_COMM != FALSE) || ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR)))
	&&  (g_u8LinConnected  !=  C_COMM_DISCONNECTED)
#endif
	&&  (g_u8LinAAMode  ==  (uint8_t)C_SNPD_SUBFUNC_INACTIVE)
	)
	{
		uint16_t u16CopyMask0;
		uint16_t u16CopyMask1;
		uint16_t u16CopyMask2;
		uint16_t u16CopyMask3;
		uint16_t u16CopyMask4;
		uint16_t u16Timer0Ctrl;
		ENTER_SECTION(SYSTEM_MODE);
		u16CopyMask0 = IO_MLX16_ITC_MASK0_S;
		u16CopyMask1 = IO_MLX16_ITC_MASK1_S;
		u16CopyMask2 = IO_MLX16_ITC_MASK2_S;
		u16CopyMask3 = IO_MLX16_ITC_MASK3_S;
		u16CopyMask4 = IO_MLX16_ITC_MASK4_S;
		u16Timer0Ctrl = IO_CTIMER0_CTRL;
		if((g_e8MotorStatus & C_MOTOR_STATUS_MASK)  ==  C_MOTOR_STATUS_STOP){HAL_ADC_PowerOff();}
		IO_CTIMER0_CTRL = B_CTIMER0_STOP;
		IO_CTIMER0_CTRL = C_SLEEP_TIMER_CONFIG;
		IO_CTIMER0_TREGB = C_SLEEP_TIMER_PERIOD;
		IO_MLX16_ITC_MASK0_S = (B_MLX16_ITC_MASK0_AWD_ATT
		| B_MLX16_ITC_MASK0_UV_VDDA
		| B_MLX16_ITC_MASK0_UV_VS
		| B_MLX16_ITC_MASK0_OVT
		| B_MLX16_ITC_MASK0_OVC
		);
		IO_MLX16_ITC_MASK1_S = (0
		| B_MLX16_ITC_MASK1_CTIMER0_3
#if (_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL == 3U) && (_SUPPORT_MICRO_STEP_COMMUTATION != FALSE)
		| HL_TIMER_MASK_CONF
#endif
		);
		IO_MLX16_ITC_MASK2_S = (B_MLX16_ITC_MASK2_COLIN_LIN
		| B_MLX16_ITC_MASK2_OV_VS
		| B_MLX16_ITC_MASK2_DIAG
		);
		IO_MLX16_ITC_MASK3_S = 0U;
		IO_MLX16_ITC_MASK4_S = 0U;
		IO_CTIMER0_CTRL = B_CTIMER0_START;
		IO_PORT_STOPMD_CTRL_S = B_PORT_STOPMD_CTRL_SEL_STOP_MODE;
		__asm__ __volatile__ (
		"mov Y, M \n\t"
		"clrb MH.2 \n\t"
		"clrb MH.1 \n\t"
		"clrb MH.0 \n\t"
		"lod AL, _g_u8LinInFrameBufState \n\t"
		"jne _HALT_10 \n\t"
		"mov M, Y \n\t"
		"HALT \n\t"
		"jmp _HALT_20 \n\t"
		"_HALT_10: \n\t"
		"mov M, Y \n\t"
		"_HALT_20:"
		:
		:
		: "A","Y"
		);
		u16TimerCnt = IO_CTIMER0_TCNT;
		{
			uint16_t u16MiscOut = IO_PORT_MISC_OUT & ~B_PORT_MISC_OUT_CLEAR_STOP;
			IO_PORT_MISC_OUT = u16MiscOut | B_PORT_MISC_OUT_CLEAR_STOP;
			IO_PORT_MISC_OUT = u16MiscOut;
		}
		IO_PORT_ADC_CTRL_S = B_PORT_ADC_CTRL_ADC_EN;
		DELAY_US(10);
		if(u16TimerCnt  <=  1U){u16TimerCnt  +=  IO_CTIMER0_TREGB;}
		IO_CTIMER0_CTRL = B_CTIMER0_STOP;
		IO_CTIMER0_CTRL = (u16Timer0Ctrl & ~(B_CTIMER0_STOP | B_CTIMER0_START));
		IO_MLX16_ITC_MASK0_S  |=  u16CopyMask0;
		IO_MLX16_ITC_MASK1_S  |=  u16CopyMask1;
		IO_MLX16_ITC_MASK2_S  |=  u16CopyMask2;
		IO_MLX16_ITC_MASK3_S  |=  u16CopyMask3;
		IO_MLX16_ITC_MASK4_S  |=  u16CopyMask4;
		EXIT_SECTION();
		TimerSleepCompensation(u16TimerCnt);
	}
}

static inline uint16_t PendIrqCheck(volatile uint16_t *pu16Pend, volatile uint16_t *pu16Mask)
{
	uint16_t u16PendingIRQ;
	u16PendingIRQ = (*pu16Pend & *pu16Mask);
	if(u16PendingIRQ  !=  0U){u16PendingIRQ = u16PendingIRQ & *pu16Pend;}
	return(u16PendingIRQ);
}

static void AppChipCheck(void)
{
	uint16_t u16PendingIRQ;
	u16PendingIRQ = PendIrqCheck(&IO_MLX16_ITC_PEND0_S, &IO_MLX16_ITC_MASK0_S);
	if(u16PendingIRQ  !=  0U)
	{
		SetLastError(C_ERR_PRIO | (C_ERR_EXT | 0x0100U));
		SetLastError(u16PendingIRQ);
	}
	u16PendingIRQ = PendIrqCheck(&IO_MLX16_ITC_PEND1_S, &IO_MLX16_ITC_MASK1_S);
	if((u16PendingIRQ & ~B_MLX16_ITC_PEND1_PWM_MASTER1_END)  !=  0U)
	{
		SetLastError(C_ERR_PRIO | (C_ERR_EXT | 0x0200U));
		SetLastError(u16PendingIRQ);
	}
	u16PendingIRQ = PendIrqCheck(&IO_MLX16_ITC_PEND2_S, &IO_MLX16_ITC_MASK2_S);
	if(u16PendingIRQ  !=  0U)
	{
		SetLastError(C_ERR_PRIO | (C_ERR_EXT | 0x0300U));
		SetLastError(u16PendingIRQ);
	}
	u16PendingIRQ = PendIrqCheck(&IO_MLX16_ITC_PEND3_S, &IO_MLX16_ITC_MASK3_S);
	if(u16PendingIRQ  !=  0U)
	{
		SetLastError(C_ERR_PRIO | (C_ERR_EXT | 0x0400U));
		SetLastError(u16PendingIRQ);
	}
	u16PendingIRQ = PendIrqCheck(&IO_MLX16_ITC_PEND4_S, &IO_MLX16_ITC_MASK4_S);
	if(u16PendingIRQ  !=  0U)
	{
		SetLastError(C_ERR_PRIO | (C_ERR_EXT | 0x0500U));
		SetLastError(u16PendingIRQ);
	}
#if ((CAN_COMM != FALSE) || (PWM_COMM != FALSE) || (SPI_COMM != FALSE) || ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR)))
	if(g_u8LinConnected  !=  C_COMM_DISCONNECTED)
#endif
	{
		if((ml_GetSLVIT() & (uint8_t)0x01U)  ==  (uint8_t)0x0U)
		{
			ml_SetSLVIT(0xABU);
			SetLastError(C_ERR_PRIO | (C_ERR_EXT | 0x0E00U));
		}
	}
	{
		uint16_t u16CpuStatus = builtin_mlx16_get_status();
		if((u16CpuStatus & 0x0700U)  !=  0x0700U)
		{
			builtin_mlx16_set_priority(15U);
			SetLastError(C_ERR_PRIO | (C_ERR_EXT | 0x0F00U));
		}
	}
	(void)TimerCheck();
}

void AppBackgroundHandler(void)
{
#if (LIN_COMM != FALSE) && ((CAN_COMM != FALSE) || (PWM_COMM != FALSE) || (SPI_COMM != FALSE) || ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR)))
	if(g_u8LinConnected  !=  C_COMM_DISCONNECTED)
#endif
	{
		AppCheckLinProc();
	}
	AppTemperatureProfileCheck();
	AppChipCheck();
	AppProcPowerSave();
}

void AppStop(void)
{
	MotorDriverStop( (uint16_t)C_STOP_WO_HOLDING);
	SetLastError(C_ERR_APPL_STOP);
	HAL_ADC_StopSafe();
#if ((_SUPPORT_SPI != FALSE) || (_SUPPORT_3WIRE_SPI != FALSE)) && (_SUPPORT_SPI_DMA != FALSE)
	SPI_Disconnect();
#endif
	IO_PORT_PPM_CTRL  &=  ~B_PORT_PPM_CTRL_PPM_EN;
	ENTER_SECTION(SYSTEM_MODE);
	IO_MLX16_ITC_MASK0_S = 0U;
	IO_MLX16_ITC_MASK1_S = 0U;
	IO_MLX16_ITC_MASK2_S = B_MLX16_ITC_MASK2_COLIN_LIN;
	IO_MLX16_ITC_MASK3_S = 0U;
	IO_MLX16_ITC_MASK4_S = 0U;
	EXIT_SECTION();
	g_e8MotorStatus = C_MOTOR_STATUS_APPL_STOP;
}

void AppSleepWithWakeUpTimer(void)
{
	IO_PORT_MISC_OUT = (IO_PORT_MISC_OUT & ~M_PORT_MISC_OUT_WUI) | C_PORT_MISC_OUT_WUI_400ms;
	AppSleep();
}

void AppSleep( void) __attribute__((noreturn));
void AppSleep(void)
{
	AppStop();
	builtin_mlx16_disable_interrupts();
	NV_AppStore(g_u16ActualPosition, g_e8ErrorElectric);
	ENTER_SECTION(SYSTEM_MODE);
	ml_ResetDrv();
	EXIT_SECTION();
	while(IO_GET(EEPROM_FLASH, EE_BUSY)  !=  0U)
	{
	};
	while((IO_GET(EEPROM_FLASH, FL_STATUS)  ==  C_EEPROM_FLASH_FL_STATUS_PAGE_PROGRAM)  ||  (IO_GET(EEPROM_FLASH, FL_STATUS)  ==  C_EEPROM_FLASH_FL_STATUS_SECTOR_ERASE))
	{
	};
	ENTER_SECTION(ATOMIC_SYSTEM_MODE);
	IO_PORT_STOPMD_CTRL_S  &=  ~B_PORT_STOPMD_CTRL_SEL_STOP_MODE;
	__asm__ ("HALT\n\t" :::);
	EXIT_SECTION();
	__builtin_unreachable();
}

void AppReset( void) __attribute__((noreturn));
void AppReset(void)
{
	(void)mlu_ApplicationStop();
	ENTER_SECTION(SYSTEM_MODE);
	ml_ResetDrv();
	g_u8LinInFrameBufState = (uint8_t)C_LIN_IN_FREE;
	allowWarmReboot();
	IO_RST_CTRL_S = B_RST_CTRL_SOFT_RESET;
	EXIT_SECTION();
	__builtin_unreachable();
}
