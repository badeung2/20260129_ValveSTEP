#include "AppBuild.h"
#include "drivelib/AppFunctions.h"
#include "drivelib/Diagnostic.h"
#include "drivelib/NV_UserPage.h"
#include "drivelib/ErrorCodes.h"
#include "drivelib/GlobalVars.h"
#include "drivelib/MotorDriver.h"
#include "camculib/private_mathlib.h"
#include "../ActADC.h"
#include <bist_inline_impl.h>
#include <eeprom_map.h>
#include <lib_clock.h>
#include <sys_tools.h>
#include <trimming.h>

static const uint16_t au16LevelOV[4] =
{
	2100U, 2300U, 3900U, 3900U
};
#pragma space dp
static uint16_t l_u16MASK0 = 0U;
static uint16_t l_u16MASK1 = 0U;
static uint16_t l_u16MASK2 = 0U;
#pragma space none
#pragma space nodp
static uint8_t l_u8VDDAF_Count = 0U;
static uint16_t l_u16DrvProt = 0U;
#pragma space none
static void HandleDiagnosticsVDS(void)
{
	uint16_t u16VdsErrorMask = (IO_PORT_DIAG_IN & (M_PORT_DIAG_IN_OV_HS_VDS_MEM | M_PORT_DIAG_IN_OV_LS_VDS_MEM));
	{
		g_e8ErrorElectric = (uint8_t)C_ERR_VDS;
		MotorDriverStop( (uint16_t)C_STOP_WO_HOLDING);
		g_u16TargetPosition = g_u16ActualPosition;
		SetLastError(C_ERR_DIAG_VDS | C_ERR_EXTW);
		SetLastError(u16VdsErrorMask);
		IO_PORT_DRV1_PROT  |=  (B_PORT_DRV1_PROT_DIS_OV_LS_VDS | B_PORT_DRV1_PROT_DIS_OV_HS_VDS);
		IO_PORT_DRV1_PROT  &=  ~(B_PORT_DRV1_PROT_DIS_OV_LS_VDS | B_PORT_DRV1_PROT_DIS_OV_HS_VDS);
	}
}

static void HandleDiagnosticsOC(void)
{
	{
		g_e8ErrorElectric = (uint8_t)C_ERR_MOTOR_OVER_CURRENT;
		MotorDriverStop( (uint16_t)C_STOP_WO_HOLDING);
		g_u16TargetPosition = g_u16ActualPosition;
		SetLastError(C_ERR_DIAG_OVER_CURRENT | C_ERR_EXTW);
		SetLastError(g_u16MotorCurrentPeak);
	}
}

static void HandleDiagnosticsOT(void)
{
	if((g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK)  !=  C_MOTOR_STATUS_STOP){DELAY(C_DELAY_mPWM);}
	else{ADC_MeasureVsupplyAndTemperature();}
	(void)ADC_Conv_TempJ(FALSE);
	if(Get_ChipTemperature() > C_CHIP_OVERTEMP_LEVEL)
	{
		MotorDriverStop( (uint16_t)C_STOP_WO_HOLDING);
		g_u16TargetPosition = g_u16ActualPosition;
		g_e8ErrorOverTemperature = (uint8_t)C_ERR_OTEMP_SHUTDOWN;
		SetLastError(C_ERR_DIAG_OVER_TEMP);
		g_e8DegradeStatus = TRUE;
	}
}

static void HandleDiagnosticsUVOV(uint8_t e8DiagVoltage)
{
	g_e8ErrorVoltage = (uint8_t)e8DiagVoltage;
	g_e8ErrorVoltageComm = (uint8_t)e8DiagVoltage;
	if(g_e8MotorRequest  !=  (uint8_t)C_MOTOR_REQUEST_NONE){g_e8DegradedMotorRequest = g_e8MotorRequest;}
	else if((g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK)  !=  C_MOTOR_STATUS_STOP){g_e8DegradedMotorRequest = (uint8_t)C_MOTOR_REQUEST_START;}
	else if(g_e8DegradedMotorRequest  ==  (uint8_t)C_MOTOR_REQUEST_NONE){g_e8DegradedMotorRequest = (uint8_t)C_MOTOR_REQUEST_STOP;}
	else{}
	g_e8DegradeStatus = TRUE;
	{
		MotorDriverStop( (uint16_t)C_STOP_EMERGENCY);
	}
}

static void VerifyDiagnosticsUVOV(void)
{
	if((IO_PORT_SUPP_IN & ((B_PORT_SUPP_IN_OV_VS_SYNC | B_PORT_SUPP_IN_OV_VS_IT) | (B_PORT_SUPP_IN_UV_VS_SYNC | B_PORT_SUPP_IN_UV_VS_IT)))  !=  0U)
	{
		uint8_t e8DiagVoltage = (uint8_t)C_ERR_VOLTAGE_IN_RANGE;
		uint16_t u16SupplyVoltage;
		if((g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK)  !=  C_MOTOR_STATUS_STOP){DELAY(C_DELAY_mPWM);}
		else{ADC_MeasureVsupplyAndTemperature();}
		u16SupplyVoltage = ADC_Conv_Vsupply();
		if(u16SupplyVoltage < (450U + (NV_IC_UV_LEVEL * 100U)))
		{
			e8DiagVoltage = (uint8_t)C_ERR_VOLTAGE_UNDER;
			SetLastError(C_ERR_DIAG_UNDER_VOLT | C_ERR_EXTW);
			SetLastError(u16SupplyVoltage);
		}
		else if(u16SupplyVoltage > au16LevelOV[NV_IC_OV_LEVEL])
		{
			e8DiagVoltage = (uint8_t)C_ERR_VOLTAGE_OVER;
			SetLastError(C_ERR_DIAG_OVER_VOLT | C_ERR_EXTW);
			SetLastError(u16SupplyVoltage);
		}
		else{}
		if(e8DiagVoltage  !=  (uint8_t)C_ERR_VOLTAGE_IN_RANGE){HandleDiagnosticsUVOV(e8DiagVoltage);}
	}
}

static void HandleDiagnosticsOV(void)
{
	HandleDiagnosticsUVOV(C_ERR_VOLTAGE_OVER);
	SetLastError(C_ERR_DIAG_OVER_VOLT);
}

void HandleDiagnosticsVDDA(void)
{
	l_u16DrvProt = IO_PORT_DRV1_PROT;
	IO_PORT_DRV1_PROT = (B_PORT_DRV1_PROT_DIS_OV_LS_VDS
	| B_PORT_DRV1_PROT_DIS_OV_HS_VDS
	| B_PORT_DRV1_PROT_DIS_OC
	| B_PORT_DRV1_PROT_DIS_UV_VDDAF
	| B_PORT_DRV1_PROT_DIS_UV_VDDA
	| B_PORT_DRV1_PROT_DIS_UV_VS
	| B_PORT_DRV1_PROT_DIS_OV_VS
	| B_PORT_DRV1_PROT_DIS_OVT);
	ENTER_SECTION(SYSTEM_MODE);
	IO_MLX16_ITC_PEND0_S = (B_MLX16_ITC_PEND0_UV_VDDA
	| B_MLX16_ITC_PEND0_UV_VS
	| B_MLX16_ITC_PEND0_UV_VDDAF
	| B_MLX16_ITC_PEND0_OVT
	| B_MLX16_ITC_PEND0_OVC
	);
	IO_MLX16_ITC_PEND2_S = (B_MLX16_ITC_PEND2_OV_VS
	| B_MLX16_ITC_PEND2_DIAG);
	EXIT_SECTION();
	SetLastError(C_ERR_VDDA | C_ERR_EXT | 0x0D00U);
}

uint16_t HandleDiagnosticsVDDAF(void)
{
	uint16_t u16Result = FALSE;
	if(l_u8VDDAF_Count  !=  0U)
	{
		g_e8ErrorElectric = (uint8_t)(C_ERR_SEMI_PERMANENT | C_ERR_SUP_VDDAF);
		MotorDriverStop( (uint16_t)C_STOP_WO_HOLDING);
		g_u16TargetPosition = g_u16ActualPosition;
		SetLastError(C_ERR_VDDAF | C_ERR_EXT | 0x0A00U);
		u16Result = TRUE;
	}
	else
	{
		l_u8VDDAF_Count = 2U;
		SetLastError(C_WRN_VREF);
		DELAY(C_DELAY_mPWM);
	}
	return(u16Result);
}

void DiagnosticInit(void)
{
	IO_PORT_MISC2_OUT  |=  (B_PORT_MISC2_OUT_ENABLE_OTD
	);
	IO_PORT_SUPP_CFG = (B_PORT_SUPP_CFG_OVT_FILT_SEL |
	B_PORT_SUPP_CFG_UV_VS_FILT_SEL |
	C_PORT_SUPP_CFG_UV_VDDAF_FILT_SEL_11 |
	B_PORT_SUPP_CFG_UV_VDDA_FILT_SEL);
	IO_PORT_DRV1_PROT = ((IO_PORT_DRV1_PROT &
	~(0
	| B_PORT_DRV1_PROT_OV_LS_VDS_PM
	| B_PORT_DRV1_PROT_OV_HS_VDS_PM
	| B_PORT_DRV1_PROT_OC_PM
	| B_PORT_DRV1_PROT_UV_VDDAF_PM
	| B_PORT_DRV1_PROT_UV_VDDA_PM
	| B_PORT_DRV1_PROT_UV_VS_PM
	| B_PORT_DRV1_PROT_OV_VS_PM
	| B_PORT_DRV1_PROT_OVT_PM
	| B_PORT_DRV1_PROT_DIS_OV_LS_VDS
	| B_PORT_DRV1_PROT_DIS_OV_HS_VDS
	| B_PORT_DRV1_PROT_DIS_OV_VS
	)
	) |
	(0
	| B_PORT_DRV1_PROT_DIS_OC
	| B_PORT_DRV1_PROT_DIS_UV_VDDAF
	| B_PORT_DRV1_PROT_DIS_UV_VDDA
	| B_PORT_DRV1_PROT_DIS_UV_VS
	| B_PORT_DRV1_PROT_DIS_OVT
	));
	IO_PORT_MISC_OUT = (IO_PORT_MISC_OUT & ~(
	M_PORT_MISC_OUT_SEL_TEMP |
	M_PORT_MISC_OUT_PROV_VS |
	M_PORT_MISC_OUT_PRUV_VS)) |
	(
	C_PORT_MISC_OUT_SEL_TEMP_MAIN |
	(0 * B_PORT_MISC_OUT_PRUV_VDDA) |
	(NV_IC_UV_LEVEL * C_PORT_MISC_OUT_PRUV_0) |
	(NV_IC_OV_LEVEL * C_PORT_MISC_OUT_PROV_0));
	ENTER_SECTION(SYSTEM_MODE);
	IO_MLX16_ITC_PEND0_S = (B_MLX16_ITC_PEND0_UV_VDDA
	| B_MLX16_ITC_PEND0_UV_VS
	| B_MLX16_ITC_PEND0_UV_VDDAF
	| B_MLX16_ITC_PEND0_OVT
	| B_MLX16_ITC_PEND0_OVC
	| B_MLX16_ITC_PEND0_OV_HS_VDS0
	| B_MLX16_ITC_PEND0_OV_HS_VDS1
	| B_MLX16_ITC_PEND0_OV_HS_VDS2
	| B_MLX16_ITC_PEND0_OV_HS_VDS3);
	IO_MLX16_ITC_PEND2_S = (B_MLX16_ITC_PEND2_OV_VS
	| B_MLX16_ITC_PEND2_DIAG);
	IO_MLX16_ITC_PRIO1_S = (IO_MLX16_ITC_PRIO1_S & ~M_MLX16_ITC_PRIO1_PWM_MASTER1_END) |
	C_MLX16_ITC_PRIO1_PWM_MASTER1_END_PRIO5;
	IO_MLX16_ITC_PRIO2_S = (IO_MLX16_ITC_PRIO2_S & ~M_MLX16_ITC_PRIO2_OV_VS) | C_MLX16_ITC_PRIO2_OV_VS_PRIO3;
	IO_MLX16_ITC_PRIO2_S = (IO_MLX16_ITC_PRIO2_S & ~M_MLX16_ITC_PRIO2_DIAG) | C_MLX16_ITC_PRIO2_DIAG_PRIO3;
	IO_MLX16_ITC_MASK0_S  |=  (B_MLX16_ITC_MASK0_AWD_ATT
	| B_MLX16_ITC_MASK0_UV_VDDA
	| B_MLX16_ITC_MASK0_UV_VS
	| B_MLX16_ITC_MASK0_OVT
	| B_MLX16_ITC_MASK0_OVC
	);
	IO_MLX16_ITC_MASK2_S  |=  (B_MLX16_ITC_MASK2_OV_VS
	| B_MLX16_ITC_MASK2_DIAG
	);
	EXIT_SECTION();
	if(((IO_PORT_SUPP_IN & B_PORT_SUPP_IN_OVT_IT)  !=  0U)  ||  ((IO_PORT_DIAG_IN & B_PORT_DIAG_IN_OVT_MEM)  !=  0U))
	{
		HandleDiagnosticsOT();
		ENTER_SECTION(SYSTEM_MODE);
		IO_MLX16_ITC_MASK0_S  &=  ~B_MLX16_ITC_MASK0_OVT;
		l_u16MASK0  |=  B_MLX16_ITC_MASK0_OVT;
		IO_MLX16_ITC_PEND0_S = B_MLX16_ITC_PEND0_OVT;
		EXIT_SECTION();
	}
	if(((IO_PORT_SUPP_IN & B_PORT_SUPP_IN_OV_VS_IT)  !=  0U)  ||  ((IO_PORT_DIAG_IN & B_PORT_DIAG_IN_OV_VS_MEM)  !=  0U))
	{
		VerifyDiagnosticsUVOV();
		ENTER_SECTION(SYSTEM_MODE);
		IO_MLX16_ITC_MASK2_S  &=  ~B_MLX16_ITC_MASK2_OV_VS;
		l_u16MASK2  |=  B_MLX16_ITC_MASK2_OV_VS;
		IO_MLX16_ITC_PEND2_S = B_MLX16_ITC_PEND2_OV_VS;
		EXIT_SECTION();
	}
}

void DiagnosticReset(void)
{
	l_u8VDDAF_Count = (uint8_t)0U;
}

__attribute__((interrupt)) void ISR_DIAG(void)
{
	if((IO_PORT_DIAG_IN & (M_PORT_DIAG_IN_OV_LS_VDS_MEM | M_PORT_DIAG_IN_OV_HS_VDS_MEM))  !=  0U){HandleDiagnosticsVDS();}
	if((IO_PORT_DIAG_IN & B_PORT_DIAG_IN_OV_VS_MEM)  !=  0U){HandleDiagnosticsOV();}
	ENTER_SECTION(SYSTEM_MODE);
	IO_MLX16_ITC_PEND2_S = B_MLX16_ITC_PEND2_DIAG;
	EXIT_SECTION();
}

__attribute__((interrupt)) void ISR_UV_VDDA(void)
{
	HandleDiagnosticsVDDA();
	ENTER_SECTION(SYSTEM_MODE);
	IO_MLX16_ITC_MASK0_S  &=  ~B_MLX16_ITC_MASK0_UV_VDDA;
	l_u16MASK0  |=  B_MLX16_ITC_MASK0_UV_VDDA;
	IO_MLX16_ITC_PEND0_S = B_MLX16_ITC_PEND0_UV_VDDA;
	EXIT_SECTION();
}

__attribute__((interrupt)) void ISR_UV_VDDAF(void)
{
	if(HandleDiagnosticsVDDAF()  !=  FALSE)
	{
		ENTER_SECTION(SYSTEM_MODE);
		IO_MLX16_ITC_MASK0_S  &=  ~B_MLX16_ITC_MASK0_UV_VDDAF;
		l_u16MASK0  |=  B_MLX16_ITC_MASK0_UV_VDDAF;
		IO_MLX16_ITC_PEND0_S = B_MLX16_ITC_PEND0_UV_VDDAF;
		EXIT_SECTION();
	}
}

#if (defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)) && _SUPPORT_DIAG_BOOST
__attribute__((interrupt)) void ISR_UV_VBOOST(void)
{
#if (_DEBUG_CPU_LOAD != FALSE)
	DEBUG_SET_IO_A();
#endif
#if (_DEBUG_DIAG_VBOOST != FALSE)
	DEBUG_SET_IO_D();
#endif
#if (_SUPPORT_DIAG_IRQCTLR_LEVEL != FALSE)
	if(HandleDiagnosticsVBOOST(1U)  !=  FALSE)
	{
#if (_SUPPORT_APP_USER_MODE != FALSE)
		ENTER_SECTION(SYSTEM_MODE);
#endif
		IO_MLX16_ITC_MASK0_S  &=  ~B_MLX16_ITC_MASK0_UV_BOOST;
		l_u16MASK0  |=  B_MLX16_ITC_MASK0_UV_BOOST;
		IO_MLX16_ITC_PEND0_S = B_MLX16_ITC_PEND0_UV_BOOST;
#if (_SUPPORT_APP_USER_MODE != FALSE)
		EXIT_SECTION();
#endif
	}
#else
	(void)HandleDiagnosticsVBOOST(1U);
#endif
#if (_DEBUG_DIAG_VBOOST != FALSE)
	DEBUG_CLR_IO_D();
	DEBUG_SET_IO_D();
	DEBUG_CLR_IO_D();
#endif
#if (_DEBUG_CPU_LOAD != FALSE)
	DEBUG_CLR_IO_A();
#endif
}

__attribute__((interrupt)) void ISR_OV_VBOOST(void)
{
#if (_DEBUG_CPU_LOAD != FALSE)
	DEBUG_SET_IO_A();
#endif
#if (_DEBUG_DIAG_VBOOST != FALSE)
	DEBUG_SET_IO_D();
#endif
#if (_SUPPORT_DIAG_IRQCTLR_LEVEL != FALSE)
	if(HandleDiagnosticsVBOOST(2U)  !=  FALSE)
	{
#if (_SUPPORT_APP_USER_MODE != FALSE)
		ENTER_SECTION(SYSTEM_MODE);
#endif
		IO_MLX16_ITC_MASK3_S  &=  ~B_MLX16_ITC_MASK3_OV_BOOST;
		l_u16MASK3  |=  B_MLX16_ITC_MASK3_OV_BOOST;
		IO_MLX16_ITC_PEND3_S = B_MLX16_ITC_PEND3_OV_BOOST;
#if (_SUPPORT_APP_USER_MODE != FALSE)
		EXIT_SECTION();
#endif
	}
#else
	(void)HandleDiagnosticsVBOOST(2U);
#endif
#if (_DEBUG_DIAG_VBOOST != FALSE)
	DEBUG_CLR_IO_D();
#endif
#if (_DEBUG_CPU_LOAD != FALSE)
	DEBUG_CLR_IO_A();
#endif
}

#endif
__attribute__((interrupt)) void ISR_OC(void)
{
	HandleDiagnosticsOC();
	ENTER_SECTION(SYSTEM_MODE);
	IO_MLX16_ITC_MASK0_S  &=  ~B_MLX16_ITC_MASK0_OVC;
	l_u16MASK0  |=  B_MLX16_ITC_MASK0_OVC;
	IO_MLX16_ITC_PEND0_S = B_MLX16_ITC_PEND0_OVC;
	EXIT_SECTION();
}

__attribute__((interrupt)) void ISR_OT(void)
{
	HandleDiagnosticsOT();
	ENTER_SECTION(SYSTEM_MODE);
	IO_MLX16_ITC_MASK0_S  &=  ~B_MLX16_ITC_MASK0_OVT;
	l_u16MASK0  |=  B_MLX16_ITC_MASK0_OVT;
	IO_MLX16_ITC_PEND0_S = B_MLX16_ITC_PEND0_OVT;
	EXIT_SECTION();
}

__attribute__((interrupt)) void ISR_UV_VS(void)
{
	VerifyDiagnosticsUVOV();
	ENTER_SECTION(SYSTEM_MODE);
	IO_MLX16_ITC_MASK0_S  &=  ~B_MLX16_ITC_MASK0_UV_VS;
	l_u16MASK0  |=  B_MLX16_ITC_MASK0_UV_VS;
	IO_MLX16_ITC_PEND0_S = B_MLX16_ITC_PEND0_UV_VS;
	EXIT_SECTION();
}

__attribute__((interrupt)) void ISR_OV_VS(void)
{
	VerifyDiagnosticsUVOV();
	ENTER_SECTION(SYSTEM_MODE);
	IO_MLX16_ITC_MASK2_S  &=  ~B_MLX16_ITC_MASK2_OV_VS;
	l_u16MASK2  |=  B_MLX16_ITC_MASK2_OV_VS;
	IO_MLX16_ITC_PEND2_S = B_MLX16_ITC_PEND2_OV_VS;
	EXIT_SECTION();
}

void DiagnosticPeriodicTimerEvent(void)
{
	if(l_u16MASK0  !=  0U)
	{
		uint16_t u16Resolved = ((IO_MLX16_ITC_PEND0_S ^ l_u16MASK0) & l_u16MASK0);
		ENTER_SECTION(SYSTEM_MODE);
		IO_MLX16_ITC_PEND0_S = l_u16MASK0;
		if(u16Resolved  !=  0U)
		{
			l_u16MASK0  ^=  u16Resolved;
			IO_MLX16_ITC_MASK0_S  |=  u16Resolved;
			if(((u16Resolved & B_MLX16_ITC_MASK0_UV_VDDA)  !=  0U)  &&  (l_u16DrvProt  !=  0U))
			{
				IO_PORT_DRV1_PROT = l_u16DrvProt;
				l_u16DrvProt = 0U;
			}
			if(l_u16MASK0  ==  0U){g_e8ErrorElectric  &=  (uint8_t) ~C_ERR_SEMI_PERMANENT;}
		}
		EXIT_SECTION();
	}
	if(l_u16MASK1  !=  0U)
	{
		uint16_t u16Resolved = ((IO_MLX16_ITC_PEND1_S ^ l_u16MASK1) & l_u16MASK1);
		ENTER_SECTION(SYSTEM_MODE);
		IO_MLX16_ITC_PEND1_S = l_u16MASK1;
		if(u16Resolved  !=  0U)
		{
			l_u16MASK1  ^=  u16Resolved;
			IO_MLX16_ITC_MASK1_S  |=  u16Resolved;
		}
		EXIT_SECTION();
	}
	if(l_u16MASK2  !=  0U)
	{
		uint16_t u16Resolved = ((IO_MLX16_ITC_PEND2_S ^ l_u16MASK2) & l_u16MASK2);
		ENTER_SECTION(SYSTEM_MODE);
		IO_MLX16_ITC_PEND2_S = l_u16MASK2;
		if((l_u16MASK2 & B_MLX16_ITC_PEND2_DIAG)  !=  0U)
		{
			uint16_t u16CopyDrvProt = IO_PORT_DRV1_PROT;
			uint16_t u16ClrDrvProt = 0U;
			if((IO_PORT_DIAG_IN & M_PORT_DIAG_IN_OV_LS_VDS_MEM)  !=  0U){u16ClrDrvProt  |=  B_PORT_DRV1_PROT_DIS_OV_LS_VDS;}
			if((IO_PORT_DIAG_IN & M_PORT_DIAG_IN_OV_HS_VDS_MEM)  !=  0U){u16ClrDrvProt  |=  B_PORT_DRV1_PROT_DIS_OV_HS_VDS;}
			if((IO_PORT_DIAG_IN & B_PORT_DIAG_IN_OV_VS_MEM)  !=  0U){u16ClrDrvProt  |=  B_PORT_DRV1_PROT_DIS_OV_VS;}
			IO_PORT_DRV1_PROT = u16CopyDrvProt | u16ClrDrvProt;
			IO_PORT_DRV1_PROT = u16CopyDrvProt;
		}
		if(u16Resolved  !=  0U)
		{
			l_u16MASK2  ^=  u16Resolved;
			IO_MLX16_ITC_MASK2_S  |=  u16Resolved;
		}
		EXIT_SECTION();
	}
	else if(((IO_PORT_DIAG_IN & M_PORT_DIAG_IN_OV_LS_VDS_MEM)  !=  0U)  ||  ((IO_PORT_DIAG_IN & M_PORT_DIAG_IN_OV_HS_VDS_MEM)  !=  0U)){l_u16MASK2 = B_MLX16_ITC_PEND2_DIAG;}
	l_u8VDDAF_Count = p_DecNzU8(l_u8VDDAF_Count);
}

__attribute__((interrupt)) void ISR_AWD_ATT(void)
{
	SetLastError(C_WRN_AWD_ATT);
}
